from __future__ import annotations

import ipaddress
import json
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock, RLock
from typing import Any
from urllib.parse import urlparse

from fleet_manager.route_core import build_editable_map_bundle_payload

from ..config import GRPC_ROBOT_TYPES
from ..services.fleet_manager import FLEET_MANAGER_ID, OperatorFleetManager
from .map_cache import MapCache, default_maps_cache_root
from .models import KnownRobot
from .registry import RobotRegistry
from ..robot_grpc_api.client import GrpcRobotAdapter, GrpcRobotError
from ..robot_grpc_api.contracts import DEFAULT_GRPC_PORT
from .workspace import OperatorWorkspace

OPERATOR_CONTROL_OWNER_ID = "operator-app"
OPERATOR_CONTROL_OWNER_NAME = "Operator App"


class RobotProbeError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class OperatorAppState:
    def __init__(self, registry_path: Path, probe_timeout: float, fleet_params_path: Path, fleet_map_dir: Path) -> None:
        self.registry = RobotRegistry(registry_path)
        self.workspace = OperatorWorkspace()
        self.legacy_map_cache_root = default_maps_cache_root().expanduser().resolve()
        self.map_cache = MapCache(robot_dir_resolver=self._maps_dir_for_robot_id)
        self.grpc_adapter = GrpcRobotAdapter(timeout=max(1.5, float(probe_timeout)))
        self.map_timeout = max(10.0, float(probe_timeout) * 10.0)
        self.fleet_params_path = Path(fleet_params_path).expanduser().resolve()
        self.fleet_manager = OperatorFleetManager(
            fleet_map_dir,
            self.fleet_params_path,
            remote_adapter=self.grpc_adapter,
        )
        self._lock = Lock()
        self._fleet_lock = RLock()

    def _maps_dir_for_robot_id(self, robot_id: str) -> Path:
        if robot_id == FLEET_MANAGER_ID:
            directory = self.workspace.maps_dir("fleet_manager")
            directory.mkdir(parents=True, exist_ok=True)
            return directory
        with self._lock:
            robots = self.registry.load()
        for robot in robots:
            if robot.id == robot_id:
                self._ensure_robot_workspace(robot)
                return self.workspace.maps_dir(robot)
        directory = self.workspace.maps_dir(robot_id)
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def _legacy_maps_dir_for_robot_id(self, robot_id: str) -> Path:
        safe = self.map_cache._safe_name(robot_id)
        return self.legacy_map_cache_root / safe

    def _ensure_robot_workspace(self, robot: KnownRobot) -> dict[str, Any]:
        return self.workspace.ensure_robot_workspace(
            robot,
            legacy_maps_dir=self._legacy_maps_dir_for_robot_id(robot.id),
        )

    def _cache_robot_params(self, robot: KnownRobot, params: dict[str, Any], *, source: str = "robot") -> dict[str, Any]:
        return self.workspace.save_params(robot, params, source=source)

    def _bootstrap_robot_workspace(self, robot: KnownRobot, endpoint: str) -> dict[str, Any]:
        workspace = self._ensure_robot_workspace(robot)
        warnings: list[str] = []
        cached_maps: list[str] = []
        map_index: dict[str, Any] = {"ok": False, "maps": []}
        active_map: dict[str, Any] = {"ok": False, "mapName": ""}

        try:
            map_index = self.grpc_adapter.list_maps(endpoint)
            if isinstance(map_index, dict):
                self.workspace.save_map_index(robot, map_index)
        except Exception as exc:
            warnings.append(f"list maps failed: {exc}")

        try:
            active_map = self.grpc_adapter.active_map(endpoint)
            if isinstance(active_map, dict):
                self.workspace.save_active_map_meta(robot, active_map)
        except Exception as exc:
            warnings.append(f"active map failed: {exc}")

        map_names: list[str] = []
        for item in map_index.get("maps", []) if isinstance(map_index, dict) else []:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or item.get("mapName") or "").strip()
            if name:
                map_names.append(name)
        active_name = str(active_map.get("mapName") or "").strip() if isinstance(active_map, dict) else ""
        if active_name and active_name not in map_names:
            map_names.insert(0, active_name)

        for map_name in map_names:
            try:
                bundle = self.grpc_adapter.get_map_bundle(endpoint, map_name)
                self.map_cache.save_pulled_map(robot.id, bundle, activate=map_name == active_name or not cached_maps)
                cached_maps.append(str(bundle.get("mapName") or map_name))
            except Exception as exc:
                warnings.append(f"map {map_name} failed: {exc}")

        try:
            params_payload = self.grpc_adapter.get_params(endpoint)
            params = params_payload.get("params") if isinstance(params_payload, dict) else None
            if isinstance(params, dict):
                self._cache_robot_params(robot, params, source="robot")
        except Exception as exc:
            warnings.append(f"params failed: {exc}")

        return {
            "workspace": workspace,
            "cachedMaps": cached_maps,
            "activeMapName": active_name,
            "warnings": warnings,
        }

    def list_robots_payload(self, probe_robots: bool = True) -> dict[str, Any]:
        with self._lock:
            robots = self.registry.load()
        if probe_robots:
            with self._fleet_lock:
                fleet_payload = self.fleet_manager.sidebar_payload()
        elif self._fleet_lock.acquire(blocking=False):
            try:
                fleet_payload = self.fleet_manager.sidebar_payload()
            finally:
                self._fleet_lock.release()
        else:
            fleet_payload = self.fleet_manager.sidebar_payload(include_runtime=False)
        items = [self._robot_payload(robot, allow_probe=probe_robots) for robot in robots]
        items.append(fleet_payload)
        return {"ok": True, "robots": items}

    def probe_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        host = self._require_host(payload)
        robot_type = self._payload_robot_type(payload)
        if robot_type not in GRPC_ROBOT_TYPES:
            raise ValueError("robot transport must be grpc")
        port = self._require_port(payload, default=DEFAULT_GRPC_PORT)
        result = self._probe_grpc_robot(host, port)
        return {"ok": True, "probe": result}

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        host = self._require_host(payload)
        robot_type = self._payload_robot_type(payload)
        if robot_type in GRPC_ROBOT_TYPES:
            port = self._require_port(payload, default=DEFAULT_GRPC_PORT)
            probe = self._probe_grpc_robot(host, port)
            identity = probe.get("identity", {})
            status = probe.get("status", {}) if isinstance(probe.get("status"), dict) else {}
            robot_name = str(
                payload.get("name")
                or identity.get("robotId")
                or identity.get("name")
                or status.get("robotId")
                or f"robot@{host}"
            ).strip()
            robot_id = str(payload.get("robotId") or payload.get("robot_id") or identity.get("robotId") or "").strip()
            if not robot_id:
                robot_id = f"grpc:{host}:{port}"
            robot = KnownRobot(
                id=robot_id,
                name=robot_name,
                host=host,
                port=port,
                type="grpc",
                last_seen=utc_now(),
                last_identity=identity if isinstance(identity, dict) else None,
            )
            with self._lock:
                self.registry.upsert(robot)
            cache = self._bootstrap_robot_workspace(robot, robot.base_url)
            return {
                "ok": True,
                "robot": self._robot_payload(robot, probe=probe),
                "cache": cache,
            }
        raise ValueError("robot transport must be grpc")

    def delete_robot_payload(self, robot_id: str) -> dict[str, Any]:
        if robot_id == FLEET_MANAGER_ID:
            raise ValueError("system robot entries cannot be removed")
        with self._lock:
            deleted = self.registry.remove(robot_id)
        if not deleted:
            raise ValueError(f"unknown robot: {robot_id}")
        return {"ok": True, "deleted": robot_id}

    def get_robot(self, robot_id: str) -> KnownRobot:
        with self._lock:
            robots = self.registry.load()
        for robot in robots:
            if robot.id == robot_id:
                return robot
        raise ValueError(f"unknown robot: {robot_id}")

    def _robot_payload(
        self,
        robot: KnownRobot,
        probe: dict[str, Any] | None = None,
        *,
        allow_probe: bool = True,
    ) -> dict[str, Any]:
        if robot.is_grpc:
            if probe is None and allow_probe:
                try:
                    probe = self._probe_grpc_robot(robot.host, robot.port)
                except Exception as exc:
                    probe = {
                        "ok": False,
                        "baseUrl": robot.base_url,
                        "online": False,
                        "error": str(exc),
                        "identity": robot.last_identity or None,
                        "status": None,
                    }
            if probe is None:
                probe = {
                    "ok": False,
                    "baseUrl": robot.base_url,
                    "online": False,
                    "identity": robot.last_identity or None,
                    "status": None,
                    "error": "",
                    "probed": False,
                }
            payload = robot.to_dict()
            payload.update(
                {
                    "online": bool(probe.get("ok", False)),
                    "baseUrl": str(probe.get("baseUrl") or robot.base_url),
                    "identity": probe.get("identity") or robot.last_identity or None,
                    "status": probe.get("status") if isinstance(probe.get("status"), dict) else None,
                    "error": str(probe.get("error") or "").strip(),
                    "probed": bool(probe.get("probed", True)),
                    "workspace": self.workspace.workspace_payload(robot),
                }
            )
            return payload

        payload = robot.to_dict()
        payload.update(
            {
                "online": False,
                "baseUrl": "",
                "identity": robot.last_identity or None,
                "status": None,
                "error": "unsupported robot transport; use grpc",
                "probed": bool(allow_probe),
            }
        )
        return payload

    def _is_grpc_robot_id(self, robot_id: str) -> bool:
        try:
            return self.get_robot(robot_id).is_grpc
        except ValueError:
            return False

    def _grpc_endpoint(self, robot: KnownRobot) -> str:
        if not robot.is_grpc:
            raise ValueError(f"robot is not gRPC-backed: {robot.id}")
        return robot.base_url

    def _probe_grpc_robot(self, host: str, port: int) -> dict[str, Any]:
        endpoint = f"grpc://{host}:{port}"
        try:
            self.grpc_adapter.client.health(endpoint)
            identity = self.grpc_adapter.identity(endpoint)
            status_payload = self.grpc_adapter.status(endpoint)
        except Exception as exc:
            raise RobotProbeError(f"{endpoint} is not reachable: {exc}") from exc
        robot_status = status_payload.get("robot", {})
        if not isinstance(robot_status, dict):
            robot_status = {}
        return {
            "ok": True,
            "online": True,
            "baseUrl": endpoint,
            "identity": identity,
            "status": robot_status,
            "probed": True,
        }

    def proxy_request(
        self,
        robot_id: str,
        method: str,
        path: str,
        *,
        headers: dict[str, str] | None = None,
        body: bytes | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        robot = self.get_robot(robot_id)
        if robot.is_grpc:
            return self._proxy_grpc_robot_request(robot_id, method, path, body=body)
        raise ValueError("unsupported robot transport; use grpc")

    def robot_maps_list_payload(self, robot_id: str) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            result = self.grpc_adapter.list_maps(self._grpc_endpoint(robot))
            if isinstance(result, dict):
                self.workspace.save_map_index(robot, result)
            return result
        raise ValueError("unsupported robot transport; use grpc")

    def robot_maps_active_payload(self, robot_id: str) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            active = self.grpc_adapter.active_map(self._grpc_endpoint(robot))
            if not active.get("signature"):
                try:
                    bundle = self.grpc_adapter.get_map_bundle(self._grpc_endpoint(robot), str(active.get("mapName") or ""))
                    active["signature"] = str(bundle.get("signature") or "")
                except Exception:
                    active.setdefault("signature", "")
            self.workspace.save_active_map_meta(robot, active)
            return active
        raise ValueError("unsupported robot transport; use grpc")

    def robot_params_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if robot.is_grpc:
            try:
                result = self.grpc_adapter.get_params(self._grpc_endpoint(robot))
                params = result.get("params") if isinstance(result, dict) else None
                if isinstance(params, dict):
                    result["cache"] = self._cache_robot_params(robot, params, source="robot")
                result["robotId"] = robot_id
                return result
            except Exception as exc:
                cached = self.workspace.load_params(robot)
                if isinstance(cached, dict):
                    return {
                        "ok": True,
                        "robotId": robot_id,
                        "cached": True,
                        "warning": f"using cached params because robot is unavailable: {exc}",
                        "params": cached,
                    }
                raise
        raise ValueError("unsupported robot transport; use grpc")

    def save_robot_params_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            params = payload.get("params") if isinstance(payload.get("params"), dict) else payload
            result = self.grpc_adapter.put_params(self._grpc_endpoint(robot), params)
            if not isinstance(result, dict):
                result = {"ok": True}
            saved_params = result.get("params") if isinstance(result.get("params"), dict) else params
            result["cache"] = self._cache_robot_params(robot, saved_params, source="operator")
            result["robotId"] = robot_id
            return result
        raise ValueError("unsupported robot transport; use grpc")

    def fleet_params_payload(self) -> dict[str, Any]:
        with self._fleet_lock:
            return self.fleet_manager.params_payload()

    def save_fleet_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._fleet_lock:
            return self.fleet_manager.save_params_payload(payload)

    def fleet_manager_get_payload(self, action: str, arg: str = "") -> dict[str, Any]:
        with self._fleet_lock:
            if action == "identity":
                return self.fleet_manager.sidebar_payload()
            if action == "status":
                return self.fleet_manager.state_payload()
            if action == "state":
                return self.fleet_manager.state_payload()
            if action == "mode":
                return self.fleet_manager.mode_payload()
            if action == "map":
                return self.fleet_manager.map_payload()
            if action == "maps_list":
                return self.fleet_manager.maps_list_payload()
            if action == "maps_active":
                return self.fleet_manager.maps_active_payload()
            if action == "maps_pull":
                return self.fleet_pull_map_payload({"mapName": arg})
            if action == "maps_local_list":
                return self.fleet_local_maps_payload()
            if action == "maps_local_active":
                return self.fleet_local_active_map_payload()
            if action == "maps_local_get":
                return self.fleet_local_map_payload(arg)
            if action == "params":
                return self.fleet_manager.params_payload()
            if action == "orders":
                return self.fleet_manager.orders_payload()
            raise ValueError(f"unknown fleet manager action: {action}")

    def fleet_manager_post_payload(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        with self._fleet_lock:
            if action == "mode":
                return self.fleet_manager.set_mode_payload(payload)
            if action == "params":
                return self.fleet_manager.save_params_payload(payload)
            if action == "plan":
                return self.fleet_manager.plan_payload(payload)
            if action == "set_order":
                return self.fleet_manager.set_order_payload(payload)
            if action == "orders_dispatch":
                return self.fleet_manager.dispatch_orders_payload(payload)
            if action == "orders_cancel":
                return self.fleet_manager.cancel_order_payload(payload)
            if action == "orders_pause":
                return self.fleet_manager.pause_order_payload(payload)
            if action == "orders_resume":
                return self.fleet_manager.resume_order_payload(payload)
            if action == "orders_clear":
                return self.fleet_manager.clear_orders_payload(payload)
            if action == "tick":
                return self.fleet_manager.tick_payload(payload)
            if action == "world":
                return self.fleet_manager.world_payload(payload)
            if action == "check":
                return self.fleet_manager.check_payload(payload)
            if action == "manual_step":
                return self.fleet_manager.manual_step_payload(payload)
            if action == "manual_stop":
                return self.fleet_manager.manual_stop_payload(payload)
            if action == "maps_load":
                return self.fleet_manager.load_map_payload(payload)
            if action == "maps_save":
                return self.fleet_manager.save_map_payload(payload)
            if action == "maps_local_save":
                return self.fleet_save_local_map_payload(payload)
            if action == "maps_local_activate":
                return self.fleet_activate_local_map_payload(payload)
            if action == "maps_pull":
                return self.fleet_pull_map_payload(payload)
            if action == "maps_pull_sync":
                return self.fleet_pull_sync_payload()
            if action == "maps_push":
                return self.fleet_push_map_payload(payload)
            if action == "maps_push_sync":
                return self.fleet_push_sync_payload()
            if action == "robots_add":
                return self.fleet_manager.add_robot_payload(payload)
            if action == "robots_remove":
                return self.fleet_manager.remove_robot_payload(payload)
            if action == "robots_update":
                return self.fleet_manager.update_robot_payload(payload)
            if action == "robots_stop":
                return self.fleet_manager.stop_robot_payload(payload)
            if action == "robots_reset":
                return self.fleet_manager.reset_robot_payload(payload)
            raise ValueError(f"unknown fleet manager action: {action}")

    def fleet_manager_stream_payload(self, initial: bool = False) -> dict[str, Any] | None:
        if initial:
            self._fleet_lock.acquire()
        elif not self._fleet_lock.acquire(blocking=False):
            return None
        try:
            state = (
                self.fleet_manager.state_payload(include_trajectories=True)
                if initial
                else self.fleet_manager.tick_payload({})
            )
            return {
                "ok": True,
                "type": "state" if initial else "tick",
                "state": state,
                "sentAt": utc_now(),
            }
        finally:
            self._fleet_lock.release()

    def fleet_local_maps_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "activeMapName": self.map_cache.active_map_name(FLEET_MANAGER_ID),
            "maps": self.map_cache.list_maps(FLEET_MANAGER_ID),
        }

    def fleet_local_active_map_payload(self) -> dict[str, Any]:
        active_name = self.map_cache.active_map_name(FLEET_MANAGER_ID)
        active_payload = self.map_cache.load_active_map(FLEET_MANAGER_ID)
        robot_active = self.fleet_manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        robot_signature = str(robot_active.get("signature") or "").strip()
        if isinstance(active_payload, dict):
            local_signature = str(active_payload.get("signature") or "").strip()
            active_payload["robotSignature"] = str(active_payload.get("robotSignature") or robot_signature)
            active_payload["robotMapName"] = str(active_payload.get("robotMapName") or robot_active_name)
            active_payload["hasLocalChanges"] = bool(
                (local_signature and robot_signature and local_signature != robot_signature)
                or (robot_active_name and str(active_payload.get("mapName") or "").strip() != robot_active_name)
            )
        return {
            "ok": True,
            "activeMapName": active_name,
            "map": active_payload.get("map") if isinstance(active_payload, dict) else None,
            "sourceMapName": str(active_payload.get("sourceMapName") or "") if isinstance(active_payload, dict) else "",
            "signature": str(active_payload.get("signature") or "") if isinstance(active_payload, dict) else "",
            "robotSignature": str(active_payload.get("robotSignature") or robot_signature) if isinstance(active_payload, dict) else robot_signature,
            "robotMapName": str(active_payload.get("robotMapName") or robot_active_name) if isinstance(active_payload, dict) else robot_active_name,
            "hasLocalChanges": bool(active_payload.get("hasLocalChanges")) if isinstance(active_payload, dict) else False,
        }

    def fleet_local_map_payload(self, map_name: str) -> dict[str, Any]:
        payload = self.map_cache.load_map(FLEET_MANAGER_ID, map_name)
        return {"ok": True, **payload}

    def fleet_save_local_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or "").strip()
        editable_map = payload.get("map")
        if not map_name:
            raise ValueError("mapName is required")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        saved = self.map_cache.save_map(
            FLEET_MANAGER_ID,
            map_name,
            editable_map,
            source_map_name=str(payload.get("sourceMapName") or map_name),
            activate=bool(payload.get("activate", True)),
        )
        return {"ok": True, "local": saved}

    def fleet_activate_local_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        self.map_cache.load_map(FLEET_MANAGER_ID, map_name)
        self.map_cache.set_active_map(FLEET_MANAGER_ID, map_name)
        return {"ok": True, "activeMapName": map_name}

    def fleet_pull_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or "").strip()
        result = self.fleet_manager.pull_map_payload(map_name)
        local_name = str(result.get("mapName") or map_name or "active").strip() or "active"
        cached = self.map_cache.save_pulled_map(FLEET_MANAGER_ID, result, activate=True)
        return {
            "ok": True,
            "pulled": result,
            "local": {
                "mapName": str(cached.get("mapName") or local_name),
                "savedAt": str(cached.get("savedAt") or ""),
            },
        }

    def fleet_push_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            local_name = str(payload.get("localMapName") or payload.get("mapName") or "").strip()
            cached = self.map_cache.load_map(FLEET_MANAGER_ID, local_name)
            editable_map = cached.get("map")
            payload = {
                **payload,
                "sourceMapName": str(payload.get("sourceMapName") or cached.get("sourceMapName") or local_name),
            }
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        target_map_name = str(payload.get("mapName") or editable_map.get("mapName") or "").strip()
        source_map_name = str(payload.get("sourceMapName") or target_map_name).strip()
        output_name = str(payload.get("outputName") or "").strip()
        if not output_name and target_map_name and source_map_name and target_map_name != source_map_name:
            output_name = target_map_name
        result = self.fleet_manager.push_map_payload(
            {
                "map": editable_map,
                "mapName": target_map_name,
                "sourceMapName": source_map_name,
                "outputName": output_name,
                "overwriteOutput": bool(payload.get("overwriteOutput", False)),
            }
        )
        local_name = str(result.get("mapName") or output_name or target_map_name or "map").strip()
        cached = self.map_cache.save_map(
            FLEET_MANAGER_ID,
            local_name,
            result,
            source_map_name=str(result.get("mapName") or local_name),
        )
        return {"ok": True, "pushed": result, "local": cached}

    def fleet_pull_sync_payload(self) -> dict[str, Any]:
        robot_active = self.fleet_manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(FLEET_MANAGER_ID)
        robot_current = self.fleet_manager.pull_map_payload(robot_active_name)
        local_signature = ""
        if isinstance(local_active, dict):
            local_map = local_active.get("map")
            if isinstance(local_map, dict):
                local_signature = str(local_map.get("signature") or "").strip()
        robot_signature = str(robot_current.get("signature") or "").strip()
        local_active_name = str(local_active.get("mapName") or "") if isinstance(local_active, dict) else ""
        if robot_active_name and robot_active_name == local_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "message": f"Operator already has active Fleet Manager map {robot_active_name}.",
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_active_name,
            }
        cached = self.map_cache.save_pulled_map(FLEET_MANAGER_ID, robot_current, activate=True)
        pulled = {
            "ok": True,
            "pulled": robot_current,
            "local": {
                "mapName": str(cached.get("mapName") or robot_active_name),
                "savedAt": str(cached.get("savedAt") or ""),
            },
        }
        return {
            "ok": True,
            "changed": True,
            "message": f"Pulled active Fleet Manager map {robot_active_name}.",
            "robotActiveMapName": robot_active_name,
            "localActiveMapName": str(pulled.get("local", {}).get("mapName") or robot_active_name),
            **pulled,
        }

    def fleet_push_sync_payload(self) -> dict[str, Any]:
        robot_active = self.fleet_manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(FLEET_MANAGER_ID)
        if not isinstance(local_active, dict):
            raise ValueError("operator has no active local map")
        local_map_name = str(local_active.get("mapName") or "").strip()
        if not local_map_name:
            raise ValueError("operator active local map is invalid")
        local_map_payload = local_active.get("map")
        local_signature = str(local_map_payload.get("signature") or "").strip() if isinstance(local_map_payload, dict) else ""
        has_local_changes = bool(local_active.get("hasLocalChanges"))
        robot_current = self.fleet_manager.pull_map_payload(robot_active_name)
        robot_signature = str(robot_current.get("signature") or "").strip()
        if (not has_local_changes) and local_map_name == robot_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "message": f"Fleet Manager already uses {robot_active_name}.",
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_map_name,
            }
        pushed = self.fleet_push_map_payload(
            {
                "localMapName": local_map_name,
                "mapName": local_map_name,
                "sourceMapName": str(local_active.get("sourceMapName") or local_map_name),
                "outputName": local_map_name if local_map_name != str(local_active.get("sourceMapName") or local_map_name) else "",
            },
        )
        loaded = self.fleet_manager.load_map_payload({"mapName": local_map_name})
        local_signature_after_push = str((pushed.get("pushed") or {}).get("signature") or local_signature).strip()
        synced_local = self.map_cache.mark_synced(
            FLEET_MANAGER_ID,
            local_map_name,
            robot_signature=local_signature_after_push,
            robot_map_name=str(loaded.get("mapName") or local_map_name),
            activate=True,
        )
        return {
            "ok": True,
            "changed": True,
            "message": f"Pushed and activated {local_map_name} on Fleet Manager.",
            "robotActiveMapName": str(loaded.get("mapName") or local_map_name),
            "localActiveMapName": local_map_name,
            "pushed": pushed.get("pushed"),
            "local": synced_local,
            "loaded": loaded,
        }

    def pull_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or "").strip()
        if robot.is_grpc:
            result = self.grpc_adapter.get_map_bundle(self._grpc_endpoint(robot), map_name)
            local_name = str(result.get("mapName") or map_name or "active").strip() or "active"
            cached = self.map_cache.save_pulled_map(robot_id, result, activate=True)
            self.workspace.save_active_map_meta(robot, {"ok": True, "mapName": str(result.get("mapName") or local_name), "signature": str(result.get("signature") or "")})
            return {
                "ok": True,
                "pulled": result,
                "local": {
                    "mapName": str(cached.get("mapName") or local_name),
                    "savedAt": str(cached.get("savedAt") or ""),
                },
            }
        raise ValueError("unsupported robot transport; use grpc")

    def local_maps_payload(self, robot_id: str) -> dict[str, Any]:
        self.get_robot(robot_id)
        return {
            "ok": True,
            "activeMapName": self.map_cache.active_map_name(robot_id),
            "maps": self.map_cache.list_maps(robot_id),
        }

    def local_active_map_payload(self, robot_id: str) -> dict[str, Any]:
        self.get_robot(robot_id)
        active_name = self.map_cache.active_map_name(robot_id)
        active_payload = self.map_cache.load_active_map(robot_id)
        if isinstance(active_payload, dict):
            robot_signature = str(active_payload.get("robotSignature") or "").strip()
            if not robot_signature:
                try:
                    robot_active = self.robot_maps_active_payload(robot_id)
                    robot_active_name = str(robot_active.get("mapName") or "").strip()
                    robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
                    local_signature = str(active_payload.get("signature") or "").strip()
                    fresh_robot_signature = str(robot_current.get("signature") or "").strip()
                    active_payload["robotSignature"] = fresh_robot_signature
                    active_payload["robotMapName"] = robot_active_name
                    active_payload["hasLocalChanges"] = bool(
                        (local_signature and fresh_robot_signature and local_signature != fresh_robot_signature)
                        or (robot_active_name and str(active_payload.get("mapName") or "").strip() != robot_active_name)
                    )
                except Exception:
                    pass
        return {
            "ok": True,
            "activeMapName": active_name,
            "map": active_payload.get("map") if isinstance(active_payload, dict) else None,
            "sourceMapName": str(active_payload.get("sourceMapName") or "") if isinstance(active_payload, dict) else "",
            "signature": str(active_payload.get("signature") or "") if isinstance(active_payload, dict) else "",
            "robotSignature": str(active_payload.get("robotSignature") or "") if isinstance(active_payload, dict) else "",
            "robotMapName": str(active_payload.get("robotMapName") or "") if isinstance(active_payload, dict) else "",
            "hasLocalChanges": bool(active_payload.get("hasLocalChanges")) if isinstance(active_payload, dict) else False,
        }

    def local_map_payload(self, robot_id: str, map_name: str) -> dict[str, Any]:
        self.get_robot(robot_id)
        payload = self.map_cache.load_map(robot_id, map_name)
        return {"ok": True, **payload}

    def save_local_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or "").strip()
        editable_map = payload.get("map")
        if not map_name:
            raise ValueError("mapName is required")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        saved = self.map_cache.save_map(
            robot_id,
            map_name,
            editable_map,
            source_map_name=str(payload.get("sourceMapName") or map_name),
            activate=bool(payload.get("activate", True)),
        )
        return {"ok": True, "local": saved}

    def activate_local_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        self.map_cache.load_map(robot_id, map_name)
        self.map_cache.set_active_map(robot_id, map_name)
        return {"ok": True, "activeMapName": map_name}

    def push_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        editable_map = payload.get("map")
        cached: dict[str, Any] | None = None
        if not isinstance(editable_map, dict):
            local_name = str(payload.get("localMapName") or payload.get("mapName") or "").strip()
            cached = self.map_cache.load_map(robot_id, local_name)
            editable_map = cached.get("map")
            payload = {
                **payload,
                "sourceMapName": str(payload.get("sourceMapName") or cached.get("sourceMapName") or local_name),
            }
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        target_map_name = str(payload.get("mapName") or editable_map.get("mapName") or "").strip()
        source_map_name = str(payload.get("sourceMapName") or target_map_name).strip()
        if robot.is_grpc:
            local_name = str(payload.get("localMapName") or target_map_name or source_map_name or editable_map.get("mapName") or "").strip()
            if not local_name:
                raise ValueError("mapName is required")
            if cached is None:
                cached = self.map_cache.save_map(
                    robot_id,
                    local_name,
                    editable_map,
                    source_map_name=source_map_name or local_name,
                    activate=True,
                )
            local_map_dir = Path(str(cached.get("path") or cached.get("mapDir") or ""))
            if not local_map_dir.is_dir():
                loaded_cache = self.map_cache.load_map(robot_id, local_name)
                local_map_dir = Path(str(loaded_cache.get("mapDir") or ""))
            if not local_map_dir.is_dir():
                raise ValueError("local map bundle is not available; pull map first")
            bundle = build_editable_map_bundle_payload(local_map_dir)
            if target_map_name:
                bundle["mapName"] = target_map_name
            result = self.grpc_adapter.put_map_bundle(
                self._grpc_endpoint(robot),
                bundle,
                map_name=target_map_name or str(bundle.get("mapName") or local_name),
                activate=False,
            )
            synced = self.map_cache.mark_synced(
                robot_id,
                local_name,
                robot_signature=str(result.get("signature") or bundle.get("signature") or ""),
                robot_map_name=str(result.get("mapName") or target_map_name or local_name),
            )
            return {"ok": True, "pushed": result, "local": synced}
        raise ValueError("unsupported robot transport; use grpc")

    def pull_sync_payload(self, robot_id: str) -> dict[str, Any]:
        robot_active = self.robot_maps_active_payload(robot_id)
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(robot_id)
        robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
        local_signature = ""
        if isinstance(local_active, dict):
            local_map = local_active.get("map")
            if isinstance(local_map, dict):
                local_signature = str(local_map.get("signature") or "").strip()
        robot_signature = str(robot_current.get("signature") or "").strip()
        local_active_name = str(local_active.get("mapName") or "") if isinstance(local_active, dict) else ""
        if robot_active_name and robot_active_name == local_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "message": f"Operator already has active robot map {robot_active_name}.",
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_active_name,
            }
        cached = self.map_cache.save_pulled_map(robot_id, robot_current, activate=True)
        robot = self.get_robot(robot_id)
        self.workspace.save_active_map_meta(
            robot,
            {"ok": True, "mapName": str(robot_current.get("mapName") or robot_active_name), "signature": str(robot_current.get("signature") or "")},
        )
        pulled = {
            "ok": True,
            "pulled": robot_current,
            "local": {
                "mapName": str(cached.get("mapName") or robot_active_name),
                "savedAt": str(cached.get("savedAt") or ""),
            },
        }
        return {
            "ok": True,
            "changed": True,
            "message": f"Pulled active robot map {robot_active_name}.",
            "robotActiveMapName": robot_active_name,
            "localActiveMapName": str(pulled.get("local", {}).get("mapName") or robot_active_name),
            **pulled,
        }

    def load_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
            if not map_name:
                raise ValueError("mapName is required")
            loaded = self.grpc_adapter.load_map(self._grpc_endpoint(robot), map_name)
            self.map_cache.set_active_map(robot_id, map_name)
            self.workspace.save_active_map_meta(robot, {"ok": True, **loaded})
            return loaded
        raise ValueError("unsupported robot transport; use grpc")

    def push_sync_payload(self, robot_id: str) -> dict[str, Any]:
        robot_active = self.robot_maps_active_payload(robot_id)
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(robot_id)
        if not isinstance(local_active, dict):
            raise ValueError("operator has no active local map")
        local_map_name = str(local_active.get("mapName") or "").strip()
        if not local_map_name:
            raise ValueError("operator active local map is invalid")
        local_map_payload = local_active.get("map")
        local_signature = str(local_map_payload.get("signature") or "").strip() if isinstance(local_map_payload, dict) else ""
        has_local_changes = bool(local_active.get("hasLocalChanges"))
        robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
        robot_signature = str(robot_current.get("signature") or "").strip()
        if (not has_local_changes) and local_map_name == robot_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "message": f"Robot already uses {robot_active_name}.",
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_map_name,
            }
        pushed = self.push_robot_map_payload(
            robot_id,
            {
                "localMapName": local_map_name,
                "mapName": local_map_name,
                "sourceMapName": str(local_active.get("sourceMapName") or local_map_name),
                "outputName": local_map_name if local_map_name != str(local_active.get("sourceMapName") or local_map_name) else "",
            },
        )
        loaded = self.load_robot_map_payload(robot_id, {"mapName": local_map_name})
        local_signature_after_push = str((pushed.get("pushed") or {}).get("signature") or local_signature).strip()
        synced_local = self.map_cache.mark_synced(
            robot_id,
            local_map_name,
            robot_signature=local_signature_after_push,
            robot_map_name=str(loaded.get("mapName") or local_map_name),
            activate=True,
        )
        self.workspace.save_active_map_meta(self.get_robot(robot_id), {"ok": True, **loaded})
        return {
            "ok": True,
            "changed": True,
            "message": f"Pushed and activated {local_map_name} on robot.",
            "robotActiveMapName": str(loaded.get("mapName") or local_map_name),
            "localActiveMapName": local_map_name,
            "pushed": pushed.get("pushed"),
            "local": synced_local,
            "loaded": loaded,
        }

    def _fetch_robot_map_payload(self, robot_id: str, map_name: str) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            return self.grpc_adapter.get_map_bundle(self._grpc_endpoint(robot), map_name)
        raise ValueError("unsupported robot transport; use grpc")

    def watch_robot_laser_scan(
        self,
        robot_id: str,
        *,
        topic: str = "/scan",
        hz: float = 1.0,
        include_intensities: bool = False,
    ) -> Any:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        return self.grpc_adapter.watch_laser_scan(
            self._grpc_endpoint(robot),
            topic=topic,
            hz=hz,
            include_intensities=include_intensities,
        )

    def robot_slam_defaults_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        result = self.grpc_adapter.get_slam_defaults(self._grpc_endpoint(robot))
        result["robotId"] = robot_id
        return result

    def robot_slam_state_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        result = self.grpc_adapter.get_slam_state(self._grpc_endpoint(robot))
        result["robotId"] = robot_id
        return result

    def start_robot_slam_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        params = payload.get("params") if isinstance(payload.get("params"), dict) else payload
        result = self.grpc_adapter.start_slam(
            self._grpc_endpoint(robot),
            params if isinstance(params, dict) else {},
            use_sim_time=bool(payload.get("useSimTime", True)),
        )
        result["robotId"] = robot_id
        return result

    def finish_robot_slam_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        map_name = str(payload.get("mapName") or payload.get("map_name") or "").strip()
        result = self.grpc_adapter.finish_slam(
            self._grpc_endpoint(robot),
            map_name=map_name,
            activate=bool(payload.get("activate", True)),
        )
        bundle = result.get("bundle") if isinstance(result.get("bundle"), dict) else None
        if isinstance(bundle, dict):
            cached = self.map_cache.save_pulled_map(robot.id, bundle, activate=True)
            result["local"] = cached
            self.workspace.save_active_map_meta(
                robot,
                {
                    "ok": True,
                    "mapName": str(result.get("mapName") or map_name),
                    "mapDir": str(result.get("mapDir") or ""),
                    "mapId": str(result.get("mapId") or ""),
                    "signature": str(result.get("signature") or ""),
                },
            )
            try:
                self.workspace.save_map_index(robot, self.grpc_adapter.list_maps(self._grpc_endpoint(robot)))
            except Exception:
                pass
        result["robotId"] = robot_id
        return result

    def cancel_robot_slam_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        result = self.grpc_adapter.cancel_slam(
            self._grpc_endpoint(robot),
            reason=str(payload.get("reason") or "SLAM canceled by operator."),
        )
        result["robotId"] = robot_id
        return result

    def watch_robot_slam_map(
        self,
        robot_id: str,
        *,
        hz: float = 1.0,
        include_cells: bool = True,
    ) -> Any:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        return self.grpc_adapter.watch_slam_map(
            self._grpc_endpoint(robot),
            hz=hz,
            include_cells=include_cells,
        )

    def robot_teleop_stream(self, robot_id: str, commands) -> Any:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        return self.grpc_adapter.teleop_stream(
            self._grpc_endpoint(robot),
            commands,
            owner_id=OPERATOR_CONTROL_OWNER_ID,
        )

    def _proxy_grpc_robot_request(self, robot_id: str, method: str, path: str, *, body: bytes | None) -> tuple[int, dict[str, str], bytes]:
        robot = self.get_robot(robot_id)
        endpoint = self._grpc_endpoint(robot)
        parsed = urlparse(path)
        route = parsed.path.rstrip("/") or "/"
        payload: dict[str, Any] = {}
        if body:
            try:
                decoded = json.loads(body.decode("utf-8"))
            except json.JSONDecodeError as exc:
                raise ValueError("invalid JSON") from exc
            if isinstance(decoded, dict):
                payload = decoded
        method = method.upper()
        try:
            if method == "GET" and route == "/health":
                return self._json_response_tuple(self.grpc_adapter.client.health(endpoint))
            if method == "GET" and route == "/api/robot/identity":
                return self._json_response_tuple(self.grpc_adapter.identity(endpoint))
            if method == "GET" and route == "/api/robot/status":
                return self._json_response_tuple(self.grpc_adapter.status(endpoint))
            if method == "POST" and route == "/api/robot/teleop":
                return self._json_response_tuple(
                    self.grpc_adapter.teleop(
                        endpoint,
                        linear=float(payload.get("linear", 0.0) or 0.0),
                        angular=float(payload.get("angular", 0.0) or 0.0),
                        timeout_ms=int(payload.get("timeoutMs", 350) or 350),
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                    )
                )
            if method == "POST" and route == "/api/robot/teleop/stop":
                return self._json_response_tuple(self.grpc_adapter.teleop_stop(endpoint, owner_id=OPERATOR_CONTROL_OWNER_ID))
            if method == "POST" and route == "/api/robot/stop":
                return self._json_response_tuple(self.grpc_adapter.stop(endpoint, owner_id=OPERATOR_CONTROL_OWNER_ID))
            if method == "POST" and route == "/api/robot/control/acquire":
                return self._json_response_tuple(
                    self.grpc_adapter.acquire_control(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        owner_name=OPERATOR_CONTROL_OWNER_NAME,
                        force=bool(payload.get("force")),
                        lease_ms=int(payload.get("leaseMs", payload.get("lease_ms", 0)) or 0),
                    )
                )
            if method == "POST" and route == "/api/robot/control/release":
                return self._json_response_tuple(
                    self.grpc_adapter.release_control(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        force=bool(payload.get("force")),
                    )
                )
            if method == "POST" and route == "/api/robot/relocate":
                pose = payload.get("pose") if isinstance(payload.get("pose"), dict) else payload
                return self._json_response_tuple(
                    self.grpc_adapter.relocate(
                        endpoint,
                        x=float(pose.get("x", 0.0) or 0.0),
                        y=float(pose.get("y", 0.0) or 0.0),
                        yaw=float(pose.get("yaw", pose.get("theta", 0.0)) or 0.0),
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        frame_id=str(payload.get("frameId") or payload.get("frame_id") or "map"),
                        covariance=payload.get("covariance") if isinstance(payload.get("covariance"), list) else None,
                        confirm=bool(payload.get("confirm")),
                    )
                )
            if method == "POST" and route == "/api/robot/localization/confirm":
                return self._json_response_tuple(
                    self.grpc_adapter.confirm_localization(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        accepted=bool(payload.get("accepted", True)),
                        message=str(payload.get("message") or ""),
                    )
                )
            if method == "POST" and route == "/api/robot/route/cancel":
                return self._json_response_tuple(self.grpc_adapter.cancel_route(endpoint, owner_id=OPERATOR_CONTROL_OWNER_ID))
            if method == "POST" and route == "/api/robot/route/pause":
                return self._json_response_tuple(
                    self.grpc_adapter.pause_route(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        message=str(payload.get("message") or ""),
                    )
                )
            if method == "POST" and route == "/api/robot/route/resume":
                return self._json_response_tuple(
                    self.grpc_adapter.resume_route(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        message=str(payload.get("message") or ""),
                    )
                )
            if method == "POST" and route == "/api/robot/route/execute":
                payload.setdefault("ownerId", OPERATOR_CONTROL_OWNER_ID)
                return self._json_response_tuple(self.grpc_adapter.execute_route(endpoint, payload))
        except GrpcRobotError as exc:
            return self._json_response_tuple({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:
            return self._json_response_tuple({"ok": False, "error": str(exc)}, status=500)
        return self._json_response_tuple({"ok": False, "error": f"unsupported gRPC robot path: {method} {route}"}, status=404)

    def _json_response_tuple(self, payload: dict[str, Any], *, status: int = 200) -> tuple[int, dict[str, str], bytes]:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        return status, {"Content-Type": "application/json; charset=utf-8"}, encoded

    @staticmethod
    def _require_host(payload: dict[str, Any]) -> str:
        host = str(payload.get("host") or "").strip()
        if not host:
            raise ValueError("host is required")
        if host in {"0.0.0.0", "::"}:
            raise ValueError(
                f"{host} is a listen/bind address, not a robot address. "
                "Use 127.0.0.1 on the same PC or the robot LAN IP like 192.168.x.x."
            )
        if any(ch.isspace() for ch in host):
            raise ValueError("robot host must not contain whitespace")
        try:
            ipaddress.ip_address(host)
        except ValueError:
            if not all(ch.isalnum() or ch in ".-_" for ch in host):
                raise ValueError("robot host must be an IP address or DNS name")
        return host

    @staticmethod
    def _require_port(payload: dict[str, Any], *, default: int = DEFAULT_GRPC_PORT) -> int:
        raw = payload.get("port", default)
        try:
            port = int(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("port must be an integer") from exc
        if port < 1 or port > 65535:
            raise ValueError("port must be in range 1..65535")
        return port

    @staticmethod
    def _payload_robot_type(payload: dict[str, Any]) -> str:
        return str(payload.get("type") or payload.get("mode") or "grpc").strip().lower() or "grpc"
