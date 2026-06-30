from __future__ import annotations

from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FLEET_ROOT = PROJECT_ROOT / "fleet_manager"
FLEET_MAP_DATA_ROOT = FLEET_ROOT / "map_data"
FLEET_MAPS_OUT_ROOT = FLEET_MAP_DATA_ROOT / "maps_out"
DEFAULT_FLEET_MAP_DIR = FLEET_MAPS_OUT_ROOT / "22.05.26_smap.smap"
FLEET_MANAGER_ID = "__fleet_manager__"

from fleet_manager.web_simulator import FleetManager
from fleet_manager.route_core import (
    WarehouseMapLoader,
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    list_editable_maps,
    load_route_params,
    save_route_params,
    save_editable_map,
)


class OperatorFleetManager:
    def __init__(self, map_dir: Path, params_path: Path, remote_adapter: Any | None = None) -> None:
        self.params_path = Path(params_path).expanduser().resolve()
        self.remote_adapter = remote_adapter
        self.mode = "simulation"
        self.map_dir = self.resolve_map_dir(map_dir)
        self.maps_root = self.map_dir.parent
        self._load_context(self.map_dir)

    def sidebar_payload(self, include_runtime: bool = True) -> dict[str, Any]:
        robots = []
        if include_runtime:
            state = self.state_payload(include_trajectories=False)
            robots = state.get("robots", [])
        return {
            "id": FLEET_MANAGER_ID,
            "name": "Fleet Manager",
            "type": "fleet_manager",
            "online": True,
            "host": "local",
            "port": 0,
            "baseUrl": "",
            "identity": {
                "robotId": "fleet-manager",
                "mapId": self.map_dir.stem.replace(".smap", ""),
                "type": "fleet_manager",
            },
            "status": {
                "state": self.mode.upper(),
                "robots": len(robots) if isinstance(robots, list) else 0,
            },
            "runtimeFresh": include_runtime,
            "error": "",
        }

    def mode_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "id": FLEET_MANAGER_ID,
            "mode": self.mode,
            "mapName": self.map_dir.stem.replace(".smap", ""),
        }

    def set_mode_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode") or "").strip().lower()
        if mode not in {"simulation", "robots"}:
            raise ValueError("mode must be simulation or robots")
        self.mode = mode
        self._sync_manager_mode()
        return self.mode_payload()

    def params_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "path": str(self.params_path),
            "params": load_route_params(self.params_path, create=True),
        }

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        params_payload = payload.get("params")
        if not isinstance(params_payload, dict):
            params_payload = payload
        save_route_params(params_payload, self.params_path)
        self._load_context(self.map_dir)
        return self.params_payload()

    def map_payload(self) -> dict[str, Any]:
        params = load_route_params(self.params_path, create=True)
        return build_editable_map_payload(self.map_dir, params=params)

    def maps_active_payload(self) -> dict[str, Any]:
        payload = self.map_payload()
        return {
            "ok": True,
            "mapName": str(payload.get("mapName") or self.map_dir.stem.replace(".smap", "")),
            "mapDir": str(self.map_dir),
            "signature": str(payload.get("signature") or ""),
        }

    def maps_list_payload(self) -> dict[str, Any]:
        return list_editable_maps(self.maps_root, active_map_dir=self.map_dir)

    def pull_map_payload(self, map_name: str = "") -> dict[str, Any]:
        target = self._resolve_map_dir_by_name(map_name)
        params = load_route_params(self.params_path, create=True)
        return build_editable_map_bundle_payload(target, params=params)

    def push_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        target_name = str(payload.get("mapName") or "").strip()
        source_name = str(payload.get("sourceMapName") or target_name).strip()
        source_dir = self._resolve_map_dir_by_name(source_name)
        output_name = str(payload.get("outputName") or "").strip()
        if not output_name and target_name and source_name and target_name != source_name:
            output_name = target_name
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        loaded_map = save_editable_map(
            source_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(payload.get("overwriteOutput", False)),
        )
        params = load_route_params(self.params_path, create=True)
        return {
            **build_editable_map_payload(loaded_map.map_dir, params=params),
            "savedAs": bool(output_name),
        }

    def load_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        safe_name = Path(map_name).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        if self.maps_root.resolve() not in target.parents and target != self.maps_root.resolve():
            raise ValueError("map must stay inside maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        self._load_context(target)
        return {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "mode": self.mode,
        }

    def save_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            editable_map = payload
        output_name = str(payload.get("outputName") or "").strip()
        loaded_map = save_editable_map(
            self.map_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(payload.get("overwriteOutput", False)),
        )
        self._load_context(loaded_map.map_dir)
        return {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "map": self.map_payload(),
            "maps": self.maps_list_payload(),
        }

    def state_payload(self, include_trajectories: bool = True) -> dict[str, Any]:
        self._sync_manager_mode()
        state = self.manager.state(include_trajectories=include_trajectories)
        return self._state_with_context(state)

    def plan_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        result = self.manager.plan(payload)
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self._result_with_context(result)

    def orders_payload(self) -> dict[str, Any]:
        self._sync_manager_mode()
        result = self.manager.orders_payload()
        return self._result_with_context(result)

    def set_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.set_order(payload))

    def dispatch_orders_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.dispatch_orders(payload or {}))

    def cancel_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.cancel_order(payload))

    def pause_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.pause_order(payload))

    def resume_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.resume_order(payload))

    def clear_orders_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.clear_orders(payload or {}))

    def tick_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        state = self.manager.tick(payload or {})
        return self._state_with_context(state)

    def world_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_world(payload))

    def check_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self.manager.check_path(payload)

    def manual_step_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if self.mode == "robots":
            result = self.manager.teleop_robot(
                {
                    "name": name,
                    "linear": float(payload.get("linear", 0.0) or 0.0),
                    "angular": float(payload.get("angular", 0.0) or 0.0),
                    "timeoutMs": int(payload.get("timeoutMs", 350) or 350),
                }
            )
            return {
                "ok": True,
                "blocked": False,
                "reason": "",
                "robot": result.get("robot"),
                "state": self._state_with_context(result.get("state")),
            }
        poses = payload.get("poses", [])
        check = self.manager.check_path({"name": name, "poses": poses})
        update_payload = {
            "name": name,
            "status": "MANUAL_BLOCKED" if check.get("blocked") else "MANUAL",
            "targetLm": "",
            "currentLm": str(
                payload.get("blockedCurrentLm" if check.get("blocked") else "currentLm")
                or payload.get("currentLm")
                or ""
            ),
        }
        pose_key = "blockedPose" if check.get("blocked") else "nextPose"
        pose = payload.get(pose_key)
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        result = self.manager.update_robot(update_payload)
        return {
            "ok": True,
            "blocked": bool(check.get("blocked")),
            "reason": str(check.get("reason") or ""),
            "index": check.get("index"),
            "pose": check.get("pose"),
            "robot": result.get("robot"),
            "state": self._state_with_context(result.get("state")),
        }

    def manual_stop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if self.mode == "robots":
            result = self.manager.teleop_stop_robot({"name": name})
            return {
                "ok": True,
                "robot": result.get("robot"),
                "state": self._state_with_context(result.get("state")),
            }

        update_payload = {
            "name": name,
            "status": "IDLE",
            "targetLm": "",
            "currentLm": str(payload.get("currentLm") or ""),
        }
        pose = payload.get("pose")
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        result = self.manager.update_robot(update_payload)
        return {
            "ok": True,
            "robot": result.get("robot"),
            "state": self._state_with_context(result.get("state")),
        }

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.add_robot(payload))

    def remove_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.remove_robot(payload))

    def update_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_robot(payload))

    def stop_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.stop_robot(payload))

    def reset_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.reset_robot(payload))

    def resolve_map_dir(self, map_dir: Path) -> Path:
        candidate = Path(map_dir).expanduser()
        safe_name = Path(candidate).name
        maps_root = FLEET_MAPS_OUT_ROOT.resolve()
        if candidate.is_absolute():
            resolved = candidate.resolve()
            if resolved.exists() and (resolved == maps_root or maps_root in resolved.parents):
                return resolved
            candidate = Path(safe_name)

        candidates = [
            FLEET_MAPS_OUT_ROOT / candidate,
            FLEET_MAP_DATA_ROOT / candidate,
            FLEET_ROOT / candidate,
            FLEET_MAPS_OUT_ROOT / safe_name,
        ]
        if not safe_name.endswith(".smap"):
            candidates.append(FLEET_MAPS_OUT_ROOT / f"{safe_name}.smap")
        for item in candidates:
            resolved = item.resolve()
            if resolved.exists() and (resolved == maps_root or maps_root in resolved.parents):
                return resolved
        return DEFAULT_FLEET_MAP_DIR.resolve()

    def _resolve_map_dir_by_name(self, map_name: str) -> Path:
        if not map_name:
            return self.map_dir
        safe_name = Path(str(map_name)).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        root = self.maps_root.resolve()
        if root not in target.parents and target != root:
            raise ValueError("map must stay inside fleet maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        return target

    def _load_context(self, map_dir: Path) -> None:
        loaded_map = WarehouseMapLoader(map_dir).load()
        params = load_route_params(self.params_path, create=True)
        self.map_dir = loaded_map.map_dir.resolve()
        self.maps_root = self.map_dir.parent
        self.manager = FleetManager(
            loaded_map.landmarks,
            loaded_map.edges,
            params=params,
            map_dir=loaded_map.map_dir,
            map_metadata=loaded_map.map_metadata,
            remote_adapter=self.remote_adapter,
        )
        self._sync_manager_mode()

    def _sync_manager_mode(self) -> None:
        if not hasattr(self, "manager"):
            return
        self.manager.set_active_robot_modes(self._active_robot_modes())

    def _active_robot_modes(self) -> set[str]:
        if self.mode == "robots":
            return {"remote"}
        return {"simulated"}

    def _state_with_context(self, state: Any) -> dict[str, Any]:
        if not isinstance(state, dict):
            state = self.manager.state()
        state["mode"] = self.mode
        state["mapName"] = self.map_dir.stem.replace(".smap", "")
        state["managerId"] = FLEET_MANAGER_ID
        return state

    def _result_with_context(self, result: dict[str, Any]) -> dict[str, Any]:
        state = result.get("state")
        if isinstance(state, dict):
            result["state"] = self._state_with_context(state)
        result["mode"] = self.mode
        result["mapName"] = self.map_dir.stem.replace(".smap", "")
        result["managerId"] = FLEET_MANAGER_ID
        return result
