"""Synchronize robot maps and parameter caches."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from fleet_manager.core.mapping.maps.map_exchange import build_editable_map_bundle_payload


class RobotMapSyncMixin:
    """Synchronize robot maps and parameter caches."""

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
        active_warning = ""
        try:
            robot_active = self.robot_maps_active_payload(robot_id)
            robot_active_name = str(robot_active.get("mapName") or "").strip()
        except Exception as exc:
            active_warning = str(exc)
            robot_active_name = ""
        local_active = self.map_cache.load_active_map(robot_id)
        robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
        robot_active_name = str(robot_current.get("mapName") or robot_active_name).strip()
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
                **({"warning": f"Active map lookup failed before pull: {active_warning}"} if active_warning else {}),
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
            **({"warning": f"Active map lookup failed before pull: {active_warning}"} if active_warning else {}),
            **pulled,
        }

    def load_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
            if not map_name:
                raise ValueError("mapName is required")
            loaded = self.grpc_adapter.load_map(self._grpc_endpoint(robot), map_name)
            local_payload: dict[str, Any] | None = None
            local_warning = ""
            try:
                self.map_cache.set_active_map(robot_id, map_name)
                active_payload = self.map_cache.load_active_map(robot_id)
                if isinstance(active_payload, dict):
                    local_payload = {
                        "activeMapName": str(active_payload.get("mapName") or map_name),
                        "map": active_payload.get("map") if isinstance(active_payload.get("map"), dict) else None,
                        "sourceMapName": str(active_payload.get("sourceMapName") or ""),
                        "signature": str(active_payload.get("signature") or ""),
                        "robotSignature": str(active_payload.get("robotSignature") or loaded.get("signature") or ""),
                        "robotMapName": str(active_payload.get("robotMapName") or loaded.get("mapName") or map_name),
                        "hasLocalChanges": bool(active_payload.get("hasLocalChanges")),
                    }
            except ValueError as exc:
                local_warning = str(exc)
            self.workspace.save_active_map_meta(robot, {"ok": True, **loaded})
            if local_payload is not None:
                loaded["local"] = local_payload
            if local_warning:
                loaded["warning"] = local_warning
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


__all__ = ["RobotMapSyncMixin"]
