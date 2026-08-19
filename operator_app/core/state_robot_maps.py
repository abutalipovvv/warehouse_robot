"""Synchronize robot maps and parameter caches."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from fleet_manager.core.mapping.maps.map_exchange import (
    build_editable_map_bundle_payload,
    editable_map_signature,
)


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
            endpoint = self._grpc_endpoint(robot)
            result = self.grpc_adapter.get_map_bundle(endpoint, map_name)
            local_name = str(result.get("mapName") or map_name or "active").strip() or "active"
            cached = self.map_cache.save_pulled_map(robot_id, result, activate=True)
            active_warning = ""
            active: dict[str, Any] = {}
            try:
                active = self.grpc_adapter.active_map(endpoint)
                self.workspace.save_active_map_meta(robot, active)
            except Exception as exc:
                active_warning = str(exc)
            pulled_signature = str(result.get("signature") or "").strip()
            activation_required = bool(
                not active_warning
                and (
                    str(active.get("mapName") or "").strip() != local_name
                    or str(active.get("signature") or "").strip() != pulled_signature
                )
            )
            return {
                "ok": True,
                "pulled": result,
                "local": {
                    "mapName": str(cached.get("mapName") or local_name),
                    "savedAt": str(cached.get("savedAt") or ""),
                },
                "activationRequired": activation_required,
                **({"warning": f"active map verification failed after Pull: {active_warning}"} if active_warning else {}),
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
        robot_active_name = ""
        robot_active_signature = ""
        stored_signature = ""
        remote_verified = False
        remote_error = ""
        if isinstance(active_payload, dict) and self._is_grpc_robot_id(robot_id):
            local_name = str(active_payload.get("mapName") or active_name).strip()
            local_signature = str(active_payload.get("signature") or "").strip()
            try:
                robot = self.get_robot(robot_id)
                remote_index = self.grpc_adapter.list_maps(self._grpc_endpoint(robot))
                robot_active_name = str(remote_index.get("active") or "").strip()
                robot_active_signature = str(remote_index.get("activeSignature") or "").strip()
                remote_item = next(
                    (
                        item
                        for item in remote_index.get("maps", [])
                        if isinstance(item, dict)
                        and str(item.get("name") or "").strip() == local_name
                    ),
                    None,
                )
                stored_signature = (
                    str(remote_item.get("signature") or "").strip()
                    if isinstance(remote_item, dict)
                    else ""
                )
                if isinstance(remote_item, dict) and not stored_signature:
                    remote_bundle = self._fetch_robot_map_payload(robot_id, local_name)
                    stored_signature = str(remote_bundle.get("signature") or "").strip()
                remote_verified = bool(stored_signature)
                active_payload["robotSignature"] = stored_signature
                active_payload["robotMapName"] = local_name if remote_verified else ""
                active_payload["hasLocalChanges"] = bool(
                    not remote_verified or not local_signature or local_signature != stored_signature
                )
            except Exception as exc:
                remote_error = str(exc)
        local_signature = str(active_payload.get("signature") or "").strip() if isinstance(active_payload, dict) else ""
        has_local_changes = bool(active_payload.get("hasLocalChanges")) if isinstance(active_payload, dict) else False
        activation_required = bool(
            isinstance(active_payload, dict)
            and remote_verified
            and not has_local_changes
            and (
                robot_active_name != str(active_payload.get("mapName") or active_name).strip()
                or robot_active_signature != local_signature
            )
        )
        if not isinstance(active_payload, dict):
            sync_state = "local_missing"
        elif remote_error:
            sync_state = "unverified"
        elif not remote_verified:
            sync_state = "local_only"
        elif has_local_changes:
            sync_state = "local_changes"
        elif activation_required:
            sync_state = "load_required"
        else:
            sync_state = "synchronized"
        return {
            "ok": True,
            "activeMapName": active_name,
            "map": active_payload.get("map") if isinstance(active_payload, dict) else None,
            "sourceMapName": str(active_payload.get("sourceMapName") or "") if isinstance(active_payload, dict) else "",
            "signature": local_signature,
            "robotSignature": stored_signature or (str(active_payload.get("robotSignature") or "") if isinstance(active_payload, dict) else ""),
            "robotMapName": str(active_payload.get("robotMapName") or "") if isinstance(active_payload, dict) else "",
            "robotActiveMapName": robot_active_name,
            "robotActiveSignature": robot_active_signature,
            "hasLocalChanges": has_local_changes,
            "activationRequired": activation_required,
            "remoteVerified": remote_verified,
            "syncState": sync_state,
            **({"remoteError": remote_error} if remote_error else {}),
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
                bundle["signature"] = editable_map_signature(bundle)
            result = self.grpc_adapter.put_map_bundle(
                self._grpc_endpoint(robot),
                bundle,
                map_name=target_map_name or str(bundle.get("mapName") or local_name),
                activate=False,
            )
            read_back = self.grpc_adapter.get_map_bundle(
                self._grpc_endpoint(robot),
                str(result.get("mapName") or target_map_name or local_name),
            )
            expected_signature = str(bundle.get("signature") or "").strip()
            stored_signature = str(read_back.get("signature") or "").strip()
            if not expected_signature or stored_signature != expected_signature:
                raise ValueError(
                    "robot map verification failed after Push: "
                    f"expected {expected_signature or '-'}, got {stored_signature or '-'}"
                )
            synced = self.map_cache.mark_synced(
                robot_id,
                local_name,
                robot_signature=stored_signature,
                robot_map_name=str(result.get("mapName") or target_map_name or local_name),
            )
            return {"ok": True, "pushed": result, "verified": read_back, "local": synced}
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
        robot_runtime_signature = str(robot_active.get("signature") or "").strip() if not active_warning else ""
        local_active_name = str(local_active.get("mapName") or "") if isinstance(local_active, dict) else ""
        load_required = bool(
            not active_warning
            and (
                robot_active_name != str(robot_current.get("mapName") or "").strip()
                or robot_runtime_signature != robot_signature
            )
        )
        if robot_active_name and robot_active_name == local_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "loadRequired": load_required,
                "message": (
                    f"Operator already has stored robot map {robot_active_name}; Load is required."
                    if load_required
                    else f"Operator already has active robot map {robot_active_name}."
                ),
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
            "loadRequired": load_required,
            "message": (
                f"Pulled stored robot map {robot_active_name}; Load is required to activate it."
                if load_required
                else f"Pulled active robot map {robot_active_name}."
            ),
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
            endpoint = self._grpc_endpoint(robot)
            previous = self.grpc_adapter.active_map(endpoint)
            loaded = self.grpc_adapter.load_map(endpoint, map_name)
            try:
                active = self.grpc_adapter.active_map(endpoint)
                loaded_name = str(active.get("mapName") or loaded.get("mapName") or "").strip()
                requested_name = map_name.removesuffix(".smap")
                if loaded_name != requested_name:
                    raise ValueError(
                        f"robot activated {loaded_name or '-'} instead of {requested_name}"
                    )
                bundle = self.grpc_adapter.get_map_bundle(endpoint, loaded_name)
                bundle_signature = str(bundle.get("signature") or "").strip()
                active_signature = str(active.get("signature") or loaded.get("signature") or "").strip()
                if not bundle_signature or active_signature != bundle_signature:
                    raise ValueError(
                        "robot active map verification failed after Load: "
                        f"runtime={active_signature or '-'} stored={bundle_signature or '-'}"
                    )
                self.map_cache.save_pulled_map(robot_id, bundle, activate=True)
                local_active = self.map_cache.load_active_map(robot_id)
                if not isinstance(local_active, dict):
                    raise ValueError("operator failed to activate the verified robot map")
                self.workspace.save_active_map_meta(robot, {"ok": True, **active})
                index = self.grpc_adapter.list_maps(endpoint)
                self.workspace.save_map_index(robot, index)
                return {
                    **loaded,
                    "signature": bundle_signature,
                    "verified": True,
                    "local": {
                        "activeMapName": str(local_active.get("mapName") or loaded_name),
                        "map": local_active.get("map"),
                        "sourceMapName": str(local_active.get("sourceMapName") or loaded_name),
                        "signature": str(local_active.get("signature") or ""),
                        "robotSignature": bundle_signature,
                        "robotMapName": loaded_name,
                        "hasLocalChanges": False,
                        "activationRequired": False,
                        "syncState": "synchronized",
                    },
                }
            except Exception as exc:
                previous_name = str(previous.get("mapName") or "").strip()
                rollback_error = ""
                if previous_name and previous_name != map_name.removesuffix(".smap"):
                    try:
                        self.grpc_adapter.load_map(endpoint, previous_name)
                    except Exception as rollback_exc:
                        rollback_error = f"; robot rollback failed: {rollback_exc}"
                raise ValueError(f"Load verification failed: {exc}{rollback_error}") from exc
        raise ValueError("unsupported robot transport; use grpc")

    def push_sync_payload(self, robot_id: str) -> dict[str, Any]:
        local_active = self.map_cache.load_active_map(robot_id)
        if not isinstance(local_active, dict):
            raise ValueError("operator has no active local map")
        local_map_name = str(local_active.get("mapName") or "").strip()
        if not local_map_name:
            raise ValueError("operator active local map is invalid")
        local_map_payload = local_active.get("map")
        local_signature = str(local_map_payload.get("signature") or "").strip() if isinstance(local_map_payload, dict) else ""
        if not local_signature:
            raise ValueError("operator active local map has no verifiable signature")
        robot = self.get_robot(robot_id)
        endpoint = self._grpc_endpoint(robot)
        before = self.grpc_adapter.list_maps(endpoint)
        robot_active_name = str(before.get("active") or "").strip()
        robot_active_signature = str(before.get("activeSignature") or "").strip()
        stored_item = next(
            (
                item
                for item in before.get("maps", [])
                if isinstance(item, dict)
                and str(item.get("name") or "").strip() == local_map_name
            ),
            None,
        )
        stored_signature = str(stored_item.get("signature") or "").strip() if isinstance(stored_item, dict) else ""
        if not stored_signature and isinstance(stored_item, dict):
            stored_signature = str(
                self._fetch_robot_map_payload(robot_id, local_map_name).get("signature") or ""
            ).strip()
        if stored_signature == local_signature:
            synced_local = self.map_cache.mark_synced(
                robot_id,
                local_map_name,
                robot_signature=stored_signature,
                robot_map_name=local_map_name,
                activate=True,
            )
            load_required = bool(
                robot_active_name != local_map_name
                or robot_active_signature != local_signature
            )
            return {
                "ok": True,
                "changed": False,
                "loadRequired": load_required,
                "message": (
                    f"Robot already stores {local_map_name}; use Load to activate it."
                    if load_required
                    else f"Robot already stores and uses {local_map_name}."
                ),
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_map_name,
                "local": synced_local,
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
        after = self.grpc_adapter.list_maps(endpoint)
        self.workspace.save_map_index(robot, after)
        robot_active_name = str(after.get("active") or robot_active_name).strip()
        robot_active_signature = str(after.get("activeSignature") or robot_active_signature).strip()
        load_required = bool(
            robot_active_name != local_map_name
            or robot_active_signature != local_signature
        )
        return {
            "ok": True,
            "changed": True,
            "loadRequired": load_required,
            "message": (
                f"Uploaded and verified {local_map_name}. Use Load to activate it on the robot."
                if load_required
                else f"Uploaded and verified active map {local_map_name}."
            ),
            "robotActiveMapName": robot_active_name,
            "localActiveMapName": local_map_name,
            "pushed": pushed.get("pushed"),
            "verified": pushed.get("verified"),
            "local": pushed.get("local"),
        }

    def _fetch_robot_map_payload(self, robot_id: str, map_name: str) -> dict[str, Any]:
        if self._is_grpc_robot_id(robot_id):
            robot = self.get_robot(robot_id)
            return self.grpc_adapter.get_map_bundle(self._grpc_endpoint(robot), map_name)
        raise ValueError("unsupported robot transport; use grpc")
