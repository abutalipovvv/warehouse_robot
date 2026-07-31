"""Synchronize fleet manager maps with the local workspace."""

from __future__ import annotations

from typing import Any

from .fleet_manager import (
    FLEET_MANAGER_ID,
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)


class FleetMapSyncMixin:
    """Synchronize fleet manager maps with the local workspace."""

    def fleet_local_maps_payload(self, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        return {
            "ok": True,
            "activeMapName": self.map_cache.active_map_name(manager_id),
            "maps": self.map_cache.list_maps(manager_id),
        }

    def fleet_local_active_map_payload(self, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        active_name = self.map_cache.active_map_name(manager_id)
        active_payload = self.map_cache.load_active_map(manager_id)
        robot_active = manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        robot_signature = str(robot_active.get("signature") or "").strip()
        sync_warning = ""
        local_active_name = str(active_payload.get("mapName") or active_name).strip() if isinstance(active_payload, dict) else active_name
        if manager_id == FLEET_MANAGER_SIM_ID and robot_active_name and local_active_name != robot_active_name:
            try:
                active_payload = self._sync_fleet_local_map_from_manager(
                    manager,
                    manager_id,
                    robot_active_name,
                    robot_active=robot_active,
                )
                active_name = self.map_cache.active_map_name(manager_id)
            except Exception as exc:
                sync_warning = str(exc)
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
            **({"warning": sync_warning} if sync_warning else {}),
        }

    def fleet_local_map_payload(self, map_name: str, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        payload = self.map_cache.load_map(manager_id, map_name)
        return {"ok": True, **payload}

    def _fleet_local_response(
        self,
        active_payload: dict[str, Any] | None,
        *,
        active_name: str,
        robot_active_name: str,
        robot_signature: str,
    ) -> dict[str, Any] | None:
        if not isinstance(active_payload, dict):
            return None
        map_payload = active_payload.get("map") if isinstance(active_payload.get("map"), dict) else None
        local_name = str(active_payload.get("mapName") or active_name).strip()
        local_signature = str(active_payload.get("signature") or "").strip()
        robot_map_name = str(active_payload.get("robotMapName") or active_payload.get("sourceMapName") or robot_active_name).strip()
        robot_sig = str(active_payload.get("robotSignature") or robot_signature).strip()
        has_local_changes = bool(
            active_payload.get("hasLocalChanges")
            or (local_signature and robot_sig and local_signature != robot_sig)
            or (local_name and robot_map_name and local_name != robot_map_name)
        )
        return {
            "activeMapName": local_name,
            "mapName": local_name,
            "map": map_payload,
            "sourceMapName": str(active_payload.get("sourceMapName") or robot_map_name or local_name),
            "signature": local_signature,
            "robotSignature": robot_sig,
            "robotMapName": robot_map_name,
            "hasLocalChanges": has_local_changes,
        }

    def _sync_fleet_local_map_from_manager(
        self,
        manager: OperatorFleetManager,
        manager_id: str,
        map_name: str,
        *,
        robot_active: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        pulled = manager.pull_map_payload(map_name)
        pulled_name = str(pulled.get("mapName") or map_name).strip()
        cached = self.map_cache.save_pulled_map(manager_id, pulled, activate=True)
        active_payload = self.map_cache.load_active_map(manager_id)
        active_name = str(cached.get("mapName") or pulled_name or self.map_cache.active_map_name(manager_id)).strip()
        robot_active_name = str((robot_active or {}).get("mapName") or pulled_name or map_name).strip()
        robot_signature = str((robot_active or {}).get("signature") or pulled.get("signature") or "").strip()
        return self._fleet_local_response(
            active_payload,
            active_name=active_name,
            robot_active_name=robot_active_name,
            robot_signature=robot_signature,
        )

    def fleet_load_map_payload(
        self,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        loaded = manager.load_map_payload(payload)
        map_name = str(loaded.get("mapName") or payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            return loaded
        try:
            local_payload = self._sync_fleet_local_map_from_manager(
                manager,
                manager_id,
                map_name,
                robot_active=loaded,
            )
            if isinstance(local_payload, dict):
                loaded["local"] = local_payload
                loaded.setdefault("signature", str(local_payload.get("robotSignature") or local_payload.get("signature") or ""))
        except Exception as exc:
            loaded["warning"] = str(exc)
        return loaded

    def fleet_save_local_map_payload(
        self,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or "").strip()
        editable_map = payload.get("map")
        if not map_name:
            raise ValueError("mapName is required")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        saved = self.map_cache.save_map(
            manager_id,
            map_name,
            editable_map,
            source_map_name=str(payload.get("sourceMapName") or map_name),
            activate=bool(payload.get("activate", True)),
        )
        return {"ok": True, "local": saved}

    def fleet_activate_local_map_payload(
        self,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        self.map_cache.load_map(manager_id, map_name)
        self.map_cache.set_active_map(manager_id, map_name)
        return {"ok": True, "activeMapName": map_name}

    def fleet_pull_map_payload(
        self,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        map_name = str(payload.get("mapName") or "").strip()
        result = manager.pull_map_payload(map_name)
        local_name = str(result.get("mapName") or map_name or "active").strip() or "active"
        cached = self.map_cache.save_pulled_map(manager_id, result, activate=True)
        return {
            "ok": True,
            "pulled": result,
            "local": {
                "mapName": str(cached.get("mapName") or local_name),
                "savedAt": str(cached.get("savedAt") or ""),
            },
        }

    def fleet_push_map_payload(
        self,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            local_name = str(payload.get("localMapName") or payload.get("mapName") or "").strip()
            cached = self.map_cache.load_map(manager_id, local_name)
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
        result = manager.push_map_payload(
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
            manager_id,
            local_name,
            result,
            source_map_name=str(result.get("mapName") or local_name),
        )
        return {"ok": True, "pushed": result, "local": cached}

    def fleet_pull_sync_payload(self, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        robot_active = manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(manager_id)
        robot_current = manager.pull_map_payload(robot_active_name)
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
        cached = self.map_cache.save_pulled_map(manager_id, robot_current, activate=True)
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

    def fleet_push_sync_payload(self, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        robot_active = manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(manager_id)
        if not isinstance(local_active, dict):
            raise ValueError("operator has no active local map")
        local_map_name = str(local_active.get("mapName") or "").strip()
        if not local_map_name:
            raise ValueError("operator active local map is invalid")
        local_map_payload = local_active.get("map")
        local_signature = str(local_map_payload.get("signature") or "").strip() if isinstance(local_map_payload, dict) else ""
        has_local_changes = bool(local_active.get("hasLocalChanges"))
        robot_current = manager.pull_map_payload(robot_active_name)
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
            manager_id=manager_id,
        )
        loaded = manager.load_map_payload({"mapName": local_map_name})
        local_signature_after_push = str((pushed.get("pushed") or {}).get("signature") or local_signature).strip()
        synced_local = self.map_cache.mark_synced(
            manager_id,
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


__all__ = ["FleetMapSyncMixin"]
