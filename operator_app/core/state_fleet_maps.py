"""Synchronize fleet manager maps with the local workspace."""

from __future__ import annotations

from typing import Any

from .fleet_manager import FLEET_MANAGER_ID


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
        robot_active = manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        robot_active_signature = str(robot_active.get("signature") or "").strip()
        active_name = self.map_cache.active_map_name(manager_id)
        active_payload = self.map_cache.load_active_map(manager_id)
        bootstrap_error = ""

        # A Fleet Manager session must open on the map that is actually active
        # in its runtime.  Keep an edited local draft selected, but replace a
        # missing or clean stale selection with the canonical runtime bundle.
        # The previous cached map is not deleted and can still be opened later.
        local_name = (
            str(active_payload.get("mapName") or active_name).strip()
            if isinstance(active_payload, dict)
            else ""
        )
        has_cached_changes = (
            bool(active_payload.get("hasLocalChanges"))
            if isinstance(active_payload, dict)
            else False
        )
        should_activate_runtime_map = bool(
            robot_active_name
            and robot_active_name != local_name
            and not has_cached_changes
        )
        if should_activate_runtime_map:
            try:
                runtime_bundle = manager.pull_map_payload(robot_active_name)
                bundle_signature = str(runtime_bundle.get("signature") or "").strip()
                if robot_active_signature and bundle_signature != robot_active_signature:
                    raise ValueError(
                        "Fleet Manager runtime map signature does not match its stored bundle"
                    )
                self.map_cache.save_pulled_map(manager_id, runtime_bundle, activate=True)
                active_name = self.map_cache.active_map_name(manager_id)
                active_payload = self.map_cache.load_active_map(manager_id)
            except Exception as exc:
                bootstrap_error = str(exc)

        stored_signature = ""
        remote_verified = False
        remote_error = bootstrap_error
        if isinstance(active_payload, dict):
            local_name = str(active_payload.get("mapName") or active_name).strip()
            local_signature = str(active_payload.get("signature") or "").strip()
            try:
                stored = manager.pull_map_payload(local_name)
                stored_signature = str(stored.get("signature") or "").strip()
                remote_verified = bool(stored_signature)
                active_payload["robotSignature"] = stored_signature
                active_payload["robotMapName"] = local_name if remote_verified else ""
                active_payload["hasLocalChanges"] = bool(
                    not remote_verified
                    or not local_signature
                    or local_signature != stored_signature
                )
            except Exception as exc:
                remote_error = str(exc)
        local_signature = str(active_payload.get("signature") or "").strip() if isinstance(active_payload, dict) else ""
        has_local_changes = bool(active_payload.get("hasLocalChanges")) if isinstance(active_payload, dict) else False
        local_name = str(active_payload.get("mapName") or active_name).strip() if isinstance(active_payload, dict) else ""
        activation_required = bool(
            isinstance(active_payload, dict)
            and remote_verified
            and not has_local_changes
            and (
                robot_active_name != local_name
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
            "signature": str(active_payload.get("signature") or "") if isinstance(active_payload, dict) else "",
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

    def fleet_local_map_payload(self, map_name: str, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        payload = self.map_cache.load_map(manager_id, map_name)
        return {"ok": True, **payload}

    def fleet_load_map_payload(
        self,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        requested_name = map_name.removesuffix(".smap")
        previous = manager.maps_active_payload()
        loaded = manager.load_map_payload(payload)
        try:
            active = manager.maps_active_payload()
            loaded_name = str(active.get("mapName") or loaded.get("mapName") or "").strip()
            if loaded_name != requested_name:
                raise ValueError(
                    f"Fleet Manager activated {loaded_name or '-'} instead of {requested_name}"
                )
            bundle = manager.pull_map_payload(loaded_name)
            bundle_signature = str(bundle.get("signature") or "").strip()
            active_signature = str(active.get("signature") or loaded.get("signature") or "").strip()
            if not bundle_signature or active_signature != bundle_signature:
                raise ValueError(
                    "Fleet Manager active map verification failed after Load: "
                    f"runtime={active_signature or '-'} stored={bundle_signature or '-'}"
                )
            self.map_cache.save_pulled_map(manager_id, bundle, activate=True)
            local_payload = self.map_cache.load_active_map(manager_id)
            if not isinstance(local_payload, dict):
                raise ValueError("operator failed to activate the verified Fleet Manager map")
            loaded.update(
                {
                    "signature": bundle_signature,
                    "verified": True,
                    "local": {
                        "activeMapName": str(local_payload.get("mapName") or loaded_name),
                        "map": local_payload.get("map"),
                        "sourceMapName": str(local_payload.get("sourceMapName") or loaded_name),
                        "signature": str(local_payload.get("signature") or ""),
                        "robotSignature": bundle_signature,
                        "robotMapName": loaded_name,
                        "hasLocalChanges": False,
                        "activationRequired": False,
                        "syncState": "synchronized",
                    },
                }
            )
            return loaded
        except Exception as exc:
            previous_name = str(previous.get("mapName") or "").strip()
            rollback_error = ""
            if previous_name and previous_name != requested_name:
                try:
                    manager.load_map_payload({"mapName": previous_name})
                except Exception as rollback_exc:
                    rollback_error = f"; Fleet Manager rollback failed: {rollback_exc}"
            raise ValueError(f"Load verification failed: {exc}{rollback_error}") from exc

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
        read_back = manager.pull_map_payload(local_name)
        expected_signature = str(result.get("signature") or "").strip()
        stored_signature = str(read_back.get("signature") or "").strip()
        if not expected_signature or stored_signature != expected_signature:
            raise ValueError(
                "Fleet Manager map verification failed after Push: "
                f"expected {expected_signature or '-'}, got {stored_signature or '-'}"
            )
        cached = self.map_cache.save_pulled_map(manager_id, read_back, activate=True)
        return {"ok": True, "pushed": result, "verified": read_back, "local": cached}

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
        robot_runtime_signature = str(robot_active.get("signature") or "").strip()
        load_required = robot_runtime_signature != robot_signature
        local_active_name = str(local_active.get("mapName") or "") if isinstance(local_active, dict) else ""
        if robot_active_name and robot_active_name == local_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "loadRequired": load_required,
                "message": (
                    f"Operator already has stored Fleet Manager map {robot_active_name}; Load is required."
                    if load_required
                    else f"Operator already has active Fleet Manager map {robot_active_name}."
                ),
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
            "loadRequired": load_required,
            "message": (
                f"Pulled stored Fleet Manager map {robot_active_name}; Load is required to activate it."
                if load_required
                else f"Pulled active Fleet Manager map {robot_active_name}."
            ),
            "robotActiveMapName": robot_active_name,
            "localActiveMapName": str(pulled.get("local", {}).get("mapName") or robot_active_name),
            **pulled,
        }

    def fleet_push_sync_payload(self, manager_id: str = FLEET_MANAGER_ID) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)
        robot_active = manager.maps_active_payload()
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        robot_active_signature = str(robot_active.get("signature") or "").strip()
        local_active = self.map_cache.load_active_map(manager_id)
        if not isinstance(local_active, dict):
            raise ValueError("operator has no active local map")
        local_map_name = str(local_active.get("mapName") or "").strip()
        if not local_map_name:
            raise ValueError("operator active local map is invalid")
        local_map_payload = local_active.get("map")
        local_signature = str(local_map_payload.get("signature") or "").strip() if isinstance(local_map_payload, dict) else ""
        stored_signature = ""
        try:
            stored = manager.pull_map_payload(local_map_name)
            stored_signature = str(stored.get("signature") or "").strip()
        except Exception:
            stored = None
        if stored_signature == local_signature:
            synced_local = self.map_cache.mark_synced(
                manager_id,
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
                    f"Fleet Manager already stores {local_map_name}; use Load to activate it."
                    if load_required
                    else f"Fleet Manager already stores and uses {local_map_name}."
                ),
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_map_name,
                "local": synced_local,
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
        after = manager.maps_active_payload()
        after_active_name = str(after.get("mapName") or robot_active_name).strip()
        after_active_signature = str(after.get("signature") or robot_active_signature).strip()
        load_required = bool(
            after_active_name != local_map_name
            or after_active_signature != local_signature
        )
        return {
            "ok": True,
            "changed": True,
            "loadRequired": load_required,
            "message": (
                f"Uploaded and verified {local_map_name}. Use Load to activate it on Fleet Manager."
                if load_required
                else f"Uploaded and verified active Fleet Manager map {local_map_name}."
            ),
            "robotActiveMapName": after_active_name,
            "localActiveMapName": local_map_name,
            "pushed": pushed.get("pushed"),
            "verified": pushed.get("verified"),
            "local": pushed.get("local"),
        }
