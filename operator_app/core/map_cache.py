from __future__ import annotations

import json
import os
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from fleet_manager.core.route_core.map_exchange import (
    build_editable_map_payload,
    list_editable_maps,
    restore_editable_map_bundle,
)
from fleet_manager.core.route_core.map_writer import save_editable_map


def default_maps_cache_root() -> Path:
    override = os.environ.get("WAREHOUSE_OPERATOR_MAP_CACHE", "").strip()
    if override:
        return Path(override).expanduser()
    return Path(__file__).resolve().parents[1] / "map_out"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class MapCache:
    def __init__(self, root: Path | None = None, *, robot_dir_resolver: Callable[[str], Path] | None = None) -> None:
        self.root = (root or default_maps_cache_root()).expanduser().resolve()
        self.robot_dir_resolver = robot_dir_resolver

    def list_maps(self, robot_id: str) -> list[dict[str, Any]]:
        robot_dir = self._robot_dir(robot_id)
        if not robot_dir.exists():
            return []
        active_dir = self._active_map_dir(robot_id)
        payload = list_editable_maps(robot_dir, active_map_dir=active_dir)
        items: list[dict[str, Any]] = []
        for item in payload.get("maps", []):
            if not isinstance(item, dict):
                continue
            map_name = str(item.get("name") or "").strip()
            if not map_name:
                continue
            info = self._map_info(robot_id, map_name)
            info["active"] = bool(item.get("active"))
            items.append(info)
        return items

    def active_map_name(self, robot_id: str) -> str:
        payload = self._state_payload(robot_id)
        return str(payload.get("activeMapName") or "").strip()

    def set_active_map(self, robot_id: str, map_name: str) -> None:
        map_dir = self._map_dir(robot_id, map_name)
        if not map_dir.is_dir():
            raise ValueError(f"local map not found: {map_name}")
        state_path = self._state_file(robot_id)
        state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = self._state_payload(robot_id)
        payload["activeMapName"] = str(map_name or "").strip()
        state_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def load_active_map(self, robot_id: str) -> dict[str, Any] | None:
        active_name = self.active_map_name(robot_id)
        if not active_name:
            return None
        try:
            return self.load_map(robot_id, active_name)
        except ValueError:
            return None

    def load_map(self, robot_id: str, map_name: str) -> dict[str, Any]:
        map_dir = self._map_dir(robot_id, map_name)
        if not map_dir.is_dir():
            raise ValueError(f"local map not found: {map_name}")
        editable = build_editable_map_payload(map_dir)
        meta = self._map_meta_payload(map_dir)
        local_signature = str(editable.get("signature") or "").strip()
        robot_signature = str(meta.get("robotSignature") or "").strip()
        robot_map_name = str(meta.get("robotMapName") or meta.get("sourceMapName") or editable.get("mapName") or map_name).strip()
        has_local_changes = bool(
            (local_signature and robot_signature and local_signature != robot_signature)
            or (robot_map_name and robot_map_name != str(editable.get("mapName") or map_name).strip())
        )
        return {
            "mapName": str(editable.get("mapName") or map_name),
            "sourceMapName": str(meta.get("sourceMapName") or editable.get("mapName") or map_name),
            "savedAt": str(meta.get("savedAt") or ""),
            "mapDir": str(map_dir),
            "map": editable,
            "signature": local_signature,
            "robotSignature": robot_signature,
            "robotMapName": robot_map_name,
            "hasLocalChanges": has_local_changes,
        }

    def save_pulled_map(self, robot_id: str, payload: dict[str, Any], *, activate: bool = True) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or "").strip()
        if not map_name:
            raise ValueError("pulled mapName is required")
        map_dir = self._map_dir(robot_id, map_name)
        if map_dir.exists():
            shutil.rmtree(map_dir)
        restore_editable_map_bundle(map_dir, payload)
        self._write_map_meta(
            map_dir,
            {
                "savedAt": utc_now(),
                "sourceMapName": map_name,
                "robotSignature": str(payload.get("signature") or "").strip(),
                "robotMapName": map_name,
                "lastSyncAt": utc_now(),
            },
        )
        if activate:
            self.set_active_map(robot_id, map_name)
        return self._map_info(robot_id, map_name)

    def save_map(
        self,
        robot_id: str,
        map_name: str,
        editable_map: dict[str, Any],
        *,
        source_map_name: str = "",
        activate: bool = True,
    ) -> dict[str, Any]:
        safe_name = str(map_name or "").strip()
        if not safe_name:
            raise ValueError("map name is required")
        target_dir = self._map_dir(robot_id, safe_name)
        base_meta: dict[str, Any] = {}
        if target_dir.exists():
            base_meta = self._map_meta_payload(target_dir)
            loaded_map = save_editable_map(target_dir, editable_map)
        else:
            base_dir = self._resolve_base_dir(robot_id, safe_name, source_map_name)
            base_meta = self._map_meta_payload(base_dir)
            loaded_map = save_editable_map(base_dir, editable_map, output_name=target_dir.name)
        self._write_map_meta(
            loaded_map.map_dir,
            {
                "savedAt": utc_now(),
                "sourceMapName": str(source_map_name or safe_name),
                "robotSignature": str(base_meta.get("robotSignature") or "").strip(),
                "robotMapName": str(base_meta.get("robotMapName") or source_map_name or safe_name).strip(),
                "lastSyncAt": str(base_meta.get("lastSyncAt") or ""),
            },
        )
        if activate:
            self.set_active_map(robot_id, loaded_map.map_dir.stem.replace(".smap", ""))
        return self._map_info(robot_id, loaded_map.map_dir.stem.replace(".smap", ""))

    def mark_synced(
        self,
        robot_id: str,
        map_name: str,
        *,
        robot_signature: str,
        robot_map_name: str,
        activate: bool | None = None,
    ) -> dict[str, Any]:
        map_dir = self._map_dir(robot_id, map_name)
        if not map_dir.is_dir():
            raise ValueError(f"local map not found: {map_name}")
        meta = self._map_meta_payload(map_dir)
        meta.update(
            {
                "savedAt": str(meta.get("savedAt") or utc_now()),
                "sourceMapName": str(meta.get("sourceMapName") or robot_map_name or map_name),
                "robotSignature": str(robot_signature or "").strip(),
                "robotMapName": str(robot_map_name or map_name).strip(),
                "lastSyncAt": utc_now(),
            }
        )
        self._write_map_meta(map_dir, meta)
        if activate is True:
            self.set_active_map(robot_id, map_name)
        return self._map_info(robot_id, map_name)

    def _resolve_base_dir(self, robot_id: str, map_name: str, source_map_name: str) -> Path:
        candidates = [
            self._map_dir(robot_id, map_name),
            self._map_dir(robot_id, source_map_name),
            self._active_map_dir(robot_id),
        ]
        for candidate in candidates:
            if candidate is not None and candidate.is_dir():
                return candidate
        raise ValueError("no local base map exists; pull a map from robot first")

    def _map_info(self, robot_id: str, map_name: str) -> dict[str, Any]:
        map_dir = self._map_dir(robot_id, map_name)
        meta = self._map_meta_payload(map_dir)
        local_signature = ""
        has_local_changes = False
        robot_signature = str(meta.get("robotSignature") or "").strip()
        robot_map_name = str(meta.get("robotMapName") or meta.get("sourceMapName") or map_name).strip()
        if map_dir.is_dir():
            try:
                payload = build_editable_map_payload(map_dir)
                local_signature = str(payload.get("signature") or "").strip()
                has_local_changes = bool(
                    (local_signature and robot_signature and local_signature != robot_signature)
                    or (robot_map_name and robot_map_name != map_name)
                )
            except Exception:
                local_signature = ""
        return {
            "mapName": map_name,
            "robotId": robot_id,
            "savedAt": str(meta.get("savedAt") or ""),
            "sourceMapName": str(meta.get("sourceMapName") or map_name),
            "robotMapName": robot_map_name,
            "signature": local_signature,
            "robotSignature": robot_signature,
            "hasLocalChanges": has_local_changes,
            "path": str(map_dir),
            "active": self.active_map_name(robot_id) == map_name,
        }

    def _active_map_dir(self, robot_id: str) -> Path | None:
        active_name = self.active_map_name(robot_id)
        if not active_name:
            return None
        map_dir = self._map_dir(robot_id, active_name)
        return map_dir if map_dir.is_dir() else None

    def _robot_dir(self, robot_id: str) -> Path:
        if self.robot_dir_resolver is not None:
            return self.robot_dir_resolver(robot_id).expanduser().resolve()
        return self.root / self._safe_name(robot_id)

    def _map_dir(self, robot_id: str, map_name: str) -> Path:
        safe_name = self._safe_name(map_name or "unnamed_map")
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        return self._robot_dir(robot_id) / safe_name

    def _state_file(self, robot_id: str) -> Path:
        return self._robot_dir(robot_id) / ".state.json"

    def _state_payload(self, robot_id: str) -> dict[str, Any]:
        path = self._state_file(robot_id)
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return payload if isinstance(payload, dict) else {}

    def _map_meta_file(self, map_dir: Path) -> Path:
        return map_dir / ".operator_meta.json"

    def _map_meta_payload(self, map_dir: Path) -> dict[str, Any]:
        path = self._map_meta_file(map_dir)
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return payload if isinstance(payload, dict) else {}

    def _write_map_meta(self, map_dir: Path, payload: dict[str, Any]) -> None:
        map_dir.mkdir(parents=True, exist_ok=True)
        self._map_meta_file(map_dir).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _safe_name(self, value: str) -> str:
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip()).strip("._") or "unnamed"
