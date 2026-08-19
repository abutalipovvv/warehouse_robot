"""Editable-map and static-scene operations for the operator fleet."""

from __future__ import annotations

import re
import shutil
import tempfile
from pathlib import Path
from typing import Any

from fleet_manager.core.mapping.maps.map_exchange import (
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    editable_map_signature,
    list_editable_maps,
)
from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.maps.models import LoadedMapData
from fleet_manager.core.mapping.maps.map_writer import (
    save_editable_map,
)
from fleet_manager.core.mapping.navigation.params import load_route_params


class FleetMapService:
    """Read, save, activate and render an operator map."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner

    def map_payload(self) -> dict[str, Any]:
        owner = self.owner
        params = load_route_params(
            owner.params_path,
            create=True,
        )
        return build_editable_map_payload(
            owner.map_dir,
            params=params,
        )

    def scene3d_payload(self) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        static_scene = owner._static_scene3d_payload()
        state = owner.manager.snapshot(
            include_trajectories=True
        )
        robots = (
            state.get("robots", [])
            if isinstance(state, dict)
            else []
        )
        return {
            **static_scene,
            "robots": robots if isinstance(robots, list) else [],
            "mode": owner.mode,
            "managerId": owner.manager_id,
            "managerName": owner.display_name,
        }

    def maps_active_payload(self) -> dict[str, Any]:
        owner = self.owner
        return {
            "ok": True,
            "mapName": owner.map_dir.stem.replace(".smap", ""),
            "mapDir": str(owner.map_dir),
            "signature": str(owner._active_map_signature or ""),
        }

    def maps_list_payload(self) -> dict[str, Any]:
        owner = self.owner
        payload = list_editable_maps(
            owner.maps_root,
            active_map_dir=owner.map_dir,
        )
        params = load_route_params(owner.params_path, create=True)
        for item in payload.get("maps", []):
            if not isinstance(item, dict):
                continue
            target = Path(str(item.get("mapDir") or ""))
            item["signature"] = str(
                build_editable_map_payload(target, params=params).get("signature") or ""
            )
        payload["activeSignature"] = str(owner._active_map_signature or "")
        return payload

    def pull_map_payload(
        self,
        map_name: str = "",
    ) -> dict[str, Any]:
        owner = self.owner
        target = owner._resolve_map_dir_by_name(map_name)
        params = load_route_params(
            owner.params_path,
            create=True,
        )
        return build_editable_map_bundle_payload(
            target,
            params=params,
        )

    def push_map_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        target_name = str(
            payload.get("mapName") or ""
        ).strip()
        source_name = str(
            payload.get("sourceMapName") or target_name
        ).strip()
        source_dir = owner._resolve_map_dir_by_name(source_name)
        output_name = str(
            payload.get("outputName") or ""
        ).strip()
        if (
            not output_name
            and target_name
            and source_name
            and target_name != source_name
        ):
            output_name = target_name
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        target = self._push_target_dir(source_dir, output_name)
        overwrite_output = bool(payload.get("overwriteOutput", False))
        if target != source_dir and target.exists() and not overwrite_output:
            raise ValueError(f"map already exists: {target.name}")
        loaded_map = self._save_map_atomically(
            source_dir=source_dir,
            target=target,
            editable_map=editable_map,
        )
        params = load_route_params(
            owner.params_path,
            create=True,
        )
        return {
            **build_editable_map_payload(
                loaded_map.map_dir,
                params=params,
            ),
            "savedAs": bool(output_name),
        }

    def load_map_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        map_name = str(
            payload.get("mapName")
            or payload.get("folder")
            or ""
        ).strip()
        if not map_name:
            raise ValueError("mapName is required")
        safe_name = Path(map_name).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (owner.maps_root / safe_name).resolve()
        maps_root = owner.maps_root.resolve()
        if (
            maps_root not in target.parents
            and target != maps_root
        ):
            raise ValueError("map must stay inside maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        owner._load_context(target)
        return {
            "ok": True,
            "mapName": owner.map_dir.stem.replace(
                ".smap",
                "",
            ),
            "mapDir": str(owner.map_dir),
            "signature": str(owner._active_map_signature or ""),
            "mode": owner.mode,
        }

    def _push_target_dir(self, source_dir: Path, output_name: str) -> Path:
        if not output_name:
            return source_dir.resolve()
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", output_name).strip("._")
        if not safe_name:
            raise ValueError("save-as name is empty")
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.owner.maps_root / safe_name).resolve()
        if self.owner.maps_root.resolve() not in target.parents:
            raise ValueError("map must stay inside fleet maps_out")
        return target

    def _save_map_atomically(
        self,
        *,
        source_dir: Path,
        target: Path,
        editable_map: dict[str, Any],
    ) -> LoadedMapData:
        root = self.owner.maps_root.resolve()
        staging = Path(
            tempfile.mkdtemp(prefix=f".{target.name}.incoming-", dir=str(root))
        ).resolve()
        backup = (root / f".{target.name}.backup").resolve()
        swapped = False
        backed_up = False
        shutil.rmtree(staging)
        try:
            shutil.copytree(source_dir, staging)
            save_editable_map(staging, editable_map)
            staged = build_editable_map_payload(staging)
            staged["mapName"] = target.stem.replace(".smap", "")
            staged_signature = editable_map_signature(staged)
            if backup.exists():
                shutil.rmtree(backup)
            if target.exists():
                target.rename(backup)
                backed_up = True
            staging.rename(target)
            swapped = True
            verified = build_editable_map_payload(target)
            if str(verified.get("signature") or "") != staged_signature:
                raise ValueError("fleet map verification failed after atomic replace")
            if backup.exists():
                shutil.rmtree(backup)
            return WarehouseMapLoader(target).load()
        except Exception:
            if swapped and target.exists():
                shutil.rmtree(target)
            if backed_up and backup.exists():
                backup.rename(target)
            raise
        finally:
            if staging.exists():
                shutil.rmtree(staging)

    def save_map_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            editable_map = payload
        output_name = str(
            payload.get("outputName") or ""
        ).strip()
        loaded_map = save_editable_map(
            owner.map_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(
                payload.get("overwriteOutput", False)
            ),
        )
        owner._load_context(loaded_map.map_dir)
        return {
            "ok": True,
            "mapName": owner.map_dir.stem.replace(
                ".smap",
                "",
            ),
            "mapDir": str(owner.map_dir),
            "map": owner.map_payload(),
            "maps": owner.maps_list_payload(),
        }

    def _static_scene3d_payload(self) -> dict[str, Any]:
        owner = self.owner
        if isinstance(owner._scene3d_cache, dict):
            return owner._scene3d_cache
        payload = owner._scene_builder.build()
        owner._scene3d_cache = payload
        return payload

    def _wall_rectangles_from_pgm(
        self,
        *,
        wall_height: float,
    ) -> list[dict[str, Any]]:
        return self.owner._scene_builder.wall_rectangles(
            wall_height=wall_height,
        )

    def _build_wall_rectangles(
        self,
        width: int,
        height: int,
        pixels: bytes,
        *,
        occupied_thresh: float,
        negate: int,
        stride: int,
        wall_height: float,
    ) -> list[dict[str, Any]]:
        return self.owner._scene_builder.merge_wall_rectangles(
            width,
            height,
            pixels,
            occupied_threshold=occupied_thresh,
            negate=negate,
            stride=stride,
            wall_height=wall_height,
        )

    def _find_ros_map_yaml(self) -> Path:
        return self.owner._scene_builder.find_ros_map_yaml()
