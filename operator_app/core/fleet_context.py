"""Lifecycle, configuration and map-directory context for the operator fleet."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.navigation.params import (
    load_route_params,
    save_route_params,
)
from fleet_manager.runtime.grpc.manager import FleetManagerROS
from fleet_manager.runtime.simulation.manager import FleetManagerSim

from .map_scene import MapSceneBuilder


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FLEET_ROOT = PROJECT_ROOT / "fleet_manager"
FLEET_MAP_DATA_ROOT = FLEET_ROOT / "map_data"
FLEET_MAPS_OUT_ROOT = FLEET_MAP_DATA_ROOT / "maps_out"
DEFAULT_FLEET_MAP_DIR = (
    FLEET_MAPS_OUT_ROOT / "22.05.26_smap.smap"
)
DEFAULT_FLEET_SIM_MAP_DIR = (
    FLEET_MAPS_OUT_ROOT / "benchmark_open_kiva.smap"
)


class FleetContextService:
    """Own manager replacement, mode synchronization and configuration."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner

    def close(self) -> None:
        close = getattr(
            getattr(self.owner, "manager", None),
            "close",
            None,
        )
        if callable(close):
            close()

    def mode_payload(self) -> dict[str, Any]:
        owner = self.owner
        return {
            "ok": True,
            "id": owner.manager_id,
            "mode": owner.mode,
            "mapName": owner.map_dir.stem.replace(".smap", ""),
        }

    def set_mode_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        mode = str(payload.get("mode") or "").strip().lower()
        if mode not in {"simulation", "robots"}:
            raise ValueError("mode must be simulation or robots")
        if mode != owner.mode:
            raise ValueError(
                f"{owner.display_name} mode is fixed to {owner.mode}"
            )
        return owner.mode_payload()

    def params_payload(self) -> dict[str, Any]:
        owner = self.owner
        return {
            "ok": True,
            "path": str(owner.params_path),
            "params": load_route_params(
                owner.params_path,
                create=True,
            ),
        }

    def save_params_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        params_payload = payload.get("params")
        if not isinstance(params_payload, dict):
            params_payload = payload
        save_route_params(params_payload, owner.params_path)
        owner._load_context(owner.map_dir)
        return owner.params_payload()

    def resolve_map_dir(self, map_dir: Path) -> Path:
        candidate = Path(map_dir).expanduser()
        safe_name = Path(candidate).name
        maps_root = FLEET_MAPS_OUT_ROOT.resolve()
        if candidate.is_absolute():
            resolved = candidate.resolve()
            if resolved.exists() and (
                resolved == maps_root
                or maps_root in resolved.parents
            ):
                return resolved
            candidate = Path(safe_name)

        candidates = [
            FLEET_MAPS_OUT_ROOT / candidate,
            FLEET_MAP_DATA_ROOT / candidate,
            FLEET_ROOT / candidate,
            FLEET_MAPS_OUT_ROOT / safe_name,
        ]
        if not safe_name.endswith(".smap"):
            candidates.append(
                FLEET_MAPS_OUT_ROOT / f"{safe_name}.smap"
            )
        for item in candidates:
            resolved = item.resolve()
            if resolved.exists() and (
                resolved == maps_root
                or maps_root in resolved.parents
            ):
                return resolved
        return DEFAULT_FLEET_MAP_DIR.resolve()

    def _resolve_map_dir_by_name(
        self,
        map_name: str,
    ) -> Path:
        owner = self.owner
        if not map_name:
            return owner.map_dir
        safe_name = Path(str(map_name)).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (owner.maps_root / safe_name).resolve()
        root = owner.maps_root.resolve()
        if root not in target.parents and target != root:
            raise ValueError(
                "map must stay inside fleet maps_out"
            )
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        return target

    def _load_context(self, map_dir: Path) -> None:
        owner = self.owner
        loaded_map = WarehouseMapLoader(map_dir).load()
        params = load_route_params(
            owner.params_path,
            create=True,
        )
        for section, values in owner.params_overrides.items():
            if not isinstance(values, dict):
                continue
            current = params.setdefault(section, {})
            if isinstance(current, dict):
                current.update(values)
        owner.loaded_map = loaded_map
        owner.map_dir = loaded_map.map_dir.resolve()
        owner.maps_root = owner.map_dir.parent
        owner._scene_builder = MapSceneBuilder(loaded_map)
        owner._scene3d_cache = None
        manager_class = (
            FleetManagerROS
            if owner.mode == "robots"
            else FleetManagerSim
        )
        previous_manager = getattr(owner, "manager", None)
        manager = manager_class(
            loaded_map.landmarks,
            loaded_map.edges,
            params=params,
            map_dir=loaded_map.map_dir,
            map_metadata=loaded_map.map_metadata,
            remote_adapter=owner.remote_adapter,
        )
        owner.manager = manager
        executor = getattr(owner, "_runtime_command_executor", None)
        if executor is not None:
            manager.set_runtime_command_executor(executor)
        previous_close = getattr(
            previous_manager,
            "close",
            None,
        )
        if callable(previous_close):
            previous_close()
        owner._reset_dynamic_benchmark()
        owner._sync_manager_mode()

    def _sync_manager_mode(self) -> None:
        owner = self.owner
        if not hasattr(owner, "manager"):
            return
        owner.manager.set_active_robot_modes(
            owner._active_robot_modes()
        )

    def _active_robot_modes(self) -> set[str]:
        if self.owner.mode == "robots":
            return {"remote"}
        return {"simulated"}
