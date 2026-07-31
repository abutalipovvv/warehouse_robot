"""Own runtimes, manager selection and workspace lifecycle."""

from __future__ import annotations

from pathlib import Path
from sys import modules
from threading import Lock, RLock
from typing import Any

from fleet_manager.runtime.loop import (
    RuntimeLoop,
    RuntimeLoopFailure,
)

from .fleet_manager import (
    DEFAULT_FLEET_SIM_MAP_DIR,
    FLEET_MANAGER_ID,
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)
from .map_cache import MapCache, default_maps_cache_root
from .models import KnownRobot
from .registry import RobotRegistry
from .grpc.client import GrpcRobotAdapter
from .workspace import OperatorWorkspace


def _state_dependency(name: str, default: Any) -> Any:
    """Respect compatibility monkeypatches made through ``core.state``."""
    facade = modules.get("operator_app.core.state")
    return getattr(facade, name, default) if facade is not None else default


class RuntimeOwnershipMixin:
    """Own runtimes, manager selection and workspace lifecycle."""

    def __init__(self, registry_path: Path, probe_timeout: float, fleet_params_path: Path, fleet_map_dir: Path) -> None:
        grpc_adapter_type = _state_dependency(
            "GrpcRobotAdapter",
            GrpcRobotAdapter,
        )
        fleet_manager_type = _state_dependency(
            "OperatorFleetManager",
            OperatorFleetManager,
        )
        self.registry = RobotRegistry(registry_path)
        self.workspace = OperatorWorkspace()
        self.legacy_map_cache_root = default_maps_cache_root().expanduser().resolve()
        self.map_cache = MapCache(robot_dir_resolver=self._maps_dir_for_robot_id)
        self.grpc_adapter = grpc_adapter_type(
            timeout=max(1.5, float(probe_timeout))
        )
        self.map_timeout = max(10.0, float(probe_timeout) * 10.0)
        self.fleet_params_path = Path(fleet_params_path).expanduser().resolve()
        self.fleet_manager = fleet_manager_type(
            fleet_map_dir,
            self.fleet_params_path,
            remote_adapter=self.grpc_adapter,
            manager_id=FLEET_MANAGER_ID,
            display_name="Fleet Manager",
            mode="robots",
        )
        sim_map_dir = DEFAULT_FLEET_SIM_MAP_DIR if DEFAULT_FLEET_SIM_MAP_DIR.exists() else fleet_map_dir
        self.fleet_manager_sim = fleet_manager_type(
            sim_map_dir,
            self.fleet_params_path,
            remote_adapter=None,
            manager_id=FLEET_MANAGER_SIM_ID,
            display_name="Fleet Manager Sim",
            mode="simulation",
        )
        self._lock = Lock()
        self._fleet_manager_lock = RLock()
        self._fleet_manager_sim_lock = RLock()
        # Historical callers use this attribute for the real manager.
        self._fleet_lock = self._fleet_manager_lock
        self._closed = False
        self._fleet_runtime_loops = (
            RuntimeLoop(
                lambda: self._fleet_runtime_step(self.fleet_manager),
                interval_seconds=0.10,
                name="fleet-robots-runtime",
                on_error=lambda failure: self._report_fleet_runtime_failure(
                    self.fleet_manager,
                    failure,
                ),
            ),
            RuntimeLoop(
                lambda: self._fleet_runtime_step(self.fleet_manager_sim),
                interval_seconds=self._simulation_runtime_interval(),
                interval_provider=self._simulation_runtime_interval,
                name="fleet-simulation-runtime",
                on_error=lambda failure: self._report_fleet_runtime_failure(
                    self.fleet_manager_sim,
                    failure,
                ),
            ),
        )
        for runtime_loop in self._fleet_runtime_loops:
            runtime_loop.start()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True

        # Signal both owners before joining either one. This lets real and
        # simulation steps finish in parallel while their managers are alive.
        for runtime_loop in self._fleet_runtime_loops:
            runtime_loop.close(timeout=0.0)
        for runtime_loop in self._fleet_runtime_loops:
            runtime_loop.close()

        self.fleet_manager.close()
        self.fleet_manager_sim.close()

    def _fleet_runtime_step(self, manager: OperatorFleetManager) -> None:
        with self._fleet_lock_for_manager(manager):
            manager.runtime_step()

    def _report_fleet_runtime_failure(
        self,
        manager: OperatorFleetManager,
        failure: RuntimeLoopFailure,
    ) -> None:
        with self._fleet_lock_for_manager(manager):
            manager.manager._event(
                "error",
                (
                    f"{manager.mode} runtime tick failed "
                    f"({failure.operation}): "
                    f"{failure.exception_type}: {failure.message}"
                ),
            )

    def _simulation_runtime_interval(self) -> float:
        with self._fleet_lock_for_id(FLEET_MANAGER_SIM_ID):
            try:
                fleet = self.fleet_manager_sim.manager.params.get("fleet", {})
                value = float(
                    fleet.get("simulation_tick_interval_sec", 0.10)
                    or 0.10
                )
            except (AttributeError, TypeError, ValueError):
                value = 0.10
        return max(0.05, min(0.20, value))

    def _maps_dir_for_robot_id(self, robot_id: str) -> Path:
        if robot_id == FLEET_MANAGER_ID:
            directory = self.workspace.maps_dir("fleet_manager")
            directory.mkdir(parents=True, exist_ok=True)
            return directory
        if robot_id == FLEET_MANAGER_SIM_ID:
            directory = self.workspace.maps_dir("fleet_manager_sim")
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

    def _fleet_manager_for_id(self, manager_id: str = FLEET_MANAGER_ID) -> OperatorFleetManager:
        if manager_id == FLEET_MANAGER_SIM_ID:
            return self.fleet_manager_sim
        if manager_id == FLEET_MANAGER_ID:
            return self.fleet_manager
        raise ValueError(f"unknown fleet manager: {manager_id}")

    def _fleet_lock_for_id(
        self,
        manager_id: str = FLEET_MANAGER_ID,
    ) -> Any:
        """Return the lock owned by one manager.

        ``__new__``-based compatibility tests and integrations may only
        provide the historical ``_fleet_lock``. In that case both selectors
        deliberately fall back to it.
        """
        if manager_id == FLEET_MANAGER_SIM_ID:
            simulation_lock = getattr(
                self,
                "_fleet_manager_sim_lock",
                None,
            )
            if simulation_lock is not None:
                return simulation_lock
            legacy_lock = getattr(self, "_fleet_lock", None)
            if legacy_lock is not None:
                return legacy_lock
            simulation_lock = RLock()
            self._fleet_manager_sim_lock = simulation_lock
            return simulation_lock

        real_lock = getattr(self, "_fleet_manager_lock", None)
        if real_lock is not None:
            return real_lock
        legacy_lock = getattr(self, "_fleet_lock", None)
        if legacy_lock is not None:
            return legacy_lock
        real_lock = RLock()
        self._fleet_manager_lock = real_lock
        self._fleet_lock = real_lock
        return real_lock

    def _fleet_lock_for_manager(
        self,
        manager: OperatorFleetManager,
    ) -> Any:
        manager_id = (
            FLEET_MANAGER_SIM_ID
            if manager is getattr(self, "fleet_manager_sim", None)
            else FLEET_MANAGER_ID
        )
        return self._fleet_lock_for_id(manager_id)

    def _fleet_sidebar_payloads(self, include_runtime: bool = True) -> list[dict[str, Any]]:
        payloads: list[dict[str, Any]] = []
        managers = (
            (FLEET_MANAGER_ID, self.fleet_manager),
            (FLEET_MANAGER_SIM_ID, self.fleet_manager_sim),
        )
        for manager_id, manager in managers:
            lock = self._fleet_lock_for_id(manager_id)
            acquired = lock.acquire(blocking=include_runtime)
            if not acquired:
                payloads.append(
                    manager.sidebar_payload(include_runtime=False)
                )
                continue
            try:
                payloads.append(
                    manager.sidebar_payload(
                        include_runtime=include_runtime
                    )
                )
            finally:
                lock.release()
        return payloads

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


__all__ = ["RuntimeOwnershipMixin"]
