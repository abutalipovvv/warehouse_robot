from __future__ import annotations

import random
from pathlib import Path
from time import time
from typing import Any

FLEET_MANAGER_ID = "__fleet_manager__"
FLEET_MANAGER_SIM_ID = "__fleet_manager_sim__"
FLEET_MANAGER_IDS = {FLEET_MANAGER_ID, FLEET_MANAGER_SIM_ID}

from .fleet_benchmark_commands import FleetBenchmarkCommandService
from .fleet_benchmark_topology import BenchmarkTopologyService
from .fleet_context import (
    DEFAULT_FLEET_MAP_DIR,
    DEFAULT_FLEET_SIM_MAP_DIR,
    FLEET_MAP_DATA_ROOT,
    FLEET_MAPS_OUT_ROOT,
    FLEET_ROOT,
    PROJECT_ROOT,
    FleetContextService,
)
from .fleet_dynamic_benchmark import DynamicBenchmarkRuntime
from .fleet_manual_control import FleetManualControlService
from .fleet_map_service import FleetMapService
from .fleet_snapshot_service import FleetSnapshotService


__all__ = (
    "DEFAULT_FLEET_MAP_DIR",
    "DEFAULT_FLEET_SIM_MAP_DIR",
    "FLEET_MANAGER_ID",
    "FLEET_MANAGER_IDS",
    "FLEET_MANAGER_SIM_ID",
    "FLEET_MAP_DATA_ROOT",
    "FLEET_MAPS_OUT_ROOT",
    "FLEET_ROOT",
    "OperatorFleetManager",
    "PROJECT_ROOT",
)


class _LazyService:
    """Create a stateless facade component for lightweight test doubles."""

    def __init__(self, factory: Any) -> None:
        self.factory = factory
        self.name = ""

    def __set_name__(self, _owner: type, name: str) -> None:
        self.name = name

    def __get__(self, instance: Any, _owner: type) -> Any:
        if instance is None:
            return self
        service = self.factory(instance)
        instance.__dict__[self.name] = service
        return service


class OperatorFleetManager:
    _context_service = _LazyService(FleetContextService)
    _map_service = _LazyService(FleetMapService)
    _manual_control = _LazyService(FleetManualControlService)
    _snapshot_service = _LazyService(FleetSnapshotService)
    _benchmark_commands = _LazyService(
        FleetBenchmarkCommandService
    )
    _benchmark_topology = _LazyService(
        BenchmarkTopologyService
    )
    _dynamic_benchmark_runtime = _LazyService(
        DynamicBenchmarkRuntime
    )

    def __init__(
        self,
        map_dir: Path,
        params_path: Path,
        remote_adapter: Any | None = None,
        *,
        manager_id: str = FLEET_MANAGER_ID,
        display_name: str = "Fleet Manager",
        mode: str = "robots",
    ) -> None:
        self._runtime_command_executor = None
        self.params_path = Path(params_path).expanduser().resolve()
        self.remote_adapter = remote_adapter
        mode = str(mode or "").strip().lower()
        if mode not in {"robots", "simulation"}:
            raise ValueError("mode must be robots or simulation")
        self.manager_id = manager_id
        self.display_name = display_name
        self.mode = mode
        self._context_service = FleetContextService(self)
        self.map_dir = self.resolve_map_dir(map_dir)
        self.maps_root = self.map_dir.parent
        self._scene3d_cache: dict[str, Any] | None = None
        self._map_service = FleetMapService(self)
        self._manual_control = FleetManualControlService(self)
        self._snapshot_service = FleetSnapshotService(self)
        self._benchmark_commands = FleetBenchmarkCommandService(self)
        self._benchmark_topology = BenchmarkTopologyService(self)
        self._dynamic_benchmark_runtime = DynamicBenchmarkRuntime(
            self,
            benchmark_sim_robots=(
                self._benchmark_commands._benchmark_sim_robots
            ),
        )
        self._load_context(self.map_dir)

    def set_runtime_command_executor(self, executor: Any | None) -> None:
        """Retain the owner boundary across map/config manager replacement."""

        if executor is not None and not callable(executor):
            raise TypeError("runtime command executor must be callable")
        self._runtime_command_executor = executor
        attach = getattr(
            getattr(self, "manager", None),
            "set_runtime_command_executor",
            None,
        )
        if callable(attach):
            attach(executor)

    def close(self) -> None:
        return self._context_service.close()

    def sidebar_payload(self, include_runtime: bool = True) -> dict[str, Any]:
        return self._snapshot_service.sidebar_payload(
            include_runtime,
        )

    def mode_payload(self) -> dict[str, Any]:
        return self._context_service.mode_payload()

    def set_mode_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._context_service.set_mode_payload(payload)

    def params_payload(self) -> dict[str, Any]:
        return self._context_service.params_payload()

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._context_service.save_params_payload(payload)

    def map_payload(self) -> dict[str, Any]:
        return self._map_service.map_payload()

    def scene3d_payload(self) -> dict[str, Any]:
        return self._map_service.scene3d_payload()

    def maps_active_payload(self) -> dict[str, Any]:
        return self._map_service.maps_active_payload()

    def maps_list_payload(self) -> dict[str, Any]:
        return self._map_service.maps_list_payload()

    def pull_map_payload(self, map_name: str = "") -> dict[str, Any]:
        return self._map_service.pull_map_payload(map_name)

    def push_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._map_service.push_map_payload(payload)

    def load_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._map_service.load_map_payload(payload)

    def save_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._map_service.save_map_payload(payload)

    def state_payload(
        self,
        include_trajectories: bool = True,
        *,
        advance_runtime: bool = False,
    ) -> dict[str, Any]:
        return self._snapshot_service.state_payload(
            include_trajectories,
            advance_runtime=advance_runtime,
        )

    def runtime_step(self) -> None:
        return self._snapshot_service.runtime_step()

    def plan_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        result = self.manager.plan(payload)
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self._result_with_context(result)

    def benchmark_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._benchmark_commands.benchmark_payload(
            payload,
        )

    def orders_payload(self) -> dict[str, Any]:
        self._sync_manager_mode()
        self._pump_dynamic_benchmark()
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

    def tick_payload(
        self,
        payload: dict[str, Any] | None = None,
        *,
        advance_runtime: bool = True,
        route_revisions: dict[str, int] | None = None,
        include_runtime_details: bool = True,
    ) -> dict[str, Any]:
        return self._snapshot_service.tick_payload(
            payload,
            advance_runtime=advance_runtime,
            route_revisions=route_revisions,
            include_runtime_details=include_runtime_details,
        )

    def world_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_world(payload))

    def check_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self.manager.check_path(payload)

    def manual_step_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._manual_control.manual_step_payload(payload)

    def manual_stop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._manual_control.manual_stop_payload(payload)

    def note_external_control_takeover(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
    ) -> bool:
        return self._manual_control.note_external_control_takeover(
            endpoint,
            owner_id=owner_id,
            owner_name=owner_name,
        )

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        payload = dict(payload)
        payload["mode"] = "remote" if self.mode == "robots" else "simulated"
        return self._result_with_context(self.manager.add_robot(payload))

    def remove_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.remove_robot(payload))

    def update_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_robot(payload))

    def stop_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        include_state = bool(payload.get("includeState", True))
        return self._result_with_context(
            self.manager.stop_robot(payload, include_state=include_state)
        )

    def reset_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.reset_robot(payload))

    def _clear_simulation_runtime(self) -> None:
        return self._benchmark_commands._clear_simulation_runtime()

    def _benchmark_sim_robots(self) -> list[Any]:
        return self._benchmark_commands._benchmark_sim_robots()

    def _ensure_benchmark_robots_payload(self, *, count: int, seed: int) -> dict[str, Any]:
        return self._benchmark_commands._ensure_benchmark_robots_payload(
            count=count,
            seed=seed,
        )

    def _start_dynamic_benchmark_payload(
        self,
        *,
        count: int,
        seed: int,
        horizon_sec: float,
        order_interval_sec: float,
        queue_depth: int,
        speed: float,
        acceleration: float,
        rotate: bool,
        turn_speed: float,
        generation_mode: str = "continuous",
    ) -> dict[str, Any]:
        return self._dynamic_benchmark_runtime._start_dynamic_benchmark_payload(
            count=count,
            seed=seed,
            horizon_sec=horizon_sec,
            order_interval_sec=order_interval_sec,
            queue_depth=queue_depth,
            speed=speed,
            acceleration=acceleration,
            rotate=rotate,
            turn_speed=turn_speed,
            generation_mode=generation_mode,
        )

    def _safe_dynamic_rolling_horizon(
        self,
        requested: float,
        *,
        robot_count: int | None = None,
    ) -> float:
        """Return the operator's committed planning window.

        Corridor scheduling lookahead and MAPF commitment are independent
        horizons. ``_rolling_planning_goal`` already extends a boundary that
        lands inside a no-wait passage to its next external safe LM, while the
        low-level budget separately includes the longest corridor. The old
        sum turned a requested 10 seconds into 66.7 seconds (120 at 4x).
        """
        return self._dynamic_benchmark_runtime._safe_dynamic_rolling_horizon(
            requested,
            robot_count=robot_count,
        )

    def _stop_dynamic_benchmark_payload(self) -> dict[str, Any]:
        return self._dynamic_benchmark_runtime._stop_dynamic_benchmark_payload()

    def _pump_dynamic_benchmark(self, now: float | None = None) -> int:
        return self._dynamic_benchmark_runtime._pump_dynamic_benchmark(
            now,
        )

    def _finish_terminal_package_waves(self, now: float) -> int:
        return self._dynamic_benchmark_runtime._finish_terminal_package_waves(
            now,
        )

    def _top_up_package_orders(self, now: float) -> int:
        return self._dynamic_benchmark_runtime._top_up_package_orders(
            now,
        )

    def _generate_package_order_wave(self, now: float) -> int:
        """Backward-compatible entry point for package coverage generation."""
        return self._dynamic_benchmark_runtime._generate_package_order_wave(
            now,
        )

    def _generate_package_orders_for_wave(
        self,
        robots: list[Any],
        wave_index: int,
        now: float,
    ) -> int:
        return self._dynamic_benchmark_runtime._generate_package_orders_for_wave(
            robots,
            wave_index,
            now,
        )

    def _next_dynamic_order_payload(self, robot: Any, now: float) -> dict[str, Any] | None:
        return self._dynamic_benchmark_runtime._next_dynamic_order_payload(
            robot,
            now,
        )

    def _dynamic_order_payload(
        self,
        robot: Any,
        target_lm: str,
        now: float,
        *,
        priority: int,
        external_prefix: str,
    ) -> dict[str, Any]:
        return self._dynamic_benchmark_runtime._dynamic_order_payload(
            robot,
            target_lm,
            now,
            priority=priority,
            external_prefix=external_prefix,
        )

    def _record_generated_dynamic_order(
        self,
        order_payload: dict[str, Any],
        now: float,
    ) -> None:
        return self._dynamic_benchmark_runtime._record_generated_dynamic_order(
            order_payload,
            now,
        )

    def _dynamic_goal_hop_window(self) -> tuple[int, int]:
        return self._benchmark_topology._dynamic_goal_hop_window()

    def _far_dynamic_goal(self, origin: str, candidates: list[str]) -> str:
        return self._benchmark_topology._far_dynamic_goal(
            origin,
            candidates,
        )

    def _package_wave_assignments(
        self,
        robots: list[Any],
        wave_index: int,
    ) -> list[tuple[Any, str]]:
        return self._benchmark_topology._package_wave_assignments(
            robots,
            wave_index,
        )

    def _package_goal_is_clear_of_occupied_lms(
        self,
        candidate: str,
        occupied_lms: set[str],
    ) -> bool:
        """Keep a package destination clear of the fleet's start footprints.

        ``robot_footprints_conflict`` checks the configured footprint in one
        orientation.  Package destinations also execute a terminal rotation,
        so use the circumscribed broad-phase diameter as the minimum centre
        distance as well.  This guarantees that a robot parked at the target
        cannot touch or overlap a neighbour that has not departed yet.
        """
        return self._benchmark_topology._package_goal_is_clear_of_occupied_lms(
            candidate,
            occupied_lms,
        )

    def _benchmark_peripheral_lms(self, robot_count: int) -> list[str]:
        return self._benchmark_topology._benchmark_peripheral_lms(
            robot_count,
        )

    def _dynamic_order_origin(self, robot_name: str) -> str:
        return self._dynamic_benchmark_runtime._dynamic_order_origin(
            robot_name,
        )

    def _dynamic_order_depth(self, robot_name: str) -> int:
        return self._dynamic_benchmark_runtime._dynamic_order_depth(
            robot_name,
        )

    def _dynamic_generation_batch_size(self) -> int:
        # Bound synchronous MAPF work per web tick. Orders still arrive for
        # every robot, but the status stream never blocks on a fleet-wide burst.
        return self._dynamic_benchmark_runtime._dynamic_generation_batch_size()

    def _prune_dynamic_order_history(self) -> None:
        return self._dynamic_benchmark_runtime._prune_dynamic_order_history()

    def _dynamic_benchmark_payload(self) -> dict[str, Any]:
        return self._dynamic_benchmark_runtime._dynamic_benchmark_payload()

    def _runtime_now(self) -> float:
        if self.mode == "simulation" and hasattr(self, "manager"):
            return self._dynamic_benchmark_runtime._runtime_now()
        return time()

    def _reset_dynamic_benchmark(self) -> None:
        return self._dynamic_benchmark_runtime._reset_dynamic_benchmark()

    def _benchmark_spawn_lms(self, count: int, seed: int) -> list[str]:
        return self._benchmark_topology._benchmark_spawn_lms(
            count,
            seed,
        )

    def _benchmark_spawn_lm_is_safe(self, name: str) -> bool:
        return self._benchmark_topology._benchmark_spawn_lm_is_safe(
            name,
        )

    def _benchmark_corridor_region(self, name: str) -> str:
        return self._benchmark_topology._benchmark_corridor_region(
            name,
        )

    def _benchmark_goal_lm_is_safe(self, name: str) -> bool:
        """Keep benchmark parking destinations out of traffic bottlenecks."""
        return self._benchmark_topology._benchmark_goal_lm_is_safe(
            name,
        )

    def _benchmark_wait_lm_is_safe(self, name: str) -> bool:
        return self._benchmark_topology._benchmark_wait_lm_is_safe(
            name,
        )

    def _corridor_safe_benchmark_lms(
        self,
        names: list[str],
        rng: random.Random,
    ) -> list[str]:
        """Prefer holding points and place at most one robot inside a corridor."""
        return self._benchmark_topology._corridor_safe_benchmark_lms(
            names,
            rng,
        )

    def _next_benchmark_robot_index(self) -> int:
        return self._benchmark_topology._next_benchmark_robot_index()

    def _apply_fast_benchmark_params(self, count: int = 0) -> dict[str, Any] | None:
        return self._benchmark_commands._apply_fast_benchmark_params(
            count,
        )

    def _restore_fleet_params(self, previous: dict[str, Any]) -> None:
        return self._benchmark_commands._restore_fleet_params(
            previous,
        )

    def _benchmark_requests(
        self,
        *,
        count: int,
        seed: int,
        stress: bool = False,
        stress_profile: int = 0,
    ) -> list[dict[str, Any]]:
        return self._benchmark_commands._benchmark_requests(
            count=count,
            seed=seed,
            stress=stress,
            stress_profile=stress_profile,
        )

    def _benchmark_requests_for_existing(
        self,
        *,
        count: int,
        seed: int,
        stress: bool = False,
        stress_profile: int = 0,
    ) -> list[dict[str, Any]]:
        return self._benchmark_commands._benchmark_requests_for_existing(
            count=count,
            seed=seed,
            stress=stress,
            stress_profile=stress_profile,
        )

    def _traffic_goal_window(
        self,
        stress: bool,
        profile: int,
        count: int,
    ) -> dict[str, int]:
        return self._benchmark_commands._traffic_goal_window(
            stress,
            profile,
            count,
        )

    def _traffic_goal_from_candidates(
        self,
        candidates: list[str],
        profile: int,
        count: int,
    ) -> str:
        return self._benchmark_commands._traffic_goal_from_candidates(
            candidates,
            profile,
            count,
        )

    def _benchmark_plan_stats(
        self,
        plans: list[Any],
        time_step_sec: float,
    ) -> dict[str, Any]:
        return self._benchmark_commands._benchmark_plan_stats(
            plans,
            time_step_sec,
        )

    def _benchmark_min_separation(self) -> float:
        return self._benchmark_topology._benchmark_min_separation()

    def _forward_benchmark_goals(
        self,
        start_lm: str,
        used_goals: set[str],
        excluded_goals: set[str],
        rng: random.Random,
        *,
        min_hops: int = 3,
        max_hops: int = 15,
    ) -> list[str]:
        return self._benchmark_topology._forward_benchmark_goals(
            start_lm,
            used_goals,
            excluded_goals,
            rng,
            min_hops=min_hops,
            max_hops=max_hops,
        )

    def _lm_is_separated_from(self, candidate: str, selected: set[str] | list[str]) -> bool:
        return self._benchmark_topology._lm_is_separated_from(
            candidate,
            selected,
        )

    def _spatially_separated_lms(self, candidates: list[str], count: int) -> list[str]:
        return self._benchmark_topology._spatially_separated_lms(
            candidates,
            count,
        )

    def _largest_benchmark_component(self) -> list[str]:
        return self._benchmark_topology._largest_benchmark_component()

    def resolve_map_dir(self, map_dir: Path) -> Path:
        return self._context_service.resolve_map_dir(map_dir)

    def _resolve_map_dir_by_name(self, map_name: str) -> Path:
        return self._context_service._resolve_map_dir_by_name(
            map_name,
        )

    def _load_context(self, map_dir: Path) -> None:
        return self._context_service._load_context(map_dir)

    def _sync_manager_mode(self) -> None:
        return self._context_service._sync_manager_mode()

    def _active_robot_modes(self) -> set[str]:
        return self._context_service._active_robot_modes()

    def _static_scene3d_payload(self) -> dict[str, Any]:
        return self._map_service._static_scene3d_payload()

    def _wall_rectangles_from_pgm(self, *, wall_height: float) -> list[dict[str, Any]]:
        return self._map_service._wall_rectangles_from_pgm(
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
        return self._map_service._build_wall_rectangles(
            width,
            height,
            pixels,
            occupied_thresh=occupied_thresh,
            negate=negate,
            stride=stride,
            wall_height=wall_height,
        )

    def _find_ros_map_yaml(self) -> Path:
        return self._map_service._find_ros_map_yaml()

    def _state_with_context(self, state: Any) -> dict[str, Any]:
        return self._snapshot_service._state_with_context(state)

    def _result_with_context(self, result: dict[str, Any]) -> dict[str, Any]:
        return self._snapshot_service._result_with_context(
            result,
        )
