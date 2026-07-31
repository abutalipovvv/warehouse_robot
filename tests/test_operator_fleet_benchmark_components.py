from __future__ import annotations

import inspect
import random
from types import SimpleNamespace

from fleet_manager.core.models import FleetOrder
from fleet_manager.core.route_core.models import Landmark
from operator_app.core.fleet_benchmark_commands import (
    FleetBenchmarkCommandService,
)
from operator_app.core.fleet_benchmark_topology import (
    BenchmarkTopologyService,
)
from operator_app.core.fleet_dynamic_benchmark import (
    DynamicBenchmarkRuntime,
)
from operator_app.core.fleet_manager import OperatorFleetManager


COMMAND_HOOKS = {
    "benchmark_payload",
    "_clear_simulation_runtime",
    "_benchmark_sim_robots",
    "_ensure_benchmark_robots_payload",
    "_apply_fast_benchmark_params",
    "_restore_fleet_params",
    "_benchmark_requests",
    "_benchmark_requests_for_existing",
    "_traffic_goal_window",
    "_traffic_goal_from_candidates",
    "_benchmark_plan_stats",
}
DYNAMIC_HOOKS = {
    "_start_dynamic_benchmark_payload",
    "_safe_dynamic_rolling_horizon",
    "_stop_dynamic_benchmark_payload",
    "_pump_dynamic_benchmark",
    "_finish_terminal_package_waves",
    "_top_up_package_orders",
    "_generate_package_order_wave",
    "_generate_package_orders_for_wave",
    "_next_dynamic_order_payload",
    "_dynamic_order_payload",
    "_record_generated_dynamic_order",
    "_dynamic_order_origin",
    "_dynamic_order_depth",
    "_dynamic_generation_batch_size",
    "_prune_dynamic_order_history",
    "_dynamic_benchmark_payload",
    "_runtime_now",
    "_reset_dynamic_benchmark",
}
TOPOLOGY_HOOKS = {
    "_dynamic_goal_hop_window",
    "_far_dynamic_goal",
    "_package_wave_assignments",
    "_package_goal_is_clear_of_occupied_lms",
    "_benchmark_peripheral_lms",
    "_benchmark_spawn_lms",
    "_benchmark_spawn_lm_is_safe",
    "_benchmark_corridor_region",
    "_benchmark_goal_lm_is_safe",
    "_benchmark_wait_lm_is_safe",
    "_corridor_safe_benchmark_lms",
    "_next_benchmark_robot_index",
    "_benchmark_min_separation",
    "_forward_benchmark_goals",
    "_lm_is_separated_from",
    "_spatially_separated_lms",
    "_largest_benchmark_component",
}


def test_component_method_signatures_match_all_facade_hooks() -> None:
    groups = (
        (FleetBenchmarkCommandService, COMMAND_HOOKS),
        (DynamicBenchmarkRuntime, DYNAMIC_HOOKS),
        (BenchmarkTopologyService, TOPOLOGY_HOOKS),
    )

    for component, hooks in groups:
        for hook in hooks:
            assert hasattr(OperatorFleetManager, hook)
            assert inspect.signature(
                getattr(OperatorFleetManager, hook)
            ) == inspect.signature(getattr(component, hook))


def test_dynamic_runtime_reset_owns_only_benchmark_state() -> None:
    owner = SimpleNamespace()
    runtime = DynamicBenchmarkRuntime(owner)

    assert runtime._benchmark_sim_robots is None
    runtime._reset_dynamic_benchmark()

    assert owner._dynamic_benchmark["active"] is False
    assert owner._dynamic_benchmark["generationMode"] == "continuous"
    assert owner._dynamic_benchmark["ordersGenerated"] == 0
    assert owner._dynamic_benchmark["packageWaveOrderIds"] == {}
    assert owner._dynamic_rng.random() == (
        random.Random(42).random()
    )


def test_dynamic_runtime_counts_all_active_orders_in_one_queue_scan() -> None:
    robots = [
        SimpleNamespace(name="robot-1"),
        SimpleNamespace(name="robot-2"),
        SimpleNamespace(name="robot-3"),
    ]
    orders = [
        FleetOrder(
            order_id="same-owner",
            target_lm="A",
            vehicle="robot-1",
            assigned_robot="robot-1",
        ),
        FleetOrder(
            order_id="transferred",
            target_lm="B",
            vehicle="robot-1",
            assigned_robot="robot-2",
            status="EXECUTING",
        ),
        FleetOrder(
            order_id="assigned-only",
            target_lm="C",
            assigned_robot="robot-2",
        ),
        FleetOrder(
            order_id="terminal",
            target_lm="D",
            vehicle="robot-3",
            status="COMPLETED",
        ),
    ]

    assert DynamicBenchmarkRuntime._dynamic_order_depths(
        robots,
        orders,
    ) == {
        "robot-1": 2,
        "robot-2": 2,
        "robot-3": 0,
    }


def test_command_service_calculates_plan_statistics_without_runtime() -> None:
    service = FleetBenchmarkCommandService(SimpleNamespace())

    stats = service._benchmark_plan_stats(
        [
            {
                "nodes": ["A", "A", "B", "C"],
                "times": [0, 2, 5, 8],
            },
            {
                "nodes": ["D", "E"],
                "times": [0, 4],
            },
        ],
        0.5,
    )

    assert stats == {
        "plannedWaitingRobots": 1,
        "plannedWaitTicks": 2,
        "plannedWaitSec": 1.0,
        "maxPlannedWaitTicks": 2,
        "averageRouteSteps": 1.5,
        "maxRouteSteps": 2,
    }


def test_topology_service_uses_owner_spacing_hook() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=3.0, y=0.0),
    }

    class Collision:
        @staticmethod
        def robot_footprints_conflict(_first, _second) -> bool:
            return False

    owner = SimpleNamespace(
        loaded_map=SimpleNamespace(landmarks=landmarks),
        manager=SimpleNamespace(collision=Collision()),
        _benchmark_min_separation=lambda: 2.0,
    )
    topology = BenchmarkTopologyService(owner)

    assert not topology._lm_is_separated_from("B", {"A"})
    assert topology._lm_is_separated_from("C", {"A"})
