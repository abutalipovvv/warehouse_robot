from __future__ import annotations

from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.simulation.manager import FleetManagerSim
from operator_app.core.fleet_manager import (
    DEFAULT_FLEET_MAP_DIR,
    DEFAULT_FLEET_SIM_MAP_DIR,
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)


def _operator_service(*, robots: int = 0) -> OperatorFleetManager:
    service = OperatorFleetManager(
        DEFAULT_FLEET_SIM_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    if robots:
        service.benchmark_payload(
            {
                "action": "add",
                "count": robots,
                "seed": 42,
                "reset": False,
            }
        )
    return service


def _small_manager() -> FleetManagerSim:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name="A",
            to_name="B",
            length=2.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(WorldPoint(0.0, 0.0), WorldPoint(2.0, 0.0)),
            properties={"direction": 0},
        )
    ]
    return FleetManagerSim(landmarks, edges)


def test_terminal_history_is_bounded_without_losing_lifetime_counts() -> None:
    service = _operator_service()
    service._dynamic_benchmark["sessionId"] = 123
    service._dynamic_benchmark["ordersGenerated"] = 150
    for index in range(150):
        status = "COMPLETED" if index < 100 else "FAILED"
        order_id = f"dynamic-123-history-{index:03d}"
        service.manager.orders[order_id] = FleetOrder(
            order_id=order_id,
            target_lm="unused",
            status=status,
            created_at=float(index),
            updated_at=float(index + 10),
        )

    service._prune_dynamic_order_history()

    retained = [
        order
        for order in service.manager.orders.values()
        if order.order_id.startswith("dynamic-123-history-")
    ]
    assert len(retained) == 120
    assert len(service._dynamic_benchmark["countedTerminalOrders"]) == 120
    assert service._dynamic_benchmark["ordersCompleted"] == 100
    assert service._dynamic_benchmark["ordersTerminated"] == 50

    service._prune_dynamic_order_history()

    assert service._dynamic_benchmark["ordersCompleted"] == 100
    assert service._dynamic_benchmark["ordersTerminated"] == 50


def test_new_benchmark_session_does_not_recount_previous_terminal_orders() -> None:
    service = _operator_service(robots=1)
    robot = service.manager.robots["bench_001"]
    old_order = FleetOrder(
        order_id="dynamic-previous-session",
        target_lm=robot.current_lm,
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="COMPLETED",
        created_at=1.0,
        updated_at=2.0,
    )
    service.manager.orders[old_order.order_id] = old_order

    result = service.benchmark_payload(
        {
            "action": "plan",
            "count": 1,
            "seed": 43,
            "reset": False,
            "horizonSec": 10,
            "queueDepth": 1,
        }
    )

    benchmark = result["benchmark"]
    assert benchmark["ordersGenerated"] == 1
    assert benchmark["ordersCompleted"] == 0
    assert benchmark["ordersOutstanding"] == 1
    assert service._dynamic_benchmark["ordersTerminated"] == 0
    assert old_order.order_id not in service.manager.orders


def test_completed_package_wave_metadata_is_pruned_idempotently() -> None:
    service = _operator_service(robots=2)
    robots = service._benchmark_sim_robots()
    order_ids: set[str] = set()
    for index, robot in enumerate(robots):
        order_id = f"dynamic-package-{index}"
        order_ids.add(order_id)
        service.manager.orders[order_id] = FleetOrder(
            order_id=order_id,
            target_lm=robot.current_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="COMPLETED",
            created_at=10.0,
            updated_at=20.0,
        )
    service._dynamic_benchmark.update(
        {
            "generationMode": "package_waves",
            "packageWaveOrderIds": {1: set(order_ids)},
            "packageWaveRobots": {1: {robot.name for robot in robots}},
            "packageWaveStartedAt": {1: 10.0},
            "packageCompletedWaves": set(),
            "wavesCompleted": 0,
        }
    )

    assert service._finish_terminal_package_waves(20.0) == 1
    assert service._dynamic_benchmark["wavesCompleted"] == 1
    assert service._dynamic_benchmark["packageWaveOrderIds"] == {}
    assert service._dynamic_benchmark["packageWaveRobots"] == {}
    assert service._dynamic_benchmark["packageWaveStartedAt"] == {}
    assert service._dynamic_benchmark["packageCompletedWaves"] == set()

    assert service._finish_terminal_package_waves(30.0) == 0
    assert service._dynamic_benchmark["wavesCompleted"] == 1


def test_legacy_string_completed_wave_marker_is_not_counted_twice() -> None:
    service = _operator_service(robots=1)
    robot = service.manager.robots["bench_001"]
    order = FleetOrder(
        order_id="dynamic-legacy-wave",
        target_lm=robot.current_lm,
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="COMPLETED",
        created_at=1.0,
        updated_at=2.0,
    )
    service.manager.orders[order.order_id] = order
    service._dynamic_benchmark.update(
        {
            "generationMode": "package_waves",
            "packageWaveOrderIds": {"1": {order.order_id}},
            "packageWaveRobots": {"1": {robot.name}},
            "packageWaveStartedAt": {"1": 1.0},
            "packageCompletedWaves": {"1"},
            "wavesCompleted": 1,
        }
    )

    assert service._finish_terminal_package_waves(3.0) == 0
    assert service._dynamic_benchmark["wavesCompleted"] == 1
    assert service._dynamic_benchmark["packageWaveOrderIds"] == {}
    assert service._dynamic_benchmark["packageCompletedWaves"] == set()


def test_planning_reset_requeues_discarded_dispatch_and_clears_ephemeral_state() -> None:
    manager = _small_manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="order-1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="PLANNING",
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    entry = (
        order,
        robot,
        {"name": robot.name, "startLm": "A", "goalLm": "B"},
        "B",
    )
    stale_job = {
        "kind": "dispatch",
        "entries": [entry],
        "done": True,
        "result": {
            "ok": True,
            "plans": [
                {
                    "robot": robot.name,
                    "startLm": "A",
                    "goalLm": "B",
                    "nodes": ["A", "B"],
                    "trajectory": [],
                }
            ],
        },
    }
    manager._dispatch_job = stale_job
    manager._last_async_job_kind = "prefetch"
    manager._rolling_prefetch_retry_at[robot.name] = 100.0
    manager._rolling_prefetch_failures[robot.name] = 3
    manager._rolling_prefetch_eligible_since[robot.name] = 90.0
    manager._rolling_prefetch_last_attempt_at[robot.name] = 95.0
    manager._stationary_order_retry_state[order.order_id] = {
        "blocked_lms": ("A",),
        "signature": (),
        "failure_count": 2,
    }
    manager._active_wait_cycles[(robot.name, "r2")] = 10.0
    manager._controlled_corridor_leases["corridor"] = (robot.name, 20.0)
    manager._traffic_zone_phase["zone"] = "east"

    manager.reset_planning_runtime_state()

    assert stale_job["discard"]
    assert manager._last_async_job_kind == ""
    assert manager._rolling_prefetch_retry_at == {}
    assert manager._rolling_prefetch_failures == {}
    assert manager._rolling_prefetch_eligible_since == {}
    assert manager._rolling_prefetch_last_attempt_at == {}
    assert manager._stationary_order_retry_state == {}
    assert manager._active_wait_cycles == {}
    assert manager._controlled_corridor_leases == {}
    assert manager._traffic_zone_phase == {}
    assert manager._finish_async_simulated_dispatch() == 0
    assert manager._dispatch_job is None
    assert order.status == "QUEUED"
