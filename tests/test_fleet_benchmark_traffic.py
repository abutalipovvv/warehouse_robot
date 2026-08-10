from __future__ import annotations

from time import monotonic, sleep

import fleet_manager.manager.manager as runtime_module
import operator_app.core.fleet_manager as service_module

from fleet_manager.manager.planning import PlanningSolverService
from operator_app.core.fleet_manager import (
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)
from operator_app.core.fleet_context import DEFAULT_FLEET_MAP_DIR


def test_plan_action_starts_continuous_random_orders_for_every_robot(monkeypatch) -> None:
    service = OperatorFleetManager(
        DEFAULT_FLEET_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 20, "seed": 42, "reset": False}
    )

    planner_calls = 0
    original_plan = service.manager._planning_solver_service._planner_call

    def counted_plan(payload):
        nonlocal planner_calls
        planner_calls += 1
        return original_plan(payload)

    service.manager._planning_solver_service = PlanningSolverService(
        counted_plan,
        service.manager._planner_lock,
    )

    result = service.benchmark_payload(
        {
            "action": "plan",
            "count": 20,
            "seed": 42,
            "reset": False,
            "fast": True,
            "horizonSec": 10,
            "orderIntervalSec": 1,
            "queueDepth": 1,
        }
    )

    benchmark = result["benchmark"]
    assert result["ok"]
    assert benchmark["planned"] == 20
    assert benchmark["scenario"] == "continuous_random_orders"
    assert benchmark["active"]
    assert benchmark["horizonSec"] == 10
    _pump_until(
        service,
        lambda: any(robot.status == "MOVING" for robot in service.manager.robots.values()),
    )
    # One runtime tick may finish the first dispatch and immediately admit the
    # next bounded batch. A soft-block fallback can add one solver call, but
    # startup must not fan out into one call per robot.
    assert 1 <= planner_calls <= 3
    moving = [
        robot for robot in service.manager.robots.values()
        if robot.status == "MOVING"
    ]
    assert moving
    assert all(
        robot.plan_nodes and len(set(robot.plan_nodes)) > 1
        for robot in moving
    )

    # The first order for every robot is queued atomically; MAPF dispatch runs
    # in the background so the Start response stays fast.
    future = float(service._dynamic_benchmark["startedAt"]) + 10.0
    for _ in range(10):
        service._pump_dynamic_benchmark(now=future)

    dynamic_orders = [
        order for order in service.manager.orders.values()
        if order.order_id.startswith("dynamic-")
    ]
    assert len(dynamic_orders) == 20
    assert {order.vehicle for order in dynamic_orders} == {
        f"bench_{index:03d}" for index in range(1, 21)
    }
    assert all(order.target_lm in service.loaded_map.landmarks for order in dynamic_orders)
    assert all(order.target_lm != order.start_lm for order in dynamic_orders if order.start_lm)
    assert service._dynamic_benchmark_payload()["averageOrderDistanceM"] >= 6.0


def test_simulated_order_replans_after_rolling_horizon_without_completing() -> None:
    service = OperatorFleetManager(
        DEFAULT_FLEET_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 1, "seed": 42, "reset": False}
    )
    service.manager.params["fleet"]["rolling_target_buffer_sec"] = 3.0
    service.benchmark_payload({
        "action": "plan",
        "count": 1,
        "seed": 42,
        "reset": False,
        "horizonSec": 3,
        "orderIntervalSec": 30,
        "queueDepth": 1,
    })

    _pump_until(
        service,
        lambda: bool(service.manager.robots["bench_001"].active_order_id),
    )

    robot = service.manager.robots["bench_001"]
    order_id = robot.active_order_id
    first_chunk = robot.route_chunk_goal_lm
    final_goal = robot.route_final_lm
    assert order_id
    assert first_chunk != final_goal

    robot.route_clock = float(robot.trajectory[-1]["t"])
    service.manager._advance_runtime()
    _pump_until(
        service,
        lambda: (
            service.manager.robots["bench_001"].active_order_id == order_id
            and service.manager.robots["bench_001"].route_chunk_goal_lm
            != first_chunk
            and service.manager.orders[order_id].status == "EXECUTING"
        ),
    )

    robot = service.manager.robots["bench_001"]
    order = service.manager.orders[order_id]
    assert robot.current_lm == first_chunk
    assert robot.active_order_id == order_id
    assert robot.route_final_lm == final_goal
    assert robot.route_chunk_goal_lm != first_chunk
    assert order.status == "EXECUTING"


def test_dynamic_runtime_keeps_robots_collision_free_and_wait_graph_acyclic(
    monkeypatch,
) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    monkeypatch.setattr(service_module, "time", lambda: clock[0])
    service = OperatorFleetManager(
        DEFAULT_FLEET_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 20, "seed": 42, "reset": False}
    )
    service.benchmark_payload({
        "action": "plan",
        "count": 20,
        "seed": 42,
        "reset": False,
        "horizonSec": 5,
        "orderIntervalSec": 1,
        "queueDepth": 1,
    })

    _pump_until(
        service,
        lambda: any(robot.route_revision > 0 for robot in service.manager.robots.values()),
    )

    for _ in range(80):
        clock[0] += 0.25
        state = service.tick_payload({})
        robots = list(service.manager.robots.values())
        for index, robot in enumerate(robots):
            if robot.pose is None:
                continue
            assert "map occupancy under footprint" not in robot.last_reason
            for other in robots[index + 1:]:
                if other.pose is not None:
                    assert not service.manager.collision.footprints_overlap(
                        robot.pose,
                        other.pose,
                    ), f"physical collision: {robot.name}/{other.name}"
        _assert_no_wait_cycle(state["robots"])
        sleep(0.002)

    benchmark = service._dynamic_benchmark_payload()
    assert benchmark["ordersGenerated"] >= 20
    assert any(robot.route_revision > 1 for robot in service.manager.robots.values())


def _pump_until(service: OperatorFleetManager, predicate, timeout: float = 6.0) -> None:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        service.tick_payload({})
        if predicate():
            return
        sleep(0.01)
    assert predicate(), "background MAPF dispatch did not finish in time"


def _assert_no_wait_cycle(robots: list[dict]) -> None:
    dependencies: dict[str, str] = {}
    for robot in robots:
        if robot.get("status") != "WAITING":
            continue
        reason = str(robot.get("reason") or "")
        for prefix in ("yield to ", "occupied by ", "keep clearance from "):
            if reason.startswith(prefix):
                dependencies[str(robot["name"])] = reason[len(prefix):]
                break
    for start in dependencies:
        visited: set[str] = set()
        current = start
        while current in dependencies:
            assert current not in visited, f"wait-for cycle from {start}: {dependencies}"
            visited.add(current)
            current = dependencies[current]
