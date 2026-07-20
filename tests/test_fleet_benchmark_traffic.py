from __future__ import annotations

from collections import Counter
from time import monotonic, sleep

import fleet_manager.core.manager as runtime_module
import operator_app.core.fleet_manager as service_module

from operator_app.core.fleet_manager import (
    DEFAULT_FLEET_MAP_DIR,
    DEFAULT_FLEET_SIM_MAP_DIR,
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)


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
    original_plan = service.manager._plan_valid_requests

    def counted_plan(requests, payload):
        nonlocal planner_calls
        planner_calls += 1
        return original_plan(requests, payload)

    monkeypatch.setattr(service.manager, "_plan_valid_requests", counted_plan)

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
    assert 1 <= planner_calls <= 2
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
    metrics = service._dynamic_benchmark_payload()
    assert metrics["averageOrderDistanceM"] >= 6.0
    assert metrics["robotsWithOrders"] == 20
    assert metrics["robotsWithoutOrders"] == 0
    robot_states = service.manager.snapshot()["robots"]
    assert all(robot["assignedOrderId"] for robot in robot_states)
    assert all(robot["assignedOrderTargetLm"] == robot["targetLm"] for robot in robot_states)
    assert all(robot["orderQueueDepth"] == 1 for robot in robot_states)


def test_time_scale_action_keeps_simulation_and_uses_virtual_order_time(monkeypatch) -> None:
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
        {"action": "add", "count": 1, "seed": 42, "reset": False}
    )

    scaled = service.benchmark_payload({
        "action": "time_scale",
        "timeScale": 4,
        "reset": False,
    })

    assert scaled["simulationTimeScale"] == 4.0
    assert scaled["simulationTimeScaleMax"] == 8.0
    assert list(service.manager.robots) == ["bench_001"]

    service.benchmark_payload({
        "action": "plan",
        "count": 1,
        "seed": 42,
        "reset": False,
        "horizonSec": 5,
        "orderIntervalSec": 10,
        "queueDepth": 1,
    })
    first_generated = int(service._dynamic_benchmark["ordersGenerated"])
    for order in service.manager.orders.values():
        if order.order_id.startswith("dynamic-"):
            order.status = "COMPLETED"
    deadline = float(service._dynamic_benchmark["nextOrderAt"]["bench_001"])
    current = service._runtime_now()
    clock[0] += max(0.0, (deadline - current) / 4.0) + 0.01

    generated = service._pump_dynamic_benchmark()

    assert generated == 1
    assert int(service._dynamic_benchmark["ordersGenerated"]) == first_generated + 1
    assert service._dynamic_benchmark_payload()["timeScale"] == 4.0


def test_continuous_generator_replenishes_every_uncovered_robot_in_one_pump() -> None:
    service = OperatorFleetManager(
        DEFAULT_FLEET_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 6, "seed": 42, "reset": False}
    )
    service.benchmark_payload({
        "action": "plan",
        "count": 6,
        "seed": 42,
        "reset": False,
        "horizonSec": 10,
        "orderIntervalSec": 30,
        "queueDepth": 1,
    })

    first_orders = sorted(
        (
            order
            for order in service.manager.orders.values()
            if order.order_id.startswith("dynamic-")
        ),
        key=lambda order: order.order_id,
    )
    assert len(first_orders) == 6
    uncovered_names = {order.vehicle for order in first_orders[:4]}
    now = service._runtime_now()
    for order in first_orders[:4]:
        order.status = "COMPLETED"
        order.updated_at = now

    assert service._pump_dynamic_benchmark(now=now) == 4
    assert all(
        service._dynamic_order_depth(robot.name) == 1
        for robot in service._benchmark_sim_robots()
    )
    metrics = service._dynamic_benchmark_payload()
    assert metrics["robotsWithOrders"] == 6
    assert metrics["robotsWithoutOrders"] == 0
    replacement_vehicles = {
        order.vehicle
        for order in service.manager.orders.values()
        if order.order_id.startswith("dynamic-")
        and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
        and order.vehicle in uncovered_names
    }
    assert replacement_vehicles == uncovered_names


def test_package_generator_replenishes_a_robot_before_the_wave_finishes() -> None:
    service = OperatorFleetManager(
        DEFAULT_FLEET_SIM_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 8, "seed": 42, "reset": False}
    )
    service.benchmark_payload({
        "action": "package_waves",
        "count": 8,
        "seed": 42,
        "reset": False,
        "horizonSec": 10,
        "queueDepth": 1,
    })

    first_wave_ids = set(service._dynamic_benchmark["waveOrderIds"])
    completed_order = service.manager.orders[min(first_wave_ids)]
    completed_robot = completed_order.vehicle
    now = service._runtime_now()
    completed_order.status = "COMPLETED"
    completed_order.updated_at = now

    assert service._pump_dynamic_benchmark(now=now) == 1
    assert service._dynamic_benchmark["wavesCompleted"] == 0
    assert service._dynamic_benchmark["waveIndex"] == 2
    assert service._dynamic_benchmark["packageWaveRobots"][2] == {
        completed_robot
    }
    assert all(
        service._dynamic_order_depth(robot.name) == 1
        for robot in service._benchmark_sim_robots()
    )
    metrics = service._dynamic_benchmark_payload()
    assert metrics["ordersGenerated"] == 9
    assert metrics["robotsWithOrders"] == 8
    assert metrics["robotsWithoutOrders"] == 0


def test_package_order_waves_use_perimeter_and_keep_metrics_after_stop(monkeypatch) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    monkeypatch.setattr(service_module, "time", lambda: clock[0])
    service = OperatorFleetManager(
        DEFAULT_FLEET_SIM_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 8, "seed": 42, "reset": False}
    )
    assert all(
        not service.manager.collision.blocked_reason(
            robot.pose,
            service.manager.obstacles,
            service.manager.obstacle_areas,
        )
        for robot in service.manager.robots.values()
        if robot.pose is not None
    )

    result = service.benchmark_payload({
        "action": "package_waves",
        "count": 8,
        "seed": 42,
        "reset": False,
        "horizonSec": 10,
        "queueDepth": 1,
    })

    benchmark = result["benchmark"]
    assert benchmark["scenario"] == "package_order_waves"
    assert benchmark["generationMode"] == "package_waves"
    assert benchmark["ordersGenerated"] == 8
    assert benchmark["robotsWithOrders"] == 8
    assert benchmark["robotsWithoutOrders"] == 0
    assert benchmark["waveIndex"] == 1
    assert benchmark["wavesStarted"] == 1
    first_wave_ids = set(service._dynamic_benchmark["waveOrderIds"])
    assert len(first_wave_ids) == 8
    perimeter = set(service._benchmark_peripheral_lms(8))
    first_targets = {
        service.manager.orders[order_id].target_lm
        for order_id in first_wave_ids
    }
    assert len(first_targets) == 8
    assert first_targets <= perimeter

    clock[0] += 20.0
    completed_at = service._runtime_now()
    for order_id in first_wave_ids:
        order = service.manager.orders[order_id]
        order.status = "COMPLETED"
        order.updated_at = completed_at

    assert service._pump_dynamic_benchmark() == 8
    assert service._dynamic_benchmark["waveIndex"] == 2
    assert service._dynamic_benchmark["wavesCompleted"] == 1
    assert service._dynamic_benchmark["ordersGenerated"] == 16

    stopped = service.benchmark_payload({
        "action": "stop",
        "count": 8,
        "reset": False,
    })
    assert not stopped["benchmark"]["active"]
    second_wave_ids = set(service._dynamic_benchmark["waveOrderIds"])
    clock[0] += 30.0
    completed_at = service._runtime_now()
    for order_id in second_wave_ids:
        order = service.manager.orders[order_id]
        order.status = "COMPLETED"
        order.updated_at = completed_at

    assert service._pump_dynamic_benchmark() == 0
    metrics = service._dynamic_benchmark_payload()
    assert not metrics["active"]
    assert metrics["ordersGenerated"] == 16
    assert metrics["ordersCompleted"] == 16
    assert metrics["ordersOutstanding"] == 0
    assert metrics["wavesCompleted"] == 2
    assert metrics["throughputOrdersPerMin"] == 19.2
    assert metrics["elapsedSimSec"] == 50.0


def test_smart_kiva_benchmark_keeps_parking_goals_out_of_controlled_corridors() -> None:
    smart_map = DEFAULT_FLEET_MAP_DIR.parent / "smart_kiva_large_w_mode.smap"
    service = OperatorFleetManager(
        smart_map,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )

    spawn_lms = service._benchmark_spawn_lms(20, 42)
    peripheral_lms = service._benchmark_peripheral_lms(20)
    assert len(spawn_lms) == 20
    assert len(peripheral_lms) >= 20
    assert all(service._benchmark_wait_lm_is_safe(lm_id) for lm_id in spawn_lms)
    assert all(service._benchmark_goal_lm_is_safe(lm_id) for lm_id in peripheral_lms)
    assert all(len(service.manager.planner.graph.get(lm_id, {})) <= 3 for lm_id in peripheral_lms)

    service.benchmark_payload(
        {"action": "add", "count": 20, "seed": 42, "reset": False}
    )
    result = service.benchmark_payload({
        "action": "package_waves",
        "count": 20,
        "seed": 42,
        "reset": False,
        "horizonSec": 10,
        "queueDepth": 1,
    })

    # The requested benchmark horizon controls rolling-route commitment and
    # is raised to this map's topology-safe minimum. It must not also inflate
    # the independently configured reservation window from 10 to 30 seconds:
    # runtime reservation visibility already has its own corridor-safe floor.
    assert result["benchmark"]["horizonRequestedSec"] == 10
    assert result["benchmark"]["horizonSec"] == 30
    assert service.manager.params["fleet"]["rolling_horizon_sec"] == 30
    assert service.manager.params["fleet"]["reservation_horizon_sec"] == 10
    corridor_floor = (
        service.manager.planner.controlled_corridor_max_ticks()
        * service.manager._reservation_time_step()
        + (2.0 * service.manager._reservation_safety_time())
    )
    effective_reservation = service.manager._reservation_horizon()
    assert abs(effective_reservation - corridor_floor) < 0.000001
    assert effective_reservation < result["benchmark"]["horizonSec"]

    assert result["benchmark"]["ordersGenerated"] == 20
    targets = {
        order.target_lm
        for order in service.manager.orders.values()
        if order.order_id.startswith("dynamic-")
    }
    assert len(targets) == 20
    assert all(service._benchmark_goal_lm_is_safe(lm_id) for lm_id in targets)
    assert all(len(service.manager.planner.graph.get(lm_id, {})) <= 3 for lm_id in targets)


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
        lambda: service.manager.robots["bench_001"].active_order_id == order_id,
    )

    robot = service.manager.robots["bench_001"]
    order = service.manager.orders[order_id]
    assert robot.current_lm == first_chunk
    assert robot.active_order_id == order_id
    assert robot.route_final_lm == final_goal
    assert robot.route_chunk_goal_lm != first_chunk
    assert order.status == "EXECUTING"


def test_rolling_prefetch_is_appended_without_resetting_active_motion() -> None:
    service = OperatorFleetManager(
        DEFAULT_FLEET_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 1, "seed": 42, "reset": False}
    )
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
    first_revision = robot.route_revision
    first_chunk = robot.route_chunk_goal_lm
    first_trajectory_length = len(robot.trajectory)
    first_chunk_end = float(robot.trajectory[-1]["t"])
    handoff_yaw = float(robot.trajectory[-1]["yaw"])
    robot.route_clock = max(0.0, first_chunk_end - 0.5)
    clock_before_prefetch = robot.route_clock
    service.manager._advance_runtime()
    _pump_until(
        service,
        lambda: service.manager.robots["bench_001"].route_revision > first_revision,
    )

    robot = service.manager.robots["bench_001"]
    assert robot.active_order_id == order_id
    assert robot.status == "MOVING"
    assert robot.route_clock >= clock_before_prefetch
    assert len(robot.trajectory) > first_trajectory_length
    assert robot.route_chunk_goal_lm != first_chunk
    assert abs(float(robot.trajectory[first_trajectory_length - 1]["yaw"]) - handoff_yaw) < 1e-6
    assert robot.pending_route is None

    # Crossing the old horizon boundary remains part of one continuous clock;
    # it does not switch through a zero-clock/IDLE route revision.
    robot.route_clock = first_chunk_end
    service.manager._advance_runtime()

    robot = service.manager.robots["bench_001"]
    assert robot.active_order_id == order_id
    assert robot.current_lm == first_chunk
    assert robot.status == "MOVING"
    assert robot.trajectory
    assert robot.route_revision > first_revision
    assert robot.pending_route is None
    assert service.manager.orders[order_id].status == "EXECUTING"


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
    assert service.manager.traffic_metrics["runtimeSafetyRollbacks"] == 0


def test_50_robot_runtime_never_exposes_route_less_moving_blockers(
    monkeypatch,
) -> None:
    clock = [2_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    monkeypatch.setattr(service_module, "time", lambda: clock[0])
    service = OperatorFleetManager(
        # The production simulation manager uses the spacious benchmark map.
        # The field map has 0.44 m LM spacing and cannot physically park 50
        # one-metre Ecom robots at once without overlapping their footprints.
        DEFAULT_FLEET_SIM_MAP_DIR,
        DEFAULT_FLEET_MAP_DIR.parents[2] / "config" / "params.yaml",
        manager_id=FLEET_MANAGER_SIM_ID,
        mode="simulation",
    )
    service.benchmark_payload(
        {"action": "add", "count": 50, "seed": 42, "reset": False}
    )
    service.benchmark_payload({
        "action": "plan",
        "count": 50,
        "seed": 42,
        "reset": False,
        "horizonSec": 10,
        "orderIntervalSec": 3,
        "queueDepth": 2,
    })
    _pump_until(
        service,
        lambda: any(robot.route_revision > 0 for robot in service.manager.robots.values()),
    )

    for _ in range(400):
        clock[0] += 0.2
        service.tick_payload({})
        route_less = [
            robot.name
            for robot in service.manager.robots.values()
            if robot.status in {"MOVING", "WAITING", "RETREATING"}
            and not robot.trajectory
        ]
        assert not route_less, f"route-less traffic blockers: {route_less}"
        sleep(0.003)

    assert len(service.manager.robots) == 50
    assert service._dynamic_benchmark_payload()["ordersGenerated"] >= 50
    assert service.manager.traffic_metrics["runtimeSafetyRollbacks"] == 0
    route_node_load: Counter[str] = Counter()
    for order in service.manager.orders.values():
        if order.status in {"COMPLETED", "FAILED", "CANCELED"}:
            continue
        route_node_load.update(set(order.spatial_route_nodes))
    assert max(route_node_load.values(), default=0) <= 8

    central_robots = []
    for robot in service.manager.robots.values():
        lm_name = str(robot.current_lm)
        if len(lm_name) != 5 or not lm_name.startswith("B"):
            continue
        try:
            row = int(lm_name[1:3])
            column = int(lm_name[3:5])
        except ValueError:
            continue
        if 11 <= row <= 21 and 14 <= column <= 24:
            central_robots.append(robot.name)
    assert len(central_robots) <= 20


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
