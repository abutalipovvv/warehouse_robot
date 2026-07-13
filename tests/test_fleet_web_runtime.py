from __future__ import annotations

import math
from threading import Event
from time import perf_counter, sleep, time

from fleet_manager.route_core import GraphEdge, Landmark, WorldPoint
from fleet_manager.web_simulator.manager import (
    FLEET_CONTROL_OWNER_ID,
    FleetOrder,
    FleetRobot,
    WebFleetManager,
)


def test_runtime_replan_is_deferred_while_robot_is_mid_edge() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "edgeId": "A->B"},
            {"x": 2.0, "y": 0.0, "yaw": 0.0, "t": 2.0, "edgeId": "A->B"},
        ],
    )

    assert manager._safe_replan_start_lm(robot) == ""
    assert not manager._maybe_replan_robot(robot, time(), "occupied by parked")
    assert robot.status == "WAITING"
    assert robot.trajectory
    assert robot.pose["x"] == 1.0


def test_manual_plan_does_not_replace_a_robot_route_mid_edge() -> None:
    manager = _manager()
    original_trajectory = [
        {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "t": 0.0,
            "edgeId": "A->B",
            "lm": "A",
        },
        {
            "x": 2.0,
            "y": 0.0,
            "yaw": 0.0,
            "t": 2.0,
            "edgeId": "A->B",
            "lm": "B",
        },
    ]
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
        trajectory=original_trajectory,
        route_clock=1.0,
    )
    manager.robots[robot.name] = robot

    result = manager.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "B",
                    "startPose": dict(robot.pose),
                }
            ]
        }
    )

    assert not result["ok"]
    assert robot.trajectory == original_trajectory
    assert robot.route_clock == 1.0
    assert not any(
        "CURRENT" in str(sample.get("edgeId", ""))
        for sample in robot.trajectory
    )


def test_current_lm_advances_only_after_a_tagged_graph_landmark() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        trajectory=[
            {"t": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 1.0, "edgeId": "A->B"},
            {"t": 2.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_clock=1.9,
    )

    manager._update_current_lm_from_trajectory(robot)
    assert robot.current_lm == "A"

    robot.route_clock = 2.0
    manager._update_current_lm_from_trajectory(robot)
    assert robot.current_lm == "B"


def test_planned_graph_wait_is_exposed_as_waiting_at_lm() -> None:
    manager = _manager()
    trajectory = [
        {"t": 0.0, "edgeId": "A->A", "lm": "A"},
        {"t": 2.0, "edgeId": "A->A", "lm": "A"},
        {"t": 4.0, "edgeId": "A->B", "lm": "B"},
    ]

    assert manager._planned_wait_lm_at_trajectory(trajectory, 1.0) == "A"
    assert manager._planned_wait_lm_at_trajectory(trajectory, 2.5) == ""


def test_deadlock_retreat_lease_overrides_soft_clearance_not_overlap(monkeypatch) -> None:
    manager = _manager()
    now = time()
    retreater = FleetRobot(
        name="r1",
        current_lm="A",
        status="RETREATING",
        active_order_id="o1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        traffic_priority_until=now + 3.0,
    )
    blocker = FleetRobot(
        name="r2",
        current_lm="B",
        status="WAITING",
        active_order_id="o2",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
    )
    monkeypatch.setattr(manager.collision, "footprints_overlap", lambda *_: False)
    monkeypatch.setattr(manager.collision, "robot_footprints_conflict", lambda *_: True)

    reason = manager._robot_conflict_reason(
        retreater,
        blocker,
        {"x": 0.1, "y": 0.0, "yaw": 0.0},
        dict(blocker.pose),
    )

    assert reason == ""


def test_runtime_wait_cycle_grants_one_robot_deterministic_priority() -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            status="WAITING",
            last_reason="yield to r2",
            blocked_since=now - 1.0,
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            status="WAITING",
            last_reason="yield to r1",
            blocked_since=now - 1.0,
        ),
    }

    manager._resolve_runtime_wait_cycles(now)

    assert manager.robots["r1"].status == "MOVING"
    assert manager.robots["r1"].traffic_priority_until > now
    assert manager.robots["r2"].status == "WAITING"
    assert manager.robots["r2"].last_reason == "yield to r1"
    assert manager._has_right_of_way(manager.robots["r1"], manager.robots["r2"])
    assert manager.traffic_metrics == {
        "waitCyclesDetected": 1,
        "waitCyclesResolved": 1,
        "cycleReplans": 0,
        "priorityGrants": 1,
        "runtimeSafetyRollbacks": 0,
    }

    # The same physical encounter can remain visible for several 10 Hz ticks.
    # It is one deadlock episode, not a new cycle and lease on every frame.
    manager.robots["r1"].status = "WAITING"
    manager.robots["r1"].last_reason = "yield to r2"
    manager._resolve_runtime_wait_cycles(now + 0.1)
    assert manager.robots["r1"].status == "WAITING"
    assert manager.robots["r1"].last_reason == "deadlock priority active"
    assert manager.robots["r2"].last_reason == "yield to r1"
    assert manager.traffic_metrics["waitCyclesDetected"] == 1
    assert manager.traffic_metrics["priorityGrants"] == 1


def test_mid_edge_deadlock_retreats_to_lm_and_queues_same_goal_detour() -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            last_reason="yield to r2",
            blocked_since=now - 4.0,
            active_order_id="o1",
            pose={"x": 0.5, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
                {"t": 1.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
            ],
            route_clock=0.25,
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            target_lm="A",
            status="WAITING",
            last_reason="yield to r1",
            blocked_since=now - 4.0,
            active_order_id="o2",
            pose={"x": 1.5, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
            ],
            route_clock=0.25,
        ),
    }
    manager.orders = {
        "o1": FleetOrder(order_id="o1", target_lm="B", vehicle="r1", status="EXECUTING"),
        "o2": FleetOrder(order_id="o2", target_lm="A", vehicle="r2", status="EXECUTING"),
    }

    manager._resolve_runtime_wait_cycles(now)

    retreater = manager.robots["r2"]
    assert retreater.status == "RETREATING"
    assert retreater.traffic_priority_until > now
    assert retreater.retreat_target_lm == "B"
    assert set(retreater.retreat_blocked_edges) == {("A", "B"), ("B", "A")}
    assert manager.robots["r1"].status == "WAITING"
    assert manager.robots["r1"].last_reason == "yield to r2"

    # Complete the graph-safe reverse traversal at the previous landmark.
    retreater.route_clock = 0.0
    retreater.last_tick_at = now
    manager._advance_deadlock_retreat(retreater, now + 0.1)

    order = manager.orders["o2"]
    assert retreater.current_lm == "B"
    assert retreater.status == "IDLE"
    assert retreater.active_order_id == ""
    assert order.status == "QUEUED"
    assert order.target_lm == "A"
    assert set(order.traffic_detour_edges) == {("A", "B"), ("B", "A")}
    assert order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["cycleReplans"] == 1


def test_deadlock_detour_keeps_goal_and_takes_longer_alternate_path() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=0.0, y=1.0),
        "D": Landmark(name="D", x=1.0, y=1.0),
    }
    edges = []
    for src, dst in (
        ("A", "B"), ("B", "A"), ("B", "D"), ("D", "B"),
        ("A", "C"), ("C", "A"), ("C", "D"), ("D", "C"),
    ):
        start = landmarks[src]
        end = landmarks[dst]
        edges.append(GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(WorldPoint(start.x, start.y), WorldPoint(end.x, end.y)),
            properties={"direction": 2},
        ))
    manager = WebFleetManager(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0},
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "planner_backend": "rolling_sipp",
                "rolling_horizon_sec": 10.0,
                "reservation_horizon_sec": 10.0,
                "reservation_time_step_sec": 1.0,
            },
        },
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        has_executed_route=True,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="D",
        vehicle="r1",
        status="QUEUED",
        traffic_detour_edges=[("A", "B"), ("B", "A")],
        traffic_detour_attempts=1,
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order

    manager.params["fleet"]["rolling_horizon_sec"] = 1.1
    assert manager._rolling_planning_goal("A", "D", order) == "C"
    manager.params["fleet"]["rolling_horizon_sec"] = 10.0

    assert manager._dispatch_order(order, force=True)
    assert order.status == "EXECUTING"
    assert robot.route_final_lm == "D"
    assert robot.plan_nodes[0] == "A"
    assert robot.plan_nodes[-1] == "D"
    assert "B" not in robot.plan_nodes
    assert "C" in robot.plan_nodes
    assert order.traffic_detour_edges == []


def test_runtime_safety_invariant_rolls_back_an_overlapping_tick(monkeypatch) -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="MOVING",
            active_order_id="o1",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "edgeId": "A->B", "lm": "A"},
                {"x": 0.4, "y": 0.0, "yaw": 0.0, "t": 0.2, "edgeId": "A->B"},
            ],
            last_tick_at=now - 0.2,
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            target_lm="A",
            status="MOVING",
            active_order_id="o2",
            pose={"x": 0.8, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"x": 0.8, "y": 0.0, "yaw": math.pi, "t": 0.0, "edgeId": "B->A", "lm": "B"},
                {"x": 0.4, "y": 0.0, "yaw": math.pi, "t": 0.2, "edgeId": "B->A"},
            ],
            last_tick_at=now - 0.2,
        ),
    }
    manager.orders = {
        "o1": FleetOrder(
            order_id="o1",
            target_lm="B",
            vehicle="r1",
            status="EXECUTING",
            assigned_robot="r1",
        ),
        "o2": FleetOrder(
            order_id="o2",
            target_lm="A",
            vehicle="r2",
            status="EXECUTING",
            assigned_robot="r2",
        ),
    }
    monkeypatch.setattr(manager, "_blocked_at_clock", lambda robot, clock: "")

    manager._advance_runtime()

    first = manager.robots["r1"]
    second = manager.robots["r2"]
    assert first.pose["x"] == 0.0
    assert second.pose["x"] == 0.8
    assert not manager.collision.footprints_overlap(first.pose, second.pose)
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 1
    assert {first.status, second.status} == {"MOVING", "WAITING"}
    assert first.active_order_id == "o1"
    assert second.active_order_id == "o2"
    assert manager.orders["o1"].status != "COMPLETED"
    assert manager.orders["o2"].status != "COMPLETED"


def test_runtime_safety_invariant_detects_a_between_tick_pass_through() -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="MOVING",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[{"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0}],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            target_lm="A",
            status="MOVING",
            pose={"x": 1.2, "y": 0.0, "yaw": math.pi},
            trajectory=[{"x": 1.2, "y": 0.0, "yaw": math.pi, "t": 0.0}],
        ),
    }
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    manager.robots["r1"].pose = {"x": 1.2, "y": 0.0, "yaw": 0.0}
    manager.robots["r2"].pose = {"x": 0.0, "y": 0.0, "yaw": math.pi}

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert manager.robots["r1"].pose["x"] == 0.0
    assert manager.robots["r2"].pose["x"] == 1.2
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 1


def test_runtime_tick_does_not_wait_for_background_mapf(monkeypatch) -> None:
    manager = _long_line_manager(edge_count=20)
    manager.add_robot({"name": "r1", "spawnLm": "N0", "mode": "simulated"})
    manager.set_order(
        {"id": "o1", "vehicle": "r1", "targetLm": "N20"},
        dispatch=False,
    )
    started = Event()
    release = Event()
    original = manager._plan_valid_requests

    def delayed_plan(requests, payload):
        started.set()
        release.wait(timeout=1.0)
        return original(requests, payload)

    monkeypatch.setattr(manager, "_plan_valid_requests", delayed_plan)
    before = perf_counter()
    manager._advance_runtime()
    elapsed = perf_counter() - before

    assert started.wait(timeout=0.5)
    assert elapsed < 0.1
    assert manager.orders["o1"].status == "PLANNING"
    release.set()
    deadline = perf_counter() + 2.0
    while perf_counter() < deadline and manager.robots["r1"].status != "MOVING":
        manager._advance_runtime()
        sleep(0.01)
    assert manager.robots["r1"].status == "MOVING"


def test_wait_timeout_replan_is_returned_to_background_queue(monkeypatch) -> None:
    manager = _manager()
    now = time()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[{"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "A"}],
        last_reason="keep clearance from parked",
        blocked_since=now - 10.0,
    )
    manager.robots[robot.name] = robot
    manager.orders["o1"] = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        status="WAITING_TRAFFIC",
        assigned_robot="r1",
    )
    monkeypatch.setattr(
        manager,
        "_maybe_replan_robot",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("synchronous MAPF")),
    )

    manager._resolve_runtime_wait_cycles(now)

    assert manager.orders["o1"].status == "QUEUED"
    assert robot.active_order_id == ""
    assert robot.status == "IDLE"


def test_remote_route_payload_contains_absolute_timed_segment_contract() -> None:
    manager = _manager()
    robot = FleetRobot(name="r1", current_lm="A", mode="remote")
    order = FleetOrder(order_id="order-1", target_lm="B", vehicle="r1")
    before = time()

    route = manager._remote_route_payload(
        robot,
        order,
        {
            "startLm": "A",
            "goalLm": "B",
            "nodes": ["A", "B"],
            "trajectory": [
                {"x": 0.0, "y": 0.0, "t": 2.0, "edgeId": "A->B"},
                {"x": 2.0, "y": 0.0, "t": 4.0, "edgeId": "A->B"},
            ],
        },
    )

    assert route["protocolVersion"] == 2
    assert route["dispatchEpochSec"] >= before + 0.24
    assert route["timedSegments"] == [
        {
            "kind": "move",
            "from": "A",
            "to": "B",
            "notBeforeSec": 2.0,
            "plannedArrivalSec": 4.0,
        }
    ]


def test_remote_robot_owned_by_operator_is_not_available_to_fleet() -> None:
    manager = _manager()
    adapter = _RemoteControlAdapter(owner_id="operator-app", owner_name="Operator App")
    manager.remote_adapter = adapter
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        mode="remote",
        base_url="grpc://robot1:50051",
    )
    manager.robots[robot.name] = robot

    assert not manager._robot_can_accept_order(robot, explicit=True)
    assert robot.remote_status["controlOwner"] == "operator-app"


def test_remote_order_pauses_for_operator_and_replans_after_release(monkeypatch) -> None:
    manager = _manager()
    adapter = _RemoteControlAdapter(owner_id="operator-app", owner_name="Operator App")
    manager.remote_adapter = adapter
    now = time()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        mode="remote",
        base_url="grpc://robot1:50051",
        active_order_id="o1",
        trajectory=[
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "A"},
            {"x": 2.0, "y": 0.0, "yaw": 0.0, "t": 2.0, "lm": "B"},
        ],
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order

    manager._advance_remote_robot_order(robot, now)

    assert robot.status == "MANUAL"
    assert robot.active_order_id == "o1"
    assert not robot.trajectory
    assert order.status == "PAUSED"
    assert "Operator App" in order.error

    dispatched: list[tuple[str, bool]] = []
    monkeypatch.setattr(
        manager,
        "_dispatch_order",
        lambda queued, force=False: dispatched.append((queued.order_id, force)) or True,
    )
    adapter.owner_id = ""
    adapter.owner_name = ""

    manager._advance_remote_robot_order(robot, now + 1.0)

    assert robot.active_order_id == ""
    assert order.status == "QUEUED"
    assert order.error == ""
    assert dispatched == [("o1", True)]


def test_direct_takeover_is_mirrored_before_next_remote_poll() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        mode="remote",
        base_url="grpc://robot1:50051",
        active_order_id="o1",
        trajectory=[{"x": 0.0, "y": 0.0, "t": 0.0, "lm": "A"}],
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order

    mirrored = manager.note_external_control_takeover(
        "grpc://robot1:50051",
        owner_id="operator-app",
        owner_name="Operator App",
    )

    assert mirrored
    assert robot.status == "MANUAL"
    assert robot.remote_status["controlOwner"] == "operator-app"
    assert order.status == "PAUSED"
    assert not robot.trajectory


def test_fleet_control_acquire_never_forces_operator_takeover() -> None:
    manager = _manager()
    adapter = _RemoteControlAdapter()
    manager.remote_adapter = adapter
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        mode="remote",
        base_url="grpc://robot1:50051",
    )

    manager._ensure_remote_control(robot, "execute route")

    assert adapter.acquire_calls == [
        {
            "endpoint": "grpc://robot1:50051",
            "owner_id": FLEET_CONTROL_OWNER_ID,
            "owner_name": "Fleet Manager",
            "force": False,
            "lease_ms": 0,
        }
    ]


def test_far_order_selects_a_reachable_prefix_before_time_aware_mapf() -> None:
    manager = _long_line_manager(edge_count=56)
    manager.add_robot({"name": "r1", "spawnLm": "N0", "mode": "simulated"})

    result = manager.set_order(
        {
            "id": "far-order",
            "vehicle": "r1",
            "targetLm": "N56",
            "speed": 1.37,
            "acceleration": 0.6,
        }
    )

    robot = manager.robots["r1"]
    order = manager.orders["far-order"]
    assert result["ok"]
    assert order.status == "EXECUTING"
    assert robot.status == "MOVING"
    assert robot.route_final_lm == "N56"
    assert robot.route_chunk_goal_lm != "N56"
    assert len(set(robot.plan_nodes)) > 1
    assert robot.to_dict()["targetLm"] == "N56"
    assert len(robot.route_preview) > len(robot.plan_nodes)
    assert math.isclose(robot.route_preview[-1]["x"], manager.landmarks["N56"].x)
    assert math.isclose(robot.route_preview[-1]["y"], manager.landmarks["N56"].y)


def test_dynamic_orders_keep_fifo_within_each_robot_queue() -> None:
    manager = _long_line_manager(edge_count=20)
    manager.add_robot({"name": "r1", "spawnLm": "N0", "mode": "simulated"})
    manager.set_order(
        {
            "id": "dynamic-session-0000001-r1",
            "vehicle": "r1",
            "targetLm": "N20",
            "priority": 0,
        },
        dispatch=False,
    )
    manager.set_order(
        {
            "id": "dynamic-session-0000002-r1",
            "vehicle": "r1",
            "targetLm": "N19",
            "priority": 10,
        },
        dispatch=False,
    )
    first = manager.orders["dynamic-session-0000001-r1"]
    second = manager.orders["dynamic-session-0000002-r1"]

    ready = manager._ready_simulated_order_entries([second, first])

    assert [entry[0].order_id for entry in ready] == [first.order_id]


class _RemoteControlAdapter:
    transport = "grpc"

    def __init__(self, owner_id: str = "", owner_name: str = "") -> None:
        self.owner_id = owner_id
        self.owner_name = owner_name
        self.acquire_calls: list[dict[str, object]] = []

    def status(self, endpoint: str) -> dict[str, object]:
        del endpoint
        return {
            "robot": {
                "robotId": "r1",
                "connected": True,
                "state": "IDLE",
                "nearestLm": "A",
                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "controlOwner": self.owner_id,
                "controlOwnerName": self.owner_name,
                "control": {
                    "ownerId": self.owner_id,
                    "ownerName": self.owner_name,
                },
            }
        }

    def acquire_control(self, endpoint: str, **kwargs) -> dict[str, object]:
        self.acquire_calls.append({"endpoint": endpoint, **kwargs})
        return {"ok": True}


def _manager() -> WebFleetManager:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=2.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(2.0, 0.0)),
        properties={"direction": 1},
    )
    return WebFleetManager(
        landmarks,
        [edge],
        params={
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "runtime_replan_lm_tolerance_m": 0.1,
                "remote_dispatch_lead_sec": 0.25,
            },
        },
    )


def _long_line_manager(edge_count: int) -> WebFleetManager:
    landmarks = {
        f"N{index}": Landmark(name=f"N{index}", x=index * 1.2, y=0.0)
        for index in range(edge_count + 1)
    }
    edges = [
        GraphEdge(
            from_name=f"N{index}",
            to_name=f"N{index + 1}",
            length=1.2,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(index * 1.2, 0.0),
                WorldPoint((index + 1) * 1.2, 0.0),
            ),
            properties={"direction": 1},
        )
        for index in range(edge_count)
    ]
    return WebFleetManager(
        landmarks,
        edges,
        params={
            "navigation": {
                "route_speed": 1.37,
                "route_acceleration": 0.6,
            },
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "planner_backend": "rolling_sipp",
                "reservation_time_step_sec": 1.0,
                "rolling_horizon_sec": 10.0,
                "reservation_horizon_sec": 10.0,
                "cbs_low_level_max_time": 160,
                "dispatch_plan_budget_per_tick": 2,
                "dispatch_joint_batch_size": 2,
            },
        },
    )
