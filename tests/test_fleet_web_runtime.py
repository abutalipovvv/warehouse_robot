from __future__ import annotations

import math
from threading import Event
from time import perf_counter, sleep, time

import pytest

import fleet_manager.core.manager as runtime_module
from fleet_manager.core.constants import FLEET_CONTROL_OWNER_ID
from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.grpc.manager import FleetManagerROS
from fleet_manager.runtime.simulation.manager import FleetManagerSim


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


def test_manual_pose_update_can_skip_full_fleet_snapshot(monkeypatch) -> None:
    manager = _manager()
    manager.robots["r1"] = FleetRobot(
        name="r1",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    monkeypatch.setattr(
        manager,
        "state",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("manual command serialized the complete fleet")
        ),
    )

    result = manager.update_robot(
        {
            "name": "r1",
            "status": "MANUAL",
            "pose": {"x": 0.03, "y": 0.0, "yaw": 0.0},
            "currentLm": "A",
        },
        include_state=False,
    )

    assert result["state"] is None
    assert result["robot"]["pose"]["x"] == pytest.approx(0.03)


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


@pytest.mark.parametrize(
    ("rotation_distance", "expected_loser_reason"),
    [
        (1.304, "yield to r1"),
        (1.10, ""),
    ],
)
def test_runtime_respects_configured_rotation_clearance(
    rotation_distance: float,
    expected_loser_reason: str,
) -> None:
    manager = _manager()
    manager.planner.rotation_min_robot_center_distance_m = rotation_distance
    rotating = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id="o1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi / 2.0,
                "edgeId": "WAIT@ROTATE:A",
                "lm": "A",
            },
            {
                "t": 3.0,
                "x": 0.0,
                "y": 1.0,
                "yaw": math.pi / 2.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
    )
    other_rotating = FleetRobot(
        name="r2",
        current_lm="near",
        target_lm="far",
        status="MOVING",
        active_order_id="o2",
        pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 1.2,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "near->near",
                "lm": "near",
            },
            {
                "t": 2.0,
                "x": 1.2,
                "y": 0.0,
                "yaw": -math.pi / 2.0,
                "edgeId": "WAIT@ROTATE:near",
                "lm": "near",
            },
        ],
    )
    manager.robots = {rotating.name: rotating, other_rotating.name: other_rotating}

    winner_reason = manager._blocked_at_clock(rotating, 0.1, [other_rotating])
    loser_reason = manager._blocked_at_clock(other_rotating, 0.1, [rotating])

    assert winner_reason == ""
    assert loser_reason == expected_loser_reason


def test_future_broadphase_does_not_block_disjoint_adjacent_footprints() -> None:
    manager = _manager()
    manager.params.update(
        {
            "robot_model": {
                "footprint": [
                    {"x": -0.523, "y": -0.3532},
                    {"x": 0.477, "y": -0.3532},
                    {"x": 0.477, "y": 0.3468},
                    {"x": -0.523, "y": 0.3468},
                ]
            },
            "navigation": {"collision_margin": 0.04, "route_speed": 1.0},
        }
    )
    manager.params["fleet"]["robot_clearance_m"] = 0.10
    manager.collision.set_params(manager.params)
    moving = FleetRobot(
        name="moving",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
        ],
    )
    adjacent = FleetRobot(
        name="adjacent",
        current_lm="B",
        status="WAITING",
        pose={"x": 0.0, "y": 1.2, "yaw": 0.0},
    )
    manager.robots = {moving.name: moving, adjacent.name: adjacent}

    assert math.hypot(0.0, 1.2) < manager.collision.robot_broadphase_distance()
    assert not manager.collision.robot_footprints_conflict(moving.pose, adjacent.pose)
    assert manager._blocked_at_clock(moving, 0.5) == ""


def test_zero_robot_clearance_allows_safe_adjacent_turns() -> None:
    manager = _manager()
    manager.params.update(
        {
            "robot_model": {
                "footprint": [
                    {"x": -0.523, "y": -0.3532},
                    {"x": 0.477, "y": -0.3532},
                    {"x": 0.477, "y": 0.3468},
                    {"x": -0.523, "y": 0.3468},
                ]
            },
            "navigation": {"collision_margin": 0.04, "route_speed": 1.0},
        }
    )
    first = {"x": 0.0, "y": 1.2, "yaw": -1.0879953107243916}
    second = {"x": 0.0, "y": 0.0, "yaw": 2.2729263727750624}

    manager.params["fleet"]["robot_clearance_m"] = 0.10
    manager.collision.set_params(manager.params)
    assert manager.collision.robot_footprints_conflict(first, second)
    assert not manager.collision.footprints_overlap(first, second)

    manager.params["fleet"]["robot_clearance_m"] = 0.0
    manager.collision.set_params(manager.params)
    assert manager.collision.robot_collision_margin() == pytest.approx(0.04)
    assert not manager.collision.robot_footprints_conflict(first, second)
    assert not manager.collision.footprints_overlap(first, second)


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
            trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            status="WAITING",
            last_reason="yield to r1",
            blocked_since=now - 1.0,
            trajectory=[{"t": 0.0, "x": 2.0, "y": 0.0, "edgeId": "B->A", "lm": "B"}],
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
        "coupledReplansStarted": 0,
        "coupledReplansSucceeded": 0,
        "coupledReplansFailed": 0,
        "priorityGrants": 1,
        "runtimeSafetyRollbacks": 0,
        "zoneAdmissionWaits": 0,
        "zoneAdmissionsGranted": 0,
    }

    # The same physical encounter can remain visible for several 10 Hz ticks.
    # It is one deadlock episode, not a new cycle and lease on every frame.
    manager.robots["r1"].status = "WAITING"
    manager.robots["r1"].last_reason = "yield to r2"
    manager._resolve_runtime_wait_cycles(now + 0.1)
    assert manager.robots["r1"].status == "MOVING"
    assert manager.robots["r1"].last_reason == "deadlock priority active"
    assert manager.robots["r2"].last_reason == "yield to r1"
    assert manager.traffic_metrics["waitCyclesDetected"] == 1
    assert manager.traffic_metrics["priorityGrants"] == 1


def test_wait_dependency_is_exposed_for_operator_visualization() -> None:
    robot = FleetRobot(
        name="waiting",
        current_lm="A",
        status="WAITING",
        last_reason="yield to blocker",
        wait_for_robot="blocker",
        wait_resource="A->B",
        wait_release_at=123.5,
    )

    payload = robot.to_dict()

    assert payload["waitDependency"] == {
        "robot": "blocker",
        "resource": "A->B",
        "releaseAt": 123.5,
    }


def test_compact_stream_tick_keeps_pose_but_omits_slow_runtime_details() -> None:
    manager = _manager()
    manager.robots["r1"] = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.2, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0},
        ],
        plan_nodes=["A", "B"],
        route_revision=7,
        route_clock=0.2,
    )

    state = manager.stream_tick(
        route_revisions={"r1": 7},
        include_runtime_details=False,
    )

    assert state["robots"][0]["pose"]["x"] == pytest.approx(0.2)
    assert state["robots"][0]["routeClock"] == pytest.approx(0.2)
    assert state["robots"][0]["trajectory"] == []
    assert state["robots"][0]["planNodes"] == []
    assert "orders" not in state
    assert "events" not in state
    assert "trafficFlow" not in state

    detailed = manager.stream_tick(include_runtime_details=True)
    assert "orders" in detailed
    assert "events" in detailed
    assert "trafficFlow" in detailed


def test_stalled_cycle_expires_bad_lease_and_prefers_clear_forward_path(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    robots = {
        "clear": FleetRobot(
            name="clear",
            current_lm="A",
            status="WAITING",
            last_reason="yield to blocked",
            blocked_since=now - 9.0,
            traffic_stall_since=now - 9.0,
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "blocked": FleetRobot(
            name="blocked",
            current_lm="B",
            status="WAITING",
            last_reason="yield to clear",
            blocked_since=now - 9.0,
            traffic_stall_since=now - 9.0,
            traffic_priority_until=now + 20.0,
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
            trajectory=[{"t": 0.0, "x": 2.0, "y": 0.0, "edgeId": "B->A", "lm": "B"}],
        ),
    }
    manager.robots = robots
    manager.orders = {
        "o-clear": FleetOrder(
            order_id="o-clear",
            target_lm="B",
            vehicle="clear",
            assigned_robot="clear",
            status="EXECUTING",
            priority=0,
        ),
        "o-blocked": FleetOrder(
            order_id="o-blocked",
            target_lm="A",
            vehicle="blocked",
            assigned_robot="blocked",
            status="EXECUTING",
            priority=100,
        ),
    }

    assert not manager._maintain_runtime_wait_cycle_lease(
        ["clear", "blocked"],
        robots,
        now,
    )
    assert robots["blocked"].traffic_priority_until == 0.0

    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda robot, _cycle: 3.0 if robot.name == "clear" else 0.2,
    )
    manager._active_wait_cycles[("blocked", "clear")] = now
    manager._break_runtime_wait_cycle(["clear", "blocked"], robots, now)

    assert robots["clear"].status == "MOVING"
    assert robots["blocked"].status == "WAITING"
    assert robots["blocked"].last_reason == "yield to clear"


def test_cycle_clearance_inspects_the_complete_committed_crossing() -> None:
    manager = _manager()
    clearing = FleetRobot(
        name="clearing",
        current_lm="A",
        status="WAITING",
        pose={"x": 0.8, "y": 2.0, "yaw": 0.0},
        route_clock=4.5,
        trajectory=[
            {"t": 4.5, "x": 0.8, "y": 2.0, "yaw": 0.0, "edgeId": "A->B"},
            {"t": 8.0, "x": 2.2, "y": 2.0, "yaw": 0.0, "edgeId": "A->B"},
        ],
    )
    blocked = FleetRobot(
        name="blocked",
        current_lm="B",
        status="WAITING",
        pose={"x": 1.0, "y": 0.2, "yaw": math.pi / 2.0},
        route_clock=4.5,
        trajectory=[
            {"t": 4.5, "x": 1.0, "y": 0.2, "yaw": math.pi / 2.0, "edgeId": "C->D"},
            {"t": 9.0, "x": 1.0, "y": 2.0, "yaw": math.pi / 2.0, "edgeId": "C->D"},
        ],
    )

    clear_distance = manager._cycle_forward_clearance(
        clearing,
        [clearing, blocked],
    )
    blocked_distance = manager._cycle_forward_clearance(
        blocked,
        [clearing, blocked],
    )

    assert clear_distance == pytest.approx(3.5)
    assert blocked_distance < clear_distance


def test_non_cycle_wait_has_hard_replan_bound_even_with_optimistic_forecast(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    waiting = FleetRobot(
        name="waiting",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        last_reason="yield to moving",
        wait_for_robot="moving",
        blocked_since=now - 9.0,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
    )
    moving = FleetRobot(
        name="moving",
        current_lm="B",
        status="MOVING",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A", "lm": "B"},
        ],
    )
    manager.robots = {waiting.name: waiting, moving.name: moving}
    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(manager, "_wait_expected_to_clear", lambda _robot: True)
    monkeypatch.setattr(
        manager,
        "_schedule_runtime_replan",
        lambda robot, _now, reason: calls.append((robot.name, reason)) or True,
    )

    manager._resolve_runtime_wait_cycles(now)

    assert calls == [("waiting", "traffic wait timeout")]


def test_mid_edge_starvation_transfers_priority_instead_of_waiting_forever(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    waiting = FleetRobot(
        name="waiting",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        last_reason="yield to blocker",
        wait_for_robot="blocker",
        blocked_since=now - 9.0,
        pose={"x": 0.5, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        last_reason="deadlock priority active",
        traffic_priority_until=now + 1.0,
        pose={"x": 1.5, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
        ],
    )
    manager.robots = {waiting.name: waiting, blocker.name: blocker}
    monkeypatch.setattr(manager, "_safe_replan_start_lm", lambda _robot: "")

    manager._resolve_runtime_wait_cycles(now)

    assert waiting.status == "MOVING"
    assert waiting.last_reason == "starvation priority active"
    assert waiting.traffic_priority_until > now
    assert blocker.status == "WAITING"
    assert blocker.last_reason == "yield to waiting"
    assert blocker.blocked_since == now

    # A planner pause longer than the original lease must not consume the
    # right-of-way grant before one actual runtime step can use it.
    expired = now + 10.0
    waiting.traffic_priority_until = expired - 1.0
    manager._refresh_runtime_priority_lease(waiting, expired)
    assert waiting.traffic_priority_until > expired


def test_persistent_wait_chain_gets_one_component_winner(monkeypatch) -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            status="WAITING",
            last_reason="yield to r2",
            blocked_since=now - 9.0,
            trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            status="WAITING",
            last_reason="yield to r3",
            blocked_since=now - 9.0,
            trajectory=[{"t": 0.0, "x": 1.0, "y": 0.0, "edgeId": "B->A", "lm": "B"}],
        ),
        "r3": FleetRobot(
            name="r3",
            current_lm="B",
            status="MOVING",
            trajectory=[{"t": 0.0, "x": 2.0, "y": 0.0, "edgeId": "B->A", "lm": "B"}],
        ),
    }
    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda robot, _robots: 3.0 if robot.name == "r3" else 1.0,
    )

    manager._resolve_runtime_wait_cycles(now)

    assert manager.robots["r3"].status == "MOVING"
    assert manager.robots["r3"].last_reason == "starvation priority active"
    assert manager.robots["r1"].last_reason == "yield to r3"
    assert manager.robots["r2"].last_reason == "yield to r3"


def test_admission_wait_times_out_to_spatial_replan(monkeypatch) -> None:
    manager = _manager()
    now = time()
    robot = FleetRobot(
        name="waiting",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        last_reason="traffic admission wait at A for flow:1:1",
        blocked_since=now - 11.0,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
    )
    manager.robots = {robot.name: robot}
    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(
        manager,
        "_schedule_runtime_replan",
        lambda target, _now, reason: calls.append((target.name, reason)) or True,
    )

    manager._resolve_runtime_wait_cycles(now)

    assert calls == [
        (
            "waiting",
            "traffic admission timeout: traffic admission wait at A for flow:1:1",
        )
    ]
    assert manager._reason_requires_spatial_replan(calls[0][1])


def test_at_lm_deadlock_waits_before_escalating_to_detour() -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="o1",
            last_reason="yield to r2",
            blocked_since=now - 1.0,
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
                {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
            ],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            target_lm="A",
            status="WAITING",
            active_order_id="o2",
            last_reason="yield to r1",
            blocked_since=now - 1.0,
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
                {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
            ],
        ),
    }
    manager.orders = {
        "o1": FleetOrder(order_id="o1", target_lm="B", vehicle="r1", status="EXECUTING"),
        "o2": FleetOrder(order_id="o2", target_lm="A", vehicle="r2", status="EXECUTING"),
    }

    manager._resolve_runtime_wait_cycles(now)

    winner = manager.robots["r1"]
    yielding = manager.robots["r2"]
    assert winner.status == "MOVING"
    assert winner.trajectory
    assert yielding.status == "WAITING"
    assert yielding.trajectory
    assert yielding.active_order_id == "o2"
    assert yielding.wait_for_robot == "r1"
    assert manager.orders["o2"].status == "EXECUTING"
    assert manager.orders["o2"].traffic_detour_attempts == 0
    assert all(
        robot.status != "MOVING" or bool(robot.trajectory)
        for robot in manager.robots.values()
    )


def test_persistent_wait_cycle_escalates_only_the_coupled_group(monkeypatch) -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="o1",
            last_reason="yield to r2",
            blocked_since=now - 6.0,
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            target_lm="A",
            status="WAITING",
            active_order_id="o2",
            last_reason="yield to r1",
            blocked_since=now - 6.0,
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
            trajectory=[{"t": 0.0, "x": 2.0, "y": 0.0, "edgeId": "B->A", "lm": "B"}],
        ),
    }
    manager.orders = {
        "o1": FleetOrder(order_id="o1", target_lm="B", vehicle="r1", status="EXECUTING"),
        "o2": FleetOrder(order_id="o2", target_lm="A", vehicle="r2", status="EXECUTING"),
    }
    calls: list[tuple[list[str], str]] = []
    monkeypatch.setattr(
        manager,
        "_start_async_coupled_replan",
        lambda robots, winner, _now: calls.append(
            (sorted(robot.name for robot in robots), winner.name)
        ) or True,
    )

    manager._resolve_runtime_wait_cycles(now)

    assert calls == [(["r1", "r2"], "r1")]
    assert manager.robots["r1"].status == "MOVING"
    assert manager.robots["r2"].status == "WAITING"
    assert manager.robots["r2"].retreat_target_clock is None


def test_runtime_normalizes_stale_moving_status_without_trajectory() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="MOVING",
        last_reason="deadlock priority granted",
        traffic_priority_until=time() + 10.0,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots[robot.name] = robot

    manager._advance_runtime()

    assert robot.status == "IDLE"
    assert robot.traffic_priority_until == 0.0
    assert robot.blocked_since is None
    assert robot.last_reason == "idle: no active trajectory"


def test_runtime_requeues_active_order_that_lost_its_trajectory(monkeypatch) -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="MOVING",
        active_order_id="o1",
        last_reason="deadlock priority granted",
        traffic_priority_until=time() + 10.0,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
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
    monkeypatch.setattr(manager, "_dispatch_orders", lambda *args, **kwargs: 0)

    manager._advance_runtime()

    assert robot.status == "IDLE"
    assert not robot.active_order_id
    assert robot.traffic_priority_until == 0.0
    assert order.status == "QUEUED"
    assert order.start_lm == "A"


def test_deadlock_retreat_never_targets_an_occupied_landmark() -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="o1",
            last_reason="yield to r2",
            blocked_since=now - 1.0,
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
            active_order_id="o2",
            last_reason="yield to r1",
            blocked_since=now - 1.0,
            pose={"x": 1.5, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
            ],
            route_clock=0.25,
        ),
        "parked": FleetRobot(
            name="parked",
            current_lm="B",
            status="IDLE",
            pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        ),
    }
    manager.orders = {
        "o1": FleetOrder(order_id="o1", target_lm="B", vehicle="r1", status="EXECUTING"),
        "o2": FleetOrder(order_id="o2", target_lm="A", vehicle="r2", status="EXECUTING"),
    }

    manager._resolve_runtime_wait_cycles(now)

    assert manager.robots["r1"].status == "MOVING"
    assert manager.robots["r2"].status == "WAITING"
    assert manager.robots["r2"].retreat_target_clock is None
    assert manager.robots["r2"].last_reason == "yield to r1"


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
            blocked_since=now - 13.0,
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
            blocked_since=now - 13.0,
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
    cycle_key = ("r1", "r2")
    manager._coupled_replan_failures[cycle_key] = 1
    manager._coupled_replan_last_attempt[cycle_key] = now

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


def test_continuous_reservation_wait_is_inserted_before_entering_edge(
    monkeypatch,
) -> None:
    manager = _manager()
    trajectory = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
        {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
    ]
    conflicts = iter(
        [
            {"time": 1.0, "other": "r2", "edge": "A->B"},
            None,
            None,
        ]
    )
    monkeypatch.setattr(
        manager,
        "_first_continuous_corridor_conflict",
        lambda *args, **kwargs: next(conflicts),
    )
    monkeypatch.setattr(manager, "_wait_duration_for_conflict", lambda *args, **kwargs: 1.0)

    scheduled, stats = manager._schedule_trajectory_against_corridors(
        "r1",
        trajectory,
    )

    wait = next(sample for sample in scheduled if sample["edgeId"].startswith("WAIT@"))
    assert wait["x"] == manager.landmarks["A"].x
    assert wait["y"] == manager.landmarks["A"].y
    assert wait["lm"] == "A"
    assert stats == {"conflicts": 1, "waits": 1, "wait": 1.0, "unresolved": 0}


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
    manager = FleetManagerSim(
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
    monkeypatch.setattr(manager, "_blocked_at_clock", lambda robot, clock, **kwargs: "")

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


def test_runtime_preflight_waits_before_crossing_without_rollback(monkeypatch) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    manager = _manager()
    manager.params["navigation"] = {
        "route_speed": 1.0,
        "route_acceleration": 0.6,
        "footprint_lookahead": 0.8,
        "stop_distance": 0.4,
    }
    manager.collision.set_params(manager.params)
    horizontal = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id="o1",
        pose={"x": -1.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": -1.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "edgeId": "A->B", "lm": "A"},
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "t": 2.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_started_at=clock[0],
        last_tick_at=clock[0],
        updated_at=clock[0],
    )
    vertical = FleetRobot(
        name="r2",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        active_order_id="o2",
        pose={"x": 0.0, "y": -1.0, "yaw": math.pi / 2.0},
        trajectory=[
            {"x": 0.0, "y": -1.0, "yaw": math.pi / 2.0, "t": 0.0, "edgeId": "B->A", "lm": "B"},
            {"x": 0.0, "y": 1.0, "yaw": math.pi / 2.0, "t": 2.0, "edgeId": "B->A", "lm": "A"},
        ],
        route_started_at=clock[0],
        last_tick_at=clock[0],
        updated_at=clock[0],
    )
    manager.robots = {"r1": horizontal, "r2": vertical}
    manager.orders = {
        "o1": FleetOrder(order_id="o1", target_lm="B", vehicle="r1", assigned_robot="r1", status="EXECUTING"),
        "o2": FleetOrder(order_id="o2", target_lm="A", vehicle="r2", assigned_robot="r2", status="EXECUTING"),
    }

    saw_early_wait = False
    previous_x = horizontal.pose["x"]
    previous_y = vertical.pose["y"]
    for _ in range(30):
        clock[0] += 0.1
        manager._advance_runtime()
        assert horizontal.pose["x"] >= previous_x - 0.000001
        assert vertical.pose["y"] >= previous_y - 0.000001
        assert not manager.collision.footprints_overlap(horizontal.pose, vertical.pose)
        previous_x = horizontal.pose["x"]
        previous_y = vertical.pose["y"]
        if vertical.status == "WAITING" and vertical.pose["y"] <= -0.95:
            saw_early_wait = True

    assert saw_early_wait
    assert horizontal.pose["x"] > 0.5
    assert vertical.pose["y"] > -0.9
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 0


def test_runtime_far_preflight_is_throttled_but_motion_stays_continuous(
    monkeypatch,
) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    manager = _manager()
    manager.params.setdefault("fleet", {})[
        "runtime_collision_preflight_interval_sec"
    ] = 0.20
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "edgeId": "A->B", "lm": "A"},
            {"x": 2.0, "y": 0.0, "yaw": 0.0, "t": 2.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_started_at=clock[0],
        last_tick_at=clock[0],
        updated_at=clock[0],
        route_revision=7,
    )
    manager.robots = {robot.name: robot}
    preflight_calls = 0

    def count_preflight(_robot, _clock):
        nonlocal preflight_calls
        preflight_calls += 1
        return ""

    monkeypatch.setattr(manager, "_blocked_ahead", count_preflight)
    monkeypatch.setattr(manager, "_blocked_at_clock", lambda *_args, **_kwargs: "")

    for _ in range(10):
        clock[0] += 0.05
        manager._advance_runtime()

    assert 2 <= preflight_calls <= 4
    assert robot.status == "MOVING"
    assert robot.route_clock == pytest.approx(0.5)
    assert robot.pose["x"] == pytest.approx(0.5)


def test_runtime_prediction_uses_common_tick_clock_after_peer_advanced() -> None:
    manager = _manager()
    peer = FleetRobot(
        name="peer",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.2, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "edgeId": "A->B"},
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "t": 1.0, "edgeId": "A->B"},
        ],
        # Simulate this peer having already been processed earlier in the
        # sequential loop while the common tick began at clock zero.
        route_clock=0.2,
    )
    manager._runtime_tick_route_clocks = {"peer": 0.0}

    predicted = manager._predicted_robot_pose(peer, 0.1)

    assert predicted is not None
    assert predicted["x"] == pytest.approx(0.1)


def test_deadlock_wait_timeout_uses_configured_value() -> None:
    manager = _manager()
    manager.params.setdefault("fleet", {})["deadlock_wait_timeout_sec"] = 4.5

    assert manager._deadlock_wait_timeout() == pytest.approx(4.5)


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


def test_prefetched_route_consumes_same_tick_remainder_without_stopping(monkeypatch) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    manager = _long_line_manager(edge_count=2)
    now = manager.simulation_time()
    order = FleetOrder(
        order_id="o1",
        target_lm="N2",
        vehicle="r1",
        status="EXECUTING",
        assigned_robot="r1",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="N0",
        target_lm="N1",
        status="MOVING",
        pose={"x": 1.14, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "N0"},
            {"x": 1.2, "y": 0.0, "yaw": 0.0, "t": 1.0, "lm": "N1"},
        ],
        plan_nodes=["N0", "N1"],
        route_clock=0.95,
        route_started_at=now,
        last_tick_at=now,
        active_order_id="o1",
        route_chunk_goal_lm="N1",
        route_final_lm="N2",
    )
    continuation = {
        "ok": True,
        "plans": [
            {
                "robot": "r1",
                "startLm": "N1",
                "goalLm": "N2",
                "finalGoalLm": "N2",
                "nodes": ["N1", "N2"],
                "times": [0, 1],
                "trajectory": [
                    {"x": 1.2, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "N1"},
                    {"x": 2.4, "y": 0.0, "yaw": 0.0, "t": 1.0, "lm": "N2"},
                ],
            }
        ],
    }
    robot.pending_route = {
        "order_id": "o1",
        "start_lm": "N1",
        "final_goal": "N2",
        "result": continuation,
    }
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot

    clock[0] += 0.10
    manager.advance_runtime()

    assert robot.status == "MOVING"
    assert robot.active_order_id == "o1"
    assert robot.current_lm == "N1"
    assert robot.route_chunk_goal_lm == "N2"
    assert robot.route_clock == pytest.approx(0.05)
    assert robot.pose["x"] > 1.2


def test_completed_rolling_chunk_keeps_active_order_while_prefetch_is_pending(
    monkeypatch,
) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    manager = _long_line_manager(edge_count=2)
    order = FleetOrder(
        order_id="o1",
        target_lm="N2",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="N1",
        target_lm="N1",
        status="MOVING",
        pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "N0"},
            {"x": 1.2, "y": 0.0, "yaw": 0.0, "t": 1.0, "lm": "N1"},
        ],
        plan_nodes=["N0", "N1"],
        route_clock=1.0,
        route_started_at=clock[0] - 1.0,
        last_tick_at=clock[0] - 0.1,
        active_order_id="o1",
        route_chunk_goal_lm="N1",
        route_final_lm="N2",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    monkeypatch.setattr(manager, "_dispatch_orders", lambda *args, **kwargs: 0)

    manager._advance_runtime()

    assert robot.status == "WAITING"
    assert robot.last_reason == "rolling continuation pending"
    assert robot.active_order_id == "o1"
    assert robot.target_lm == "N1"
    assert robot.route_clock == pytest.approx(1.0)
    assert robot.trajectory
    assert order.status == "PLANNING"
    assert order.error == "rolling continuation pending"


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
    assert robot.target_lm == ""
    assert robot.last_reason == "route replan queued"


def test_legacy_idle_deadlock_replan_state_is_released_for_dispatch(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        last_reason=(
            "background replan queued: "
            "deadlock at LM; alternate corridor required"
        ),
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="QUEUED",
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    observed: list[tuple[str, str, bool]] = []

    def capture_dispatch(*args, **kwargs):
        observed.append(
            (
                robot.target_lm,
                robot.last_reason,
                manager._robot_can_accept_order(robot, explicit=True),
            )
        )
        return 0

    monkeypatch.setattr(manager, "_dispatch_orders", capture_dispatch)

    manager._advance_runtime()

    assert observed == [("", "route replan queued", True)]


def test_deadlock_recovery_dispatch_preempts_nonurgent_prefetch(monkeypatch) -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        last_reason="route replan queued",
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="QUEUED",
        traffic_blocked_since=time(),
    )
    request = {"name": "r1", "startLm": "A", "goalLm": "B"}
    entry = (order, robot, request, "B")
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    calls: list[str] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(manager, "_ready_simulated_order_entries", lambda orders: [entry])
    monkeypatch.setattr(
        manager,
        "_ready_rolling_prefetch_entry",
        lambda: (order, robot, request, "B", 5.0),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda entries: calls.append("recovery"),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda prefetch: calls.append("prefetch"),
    )

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["recovery"]


def test_remote_route_payload_contains_absolute_timed_segment_contract() -> None:
    manager = _remote_manager()
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
                "motionDirection": "not_specified",
                "notBeforeSec": 2.0,
            "plannedArrivalSec": 4.0,
        }
    ]


def test_remote_timed_contract_keeps_explicit_rotation_action() -> None:
    manager = _remote_manager()

    segments = manager._timed_segments_from_trajectory(
        [
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "t": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "t": 4.0,
                "edgeId": "WAIT@ROTATE:A",
                "lm": "A",
            },
            {
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "t": 6.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ]
    )

    assert segments[0] == {
        "kind": "rotate",
        "node": "A",
        "fromYaw": math.pi,
        "toYaw": 0.0,
        "notBeforeSec": 0.0,
        "plannedArrivalSec": 4.0,
    }
    assert segments[1]["kind"] == "move"
    assert segments[1]["notBeforeSec"] == 4.0


def test_remote_robot_owned_by_operator_is_not_available_to_fleet() -> None:
    manager = _remote_manager()
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
    manager = _remote_manager()
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
    manager = _remote_manager()
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
    manager = _remote_manager()
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


def test_route_preview_is_replaced_from_each_committed_mapf_revision() -> None:
    manager = _manager()
    robot = FleetRobot(name="r1", current_lm="A")
    order = FleetOrder(order_id="o1", target_lm="B", vehicle="r1", status="EXECUTING")
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    first_plan = {
        "goalLm": "B",
        "finalGoalLm": "B",
        "nodes": ["A", "B"],
        "trajectory": [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0},
            {"x": 2.0, "y": 0.0, "yaw": 0.0, "t": 2.0},
        ],
    }
    manager._apply_simulated_route_metadata(robot, order, first_plan, time())
    first_revision = robot.route_revision

    replanned = {
        **first_plan,
        "trajectory": [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0},
            {"x": 1.0, "y": 0.4, "yaw": 0.0, "t": 1.0},
            {"x": 2.0, "y": 0.0, "yaw": 0.0, "t": 2.0},
        ],
    }
    manager._apply_simulated_route_metadata(robot, order, replanned, time())

    assert robot.route_revision > first_revision
    assert robot.route_preview[1]["y"] == 0.4
    assert all(point["phase"] == "committed" for point in robot.route_preview)


def test_spatial_route_stays_committed_across_rolling_chunks() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "D": Landmark(name="D", x=1.0, y=1.0),
    }
    edges = []
    for start, goal, length in (
        ("A", "B", 1.0),
        ("B", "C", 1.0),
        ("A", "D", 2.0),
        ("D", "C", 2.0),
    ):
        edges.append(
            GraphEdge(
                from_name=start,
                to_name=goal,
                length=length,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[start].x, landmarks[start].y),
                    WorldPoint(landmarks[goal].x, landmarks[goal].y),
                ),
                properties={"direction": 2},
            )
        )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0},
            "fleet": {
                "rolling_horizon_sec": 1.1,
                "reservation_time_step_sec": 1.0,
            },
        },
    )
    order = FleetOrder(order_id="o1", target_lm="C", vehicle="r1")

    assert manager._rolling_planning_goal("A", "C", order) == "B"
    revision = order.spatial_route_revision
    assert order.spatial_route_nodes == ["A", "B", "C"]
    assert manager._rolling_planning_goal("B", "C", order) == "C"
    assert order.spatial_route_nodes == ["A", "B", "C"]
    assert order.spatial_route_revision == revision


def test_traffic_retry_shortens_only_temporal_chunk_to_next_lm() -> None:
    manager = _long_line_manager(edge_count=6)
    manager.params["fleet"]["rolling_horizon_sec"] = 20.0
    order = FleetOrder(
        order_id="o1",
        target_lm="N6",
        vehicle="r1",
        dispatch_failures=1,
    )

    planning_goal = manager._rolling_planning_goal("N0", "N6", order)

    assert planning_goal == "N1"
    assert order.spatial_route_nodes == [f"N{index}" for index in range(7)]


def test_repeated_rolling_failure_uses_joint_recovery_group_and_cbs() -> None:
    manager = _long_line_manager(edge_count=6)
    robot = FleetRobot(name="r1", current_lm="N0", has_executed_route=True)
    manager.robots[robot.name] = robot
    order = FleetOrder(
        order_id="o1",
        target_lm="N6",
        vehicle="r1",
        dispatch_failures=3,
    )

    assert manager._dispatch_recovery_group_limit(order, robot, 2) == 5
    assert manager._order_plan_payload(
        order,
        {"name": "r1", "startLm": "N0", "goalLm": "N1"},
    )["allowCbsFallback"]

    order.dispatch_failures = 0
    assert manager._dispatch_recovery_group_limit(order, robot, 2) == 1
    assert not manager._order_plan_payload(
        order,
        {"name": "r1", "startLm": "N0", "goalLm": "N1"},
    )["allowCbsFallback"]


def test_planned_wait_robot_is_predicted_to_leave_on_its_timeline() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="WAITING",
        last_reason="planned traffic wait at A",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->A", "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "WAIT@A->A", "lm": "A"},
            {"t": 3.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
    )

    predicted = manager._predicted_robot_pose(robot, 2.5)

    assert predicted is not None
    assert predicted["x"] == pytest.approx(1.0)


def test_rolling_schedule_commit_does_not_mark_spatial_replan() -> None:
    manager = _manager()
    robot = FleetRobot(name="r1", current_lm="A", last_replan_at=123.0)
    manager.robots[robot.name] = robot
    result = {
        "ok": True,
        "plans": [
            {
                "robot": "r1",
                "startLm": "A",
                "goalLm": "B",
                "nodes": ["A", "B"],
                "trajectory": [
                    {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                    {"t": 1.0, "x": 2.0, "y": 0.0, "yaw": 0.0},
                ],
            }
        ],
    }

    manager._apply_planner_result(result, now=456.0)

    assert robot.last_replan_at == 123.0


def test_initial_soft_clearance_allows_departure_but_not_approach() -> None:
    manager = _manager()
    manager.params.update(
        {
            "robot_model": {
                "footprint": [
                    {"x": -0.50, "y": -0.35},
                    {"x": 0.50, "y": -0.35},
                    {"x": 0.50, "y": 0.35},
                    {"x": -0.50, "y": 0.35},
                ]
            },
            "navigation": {"collision_margin": 0.04},
        }
    )
    manager.params["fleet"]["robot_clearance_m"] = 0.10
    manager.params["fleet"]["reservation_horizon_sec"] = 2.0
    manager.collision.set_params(manager.params)
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="IDLE",
        pose={"x": 1.08, "y": 0.0, "yaw": 0.0},
    )
    manager.robots[blocker.name] = blocker
    departing = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->A"},
        {"t": 1.0, "x": -1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->OUT"},
    ]
    approaching = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->A"},
        {"t": 1.0, "x": 0.5, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
    ]

    assert manager._first_continuous_corridor_conflict("moving", departing) is None
    conflict = manager._first_continuous_corridor_conflict("moving", approaching)
    assert conflict is not None
    assert conflict["time"] == pytest.approx(0.0)


def test_forced_spatial_replan_prefers_global_low_congestion_route() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "D": Landmark(name="D", x=0.0, y=1.0),
    }
    edges = []
    for start, goal, length in (
        ("A", "B", 1.0),
        ("B", "C", 1.0),
        ("A", "D", 2.0),
        ("D", "C", 2.0),
    ):
        edges.append(
            GraphEdge(
                from_name=start,
                to_name=goal,
                length=length,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[start].x, landmarks[start].y),
                    WorldPoint(landmarks[goal].x, landmarks[goal].y),
                ),
                properties={"direction": 1},
            )
        )
    manager = FleetManagerSim(landmarks, edges, params={"fleet": {}})
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="B",
        status="IDLE",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="C",
        vehicle="moving",
        # A forced-replan marker which does not itself block the forward A->B
        # edge. The occupied-node penalty must be what selects the long path.
        traffic_detour_edges=[("B", "A")],
    )

    route = manager._ensure_order_spatial_route(order, "A", "C")

    assert route == ["A", "D", "C"]


def test_committed_spatial_route_is_invalidated_when_robot_parks_on_it() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "D": Landmark(name="D", x=1.0, y=1.0),
    }
    edges = []
    for src, dst, length in (
        ("A", "B", 1.0),
        ("B", "C", 1.0),
        ("A", "D", 2.0),
        ("D", "C", 2.0),
    ):
        edges.append(GraphEdge(
            from_name=src,
            to_name=dst,
            length=length,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[src].x, landmarks[src].y),
                WorldPoint(landmarks[dst].x, landmarks[dst].y),
            ),
            properties={"direction": 1},
        ))
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={"fleet": {"congestion_tie_break_penalty_m": 0.0}},
    )
    manager.robots["moving"] = FleetRobot(
        name="moving",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="C",
        vehicle="moving",
        spatial_route_nodes=["A", "B", "C"],
    )
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
    )

    route = manager._ensure_order_spatial_route(order, "A", "C")

    assert route == ["A", "D", "C"]
    assert order.spatial_route_nodes == route


def test_stationary_robot_without_detour_keeps_order_waiting() -> None:
    manager = _manager()
    manager.robots["moving"] = FleetRobot(
        name="moving",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    request = {
        "name": "moving",
        "startLm": "A",
        "goalLm": "B",
        "startPose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }

    blocked = manager._plan_valid_requests(
        [request],
        {
            "robots": [request],
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
        },
    )

    assert not blocked["ok"]
    assert blocked["plans"] == []
    assert blocked["debug"]["stationaryRobotWait"]
    assert blocked["debug"]["softBlockedLms"] == ["B"]

    del manager.robots["parked"]
    released = manager._plan_valid_requests(
        [request],
        {
            "robots": [request],
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
        },
    )

    assert released["ok"]
    assert released["plans"][0]["nodes"] == ["A", "B"]


def test_stopped_robot_with_queued_assignment_still_blocks_mapf() -> None:
    manager = _manager()
    manager.robots["moving"] = FleetRobot(
        name="moving",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="B",
        status="STOPPED",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    manager.orders["parked-later"] = FleetOrder(
        order_id="parked-later",
        target_lm="A",
        vehicle="parked",
        assigned_robot="parked",
        status="QUEUED",
    )
    request = {
        "name": "moving",
        "startLm": "A",
        "goalLm": "B",
        "startPose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }

    assert manager._stationary_robot_blocked_lms(
        exclude_robot_names={"moving"},
    ) == {"B"}
    result = manager._plan_valid_requests(
        [request],
        {
            "robots": [request],
            "strictStationaryRobotAvoidance": True,
        },
    )

    assert not result["ok"]
    assert result["debug"]["stationaryRobotWait"]
    assert result["debug"]["softBlockedLms"] == ["B"]


def test_queued_idle_robots_invalidate_cached_route_and_force_free_detour() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "G": Landmark(name="G", x=3.0, y=0.0),
        "D": Landmark(name="D", x=0.0, y=1.0),
        "E": Landmark(name="E", x=1.0, y=1.0),
        "F": Landmark(name="F", x=2.0, y=1.0),
    }
    edges = []
    for src, dst in (
        ("A", "B"),
        ("B", "C"),
        ("C", "G"),
        ("A", "D"),
        ("D", "E"),
        ("E", "F"),
        ("F", "G"),
    ):
        edges.append(
            GraphEdge(
                from_name=src,
                to_name=dst,
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[src].x, landmarks[src].y),
                    WorldPoint(landmarks[dst].x, landmarks[dst].y),
                ),
                properties={"direction": 1},
            )
        )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={"fleet": {"congestion_tie_break_penalty_m": 0.0}},
    )
    manager.robots = {
        "moving": FleetRobot(
            name="moving",
            current_lm="A",
            status="IDLE",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        ),
        "parked-1": FleetRobot(
            name="parked-1",
            current_lm="B",
            status="ARRIVED",
            pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
            has_executed_route=True,
        ),
        "parked-2": FleetRobot(
            name="parked-2",
            current_lm="C",
            status="IDLE",
            pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
            has_executed_route=True,
        ),
    }
    manager.orders = {
        "later-1": FleetOrder(
            order_id="later-1",
            target_lm="D",
            vehicle="parked-1",
            status="QUEUED",
        ),
        "later-2": FleetOrder(
            order_id="later-2",
            target_lm="E",
            vehicle="parked-2",
            status="PLANNING",
        ),
    }
    moving_order = FleetOrder(
        order_id="moving-now",
        target_lm="G",
        vehicle="moving",
        spatial_route_nodes=["A", "B", "C", "G"],
    )

    assert manager._stationary_robot_blocked_lms(
        exclude_robot_names={"moving"},
    ) == {"B", "C"}

    route = manager._ensure_order_spatial_route(moving_order, "A", "G")

    assert route == ["A", "D", "E", "F", "G"]
    assert moving_order.spatial_route_nodes == route
    assert not {"B", "C"}.intersection(route)


def test_new_spatial_route_avoids_other_committed_route_demand() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=0.0, y=1.0),
        "D": Landmark(name="D", x=1.0, y=1.0),
    }
    edges = []
    for src, dst in (("A", "B"), ("B", "D"), ("A", "C"), ("C", "D")):
        edges.append(
            GraphEdge(
                from_name=src,
                to_name=dst,
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[src].x, landmarks[src].y),
                    WorldPoint(landmarks[dst].x, landmarks[dst].y),
                ),
                properties={"direction": 1},
            )
        )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "congestion_routing_enabled": True,
                "congestion_edge_load_penalty_m": 2.0,
                "congestion_opposing_load_penalty_m": 2.0,
                "congestion_node_load_penalty_m": 0.5,
                "congestion_tie_break_penalty_m": 0.0,
                "traffic_zone_control_enabled": False,
            }
        },
    )
    manager.robots["existing"] = FleetRobot(
        name="existing",
        current_lm="A",
        status="MOVING",
        active_order_id="old",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
            {"t": 2.0, "x": 1.0, "y": 1.0, "yaw": math.pi / 2.0, "edgeId": "B->D", "lm": "D"},
        ],
    )
    manager.orders["old"] = FleetOrder(
        order_id="old",
        target_lm="D",
        vehicle="existing",
        assigned_robot="existing",
        status="EXECUTING",
        spatial_route_nodes=["A", "B", "D"],
    )
    new_order = FleetOrder(order_id="new", target_lm="D", vehicle="new")

    route = manager._ensure_order_spatial_route(new_order, "A", "D")

    assert route == ["A", "C", "D"]
    revision = new_order.spatial_route_revision
    assert manager._ensure_order_spatial_route(new_order, "A", "D") == route
    assert new_order.spatial_route_revision == revision


def test_parked_robot_uses_short_replan_timeout_but_moving_robot_does_not() -> None:
    manager = _manager()
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots["moving"] = FleetRobot(
        name="moving",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
        ],
    )

    assert manager._blocked_replan_after("keep clearance from parked") == 1.0
    assert manager._blocked_replan_after("yield to moving") == 3.0
    assert manager._deadlock_coupled_replan_after() == 1.5
    assert manager._deadlock_retreat_after() == 4.5


def test_parked_detour_blocks_the_conflicted_segment_not_single_exit() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "X": Landmark(name="X", x=2.0, y=1.0),
        "D": Landmark(name="D", x=3.0, y=0.0),
    }
    edges = []
    for src, dst in (
        ("A", "B"),
        ("B", "C"),
        ("C", "D"),
        ("B", "X"),
        ("X", "D"),
    ):
        edges.append(GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[src].x, landmarks[src].y),
                WorldPoint(landmarks[dst].x, landmarks[dst].y),
            ),
            properties={"direction": 1},
        ))
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={"fleet": {"congestion_routing_enabled": True}},
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="D",
        vehicle="moving",
        spatial_route_nodes=["A", "B", "C", "D"],
    )

    queued = manager._queue_alternate_corridor_detour(
        order,
        "A",
        "D",
        avoid_lm="C",
    )

    assert queued
    assert set(order.traffic_detour_edges) == {("B", "C"), ("C", "B")}


def test_zone_admission_releases_compatible_batch_and_prevents_starvation() -> None:
    landmarks = {
        "WEST": Landmark(name="WEST", x=0.0, y=7.0),
        "NORTH": Landmark(name="NORTH", x=7.0, y=0.0),
        "HUB": Landmark(name="HUB", x=7.0, y=7.0),
    }
    edges = []
    for src in ("WEST", "NORTH"):
        edges.append(
            GraphEdge(
                from_name=src,
                to_name="HUB",
                length=7.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[src].x, landmarks[src].y),
                    WorldPoint(landmarks["HUB"].x, landmarks["HUB"].y),
                ),
                properties={"direction": 1},
            )
        )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "traffic_zone_control_enabled": True,
                "traffic_zone_size_m": 6.0,
                "traffic_zone_demand_threshold": 2,
                "traffic_zone_capacity": 1,
                "traffic_zone_batch_size": 1,
                "traffic_zone_phase_sec": 30.0,
                "traffic_zone_admission_lease_sec": 4.0,
                "traffic_zone_starvation_sec": 8.0,
            }
        },
    )
    for name, start, priority in (("r1", "WEST", 1), ("r2", "NORTH", 5)):
        pose = {"x": landmarks[start].x, "y": landmarks[start].y, "yaw": 0.0}
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=start,
            target_lm="HUB",
            status="MOVING",
            active_order_id=f"o-{name}",
            pose=pose,
            trajectory=[
                {"t": 0.0, **pose, "edgeId": f"{start}->HUB", "lm": start},
                {
                    "t": 7.0,
                    "x": landmarks["HUB"].x,
                    "y": landmarks["HUB"].y,
                    "yaw": 0.0,
                    "edgeId": f"{start}->HUB",
                    "lm": "HUB",
                },
            ],
        )
        manager.orders[f"o-{name}"] = FleetOrder(
            order_id=f"o-{name}",
            target_lm="HUB",
            vehicle=name,
            assigned_robot=name,
            status="EXECUTING",
            priority=priority,
            spatial_route_nodes=[start, "HUB"],
        )
    now = 1_000.0

    manager._prepare_traffic_zone_admissions(now)

    target_zone = manager._traffic_zone_by_lm["HUB"]
    assert manager._traffic_zone_winners == {"r2": target_zone}
    assert manager._traffic_zone_admission_reason(manager.robots["r2"], 0.1) == ""
    assert manager._traffic_zone_admission_reason(
        manager.robots["r1"],
        0.1,
    ).startswith("traffic admission wait at WEST")
    assert not manager._should_replan_for_blocked_reason(
        "traffic admission wait at WEST for flow:1:1"
    )

    manager._traffic_zone_leases.clear()
    manager._traffic_zone_wait_since[(target_zone, "r1")] = now - 9.0
    manager._traffic_zone_wait_since[(target_zone, "r2")] = now
    manager._prepare_traffic_zone_admissions(now)

    assert manager._traffic_zone_winners == {"r1": target_zone}
    assert manager._traffic_zone_phase[target_zone][0] == "E"

    # A robot already inside can keep nominal capacity at zero for a long
    # time. Starvation handling must still release one oldest entrant per
    # phase; exact collision checks remain responsible for physical safety.
    inside_pose = {
        "x": landmarks["HUB"].x,
        "y": landmarks["HUB"].y,
        "yaw": 0.0,
    }
    manager.robots["inside"] = FleetRobot(
        name="inside",
        current_lm="HUB",
        status="IDLE",
        pose=inside_pose,
    )
    manager._traffic_zone_leases.clear()
    manager._traffic_zone_wait_since[(target_zone, "r1")] = now - 20.0
    manager._traffic_zone_wait_since[(target_zone, "r2")] = now

    manager._prepare_traffic_zone_admissions(now + 1.0)

    assert manager._traffic_zone_occupancy[target_zone] >= 1
    assert manager._traffic_zone_winners == {"r1": target_zone}
    assert manager._traffic_zone_emergency_until[target_zone] > now + 1.0


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


def test_simulation_time_scale_accelerates_motion_and_rotation(monkeypatch) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    manager = _manager()
    assert manager.set_simulation_time_scale(4) == 4.0
    started_at = manager.simulation_time()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "t": 0.0,
                "edgeId": "WAIT@ROTATE:A",
                "lm": "A",
            },
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi / 2.0,
                "t": 2.0,
                "edgeId": "WAIT@ROTATE:A",
                "lm": "A",
            },
            {
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi / 2.0,
                "t": 10.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_started_at=started_at,
        last_tick_at=started_at,
        updated_at=started_at,
    )
    manager.robots[robot.name] = robot

    clock[0] += 0.1
    manager.advance_runtime()

    assert robot.route_clock == pytest.approx(0.4)
    assert robot.pose["yaw"] == pytest.approx(math.pi * 0.1)
    state = manager.snapshot()
    assert state["simulationTimeScale"] == 4.0
    assert state["simulationTimeScaleMax"] == 8.0


def test_simulation_time_scale_is_clamped_to_safe_range() -> None:
    manager = _manager()

    assert manager.set_simulation_time_scale(100) == 8.0
    assert manager.set_simulation_time_scale(0) == 1.0


def _manager() -> FleetManagerSim:
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
    return FleetManagerSim(
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


def _remote_manager() -> FleetManagerROS:
    simulation = _manager()
    return FleetManagerROS(
        simulation.landmarks,
        simulation.edges,
        params=simulation.params,
        remote_adapter=_RemoteControlAdapter(),
    )


def _long_line_manager(edge_count: int) -> FleetManagerSim:
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
    return FleetManagerSim(
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
