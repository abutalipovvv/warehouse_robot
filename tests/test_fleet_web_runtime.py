from __future__ import annotations

import math
from threading import Event
from time import perf_counter, sleep, time
from types import SimpleNamespace

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
        "corridorAdmissionWaits": 0,
        "corridorAdmissionsGranted": 0,
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
    first_grants = manager.traffic_metrics["priorityGrants"]

    # The unchanged 10 Hz wait snapshot must not be counted/re-granted on
    # every frame after the stale lease expires.  The next arbitration is
    # allowed once the bounded lease interval has elapsed.
    manager._break_runtime_wait_cycle(
        ["clear", "blocked"],
        robots,
        now + 0.1,
        new_episode=False,
    )
    assert manager.traffic_metrics["priorityGrants"] == first_grants

    manager._break_runtime_wait_cycle(
        ["clear", "blocked"],
        robots,
        now + manager._deadlock_priority_lease() + 0.01,
        new_episode=False,
    )
    assert manager.traffic_metrics["priorityGrants"] == first_grants + 1


def test_priority_nudge_does_not_reset_an_uncleared_wait_cycle(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    winner = FleetRobot(
        name="winner",
        current_lm="A",
        status="MOVING",
        pose={"x": 0.1, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
        route_clock=0.1,
    )
    peer = FleetRobot(
        name="peer",
        current_lm="B",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 1.0, "y": 0.0, "yaw": math.pi, "lm": "B"},
        ],
    )
    manager.robots = {winner.name: winner, peer.name: peer}
    cycle_key = ("peer", "winner")
    cycle_started = now - 3.0
    manager._active_wait_cycles[cycle_key] = cycle_started
    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda *_args: 0.2,
    )

    manager._record_traffic_progress(winner)

    assert manager._active_wait_cycles[cycle_key] == cycle_started
    assert winner.traffic_stall_since == cycle_started

    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda *_args: 2.0,
    )
    manager._record_traffic_progress(winner)

    assert cycle_key not in manager._active_wait_cycles


def test_active_priority_lease_keeps_temporarily_hidden_cycle_episode() -> None:
    manager = _manager()
    now = time()
    winner = FleetRobot(
        name="winner",
        current_lm="A",
        status="MOVING",
        traffic_priority_until=now + 1.0,
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
    )
    peer = FleetRobot(
        name="peer",
        current_lm="B",
        status="WAITING",
        last_reason="yield to winner",
        wait_for_robot="winner",
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "lm": "B"},
        ],
    )
    manager.robots = {winner.name: winner, peer.name: peer}
    cycle_key = ("peer", "winner")
    manager._active_wait_cycles[cycle_key] = now - 2.0

    manager._resolve_runtime_wait_cycles(now)

    assert cycle_key in manager._active_wait_cycles


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


def test_starvation_releases_the_corridor_blocker_before_its_follower(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    follower = FleetRobot(
        name="follower",
        current_lm="A",
        status="WAITING",
        last_reason="occupied by portal",
        trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0}],
    )
    portal = FleetRobot(
        name="portal",
        current_lm="B",
        status="WAITING",
        last_reason="corridor admission wait at A for region-1",
        trajectory=[{"t": 0.0, "x": 1.0, "y": 0.0, "yaw": 0.0}],
    )
    manager.robots = {follower.name: follower, portal.name: portal}
    monkeypatch.setattr(
        manager,
        "_transfer_controlled_corridor_lease",
        lambda candidate, _participants, _now: candidate.name == "portal",
    )

    assert manager._grant_starvation_priority(follower, portal, now)

    assert portal.status == "MOVING"
    assert portal.last_reason == "starvation priority active"
    assert follower.status == "WAITING"
    assert follower.last_reason == "yield to portal"


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


def test_wait_chain_active_terminal_lease_is_not_granted_again() -> None:
    manager = _manager()
    now = time()
    stale_since = now - 10.0
    terminal = FleetRobot(
        name="r3",
        current_lm="A",
        status="WAITING",
        last_reason="corridor admission wait at A for region-1",
        blocked_since=stale_since,
        traffic_stall_since=stale_since,
        traffic_priority_until=now + 1.0,
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
        ],
    )
    participants = [
        FleetRobot(
            name="r1",
            current_lm="A",
            status="WAITING",
            last_reason="yield to r2",
            blocked_since=stale_since,
            traffic_stall_since=stale_since,
            trajectory=[
                {"t": 0.0, "x": 0.5, "y": 0.0, "edgeId": "A->B", "lm": "A"},
            ],
        ),
        FleetRobot(
            name="r2",
            current_lm="A",
            status="WAITING",
            last_reason="yield to r3",
            blocked_since=stale_since,
            traffic_stall_since=stale_since,
            trajectory=[
                {"t": 0.0, "x": 1.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
            ],
        ),
        terminal,
    ]
    manager.robots = {robot.name: robot for robot in participants}
    grants_before = manager.traffic_metrics["priorityGrants"]

    assert manager._grant_wait_chain_priority(participants, terminal, now)

    assert terminal.traffic_priority_until > now
    assert manager.traffic_metrics["priorityGrants"] == grants_before


def test_wait_chain_behind_parked_terminal_evacuates_immediate_follower(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            status="WAITING",
            last_reason="occupied by r2",
            blocked_since=now - 9.0,
            traffic_stall_since=now - 9.0,
            active_order_id="o1",
            trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="B",
            status="WAITING",
            last_reason="occupied by r3",
            blocked_since=now - 9.0,
            traffic_stall_since=now - 9.0,
            active_order_id="o2",
            trajectory=[{"t": 0.0, "x": 1.0, "y": 0.0, "edgeId": "B->C", "lm": "B"}],
        ),
        "r3": FleetRobot(name="r3", current_lm="C", status="IDLE"),
    }
    evacuations: list[tuple[list[str], str]] = []
    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        lambda robots, winner, _now: (
            evacuations.append(([robot.name for robot in robots], winner.name))
            or "r2"
        ),
    )

    manager._resolve_runtime_wait_cycles(now)

    assert evacuations == [(["r3", "r2"], "r3")]
    assert manager.traffic_metrics["priorityGrants"] == 0


def test_stalled_individual_dependency_retreats_before_another_priority_grant(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        status="WAITING",
        last_reason="occupied by blocker",
        blocked_since=now - 9.0,
        traffic_stall_since=now - 9.0,
        active_order_id="o1",
        trajectory=[{"t": 0.0, "x": 0.5, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="MOVING",
        trajectory=[{"t": 0.0, "x": 1.5, "y": 0.0, "edgeId": "B->A", "lm": "B"}],
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    monkeypatch.setattr(manager, "_safe_replan_start_lm", lambda _robot: "")
    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        lambda robots, winner, _now: (
            "waiter"
            if [robot.name for robot in robots] == ["blocker", "waiter"]
            and winner.name == "blocker"
            else ""
        ),
    )

    manager._resolve_runtime_wait_cycles(now)

    assert manager.traffic_metrics["priorityGrants"] == 0


def test_wait_chain_releases_exhausted_terminal_before_upstream_followers(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    terminal_order = FleetOrder(
        order_id="o3",
        target_lm="B",
        vehicle="r3",
        assigned_robot="r3",
        status="PLANNING",
    )
    manager.orders[terminal_order.order_id] = terminal_order
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            status="WAITING",
            last_reason="yield to r2",
            wait_for_robot="r2",
            blocked_since=now - 10.0,
            trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "r2": FleetRobot(
            name="r2",
            current_lm="A",
            status="WAITING",
            last_reason="occupied by r3",
            wait_for_robot="r3",
            blocked_since=now - 10.0,
            trajectory=[{"t": 0.0, "x": 1.0, "y": 0.0, "edgeId": "A->B", "lm": "A"}],
        ),
        "r3": FleetRobot(
            name="r3",
            current_lm="A",
            target_lm="A",
            status="WAITING",
            active_order_id="o3",
            last_reason="rolling continuation pending",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->A", "lm": "A"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "edgeId": "A->A", "lm": "A"},
            ],
            route_clock=1.0,
            route_chunk_goal_lm="A",
            route_final_lm="B",
            has_executed_route=True,
        ),
    }
    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda *_args: pytest.fail(
            "acyclic chain priority must follow the terminal dependency"
        ),
    )

    manager._resolve_runtime_wait_cycles(now)

    terminal = manager.robots["r3"]
    assert terminal.status == "WAITING"
    assert terminal.last_reason == "rolling continuation pending"
    assert manager.robots["r1"].wait_for_robot == "r3"
    assert manager.robots["r2"].wait_for_robot == "r3"
    assert manager._release_blocker_names_for_requests(
        [{"name": "r3", "startLm": "A", "goalLm": "B"}]
    ) == {"r1", "r2"}
    assert [entry[1].name for entry in manager._ready_rolling_prefetch_entries()] == [
        "r3"
    ]
    assert manager.traffic_metrics["priorityGrants"] == 0

    manager._resolve_runtime_wait_cycles(now + 0.1)
    assert manager.traffic_metrics["priorityGrants"] == 0


def test_wait_chain_upstream_robot_is_reserved_as_stopped_body(
    monkeypatch,
) -> None:
    manager = _manager()
    held = FleetRobot(
        name="held",
        current_lm="B",
        target_lm="A",
        status="WAITING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "B",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "A",
            },
        ],
        last_reason="yield to terminal",
        wait_for_robot="terminal",
    )
    manager.robots[held.name] = held
    request = {
        "name": "terminal",
        "startLm": "A",
        "goalLm": "B",
        "startPose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }
    captured: list[dict] = []

    def capture_plan(payload):
        captured.append(payload)
        return {
            "ok": False,
            "plans": [],
            "debug": {"reason": "held blocker test"},
        }

    monkeypatch.setattr(manager.planner, "plan", capture_plan)

    manager._plan_valid_requests([request], {"robots": [request]})

    assert len(captured) == 1
    held_vertices = [
        interval
        for interval in captured[0]["reserved_vertex_intervals"]
        if interval["robot"] == held.name
    ]
    assert held_vertices == [{
        "node": "B",
        "start": 0.0,
        "end": manager._reservation_horizon(),
        "robot": held.name,
    }]
    assert all(
        interval["robot"] != held.name
        for interval in captured[0]["reserved_edge_intervals"]
    )


def test_failed_stationary_departure_evacuates_incoming_head_on_robot() -> None:
    manager = _manager()
    now = time()
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="B",
        vehicle="waiter",
        assigned_robot="waiter",
        status="WAITING_TRAFFIC",
    )
    blocker_order = FleetOrder(
        order_id="block-order",
        target_lm="A",
        vehicle="blocker",
        assigned_robot="blocker",
        status="QUEUED",
        dispatch_failures=2,
    )
    manager.orders = {
        waiter_order.order_id: waiter_order,
        blocker_order.order_id: blocker_order,
    }
    manager.robots = {
        "waiter": FleetRobot(
            name="waiter",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="wait-order",
            last_reason="occupied by blocker",
            wait_for_robot="blocker",
            blocked_since=now - 10.0,
            traffic_stall_since=now - 10.0,
            pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
                {"t": 2.0, "x": 2.0, "y": 0.0, "edgeId": "A->B", "lm": "B"},
            ],
            route_clock=1.0,
        ),
        "blocker": FleetRobot(
            name="blocker",
            current_lm="B",
            status="IDLE",
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        ),
    }

    manager._resolve_runtime_wait_cycles(now)

    waiter = manager.robots["waiter"]
    assert waiter.status == "RETREATING"
    assert waiter.retreat_target_lm == "A"
    assert waiter.traffic_priority_until > now
    assert any(
        "evacuating for queued departure blocker" in event.message
        for event in manager.events
    )


def test_failed_rolling_boundary_departure_uses_prefetch_failure_counter() -> None:
    manager = _manager()
    now = time()
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="B",
        vehicle="waiter",
        assigned_robot="waiter",
        status="WAITING_TRAFFIC",
    )
    blocker_order = FleetOrder(
        order_id="block-order",
        target_lm="A",
        vehicle="blocker",
        assigned_robot="blocker",
        status="PLANNING",
        dispatch_failures=0,
    )
    manager.orders = {
        waiter_order.order_id: waiter_order,
        blocker_order.order_id: blocker_order,
    }
    manager.robots = {
        "waiter": FleetRobot(
            name="waiter",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="wait-order",
            last_reason="occupied by blocker",
            wait_for_robot="blocker",
            blocked_since=now - 10.0,
            traffic_stall_since=now - 10.0,
            pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
                {"t": 2.0, "x": 2.0, "y": 0.0, "edgeId": "A->B", "lm": "B"},
            ],
            route_clock=1.0,
        ),
        "blocker": FleetRobot(
            name="blocker",
            current_lm="B",
            target_lm="B",
            status="WAITING",
            active_order_id="block-order",
            last_reason="rolling continuation pending",
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
                {"t": 1.0, "x": 2.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
            ],
            route_clock=1.0,
            route_chunk_goal_lm="B",
            route_final_lm="A",
            has_executed_route=True,
        ),
    }
    manager._rolling_prefetch_failures["blocker"] = 2

    manager._resolve_runtime_wait_cycles(now)

    waiter = manager.robots["waiter"]
    assert blocker_order.dispatch_failures == 0
    assert waiter.status == "RETREATING"
    assert waiter.retreat_target_lm == "A"
    assert any(
        "evacuating for queued departure blocker" in event.message
        for event in manager.events
    )


@pytest.mark.parametrize(
    "failure",
    (
        "cbs_resource_conflict:r1",
        "edge_resource_constrained:A->B@1",
    ),
)
def test_resource_failure_enters_bounded_traffic_detour(
    monkeypatch,
    failure: str,
) -> None:
    manager = _manager()
    clock = [100.0]
    monkeypatch.setattr(manager, "_now", lambda: clock[0])
    detours: list[tuple[str, str]] = []
    monkeypatch.setattr(
        manager,
        "_queue_alternate_corridor_detour",
        lambda _order, start, goal: detours.append((start, goal)) or True,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        start_lm="A",
        spatial_route_nodes=["A", "B"],
    )

    manager._set_order_error(order, failure)
    assert order.traffic_blocked_since == pytest.approx(100.0)
    clock[0] += manager._traffic_replan_after() + 0.1
    manager._set_order_error(order, failure)

    assert detours == [("A", "B")]
    assert order.spatial_route_nodes == []


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


def test_previous_trajectory_lm_keeps_mid_edge_retreat_anchor() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        trajectory=[
            {"t": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_clock=1.0,
    )

    assert manager._previous_trajectory_lm(robot) == (0.0, "A")


def test_previous_trajectory_lm_skips_duplicate_exhausted_endpoint() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="B",
        trajectory=[
            {"t": 0.0, "edgeId": "A->A", "lm": "A"},
            {"t": 0.5, "edgeId": "WAIT@ROTATE:A->B", "lm": "A"},
            {"t": 2.0, "edgeId": "A->B", "lm": "B"},
            {"t": 3.0, "edgeId": "B->B", "lm": "B"},
        ],
        route_clock=3.0,
    )

    assert manager._previous_trajectory_lm(robot) == (0.5, "A")


def test_exhausted_corridor_trajectory_starts_physical_retreat(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    winner = FleetRobot(name="winner", current_lm="A", status="WAITING")
    retreater = FleetRobot(
        name="retreater",
        current_lm="B",
        target_lm="A",
        status="WAITING",
        active_order_id="o1",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_clock=2.0,
    )
    manager.robots = {
        winner.name: winner,
        retreater.name: retreater,
    }
    manager.orders = {
        "o1": FleetOrder(
            order_id="o1",
            target_lm="A",
            vehicle=retreater.name,
            status="EXECUTING",
        ),
    }
    monkeypatch.setattr(
        manager,
        "_queue_active_order_for_background_replan",
        lambda *_args, **_kwargs: pytest.fail(
            "an exhausted endpoint must reverse before replanning"
        ),
    )

    evacuated = manager._start_deadlock_corridor_evacuation(
        [winner, retreater],
        winner,
        now,
    )

    assert evacuated == retreater.name
    assert retreater.status == "RETREATING"
    assert retreater.retreat_target_clock == pytest.approx(0.0)
    assert retreater.retreat_target_lm == "A"


def test_repeated_current_lm_tag_queues_detour_instead_of_noop_retreat() -> None:
    manager = _manager()
    now = time()
    manager._controlled_corridor_graph = SimpleNamespace(
        vertices={
            "A": SimpleNamespace(controlled_region_ids=("corridor-A-B",)),
        },
    )
    winner = FleetRobot(name="winner", current_lm="B", status="WAITING")
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
        pose={"x": 0.05, "y": 0.0, "yaw": 0.0},
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
                "t": 1.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "WAIT@ROTATE:A->B",
                "lm": "A",
            },
            {
                "t": 3.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_clock=1.05,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="EXECUTING",
    )
    manager.robots = {winner.name: winner, robot.name: robot}
    manager.orders = {order.order_id: order}

    evacuated = manager._start_deadlock_corridor_evacuation(
        [winner, robot],
        winner,
        now,
    )

    assert evacuated == robot.name
    assert order.status == "QUEUED"
    assert order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["cycleReplans"] == 1
    assert robot.status == "IDLE"
    assert robot.active_order_id == ""
    assert robot.trajectory == []
    assert robot.retreat_target_clock is None
    assert (
        manager._start_deadlock_corridor_evacuation(
            [winner, robot],
            winner,
            now + 0.1,
        )
        == ""
    )
    assert order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["cycleReplans"] == 1


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


def test_safety_yield_pauses_a_retreat_until_winner_lease_expires(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        last_reason="yield to r2",
        wait_for_robot="r2",
        wait_resource="runtime_safety",
        wait_release_at=12.0,
        pose={"x": 0.5, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
        route_clock=0.5,
        retreat_target_clock=0.0,
        retreat_target_lm="A",
        last_tick_at=10.0,
    )
    manager.robots[robot.name] = robot
    monkeypatch.setattr(
        manager,
        "_blocked_at_clock",
        lambda *_args, **_kwargs: pytest.fail("paused retreat must not move"),
    )

    manager._advance_deadlock_retreat(robot, 11.0)

    assert robot.route_clock == pytest.approx(0.5)
    assert robot.last_tick_at == pytest.approx(11.0)


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


def test_full_horizon_continuous_wait_remains_unresolved(
    monkeypatch,
) -> None:
    manager = _manager()
    trajectory = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
        {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
    ]
    conflict = {"time": 1.0, "other": "r2", "edge": "A->B"}

    def conflict_until_hidden_by_wait(
        _robot_name: str,
        candidate: list[dict[str, object]],
        **_kwargs: object,
    ) -> dict[str, object] | None:
        if any(
            str(sample.get("edgeId", "")).startswith("WAIT@")
            for sample in candidate
        ):
            return None
        return conflict

    monkeypatch.setattr(
        manager,
        "_first_continuous_corridor_conflict",
        conflict_until_hidden_by_wait,
    )
    monkeypatch.setattr(manager, "_reservation_horizon", lambda: 5.0)
    monkeypatch.setattr(
        manager,
        "_wait_duration_for_conflict",
        lambda *args, **kwargs: 5.0,
    )

    scheduled, stats = manager._schedule_trajectory_against_corridors(
        "r1",
        trajectory,
    )

    assert not any(
        str(sample.get("edgeId", "")).startswith("WAIT@")
        for sample in scheduled
    )
    assert stats == {"conflicts": 1, "waits": 0, "wait": 0.0, "unresolved": 1}


def test_batch_conflict_keeps_request_priority_when_peer_already_waits(
    monkeypatch,
) -> None:
    manager = _manager()
    plans = [
        {
            "robot": "priority",
            "trajectory": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
                {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            ],
        },
        {
            "robot": "waiting",
            "trajectory": [
                {
                    "t": 0.0,
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "edgeId": "WAIT@ROTATE:A",
                },
                {
                    "t": 1.0,
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 1.0,
                    "edgeId": "WAIT@ROTATE:A",
                },
            ],
        },
    ]
    monkeypatch.setattr(
        manager.collision,
        "robot_footprints_conflict",
        lambda *_args: True,
    )
    monkeypatch.setattr(
        manager.collision,
        "footprints_overlap",
        lambda *_args: True,
    )

    conflict = manager._first_batch_trajectory_conflict(plans)

    assert conflict is not None
    assert conflict["priorityIndex"] == 0
    assert conflict["waitIndex"] == 1


def test_batch_no_clear_peer_wait_is_rejected_without_mutating_plans(
    monkeypatch,
) -> None:
    manager = _manager()
    plans = [
        {
            "robot": "priority",
            "trajectory": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
                {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            ],
        },
        {
            "robot": "waiting",
            "trajectory": [
                {"t": 0.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A"},
            ],
        },
    ]
    conflict = {
        "time": 0.5,
        "priorityIndex": 0,
        "waitIndex": 1,
        "edge": "B->A",
    }
    monkeypatch.setattr(
        manager,
        "_first_batch_trajectory_conflict",
        lambda *_args: conflict,
    )
    monkeypatch.setattr(manager, "_reservation_horizon", lambda: 5.0)
    monkeypatch.setattr(
        manager,
        "_wait_duration_for_peer_conflict",
        lambda *_args: 5.0,
    )

    stats = manager._schedule_batch_trajectories(plans)

    assert stats == {
        "conflicts": 0,
        "waits": 0,
        "wait": 0.0,
        "unresolved": 1,
    }
    assert all(
        not str(sample.get("edgeId", "")).startswith("WAIT@")
        for plan in plans
        for sample in plan["trajectory"]
    )


def test_batch_same_dependency_stops_after_one_nonprogress_wait(
    monkeypatch,
) -> None:
    manager = _manager()
    plans = [
        {
            "robot": "priority",
            "trajectory": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
                {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            ],
        },
        {
            "robot": "waiting",
            "trajectory": [
                {"t": 0.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A"},
            ],
        },
    ]
    conflict = {
        "time": 0.5,
        "priorityIndex": 0,
        "waitIndex": 1,
        "edge": "B->A",
    }
    monkeypatch.setattr(
        manager,
        "_first_batch_trajectory_conflict",
        lambda *_args: conflict,
    )
    monkeypatch.setattr(manager, "_reservation_horizon", lambda: 5.0)
    monkeypatch.setattr(
        manager,
        "_wait_duration_for_peer_conflict",
        lambda *_args: 1.0,
    )

    stats = manager._schedule_batch_trajectories(plans)

    assert stats == {
        "conflicts": 1,
        "waits": 1,
        "wait": 1.0,
        "unresolved": 1,
    }
    assert not any(
        str(sample.get("edgeId", "")).startswith("WAIT@")
        for sample in plans[0]["trajectory"]
    )
    assert sum(
        str(sample.get("edgeId", "")).startswith("WAIT@")
        for sample in plans[1]["trajectory"]
    ) == 1


def test_batch_total_wait_is_bounded_by_reservation_horizon(
    monkeypatch,
) -> None:
    manager = _manager()
    plans = [
        {
            "robot": "priority",
            "trajectory": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
                {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            ],
        },
        {
            "robot": "waiting",
            "trajectory": [
                {"t": 0.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->A"},
            ],
        },
    ]
    conflicts = iter(
        [
            {
                "time": 0.25,
                "priorityIndex": 0,
                "waitIndex": 1,
                "edge": "shared:first",
            },
            {
                "time": 0.75,
                "priorityIndex": 0,
                "waitIndex": 1,
                "edge": "shared:second",
            },
            {
                "time": 0.75,
                "priorityIndex": 0,
                "waitIndex": 1,
                "edge": "shared:second",
            },
        ]
    )
    monkeypatch.setattr(
        manager,
        "_first_batch_trajectory_conflict",
        lambda *_args: next(conflicts),
    )
    monkeypatch.setattr(manager, "_reservation_horizon", lambda: 5.0)
    monkeypatch.setattr(
        manager,
        "_wait_duration_for_peer_conflict",
        lambda *_args: 3.0,
    )

    stats = manager._schedule_batch_trajectories(plans)

    assert stats == {
        "conflicts": 1,
        "waits": 1,
        "wait": 3.0,
        "unresolved": 1,
    }
    assert float(plans[1]["arrivalTime"]) == pytest.approx(4.0)


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
    loser = first if first.status == "WAITING" else second
    winner = second if loser is first else first
    assert loser.wait_for_robot == winner.name
    assert loser.wait_resource == "runtime_safety"
    assert loser.wait_release_at == winner.traffic_priority_until
    assert first.active_order_id == "o1"
    assert second.active_order_id == "o2"
    assert manager.orders["o1"].status != "COMPLETED"
    assert manager.orders["o2"].status != "COMPLETED"


def test_runtime_safety_immediately_evacuates_from_stationary_blocker(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    blocker = FleetRobot(
        name="blocker",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    mover = FleetRobot(
        name="mover",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        active_order_id="mover-order",
        pose={"x": 1.2, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 1.2, "y": 0.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
    )
    manager.robots = {blocker.name: blocker, mover.name: mover}
    manager.orders["mover-order"] = FleetOrder(
        order_id="mover-order",
        target_lm="A",
        vehicle=mover.name,
        assigned_robot=mover.name,
        status="EXECUTING",
    )
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    mover.pose = {"x": 0.2, "y": 0.0, "yaw": math.pi}
    calls: list[tuple[list[str], str]] = []

    def evacuate(robots, winner, _now):
        calls.append(([robot.name for robot in robots], winner.name))
        mover.status = "RETREATING"
        mover.retreat_target_clock = 0.0
        return mover.name

    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        evacuate,
    )

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert calls == [(["blocker", "mover"], "blocker")]
    assert mover.status == "RETREATING"
    assert mover.traffic_priority_until > now
    assert blocker.traffic_priority_until == 0.0
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 1


def test_runtime_safety_replans_a_retreat_blocked_by_another_body(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    blocker = FleetRobot(
        name="blocker",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    mover = FleetRobot(
        name="mover",
        current_lm="B",
        target_lm="A",
        status="RETREATING",
        active_order_id="mover-order",
        pose={"x": 1.2, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 1.2, "y": 0.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
        retreat_target_clock=0.0,
        retreat_target_lm="B",
        retreat_blocked_edges=[("B", "A")],
    )
    order = FleetOrder(
        order_id="mover-order",
        target_lm="A",
        vehicle=mover.name,
        assigned_robot=mover.name,
        status="EXECUTING",
    )
    manager.robots = {blocker.name: blocker, mover.name: mover}
    manager.orders = {order.order_id: order}
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    mover.pose = {"x": 0.2, "y": 0.0, "yaw": math.pi}
    queued: list[tuple[str, str, bool]] = []

    def queue_replan(robot, _now, reason, **kwargs):
        queued.append(
            (
                robot.name,
                reason,
                bool(kwargs.get("allow_controlled_corridor_replan")),
            )
        )
        robot.status = "IDLE"
        robot.active_order_id = ""
        robot.trajectory = []
        return True

    monkeypatch.setattr(
        manager,
        "_queue_active_order_for_background_replan",
        queue_replan,
    )
    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        lambda *_args, **_kwargs: pytest.fail(
            "a failed retreat must not be armed a second time"
        ),
    )

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert queued == [
        (
            "mover",
            "deadlock retreat blocked; alternate route required",
            True,
        )
    ]
    assert mover.status == "IDLE"
    assert mover.trajectory == []
    assert mover.retreat_target_clock is None
    assert order.traffic_detour_edges == [("B", "A")]
    assert order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 1
    assert manager.traffic_metrics["cycleReplans"] == 1


def test_runtime_safety_holds_a_mid_edge_retreat_until_blocker_clears(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    blocker = FleetRobot(
        name="blocker",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    mover = FleetRobot(
        name="mover",
        current_lm="B",
        target_lm="A",
        status="RETREATING",
        active_order_id="mover-order",
        pose={"x": 1.2, "y": 0.1, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 1.2, "y": 0.1, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
        retreat_target_clock=0.0,
        retreat_target_lm="B",
        retreat_blocked_edges=[("B", "A")],
    )
    order = FleetOrder(
        order_id="mover-order",
        target_lm="A",
        vehicle=mover.name,
        assigned_robot=mover.name,
        status="EXECUTING",
    )
    manager.robots = {blocker.name: blocker, mover.name: mover}
    manager.orders = {order.order_id: order}
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    mover.pose = {"x": 0.2, "y": 0.0, "yaw": math.pi}
    monkeypatch.setattr(
        manager,
        "_queue_active_order_for_background_replan",
        lambda *_args, **_kwargs: False,
    )
    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        lambda *_args, **_kwargs: pytest.fail(
            "a blocked mid-edge retreat must be held, not armed again"
        ),
    )

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert mover.status == "WAITING"
    assert mover.pose == snapshots["mover"]["pose"]
    assert mover.retreat_target_clock == 0.0
    assert mover.wait_for_robot == "blocker"
    assert mover.wait_resource == "blocked_retreat"
    assert mover.wait_release_at >= now + 1.0
    assert blocker.traffic_priority_until == mover.wait_release_at
    assert order.traffic_detour_edges == [("B", "A")]
    assert order.traffic_detour_attempts == 0
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 1


def test_completed_retreat_preserves_safe_trajectory_yaw_at_landmark(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params.setdefault("robot_model", {})["footprint"] = [
        {"x": -0.523, "y": -0.3532},
        {"x": 0.477, "y": -0.3532},
        {"x": 0.477, "y": 0.3468},
        {"x": -0.523, "y": 0.3468},
    ]
    manager.collision.set_params(manager.params)
    now = time()
    arrival_pose = {
        "x": 2.0,
        "y": 0.0,
        "yaw": -math.pi / 2.0,
    }
    blocker = FleetRobot(
        name="blocker",
        current_lm="A",
        status="IDLE",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
    )
    retreater = FleetRobot(
        name="retreater",
        current_lm="B",
        status="RETREATING",
        pose={"x": 2.0, "y": 0.1, "yaw": -math.pi / 2.0},
        trajectory=[
            {
                "t": 0.0,
                **arrival_pose,
                "edgeId": "A->B",
                "lm": "B",
            },
            {
                "t": 1.0,
                "x": 2.0,
                "y": 1.0,
                "yaw": -math.pi / 2.0,
                "edgeId": "A->B",
            },
        ],
        route_clock=0.1,
        last_tick_at=now - 0.2,
        retreat_target_clock=0.0,
        retreat_target_lm="B",
        retreat_blocked_edges=[("A", "B")],
    )
    manager.robots = {
        blocker.name: blocker,
        retreater.name: retreater,
    }
    synthetic_lm_pose = manager._pose_at_landmark("B")
    assert synthetic_lm_pose is not None
    assert not manager.collision.footprints_overlap(
        arrival_pose,
        blocker.pose,
    )
    assert manager.collision.footprints_overlap(
        synthetic_lm_pose,
        blocker.pose,
    )
    monkeypatch.setattr(
        manager,
        "_blocked_at_clock",
        lambda *_args, **_kwargs: "",
    )
    monkeypatch.setattr(
        manager,
        "_queue_active_order_for_background_replan",
        lambda *_args, **_kwargs: False,
    )

    manager._advance_deadlock_retreat(retreater, now)

    assert retreater.route_clock == pytest.approx(0.0)
    assert retreater.current_lm == "B"
    assert retreater.pose == pytest.approx(arrival_pose)
    assert not manager.collision.footprints_overlap(
        retreater.pose,
        blocker.pose,
    )


def test_runtime_safety_resolves_independent_pair_while_holding_retreat(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    blocker = FleetRobot(
        name="blocker",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    retreater = FleetRobot(
        name="retreater",
        current_lm="B",
        status="RETREATING",
        active_order_id="retreat-order",
        pose={"x": 1.2, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 1.2, "y": 0.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
        retreat_target_clock=0.0,
        retreat_target_lm="B",
        retreat_blocked_edges=[("B", "A")],
        blocked_since=now - 10.0,
    )
    first = FleetRobot(
        name="first",
        current_lm="A",
        status="MOVING",
        pose={"x": 0.0, "y": 10.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 10.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 1.2, "y": 10.0, "yaw": 0.0, "lm": "B"},
        ],
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        status="MOVING",
        pose={"x": 1.2, "y": 10.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 1.2, "y": 10.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 10.0, "yaw": math.pi, "lm": "A"},
        ],
    )
    manager.robots = {
        robot.name: robot
        for robot in (blocker, retreater, first, second)
    }
    manager.orders = {
        "retreat-order": FleetOrder(
            order_id="retreat-order",
            target_lm="A",
            vehicle=retreater.name,
            assigned_robot=retreater.name,
            status="EXECUTING",
        ),
    }
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    retreater.pose = {"x": 0.2, "y": 0.0, "yaw": math.pi}
    first.pose = {"x": 0.6, "y": 10.0, "yaw": 0.0}
    second.pose = {"x": 0.6, "y": 10.0, "yaw": math.pi}
    monkeypatch.setattr(
        manager,
        "_queue_active_order_for_background_replan",
        lambda *_args, **_kwargs: False,
    )

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert retreater.status == "WAITING"
    assert retreater.wait_resource == "blocked_retreat"
    assert retreater.wait_for_robot == blocker.name
    assert retreater.pose == snapshots[retreater.name]["pose"]
    assert {first.status, second.status} == {"MOVING", "WAITING"}
    pair_loser = first if first.status == "WAITING" else second
    pair_winner = second if pair_loser is first else first
    assert pair_loser.wait_for_robot == pair_winner.name
    assert pair_loser.wait_resource == "runtime_safety"
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 1

    monkeypatch.setattr(
        manager,
        "_schedule_runtime_replan",
        lambda *_args, **_kwargs: pytest.fail(
            "an active blocked-retreat lease must not be replanned"
        ),
    )
    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        lambda *_args, **_kwargs: pytest.fail(
            "an active blocked-retreat lease must not be re-armed"
        ),
    )
    manager._resolve_runtime_wait_cycles(now)
    assert retreater.status == "WAITING"
    assert retreater.retreat_target_clock == pytest.approx(0.0)


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
    assert robot.updated_at == pytest.approx(clock[0])


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
    manager.robots["r1"].retreat_target_clock = 0.0
    manager.robots["r1"].retreat_target_lm = "A"
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    # Simulate the retreat completing and clearing its marker before the
    # swept-footprint invariant rolls the route state back.
    manager.robots["r1"].retreat_target_clock = None
    manager.robots["r1"].retreat_target_lm = ""
    manager.robots["r1"].pose = {"x": 1.2, "y": 0.0, "yaw": 0.0}
    manager.robots["r2"].pose = {"x": 0.0, "y": 0.0, "yaw": math.pi}

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert manager.robots["r1"].pose["x"] == 0.0
    assert manager.robots["r2"].pose["x"] == 1.2
    assert manager.robots["r1"].status == "WAITING"
    assert manager.robots["r1"].retreat_target_clock == pytest.approx(0.0)
    assert manager.robots["r1"].retreat_target_lm == "A"
    assert manager.robots["r2"].status == "MOVING"
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
    assert manager.orders["o1"].error.startswith("runtime replan cooldown:")
    assert manager.orders["o1"].dispatch_failures == 1
    assert robot.active_order_id == ""
    assert robot.status == "IDLE"
    assert robot.target_lm == ""
    assert robot.last_reason == "route replan queued"


def test_active_corridor_order_never_falls_back_to_synchronous_replan(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
        last_reason="corridor admission wait at A for corridor-A-B",
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
    )
    manager.robots[robot.name] = robot
    manager.orders["o1"] = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
    )
    monkeypatch.setattr(
        manager,
        "_queue_active_order_for_background_replan",
        lambda *_args, **_kwargs: False,
    )
    monkeypatch.setattr(
        manager,
        "_maybe_replan_robot",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("synchronous MAPF")
        ),
    )

    assert not manager._schedule_runtime_replan(
        robot,
        now,
        robot.last_reason,
    )
    assert robot.last_reason.startswith("corridor admission wait")
    assert robot.last_replan_at == now


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


def test_failed_urgent_prefetch_yields_planner_slot_to_stationary_recovery(
    monkeypatch,
) -> None:
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
    recovery_entry = (order, robot, request, "B")
    prefetch_robot = FleetRobot(name="rolling", current_lm="A")
    prefetch_entry = (order, prefetch_robot, request, "B", 0.0)
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    manager._rolling_prefetch_failures[prefetch_robot.name] = 1
    calls: list[str] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda orders: [recovery_entry],
    )
    monkeypatch.setattr(
        manager,
        "_ready_rolling_prefetch_entries",
        lambda: [prefetch_entry],
    )
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda entries: calls.append("recovery"),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda entries: calls.append("prefetch"),
    )

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["recovery"]
    calls.clear()
    manager._last_async_job_kind = "dispatch"

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["prefetch"]


def test_successful_prefetch_yields_next_planner_turn_to_ready_dispatch(
    monkeypatch,
) -> None:
    manager = _manager()
    queued_robot = FleetRobot(
        name="queued",
        current_lm="A",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    queued_order = FleetOrder(
        order_id="queued-order",
        target_lm="B",
        vehicle=queued_robot.name,
        assigned_robot=queued_robot.name,
        status="QUEUED",
    )
    queued_request = {"name": queued_robot.name, "startLm": "A", "goalLm": "B"}
    queued_entry = (
        queued_order,
        queued_robot,
        queued_request,
        "B",
    )
    rolling_robot = FleetRobot(name="rolling", current_lm="A")
    rolling_order = FleetOrder(
        order_id="rolling-order",
        target_lm="B",
        vehicle=rolling_robot.name,
        assigned_robot=rolling_robot.name,
        status="PLANNING",
    )
    rolling_request = {
        "name": rolling_robot.name,
        "startLm": "A",
        "goalLm": "B",
    }
    urgent_prefetch = (
        rolling_order,
        rolling_robot,
        rolling_request,
        "B",
        0.0,
    )
    manager.robots = {
        queued_robot.name: queued_robot,
        rolling_robot.name: rolling_robot,
    }
    manager.orders = {
        queued_order.order_id: queued_order,
        rolling_order.order_id: rolling_order,
    }
    manager._last_async_job_kind = "prefetch"
    calls: list[str] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda orders: [queued_entry],
    )
    monkeypatch.setattr(
        manager,
        "_ready_rolling_prefetch_entries",
        lambda: [urgent_prefetch],
    )
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda entries: calls.append("dispatch"),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda entries: calls.append("prefetch"),
    )

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["dispatch"]
    calls.clear()
    manager._last_async_job_kind = "dispatch"

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["prefetch"]


def test_second_coupled_replan_yields_planner_slot_to_ready_dispatch(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    first = FleetRobot(
        name="cycle-1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="cycle-order-1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
    )
    second = FleetRobot(
        name="cycle-2",
        current_lm="B",
        target_lm="A",
        status="WAITING",
        active_order_id="cycle-order-2",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
    )
    queued = FleetRobot(
        name="queued",
        current_lm="A",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {
        first.name: first,
        second.name: second,
        queued.name: queued,
    }
    manager.orders = {
        "cycle-order-1": FleetOrder(
            order_id="cycle-order-1",
            target_lm="B",
            vehicle=first.name,
            status="WAITING_TRAFFIC",
        ),
        "cycle-order-2": FleetOrder(
            order_id="cycle-order-2",
            target_lm="A",
            vehicle=second.name,
            status="WAITING_TRAFFIC",
        ),
        "queued-order": FleetOrder(
            order_id="queued-order",
            target_lm="B",
            vehicle=queued.name,
            status="QUEUED",
        ),
    }
    manager.task_manager.replace_storage(manager.orders)
    manager._last_async_job_kind = "coupled_replan"
    monkeypatch.setattr(
        manager,
        "_plan_valid_requests",
        lambda *_args, **_kwargs: {
            "ok": False,
            "plans": [],
            "debug": {"reason": "unexpected coupled planner call"},
        },
    )

    assert not manager._start_async_coupled_replan(
        [first, second],
        first,
        now,
    )
    assert not manager._async_simulated_dispatch_active()

    dispatched: list[list[str]] = []
    monkeypatch.setattr(manager, "_ready_rolling_prefetch_entries", lambda: [])
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda entries: dispatched.append([entry[1].name for entry in entries]),
    )

    manager._dispatch_orders(async_simulated=True)

    assert dispatched == [["queued"]]


def test_coupled_replan_yields_first_turn_to_urgent_rolling_prefetch(
    monkeypatch,
) -> None:
    manager = _manager()
    first = FleetRobot(name="cycle-1", current_lm="A")
    second = FleetRobot(name="cycle-2", current_lm="B")
    prefetch_order = FleetOrder(
        order_id="rolling-order",
        target_lm="B",
        vehicle="rolling",
        status="PLANNING",
    )
    prefetch_robot = FleetRobot(name="rolling", current_lm="A")
    prefetch = (
        prefetch_order,
        prefetch_robot,
        {"name": "rolling", "startLm": "A", "goalLm": "B"},
        "B",
        0.0,
    )
    manager._last_async_job_kind = "coupled_replan"
    monkeypatch.setattr(
        manager,
        "_ready_rolling_prefetch_entries",
        lambda: [prefetch],
    )

    assert not manager._start_async_coupled_replan(
        [first, second],
        first,
        time(),
    )
    assert not manager._async_simulated_dispatch_active()


def test_rolling_boundary_holders_are_prefetched_as_one_recovery_batch(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params["fleet"]["rolling_prefetch_recovery_batch_size"] = 8
    for index in range(2):
        name = f"r{index + 1}"
        order = FleetOrder(
            order_id=f"o{index + 1}",
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        robot = FleetRobot(
            name=name,
            current_lm="A",
            target_lm="A",
            status="WAITING",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "A"},
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 1.0, "lm": "A"},
            ],
            route_clock=1.0,
            route_chunk_goal_lm="A",
            route_final_lm="B",
            active_order_id=order.order_id,
            has_executed_route=True,
            last_reason="rolling continuation pending",
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = robot

    entries = manager._ready_rolling_prefetch_entries()

    assert [entry[1].name for entry in entries] == ["r1", "r2"]
    assert all(entry[-1] == pytest.approx(0.0) for entry in entries)

    planned_names: list[str] = []
    planned = Event()

    def capture_plan(requests, payload):
        del payload
        planned_names.extend(str(request["name"]) for request in requests)
        planned.set()
        return {
            "ok": False,
            "plans": [],
            "debug": {"reason": "captured recovery batch"},
        }

    monkeypatch.setattr(manager, "_plan_valid_requests", capture_plan)
    manager._start_async_rolling_prefetch(entries)

    assert planned.wait(1.0)
    assert planned_names == ["r1", "r2"]


@pytest.mark.parametrize(
    ("offset", "expected_goal_selector"),
    (
        (0.0, "recovery"),
        (1.0, "rolling"),
    ),
)
def test_singleton_prefetch_selects_boundary_goal_only_after_chunk_exhaustion(
    monkeypatch,
    offset: float,
    expected_goal_selector: str,
) -> None:
    manager = _manager()
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="PLANNING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id=order.order_id,
        route_chunk_goal_lm="A",
        route_final_lm="B",
        route_revision=7,
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    entry = (
        order,
        robot,
        {"name": robot.name, "startLm": "A", "goalLm": "B"},
        "B",
        offset,
    )
    selected: list[str] = []
    planned = Event()

    def recovery_goal(*_args, **_kwargs):
        selected.append("recovery")
        return "B"

    def rolling_goal(*_args, **_kwargs):
        selected.append("rolling")
        return "B"

    monkeypatch.setattr(
        manager,
        "_rolling_recovery_planning_goal",
        recovery_goal,
    )
    monkeypatch.setattr(manager, "_rolling_planning_goal", rolling_goal)
    monkeypatch.setattr(
        manager,
        "_plan_valid_requests",
        lambda *_args, **_kwargs: (
            planned.set()
            or {
                "ok": False,
                "plans": [],
                "debug": {"reason": "captured singleton prefetch"},
            }
        ),
    )

    manager._start_async_rolling_prefetch(entry)

    assert planned.wait(1.0)
    assert selected == [expected_goal_selector]


def test_failed_rolling_boundary_recovery_expands_and_rotates_the_group() -> None:
    manager = _manager()
    manager.params["fleet"]["rolling_prefetch_recovery_batch_size"] = 2
    for index in range(6):
        name = f"r{index + 1}"
        order = FleetOrder(
            order_id=f"o{index + 1}",
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        robot = FleetRobot(
            name=name,
            current_lm="A",
            target_lm="A",
            status="WAITING",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, "lm": "A"},
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 1.0, "lm": "A"},
            ],
            route_clock=1.0,
            route_chunk_goal_lm="A",
            route_final_lm="B",
            active_order_id=order.order_id,
            has_executed_route=True,
            last_reason="rolling continuation pending",
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = robot

    assert len(manager._ready_rolling_prefetch_entries()) == 2
    manager._rolling_prefetch_failures["r1"] = 1
    second = manager._ready_rolling_prefetch_entries()
    assert len(second) == 4
    assert "r1" not in {entry[1].name for entry in second}
    manager._rolling_prefetch_failures["r1"] = 2
    assert len(manager._ready_rolling_prefetch_entries()) == 6


def test_failed_direct_boundary_release_escalates_to_the_endpoint_batch() -> None:
    manager = _manager()
    manager.params["fleet"]["rolling_prefetch_recovery_batch_size"] = 2
    for index in range(4):
        name = f"r{index + 1}"
        order = FleetOrder(
            order_id=f"o{index + 1}",
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm="A",
            target_lm="A",
            status="WAITING",
            active_order_id=order.order_id,
            last_reason="rolling continuation pending",
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->A", "lm": "A"},
                {"t": 1.0, "x": 0.0, "y": 0.0, "edgeId": "A->A", "lm": "A"},
            ],
            route_clock=1.0,
            route_chunk_goal_lm="A",
            route_final_lm="B",
            has_executed_route=True,
        )
    manager.robots["tail"] = FleetRobot(
        name="tail",
        current_lm="A",
        status="WAITING",
        last_reason="occupied by r1",
        wait_for_robot="r1",
        trajectory=[
            {"t": 0.0, "x": -1.0, "y": 0.0, "edgeId": "A->A", "lm": "A"},
        ],
    )

    assert [
        entry[1].name for entry in manager._ready_rolling_prefetch_entries()
    ] == ["r1"]

    manager._rolling_prefetch_failures["r1"] = 1
    recovered = manager._ready_rolling_prefetch_entries()

    assert recovered[0][1].name == "r1"
    assert {entry[1].name for entry in recovered} == {
        "r1",
        "r2",
        "r3",
        "r4",
    }


def test_rolling_boundary_recovery_follows_transitive_route_dependencies(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params["fleet"]["rolling_prefetch_recovery_batch_size"] = 2

    def entry(
        name: str,
        start_lm: str,
        route_nodes: list[str],
    ) -> tuple[FleetOrder, FleetRobot, dict[str, object], str, float]:
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm=route_nodes[-1],
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
            spatial_route_nodes=list(route_nodes),
        )
        robot = FleetRobot(
            name=name,
            current_lm=start_lm,
            target_lm=start_lm,
            status="WAITING",
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm=route_nodes[-1],
            last_reason="rolling continuation pending",
        )
        request: dict[str, object] = {
            "name": name,
            "startLm": start_lm,
            "goalLm": route_nodes[-1],
            "routeNodes": list(route_nodes),
        }
        return order, robot, request, route_nodes[-1], 0.0

    candidates = [
        entry("seed", "A", ["A", "B", "X"]),
        entry("bridge", "B", ["B", "C", "Y"]),
        entry("linked", "C", ["C", "Z"]),
        entry("other-1", "D", ["D", "E"]),
        entry("other-2", "E", ["E", "F"]),
    ]
    monkeypatch.setattr(
        manager,
        "_rolling_prefetch_candidates",
        lambda: list(candidates),
    )
    monkeypatch.setattr(
        manager,
        "_rolling_boundary_release_pressure",
        lambda: {"seed": 3},
    )
    manager._rolling_prefetch_failures["seed"] = 1

    recovered = manager._ready_rolling_prefetch_entries()

    assert recovered[0][1].name == "seed"
    assert {item[1].name for item in recovered} == {
        "seed",
        "bridge",
        "linked",
    }


def test_failed_rolling_component_does_not_expand_into_independent_routes(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params["fleet"]["rolling_prefetch_recovery_batch_size"] = 2

    def entry(
        name: str,
        start_lm: str,
        route_nodes: list[str],
    ) -> tuple[FleetOrder, FleetRobot, dict[str, object], str, float]:
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm=route_nodes[-1],
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
            spatial_route_nodes=list(route_nodes),
        )
        robot = FleetRobot(
            name=name,
            current_lm=start_lm,
            target_lm=start_lm,
            status="WAITING",
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm=route_nodes[-1],
            last_reason="rolling continuation pending",
        )
        request: dict[str, object] = {
            "name": name,
            "startLm": start_lm,
            "goalLm": route_nodes[-1],
            "routeNodes": list(route_nodes),
        }
        return order, robot, request, route_nodes[-1], 0.0

    candidates = [
        entry("local-1", "A", ["A", "B", "C"]),
        entry("local-2", "B", ["B", "C"]),
        entry("other-1", "D", ["D", "E", "F"]),
        entry("other-2", "E", ["E", "F"]),
        entry("isolated", "G", ["G", "H"]),
    ]
    monkeypatch.setattr(
        manager,
        "_rolling_prefetch_candidates",
        lambda: list(candidates),
    )
    monkeypatch.setattr(
        manager,
        "_rolling_boundary_release_pressure",
        lambda: {"local-1": 5},
    )
    manager._rolling_prefetch_failures.update({
        "local-1": 8,
        "local-2": 8,
    })

    recovered = manager._ready_rolling_prefetch_entries()

    assert recovered[0][1].name == "local-1"
    assert {item[1].name for item in recovered} == {
        "local-1",
        "local-2",
    }


def test_fresh_independent_rolling_boundaries_are_retried_one_at_a_time(
    monkeypatch,
) -> None:
    manager = _manager()

    def entry(
        name: str,
        start_lm: str,
        goal_lm: str,
    ) -> tuple[FleetOrder, FleetRobot, dict[str, object], str, float]:
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm=goal_lm,
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        robot = FleetRobot(
            name=name,
            current_lm=start_lm,
            target_lm=start_lm,
            status="WAITING",
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm=goal_lm,
            last_reason="rolling continuation pending",
        )
        return (
            order,
            robot,
            {
                "name": name,
                "startLm": start_lm,
                "goalLm": goal_lm,
                "routeNodes": [start_lm, goal_lm],
            },
            goal_lm,
            0.0,
        )

    candidates = [
        entry("r1", "A", "B"),
        entry("r2", "C", "D"),
        entry("r3", "E", "F"),
    ]
    monkeypatch.setattr(
        manager,
        "_rolling_prefetch_candidates",
        lambda: list(candidates),
    )
    monkeypatch.setattr(
        manager,
        "_rolling_boundary_release_pressure",
        lambda: {},
    )

    assert [
        item[1].name for item in manager._ready_rolling_prefetch_entries()
    ] == ["r1"]

    manager._rolling_prefetch_failures["r1"] = 1

    assert [
        item[1].name for item in manager._ready_rolling_prefetch_entries()
    ] == ["r2"]


def test_rolling_boundary_recovery_never_becomes_a_fleet_wide_batch(
    monkeypatch,
) -> None:
    manager = _manager()
    candidates = []
    count = manager.planner.local_cbs_max_robots + 4
    for index in range(count):
        name = f"r{index:02d}"
        start = f"N{index}"
        route_nodes = [
            start,
            *[f"N{next_index}" for next_index in range(index + 1, count)],
        ]
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm=route_nodes[-1],
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        robot = FleetRobot(
            name=name,
            current_lm=start,
            status="WAITING",
            active_order_id=order.order_id,
            route_chunk_goal_lm=start,
            route_final_lm=route_nodes[-1],
            last_reason="rolling continuation pending",
        )
        candidates.append(
            (
                order,
                robot,
                {
                    "name": name,
                    "startLm": start,
                    "goalLm": route_nodes[-1],
                    "routeNodes": route_nodes,
                },
                route_nodes[-1],
                0.0,
            )
        )
        manager._rolling_prefetch_failures[name] = 8

    monkeypatch.setattr(
        manager,
        "_rolling_prefetch_candidates",
        lambda: list(candidates),
    )
    monkeypatch.setattr(
        manager,
        "_rolling_boundary_release_pressure",
        lambda: {},
    )

    recovered = manager._ready_rolling_prefetch_entries()

    assert len(recovered) == manager.planner.local_cbs_max_robots


def test_full_rolling_collapse_recovers_empty_routes_and_releases_sink() -> None:
    landmarks = {
        name: Landmark(name=name, x=float(index), y=0.0)
        for index, name in enumerate(("A", "B", "C", "P"))
    }
    edges = [
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
        for src, dst in (("A", "B"), ("B", "C"), ("C", "P"))
    ]
    manager = FleetManagerSim(landmarks, edges)
    routes = {
        "r1": ["A", "B", "C", "P"],
        "r2": ["B", "C", "P"],
        "r3": ["C", "P"],
    }
    for revision, (name, route_nodes) in enumerate(routes.items(), start=1):
        start_lm = route_nodes[0]
        landmark = landmarks[start_lm]
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm="P",
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
            # A failed ordinary continuation may already have invalidated the
            # cached suffix. Collapse recovery must reconstruct it without
            # treating every alphabetic seed as an independent sink.
            spatial_route_nodes=[],
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=start_lm,
            target_lm=start_lm,
            status="WAITING",
            pose={"x": landmark.x, "y": landmark.y, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": landmark.x, "y": landmark.y, "lm": start_lm},
                {"t": 1.0, "x": landmark.x, "y": landmark.y, "lm": start_lm},
            ],
            route_clock=1.0,
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm="P",
            route_revision=revision,
            has_executed_route=True,
            last_reason="rolling continuation pending",
        )
        manager._rolling_prefetch_failures[name] = 1
        manager._rolling_prefetch_retry_at[name] = manager._now() + 1000.0

    assert [
        entry[1].name for entry in manager._ready_rolling_prefetch_entries()
    ] == ["r3"]

    manager._rolling_prefetch_failures["r3"] = 0
    assert manager._ready_rolling_prefetch_entries() == []
    manager._rolling_prefetch_failures["r3"] = 1
    manager.robots["r3"].status = "MOVING"
    assert manager._ready_rolling_prefetch_entries() == []
    manager.robots["r3"].status = "WAITING"
    manager.robots["r3"].pending_route = {"planned": True}
    assert manager._ready_rolling_prefetch_entries() == []


def test_full_collapse_dependency_uses_shared_corridor_resources() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "X": Landmark(
            name="X",
            x=1.0,
            y=0.0,
            properties={"controlled_region": "shared-corridor"},
        ),
        "G": Landmark(name="G", x=2.0, y=0.0),
        "B": Landmark(
            name="B",
            x=1.0,
            y=1.0,
            properties={"controlled_region": "shared-corridor"},
        ),
        "P": Landmark(name="P", x=2.0, y=1.0),
    }
    edges = [
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
        for src, dst in (("A", "X"), ("X", "G"), ("B", "P"))
    ]
    manager = FleetManagerSim(landmarks, edges)
    definitions = (
        ("a-seed", "A", "G", ["A", "X", "G"]),
        ("z-sink", "B", "P", ["B", "P"]),
    )
    for revision, (name, start_lm, final_lm, route_nodes) in enumerate(
        definitions,
        start=1,
    ):
        landmark = landmarks[start_lm]
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm=final_lm,
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
            spatial_route_nodes=route_nodes,
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=start_lm,
            status="WAITING",
            pose={"x": landmark.x, "y": landmark.y, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": landmark.x, "y": landmark.y, "lm": start_lm},
                {"t": 1.0, "x": landmark.x, "y": landmark.y, "lm": start_lm},
            ],
            route_clock=1.0,
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm=final_lm,
            route_revision=revision,
            has_executed_route=True,
            last_reason="rolling continuation pending",
        )
        manager._rolling_prefetch_failures[name] = 1
        manager._rolling_prefetch_retry_at[name] = manager._now() + 1000.0

    release = manager._ready_rolling_prefetch_entries()

    # B is not a node on A->X, but B and X occupy the same controlled
    # resource. Releasing alphabetic a-seed first would leave that token held.
    assert [entry[1].name for entry in release] == ["z-sink"]


def test_cyclic_rolling_collapse_uses_fixed_pocket_and_blacklists_failure(
    monkeypatch,
) -> None:
    landmarks = {
        "A": Landmark(
            name="A",
            x=0.0,
            y=0.0,
            properties={"controlled_region": "corridor-a"},
        ),
        "B": Landmark(name="B", x=2.0, y=0.0),
        "P1": Landmark(
            name="P1",
            x=0.0,
            y=2.0,
            properties={"controlled_region": "corridor-a"},
        ),
        "P2": Landmark(name="P2", x=2.0, y=2.0),
        "P3": Landmark(name="P3", x=-2.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=2.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[src].x, landmarks[src].y),
                WorldPoint(landmarks[dst].x, landmarks[dst].y),
            ),
            properties={"direction": 1},
        )
        for src, dst in (
            ("A", "B"),
            ("B", "A"),
            ("A", "P1"),
            ("B", "P2"),
            ("A", "P3"),
        )
    ]
    manager = FleetManagerSim(landmarks, edges)
    for revision, (name, start_lm, final_lm) in enumerate(
        (("r1", "A", "B"), ("r2", "B", "A")),
        start=1,
    ):
        landmark = landmarks[start_lm]
        order = FleetOrder(
            order_id=f"o-{name}",
            target_lm=final_lm,
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
            spatial_route_nodes=[start_lm, final_lm],
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=start_lm,
            target_lm=start_lm,
            status="WAITING",
            pose={"x": landmark.x, "y": landmark.y, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": landmark.x, "y": landmark.y, "lm": start_lm},
                {"t": 1.0, "x": landmark.x, "y": landmark.y, "lm": start_lm},
            ],
            route_clock=1.0,
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm=final_lm,
            route_revision=revision,
            has_executed_route=True,
            last_reason="rolling continuation pending",
        )
        manager._rolling_prefetch_failures[name] = 1
        manager._rolling_prefetch_retry_at[name] = manager._now() + 1000.0

    first = manager._ready_rolling_prefetch_entries()
    assert len(first) == 1
    assert first[0][1].name == "r1"
    assert first[0][2]["vacancyRecovery"] is True
    # P1 is geometrically free but retains A's controlled-region token, so it
    # cannot create a vacancy. The fixed escape must leave that resource.
    assert first[0][2]["goalLm"] == "P3"
    assert first[0][2]["routeNodes"] == ["A", "P3"]
    assert first[0][3] == "B"

    captured: dict[str, object] = {}
    planner_called = Event()

    def fail_fixed_path(requests, payload):
        captured["requests"] = requests
        captured["payload"] = payload
        planner_called.set()
        return {
            "ok": False,
            "plans": [],
            "debug": {"reason": "no_low_level_path:r1:fixed pocket blocked"},
        }

    monkeypatch.setattr(manager, "_plan_valid_requests", fail_fixed_path)
    manager._start_async_rolling_prefetch(first)

    assert planner_called.wait(1.0)
    deadline = perf_counter() + 1.0
    while (
        manager._dispatch_job is not None
        and not manager._dispatch_job.get("done")
        and perf_counter() < deadline
    ):
        sleep(0.001)
    assert captured["payload"]["allowCbsFallback"] is False
    assert captured["requests"][0]["routeNodes"] == ["A", "P3"]
    assert captured["requests"][0]["vacancyRecovery"] is True
    assert manager.orders["o-r1"].target_lm == "B"

    signature = manager._rolling_vacancy_recovery_signature
    manager._finish_async_simulated_dispatch()

    assert (signature, "r1", "P3") in (
        manager._rolling_vacancy_recovery_blacklist
    )
    second = manager._ready_rolling_prefetch_entries()
    assert second[0][1].name == "r2"
    assert second[0][2]["goalLm"] == "P2"

    manager.robots["r1"].route_revision += 1
    third = manager._ready_rolling_prefetch_entries()
    assert third[0][1].name == "r1"
    assert third[0][2]["goalLm"] == "P3"
    assert not manager._rolling_vacancy_recovery_blacklist


@pytest.mark.parametrize(
    "reason",
    (
        "cbs_resource_conflict:r2",
        "no_low_level_path:r2:A->B:stationary_robot_blocks_route",
        "no_low_level_path:r2:resource_constrained:S015002@6",
        "no_low_level_path:r2:cannot_wait:S015002@6-8",
    ),
)
def test_named_batch_conflict_does_not_back_off_unaffected_orders(
    reason: str,
) -> None:
    manager = _manager()
    entries = []
    for index in range(2):
        name = f"r{index + 1}"
        robot = FleetRobot(
            name=name,
            current_lm="A",
            status="IDLE",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        )
        order = FleetOrder(
            order_id=f"o{index + 1}",
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        manager.robots[name] = robot
        manager.orders[order.order_id] = order
        entries.append(
            (order, robot, {"name": name, "startLm": "A", "goalLm": "B"}, "B")
        )

    manager._finish_simulated_order_batch(
        entries,
        {
            "ok": False,
            "plans": [],
            "debug": {"reason": reason},
        },
    )

    assert manager.orders["o1"].status == "QUEUED"
    assert manager.orders["o1"].error == ""
    assert manager.orders["o1"].dispatch_failures == 0
    assert manager.orders["o2"].error == reason
    assert manager.orders["o2"].dispatch_failures == 1


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


def test_traffic_retry_keeps_the_full_safe_temporal_chunk() -> None:
    manager = _long_line_manager(edge_count=6)
    manager.params["fleet"]["rolling_horizon_sec"] = 20.0
    order = FleetOrder(
        order_id="o1",
        target_lm="N6",
        vehicle="r1",
        dispatch_failures=1,
    )

    planning_goal = manager._rolling_planning_goal("N0", "N6", order)

    assert planning_goal == "N6"
    assert order.spatial_route_nodes == [f"N{index}" for index in range(7)]


def test_non_final_rolling_chunk_holds_before_transit_junction() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "I1": Landmark(name="I1", x=1.0, y=0.0),
        "I2": Landmark(name="I2", x=2.0, y=0.0),
        "J": Landmark(
            name="J",
            x=3.0,
            y=0.0,
            properties={"waitAllowed": True},
        ),
        "G": Landmark(name="G", x=4.0, y=0.0),
        "X": Landmark(name="X", x=3.0, y=1.0),
    }
    edges: list[GraphEdge] = []
    for first, second in (
        ("A", "I1"),
        ("I1", "I2"),
        ("I2", "J"),
        ("J", "G"),
        ("J", "X"),
    ):
        for src, dst in ((first, second), (second, first)):
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
                    properties={"direction": 2, "smart": True},
                )
            )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {
                "route_speed": 1.0,
                "route_acceleration": 1.0,
                "simulate_rotation": False,
            },
            "fleet": {
                "planner_backend": "rolling_sipp",
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": True,
                "controlled_corridor_min_edges": 1,
                "reservation_time_step_sec": 1.0,
                "rolling_horizon_sec": 3.1,
                "mapf_min_robot_center_distance_m": 0.8,
            },
        },
    )
    order = FleetOrder(order_id="o1", target_lm="G", vehicle="r1")
    traffic_graph = manager.planner._traffic_graph(1.0)

    assert not traffic_graph.vertices["J"].can_wait
    assert traffic_graph.vertices["I2"].can_wait
    assert manager._rolling_planning_goal("A", "G", order) == "I2"

    # Reservation waits can make the result trimmer stop earlier than the
    # graph waypoint selected above. It must apply the same stop-line rule
    # instead of recreating a chunk that ends inside J.
    result = {
        "ok": True,
        "timeStepSec": 1.0,
        "debug": {"routeSpeed": 1.0},
        "plans": [
            {
                "robot": "r1",
                "startLm": "A",
                "goalLm": "G",
                "nodes": ["A", "I1", "I2", "J", "G"],
                "times": [0, 1, 2, 3, 10],
                "trajectory": [
                    {
                        "t": float(index),
                        "x": landmarks[node].x,
                        "y": landmarks[node].y,
                        "yaw": 0.0,
                        "edgeId": f"{node}->{node}",
                        "lm": node,
                    }
                    for index, node in enumerate(("A", "I1", "I2", "J", "G"))
                ],
            }
        ],
    }

    trimmed = manager._rolling_result(result, {"r1": "G"})

    assert trimmed["plans"][0]["goalLm"] == "I2"
    assert trimmed["plans"][0]["nodes"][-1] == "I2"


def test_rolling_batch_goal_never_uses_another_robot_start() -> None:
    manager = _long_line_manager(edge_count=3)
    order = FleetOrder(
        order_id="o1",
        target_lm="N3",
        vehicle="r1",
        spatial_route_nodes=["N0", "N1", "N2", "N3"],
    )

    goal = manager._distinct_rolling_batch_goal(
        order,
        "N0",
        "N3",
        "N1",
        reserved_goals=set(),
        protected_starts={"N0", "N1"},
        release_robot_names={"r1", "r2"},
    )

    assert goal == "N2"


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


def test_fresh_serialized_departures_do_not_become_infinite_reservations() -> None:
    manager = _manager()
    manager.robots["moving"] = FleetRobot(
        name="moving",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots["departing"] = FleetRobot(
        name="departing",
        current_lm="B",
        status="IDLE",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    manager.orders["departing-order"] = FleetOrder(
        order_id="departing-order",
        target_lm="A",
        vehicle="departing",
        assigned_robot="departing",
        status="QUEUED",
        dispatch_failures=1,
    )
    request = {
        "name": "moving",
        "startLm": "A",
        "goalLm": "B",
        "startPose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }

    assert manager._bootstrap_departure_robot_names([request]) == {"departing"}
    assert manager._stationary_robot_blocked_lms(
        exclude_robot_names={"moving"},
    ) == set()

    manager.robots["departing"].has_executed_route = True
    assert manager._bootstrap_departure_robot_names([request]) == {"departing"}
    assert manager._stationary_robot_blocked_lms(
        exclude_robot_names={"moving"},
    ) == set()


def test_exhausted_rolling_holder_is_a_spatial_route_obstacle() -> None:
    manager = _manager()
    order = FleetOrder(
        order_id="holder-order",
        target_lm="B",
        vehicle="holder",
        assigned_robot="holder",
        status="PLANNING",
    )
    holder = FleetRobot(
        name="holder",
        current_lm="A",
        target_lm="A",
        status="WAITING",
        active_order_id=order.order_id,
        last_reason="rolling continuation pending",
        route_chunk_goal_lm="A",
        route_final_lm="B",
        route_clock=1.0,
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
        ],
    )
    manager.orders[order.order_id] = order
    manager.robots[holder.name] = holder

    assert manager._stationary_robot_blocked_lms() == {"A"}
    assert manager._stationary_robot_blocked_lms(
        exclude_robot_names={holder.name},
    ) == set()


def test_queued_idle_departures_do_not_invalidate_a_shared_departure_route() -> None:
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
    ) == set()

    route = manager._ensure_order_spatial_route(moving_order, "A", "G")

    assert route == ["A", "B", "C", "G"]
    assert moving_order.spatial_route_nodes == route


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


def test_boundary_recovery_uses_normal_horizon_outside_controlled_corridor() -> None:
    landmarks = {
        name: Landmark(name=name, x=float(index), y=0.0)
        for index, name in enumerate(("A", "B", "C", "D", "E", "F"))
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[src].x, 0.0),
                WorldPoint(landmarks[dst].x, 0.0),
            ),
            properties={"direction": 1},
        )
        for src, dst in zip(("A", "B", "C", "D", "E"), ("B", "C", "D", "E", "F"))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "rolling_horizon_sec": 3.0,
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="F",
        vehicle="r1",
        spatial_route_nodes=["A", "B", "C", "D", "E", "F"],
    )

    normal_goal = manager._rolling_planning_goal("A", "F", order)
    recovery_goal = manager._rolling_recovery_planning_goal(
        "A",
        "F",
        order,
        release_robot_names={"r1"},
    )

    assert normal_goal == "D"
    assert recovery_goal == normal_goal
    assert recovery_goal != "B"


def test_boundary_recovery_commits_through_explicit_corridor_exit() -> None:
    region = "corridor:A<=>D"
    landmarks = {
        "A": Landmark(
            name="A",
            x=0.0,
            y=0.0,
            properties={"holding_point": True},
        ),
        "B": Landmark(
            name="B",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "C": Landmark(
            name="C",
            x=2.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "D": Landmark(
            name="D",
            x=3.0,
            y=0.0,
            properties={"holding_point": True},
        ),
        "E": Landmark(name="E", x=4.0, y=0.0),
    }
    edges = []
    for src, dst in zip(("A", "B", "C", "D"), ("B", "C", "D", "E")):
        properties = {"direction": 1}
        if dst != "E":
            properties["controlled_region"] = region
        edges.append(
            GraphEdge(
                from_name=src,
                to_name=dst,
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[src].x, 0.0),
                    WorldPoint(landmarks[dst].x, 0.0),
                ),
                properties=properties,
            )
        )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "rolling_horizon_sec": 1.0,
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="E",
        vehicle="r1",
        spatial_route_nodes=["A", "B", "C", "D", "E"],
    )

    recovery_goal = manager._rolling_recovery_planning_goal(
        "A",
        "E",
        order,
        release_robot_names={"r1"},
    )

    assert recovery_goal == "D"


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


def test_explicit_corridor_is_local_authority_over_dynamic_zone_gate() -> None:
    landmarks = {
        "WEST": Landmark(name="WEST", x=0.0, y=7.0),
        "NORTH": Landmark(name="NORTH", x=7.0, y=0.0),
        "HUB": Landmark(name="HUB", x=7.0, y=7.0),
    }
    edges = [
        GraphEdge(
            from_name="WEST",
            to_name="HUB",
            length=7.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks["WEST"].x, landmarks["WEST"].y),
                WorldPoint(landmarks["HUB"].x, landmarks["HUB"].y),
            ),
            properties={
                "direction": 1,
                "controlled_region": "corridor:west-hub",
            },
        ),
        GraphEdge(
            from_name="NORTH",
            to_name="HUB",
            length=7.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks["NORTH"].x, landmarks["NORTH"].y),
                WorldPoint(landmarks["HUB"].x, landmarks["HUB"].y),
            ),
            properties={"direction": 1},
        ),
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": True,
                "traffic_zone_size_m": 6.0,
                "traffic_zone_demand_threshold": 2,
                "traffic_zone_capacity": 1,
                "traffic_zone_batch_size": 1,
            }
        },
    )
    for name, start in (("corridor", "WEST"), ("zone", "NORTH")):
        pose = {
            "x": landmarks[start].x,
            "y": landmarks[start].y,
            "yaw": 0.0,
        }
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
            spatial_route_nodes=[start, "HUB"],
        )

    now = 1_000.0
    manager._prepare_controlled_corridor_admissions(now)
    manager._prepare_traffic_zone_admissions(now)

    target_zone = manager._traffic_zone_by_lm["HUB"]
    assert manager._controlled_corridor_winners == {
        "corridor": "corridor:west-hub"
    }
    assert manager._traffic_zone_winners == {"zone": target_zone}
    assert manager._next_traffic_zone_transition(
        manager.robots["corridor"]
    ) is None
    assert manager._traffic_zone_admission_reason(
        manager.robots["corridor"],
        0.1,
    ) == ""


def test_controlled_corridor_admission_holds_loser_at_boundary() -> None:
    landmarks = {
        name: Landmark(name=name, x=x, y=0.0)
        for name, x in (("L", 0.0), ("B", 1.0), ("C", 2.0), ("R", 3.0))
    }
    edges = []
    for first, second in (("L", "B"), ("B", "C"), ("C", "R")):
        for src, dst in ((first, second), (second, first)):
            edges.append(
                GraphEdge(
                    from_name=src,
                    to_name=dst,
                    length=1.0,
                    kind="line",
                    edge_type="FeatureLine",
                    world_points=(
                        WorldPoint(landmarks[src].x, 0.0),
                        WorldPoint(landmarks[dst].x, 0.0),
                    ),
                    properties={"direction": 2, "smart": True},
                )
            )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": "auto",
                "reservation_horizon_sec": 1.0,
                "traffic_zone_control_enabled": False,
            },
        },
    )

    def trajectory(nodes: list[str]) -> list[dict[str, object]]:
        result: list[dict[str, object]] = []
        for index, node in enumerate(nodes):
            edge_id = (
                f"{nodes[0]}->{nodes[1]}"
                if index == 0
                else f"{nodes[index - 1]}->{node}"
            )
            result.append({
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": edge_id,
                "lm": node,
            })
        return result

    for name, nodes, priority in (
        ("r1", ["L", "B", "C", "R"], 1),
        ("r2", ["R", "C", "B", "L"], 5),
    ):
        start = nodes[0]
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=start,
            target_lm=nodes[-1],
            status="MOVING",
            active_order_id=f"o-{name}",
            pose={"x": landmarks[start].x, "y": 0.0, "yaw": 0.0},
            trajectory=trajectory(nodes),
        )
        manager.orders[f"o-{name}"] = FleetOrder(
            order_id=f"o-{name}",
            target_lm=nodes[-1],
            vehicle=name,
            assigned_robot=name,
            status="EXECUTING",
            priority=priority,
            spatial_route_nodes=nodes,
        )

    now = 1_000.0
    manager._prepare_controlled_corridor_admissions(now)
    region_id = manager.planner._traffic_graph(1.0).controlled_region_ids()[0]

    assert manager._controlled_corridor_winners == {"r2": region_id}
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r2"],
        0.1,
    ) == ""
    loser_reason = manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.1,
    )
    assert loser_reason.startswith("corridor admission wait at L")
    manager.robots["r1"].last_reason = loser_reason
    assert manager._blocked_at_clock(manager.robots["r1"], 0.1).startswith(
        "corridor admission wait at L"
    )
    assert (
        manager._blocked_at_clock(
            manager.robots["r1"],
            0.1,
            ignore_admission=True,
        )
        == ""
    )
    manager.robots["r1"].traffic_priority_until = now + 1.0
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.1,
    ).startswith("corridor admission wait at L")
    assert manager._transfer_controlled_corridor_lease(
        manager.robots["r1"],
        [manager.robots["r1"], manager.robots["r2"]],
        now,
    )
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.1,
    ) == ""
    manager.robots["r1"].traffic_priority_until = 0.0
    assert manager._reservation_horizon() > 3.0
    manager._controlled_corridor_leases[region_id] = ("r2", now + 10.0)
    manager._controlled_corridor_winners = {"r2": region_id}

    # A rotate/wait sample at the portal must not hide the following corridor
    # edge from the runtime gate. This was the live path that fell through to
    # pairwise collision arbitration and generated thousands of priority
    # grants while the robot was still standing at the entrance.
    original = trajectory(["L", "B", "C", "R"])
    manager.robots["r1"].trajectory = [
        {
            "t": 0.0,
            "x": landmarks["L"].x,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "L->L",
            "lm": "L",
        },
        {
            "t": 1.0,
            "x": landmarks["L"].x,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "WAIT@ROTATE:L",
            "lm": "L",
        },
        *[
            {**sample, "t": float(sample["t"]) + 2.0}
            for sample in original
        ],
    ]
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.5,
    ).startswith("corridor admission wait at L")

    manager.robots["r2"].route_clock = 0.5
    manager.robots["r2"].pose = {"x": 2.5, "y": 0.0, "yaw": 0.0}
    manager._prepare_controlled_corridor_admissions(now + 0.5)

    assert manager._controlled_corridor_occupancy == {region_id: ["r2"]}
    assert "owner r2" in manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.1,
    )
    corridor_wait = manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.1,
    )
    assert manager._is_robot_conflict(corridor_wait)
    assert manager._robot_name_from_conflict_reason(corridor_wait) == "r2"
    assert not manager._should_replan_for_blocked_reason(
        f"corridor admission wait at L for {region_id}; owner r2"
    )
    assert manager._reason_requires_spatial_replan(
        f"corridor admission timeout: wait at L for {region_id}"
    )

    # A temporary traffic wait inside the corridor must retain its executable
    # route. Clearing it would make the robot a permanent stationary corridor
    # owner and prevent both arbitration and graph-safe retreat.
    internal = manager.robots["r2"]
    internal.current_lm = "B"
    internal.pose = {"x": landmarks["B"].x, "y": 0.0, "yaw": 0.0}
    internal.route_clock = 1.0
    assert not manager._queue_active_order_for_background_replan(
        internal,
        now + 1.0,
        "traffic wait timeout: occupied by r1",
    )
    assert internal.trajectory
    assert internal.active_order_id == "o-r2"


def test_controlled_corridor_exit_boundary_selects_the_next_corridor() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "J": Landmark(name="J", x=1.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
        "X": Landmark(name="X", x=1.0, y=1.0),
    }
    edges: list[GraphEdge] = []
    for first, second in (("A", "J"), ("J", "B"), ("J", "X")):
        for src, dst in ((first, second), (second, first)):
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
                    properties={"direction": 2, "smart": True},
                )
            )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": "auto",
                "controlled_corridor_min_edges": 1,
                "traffic_zone_control_enabled": False,
            },
        },
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.994, "y": 0.0, "yaw": 0.0},
        route_clock=0.994,
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->J",
                "lm": "A",
            },
            {
                "t": 1.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->J",
                "lm": "J",
            },
            {
                "t": 2.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "J->B",
                "lm": "B",
            },
        ],
    )
    manager.robots = {robot.name: robot}
    graph = manager._controlled_corridor_graph
    assert graph is not None
    previous_lane = graph.lane_for("A", "J")
    next_lane = graph.lane_for("J", "B")
    assert previous_lane is not None
    assert next_lane is not None
    previous_region = previous_lane.controlled_region_ids[0]
    next_region = next_lane.controlled_region_ids[0]
    assert previous_region != next_region

    entry = manager._next_controlled_corridor_entry(robot)

    assert entry is not None
    assert entry["region"] == next_region
    now = 1_000.0
    manager._controlled_corridor_leases[next_region] = (
        robot.name,
        now + 10.0,
    )
    manager._prepare_controlled_corridor_admissions(now)
    assert manager._controlled_corridor_leases[next_region][0] == robot.name
    assert manager._controlled_corridor_winners == {robot.name: next_region}


def test_consecutive_corridor_transition_never_stops_inside_upstream_zone() -> None:
    region_a = "corridor:a"
    region_b = "corridor:b"
    landmarks = {
        "HOLD": Landmark(name="HOLD", x=0.0, y=0.0),
        "A": Landmark(
            name="A",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region_a},
        ),
        "B": Landmark(
            name="B",
            x=2.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region_b},
        ),
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={"controlled_region": regions},
        )
        for src, dst, regions in (
            ("HOLD", "A", region_a),
            ("A", "B", f"{region_a},{region_b}"),
        )
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 1.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
    )
    manager.robots = {robot.name: robot}
    manager._controlled_corridor_leases[region_b] = ("other", 100.0)
    manager._controlled_corridor_winners = {"other": region_b}

    assert manager._next_controlled_corridor_entry(robot) is None
    assert manager._controlled_corridor_admission_reason(robot, 0.1) == ""


def test_controlled_corridor_owner_wins_and_boundary_entrant_queues_detour() -> None:
    manager = _manager()
    now = time()
    owner = FleetRobot(
        name="owner",
        current_lm="B",
        target_lm="A",
        status="WAITING",
        active_order_id="owner-order",
        last_reason="occupied by entrant",
        wait_for_robot="entrant",
        blocked_since=now - 2.0,
        traffic_stall_since=now - 2.0,
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "B",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "A",
            },
        ],
    )
    entrant = FleetRobot(
        name="entrant",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="entrant-order",
        last_reason=(
            "corridor admission wait at A for corridor-A-B; owner owner"
        ),
        wait_for_robot="owner",
        blocked_since=now - 2.0,
        traffic_stall_since=now - 2.0,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
    )
    manager.robots = {owner.name: owner, entrant.name: entrant}
    manager.orders = {
        "owner-order": FleetOrder(
            order_id="owner-order",
            target_lm="A",
            vehicle=owner.name,
            assigned_robot=owner.name,
            status="EXECUTING",
        ),
        "entrant-order": FleetOrder(
            order_id="entrant-order",
            target_lm="B",
            vehicle=entrant.name,
            assigned_robot=entrant.name,
            status="EXECUTING",
            spatial_route_nodes=["A", "B"],
        ),
    }
    manager._controlled_corridor_occupancy = {
        "corridor-A-B": [owner.name],
    }
    cycle_key = tuple(sorted(manager.robots))
    manager._active_wait_cycles[cycle_key] = (
        now - manager._deadlock_wait_timeout() - 0.1
    )

    manager._break_runtime_wait_cycle(
        [entrant.name, owner.name],
        manager.robots,
        now,
        new_episode=False,
    )

    assert owner.status == "MOVING"
    assert owner.last_reason == "deadlock priority granted"
    assert entrant.status == "IDLE"
    assert not entrant.trajectory
    assert not entrant.active_order_id
    entrant_order = manager.orders["entrant-order"]
    assert entrant_order.status == "QUEUED"
    assert entrant_order.traffic_detour_edges == [("A", "B"), ("B", "A")]
    assert entrant_order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["cycleReplans"] == 1


def test_controlled_corridor_lease_cannot_preempt_physical_owner() -> None:
    manager = _manager()
    now = time()
    owner = FleetRobot(name="owner", current_lm="B")
    entrant = FleetRobot(
        name="entrant",
        current_lm="A",
        last_reason=(
            "corridor admission wait at A for corridor-A-B; owner owner"
        ),
    )
    manager._controlled_corridor_occupancy = {
        "corridor-A-B": [owner.name],
    }
    manager._controlled_corridor_leases = {
        "corridor-A-B": (owner.name, now + 10.0),
    }
    manager._controlled_corridor_winners = {
        owner.name: "corridor-A-B",
    }

    transferred = manager._transfer_controlled_corridor_lease(
        entrant,
        [entrant, owner],
        now,
    )

    assert not transferred
    assert manager._controlled_corridor_leases["corridor-A-B"][0] == owner.name
    assert manager._controlled_corridor_winners == {
        owner.name: "corridor-A-B",
    }


def test_stationary_blocker_with_queued_order_is_released_first() -> None:
    manager = _manager()
    manager.robots = {
        "waiter": FleetRobot(
            name="waiter",
            current_lm="B",
            target_lm="A",
            status="WAITING",
            active_order_id="wait-order",
            last_reason="occupied by blocker",
            wait_for_robot="blocker",
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
                {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
            ],
        ),
        "blocker": FleetRobot(
            name="blocker",
            current_lm="A",
            status="IDLE",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        ),
    }
    manager.orders = {
        "wait-order": FleetOrder(
            order_id="wait-order",
            target_lm="A",
            vehicle="waiter",
            assigned_robot="waiter",
            status="WAITING_TRAFFIC",
        ),
        "release-order": FleetOrder(
            order_id="release-order",
            target_lm="B",
            vehicle="blocker",
            assigned_robot="blocker",
            status="QUEUED",
        ),
    }

    assert manager._stationary_release_robot_names() == {"blocker"}
    assert manager._robot_departure_pending(manager.robots["blocker"])
    assert manager._wait_expected_to_clear(manager.robots["waiter"])


def test_stationary_blockers_are_dispatched_as_one_release_wave(
    monkeypatch,
) -> None:
    manager = _manager()
    entries = []
    for index in range(2):
        name = f"blocker-{index}"
        robot = FleetRobot(
            name=name,
            current_lm="A",
            status="IDLE",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        )
        order = FleetOrder(
            order_id=f"release-{index}",
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="QUEUED",
            traffic_detour_edges=[("A", "B")],
            spatial_route_nodes=["A", "B"],
        )
        manager.robots[name] = robot
        manager.orders[order.order_id] = order
        entries.append(
            (order, robot, {"name": name, "startLm": "A", "goalLm": "B"}, "B")
        )
    captured: list[list[str]] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda _orders: list(entries),
    )
    monkeypatch.setattr(
        manager,
        "_stationary_release_robot_names",
        lambda: {"blocker-0", "blocker-1"},
    )
    monkeypatch.setattr(manager, "_ready_rolling_prefetch_entries", lambda: [])
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda group: captured.append([entry[1].name for entry in group]),
    )

    manager._dispatch_orders(async_simulated=True)

    assert captured == [["blocker-0", "blocker-1"]]
    assert all(
        entry[0].traffic_detour_edges == [("A", "B")]
        for entry in entries
    )
    assert all(not entry[0].spatial_route_nodes for entry in entries)


def test_stationary_release_keeps_an_explicit_alternate_corridor(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = FleetRobot(
        name="entrant",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    detour_edges = [("A", "B"), ("B", "A")]
    order = FleetOrder(
        order_id="entrant-order",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="QUEUED",
        traffic_detour_edges=list(detour_edges),
        spatial_route_nodes=["A", "B"],
    )
    entry = (
        order,
        robot,
        {"name": robot.name, "startLm": "A", "goalLm": "B"},
        "B",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}
    captured: list[list[tuple[str, str]]] = []
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda _orders: [entry],
    )
    monkeypatch.setattr(
        manager,
        "_stationary_release_robot_names",
        lambda: {robot.name},
    )
    monkeypatch.setattr(
        manager,
        "_dispatch_simulated_order_batch",
        lambda group: (
            captured.append(list(group[0][0].traffic_detour_edges)) or 0,
            {order.order_id},
        ),
    )

    manager._dispatch_orders()

    assert captured == [detour_edges]
    assert order.traffic_detour_edges == detour_edges
    assert not order.spatial_route_nodes


def test_explicit_detour_order_is_not_grouped_under_plain_order_payload(
    monkeypatch,
) -> None:
    manager = _manager()
    entries = []
    for name, detour_edges in (
        ("plain", []),
        ("detour", [("A", "B"), ("B", "A")]),
    ):
        robot = FleetRobot(
            name=name,
            current_lm="A",
            status="IDLE",
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        )
        order = FleetOrder(
            order_id=f"{name}-order",
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="QUEUED",
            traffic_detour_edges=list(detour_edges),
        )
        manager.robots[name] = robot
        manager.orders[order.order_id] = order
        entries.append(
            (
                order,
                robot,
                {"name": name, "startLm": "A", "goalLm": "B"},
                "B",
            )
        )
    captured: list[list[str]] = []
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda _orders: list(entries),
    )
    monkeypatch.setattr(manager, "_stationary_release_robot_names", lambda: set())
    monkeypatch.setattr(
        manager,
        "_dispatch_simulated_order_batch",
        lambda group: (
            captured.append([entry[1].name for entry in group]) or 0,
            {entry[0].order_id for entry in group},
        ),
    )

    manager._dispatch_orders()

    assert captured == [["plain"], ["detour"]]


def test_mid_edge_wait_cycle_arms_retreat_when_local_cbs_cannot_start() -> None:
    manager = _manager()
    now = time()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="o1",
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
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
                {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
            ],
            route_clock=1.0,
        ),
    }
    manager.orders = {
        "o1": FleetOrder(order_id="o1", target_lm="B", vehicle="r1", status="EXECUTING"),
        "o2": FleetOrder(order_id="o2", target_lm="A", vehicle="r2", status="EXECUTING"),
    }

    assert not manager._start_async_coupled_replan(
        [manager.robots["r1"], manager.robots["r2"]],
        manager.robots["r1"],
        now,
    )
    assert manager._coupled_replan_failures[("r1", "r2")] == 1
    assert manager.traffic_metrics["coupledReplansFailed"] == 1


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
    assert state["simulationTimeScaleMax"] == 4.0


def test_simulation_time_scale_is_clamped_to_safe_range() -> None:
    manager = _manager()

    assert manager.set_simulation_time_scale(100) == 4.0
    assert manager.set_simulation_time_scale(0) == 1.0


def test_rolling_window_and_prefetch_deadline_scale_with_simulation_speed() -> None:
    manager = _manager()
    manager.params["fleet"].update({
        "rolling_horizon_sec": 30.0,
        "rolling_prefetch_lead_sec": 8.0,
        "rolling_horizon_max_sec": 120.0,
    })

    assert manager._rolling_horizon() == pytest.approx(30.0)
    assert manager._rolling_prefetch_lead() == pytest.approx(8.0)
    manager.set_simulation_time_scale(4.0)
    assert manager._rolling_horizon() == pytest.approx(120.0)
    assert manager._rolling_prefetch_lead() == pytest.approx(32.0)
    assert manager._rolling_prefetch_urgent_lead() == pytest.approx(12.8)


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
