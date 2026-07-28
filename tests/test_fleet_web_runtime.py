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
from fleet_manager.core.traffic.corridor_scheduler import (
    CentralCorridorScheduler,
)
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


def test_runtime_checks_complete_edge_before_leaving_safe_lm(
    monkeypatch,
) -> None:
    clock = [1_000.0]
    monkeypatch.setattr(runtime_module, "time", lambda: clock[0])
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=5.0, y=0.0),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=5.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(5.0, 0.0)),
        properties={"direction": 0},
    )
    manager = FleetManagerSim(
        landmarks,
        [edge],
        params={
            "navigation": {
                "route_speed": 1.0,
                "route_acceleration": 1.0,
                "collision_margin": 0.04,
            },
            "fleet": {
                "runtime_collision_lookahead_sec": 1.0,
                "runtime_collision_preflight_interval_sec": 0.2,
            },
        },
    )
    mover = FleetRobot(
        name="mover",
        current_lm="A",
        target_lm="B",
        status="MOVING",
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
                "t": 5.0,
                "x": 5.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_started_at=clock[0],
        last_tick_at=clock[0],
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 5.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {mover.name: mover, blocker.name: blocker}

    # The blocker is five seconds away, well outside the configured one
    # second lookahead.  The graph-segment guard must still see it before the
    # mover leaves A, instead of stopping the mover midway a few ticks later.
    clock[0] += 0.1
    manager.advance_runtime()

    assert mover.status == "WAITING"
    assert mover.route_clock == pytest.approx(0.0)
    assert mover.pose == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert "blocker" in mover.last_reason


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
        # A priority lease is an arbitration attempt. Resolution is recorded
        # only after the winner actually changes the component geometry.
        "waitCyclesResolved": 0,
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
    # every frame after the stale lease expires. Merely waiting past the
    # bounded lease does not change the physical decision; CBS/evacuation owns
    # the next recovery level.
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
    assert manager.traffic_metrics["priorityGrants"] == first_grants


def test_failed_wait_cycle_lease_is_not_regranted_for_unchanged_snapshot(
    monkeypatch,
) -> None:
    manager = _manager()
    now = time()
    stalled_since = now - 20.0
    lease = manager._deadlock_priority_lease()
    manager.robots = {
        "r1": FleetRobot(
            name="r1",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            last_reason="yield to r2",
            wait_for_robot="r2",
            blocked_since=stalled_since,
            traffic_stall_since=stalled_since,
            traffic_priority_until=now + 0.25,
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
            last_reason="yield to r1",
            wait_for_robot="r1",
            blocked_since=stalled_since,
            traffic_stall_since=stalled_since,
            pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
            trajectory=[
                {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "B"},
                {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A", "lm": "A"},
            ],
        ),
    }
    cycle_key = ("r1", "r2")
    manager._active_wait_cycles[cycle_key] = stalled_since
    manager._wait_cycle_last_arbitration[cycle_key] = now - 0.1
    manager.traffic_metrics["waitCyclesDetected"] = 1
    manager.traffic_metrics["waitCyclesResolved"] = 1
    manager.traffic_metrics["priorityGrants"] = 1
    monkeypatch.setattr(
        manager,
        "_start_async_coupled_replan",
        lambda *_args, **_kwargs: None,
    )

    # The old lease has produced no route-clock progress by the recovery
    # deadline. The maintainer expires it and the arbitration debounce keeps
    # the unchanged 10 Hz snapshot stopped for this frame.
    manager._resolve_runtime_wait_cycles(now)
    assert all(
        robot.traffic_priority_until == 0.0
        for robot in manager.robots.values()
    )
    assert manager.traffic_metrics["priorityGrants"] == 1

    # Merely waiting beyond the nominal lease interval is not a new physical
    # episode. Until a route clock, LM, order, or wait dependency changes, the
    # failed decision must not be counted or armed again.
    manager._resolve_runtime_wait_cycles(now + lease + 0.1)
    assert all(
        robot.traffic_priority_until == 0.0
        for robot in manager.robots.values()
    )
    assert manager.traffic_metrics["waitCyclesDetected"] == 1
    assert manager.traffic_metrics["priorityGrants"] == 1


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
    manager._wait_cycle_grant_signatures[cycle_key] = (
        manager._wait_cycle_grant_signature([winner, peer])
    )
    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda *_args: 0.2,
    )

    manager._record_traffic_progress(winner)

    assert manager._active_wait_cycles[cycle_key] == cycle_started
    assert winner.traffic_stall_since == cycle_started
    assert manager.traffic_metrics["waitCyclesResolved"] == 0

    monkeypatch.setattr(
        manager,
        "_cycle_forward_clearance",
        lambda *_args: 2.0,
    )
    manager._record_traffic_progress(winner)

    assert cycle_key not in manager._active_wait_cycles
    assert manager.traffic_metrics["waitCyclesResolved"] == 1


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


def test_held_runtime_replan_preserves_unchanged_wait_cycle_episode() -> None:
    manager = _manager()
    now = time()
    first = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        active_order_id="o1",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        route_revision=11,
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
        last_reason="replanning route while holding: occupied by r2",
        blocked_since=now,
    )
    second = FleetRobot(
        name="r2",
        current_lm="B",
        target_lm="A",
        active_order_id="o2",
        status="WAITING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        route_clock=0.0,
        route_revision=22,
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
        last_reason="occupied by r1",
        wait_for_robot="r1",
        blocked_since=now,
    )
    manager.robots = {first.name: first, second.name: second}
    manager.orders = {
        "o1": FleetOrder(
            order_id="o1",
            target_lm="B",
            vehicle=first.name,
            assigned_robot=first.name,
            status="PLANNING",
        ),
        "o2": FleetOrder(
            order_id="o2",
            target_lm="A",
            vehicle=second.name,
            assigned_robot=second.name,
            status="EXECUTING",
        ),
    }
    cycle_key = tuple(sorted(manager.robots))
    cycle_started = now - 5.0
    manager._active_wait_cycles[cycle_key] = cycle_started
    manager._wait_cycle_last_arbitration[cycle_key] = now - 1.0
    manager._wait_cycle_grant_signatures[cycle_key] = (
        manager._wait_cycle_grant_signature([first, second])
    )
    manager._coupled_replan_last_attempt[cycle_key] = now - 0.5
    manager._coupled_replan_failures[cycle_key] = 1
    manager._runtime_replans[first.name] = {
        "order_id": "o1",
        "start_lm": "A",
        "route_revision": 11,
        "route_clock": 0.0,
        "blocker_names": ("r2",),
        "stage": "planning",
    }

    manager._resolve_runtime_wait_cycles(now)

    assert manager._active_wait_cycles[cycle_key] == cycle_started
    assert cycle_key in manager._wait_cycle_last_arbitration
    assert cycle_key in manager._wait_cycle_grant_signatures
    assert manager._coupled_replan_last_attempt[cycle_key] == now - 0.5
    assert manager._coupled_replan_failures[cycle_key] == 1

    # Real route-clock progress invalidates the captured transaction geometry
    # and lets normal cycle cleanup run immediately.
    first.route_clock = 0.1
    manager._resolve_runtime_wait_cycles(now + 0.1)

    assert cycle_key not in manager._active_wait_cycles
    assert cycle_key not in manager._wait_cycle_last_arbitration
    assert cycle_key not in manager._wait_cycle_grant_signatures
    assert cycle_key not in manager._coupled_replan_last_attempt
    assert cycle_key not in manager._coupled_replan_failures


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


def test_failed_rolling_request_replaces_waiting_blocker_future_with_body(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    requester = FleetRobot(
        name="requester",
        current_lm="A",
        target_lm="A",
        status="WAITING",
        active_order_id="requester-order",
        route_chunk_goal_lm="A",
        route_revision=3,
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
            {
                "t": 1.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
        ],
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="C",
        target_lm="A",
        status="WAITING",
        active_order_id="blocker-order",
        route_chunk_goal_lm="A",
        route_revision=7,
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "C->B",
                "lm": "C",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "C->B",
                "lm": "B",
            },
            {
                "t": 4.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "A",
            },
        ],
    )
    manager.robots = {
        requester.name: requester,
        blocker.name: blocker,
    }
    manager._rolling_prefetch_blockers[requester.name] = {
        "requester": (
            requester.route_revision,
            requester.route_chunk_goal_lm,
            requester.active_order_id,
        ),
        "blockers": {
            blocker.name: (
                blocker.route_revision,
                blocker.route_chunk_goal_lm,
                blocker.active_order_id,
            ),
        },
    }
    request = {
        "name": requester.name,
        "startLm": "A",
        "goalLm": "B",
        "startPose": dict(requester.pose),
    }
    captured: list[dict] = []
    monkeypatch.setattr(
        manager.planner,
        "plan",
        lambda payload: (
            captured.append(payload)
            or {"ok": False, "plans": [], "debug": {"reason": "capture"}}
        ),
    )

    assert manager._release_blocker_names_for_requests([request]) == {
        blocker.name
    }
    manager._plan_valid_requests([request], {"robots": [request]})

    assert len(captured) == 1
    assert all(
        interval["robot"] != blocker.name
        for interval in captured[0]["reserved_edge_intervals"]
    )
    assert [
        interval
        for interval in captured[0]["reserved_vertex_intervals"]
        if interval["robot"] == blocker.name
    ] == [{
        "node": "C",
        "start": 0.0,
        "end": manager._reservation_horizon(),
        "robot": blocker.name,
    }]


def test_terminal_departure_holds_complete_upstream_wait_chain_as_bodies(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    positions = {
        "terminal": ("A", -2.0, 0.0),
        "direct": ("B", 0.0, 0.0),
        "middle": ("C", 2.0, 0.0),
        "upstream": ("P", 0.0, 2.0),
    }

    def waiting_robot(name: str, dependency: str, future_lm: str) -> FleetRobot:
        lm_name, x, y = positions[name]
        target = manager.landmarks[future_lm]
        return FleetRobot(
            name=name,
            current_lm=lm_name,
            status="WAITING",
            last_reason=f"occupied by {dependency}",
            wait_for_robot=dependency,
            pose={"x": x, "y": y, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": x, "y": y, "edgeId": f"{lm_name}->{future_lm}", "lm": lm_name},
                {"t": 2.0, "x": target.x, "y": target.y, "edgeId": f"{lm_name}->{future_lm}", "lm": future_lm},
            ],
        )

    terminal = FleetRobot(
        name="terminal",
        current_lm="A",
        status="ARRIVED",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {
        terminal.name: terminal,
        "direct": waiting_robot("direct", "terminal", "P"),
        "middle": waiting_robot("middle", "direct", "B"),
        "upstream": waiting_robot("upstream", "middle", "B"),
    }
    request = {
        "name": terminal.name,
        "startLm": "A",
        "goalLm": "C",
        "startPose": dict(terminal.pose),
    }
    captured: list[dict] = []
    monkeypatch.setattr(
        manager.planner,
        "plan",
        lambda payload: (
            captured.append(payload)
            or {"ok": False, "plans": [], "debug": {"reason": "capture"}}
        ),
    )

    assert manager._release_blocker_names_for_requests([request]) == {
        "direct",
        "middle",
        "upstream",
    }
    manager._plan_valid_requests([request], {"robots": [request]})

    assert len(captured) == 1
    held_names = {"direct", "middle", "upstream"}
    assert not any(
        interval["robot"] in held_names
        for interval in captured[0]["reserved_edge_intervals"]
    )
    held_vertices = {
        interval["robot"]: interval["node"]
        for interval in captured[0]["reserved_vertex_intervals"]
        if interval["robot"] in held_names
    }
    assert held_vertices == {
        "direct": "B",
        "middle": "C",
        "upstream": "P",
    }


def test_stationary_departure_ignores_yielding_waiter_suffix_but_not_body(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    terminal = FleetRobot(
        name="terminal",
        current_lm="A",
        status="IDLE",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    held = FleetRobot(
        name="held",
        current_lm="C",
        target_lm="A",
        status="WAITING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "C->B",
                "lm": "C",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "C->B",
                "lm": "B",
            },
            {
                "t": 4.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "A",
            },
        ],
        last_reason="yield to terminal",
        wait_for_robot="terminal",
    )
    manager.robots = {terminal.name: terminal, held.name: held}
    request = {
        "name": terminal.name,
        "startLm": "A",
        "goalLm": "B",
        "startPose": dict(terminal.pose),
    }

    def departure_plan(_payload):
        return {
            "ok": True,
            "plans": [{
                "robot": terminal.name,
                "startLm": "A",
                "goalLm": "B",
                "nodes": ["A", "B"],
                "trajectory": [
                    {
                        "t": 0.0,
                        "x": -2.0,
                        "y": 0.0,
                        "yaw": 0.0,
                        "edgeId": "A->B",
                        "lm": "A",
                    },
                    {
                        "t": 2.0,
                        "x": 0.0,
                        "y": 0.0,
                        "yaw": 0.0,
                        "edgeId": "A->B",
                        "lm": "B",
                    },
                ],
            }],
            "debug": {"reason": "success"},
        }

    monkeypatch.setattr(manager.planner, "plan", departure_plan)

    result = manager._plan_valid_requests([request], {"robots": [request]})

    # The held robot explicitly yields and therefore remains at C. Its stale
    # C->B->A suffix must not collide with the terminal's A->B departure.
    assert result["ok"]
    assert result["plans"]
    # The same held robot is still a physical body. A plan that reaches its
    # actual stopped pose must remain a continuous collision.
    collision = manager._first_continuous_corridor_conflict(
        terminal.name,
        [
            {"t": 0.0, "x": -2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->C"},
            {"t": 4.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->C"},
        ],
        stationary_robot_names={held.name},
    )
    assert collision is not None
    assert collision["other"] == held.name


def test_clearance_departure_ignores_transactional_waiter_suffix(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    terminal = FleetRobot(
        name="terminal",
        current_lm="A",
        status="IDLE",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    held = FleetRobot(
        name="held",
        current_lm="C",
        target_lm="A",
        status="WAITING",
        active_order_id="held-order",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "C->B",
                "lm": "C",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "C->B",
                "lm": "B",
            },
            {
                "t": 4.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->A",
                "lm": "A",
            },
        ],
        route_revision=9,
    )
    held_order = FleetOrder(
        order_id="held-order",
        target_lm="A",
        vehicle=held.name,
        assigned_robot=held.name,
        status="WAITING_OBSTACLE",
    )
    manager.robots = {terminal.name: terminal, held.name: held}
    manager.orders = {held_order.order_id: held_order}
    manager._runtime_replans[held.name] = {
        "order_id": held_order.order_id,
        "start_lm": "C",
        "route_revision": held.route_revision,
        "route_clock": held.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "blocker_names": (terminal.name,),
        "stage": "retry",
    }
    request = {
        "name": terminal.name,
        "startLm": "A",
        "goalLm": "B",
        "startPose": dict(terminal.pose),
    }
    captured: list[dict] = []

    def capture_plan(payload):
        captured.append(payload)
        return {"ok": False, "plans": [], "debug": {"reason": "capture"}}

    monkeypatch.setattr(manager.planner, "plan", capture_plan)

    manager._plan_valid_requests([request], {"robots": [request]})

    assert manager._release_blocker_names_for_requests([request]) == {held.name}
    assert len(captured) == 1
    assert all(
        interval["robot"] != held.name
        for interval in captured[0]["reserved_edge_intervals"]
    )
    assert [
        interval
        for interval in captured[0]["reserved_vertex_intervals"]
        if interval["robot"] == held.name
    ] == [{
        "node": "C",
        "start": 0.0,
        "end": manager._reservation_horizon(),
        "robot": held.name,
    }]


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


def test_existing_stationary_departure_replan_does_not_refresh_evacuation_lease(
) -> None:
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
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="B",
        route_final_lm="B",
        status="WAITING",
        active_order_id=waiter_order.order_id,
        last_reason="occupied by blocker",
        wait_for_robot="blocker",
        blocked_since=now - 10.0,
        traffic_stall_since=now - 10.0,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->A", "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_clock=0.0,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}

    assert manager._evacuate_for_failed_stationary_departure(
        waiter,
        blocker,
        now,
    )
    transaction = manager._runtime_replans[waiter.name]
    first_lease = waiter.traffic_priority_until
    first_events = sum(
        "evacuating for queued departure blocker" in event.message
        for event in manager.events
    )
    assert first_events == 1
    assert waiter_order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["cycleReplans"] == 1

    # The graph-stable encounter is observed again after the debounce period,
    # but its transactional replan is still the same object. It remains
    # handled without pretending that another evacuation started.
    later = now + manager._wait_cycle_recovery_cooldown() + 0.01
    assert manager._evacuate_for_failed_stationary_departure(
        waiter,
        blocker,
        later,
    )
    assert manager._runtime_replans[waiter.name] is transaction
    assert waiter.traffic_priority_until == first_lease
    assert waiter_order.traffic_detour_attempts == 1
    assert manager.traffic_metrics["cycleReplans"] == 1
    assert sum(
        "evacuating for queued departure blocker" in event.message
        for event in manager.events
    ) == first_events


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


def test_runtime_replans_active_order_that_lost_its_trajectory_in_place(monkeypatch) -> None:
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

    assert robot.status == "WAITING"
    assert robot.active_order_id == order.order_id
    assert robot.traffic_priority_until == 0.0
    assert order.status == "WAITING_OBSTACLE"
    assert order.start_lm == "A"
    assert manager._runtime_replans[robot.name]["stage"] == "queued"


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


def test_deadlock_retreat_rejects_a_robot_on_an_intermediate_lm(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    now = time()
    winner = FleetRobot(
        name="winner",
        current_lm="B",
        status="WAITING",
        active_order_id="winner-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        last_reason="yield to retreater",
    )
    retreater = FleetRobot(
        name="retreater",
        current_lm="C",
        target_lm="A",
        status="WAITING",
        active_order_id="retreater-order",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "A->B",
                "lm": "B",
            },
            {
                "t": 4.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->C",
                "lm": "C",
            },
        ],
        route_clock=4.0,
    )
    manager.robots = {
        winner.name: winner,
        retreater.name: retreater,
    }
    manager.orders = {
        "winner-order": FleetOrder(
            order_id="winner-order",
            target_lm="C",
            vehicle=winner.name,
            assigned_robot=winner.name,
            status="EXECUTING",
        ),
        "retreater-order": FleetOrder(
            order_id="retreater-order",
            target_lm="A",
            vehicle=retreater.name,
            assigned_robot=retreater.name,
            status="EXECUTING",
        ),
    }
    # Reproduce a multi-LM portal escape: the final A pocket is clear, but B
    # on the reverse sweep is occupied by the robot waiting for this one.
    monkeypatch.setattr(
        manager,
        "_previous_trajectory_lm",
        lambda robot: (0.0, "A") if robot.name == retreater.name else None,
    )

    assert manager._deadlock_retreat_target_blocker(retreater, 0.0) == ""
    assert manager._deadlock_retreat_path_blocker(retreater, 0.0) == winner.name
    assert manager._start_deadlock_corridor_evacuation(
        [winner, retreater],
        winner,
        now,
    ) == ""
    assert retreater.retreat_target_clock is None
    assert retreater.status == "WAITING"


def test_blocked_retreat_timeout_replans_same_order_and_breaks_mutual_wait(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    now = time()
    retreater = FleetRobot(
        name="retreater",
        current_lm="C",
        target_lm="A",
        status="RETREATING",
        active_order_id="retreater-order",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {
                "t": 0.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "A->B",
                "lm": "B",
            },
            {
                "t": 4.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi,
                "edgeId": "B->C",
                "lm": "C",
            },
        ],
        plan_nodes=["A", "B", "C"],
        route_clock=4.0,
        route_revision=7,
        retreat_target_clock=0.0,
        retreat_target_lm="A",
        retreat_blocked_edges=[("C", "P"), ("P", "C")],
        blocked_since=now - 10.0,
        traffic_stall_since=now - 10.0,
        last_tick_at=now - 0.1,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="WAITING",
        active_order_id="blocker-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        last_reason="yield to retreater",
        wait_for_robot="retreater",
        wait_resource="B->C",
        blocked_since=now - 10.0,
        traffic_stall_since=now - 10.0,
    )
    order = FleetOrder(
        order_id="retreater-order",
        target_lm="A",
        vehicle=retreater.name,
        assigned_robot=retreater.name,
        status="EXECUTING",
    )
    manager.robots = {
        retreater.name: retreater,
        blocker.name: blocker,
    }
    manager.orders = {
        order.order_id: order,
        "blocker-order": FleetOrder(
            order_id="blocker-order",
            target_lm="C",
            vehicle=blocker.name,
            assigned_robot=blocker.name,
            status="EXECUTING",
        ),
    }
    monkeypatch.setattr(
        manager,
        "_blocked_at_clock",
        lambda *_args, **_kwargs: "yield to blocker",
    )
    # Force the degree-two production fallback: no side pocket is reachable
    # without crossing the exact blocker, so preserve the active order and
    # transactionally replace its route from the current LM.
    monkeypatch.setattr(manager, "_stationary_clearance_route", lambda *_args, **_kwargs: [])

    manager._advance_deadlock_retreat(retreater, now)

    assert retreater.active_order_id == order.order_id
    assert retreater.retreat_target_clock is None
    assert retreater.status == "WAITING"
    assert retreater.last_reason.startswith("replanning route while holding:")
    assert order.status == "PLANNING"
    assert order.traffic_detour_edges == [("B", "C"), ("C", "B")]
    assert order.traffic_detour_attempts == 1
    assert manager._runtime_replans[retreater.name]["order_id"] == order.order_id
    assert manager.traffic_metrics["cycleReplans"] == 1
    # The old peer dependency is invalidated in the same transaction.  It is
    # re-evaluated by collision preflight instead of recreating A<->B forever.
    assert blocker.wait_for_robot == ""
    assert blocker.last_reason == "retreat dependency changed; collision preflight pending"
    assert blocker.blocked_since is None


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


def test_previous_trajectory_lm_skips_current_lm_during_planned_wait() -> None:
    manager = _clearance_manager()
    robot = FleetRobot(
        name="r1",
        current_lm="B",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": -2.0,
                "y": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
            {
                "t": 3.0,
                "x": 0.0,
                "y": 0.0,
                "edgeId": "B->B",
                "lm": "B",
            },
            {
                "t": 5.0,
                "x": 2.0,
                "y": 0.0,
                "edgeId": "B->C",
                "lm": "C",
            },
        ],
        route_clock=2.5,
    )

    assert manager._previous_trajectory_lm(robot) == (0.0, "A")
    assert manager._deadlock_detour_edges(robot) == [
        ("B", "C"),
        ("C", "B"),
    ]


def test_previous_trajectory_lm_uses_older_external_stop_before_corridor() -> None:
    region = "corridor:test"
    landmarks = {
        "X": Landmark(
            name="X",
            x=0.0,
            y=0.0,
            properties={"can_wait": True},
        ),
        "I": Landmark(
            name="I",
            x=1.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": region,
            },
        ),
        "E": Landmark(
            name="E",
            x=3.0,
            y=0.0,
            properties={"can_wait": True},
        ),
    }
    edges = [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=abs(landmarks[target].x - landmarks[source].x),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[source].to_point(),
                landmarks[target].to_point(),
            ),
            properties={
                "direction": 2,
                "controlled_region": region,
            },
        )
        for source, target in (("X", "I"), ("I", "E"))
    ]
    manager = FleetManagerSim(landmarks, edges)
    # E and X are external waiting LMs, while I is an internal no-wait LM.
    # The body has reached E and is waiting there for a future edge.  Recovery
    # must not return E again and must not stop at I; it reverses to X.
    robot = FleetRobot(
        name="r1",
        current_lm="E",
        pose={"x": 3.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "edgeId": "X->X",
                "lm": "X",
            },
            {
                "t": 1.0,
                "x": 1.0,
                "y": 0.0,
                "edgeId": "X->I",
                "lm": "I",
            },
            {
                "t": 2.0,
                "x": 3.0,
                "y": 0.0,
                "edgeId": "I->E",
                "lm": "E",
            },
            {
                "t": 3.0,
                "x": 3.0,
                "y": 0.0,
                "edgeId": "E->E",
                "lm": "E",
            },
        ],
        route_clock=2.5,
    )

    assert manager._previous_trajectory_lm(robot) == (0.0, "X")


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


def test_internal_corridor_without_safe_retreat_keeps_executable_route() -> None:
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

    assert evacuated == ""
    assert order.status == "EXECUTING"
    assert order.traffic_detour_attempts == 0
    assert manager.traffic_metrics["cycleReplans"] == 0
    assert robot.status == "WAITING"
    assert robot.active_order_id == order.order_id
    assert robot.trajectory
    assert robot.retreat_target_clock is None
    assert (
        manager._start_deadlock_corridor_evacuation(
            [winner, robot],
            winner,
            now + 0.1,
        )
        == ""
    )
    assert order.traffic_detour_attempts == 0
    assert manager.traffic_metrics["cycleReplans"] == 0


def test_identical_deadlock_detour_is_queued_and_counted_once() -> None:
    manager = _manager()
    now = time()
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="IDLE",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
    )
    robot = FleetRobot(
        name="robot",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="robot-order",
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
    order = FleetOrder(
        order_id="robot-order",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="EXECUTING",
        spatial_route_nodes=["A", "B"],
    )
    manager.robots = {blocker.name: blocker, robot.name: robot}
    manager.orders = {order.order_id: order}

    assert manager._start_deadlock_corridor_evacuation(
        [blocker, robot],
        blocker,
        now,
    ) == robot.name
    assert manager.traffic_metrics["cycleReplans"] == 1
    assert order.traffic_detour_attempts == 1
    transaction = manager._runtime_replans[robot.name]
    assert transaction["retained_route_superseded"] is True
    assert order.spatial_route_nodes == []
    original_stage = transaction["stage"]
    transaction["stage"] = "retry"
    assert manager._runtime_replan_holds_robot(robot)
    transaction["stage"] = original_stage

    # The wait-for graph is temporarily hidden by the transactional planning
    # status, then the retained route exposes the same graph-stable encounter.
    # This is one recovery episode, not a new detour on every runtime frame.
    assert manager._start_deadlock_corridor_evacuation(
        [blocker, robot],
        blocker,
        now + 0.1,
    ) == ""
    assert manager._runtime_replans[robot.name] is transaction
    assert manager.traffic_metrics["cycleReplans"] == 1
    assert order.traffic_detour_attempts == 1

    # Even after the debounce interval, the idempotent queue result means the
    # still-existing transaction is handled but is not counted as new work.
    assert manager._start_deadlock_corridor_evacuation(
        [blocker, robot],
        blocker,
        now + manager._wait_cycle_recovery_cooldown() + 0.01,
    ) == robot.name
    assert manager._runtime_replans[robot.name] is transaction
    assert manager.traffic_metrics["cycleReplans"] == 1
    assert order.traffic_detour_attempts == 1


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
    assert retreater.status == "WAITING"
    assert retreater.active_order_id == "o2"
    assert retreater.trajectory
    assert order.status == "PLANNING"
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


def test_runtime_safety_telemetry_records_endpoint_overlap(monkeypatch) -> None:
    manager = _manager()
    now = time()
    first = FleetRobot(
        name="first",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
        ],
        route_clock=0.1,
        route_revision=11,
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A"},
            {"t": 1.0, "x": 1.0, "y": 0.0, "yaw": math.pi, "edgeId": "B->A"},
        ],
        route_clock=0.2,
        route_revision=22,
    )
    manager.robots = {first.name: first, second.name: second}
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    first.pose = {"x": 0.9, "y": 0.0, "yaw": 0.0}
    first.route_clock = 0.5
    second.pose = {"x": 1.1, "y": 0.0, "yaw": math.pi}
    second.route_clock = 0.6
    # Keep this regression specific to the endpoint classifier. Swept-only
    # telemetry is covered separately below.
    monkeypatch.setattr(manager, "_swept_footprints_overlap", lambda *_: False)

    manager._enforce_runtime_safety_invariant(snapshots, now)

    telemetry = manager._last_runtime_safety_rollback
    assert telemetry is not None
    assert telemetry["sequence"] == 1
    assert telemetry["pairCount"] == 1
    pair = telemetry["pairs"][0]
    assert pair["robots"] == ["first", "second"]
    assert pair["kind"] == "endpoint"
    assert pair["before"]["first"] == {
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "currentLm": "A",
        "status": "MOVING",
        "routeClock": 0.1,
        "routeRevision": 11,
        "edgeId": "A->B",
    }
    assert pair["proposed"]["second"]["pose"]["x"] == pytest.approx(1.1)
    assert pair["proposed"]["second"]["routeClock"] == pytest.approx(0.6)
    assert pair["proposed"]["second"]["edgeId"] == "B->A"
    assert manager.snapshot()["lastRuntimeSafetyRollback"] == telemetry


def test_runtime_safety_telemetry_records_swept_swap() -> None:
    manager = _manager()
    now = time()
    first = FleetRobot(
        name="first",
        current_lm="A",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[{"t": 0.0, "edgeId": "A->B"}],
        route_revision=31,
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        status="MOVING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[{"t": 0.0, "edgeId": "B->A"}],
        route_revision=32,
    )
    manager.robots = {first.name: first, second.name: second}
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    # Safe endpoints have swapped sides, but their swept bodies cross between
    # the two physics snapshots.
    first.pose = {"x": 2.0, "y": 0.0, "yaw": 0.0}
    second.pose = {"x": 0.0, "y": 0.0, "yaw": math.pi}

    manager._enforce_runtime_safety_invariant(snapshots, now)

    telemetry = manager._last_runtime_safety_rollback
    assert telemetry is not None
    assert telemetry["pairs"][0]["robots"] == ["first", "second"]
    assert telemetry["pairs"][0]["kind"] == "swept"
    assert telemetry["pairs"][0]["before"]["second"]["currentLm"] == "B"
    assert telemetry["pairs"][0]["proposed"]["first"]["pose"]["x"] == 2.0


def test_runtime_safety_telemetry_ignores_preexisting_overlap() -> None:
    manager = _manager()
    now = time()
    first = FleetRobot(
        name="first",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    second = FleetRobot(
        name="second",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.2, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {first.name: first, second.name: second}
    snapshots = {
        name: manager._runtime_safety_snapshot(robot)
        for name, robot in manager.robots.items()
    }
    first.pose = {"x": 0.05, "y": 0.0, "yaw": 0.0}
    second.pose = {"x": 0.25, "y": 0.0, "yaw": 0.0}

    manager._enforce_runtime_safety_invariant(snapshots, now)

    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 0
    assert manager._last_runtime_safety_rollback is None
    assert manager.snapshot()["lastRuntimeSafetyRollback"] is None


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
    queued: list[tuple[str, str]] = []

    def queue_replan(robot, _now, reason, **kwargs):
        queued.append((robot.name, reason))
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


def test_immediate_substep_cannot_consume_a_moving_peers_current_body(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params["robot_model"] = {
        "footprint": [
            {"x": -0.523, "y": -0.3532},
            {"x": 0.477, "y": -0.3532},
            {"x": 0.477, "y": 0.3468},
            {"x": -0.523, "y": 0.3468},
        ],
    }
    manager.params.setdefault("navigation", {})["collision_margin"] = 0.04
    manager.collision.set_params(manager.params)
    manager._simulation_time_scale = 2.0
    now = 1_000.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    follower = FleetRobot(
        name="follower",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id="follower-order",
        pose={"x": 20.077, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 20.077, "y": 0.0, "yaw": 0.0, "t": 0.0},
            {"x": 21.077, "y": 0.0, "yaw": 0.0, "t": 1.0},
        ],
        route_clock=0.0,
        last_tick_at=now - 0.40,
    )
    leader = FleetRobot(
        name="leader",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        active_order_id="leader-order",
        pose={"x": 21.438, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"x": 21.438, "y": 0.0, "yaw": math.pi, "t": 0.0},
            {"x": 22.438, "y": 0.0, "yaw": math.pi, "t": 1.0},
        ],
        route_clock=0.0,
        last_tick_at=now - 0.40,
    )
    manager.robots = {follower.name: follower, leader.name: leader}
    manager.orders = {
        "follower-order": FleetOrder(
            order_id="follower-order",
            target_lm="B",
            vehicle=follower.name,
            assigned_robot=follower.name,
            status="EXECUTING",
        ),
        "leader-order": FleetOrder(
            order_id="leader-order",
            target_lm="A",
            vehicle=leader.name,
            assigned_robot=leader.name,
            status="EXECUTING",
        ),
    }
    monkeypatch.setattr(manager, "_has_right_of_way", lambda *_args: False)
    original_blocked_ahead = manager._blocked_ahead

    def stop_leader_after_follower_was_processed(robot, proposed_clock):
        if robot.name == leader.name:
            return "planned traffic wait at B"
        return original_blocked_ahead(robot, proposed_clock)

    monkeypatch.setattr(manager, "_blocked_ahead", stop_leader_after_follower_was_processed)

    candidate = manager._pose_at_trajectory(follower.trajectory, 0.40)
    predicted_leader = manager._predicted_robot_pose(leader, 0.40)
    assert candidate is not None and predicted_leader is not None
    assert not manager.collision.footprints_overlap(candidate, predicted_leader)
    assert manager.collision.footprints_overlap(candidate, leader.pose)

    # The leader's future motion is only a prediction: its own check can stop
    # it later in this tick. The follower must stop before its current body,
    # so the post-commit invariant never has to roll both poses back.
    manager._advance_runtime()

    assert follower.status == "WAITING"
    assert follower.last_reason == "yield to leader"
    assert follower.route_clock > 2.0 * manager._runtime_motion_step()
    assert follower.route_clock < 0.40
    assert leader.status == "WAITING"
    assert leader.pose["x"] == pytest.approx(21.438)
    assert not manager.collision.footprints_overlap(follower.pose, leader.pose)
    assert manager.traffic_metrics["runtimeSafetyRollbacks"] == 0


def test_immediate_substep_can_escape_a_preexisting_physical_overlap(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params["robot_model"] = {
        "footprint": [
            {"x": -0.523, "y": -0.3532},
            {"x": 0.477, "y": -0.3532},
            {"x": 0.477, "y": 0.3468},
            {"x": -0.523, "y": 0.3468},
        ],
    }
    manager.params.setdefault("navigation", {})["collision_margin"] = 0.04
    manager.collision.set_params(manager.params)
    mover = FleetRobot(
        name="mover",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id="mover-order",
        pose={"x": 20.477, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"x": 20.477, "y": 0.0, "yaw": 0.0, "t": 0.0},
            {"x": 19.477, "y": 0.0, "yaw": 0.0, "t": 1.0},
        ],
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="WAITING",
        pose={"x": 21.438, "y": 0.0, "yaw": math.pi},
    )
    manager.robots = {mover.name: mover, blocker.name: blocker}
    manager.orders = {
        "mover-order": FleetOrder(
            order_id="mover-order",
            target_lm="B",
            vehicle=mover.name,
            assigned_robot=mover.name,
            status="EXECUTING",
        ),
    }
    manager._runtime_tick_route_clocks = {mover.name: 0.0, blocker.name: 0.0}
    monkeypatch.setattr(manager, "_has_right_of_way", lambda *_args: False)

    candidate = manager._pose_at_trajectory(mover.trajectory, 0.05)
    assert candidate is not None
    assert manager.collision.footprints_overlap(mover.pose, blocker.pose)
    assert manager._candidate_moves_away(mover.pose, candidate, blocker.pose)

    assert manager._blocked_at_clock(mover, 0.05) == ""


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


def test_completed_chunk_repairs_stale_goal_from_physical_endpoint() -> None:
    manager = _long_line_manager(edge_count=3)
    now = manager.simulation_time()
    order = FleetOrder(
        order_id="o1",
        target_lm="N3",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="N0",
        # Reproduces a legacy rolling trim which advertised the following LM
        # although its executable trajectory stopped at N1.
        target_lm="N2",
        status="MOVING",
        pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "t": 0.0,
                "lm": "N0",
            },
            {
                "x": 1.2,
                "y": 0.0,
                "yaw": 0.0,
                "t": 1.0,
                "lm": "N1",
            },
        ],
        plan_nodes=["N0", "N1"],
        route_clock=1.0,
        active_order_id=order.order_id,
        route_chunk_goal_lm="N2",
        route_final_lm="N3",
        route_revision=7,
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot

    manager._settle_completed_trajectory_endpoint(robot, now)

    assert robot.current_lm == "N1"
    assert robot.target_lm == "N1"
    assert robot.route_chunk_goal_lm == "N1"
    assert robot.route_final_lm == "N3"
    assert robot.route_revision != 7
    assert manager._complete_simulated_route_chunk(robot, now)
    assert robot.status == "WAITING"
    assert robot.last_reason == "rolling continuation pending"


def test_zero_duration_route_at_active_target_completes_instead_of_false_moving(
    monkeypatch,
) -> None:
    manager = _manager()
    now = manager._now()
    order = FleetOrder(
        order_id="o1",
        target_lm="A",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="A",
        status="MOVING",
        active_order_id=order.order_id,
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
        trajectory=[{
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            # A degenerate planner sample must not instantaneously rotate the
            # physical robot from its already committed heading.
            "yaw": 0.0,
            "edgeId": "A->A",
            "lm": "A",
        }],
        plan_nodes=["A"],
        route_started_at=now,
        last_tick_at=now,
        route_revision=17,
        route_chunk_goal_lm="A",
        route_final_lm="A",
        route_preview=[{"x": 0.0, "y": 0.0, "phase": "committed"}],
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    monkeypatch.setattr(manager, "_dispatch_orders", lambda *args, **kwargs: 0)

    manager._advance_runtime()

    assert order.status == "COMPLETED"
    assert robot.status == "ARRIVED"
    assert robot.current_lm == "A"
    assert robot.pose["yaw"] == pytest.approx(math.pi)
    assert robot.active_order_id == ""
    assert robot.target_lm == ""
    assert robot.trajectory == []
    assert robot.route_revision == 0
    assert robot.route_chunk_goal_lm == ""
    assert robot.route_final_lm == ""


def test_zero_duration_route_away_from_target_holds_for_transactional_replan(
    monkeypatch,
) -> None:
    manager = _manager()
    now = manager._now()
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id=order.order_id,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[{
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "A->A",
            "lm": "A",
        }],
        plan_nodes=["A"],
        route_started_at=now,
        last_tick_at=now,
        route_revision=23,
        route_chunk_goal_lm="B",
        route_final_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    monkeypatch.setattr(manager, "_dispatch_orders", lambda *args, **kwargs: 0)

    manager._advance_runtime()

    assert robot.status == "WAITING"
    assert robot.status != "MOVING"
    assert robot.current_lm == "A"
    assert robot.active_order_id == order.order_id
    assert robot.trajectory
    assert order.status == "PLANNING"
    assert manager._runtime_replans[robot.name]["order_id"] == order.order_id
    assert manager._runtime_replans[robot.name]["route_revision"] == 23


def test_zero_duration_rolling_boundary_holds_for_continuation(
    monkeypatch,
) -> None:
    manager = _long_line_manager(edge_count=2)
    now = manager._now()
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
        active_order_id=order.order_id,
        pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        trajectory=[{
            "t": 0.0,
            "x": 1.2,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "N1->N1",
            "lm": "N1",
        }],
        plan_nodes=["N1"],
        route_started_at=now,
        last_tick_at=now,
        route_chunk_goal_lm="N1",
        route_final_lm="N2",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    monkeypatch.setattr(manager, "_dispatch_orders", lambda *args, **kwargs: 0)

    manager._advance_runtime()

    assert robot.status == "WAITING"
    assert robot.last_reason == "rolling continuation pending"
    assert robot.active_order_id == order.order_id
    assert robot.trajectory
    assert order.status == "PLANNING"


def test_positive_duration_wait_only_trajectory_remains_a_planned_wait(
    monkeypatch,
) -> None:
    manager = _manager()
    now = manager._now()
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id=order.order_id,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->A", "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->A", "lm": "A"},
            {"t": 4.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        plan_nodes=["A", "A", "B"],
        route_clock=0.5,
        route_started_at=now - 0.5,
        last_tick_at=now,
        route_chunk_goal_lm="B",
        route_final_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    monkeypatch.setattr(manager, "_dispatch_orders", lambda *args, **kwargs: 0)

    manager._advance_runtime()

    assert robot.status == "WAITING"
    assert robot.last_reason == "planned traffic wait at A"
    assert order.status == "WAITING_TRAFFIC"
    assert robot.name not in manager._runtime_replans
    assert robot.trajectory[-1]["t"] == pytest.approx(4.0)


def test_wait_timeout_replan_keeps_the_active_route_transactionally(monkeypatch) -> None:
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

    assert manager.orders["o1"].status == "PLANNING"
    assert manager.orders["o1"].error.startswith("runtime replan pending:")
    assert manager.orders["o1"].dispatch_failures == 0
    assert robot.active_order_id == "o1"
    assert robot.status == "WAITING"
    assert robot.target_lm == "B"
    assert robot.trajectory
    assert robot.last_reason.startswith("replanning route while holding:")
    assert manager._runtime_replans[robot.name]["start_lm"] == "A"


def test_transactional_runtime_replan_commits_without_idle_route_gap() -> None:
    manager = _manager()
    now = manager._now()
    old_trajectory = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 4.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
    ]
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=list(old_trajectory),
        plan_nodes=["A", "B"],
        route_revision=7,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}

    assert manager._queue_active_order_for_background_replan(
        robot,
        now,
        "occupied by parked",
    )
    assert robot.trajectory == old_trajectory
    assert robot.active_order_id == order.order_id
    state = manager._runtime_replans[robot.name]
    state["stage"] = "planning"
    replacement = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
    ]
    committed = manager._finish_async_runtime_replan({
        "kind": "runtime_replan",
        "order_id": order.order_id,
        "robot_name": robot.name,
        "generation": state["generation"],
        "route_revision": robot.route_revision,
        "route_clock": robot.route_clock,
        "start_lm": "A",
        "final_goal": "B",
        "result": {
            "ok": True,
            "plans": [{
                "robot": robot.name,
                "startLm": "A",
                "goalLm": "B",
                "finalGoalLm": "B",
                "nodes": ["A", "B"],
                "trajectory": replacement,
            }],
            "debug": {"reason": "success"},
        },
    })

    assert committed == 1
    assert robot.active_order_id == order.order_id
    assert robot.status == "MOVING"
    assert robot.trajectory == replacement
    assert robot.pose == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert robot.name not in manager._runtime_replans
    assert order.status == "EXECUTING"


def test_failed_transactional_runtime_replan_retains_route_with_backoff() -> None:
    manager = _manager()
    now = manager._now()
    trajectory = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
    ]
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=list(trajectory),
        plan_nodes=["A", "B"],
        route_revision=3,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}
    assert manager._queue_active_order_for_background_replan(
        robot,
        now,
        "occupied by parked",
    )
    state = manager._runtime_replans[robot.name]
    state["stage"] = "planning"

    assert manager._finish_async_runtime_replan({
        "order_id": order.order_id,
        "robot_name": robot.name,
        "generation": state["generation"],
        "route_revision": robot.route_revision,
        "route_clock": robot.route_clock,
        "start_lm": "A",
        "final_goal": "B",
        "result": {"ok": False, "plans": [], "debug": {"reason": "no_sipp_path"}},
    }) == 0

    assert robot.active_order_id == order.order_id
    assert robot.trajectory == trajectory
    assert robot.status == "WAITING"
    assert state["stage"] == "retry"
    assert state["retry_at"] > now
    assert order.status == "WAITING_OBSTACLE"
    assert order.dispatch_failures == 1
    assert manager._ready_runtime_replan_entry(now) is None


def test_post_evacuation_replan_never_resumes_superseded_route() -> None:
    manager = _manager()
    now = manager._now()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
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
        route_revision=3,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}

    assert manager._queue_active_order_for_background_replan(
        robot,
        now,
        "deadlock corridor evacuated; alternate route required",
    )
    state = manager._runtime_replans[robot.name]
    assert state["retained_route_superseded"] is True

    state["stage"] = "retry"
    assert manager._runtime_replan_holds_robot(robot)
    assert robot.route_clock == pytest.approx(0.0)


def test_post_evacuation_promotes_existing_runtime_replan_transaction() -> None:
    manager = _manager()
    now = manager._now()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
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
        route_revision=3,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
        spatial_route_nodes=["A", "B"],
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}

    assert manager._queue_active_order_for_background_replan(
        robot,
        now,
        "runtime traffic changed",
    )
    state = manager._runtime_replans[robot.name]
    state["stage"] = "planning"
    old_generation = state["generation"]

    assert manager._queue_active_order_for_background_replan(
        robot,
        now + 0.1,
        "deadlock corridor evacuated; alternate route required",
    )

    assert manager._runtime_replans[robot.name] is state
    assert state["retained_route_superseded"] is True
    assert state["stage"] == "queued"
    assert state["generation"] == old_generation + 1
    assert "corridor evacuated" in state["reason"]
    assert order.spatial_route_nodes == []
    assert manager._runtime_replan_holds_robot(robot)


def test_superseded_runtime_replan_admission_is_bounded() -> None:
    manager = _manager()
    manager.params.setdefault("fleet", {})[
        "max_superseded_runtime_replans"
    ] = 2
    now = manager._now()
    robots: list[FleetRobot] = []
    for index in range(3):
        name = f"r{index + 1}"
        order_id = f"o{index + 1}"
        robot = FleetRobot(
            name=name,
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id=order_id,
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
            route_revision=index + 1,
        )
        manager.robots[name] = robot
        manager.orders[order_id] = FleetOrder(
            order_id=order_id,
            target_lm="B",
            vehicle=name,
            assigned_robot=name,
            status="WAITING_TRAFFIC",
        )
        robots.append(robot)

    for robot in robots[:2]:
        assert manager._queue_active_order_for_background_replan(
            robot,
            now,
            "deadlock corridor evacuated; alternate route required",
        )

    assert not manager._queue_active_order_for_background_replan(
        robots[2],
        now,
        "deadlock corridor evacuated; alternate route required",
    )
    assert set(manager._runtime_replans) == {"r1", "r2"}
    assert robots[2].status == "WAITING"

    manager._runtime_replans.pop("r1")
    assert manager._queue_active_order_for_background_replan(
        robots[2],
        now + 0.1,
        "deadlock corridor evacuated; alternate route required",
    )
    assert set(manager._runtime_replans) == {"r2", "r3"}


def test_queued_transient_runtime_replan_keeps_old_route_executable() -> None:
    manager = _manager()
    now = manager._now()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
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
        route_revision=3,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}

    assert manager._queue_active_order_for_background_replan(
        robot,
        now,
        "traffic wait timeout",
    )
    state = manager._runtime_replans[robot.name]
    assert state["stage"] == "queued"
    assert not state["retained_route_superseded"]
    assert not manager._runtime_replan_holds_robot(robot)

    robot.route_clock = 0.1
    manager._discard_runtime_replan_after_progress(robot)

    assert robot.name not in manager._runtime_replans
    assert order.status == "EXECUTING"
    assert order.error == ""


def test_identical_ordinary_runtime_replan_enters_wait_graph_after_two_failures(
) -> None:
    manager = _manager()
    now = manager._now()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="wait-order",
        wait_for_robot="blocker",
        wait_resource="A->B",
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
        route_revision=3,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        target_lm="A",
        status="WAITING",
        active_order_id="block-order",
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
        route_revision=9,
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="B",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}

    assert manager._queue_active_order_for_background_replan(
        waiter,
        now,
        "traffic wait timeout",
    )
    state = manager._runtime_replans[waiter.name]
    failure = "no_low_level_path:waiter:no_sipp_path:waiter:A->B"

    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        failure,
    )
    assert state["stage"] == "retry"

    state["stage"] = "planning"
    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        failure,
    )

    assert state["stage"] == "deadlock_escalated"
    assert state["escalation_signature_kind"] == "wait_dependency"
    assert waiter.wait_for_robot == blocker.name
    assert manager._ready_runtime_replan_entry(now + 100.0) is None

    blocker.route_revision += 1
    ready = manager._ready_runtime_replan_entry(now + 100.0)
    assert ready is not None
    assert ready[1] is waiter
    assert state["stage"] == "queued"


def test_repeated_dynamic_runtime_conflict_replaces_stale_detour(
    monkeypatch,
) -> None:
    manager = _manager()
    now = manager._now()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="o1",
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
        route_revision=3,
    )
    blocker = FleetRobot(
        name="blocker",
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
        route_revision=9,
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="PLANNING",
        spatial_route_nodes=["A", "B"],
        traffic_detour_edges=[("B", "A"), ("A", "B")],
    )
    state = {
        "order_id": order.order_id,
        "start_lm": "A",
        "route_revision": robot.route_revision,
        "route_clock": robot.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "queued_at": now,
        "retry_at": now,
        "failures": 0,
        "generation": 1,
        "stage": "planning",
        "retained_route_superseded": True,
    }
    manager.robots = {robot.name: robot, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    manager._runtime_replans[robot.name] = state
    replacements: list[dict] = []
    monkeypatch.setattr(
        manager,
        "_queue_alternate_corridor_detour",
        lambda *_args, **kwargs: replacements.append(kwargs) or True,
    )
    debug = {
        "continuousConflictRobot": blocker.name,
        "continuousConflictEdge": "WAIT@ROTATE:A",
    }

    manager._defer_runtime_replan(
        order,
        robot,
        state,
        "continuous reservation conflict",
        debug=debug,
    )
    assert replacements == []

    state["stage"] = "planning"
    manager._defer_runtime_replan(
        order,
        robot,
        state,
        "continuous reservation conflict",
        debug=debug,
    )

    assert replacements == [{
        "avoid_lm": "B",
        "replace_existing": True,
    }]
    assert state["stage"] == "queued"
    assert state["detour_replaced_signature"] == (
        blocker.name,
        "WAIT@ROTATE:A",
        "B",
        blocker.route_revision,
        blocker.route_clock,
    )
    state["stage"] = "planning"
    manager._defer_runtime_replan(
        order,
        robot,
        state,
        "continuous reservation conflict",
        debug=debug,
    )

    assert len(replacements) == 1
    assert state["stage"] == "deadlock_escalated"
    assert robot.wait_for_robot == blocker.name
    assert robot.last_reason == f"occupied by {blocker.name}"
    assert manager._runtime_replan_holds_robot(robot)
    assert manager._ready_runtime_replan_entry(now + 100.0) is None

    blocker.route_clock += 0.25
    ready = manager._ready_runtime_replan_entry(now + 100.0)
    assert ready is not None
    assert ready[1] is robot
    assert state["stage"] == "queued"
    assert robot.wait_for_robot == ""


@pytest.mark.parametrize(
    ("reason", "resource"),
    [
        (
            "no_low_level_path:r:reserved_lm_interval:B@13:owner",
            "B",
        ),
        (
            "no_low_level_path:r:"
            "reserved_edge_interval:A->B@3-8:owner",
            "A->B",
        ),
        (
            "no_low_level_path:r:"
            "rotation_vertex_reserved:A@2-5:owner",
            "A",
        ),
    ],
)
def test_runtime_replan_parses_owned_interval_resource(
    reason: str,
    resource: str,
) -> None:
    manager = _manager()
    assert manager._runtime_replan_failure_resource(reason) == resource


def test_repeated_resource_owner_failure_enters_wait_graph_without_clock_noise() -> None:
    manager = _manager()
    now = manager._now()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
        route_revision=7,
    )
    owner = FleetRobot(
        name="owner",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        active_order_id="owner-order",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
        route_revision=11,
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="B",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
    )
    manager.robots = {waiter.name: waiter, owner.name: owner}
    manager.orders = {order.order_id: order}
    assert manager._queue_active_order_for_background_replan(
        waiter,
        now,
        "deadlock corridor evacuated; alternate route required",
    )
    state = manager._runtime_replans[waiter.name]
    debug = {
        "reservationBlockerRobots": [owner.name],
        "reservationBlockers": [
            {
                "robot": owner.name,
                "resource": "vertex:B",
            }
        ],
    }
    failure = "no_low_level_path:waiter:no_sipp_path:waiter:A->B"

    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        failure,
        debug=debug,
    )
    assert state["stage"] == "retry"

    state["stage"] = "planning"
    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        failure,
        debug=debug,
    )

    assert state["stage"] == "deadlock_escalated"
    assert state["escalation_signature_kind"] == "reservation"
    assert state["escalated_resource"] == "vertex:B"
    assert waiter.wait_for_robot == owner.name
    assert waiter.last_reason == f"occupied by {owner.name}"

    # Sub-LM progress does not dissolve a corridor/resource dependency.
    owner.route_clock = 0.25
    assert manager._ready_runtime_replan_entry(now + 100.0) is None
    assert state["stage"] == "deadlock_escalated"


def test_superseded_route_is_reserved_as_current_body_not_old_future(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    now = manager._now()
    holder = FleetRobot(
        name="holder",
        current_lm="B",
        target_lm="C",
        status="WAITING",
        active_order_id="holder-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "B->C",
                "lm": "B",
            },
            {
                "t": 2.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "B->C",
                "lm": "C",
            },
        ],
        route_revision=5,
    )
    manager.robots = {holder.name: holder}
    manager.orders = {
        "holder-order": FleetOrder(
            order_id="holder-order",
            target_lm="C",
            vehicle=holder.name,
            assigned_robot=holder.name,
            status="PLANNING",
        )
    }
    manager._runtime_replans[holder.name] = {
        "order_id": "holder-order",
        "start_lm": "B",
        "route_revision": holder.route_revision,
        "route_clock": holder.route_clock,
        "queued_at": now,
        "retry_at": now,
        "stage": "retry",
        "retained_route_superseded": True,
    }
    captured: list[dict] = []
    monkeypatch.setattr(
        manager.planner,
        "plan",
        lambda payload: captured.append(payload) or {
            "ok": False,
            "plans": [],
            "debug": {"reason": "captured"},
        },
    )

    manager._plan_valid_requests(
        [{"name": "requester", "startLm": "A", "goalLm": "C"}],
        {
            "skipSoftBlockedDetour": True,
            "strictStationaryRobotAvoidance": True,
        },
    )

    assert len(captured) == 1
    assert not any(
        interval.get("robot") == holder.name
        for interval in captured[0]["reserved_edge_intervals"]
    )
    assert any(
        interval.get("robot") == holder.name
        and interval.get("node") == "B"
        for interval in captured[0]["reserved_vertex_intervals"]
    )


def test_hidden_post_evacuation_cycle_restarts_atomic_coupled_plan(
    monkeypatch,
) -> None:
    manager = _manager()
    now = manager._now()
    first = FleetRobot(
        name="first",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="first-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
        route_revision=3,
        last_reason="replanning route while holding",
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        target_lm="A",
        status="WAITING",
        active_order_id="second-order",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "lm": "A"},
        ],
        route_revision=4,
        last_reason="replanning route while holding",
    )
    manager.robots = {first.name: first, second.name: second}
    manager.orders = {
        "first-order": FleetOrder(
            order_id="first-order",
            target_lm="B",
            vehicle=first.name,
            assigned_robot=first.name,
            status="PLANNING",
        ),
        "second-order": FleetOrder(
            order_id="second-order",
            target_lm="A",
            vehicle=second.name,
            assigned_robot=second.name,
            status="PLANNING",
        ),
    }
    cycle_key = tuple(sorted(manager.robots))
    expanded_key = tuple(sorted((*cycle_key, "third")))
    manager._active_wait_cycles[cycle_key] = now - 3.0
    manager._wait_cycle_grant_signatures[cycle_key] = (
        manager._wait_cycle_grant_signature([first, second])
    )
    for robot, blocker in ((first, second), (second, first)):
        manager._runtime_replans[robot.name] = {
            "order_id": robot.active_order_id,
            "start_lm": robot.current_lm,
            "route_revision": robot.route_revision,
            "route_clock": robot.route_clock,
            "blocker_names": (blocker.name,),
            "queued_at": now - 1.0,
            "retry_at": now,
            "stage": "retry",
            "retained_route_superseded": True,
        }
    # The actual planner may expand this visible pair with a connected held
    # neighbour. Its failure/attempt must still belong to the original cycle.
    manager._coupled_replan_failures[expanded_key] = 1
    manager._coupled_replan_last_attempt[expanded_key] = now - 10.0
    calls: list[tuple[list[str], str]] = []
    monkeypatch.setattr(
        manager,
        "_start_async_coupled_replan",
        lambda robots, winner, _now: (
            calls.append(([item.name for item in robots], winner.name))
            or True
        ),
    )

    manager._resolve_runtime_wait_cycles(now)

    assert calls == [(["first", "second"], "first")]
    assert expanded_key not in manager._coupled_replan_failures
    assert expanded_key not in manager._coupled_replan_last_attempt


def test_coupled_component_ignores_stale_runtime_dependency() -> None:
    manager = _manager()
    first = FleetRobot(
        name="first",
        current_lm="A",
        status="WAITING",
        active_order_id="first-order",
        route_revision=1,
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        status="WAITING",
        active_order_id="second-order",
        route_revision=2,
    )
    stale = FleetRobot(
        name="stale",
        current_lm="A",
        status="WAITING",
        active_order_id="stale-order",
        route_revision=3,
    )
    manager.robots = {
        first.name: first,
        second.name: second,
        stale.name: stale,
    }
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B" if robot.current_lm == "A" else "A",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="WAITING_TRAFFIC",
        )
        for robot in manager.robots.values()
    }
    manager._active_wait_cycles[("first", "second")] = manager._now()
    manager._runtime_replans[stale.name] = {
        "order_id": stale.active_order_id,
        "start_lm": stale.current_lm,
        "route_revision": stale.route_revision - 1,
        "route_clock": stale.route_clock,
        "blocker_names": (first.name,),
        "stage": "retry",
        "retained_route_superseded": True,
    }

    expanded = manager._expand_coupled_replan_component(
        [first, second],
        first,
    )
    assert [robot.name for robot in expanded] == ["first", "second"]

    manager._runtime_replans[stale.name]["route_revision"] = (
        stale.route_revision
    )
    expanded = manager._expand_coupled_replan_component(
        [first, second],
        first,
    )
    assert {robot.name for robot in expanded} == {
        "first",
        "second",
        "stale",
    }


@pytest.mark.parametrize(
    ("fixed_escape", "expected_ready"),
    [(True, True), (False, False)],
)
def test_only_fixed_escape_runtime_replan_can_start_inside_controlled_region(
    fixed_escape: bool,
    expected_ready: bool,
) -> None:
    """A commanded vacancy escape may start inside its controlled region."""
    region = "corridor:X-P"
    landmarks = {
        "X": Landmark(
            name="X",
            x=0.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "Y": Landmark(
            name="Y",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "P": Landmark(
            name="P",
            x=2.0,
            y=0.0,
            properties={"holding_point": True},
        ),
    }
    edges = [
        GraphEdge(
            from_name="X",
            to_name="Y",
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
            properties={"direction": 2, "controlled_region": region},
        ),
        GraphEdge(
            from_name="Y",
            to_name="P",
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(WorldPoint(1.0, 0.0), WorldPoint(2.0, 0.0)),
            properties={"direction": 2},
        ),
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "runtime_replan_lm_tolerance_m": 0.1,
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )
    now = manager._now()
    robot = FleetRobot(
        name="waiter",
        current_lm="X",
        target_lm="P",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "X->Y",
                "lm": "X",
            },
            {
                "t": 1.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "X->Y",
                "lm": "Y",
            },
        ],
        route_revision=7,
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="P",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="PLANNING",
    )
    state = {
        "order_id": order.order_id,
        "start_lm": "X",
        "route_revision": robot.route_revision,
        "route_clock": robot.route_clock,
        "queued_at": now,
        "retry_at": now,
        "generation": 1,
        "stage": "queued",
    }
    if fixed_escape:
        state.update({
            "escape_route_nodes": ["X", "Y", "P"],
            "escape_goal": "P",
            "queued_departure_sink": "parked-sink",
        })
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}
    manager._runtime_replans[robot.name] = state

    vertex = manager._controlled_corridor_graph.vertices["X"]
    assert vertex.controlled_region_ids == (region,)
    ready = manager._ready_runtime_replan_entry(now + 1.0)

    if expected_ready:
        assert ready == (order, robot, state)
        assert manager._runtime_replans[robot.name] is state
    else:
        assert ready is None
        assert robot.name not in manager._runtime_replans


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
        lambda entries: calls.append("prefetch") or True,
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
        lambda entries: calls.append("prefetch") or True,
    )

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["dispatch"]
    calls.clear()
    manager._last_async_job_kind = "dispatch"

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["prefetch"]


def test_corridor_slot_pending_prefetch_does_not_starve_ready_dispatch(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = FleetRobot(
        name="queued",
        current_lm="A",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="queued-order",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="QUEUED",
    )
    request = {
        "name": robot.name,
        "startLm": "A",
        "goalLm": "B",
    }
    dispatch_entry = (order, robot, request, "B")
    prefetch_entry = (order, robot, request, "B", 0.0)
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    manager._rolling_prefetch_failures[robot.name] = 1
    manager._last_async_job_kind = "dispatch"
    calls: list[str] = []
    monkeypatch.setattr(
        manager,
        "_finish_async_simulated_dispatch",
        lambda: 0,
    )
    monkeypatch.setattr(
        manager,
        "_ready_rolling_prefetch_entries",
        lambda: [prefetch_entry],
    )
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda _entries: calls.append("prefetch-pending") or False,
    )
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda _orders: [dispatch_entry],
    )
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda _entries: calls.append("dispatch"),
    )

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["prefetch-pending", "dispatch"]


def test_runtime_replan_yields_next_planner_turn_to_ready_dispatch(
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
    queued_entry = (
        queued_order,
        queued_robot,
        {"name": queued_robot.name, "startLm": "A", "goalLm": "B"},
        "B",
    )
    manager.robots = {queued_robot.name: queued_robot}
    manager.orders = {queued_order.order_id: queued_order}
    manager._last_async_job_kind = "runtime_replan"
    calls: list[str] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(
        manager,
        "_queue_commanded_sink_vacancy_replan",
        lambda _now: calls.append("vacancy") or True,
    )
    monkeypatch.setattr(
        manager,
        "_ready_runtime_replan_entry",
        lambda _now: (
            calls.append("runtime-ready")
            or (queued_order, queued_robot, {})
        ),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_runtime_replan",
        lambda _entry: calls.append("runtime") or True,
    )
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda _orders: [queued_entry],
    )
    monkeypatch.setattr(
        manager,
        "_ready_rolling_prefetch_entries",
        lambda: [(
            queued_order,
            queued_robot,
            queued_entry[2],
            "B",
            0.0,
        )],
    )
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda _entries: calls.append("prefetch") or True,
    )
    monkeypatch.setattr(
        manager,
        "_start_async_simulated_dispatch",
        lambda _entries: calls.append("dispatch"),
    )

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["dispatch"]
    calls.clear()
    manager._last_async_job_kind = "dispatch"

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["prefetch"]
    calls.clear()
    manager._last_async_job_kind = "prefetch"

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["vacancy", "runtime-ready", "runtime"]


def test_quarantined_queued_order_does_not_suppress_runtime_recovery(
    monkeypatch,
) -> None:
    manager = _manager()
    now = 100.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    robot = FleetRobot(
        name="quarantined",
        current_lm="A",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="quarantined-order",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="QUEUED",
        error="unchanged stationary conflict",
        dispatch_failures=8,
        updated_at=0.0,
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}
    blocked_lms = ("A",)
    manager._stationary_order_retry_state[order.order_id] = {
        "blocked_lms": blocked_lms,
        "blocker_names": (),
        "signature": manager._stationary_blocker_signature(blocked_lms),
        "failure_count": manager._stationary_retry_failure_limit(),
    }
    manager._last_async_job_kind = "runtime_replan"
    calls: list[str] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(
        manager,
        "_queue_commanded_sink_vacancy_replan",
        lambda _now: calls.append("vacancy") or False,
    )
    monkeypatch.setattr(
        manager,
        "_ready_runtime_replan_entry",
        lambda _now: (order, robot, {}),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_runtime_replan",
        lambda _entry: calls.append("runtime") or True,
    )

    assert not manager._queued_simulated_dispatch_waiting(now)

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["vacancy", "runtime"]


def test_clearance_in_dispatch_backoff_does_not_suppress_runtime_recovery(
    monkeypatch,
) -> None:
    manager = _manager()
    now = 100.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    robot = FleetRobot(
        name="clearance",
        current_lm="A",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    clearance = FleetOrder(
        order_id="traffic-clearance-clearance",
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="QUEUED",
        error="clearance planner retry",
        updated_at=now,
        internal_kind="traffic_clearance",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {clearance.order_id: clearance}
    calls: list[str] = []
    monkeypatch.setattr(manager, "_finish_async_simulated_dispatch", lambda: 0)
    monkeypatch.setattr(
        manager,
        "_queue_commanded_sink_vacancy_replan",
        lambda _now: calls.append("vacancy") or False,
    )
    monkeypatch.setattr(
        manager,
        "_ready_runtime_replan_entry",
        lambda _now: (clearance, robot, {}),
    )
    monkeypatch.setattr(
        manager,
        "_start_async_runtime_replan",
        lambda _entry: calls.append("runtime") or True,
    )

    assert not manager._queued_simulated_order_dispatch_ready(clearance, now)

    manager._dispatch_orders(async_simulated=True)

    assert calls == ["vacancy", "runtime"]


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


def test_coupled_replan_result_is_rejected_after_same_route_progress(
    monkeypatch,
) -> None:
    """An async plan from an older continuous pose must never be committed."""
    manager = _manager()
    robot = FleetRobot(
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
        route_clock=0.25,
        route_revision=7,
    )
    order = FleetOrder(
        order_id=robot.active_order_id,
        target_lm="B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}
    assert manager._safe_replan_start_lm(robot) == "A"
    failures: list[object] = []
    monkeypatch.setattr(
        manager,
        "_record_coupled_replan_failure",
        lambda *_args: failures.append(object()),
    )
    job = {
        "cycle": (robot.name,),
        "entries": [
            {
                "robot": robot.name,
                "order": order.order_id,
                "start": "A",
                "finalGoal": "B",
                "routeRevision": robot.route_revision,
                "routeClock": 0.0,
            }
        ],
        "result": {
            "ok": True,
            "plans": [{"robot": robot.name, "nodes": ["A", "B"]}],
        },
    }

    assert manager._finish_async_coupled_replan(job) == 0
    assert not failures
    assert robot.route_clock == pytest.approx(0.25)
    assert robot.route_revision == 7


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


def test_rolling_boundary_recovery_uses_exact_sipp_blocker_evidence(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.params["fleet"]["rolling_prefetch_recovery_batch_size"] = 2

    def entry(
        name: str,
        start_lm: str,
        route_nodes: list[str],
        revision: int,
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
            route_revision=revision,
            last_reason="rolling continuation pending",
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = robot
        request: dict[str, object] = {
            "name": name,
            "startLm": start_lm,
            "goalLm": route_nodes[-1],
            "routeNodes": list(route_nodes),
        }
        return order, robot, request, route_nodes[-1], 0.0

    seed = entry("seed", "A", ["A", "X"], 11)
    blocker = entry("blocker", "B", ["B", "Y"], 22)
    entries = [seed, blocker]
    manager._record_rolling_prefetch_blockers(
        [seed],
        {"reservationBlockerRobots": ["blocker"]},
    )
    monkeypatch.setattr(
        manager,
        "_rolling_prefetch_candidates",
        lambda: list(entries),
    )
    monkeypatch.setattr(
        manager,
        "_rolling_boundary_release_pressure",
        lambda: {"seed": 2},
    )
    manager._rolling_prefetch_failures["seed"] = 1

    recovered = manager._ready_rolling_prefetch_entries()

    assert [item[1].name for item in recovered] == ["seed", "blocker"]


def test_rolling_prefetch_blocker_evidence_expires_when_route_advances() -> None:
    manager = _manager()
    seed_order = FleetOrder(
        order_id="o-seed",
        target_lm="X",
        vehicle="seed",
        assigned_robot="seed",
        status="PLANNING",
    )
    blocker_order = FleetOrder(
        order_id="o-blocker",
        target_lm="Y",
        vehicle="blocker",
        assigned_robot="blocker",
        status="PLANNING",
    )
    seed = FleetRobot(
        name="seed",
        current_lm="A",
        status="WAITING",
        active_order_id=seed_order.order_id,
        route_chunk_goal_lm="A",
        route_revision=1,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="WAITING",
        active_order_id=blocker_order.order_id,
        route_chunk_goal_lm="B",
        route_revision=2,
    )
    manager.orders.update({
        seed_order.order_id: seed_order,
        blocker_order.order_id: blocker_order,
    })
    manager.robots.update({
        seed.name: seed,
        blocker.name: blocker,
    })
    entries = [
        (
            seed_order,
            seed,
            {
                "name": "seed",
                "startLm": "A",
                "goalLm": "X",
                "routeNodes": ["A", "X"],
            },
            "X",
            0.0,
        ),
        (
            blocker_order,
            blocker,
            {
                "name": "blocker",
                "startLm": "B",
                "goalLm": "Y",
                "routeNodes": ["B", "Y"],
            },
            "Y",
            0.0,
        ),
    ]
    manager._record_rolling_prefetch_blockers(
        [entries[0]],
        {
            "continuousUnresolvedConflicts": [
                {
                    "robot": "seed",
                    "other": "blocker",
                    "edge": "A->X",
                }
            ]
        },
    )

    assert manager._rolling_boundary_dependency_component(
        entries,
        entries[0],
    ) == entries

    blocker.route_revision += 1

    assert manager._rolling_boundary_dependency_component(
        entries,
        entries[0],
    ) == [entries[0]]
    assert "seed" not in manager._rolling_prefetch_blockers


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


def test_rolling_batch_goal_does_not_extend_through_another_robot_start() -> None:
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

    assert goal == "N1"


def test_repeated_dispatch_failure_does_not_absorb_unrelated_robots() -> None:
    manager = _long_line_manager(edge_count=6)
    robot = FleetRobot(name="r1", current_lm="N0", has_executed_route=True)
    manager.robots[robot.name] = robot
    order = FleetOrder(
        order_id="o1",
        target_lm="N6",
        vehicle="r1",
        dispatch_failures=3,
    )

    assert manager._dispatch_recovery_group_limit(order, robot, 2) == 1
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


def test_parked_detour_blocks_every_edge_incident_to_avoided_lm() -> None:
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
    assert set(order.traffic_detour_edges) == {
        ("B", "C"),
        ("C", "D"),
    }
    alternate = manager.planner.route_planner.find_route(
        "A",
        "D",
        blocked_edges=set(order.traffic_detour_edges),
    )
    assert alternate.nodes == ["A", "B", "X", "D"]


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

    # D is the first waitable vertex outside the controlled region, but a
    # stopped full-size body there still seals the corridor exit. The rolling
    # boundary therefore includes the short open-space clearance tail to E.
    assert recovery_goal == "E"


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


def test_explicit_corridor_still_respects_terminal_body_from_dynamic_zone() -> None:
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
    # Authority is local, but physical occupancy is global. The zone robot's
    # committed trajectory ends at HUB, so letting the corridor robot enter
    # would knowingly block its exit.
    assert manager._controlled_corridor_winners == {}
    assert manager._controlled_corridor_blockers == {
        "corridor": "zone",
    }
    assert manager._traffic_zone_winners == {"zone": target_zone}
    assert manager._next_traffic_zone_transition(
        manager.robots["corridor"]
    ) is None
    assert manager._traffic_zone_admission_reason(
        manager.robots["corridor"],
        0.1,
    ) == ""


def test_corridor_admission_prefers_reachable_front_candidate_over_old_tail(
    monkeypatch,
) -> None:
    manager = _manager()
    region_id = "corridor:test"
    manager._controlled_corridor_graph = SimpleNamespace()
    manager._controlled_corridor_scheduler = CentralCorridorScheduler(
        {region_id}
    )
    manager.robots = {
        "tail": FleetRobot(
            name="tail",
            current_lm="A",
            target_lm="B",
            status="WAITING",
            active_order_id="o-tail",
            last_reason="occupied by front",
            wait_for_robot="front",
            pose={"x": -1.0, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
                {"t": 2.0, "x": 2.0, "y": 0.0, "edgeId": "A->B", "lm": "B"},
            ],
        ),
        "front": FleetRobot(
            name="front",
            current_lm="A",
            target_lm="B",
            status="MOVING",
            active_order_id="o-front",
            pose={"x": 0.5, "y": 0.0, "yaw": 0.0},
            trajectory=[
                {"t": 0.0, "x": 1.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
                {"t": 1.0, "x": 2.0, "y": 0.0, "edgeId": "A->B", "lm": "B"},
            ],
        ),
    }
    manager.orders = {
        "o-tail": FleetOrder(
            order_id="o-tail",
            target_lm="B",
            vehicle="tail",
            assigned_robot="tail",
            status="EXECUTING",
            priority=100,
        ),
        "o-front": FleetOrder(
            order_id="o-front",
            target_lm="B",
            vehicle="front",
            assigned_robot="front",
            status="EXECUTING",
            priority=1,
        ),
    }
    entries = {
        "tail": {
            "region": region_id,
            "regions": (region_id,),
            "passage": region_id,
            "src": "A",
            "dst": "B",
            "holding_lm": "A",
            "staging_clock": 0.0,
            "entry_clock": 1.0,
            "exit_lm": "B",
            "eta": 1.0,
            "at_staging": False,
            "passed_staging": False,
        },
        "front": {
            "region": region_id,
            "regions": (region_id,),
            "passage": region_id,
            "src": "A",
            "dst": "B",
            "holding_lm": "A",
            "staging_clock": 0.0,
            "entry_clock": 0.5,
            "exit_lm": "B",
            "eta": 0.5,
            "at_staging": False,
            "passed_staging": False,
        },
    }
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda robot: dict(entries[robot.name]),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    now = 1_000.0
    manager._controlled_corridor_wait_since[(region_id, "tail")] = now - 30.0

    manager._prepare_controlled_corridor_admissions(now)

    assert manager._controlled_corridor_winners == {"front": region_id}
    assert manager._controlled_corridor_queues == {region_id: ["tail"]}


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
    assert not manager._transfer_controlled_corridor_lease(
        manager.robots["r1"],
        [manager.robots["r1"], manager.robots["r2"]],
        now,
    )
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.1,
    ).startswith("corridor admission wait at L")
    manager.robots["r1"].traffic_priority_until = 0.0
    assert manager._reservation_horizon() > 3.0
    manager._controlled_corridor_leases[region_id] = ("r2", now + 10.0)
    manager._controlled_corridor_winners = {"r2": region_id}

    # A rotate/wait sample at the portal must not hide the following corridor
    # edge from the runtime gate. The preparatory in-place turn itself is safe
    # and may complete before the route-clock stop line; the subsequent
    # crossing must still remain red.
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
    manager._controlled_corridor_passages.clear()
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        0.5,
    ) == ""
    manager.robots["r1"].route_clock = 1.95
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r1"],
        2.0,
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
    assert not manager._queue_active_order_for_background_replan(
        internal,
        now + 1.1,
        "deadlock retreat blocked; alternate route required",
    )
    assert internal.trajectory
    assert internal.active_order_id == "o-r2"


def test_same_direction_leader_preempts_committed_follower_corridor_lease() -> None:
    """A follower cannot own a downstream traffic light through its leader."""
    region = "corridor:same-direction"
    landmarks = {
        "B": Landmark(name="B", x=-1.0, y=0.0),
        "E": Landmark(name="E", x=0.0, y=0.0),
        "I": Landmark(
            name="I",
            x=1.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": region,
            },
        ),
        "X": Landmark(name="X", x=2.0, y=0.0),
    }
    controlled_pairs = {
        frozenset(("E", "I")),
        frozenset(("I", "X")),
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[src].to_point(),
                landmarks[dst].to_point(),
            ),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": region}
                    if frozenset((src, dst)) in controlled_pairs
                    else {}
                ),
            },
        )
        for first, second in (("B", "E"), ("E", "I"), ("I", "X"))
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {
                "route_speed": 1.0,
                "route_acceleration": 1.0,
            },
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    now = 1_000.0
    leader = FleetRobot(
        name="leader",
        current_lm="E",
        target_lm="X",
        status="WAITING",
        active_order_id="leader-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=route(["E", "I", "X"]),
        route_revision=21,
        last_reason="occupied by follower",
        wait_for_robot="follower",
        blocked_since=now,
        traffic_stall_since=now,
    )
    follower = FleetRobot(
        name="follower",
        current_lm="B",
        target_lm="X",
        status="WAITING",
        active_order_id="follower-order",
        pose={"x": -0.1, "y": 0.0, "yaw": 0.0},
        route_clock=0.9,
        trajectory=route(["B", "E", "I", "X"]),
        route_revision=11,
        last_reason="occupied by leader",
        wait_for_robot="leader",
        blocked_since=now,
        traffic_stall_since=now,
    )
    manager.robots = {
        leader.name: leader,
        follower.name: follower,
    }
    for robot in manager.robots.values():
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    # The fleet-wide scheduler, rather than a local wait-cycle resolver,
    # assigns the passage.  The leader is already at the stop line while the
    # follower is still approaching it, so the stable calendar must put the
    # physical queue leader first.
    manager._prepare_controlled_corridor_admissions(now)

    assert manager._controlled_corridor_cycle_owner(
        [leader, follower]
    ) is leader

    manager._break_runtime_wait_cycle(
        [leader.name, follower.name],
        manager.robots,
        now,
        new_episode=True,
    )

    assert leader.status == "MOVING"
    assert leader.last_reason == "deadlock priority granted"
    assert manager._controlled_corridor_leases[region][0] == leader.name
    assert leader.name in manager._controlled_corridor_passages
    follower_slot = manager._controlled_corridor_schedule.slot_for(
        follower.name
    )
    leader_slot = manager._controlled_corridor_schedule.slot_for(leader.name)
    assert follower_slot is not None
    assert leader_slot is not None
    follower_window = follower_slot.resource_windows[0]
    leader_window = leader_slot.resource_windows[0]
    assert (
        follower_slot.entry_time + follower_window.entry_offset_sec
        >= leader_slot.entry_time + leader_window.entry_offset_sec + 1.0
    )
    assert (
        follower_slot.entry_time + follower_window.exit_offset_sec
        >= leader_slot.entry_time + leader_window.exit_offset_sec + 1.0
    )
    # The atomic bundle lets the follower approach the stop line while the
    # leader occupies the actual narrow resource.  Local collision priority,
    # rather than revoking the complete route bundle, keeps it waiting here.
    assert manager._controlled_corridor_has_grant(
        follower.name,
        (region,),
    )
    assert follower.status == "WAITING"
    assert follower.wait_for_robot == leader.name


def test_controlled_corridor_exit_boundary_does_not_create_internal_gate() -> None:
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

    # J is not a safe external holding LM between the two auto-detected
    # resources. A red light here would stop the body inside the upstream
    # passage; admission must have been acquired as a bundle before A.
    assert entry is None
    assert manager._controlled_corridor_admission_reason(robot, 1.01) == ""


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


def test_controlled_corridor_passage_grants_all_regions_atomically() -> None:
    region_a = "corridor:a"
    region_b = "corridor:b"
    landmarks = {
        "H": Landmark(name="H", x=0.0, y=0.0),
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
        "X": Landmark(name="X", x=3.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={"direction": 2, "controlled_region": regions},
        )
        for src, dst, regions in (
            ("H", "A", region_a),
            ("A", "B", f"{region_a},{region_b}"),
            ("B", "X", region_b),
            ("X", "B", region_b),
            ("B", "A", f"{region_a},{region_b}"),
            ("A", "H", region_a),
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

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0 if nodes[-1] == "X" else math.pi,
                "edgeId": (
                    f"{nodes[0]}->{nodes[1]}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    for name, nodes, priority in (
        ("r1", ["H", "A", "B", "X"], 5),
        ("r2", ["X", "B", "A", "H"], 1),
    ):
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=nodes[0],
            target_lm=nodes[-1],
            status="MOVING",
            active_order_id=f"o-{name}",
            pose={
                "x": landmarks[nodes[0]].x,
                "y": 0.0,
                "yaw": 0.0 if name == "r1" else math.pi,
            },
            trajectory=route(nodes),
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

    entry = manager._next_controlled_corridor_entry(manager.robots["r1"])
    assert entry is not None
    assert entry["holding_lm"] == "H"
    assert entry["exit_lm"] == "X"
    assert entry["regions"] == (region_a, region_b)

    now = 1_000.0
    manager._prepare_controlled_corridor_admissions(now)
    assert manager._controlled_corridor_passages["r1"]["regions"] == (
        region_a,
        region_b,
    )
    assert manager._controlled_corridor_leases[region_a][0] == "r1"
    # The complete A+B passage is granted atomically, while the derived
    # per-region lease view exposes only the resource whose physical time
    # window is active. Region B remains protected by the same slot without
    # being painted occupied for the whole traversal.
    assert region_b not in manager._controlled_corridor_leases
    assert manager._controlled_corridor_has_grant(
        "r1",
        (region_a, region_b),
    )

    # The gate checks the substep crossing itself. Even if localization is
    # 4 cm off the stop LM, r2 cannot jump across the old 3 cm point test.
    manager.robots["r2"].pose = {"x": 3.04, "y": 0.0, "yaw": math.pi}
    assert manager._controlled_corridor_admission_reason(
        manager.robots["r2"],
        0.05,
    ).startswith("corridor admission wait at X")

    # Entering region A retains the downstream region-B lease; it is not
    # released by wall-clock TTL or by the first rectangle's occupancy.
    r1 = manager.robots["r1"]
    r1.route_clock = 0.9
    r1.pose = {"x": 0.9, "y": 0.0, "yaw": 0.0}
    manager._prepare_controlled_corridor_admissions(now + 20.0)
    assert manager._controlled_corridor_passages["r1"]["entered"]
    assert manager._controlled_corridor_leases[region_b][0] == "r1"
    r2 = manager.robots["r2"]
    r2.last_reason = (
        f"corridor admission wait at X for {region_b}; owner r1"
    )
    assert not manager._transfer_controlled_corridor_lease(
        r2,
        [r1, r2],
        now + 20.05,
    )
    assert manager._controlled_corridor_leases[region_a][0] == "r1"
    assert manager._controlled_corridor_leases[region_b][0] == "r1"

    # After the centre reaches the external safe exit, keep one bounded
    # occupancy-recheck interval.  A single missed localization observation
    # must not release an entered atomic bundle, but the stale predicted exit
    # must not survive beyond that short confirmation cycle.
    r1.route_clock = 3.0
    r1.current_lm = "X"
    r1.pose = {"x": 3.0, "y": 0.0, "yaw": 0.0}
    manager._prepare_controlled_corridor_admissions(now + 20.1)
    assert "r1" in manager._controlled_corridor_passages
    manager._prepare_controlled_corridor_admissions(now + 22.21)
    assert "r1" not in manager._controlled_corridor_passages
    r2_slot = manager._controlled_corridor_schedule.slot_for("r2")
    assert r2_slot is not None
    assert r2_slot.entry_time > now + 22.21
    assert region_a not in manager._controlled_corridor_leases
    manager._prepare_controlled_corridor_admissions(r2_slot.entry_time)
    assert manager._controlled_corridor_leases[region_b][0] == "r2"
    assert region_a not in manager._controlled_corridor_leases
    assert manager._controlled_corridor_has_grant(
        "r2",
        (region_b, region_a),
    )


def test_controlled_corridor_loser_stages_clear_of_portal_broadphase() -> None:
    region = "corridor:narrow"
    coordinates = {
        "Q": -3.0,
        "S": -2.0,
        "N": -1.0,
        "P": 0.0,
        "I": 1.0,
        "X": 2.0,
    }
    landmarks = {
        name: Landmark(
            name=name,
            x=x,
            y=0.0,
            properties=(
                {"can_wait": False, "controlled_region": region}
                if name == "I"
                else {"can_wait": True}
            ),
        )
        for name, x in coordinates.items()
    }
    edges: list[GraphEdge] = []
    for first, second in zip(coordinates, list(coordinates)[1:]):
        for src, dst in ((first, second), (second, first)):
            controlled = {"controlled_region": region} if {src, dst} & {"I"} else {}
            edges.append(
                GraphEdge(
                    from_name=src,
                    to_name=dst,
                    length=1.0,
                    kind="line",
                    edge_type="FeatureLine",
                    world_points=(
                        landmarks[src].to_point(),
                        landmarks[dst].to_point(),
                    ),
                    properties={"direction": 2, **controlled},
                )
            )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {
                "footprint": [
                    {"x": -0.5, "y": -0.35},
                    {"x": 0.5, "y": -0.35},
                    {"x": 0.5, "y": 0.35},
                    {"x": -0.5, "y": 0.35},
                ],
            },
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0 if nodes[0] == "Q" else math.pi,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    entrant = FleetRobot(
        name="entrant",
        current_lm="Q",
        target_lm="X",
        status="MOVING",
        pose={"x": -2.05, "y": 0.0, "yaw": 0.0},
        route_clock=0.95,
        trajectory=route(["Q", "S", "N", "P", "I", "X"]),
    )
    manager.robots = {entrant.name: entrant}
    entry = manager._next_controlled_corridor_entry(entrant)

    assert entry is not None
    assert entry["src"] == "P"
    assert entry["holding_lm"] == "S"
    assert entry["staging_clock"] == 1.0
    assert math.dist(
        (landmarks["S"].x, landmarks["S"].y),
        (landmarks["P"].x, landmarks["P"].y),
    ) >= manager.collision.robot_broadphase_distance()
    assert math.dist(
        (landmarks["N"].x, landmarks["N"].y),
        (landmarks["P"].x, landmarks["P"].y),
    ) < manager.collision.robot_broadphase_distance()

    manager._controlled_corridor_leases[region] = ("owner", 2_000.0)
    reason = manager._controlled_corridor_admission_reason(entrant, 1.05)
    assert reason.startswith(f"corridor admission wait at S for {region}")


def test_controlled_corridor_lease_commits_after_staging_before_entry() -> None:
    region = "corridor:narrow"
    coordinates = {
        "Q": -3.0,
        "S": -2.0,
        "N": -1.0,
        "P": 0.0,
        "I": 1.0,
        "X": 2.0,
    }
    landmarks = {
        name: Landmark(
            name=name,
            x=x,
            y=0.0,
            properties=(
                {"can_wait": False, "controlled_region": region}
                if name == "I"
                else {"can_wait": True}
            ),
        )
        for name, x in coordinates.items()
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **({"controlled_region": region} if {src, dst} & {"I"} else {}),
            },
        )
        for first, second in zip(coordinates, list(coordinates)[1:])
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {
                "footprint": [
                    {"x": -0.5, "y": -0.35},
                    {"x": 0.5, "y": -0.35},
                    {"x": 0.5, "y": 0.35},
                    {"x": -0.5, "y": 0.35},
                ],
            },
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
                "controlled_corridor_admission_lease_sec": 1.0,
                "controlled_corridor_entry_lookahead_sec": 3.0,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0 if nodes[0] == "Q" else math.pi,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner = FleetRobot(
        name="owner",
        current_lm="S",
        target_lm="X",
        status="MOVING",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        route_clock=1.0,
        trajectory=route(["Q", "S", "N", "P", "I", "X"]),
    )
    manager.robots = {owner.name: owner}
    now = 1_000.0
    manager._prepare_controlled_corridor_admissions(now)
    initial = manager._controlled_corridor_passages[owner.name]
    assert initial["committed"] is True
    assert initial["past_commit_point"] is False
    # A slot releases the robot from its upstream staging LM. Waiting until
    # the later portal ETA would make the red light slide forward forever.
    assert manager._controlled_corridor_winners == {
        owner.name: region,
    }

    # The owner has crossed the backed-off staging LM. N/P are graph LMs, but
    # they lie inside the protected exit-clearance envelope, so the complete
    # approach is now committed before its centre enters the rectangle. A
    # blocked approach must retreat to S; it must never grant the opposing
    # robot a simultaneous slot through the same mouth.
    owner.route_clock = 1.5
    owner.pose = {"x": -1.5, "y": 0.0, "yaw": 0.0}
    challenger = FleetRobot(
        name="challenger",
        current_lm="X",
        target_lm="Q",
        status="MOVING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=route(["X", "I", "P", "N", "S", "Q"]),
    )
    manager.robots[challenger.name] = challenger
    manager._prepare_controlled_corridor_admissions(now + 20.0)

    passage = manager._controlled_corridor_passages[owner.name]
    assert passage["committed"] is True
    assert passage["entered"] is False
    assert passage["past_commit_point"] is True
    assert manager._controlled_corridor_leases[region][0] == owner.name
    assert manager._controlled_corridor_has_grant(
        owner.name,
        (region,),
    )
    # The opposing robot may receive a future tentative slot behind the
    # owner's rebased passage, but it cannot become committed/current while
    # the point-of-no-return owner still holds the approach authority.
    assert challenger.name in manager._controlled_corridor_passages
    assert not manager._controlled_corridor_passages[challenger.name][
        "entered"
    ]
    assert challenger.name not in manager._controlled_corridor_winners

    challenger.last_reason = manager._controlled_corridor_admission_reason(
        challenger,
        0.1,
    )
    assert challenger.last_reason.startswith("corridor admission wait at X")
    assert not manager._transfer_controlled_corridor_lease(
        challenger,
        [owner, challenger],
        now + 20.1,
    )
    assert manager._controlled_corridor_leases[region][0] == owner.name

    # Commitment belongs to one exact route revision. If a transactional
    # replan turns the pre-entry owner away from the passage, its old bundle
    # must not survive as a lifelong phantom lease.
    owner.route_revision += 1
    owner.current_lm = "S"
    owner.target_lm = "Q"
    owner.route_clock = 0.0
    owner.pose = {"x": -2.0, "y": 0.0, "yaw": math.pi}
    owner.trajectory = route(["S", "Q"])
    manager._prepare_controlled_corridor_admissions(now + 20.2)

    assert owner.name not in manager._controlled_corridor_passages
    challenger_slot = manager._controlled_corridor_schedule.slot_for(
        challenger.name
    )
    assert challenger_slot is not None
    # The previous owner never entered, so cancelling its exact route
    # revision frees the empty resource immediately; no direction-change
    # clearance is needed for a body which was never inside.
    assert challenger_slot.entry_time >= now + 20.2
    manager._prepare_controlled_corridor_admissions(
        challenger_slot.entry_time
    )
    assert manager._controlled_corridor_leases[region][0] == challenger.name


def test_controlled_corridor_portal_deadlock_retreats_external_loser_to_staging() -> None:
    region = "corridor:narrow"
    coordinates = {
        "Q": -3.0,
        "S": -2.0,
        "N": -1.0,
        "P": 0.0,
        "I": 1.0,
        "X": 2.0,
    }
    landmarks = {
        name: Landmark(
            name=name,
            x=x,
            y=0.0,
            properties=(
                {"can_wait": False, "controlled_region": region}
                if name == "I"
                else {"can_wait": True}
            ),
        )
        for name, x in coordinates.items()
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **({"controlled_region": region} if {src, dst} & {"I"} else {}),
            },
        )
        for first, second in zip(coordinates, list(coordinates)[1:])
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {
                "footprint": [
                    {"x": -0.5, "y": -0.35},
                    {"x": 0.5, "y": -0.35},
                    {"x": 0.5, "y": 0.35},
                    {"x": -0.5, "y": 0.35},
                ],
            },
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0 if nodes[0] == "Q" else math.pi,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["X", "I", "P", "N", "S", "Q"])
    owner = FleetRobot(
        name="owner",
        current_lm="I",
        target_lm="Q",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        route_clock=1.0,
        trajectory=owner_route,
        last_reason="occupied by entrant",
        wait_for_robot="entrant",
    )
    entrant = FleetRobot(
        name="entrant",
        current_lm="P",
        target_lm="X",
        active_order_id="entrant-order",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_clock=3.0,
        trajectory=route(["Q", "S", "N", "P", "I", "X"]),
        last_reason=(
            f"corridor admission wait at P for {region}; owner owner"
        ),
        wait_for_robot="owner",
    )
    manager.robots = {owner.name: owner, entrant.name: entrant}
    for robot in (owner, entrant):
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )

    evacuated = manager._start_deadlock_corridor_evacuation(
        [owner, entrant],
        owner,
        1_000.0,
    )

    assert evacuated == entrant.name
    assert entrant.status == "RETREATING"
    assert entrant.retreat_target_lm == "S"
    assert entrant.retreat_target_clock == 1.0
    assert owner.status == "WAITING"
    assert owner.active_order_id == "owner-order"
    assert owner.trajectory is owner_route
    assert owner.retreat_target_clock is None


def test_portal_entrant_without_route_history_retreats_to_external_graph_pocket() -> None:
    region = "corridor:narrow"
    landmarks = {
        "S": Landmark(name="S", x=0.0, y=-2.0, properties={"can_wait": True}),
        "P": Landmark(name="P", x=0.0, y=0.0, properties={"can_wait": True}),
        "I": Landmark(
            name="I",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "X": Landmark(name="X", x=2.0, y=0.0, properties={"can_wait": True}),
    }
    edge_specs = (
        ("S", "P", ""),
        ("P", "S", ""),
        ("P", "I", region),
        ("I", "P", region),
        ("I", "X", region),
        ("X", "I", region),
    )
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=math.hypot(
                landmarks[dst].x - landmarks[src].x,
                landmarks[dst].y - landmarks[src].y,
            ),
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **({"controlled_region": controlled} if controlled else {}),
            },
        )
        for src, dst, controlled in edge_specs
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

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0 if nodes[0] == "P" else math.pi,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["X", "I", "P"])
    owner = FleetRobot(
        name="owner",
        current_lm="I",
        target_lm="P",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        route_clock=1.0,
        trajectory=owner_route,
        last_reason="occupied by entrant",
        wait_for_robot="entrant",
    )
    # This rolling chunk starts exactly at the portal. It intentionally has no
    # historical S sample for the ordinary previous-trajectory retreat code.
    entrant = FleetRobot(
        name="entrant",
        current_lm="P",
        target_lm="X",
        active_order_id="entrant-order",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        trajectory=route(["P", "I", "X"]),
        last_reason=(
            f"corridor admission wait at P for {region}; owner owner"
        ),
        wait_for_robot="owner",
    )
    manager.robots = {owner.name: owner, entrant.name: entrant}
    for robot in (owner, entrant):
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )

    pocket = manager._controlled_corridor_graph.vertices["S"]
    assert pocket.can_wait
    assert not pocket.controlled_region_ids
    assert manager._next_controlled_corridor_entry(entrant)["holding_lm"] == "P"

    evacuated = manager._start_deadlock_corridor_evacuation(
        [owner, entrant],
        owner,
        1_000.0,
    )

    assert evacuated == entrant.name
    assert entrant.status == "RETREATING"
    assert entrant.retreat_target_lm == "S"
    assert any(sample.get("lm") == "S" for sample in entrant.trajectory)
    assert owner.trajectory is owner_route
    assert owner.retreat_target_clock is None


def test_portal_entrant_near_future_staging_lm_escapes_from_pose_safe_lm() -> None:
    """A source-side currentLm must not poison a stop-line escape plan.

    Admission keeps the source LM authoritative until the trajectory reaches
    the destination timestamp.  In production the robot was already 3 cm from
    its staging LM, though, and MAPF rejected every escape because it received
    the previous LM together with the new pose.  The internal owner then had
    no way to leave the capacity-one corridor.
    """
    region = "corridor:narrow"
    landmarks = {
        "S": Landmark(name="S", x=-5.0, y=0.0, properties={"can_wait": True}),
        "E": Landmark(name="E", x=-3.0, y=0.0, properties={"can_wait": True}),
        "H": Landmark(name="H", x=-2.0, y=0.0, properties={"can_wait": True}),
        "P": Landmark(name="P", x=0.0, y=0.0, properties={"can_wait": True}),
        "I": Landmark(
            name="I",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "X": Landmark(name="X", x=2.0, y=0.0, properties={"can_wait": True}),
    }
    edge_specs = (
        ("S", "E", ""),
        ("E", "S", ""),
        ("E", "H", ""),
        ("H", "E", ""),
        ("H", "P", ""),
        ("P", "H", ""),
        ("P", "I", region),
        ("I", "P", region),
        ("I", "X", region),
        ("X", "I", region),
    )
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=abs(landmarks[dst].x - landmarks[src].x),
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **({"controlled_region": controlled} if controlled else {}),
            },
        )
        for src, dst, controlled in edge_specs
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
                "runtime_replan_lm_tolerance_m": 0.10,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["X", "I", "P"])
    owner = FleetRobot(
        name="owner",
        current_lm="I",
        target_lm="P",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        route_clock=1.0,
        trajectory=owner_route,
        last_reason="occupied by entrant",
        wait_for_robot="entrant",
    )
    entrant = FleetRobot(
        name="entrant",
        # Still source-side for admission/occupancy purposes.
        current_lm="E",
        target_lm="X",
        active_order_id="entrant-order",
        status="WAITING",
        # Physically within replan tolerance of the future staging sample H.
        pose={"x": -2.03, "y": 0.0, "yaw": 0.0},
        route_clock=0.97,
        trajectory=route(["E", "H", "P", "I", "X"]),
        last_reason=(
            f"corridor admission wait at H for {region}; owner owner"
        ),
        wait_for_robot="owner",
    )
    manager.robots = {owner.name: owner, entrant.name: entrant}
    for robot in (owner, entrant):
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    manager._controlled_corridor_passages[owner.name] = {
        "regions": (region,),
        "entered": True,
        "committed": True,
        "route_revision": owner.route_revision,
    }

    upcoming = manager._next_controlled_corridor_entry(entrant)
    assert upcoming is not None
    assert upcoming["holding_lm"] == "H"
    assert upcoming["staging_clock"] == pytest.approx(1.0)
    assert manager._traffic_lm_for_robot(entrant) == "E"
    assert manager._safe_replan_start_lm(entrant) == "H"

    evacuated = manager._start_deadlock_corridor_evacuation(
        [owner, entrant],
        owner,
        1_000.0,
    )

    assert evacuated == entrant.name
    assert entrant.status == "RETREATING"
    assert entrant.retreat_target_lm == "S"
    assert entrant.plan_nodes == ["S", "E", "H"]
    assert entrant.current_lm == "E"
    assert manager.orders["entrant-order"].target_lm == "X"
    assert owner.trajectory is owner_route
    assert owner.retreat_target_clock is None
    assert manager._controlled_corridor_passages[owner.name]["entered"] is True


def test_physical_corridor_owner_retreats_mid_edge_committed_entrant_to_anchor() -> None:
    """A mid-edge entrant must first reverse to an LM, not fake a MAPF start."""
    region = "corridor:narrow"
    landmarks = {
        "S": Landmark(name="S", x=-5.0, y=0.0, properties={"can_wait": True}),
        "E": Landmark(name="E", x=-3.0, y=0.0, properties={"can_wait": True}),
        "H": Landmark(name="H", x=-2.0, y=0.0, properties={"can_wait": True}),
        "P": Landmark(name="P", x=0.0, y=0.0, properties={"can_wait": True}),
        "I": Landmark(
            name="I",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "X": Landmark(name="X", x=2.0, y=0.0, properties={"can_wait": True}),
    }
    edge_specs = (
        ("S", "E", ""),
        ("E", "S", ""),
        ("E", "H", ""),
        ("H", "E", ""),
        ("H", "P", ""),
        ("P", "H", ""),
        ("P", "I", region),
        ("I", "P", region),
        ("I", "X", region),
        ("X", "I", region),
    )
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=abs(landmarks[dst].x - landmarks[src].x),
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **({"controlled_region": controlled} if controlled else {}),
            },
        )
        for src, dst, controlled in edge_specs
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
                "runtime_replan_lm_tolerance_m": 0.10,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["X", "I", "P"])
    owner = FleetRobot(
        name="owner",
        current_lm="I",
        target_lm="P",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        route_clock=1.0,
        route_revision=11,
        trajectory=owner_route,
        last_reason="occupied by entrant",
        wait_for_robot="entrant",
    )
    entrant_route = route(["E", "H", "P", "I", "X"])
    entrant = FleetRobot(
        name="entrant",
        current_lm="E",
        target_lm="X",
        active_order_id="entrant-order",
        status="WAITING",
        # Farther than the replan tolerance from both E and H.
        pose={"x": -2.6, "y": 0.0, "yaw": 0.0},
        route_clock=0.4,
        route_revision=7,
        trajectory=entrant_route,
        last_reason="occupied by owner",
        wait_for_robot="owner",
    )
    manager.robots = {owner.name: owner, entrant.name: entrant}
    for robot in (owner, entrant):
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    manager._controlled_corridor_occupancy = {region: [owner.name]}
    manager._controlled_corridor_passages[entrant.name] = {
        "regions": (region,),
        "entered": False,
        "committed": True,
        "route_revision": entrant.route_revision,
    }

    upcoming = manager._next_controlled_corridor_entry(entrant)
    assert upcoming is not None
    assert upcoming["staging_clock"] == pytest.approx(1.0)
    assert manager._safe_replan_start_lm(entrant) == ""
    assert manager._previous_trajectory_lm(entrant) == (0.0, "E")
    # Physical occupancy must override the external robot's admission promise.
    assert manager._controlled_corridor_cycle_owner([entrant, owner]) is owner

    evacuated = manager._start_deadlock_corridor_evacuation(
        [owner, entrant],
        owner,
        1_000.0,
    )

    assert evacuated == entrant.name
    assert entrant.status == "RETREATING"
    assert entrant.retreat_target_lm == "E"
    assert entrant.retreat_target_clock == pytest.approx(0.0)
    assert entrant.trajectory is entrant_route
    assert entrant.retreat_blocked_edges
    assert owner.trajectory is owner_route
    assert owner.retreat_target_clock is None


def test_illegal_opposing_physical_owners_evict_one_to_external_lm() -> None:
    """A corrupted capacity-one corridor must have a deterministic escape."""
    region = "corridor:shared-physical"
    landmarks = {
        name: Landmark(
            name=name,
            x=x,
            y=0.0,
            properties=(
                {"can_wait": False, "controlled_region": region}
                if name in {"I1", "I2"}
                else {"can_wait": True}
            ),
        )
        for name, x in (
            ("A", 0.0),
            ("I1", 2.0),
            ("I2", 4.0),
            ("B", 6.0),
        )
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=abs(landmarks[dst].x - landmarks[src].x),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[src].to_point(),
                landmarks[dst].to_point(),
            ),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": region}
                    if {src, dst} & {"I1", "I2"}
                    else {}
                ),
            },
        )
        for first, second in (("A", "I1"), ("I1", "I2"), ("I2", "B"))
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            }
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0 if nodes[0] == "A" else math.pi,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    winner = FleetRobot(
        name="winner",
        current_lm="I1",
        target_lm="B",
        active_order_id="winner-order",
        status="WAITING",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        route_clock=1.0,
        trajectory=route(["A", "I1", "I2", "B"]),
        last_reason="occupied by loser",
        wait_for_robot="loser",
    )
    loser = FleetRobot(
        name="loser",
        current_lm="I2",
        target_lm="A",
        active_order_id="loser-order",
        status="WAITING",
        pose={"x": 4.0, "y": 0.0, "yaw": math.pi},
        route_clock=1.0,
        trajectory=route(["B", "I2", "I1", "A"]),
        last_reason="occupied by winner",
        wait_for_robot="winner",
    )
    manager.robots = {winner.name: winner, loser.name: loser}
    manager._controlled_corridor_occupancy = {
        region: [winner.name, loser.name],
    }
    for robot in manager.robots.values():
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )

    evacuated = manager._start_deadlock_corridor_evacuation(
        [winner, loser],
        winner,
        1_000.0,
    )

    assert evacuated == loser.name
    assert loser.status == "RETREATING"
    assert loser.retreat_target_lm == "B"
    assert winner.status == "WAITING"
    assert winner.retreat_target_clock is None


def test_entered_corridor_owner_clears_reciprocal_external_goal_blocker() -> None:
    """An owner exit cycle must not depend on an admission-reason string.

    Once collision arbitration observes both bodies, their reasons become
    ordinary reciprocal ``occupied by`` dependencies.  The outside robot can
    also be following a route that never enters the controlled region, so it
    has no upcoming corridor staging record.  Its previous LM is then its
    current LM: the resolver must choose a real graph pocket instead of
    replanning the same blocked suffix.
    """
    region = "corridor:exit-cycle"
    landmarks = {
        "B": Landmark(name="B", x=0.0, y=0.0),
        "P": Landmark(name="P", x=1.0, y=0.0),
        "E": Landmark(name="E", x=2.0, y=0.0),
        "I": Landmark(
            name="I",
            x=3.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": region},
        ),
        "X": Landmark(name="X", x=4.0, y=0.0),
        "Q": Landmark(name="Q", x=0.0, y=-2.0),
    }
    controlled_pairs = {frozenset(("X", "I")), frozenset(("I", "E"))}
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=math.hypot(
                landmarks[dst].x - landmarks[src].x,
                landmarks[dst].y - landmarks[src].y,
            ),
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": region}
                    if frozenset((src, dst)) in controlled_pairs
                    else {}
                ),
            },
        )
        for first, second in (("X", "I"), ("I", "E"), ("E", "P"), ("P", "B"), ("B", "Q"))
        for src, dst in ((first, second), (second, first))
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
                "deadlock_retreat_after_sec": 0.5,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": math.pi if nodes[0] == "X" else 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["X", "I", "E", "P", "B"])
    owner = FleetRobot(
        name="owner",
        current_lm="I",
        target_lm="B",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 2.8, "y": 0.0, "yaw": math.pi},
        route_clock=1.2,
        route_revision=11,
        trajectory=owner_route,
        last_reason="occupied by blocker",
        wait_for_robot="blocker",
    )
    blocker_route = route(["B", "P", "E"])
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        target_lm="E",
        active_order_id="blocker-order",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        route_revision=22,
        trajectory=blocker_route,
        last_reason="occupied by owner",
        wait_for_robot="owner",
    )
    manager.robots = {owner.name: owner, blocker.name: blocker}
    for robot in (owner, blocker):
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    owner_passage = {
        "regions": (region,),
        "entered": True,
        "committed": True,
        "route_revision": owner.route_revision,
    }
    manager._controlled_corridor_passages[owner.name] = owner_passage
    manager._controlled_corridor_occupancy = {region: [owner.name]}
    manager._controlled_corridor_leases[region] = (owner.name, 2_000.0)

    assert manager._next_controlled_corridor_entry(blocker) is None
    assert manager._previous_trajectory_lm(blocker) == (0.0, "B")
    assert manager._controlled_corridor_cycle_owner([blocker, owner]) is owner

    now = 1_000.0
    cycle_key = tuple(sorted(manager.robots))
    manager._active_wait_cycles[cycle_key] = now - 2.0
    manager._break_runtime_wait_cycle(
        [blocker.name, owner.name],
        manager.robots,
        now,
        new_episode=False,
    )

    assert blocker.status == "RETREATING"
    assert blocker.retreat_target_lm == "Q"
    assert any(sample.get("lm") == "Q" for sample in blocker.trajectory)
    assert blocker.active_order_id == "blocker-order"
    assert blocker.target_lm == "E"
    assert manager.orders["blocker-order"].target_lm == "E"
    assert owner.trajectory is owner_route
    assert owner.active_order_id == "owner-order"
    assert manager._controlled_corridor_passages[owner.name] is owner_passage
    assert manager._controlled_corridor_leases[region][0] == owner.name


def test_corridor_owner_evacuation_avoids_blocked_historical_staging_side() -> None:
    """A portal turn-around must escape outward, not reverse through its owner.

    A motion-direction manoeuvre can temporarily carry a future entrant across
    the portal before its actual controlled suffix begins.  The nearest older
    holding LM lies behind the committed owner, so the admission stop line
    must remain at the reachable external pocket and recovery must escape
    outward from there.
    """
    first_region = "corridor:bundle:first"
    second_region = "corridor:bundle:second"
    landmarks = {
        "P": Landmark(name="P", x=-1.0, y=0.0),
        "J": Landmark(
            name="J",
            x=0.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": second_region,
            },
        ),
        "E": Landmark(name="E", x=1.0, y=0.0),
        "Q": Landmark(name="Q", x=3.0, y=0.0),
        "N": Landmark(
            name="N",
            x=0.0,
            y=-1.0,
            properties={
                "can_wait": False,
                "controlled_region": first_region,
            },
        ),
        "X": Landmark(name="X", x=0.0, y=-3.0),
    }
    controlled_pairs = {
        frozenset(("P", "J")): second_region,
        frozenset(("J", "E")): second_region,
        frozenset(("J", "N")): first_region,
        frozenset(("N", "X")): first_region,
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=math.hypot(
                landmarks[dst].x - landmarks[src].x,
                landmarks[dst].y - landmarks[src].y,
            ),
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": region}
                    if (region := controlled_pairs.get(frozenset((src, dst))))
                    else {}
                ),
            },
        )
        for first, second in (
            ("P", "J"),
            ("J", "E"),
            ("E", "Q"),
            ("J", "N"),
            ("N", "X"),
        )
        for src, dst in ((first, second), (second, first))
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

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["P", "J", "N", "X"])
    owner = FleetRobot(
        name="owner",
        current_lm="P",
        target_lm="X",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": -1.0, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        route_revision=11,
        trajectory=owner_route,
        last_reason="occupied by entrant",
        wait_for_robot="entrant",
    )
    entrant_route = route(["P", "J", "E", "J", "N", "X"])
    entrant = FleetRobot(
        name="entrant",
        current_lm="E",
        target_lm="X",
        active_order_id="entrant-order",
        status="WAITING",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        route_clock=2.0,
        route_revision=22,
        trajectory=entrant_route,
        last_reason=(
            "corridor admission wait at P for "
            f"{second_region}; owner owner"
        ),
        wait_for_robot="owner",
    )
    manager.robots = {owner.name: owner, entrant.name: entrant}
    for robot in (owner, entrant):
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    owner_passage = {
        "regions": (first_region, second_region),
        "entered": False,
        "committed": True,
        "route_revision": owner.route_revision,
    }
    manager._controlled_corridor_passages[owner.name] = owner_passage
    manager._controlled_corridor_leases = {
        first_region: (owner.name, 2_000.0),
        second_region: (owner.name, 2_000.0),
    }

    upcoming = manager._next_controlled_corridor_entry(entrant)
    assert upcoming is not None
    assert upcoming["holding_lm"] == "E"
    assert upcoming["staging_clock"] == 2.0
    # The obsolete historical choice P is physically unreachable without
    # crossing the owner; retain that assertion as the safety premise for the
    # outward graph escape below.
    assert manager._deadlock_retreat_path_blocker(entrant, 0.0) == owner.name

    evacuated = manager._start_deadlock_corridor_evacuation(
        [owner, entrant],
        owner,
        1_000.0,
    )

    assert evacuated == entrant.name
    assert entrant.status == "RETREATING"
    assert entrant.retreat_target_lm == "Q"
    assert entrant.plan_nodes == ["Q", "E"]
    assert entrant.active_order_id == "entrant-order"
    assert manager.orders["entrant-order"].target_lm == "X"
    assert owner.trajectory is owner_route
    assert manager._controlled_corridor_passages[owner.name] is owner_passage
    assert manager._controlled_corridor_leases[first_region][0] == owner.name
    assert manager._controlled_corridor_leases[second_region][0] == owner.name


@pytest.mark.parametrize("queue_depth", [1, 2])
def test_entered_corridor_exit_cycle_unwinds_external_queue_from_tail(
    monkeypatch: pytest.MonkeyPatch,
    queue_depth: int,
) -> None:
    """A blocked portal must be opened from the clear tail of its queue.

    The wait-for cycle itself contains only the entered corridor owner and the
    opposing robot at the portal.  Moving that direct blocker backwards is not
    executable while one or two admission losers occupy its only escape arm.
    The resolver therefore has to discover the bounded upstream queue and move
    its tail first, without replacing any active package goal or the owner's
    committed passage.
    """
    region = "corridor:tail-cascade"
    coordinates = {
        "H": -4.0,
        "G": -3.0,
        "T": -2.0,
        "A": -1.0,
        "B": 0.0,
        "P": 1.0,
        "E": 2.0,
        "I": 3.0,
        "X": 4.0,
    }
    landmarks = {
        name: Landmark(
            name=name,
            x=x,
            y=0.0,
            properties=(
                {"can_wait": False, "controlled_region": region}
                if name == "I"
                else {"can_wait": True}
            ),
        )
        for name, x in coordinates.items()
    }
    controlled_pairs = {frozenset(("X", "I")), frozenset(("I", "E"))}
    node_names = list(coordinates)
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": region}
                    if frozenset((src, dst)) in controlled_pairs
                    else {}
                ),
            },
        )
        for first, second in zip(node_names, node_names[1:])
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {
                "footprint": [
                    {"x": -0.5, "y": -0.35},
                    {"x": 0.5, "y": -0.35},
                    {"x": 0.5, "y": 0.35},
                    {"x": -0.5, "y": 0.35},
                ],
            },
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
                "deadlock_retreat_after_sec": 0.5,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner_route = route(["X", "I", "E", "P", "B", "A"])
    owner = FleetRobot(
        name="owner",
        current_lm="I",
        target_lm="A",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 2.8, "y": 0.0, "yaw": math.pi},
        route_clock=1.2,
        route_revision=11,
        trajectory=owner_route,
        last_reason="occupied by portal_blocker",
        wait_for_robot="portal_blocker",
    )
    blocker_route = route(["B", "P", "E", "I", "X"])
    blocker = FleetRobot(
        name="portal_blocker",
        current_lm="B",
        target_lm="X",
        active_order_id="blocker-order",
        status="WAITING",
        pose={"x": landmarks["B"].x, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        route_revision=22,
        trajectory=blocker_route,
        last_reason="occupied by owner",
        wait_for_robot="owner",
    )
    robots = {owner.name: owner, blocker.name: blocker}

    queue_trajectories: dict[str, list[dict[str, object]]] = {}
    if queue_depth == 2:
        middle_route = route(["A", "B", "P", "E", "I", "X"])
        middle = FleetRobot(
            name="queue_middle",
            current_lm="A",
            target_lm="X",
            active_order_id="middle-order",
            status="WAITING",
            pose={"x": landmarks["A"].x, "y": 0.0, "yaw": 0.0},
            trajectory=middle_route,
            last_reason=(
                f"corridor admission wait at E for {region}; owner owner"
            ),
            wait_for_robot="owner",
        )
        robots[middle.name] = middle
        queue_trajectories[middle.name] = middle_route
        tail_current_lm = "T"
        tail_clock = 2.0
        tail_dependency = middle.name
    else:
        tail_current_lm = "A"
        tail_clock = 3.0
        tail_dependency = blocker.name

    tail_route = route(["H", "G", "T", "A", "B", "P", "E", "I", "X"])
    tail = FleetRobot(
        name="queue_tail",
        current_lm=tail_current_lm,
        target_lm="X",
        active_order_id="tail-order",
        status="WAITING",
        pose={"x": landmarks[tail_current_lm].x, "y": 0.0, "yaw": 0.0},
        route_clock=tail_clock,
        route_revision=33,
        trajectory=tail_route,
        last_reason=f"occupied by {tail_dependency}",
        wait_for_robot=tail_dependency,
    )
    robots[tail.name] = tail
    queue_trajectories[tail.name] = tail_route
    manager.robots = robots

    for robot in robots.values():
        assert robot.active_order_id is not None
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )

    owner_passage = {
        "regions": (region,),
        "entered": True,
        "committed": True,
        "route_revision": owner.route_revision,
    }
    manager._controlled_corridor_passages[owner.name] = owner_passage
    manager._controlled_corridor_occupancy = {region: [owner.name]}
    manager._controlled_corridor_leases[region] = (owner.name, 2_000.0)

    # The first action must not be another same-goal plan for the direct
    # blocker.  Keep the assertion synchronous and avoid leaving an executor
    # job behind if this regression fails.
    replan_calls: list[str] = []

    def record_replan(
        robot: FleetRobot,
        *_args: object,
        **_kwargs: object,
    ) -> tuple[bool, bool]:
        replan_calls.append(robot.name)
        return False, False

    monkeypatch.setattr(
        manager,
        "_queue_background_replan_recovery_action",
        record_replan,
    )

    now = 1_000.0
    cycle_key = tuple(sorted((owner.name, blocker.name)))
    manager._active_wait_cycles[cycle_key] = now - 2.0
    manager._break_runtime_wait_cycle(
        [owner.name, blocker.name],
        manager.robots,
        now,
        new_episode=False,
    )

    assert tail.status == "RETREATING"
    assert tail.retreat_target_lm in landmarks
    assert landmarks[tail.retreat_target_lm].x < landmarks[tail_current_lm].x
    assert blocker.status != "RETREATING"
    assert blocker.trajectory is blocker_route
    assert blocker.name not in manager._runtime_replans
    for name, original_trajectory in queue_trajectories.items():
        if name == tail.name:
            continue
        assert manager.robots[name].status != "RETREATING"
        assert manager.robots[name].trajectory is original_trajectory
    assert replan_calls == []

    assert owner.status != "RETREATING"
    assert owner.trajectory is owner_route
    assert owner.retreat_target_clock is None
    assert owner.name not in manager._runtime_replans
    assert manager._controlled_corridor_passages[owner.name] is owner_passage
    assert manager._controlled_corridor_leases[region][0] == owner.name
    for robot in robots.values():
        assert robot.active_order_id is not None
        order = manager.orders[robot.active_order_id]
        assert order.status == "EXECUTING"
        assert order.target_lm == robot.target_lm
        assert order.vehicle == robot.name


def test_corridor_exit_cycle_first_moves_inactive_body_sealing_escape_arm(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A parked third robot behind the admission loser is a causal queue tail."""
    region = "corridor:parked-tail"
    coordinates = {
        "Q": -4.0,
        "D": -3.0,
        "C": -2.0,
        "B": -1.0,
        "A": 0.0,
        "P": 1.0,
        "E": 2.0,
        "X": 3.0,
    }
    landmarks = {
        name: Landmark(name=name, x=x, y=0.0)
        for name, x in coordinates.items()
    }
    controlled_pairs = {
        frozenset(("A", "P")),
        frozenset(("P", "E")),
    }
    node_names = list(coordinates)
    edges = [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[source].to_point(),
                landmarks[target].to_point(),
            ),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": region}
                    if frozenset((source, target)) in controlled_pairs
                    else {}
                ),
            },
        )
        for first, second in zip(node_names, node_names[1:])
        for source, target in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {
                "footprint": [
                    {"x": -0.5, "y": -0.35},
                    {"x": 0.5, "y": -0.35},
                    {"x": 0.5, "y": 0.35},
                    {"x": -0.5, "y": 0.35},
                ],
            },
            "navigation": {
                "route_speed": 1.0,
                "route_acceleration": 1.0,
            },
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
                "parked_clearance_relocation_enabled": True,
                "parked_clearance_relocation_max_hops": 8,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0 if nodes[-1] == "X" else math.pi,
                "edgeId": (
                    f"{nodes[0]}->{nodes[1]}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner = FleetRobot(
        name="owner",
        current_lm="E",
        target_lm="A",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        route_clock=0.0,
        route_revision=11,
        trajectory=route(["E", "P", "A"]),
        last_reason="occupied by admission_loser",
        wait_for_robot="admission_loser",
    )
    loser = FleetRobot(
        name="admission_loser",
        current_lm="A",
        target_lm="X",
        active_order_id="loser-order",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        route_revision=22,
        trajectory=route(["A", "P", "E", "X"]),
        last_reason=(
            f"corridor admission wait at A for {region}; owner owner"
        ),
        wait_for_robot="owner",
    )
    parked_tail = FleetRobot(
        name="parked_tail",
        current_lm="C",
        status="ARRIVED",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        route_revision=33,
    )
    manager.robots = {
        owner.name: owner,
        loser.name: loser,
        parked_tail.name: parked_tail,
    }
    manager.orders = {
        owner.active_order_id: FleetOrder(
            order_id=owner.active_order_id,
            target_lm=owner.target_lm,
            vehicle=owner.name,
            assigned_robot=owner.name,
            status="WAITING_TRAFFIC",
        ),
        loser.active_order_id: FleetOrder(
            order_id=loser.active_order_id,
            target_lm=loser.target_lm,
            vehicle=loser.name,
            assigned_robot=loser.name,
            status="WAITING_TRAFFIC",
            spatial_route_nodes=["A", "P", "E", "X"],
        ),
    }
    manager._controlled_corridor_passages[owner.name] = {
        "regions": (region,),
        "entered": False,
        "committed": True,
        "route_revision": owner.route_revision,
    }
    manager._controlled_corridor_leases[region] = (owner.name, 2_000.0)

    upcoming = manager._next_controlled_corridor_entry(loser)
    assert upcoming is not None
    portal_edges = {
        (str(upcoming["src"]), str(upcoming["dst"])),
        (str(upcoming["dst"]), str(upcoming["src"])),
    }
    assert manager._stationary_clearance_route(
        owner,
        loser,
        extra_blocked_edges=portal_edges,
        avoid_controlled_regions=True,
        start_lm_override="A",
    ) == []
    assert manager._stationary_clearance_route(
        loser,
        parked_tail,
        avoid_controlled_regions=True,
        require_waiter_release=True,
    ) == ["C", "D", "Q"]

    monkeypatch.setattr(
        manager,
        "_queue_background_replan_recovery_action",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("same-goal replan must not run before tail clearance")
        ),
    )
    recovered = manager._start_deadlock_corridor_evacuation(
        [owner, loser],
        owner,
        1_000.0,
    )

    assert recovered == parked_tail.name
    clearances = [
        order
        for order in manager.orders.values()
        if order.internal_kind == "traffic_clearance"
    ]
    assert len(clearances) == 1
    assert clearances[0].vehicle == parked_tail.name
    assert clearances[0].spatial_route_nodes == ["C", "D", "Q"]
    assert loser.wait_for_robot == parked_tail.name
    assert loser.name not in manager._runtime_replans
    assert manager.orders["loser-order"].spatial_route_nodes == [
        "A",
        "P",
        "E",
        "X",
    ]


def test_open_aisle_head_on_unwinds_external_queue_from_tail(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An unannotated aisle must include the physical tail in recovery."""
    coordinates = {
        "H": -3.0,
        "G": -2.0,
        "T": -1.0,
        "A": 0.0,
        "B": 1.0,
        "P": 2.0,
        "E": 3.0,
        "X": 4.0,
    }
    landmarks = {
        name: Landmark(name=name, x=x, y=0.0)
        for name, x in coordinates.items()
    }
    node_names = list(coordinates)
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={"direction": 2},
        )
        for first, second in zip(node_names, node_names[1:])
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {
                "footprint": [
                    {"x": -0.5, "y": -0.35},
                    {"x": 0.5, "y": -0.35},
                    {"x": 0.5, "y": 0.35},
                    {"x": -0.5, "y": 0.35},
                ],
            },
            "navigation": {"route_speed": 1.0, "route_acceleration": 1.0},
            "fleet": {
                "controlled_corridors_enabled": False,
                "traffic_zone_control_enabled": False,
            },
        },
    )

    def route(nodes: list[str]) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    owner = FleetRobot(
        name="head_on_owner",
        current_lm="X",
        target_lm="H",
        active_order_id="owner-order",
        status="WAITING",
        pose={"x": 3.8, "y": 0.0, "yaw": math.pi},
        route_clock=0.2,
        trajectory=route(["X", "E", "P", "B", "A", "T", "G", "H"]),
        last_reason="occupied by direct_blocker",
        wait_for_robot="direct_blocker",
    )
    direct = FleetRobot(
        name="direct_blocker",
        current_lm="P",
        target_lm="X",
        active_order_id="direct-order",
        status="WAITING",
        pose={"x": 2.6, "y": 0.0, "yaw": 0.0},
        route_clock=2.6,
        trajectory=route(["A", "B", "P", "E", "X"]),
        last_reason="occupied by head_on_owner",
        wait_for_robot="head_on_owner",
    )
    middle = FleetRobot(
        name="queue_middle",
        current_lm="B",
        target_lm="X",
        active_order_id="middle-order",
        status="WAITING",
        pose={"x": 1.4, "y": 0.0, "yaw": 0.0},
        route_clock=2.4,
        trajectory=route(["T", "A", "B", "P", "E", "X"]),
        last_reason="occupied by direct_blocker",
        wait_for_robot="direct_blocker",
    )
    tail = FleetRobot(
        name="queue_tail",
        current_lm="A",
        target_lm="X",
        active_order_id="tail-order",
        status="WAITING",
        pose={"x": 0.2, "y": 0.0, "yaw": 0.0},
        route_clock=3.2,
        trajectory=route(["H", "G", "T", "A", "B", "P", "E", "X"]),
        last_reason="occupied by queue_middle",
        wait_for_robot="queue_middle",
    )
    manager.robots = {
        robot.name: robot
        for robot in (owner, direct, middle, tail)
    }
    for robot in manager.robots.values():
        manager.orders[robot.active_order_id] = FleetOrder(
            order_id=robot.active_order_id,
            target_lm=robot.target_lm,
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )

    replan_calls: list[str] = []

    def record_replan(
        robot: FleetRobot,
        *_args: object,
        **_kwargs: object,
    ) -> tuple[bool, bool]:
        replan_calls.append(robot.name)
        return False, False

    monkeypatch.setattr(
        manager,
        "_queue_background_replan_recovery_action",
        record_replan,
    )

    evacuated = manager._start_deadlock_corridor_evacuation(
        [owner, direct],
        owner,
        1_000.0,
    )

    assert evacuated == tail.name
    assert tail.status == "RETREATING"
    assert tail.retreat_target_lm in landmarks
    assert landmarks[tail.retreat_target_lm].x < tail.pose["x"]
    assert direct.status == "WAITING"
    assert middle.status == "WAITING"
    assert replan_calls == []


def test_plain_head_on_cycle_without_history_escapes_to_side_pocket_after_cbs_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A fresh ordinary-aisle chunk must not replan from the same blocked LM."""
    coordinates = {
        "L": (-2.0, 0.0),
        "A": (0.0, 0.0),
        "B": (2.0, 0.0),
        "R": (4.0, 0.0),
        "P": (2.0, 2.0),
    }
    landmarks = {
        name: Landmark(name=name, x=x, y=y)
        for name, (x, y) in coordinates.items()
    }
    undirected_edges = (("L", "A"), ("A", "B"), ("B", "R"), ("B", "P"))
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=math.hypot(
                landmarks[dst].x - landmarks[src].x,
                landmarks[dst].y - landmarks[src].y,
            ),
            kind="line",
            edge_type="FeatureLine",
            world_points=(landmarks[src].to_point(), landmarks[dst].to_point()),
            properties={"direction": 2},
        )
        for first, second in undirected_edges
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {
                "route_speed": 1.0,
                "route_acceleration": 1.0,
            },
            "fleet": {
                "controlled_corridors_enabled": False,
                "traffic_zone_control_enabled": False,
                "deadlock_retreat_after_sec": 0.5,
            },
        },
    )

    def route(nodes: list[str], yaw: float) -> list[dict[str, object]]:
        return [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": yaw,
                "edgeId": (
                    f"{node}->{node}"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ]

    winner_route = route(["A", "B", "R"], 0.0)
    loser_route = route(["B", "A", "L"], math.pi)
    winner = FleetRobot(
        name="a_winner",
        current_lm="A",
        target_lm="R",
        active_order_id="winner-order",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_clock=0.0,
        route_revision=11,
        trajectory=winner_route,
        last_reason="occupied by z_loser",
        wait_for_robot="z_loser",
    )
    loser = FleetRobot(
        name="z_loser",
        current_lm="B",
        target_lm="L",
        active_order_id="loser-order",
        status="WAITING",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        route_clock=0.0,
        route_revision=22,
        trajectory=loser_route,
        last_reason="occupied by a_winner",
        wait_for_robot="a_winner",
    )
    manager.robots = {
        winner.name: winner,
        loser.name: loser,
    }
    manager.orders = {
        "winner-order": FleetOrder(
            order_id="winner-order",
            target_lm="R",
            vehicle=winner.name,
            assigned_robot=winner.name,
            status="EXECUTING",
            spatial_route_nodes=["A", "B", "R"],
        ),
        "loser-order": FleetOrder(
            order_id="loser-order",
            target_lm="L",
            vehicle=loser.name,
            assigned_robot=loser.name,
            status="EXECUTING",
            spatial_route_nodes=["B", "A", "L"],
        ),
    }

    now = 1_000.0
    cycle_key = tuple(sorted(manager.robots))
    manager._active_wait_cycles[cycle_key] = now - 2.0
    manager._coupled_replan_failures[cycle_key] = 1
    monkeypatch.setattr(
        manager,
        "_start_async_coupled_replan",
        lambda *_args, **_kwargs: False,
    )

    manager._resolve_runtime_wait_cycles(now)

    assert loser.status == "RETREATING"
    assert loser.retreat_target_lm == "P"
    assert any(sample.get("lm") == "P" for sample in loser.trajectory)
    assert loser.active_order_id == "loser-order"
    assert manager.orders["loser-order"].target_lm == "L"
    assert winner.trajectory is winner_route
    assert winner.active_order_id == "winner-order"

    # Completing the escape is a transaction boundary: the original order is
    # retained, but its replacement route must start at the side pocket.
    loser.route_clock = 0.0
    manager._advance_deadlock_retreat(loser, now + 0.1)

    assert loser.current_lm == "P"
    assert loser.active_order_id == "loser-order"
    assert manager.orders["loser-order"].target_lm == "L"
    assert manager._runtime_replans[loser.name]["order_id"] == "loser-order"
    assert manager._runtime_replans[loser.name]["start_lm"] == "P"


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
    assert entrant.status == "WAITING"
    assert entrant.trajectory
    assert entrant.active_order_id == "entrant-order"
    entrant_order = manager.orders["entrant-order"]
    assert entrant_order.status == "EXECUTING"
    assert entrant_order.traffic_detour_edges == []
    assert entrant_order.traffic_detour_attempts == 0
    assert manager.traffic_metrics["cycleReplans"] == 0


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


def test_inactive_blocker_gets_hidden_clearance_order_to_unused_pocket(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        last_reason="occupied by blocker",
        wait_for_robot="blocker",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": -2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
            {"t": 4.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "B->C", "lm": "C"},
        ],
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {
        "wait-order": FleetOrder(
            order_id="wait-order",
            target_lm="C",
            vehicle=waiter.name,
            assigned_robot=waiter.name,
            status="WAITING_TRAFFIC",
            spatial_route_nodes=["A", "B", "C"],
        ),
    }

    assert manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="focused test",
    )
    internal = [
        order for order in manager.orders.values()
        if order.internal_kind == "traffic_clearance"
    ]
    assert len(internal) == 1
    assert internal[0].vehicle == "blocker"
    assert internal[0].target_lm == "P"
    assert internal[0].spatial_route_nodes == ["B", "P"]
    assert internal[0].priority >= 10_000
    assert not manager._order_enabled(internal[0])
    assert internal[0].order_id not in {
        payload["id"] for payload in manager._orders_list()
    }
    captured_routes: list[list[str]] = []
    monkeypatch.setattr(
        manager,
        "_dispatch_simulated_order_batch",
        lambda group: (
            captured_routes.append(list(group[0][0].spatial_route_nodes)) or 0,
            set(),
        ),
    )
    manager._dispatch_orders()
    assert captured_routes == [["B", "P"]]
    assert not manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="duplicate",
    )


def test_stationary_clearance_at_owned_corridor_portal_vacates_outward() -> None:
    landmarks = {
        "OWNER": Landmark(name="OWNER", x=0.0, y=10.0),
        "W2": Landmark(name="W2", x=-2.0, y=0.0),
        "W1": Landmark(name="W1", x=-1.0, y=0.0),
        "P": Landmark(
            name="P",
            x=0.0,
            y=0.0,
            properties={"holding_point": True},
        ),
        "E": Landmark(
            name="E",
            x=2.0,
            y=0.0,
            properties={"holding_point": True},
        ),
    }
    edges = []
    region_id = "corridor-test"
    for first, second, controlled in (
        ("P", "W1", False),
        ("W1", "W2", False),
        # A valid authored corridor can consist of a single controlled lane
        # whose two endpoints are both external stop lines.
        ("P", "E", True),
    ):
        for source, target in ((first, second), (second, first)):
            start = landmarks[source]
            goal = landmarks[target]
            properties: dict[str, object] = {"direction": 2}
            if controlled:
                properties["controlled_region"] = region_id
            edges.append(GraphEdge(
                from_name=source,
                to_name=target,
                length=math.hypot(goal.x - start.x, goal.y - start.y),
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(start.x, start.y),
                    WorldPoint(goal.x, goal.y),
                ),
                properties=properties,
            ))
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "parked_clearance_relocation_enabled": True,
                "parked_clearance_relocation_max_hops": 4,
            },
        },
    )
    owner = FleetRobot(
        name="owner",
        current_lm="OWNER",
        target_lm="OWNER",
        status="WAITING",
        active_order_id="owner-order",
        last_reason="occupied by blocker",
        wait_for_robot="blocker",
        pose={"x": 0.0, "y": 10.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="P",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {owner.name: owner, blocker.name: blocker}
    manager.orders = {
        "owner-order": FleetOrder(
            order_id="owner-order",
            target_lm="OWNER",
            vehicle=owner.name,
            assigned_robot=owner.name,
            status="WAITING_TRAFFIC",
        ),
    }
    lease = (owner.name, manager._now() + 100.0)
    manager._controlled_corridor_leases[region_id] = lease

    assert manager._queue_stationary_clearance_relocation(
        owner,
        blocker,
        cause="corridor exit portal occupied",
    )

    clearance = next(
        order
        for order in manager.orders.values()
        if order.internal_kind == "traffic_clearance"
    )
    assert clearance.spatial_route_nodes == ["P", "W1", "W2"]
    assert all(
        not manager._controlled_corridor_graph.lane_for(
            source,
            target,
        ).controlled_region_ids
        for source, target in zip(
            clearance.spatial_route_nodes,
            clearance.spatial_route_nodes[1:],
        )
    )
    # Maintenance never steals or invalidates the authoritative owner's
    # passage; it simply leaves through the external graph arm.
    assert manager._controlled_corridor_leases[region_id] == lease


def test_stationary_clearance_uses_only_an_unowned_corridor_that_releases_waiter(
) -> None:
    """Do not march a parked blocker through the waiter's unavoidable lane."""
    landmarks = {
        "G": Landmark(name="G", x=0.0, y=0.0),
        "P": Landmark(name="P", x=1.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
        "J": Landmark(name="J", x=3.0, y=0.0),
        "W": Landmark(name="W", x=4.0, y=0.0),
        "Q": Landmark(
            name="Q",
            x=2.0,
            y=3.0,
            properties={"holding_point": True},
        ),
    }
    region_id = "corridor-release-branch"
    edges: list[GraphEdge] = []
    for first, second, controlled in (
        ("G", "P", False),
        ("P", "B", False),
        ("B", "J", False),
        ("J", "W", False),
        ("B", "Q", True),
    ):
        for source, target in ((first, second), (second, first)):
            start = landmarks[source]
            goal = landmarks[target]
            properties: dict[str, object] = {"direction": 2}
            if controlled:
                properties["controlled_region"] = region_id
            edges.append(GraphEdge(
                from_name=source,
                to_name=target,
                length=math.hypot(goal.x - start.x, goal.y - start.y),
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(start.x, start.y),
                    WorldPoint(goal.x, goal.y),
                ),
                properties=properties,
            ))
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "parked_clearance_relocation_enabled": True,
                "parked_clearance_relocation_max_hops": 4,
            },
        },
    )
    waiter = FleetRobot(
        name="waiter",
        current_lm="W",
        target_lm="G",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": 4.0, "y": 0.0, "yaw": math.pi},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="G",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}

    # The only useful pocket is Q. P/J merely relocate the same obstacle
    # along W->G, while a waiter-owned corridor would create a circular wait.
    manager._controlled_corridor_leases[region_id] = (
        waiter.name,
        manager._now() + 100.0,
    )
    assert not manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="owned release branch",
    )
    manager._controlled_corridor_leases.pop(region_id)

    assert manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="free release branch",
    )
    clearance = next(
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    )
    assert clearance.spatial_route_nodes == ["B", "Q"]


def test_clearance_traffic_timeout_preserves_external_route_and_owner_lease(
) -> None:
    manager = _clearance_manager()
    now = manager._now()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        last_reason="occupied by blocker",
        wait_for_robot="blocker",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
        spatial_route_nodes=["A", "B", "C"],
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {waiter_order.order_id: waiter_order}
    assert manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="focused timeout invariant",
    )

    clearance = next(
        order
        for order in manager.orders.values()
        if order.internal_kind == "traffic_clearance"
    )
    route = list(clearance.spatial_route_nodes)
    assert route == ["B", "P"]
    traffic_graph = manager.planner._traffic_graph(
        manager.planner._route_speed({}),
    )
    assert all(
        not traffic_graph.lane_for(
            source,
            target,
        ).controlled_region_ids
        for source, target in zip(route, route[1:])
    )

    clearance.status = "EXECUTING"
    clearance.assigned_robot = blocker.name
    clearance.start_lm = "B"
    blocker.active_order_id = clearance.order_id
    blocker.target_lm = clearance.target_lm
    blocker.status = "WAITING"
    blocker.trajectory = [
        {
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "B->P",
            "lm": "B",
        },
        {
            "t": 2.0,
            "x": 0.0,
            "y": 2.0,
            "yaw": math.pi / 2.0,
            "edgeId": "B->P",
            "lm": "P",
        },
    ]
    blocker.plan_nodes = list(route)
    blocker.route_revision = 7
    original_stall_since = now - 30.0
    blocker.traffic_stall_since = original_stall_since

    region_id = "corridor-owner"
    lease = (waiter.name, now + 100.0)
    manager._controlled_corridor_leases[region_id] = lease

    assert not manager._schedule_runtime_replan(
        blocker,
        now,
        (
            "corridor admission timeout: wait at B for "
            f"{region_id}; owner {waiter.name}"
        ),
    )
    assert clearance.spatial_route_nodes == route
    assert clearance.traffic_detour_edges == []
    assert blocker.name not in manager._runtime_replans
    assert manager._controlled_corridor_leases[region_id] == lease
    assert blocker.traffic_stall_since == original_stall_since
    assert not manager._schedule_runtime_replan(
        blocker,
        now + 1.0,
        (
            "corridor admission timeout: wait at B for "
            f"{region_id}; owner {waiter.name}"
        ),
    )
    assert blocker.traffic_stall_since == original_stall_since

    # A generic dispatch failure used to clear the same explicit route after
    # the traffic timeout, so the following retry ran congestion A* and could
    # enter the owner's corridor.
    clearance.traffic_blocked_since = now - 100.0
    manager._set_order_error(clearance, "no_sipp_path:reserved_edge")
    assert clearance.spatial_route_nodes == route
    assert clearance.traffic_detour_edges == []
    assert manager._controlled_corridor_leases[region_id] == lease

    # Even if the pocket is temporarily occupied, the maintenance order waits
    # on its authoritative route instead of replacing it spatially.
    pocket_occupant = FleetRobot(
        name="pocket-occupant",
        current_lm="P",
        status="IDLE",
        pose={"x": 0.0, "y": 2.0, "yaw": 0.0},
    )
    manager.robots[pocket_occupant.name] = pocket_occupant
    assert manager._ensure_order_spatial_route(
        clearance,
        "B",
        "P",
    ) == route
    assert clearance.spatial_route_nodes == route
    assert manager._controlled_corridor_leases[region_id] == lease


def test_invalid_clearance_route_never_falls_back_to_free_spatial_planning(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    clearance = FleetOrder(
        order_id="traffic-clearance-blocker",
        target_lm="P",
        vehicle=blocker.name,
        status="QUEUED",
        # B->C is valid graph data, but it is not the authored route to P.
        spatial_route_nodes=["B", "C"],
        internal_kind="traffic_clearance",
    )
    manager.robots = {blocker.name: blocker}
    manager.orders = {clearance.order_id: clearance}
    planner_calls: list[list[dict[str, object]]] = []
    monkeypatch.setattr(
        manager,
        "_plan_valid_requests",
        lambda requests, _payload: (
            planner_calls.append(requests)
            or {"ok": False, "plans": [], "debug": {}}
        ),
    )

    manager._start_async_simulated_dispatch(
        [(
            clearance,
            blocker,
            {"name": blocker.name, "startLm": "B", "goalLm": "P"},
            "P",
        )],
    )

    assert manager._dispatch_job is None
    assert planner_calls == []
    assert clearance.status == "QUEUED"
    assert clearance.spatial_route_nodes == ["B", "C"]
    assert "traffic clearance route invalid" in clearance.error
    dispatched, handled = manager._dispatch_simulated_order_batch(
        [(
            clearance,
            blocker,
            {"name": blocker.name, "startLm": "B", "goalLm": "P"},
            "P",
        )],
    )
    assert dispatched == 0
    assert handled == {clearance.order_id}
    assert planner_calls == []


def test_clearance_plan_must_exactly_follow_requested_route() -> None:
    manager = _clearance_manager()
    clearance = FleetOrder(
        order_id="traffic-clearance-blocker",
        target_lm="P",
        vehicle="blocker",
        status="PLANNING",
        spatial_route_nodes=["B", "P"],
        internal_kind="traffic_clearance",
    )
    request = {
        "name": "blocker",
        "startLm": "B",
        "goalLm": "P",
        "routeNodes": ["B", "C", "P"],
    }

    assert manager._plan_follows_requested_clearance_route(
        clearance,
        request,
        {"nodes": ["B", "B", "C"]},
    )
    assert not manager._plan_follows_requested_clearance_route(
        clearance,
        request,
        {"nodes": ["B", "P"]},
    )


def test_local_cbs_never_reroutes_active_traffic_clearance() -> None:
    manager = _clearance_manager()
    clearance_robot = FleetRobot(
        name="clearance",
        current_lm="B",
        status="WAITING",
        active_order_id="traffic-clearance",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    normal_robot = FleetRobot(
        name="normal",
        current_lm="A",
        status="WAITING",
        active_order_id="normal-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {
        clearance_robot.name: clearance_robot,
        normal_robot.name: normal_robot,
    }
    manager.orders = {
        "traffic-clearance": FleetOrder(
            order_id="traffic-clearance",
            target_lm="P",
            vehicle=clearance_robot.name,
            assigned_robot=clearance_robot.name,
            status="WAITING_TRAFFIC",
            spatial_route_nodes=["B", "P"],
            internal_kind="traffic_clearance",
        ),
        "normal-order": FleetOrder(
            order_id="normal-order",
            target_lm="C",
            vehicle=normal_robot.name,
            assigned_robot=normal_robot.name,
            status="WAITING_TRAFFIC",
            spatial_route_nodes=["A", "B", "C"],
        ),
    }

    assert not manager._start_async_coupled_replan(
        [clearance_robot, normal_robot],
        normal_robot,
        manager._now(),
    )
    assert manager._dispatch_job is None
    assert manager.orders["traffic-clearance"].spatial_route_nodes == ["B", "P"]
    assert manager._rolling_full_collapse_release_entries() is None


def test_orphaned_clearance_is_canceled_at_safe_lm_without_touching_waiter_order(
) -> None:
    manager = _clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {waiter_order.order_id: waiter_order}

    assert manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="focused lifecycle test",
    )
    state = manager._stationary_clearance_relocations[blocker.name]
    clearance_id = str(state["order_id"])
    clearance = manager.orders[clearance_id]
    clearance.status = "WAITING_TRAFFIC"
    blocker.active_order_id = clearance_id
    blocker.target_lm = clearance.target_lm
    blocker.status = "WAITING"
    blocker.trajectory = [
        {
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "B->B",
            "lm": "B",
        },
        {
            "t": 1.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "B->B",
            "lm": "B",
        },
    ]

    # The package order is authoritative. Finishing it invalidates the
    # maintenance lease but must never rewrite its terminal result.
    waiter_order.status = "COMPLETED"
    manager._prune_stationary_clearance_relocations(manager._now())

    assert waiter_order.status == "COMPLETED"
    assert clearance_id not in manager.orders
    assert state["order_id"] == ""
    assert blocker.active_order_id == ""
    assert blocker.target_lm == ""
    assert blocker.status == "IDLE"
    assert blocker.current_lm == "B"
    assert blocker.trajectory == []


def test_expired_clearance_mid_edge_keeps_motion_to_next_safe_lm() -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_timeout_sec"] = 20.0
    now = manager._now()
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        target_lm="P",
        status="MOVING",
        active_order_id="traffic-clearance-blocker",
        pose={"x": 0.0, "y": 1.0, "yaw": math.pi / 2.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": math.pi / 2.0,
                "edgeId": "B->B",
                "lm": "B",
            },
            {
                "t": 1.0,
                "x": 0.0,
                "y": 1.0,
                "yaw": math.pi / 2.0,
                "edgeId": "B->P",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 2.0,
                "yaw": math.pi / 2.0,
                "edgeId": "B->P",
                "lm": "P",
            },
        ],
        plan_nodes=["B", "P"],
        route_clock=1.0,
        route_revision=7,
    )
    clearance = FleetOrder(
        order_id=blocker.active_order_id,
        target_lm="P",
        targets=["P"],
        vehicle=blocker.name,
        assigned_robot=blocker.name,
        status="EXECUTING",
        spatial_route_nodes=["B", "P"],
        internal_kind="traffic_clearance",
    )
    state = {
        "order_id": clearance.order_id,
        "waiter_signature": (),
        "queued_at": now - 21.0,
        "cooldown_sec": manager._stationary_clearance_cooldown(),
    }
    manager.robots = {blocker.name: blocker}
    manager.orders = {clearance.order_id: clearance}
    manager._stationary_clearance_relocations = {blocker.name: state}
    pose_before = dict(blocker.pose)

    manager._prune_stationary_clearance_relocations(now)

    assert blocker.pose == pose_before
    assert blocker.active_order_id == clearance.order_id
    assert clearance.order_id in manager.orders
    assert state["safe_stop_lm"] == "P"
    assert blocker.trajectory[-1]["lm"] == "P"
    assert float(blocker.trajectory[-1]["t"]) == 2.0

    # Once telemetry/motion reaches the armed boundary, cleanup is atomic and
    # must not snap the visual/physical pose onto the LM centre.
    blocker.pose = {"x": 0.04, "y": 2.0, "yaw": math.pi / 2.0}
    reached_pose = dict(blocker.pose)
    blocker.route_clock = 2.0
    blocker.status = "WAITING"
    manager._prune_stationary_clearance_relocations(now + 1.0)

    assert blocker.pose == reached_pose
    assert blocker.active_order_id == ""
    assert clearance.order_id not in manager.orders
    assert state["order_id"] == ""


def test_expired_remote_clearance_cancels_transport_before_local_cleanup(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_timeout_sec"] = 20.0
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    remote = FleetRobot(
        name="remote-blocker",
        current_lm="B",
        mode="grpc",
        target_lm="P",
        status="WAITING",
        active_order_id="traffic-clearance-remote",
        base_url="grpc://127.0.0.1:50051",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
            {"t": 1.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
        ],
    )
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
    )
    clearance = FleetOrder(
        order_id=remote.active_order_id,
        target_lm="P",
        vehicle=remote.name,
        assigned_robot=remote.name,
        status="WAITING_TRAFFIC",
        internal_kind="traffic_clearance",
    )
    now = manager._now()
    manager.robots = {waiter.name: waiter, remote.name: remote}
    manager.orders = {
        waiter_order.order_id: waiter_order,
        clearance.order_id: clearance,
    }
    manager._stationary_clearance_relocations = {
        remote.name: {
            "order_id": clearance.order_id,
            "waiter": waiter.name,
            "waiter_signature": (
                waiter.name,
                waiter_order.order_id,
                waiter.current_lm,
                waiter_order.target_lm,
            ),
            "queued_at": now - 21.0,
            "cooldown_sec": manager._stationary_clearance_cooldown(),
        },
    }
    cancel_calls: list[tuple[str, str, str]] = []
    monkeypatch.setattr(
        manager,
        "_cancel_remote_route",
        lambda robot, reason: (
            cancel_calls.append((robot.name, robot.active_order_id, reason))
            or True
        ),
    )

    manager._prune_stationary_clearance_relocations(now)

    assert cancel_calls == [(
        remote.name,
        clearance.order_id,
        "traffic clearance canceled: maintenance move exceeded its bounded lifetime",
    )]
    assert waiter_order.status == "WAITING_TRAFFIC"
    assert remote.active_order_id == ""
    assert remote.status == "IDLE"
    assert clearance.order_id not in manager.orders


def test_clearance_order_never_spawns_a_nested_clearance_chain(
) -> None:
    manager = _clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    first_blocker = FleetRobot(
        name="first-blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    second_blocker = FleetRobot(
        name="second-blocker",
        current_lm="C",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {
        robot.name: robot
        for robot in (waiter, first_blocker)
    }
    manager.orders = {
        "wait-order": FleetOrder(
            order_id="wait-order",
            target_lm="C",
            vehicle=waiter.name,
            assigned_robot=waiter.name,
            status="WAITING_TRAFFIC",
        ),
    }

    assert manager._queue_stationary_clearance_relocation(
        waiter,
        first_blocker,
        cause="first recovery",
    )
    first_clearance = next(
        order
        for order in manager.orders.values()
        if order.internal_kind == "traffic_clearance"
    )
    manager.robots[second_blocker.name] = second_blocker

    # A maintenance order is never itself a causal user order.
    first_blocker.active_order_id = first_clearance.order_id
    assert not manager._queue_stationary_clearance_relocation(
        first_blocker,
        second_blocker,
        cause="must not chain",
    )
    first_blocker.status = "WAITING"
    first_blocker.wait_for_robot = second_blocker.name
    first_blocker.last_reason = f"occupied by {second_blocker.name}"
    first_blocker.trajectory = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
        {"t": 1.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
    ]
    manager.orders["second-departure"] = FleetOrder(
        order_id="second-departure",
        target_lm="P",
        vehicle=second_blocker.name,
        assigned_robot=second_blocker.name,
        status="QUEUED",
    )

    # Nor may the same hidden move promote the robot it happens to wait for
    # into the normal commanded-sink recovery path.
    assert manager._live_stationary_wait_chain_sink_names() == set()
    assert manager._live_waiters_for_stationary_sink(second_blocker) == []

    assert [
        order.order_id
        for order in manager.orders.values()
        if order.internal_kind == "traffic_clearance"
        and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
    ] == [first_clearance.order_id]


def test_mutual_orphaned_clearances_are_retired_and_next_wave_can_dispatch(
) -> None:
    manager = _clearance_manager()
    first = FleetRobot(
        name="bench_008",
        current_lm="B",
        target_lm="P",
        status="WAITING",
        active_order_id="traffic-clearance-bench_008",
        last_reason="occupied by bench_012",
        wait_for_robot="bench_012",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
            {"t": 1.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
        ],
    )
    second = FleetRobot(
        name="bench_012",
        current_lm="P",
        target_lm="B",
        status="WAITING",
        active_order_id="traffic-clearance-bench_012",
        last_reason="corridor admission owner bench_008",
        wait_for_robot="bench_008",
        pose={"x": 0.0, "y": 2.0, "yaw": math.pi},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 2.0, "edgeId": "P->P", "lm": "P"},
            {"t": 1.0, "x": 0.0, "y": 2.0, "edgeId": "P->P", "lm": "P"},
        ],
    )
    first_clearance = FleetOrder(
        order_id=first.active_order_id,
        target_lm="P",
        vehicle=first.name,
        assigned_robot=first.name,
        status="WAITING_TRAFFIC",
        internal_kind="traffic_clearance",
    )
    second_clearance = FleetOrder(
        order_id=second.active_order_id,
        target_lm="B",
        vehicle=second.name,
        assigned_robot=second.name,
        status="WAITING_TRAFFIC",
        internal_kind="traffic_clearance",
    )
    now = manager._now()
    manager.robots = {first.name: first, second.name: second}
    manager.orders = {
        first_clearance.order_id: first_clearance,
        second_clearance.order_id: second_clearance,
    }
    manager._stationary_clearance_relocations = {
        first.name: {
            "order_id": first_clearance.order_id,
            "waiter": second.name,
            "waiter_signature": (
                second.name,
                second_clearance.order_id,
                second.current_lm,
                second_clearance.target_lm,
            ),
            "queued_at": now - 10.0,
            "cooldown_sec": manager._stationary_clearance_cooldown(),
        },
        second.name: {
            "order_id": second_clearance.order_id,
            "waiter": first.name,
            "waiter_signature": (
                first.name,
                first_clearance.order_id,
                first.current_lm,
                first_clearance.target_lm,
            ),
            "queued_at": now - 10.0,
            "cooldown_sec": manager._stationary_clearance_cooldown(),
        },
    }

    manager._prune_stationary_clearance_relocations(now)

    assert all(robot.status == "IDLE" for robot in (first, second))
    assert all(not robot.active_order_id for robot in (first, second))
    assert all(not robot.trajectory for robot in (first, second))
    assert not any(
        order.internal_kind == "traffic_clearance"
        for order in manager.orders.values()
    )

    first_next = FleetOrder(
        order_id="dynamic-next-008",
        target_lm="A",
        vehicle=first.name,
        status="QUEUED",
    )
    second_next = FleetOrder(
        order_id="dynamic-next-012",
        target_lm="C",
        vehicle=second.name,
        status="QUEUED",
    )
    manager.orders.update({
        first_next.order_id: first_next,
        second_next.order_id: second_next,
    })

    ready = manager._ready_simulated_order_entries([first_next, second_next])
    assert {robot.name for _, robot, _, _ in ready} == {
        first.name,
        second.name,
    }


def test_stationary_clearance_pocket_stays_outside_active_goal_envelope() -> None:
    landmarks = {
        "A": Landmark(name="A", x=-2.0, y=0.0),
        "B": Landmark(name="B", x=0.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "P": Landmark(name="P", x=1.4, y=0.0),
        "Q": Landmark(name="Q", x=0.0, y=3.5),
    }
    edges = []
    for source, target in (("A", "B"), ("B", "C"), ("B", "P"), ("B", "Q")):
        start = landmarks[source]
        goal = landmarks[target]
        edges.append(GraphEdge(
            from_name=source,
            to_name=target,
            length=math.hypot(goal.x - start.x, goal.y - start.y),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(start.x, start.y),
                WorldPoint(goal.x, goal.y),
            ),
            properties={"direction": 2},
        ))
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {"parked_clearance_relocation_max_hops": 4},
        },
    )
    # P is a legal graph LM and is far enough from the blocker's origin B,
    # but a robot parked there would still overlap the arrival/turn envelope
    # of the active goal C.
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {
        "wait-order": FleetOrder(
            order_id="wait-order",
            target_lm="C",
            vehicle=waiter.name,
            assigned_robot=waiter.name,
            status="WAITING_TRAFFIC",
            spatial_route_nodes=["A", "B", "C"],
        ),
    }

    assert manager._stationary_clearance_route(waiter, blocker) == ["B", "Q"]


def test_stationary_clearance_route_rejects_causally_held_waiter_body() -> None:
    manager = _causal_clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="G",
        status="WAITING",
        active_order_id="wait-order",
        wait_for_robot="blocker",
        last_reason="occupied by blocker",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_revision=7,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        route_revision=3,
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="G",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
        spatial_route_nodes=["A", "B", "G"],
    )
    now = manager._now()
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    manager._runtime_replans[waiter.name] = {
        "order_id": order.order_id,
        "start_lm": "A",
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "blocker_names": (blocker.name,),
        "causal_blocker_signatures": ((
            blocker.name,
            blocker.current_lm,
            blocker.route_revision,
        ),),
        "queued_at": now,
        "retry_at": now,
        "failures": 0,
        "generation": 1,
        "stage": "planning",
    }

    assert manager._runtime_replan_holds_robot(waiter)
    # B->A->P is the only pocket route, but it would drive the blocker
    # directly through the body which is being held at A for this blocker.
    assert manager._stationary_clearance_route(waiter, blocker) == []


@pytest.mark.parametrize("with_blocker_identity", [True, False])
def test_runtime_replan_releases_a_proven_alternate_stationary_cut(
    with_blocker_identity: bool,
) -> None:
    manager = _stationary_cut_manager(staged=False)
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="G",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 3.0, "x": 3.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_revision=7,
    )
    direct_blocker = FleetRobot(
        name="direct-blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 3.0, "y": 0.0, "yaw": math.pi},
    )
    alternate_blocker = FleetRobot(
        name="alternate-blocker",
        current_lm="D",
        status="ARRIVED",
        pose={"x": 3.0, "y": 3.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="G",
        targets=["G"],
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
    )
    now = manager._now()
    state: dict = {
        "order_id": order.order_id,
        "start_lm": "A",
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "queued_at": now,
        "retry_at": now,
        "failures": 0,
        "generation": 1,
        "stage": "planning",
        "retained_route_superseded": True,
    }
    if with_blocker_identity:
        state["blocker_names"] = (direct_blocker.name,)
        state["causal_blocker_signatures"] = ((
            direct_blocker.name,
            direct_blocker.current_lm,
            direct_blocker.route_revision,
        ),)
    manager.robots = {
        robot.name: robot
        for robot in (waiter, direct_blocker, alternate_blocker)
    }
    manager.orders = {order.order_id: order}
    manager._runtime_replans[waiter.name] = state

    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        "stationary_robot_blocks_route",
        debug=(
            {
                "stationaryBlockerRobots": [direct_blocker.name],
                "softBlockedLms": ["B", "D"],
            }
            if with_blocker_identity
            else {}
        ),
    )

    clearances = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearances) == 1
    assert clearances[0].vehicle == alternate_blocker.name
    assert clearances[0].spatial_route_nodes == ["D", "Q"]
    assert state["clearance_blocker_names"] == (alternate_blocker.name,)
    assert waiter.status == "WAITING"
    assert waiter.active_order_id == order.order_id
    assert order.target_lm == "G"


def test_active_waiter_escapes_before_parked_blocker_clearance() -> None:
    manager = _stationary_cut_manager(staged=True)
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="G",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 3.0, "x": 3.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_revision=7,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 3.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="G",
        targets=["G"],
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
    )
    now = manager._now()
    state = {
        "order_id": order.order_id,
        "start_lm": "A",
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "blocker_names": (blocker.name,),
        "causal_blocker_signatures": ((
            blocker.name,
            blocker.current_lm,
            blocker.route_revision,
        ),),
        "queued_at": now,
        "retry_at": now,
        "failures": 0,
        "generation": 1,
        "stage": "planning",
        "retained_route_superseded": True,
    }
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    manager._runtime_replans[waiter.name] = state

    # The blocker's only useful pocket path crosses waiter@A.
    assert not manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="pre-escape proof",
    )
    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        "stationary_robot_blocks_route",
        debug={
            "stationaryBlockerRobots": [blocker.name],
            "softBlockedLms": [blocker.current_lm],
        },
    )

    assert waiter.status == "RETREATING"
    assert waiter.retreat_target_lm == "P"
    assert waiter.retreat_blocker_signatures == [
        (blocker.name, blocker.current_lm, blocker.route_revision),
    ]
    assert waiter.active_order_id == order.order_id
    assert order.status == "EXECUTING"
    assert order.target_lm == "G"
    assert waiter.name not in manager._runtime_replans

    # Complete the graph escape without advancing the original goal route.
    waiter.route_clock = 0.0
    waiter.pose = manager._pose_at_trajectory(waiter.trajectory, 0.0)
    manager._advance_deadlock_retreat(waiter, manager._now() + 0.1)

    assert waiter.current_lm == "P"
    replan = manager._runtime_replans[waiter.name]
    assert replan["causal_blocker_signatures"] == ((
        blocker.name,
        blocker.current_lm,
        blocker.route_revision,
    ),)
    assert replan["clearance_blocker_names"] == (blocker.name,)
    clearances = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearances) == 1
    assert clearances[0].vehicle == blocker.name
    assert clearances[0].spatial_route_nodes == ["B", "A", "Q"]
    assert manager._ready_runtime_replan_entry(
        replan["retry_at"] + 100.0,
    ) is None

    clearances[0].status = "COMPLETED"
    ready = manager._ready_runtime_replan_entry(
        replan["retry_at"] + 100.0,
    )
    assert ready is not None
    assert ready[1] is waiter
    assert order.target_lm == "G"


def test_prune_cancels_clearance_crossing_causally_held_waiter() -> None:
    manager = _causal_clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="G",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_revision=7,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        route_revision=3,
    )
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="G",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_OBSTACLE",
        spatial_route_nodes=["A", "B", "G"],
    )
    clearance = FleetOrder(
        order_id="traffic-clearance-blocker",
        target_lm="P",
        vehicle=blocker.name,
        status="QUEUED",
        spatial_route_nodes=["B", "A", "P"],
        internal_kind="traffic_clearance",
    )
    now = manager._now()
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {
        waiter_order.order_id: waiter_order,
        clearance.order_id: clearance,
    }
    manager._runtime_replans[waiter.name] = {
        "order_id": waiter_order.order_id,
        "start_lm": "A",
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "blocker_names": (blocker.name,),
        "causal_blocker_signatures": ((
            blocker.name,
            blocker.current_lm,
            blocker.route_revision,
        ),),
        "clearance_blocker_names": (blocker.name,),
        "queued_at": now,
        "retry_at": now,
        "failures": 1,
        "generation": 1,
        "stage": "queued",
    }
    manager._stationary_clearance_relocations = {
        blocker.name: {
            "order_id": clearance.order_id,
            "waiter": waiter.name,
            "waiter_signature": (
                waiter.name,
                waiter_order.order_id,
                waiter.current_lm,
                waiter_order.target_lm,
            ),
            "origin_lm": "B",
            "target_lm": "P",
            "visited_lms": ("B", "P"),
            "queued_at": now,
            "cooldown_until": 0.0,
            "cooldown_sec": manager._stationary_clearance_cooldown(),
        },
    }

    assert manager._runtime_replan_holds_robot(waiter)
    manager._prune_stationary_clearance_relocations(now)

    assert clearance.order_id not in manager.orders
    ready = manager._ready_runtime_replan_entry(now)
    assert ready is not None
    assert ready[0] is waiter_order
    assert ready[1] is waiter
    assert ready[2] is manager._runtime_replans[waiter.name]


def test_runtime_replan_failure_queues_parked_clearance_before_retry() -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 1
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": -2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_revision=7,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
        spatial_route_nodes=["A", "B", "C"],
    )
    state = {
        "reason": "occupied by blocker",
        "stage": "planning",
        "failures": 0,
    }
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    manager._runtime_replans[waiter.name] = state

    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        "stationary_robot_blocks_route",
    )

    assert state["stage"] == "retry"
    assert any(
        candidate.internal_kind == "traffic_clearance"
        and candidate.vehicle == blocker.name
        for candidate in manager.orders.values()
    )


def test_runtime_replan_uses_debug_stationary_blocker_once() -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 2
    trajectory = [
        {"t": 0.0, "x": -2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
    ]
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=list(trajectory),
        route_revision=7,
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
        spatial_route_nodes=["A", "B", "C"],
    )
    state = {
        "order_id": order.order_id,
        "start_lm": "A",
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        "queued_at": manager._now(),
        "retry_at": manager._now(),
        "failures": 0,
        "generation": 1,
        "stage": "planning",
    }
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    manager._runtime_replans[waiter.name] = state
    result = {
        "ok": False,
        "plans": [],
        "debug": {
            "reason": "no_low_level_path:stationary_robot_blocks_route",
            "stationaryRobotWait": True,
            # The original transaction reason intentionally has no identity;
            # only planner validation knows the actual inactive owner.
            "stationaryBlockerRobots": [blocker.name],
            "softBlockedLms": ["B"],
        },
    }
    job = {
        "order_id": order.order_id,
        "robot_name": waiter.name,
        "generation": state["generation"],
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "start_lm": "A",
        "final_goal": "C",
        "result": result,
    }

    assert manager._finish_async_runtime_replan(job) == 0
    assert not any(
        candidate.internal_kind == "traffic_clearance"
        for candidate in manager.orders.values()
    )
    assert waiter.trajectory == trajectory
    assert waiter.active_order_id == order.order_id

    state["stage"] = "planning"
    assert manager._finish_async_runtime_replan(job) == 0
    state["stage"] = "planning"
    assert manager._finish_async_runtime_replan(job) == 0

    clearance = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearance) == 1
    assert clearance[0].vehicle == blocker.name
    assert clearance[0].spatial_route_nodes == ["B", "P"]
    assert waiter.trajectory == trajectory
    assert waiter.active_order_id == order.order_id


def test_completed_retreat_holds_for_exact_parked_blocker_not_planner_bystander(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 1
    now = manager._now()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="RETREATING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_clock=0.0,
        route_revision=7,
        retreat_target_clock=0.0,
        retreat_target_lm="A",
        retreat_blocked_edges=[("A", "B"), ("B", "A")],
        retreat_blocker_signatures=[("causal", "B", 0)],
    )
    causal = FleetRobot(
        name="causal",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    bystander = FleetRobot(
        name="bystander",
        current_lm="C",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="EXECUTING",
        spatial_route_nodes=["A", "B", "C"],
    )
    manager.robots = {
        waiter.name: waiter,
        causal.name: causal,
        bystander.name: bystander,
    }
    manager.orders = {order.order_id: order}
    monkeypatch.setattr(
        manager,
        "_blocked_at_clock",
        lambda *_args, **_kwargs: "",
    )

    manager._advance_deadlock_retreat(waiter, now)

    state = manager._runtime_replans[waiter.name]
    assert state["causal_blocker_signatures"] == ((causal.name, "B", 0),)
    assert waiter.retreat_blocker_signatures == []

    state["stage"] = "planning"
    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        "planning_timeout:stationary_robot_blocks_route",
        debug={
            # Congestion planning may report another stationary body on a
            # rejected alternate suffix. It must not replace the robot that
            # physically caused this evacuation.
            "stationaryBlockerRobots": [bystander.name],
            "softBlockedLms": [bystander.current_lm],
        },
    )

    clearance = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearance) == 1
    assert clearance[0].vehicle == causal.name
    assert state["stage"] == "queued"
    assert state["clearance_blocker_names"] == (causal.name,)
    assert manager._runtime_replan_holds_robot(waiter)
    assert manager._ready_runtime_replan_entry(state["retry_at"] + 1.0) is None

    clearance[0].status = "COMPLETED"
    ready = manager._ready_runtime_replan_entry(state["retry_at"] + 1.0)
    assert ready is not None
    assert ready[1] is waiter


def test_completed_portal_tail_retreat_waits_for_captured_owner_occupancy() -> None:
    """The evacuated tail must not immediately drive back into the portal."""
    manager = _manager()
    now = manager._now()
    region = "corridor:captured-mouth"
    owner = FleetRobot(
        name="owner",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        active_order_id="owner-order",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        route_revision=4,
    )
    tail = FleetRobot(
        name="tail",
        current_lm="A",
        target_lm="B",
        status="RETREATING",
        active_order_id="tail-order",
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
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_clock=0.0,
        route_revision=7,
        retreat_target_clock=0.0,
        retreat_target_lm="A",
        retreat_blocked_edges=[("A", "B"), ("B", "A")],
    )
    manager.robots = {owner.name: owner, tail.name: tail}
    owner_order = FleetOrder(
        order_id="owner-order",
        target_lm="A",
        vehicle=owner.name,
        assigned_robot=owner.name,
        status="EXECUTING",
    )
    tail_order = FleetOrder(
        order_id="tail-order",
        target_lm="B",
        vehicle=tail.name,
        assigned_robot=tail.name,
        status="EXECUTING",
        spatial_route_nodes=["A", "B"],
    )
    manager.orders = {
        owner_order.order_id: owner_order,
        tail_order.order_id: tail_order,
    }
    manager._controlled_corridor_occupancy = {region: [owner.name]}
    captured_hold = manager._corridor_clearance_hold_for(owner, {region})
    assert captured_hold == {
        "owner": owner.name,
        "owner_order_id": owner_order.order_id,
        "regions": (region,),
        "physical_only": True,
    }
    tail.retreat_corridor_hold = captured_hold

    manager._advance_deadlock_retreat(tail, now)

    state = manager._runtime_replans[tail.name]
    assert state["corridor_clearance_hold"] == captured_hold
    assert tail.retreat_corridor_hold is None
    assert tail.active_order_id == tail_order.order_id
    assert tail.target_lm == "B"
    assert tail_order.target_lm == "B"
    assert manager._ready_runtime_replan_entry(now + 10.0) is None

    # The owner has physically exited the captured resource.  Its original
    # order may still be active; occupancy, not task completion, releases the
    # tail's same-goal transaction.
    manager._controlled_corridor_occupancy.clear()
    ready = manager._ready_runtime_replan_entry(now + 10.0)

    assert ready is not None
    assert ready[0] is tail_order
    assert ready[1] is tail
    assert ready[2] is state
    assert "corridor_clearance_hold" not in state
    assert tail.active_order_id == tail_order.order_id
    assert tail_order.target_lm == "B"


def test_portal_clearance_hold_releases_if_owner_still_waits_for_tail() -> None:
    manager = _manager()
    region = "corridor:shared-exit-arm"
    owner = FleetRobot(
        name="owner",
        current_lm="B",
        active_order_id="owner-order",
        status="WAITING",
        wait_for_robot="tail",
        last_reason="occupied by tail",
    )
    tail = FleetRobot(
        name="tail",
        current_lm="A",
        active_order_id="tail-order",
        status="WAITING",
    )
    manager.robots = {owner.name: owner, tail.name: tail}
    manager._controlled_corridor_occupancy = {region: [owner.name]}
    hold = {
        "owner": owner.name,
        "owner_order_id": owner.active_order_id,
        "regions": (region,),
        "physical_only": True,
    }

    assert not manager._corridor_clearance_hold_active(hold, tail.name)


def test_moved_causal_blocker_defers_to_fresh_planner_identity() -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 1
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": -2.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 2.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
        route_revision=7,
    )
    moved = FleetRobot(
        name="moved",
        current_lm="C",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
    )
    fresh = FleetRobot(
        name="fresh",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="PLANNING",
        spatial_route_nodes=["A", "B", "C"],
    )
    state = {
        "order_id": order.order_id,
        "start_lm": "A",
        "route_revision": waiter.route_revision,
        "route_clock": waiter.route_clock,
        "reason": "deadlock corridor evacuated; alternate route required",
        # The physical blocker was at B when captured but has since moved to C.
        "causal_blocker_signatures": ((moved.name, "B", 0),),
        "blocker_names": (moved.name,),
        "queued_at": manager._now(),
        "retry_at": manager._now(),
        "failures": 0,
        "generation": 1,
        "stage": "planning",
    }
    manager.robots = {
        waiter.name: waiter,
        moved.name: moved,
        fresh.name: fresh,
    }
    manager.orders = {order.order_id: order}
    manager._runtime_replans[waiter.name] = state

    manager._defer_runtime_replan(
        order,
        waiter,
        state,
        "stationary_robot_blocks_route",
        debug={
            "stationaryBlockerRobots": [fresh.name],
            "softBlockedLms": [fresh.current_lm],
        },
    )

    clearance = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearance) == 1
    assert clearance[0].vehicle == fresh.name
    assert "causal_blocker_signatures" not in state
    assert state["stage"] == "retry"


def test_clearance_target_rejects_static_obstacle_under_robot_footprint() -> None:
    manager = _clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        status="IDLE",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {
        "wait-order": FleetOrder(
            order_id="wait-order",
            target_lm="C",
            vehicle=waiter.name,
            assigned_robot=waiter.name,
            status="QUEUED",
            spatial_route_nodes=["A", "B", "C"],
        ),
    }
    manager.obstacles = [{"x": 0.0, "y": 2.0, "radius": 0.20}]

    route = manager._stationary_clearance_route(waiter, blocker)

    # C is the waiter's active destination, so after rejecting the obstructed
    # pocket the safe outcome is quarantine/wait—not moving the blocker onto
    # the same terminal resource one step later.
    assert route == []


def test_repeated_initial_plan_failure_queues_parked_clearance() -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 2
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        status="IDLE",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="QUEUED",
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    debug = {
        "softBlockedLms": ["B"],
        "stationaryBlockerRobots": [blocker.name],
    }

    manager._record_stationary_order_failure(order, debug)
    assert not any(
        candidate.internal_kind == "traffic_clearance"
        for candidate in manager.orders.values()
    )
    manager._record_stationary_order_failure(order, debug)
    assert any(
        candidate.internal_kind == "traffic_clearance"
        and candidate.vehicle == blocker.name
        for candidate in manager.orders.values()
    )


def test_exact_parked_blocker_retry_ignores_unrelated_fleet_churn() -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 2
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="C",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_revision=3,
    )
    unrelated = FleetRobot(
        name="unrelated",
        current_lm="P",
        status="ARRIVED",
        pose={"x": 0.0, "y": 2.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_OBSTACLE",
    )
    manager.robots = {
        waiter.name: waiter,
        blocker.name: blocker,
        unrelated.name: unrelated,
    }
    manager.orders = {order.order_id: order}
    debug = {
        "softBlockedLms": ["B", "P"],
        "stationaryBlockerRobots": [blocker.name],
    }

    manager._record_stationary_order_failure(order, debug)
    first_state = manager._stationary_order_retry_state[order.order_id]
    assert first_state["failure_count"] == 1
    assert first_state["blocked_lms"] == ("B",)

    # Another parked robot finishes elsewhere between identical failures.
    # It must neither reset the causal debounce nor be relocated.
    unrelated.current_lm = "C"
    unrelated.pose = {"x": 2.0, "y": 0.0, "yaw": 0.0}
    unrelated.route_revision += 1
    manager._record_stationary_order_failure(order, debug)

    second_state = manager._stationary_order_retry_state[order.order_id]
    assert second_state["failure_count"] == 2
    clearance = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearance) == 1
    assert clearance[0].vehicle == blocker.name
    assert clearance[0].spatial_route_nodes == ["B", "P"]


def test_stationary_failure_without_blocker_identity_never_moves_random_robot(
) -> None:
    manager = _clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 1
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        status="ARRIVED",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    unrelated = FleetRobot(
        name="unrelated",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="QUEUED",
    )
    manager.robots = {waiter.name: waiter, unrelated.name: unrelated}
    manager.orders = {order.order_id: order}

    manager._record_stationary_order_failure(
        order,
        {
            "stationaryRobotWait": True,
            "softBlockedLms": [unrelated.current_lm],
            # No continuousConflictRobot/stationaryBlockerRobots: the planner
            # has not established that this body caused the temporal failure.
        },
    )

    assert manager._stationary_order_retry_state[order.order_id][
        "blocker_names"
    ] == ()
    assert not any(
        candidate.internal_kind == "traffic_clearance"
        for candidate in manager.orders.values()
    )


def test_unrelated_temporal_failure_is_not_classified_as_stationary(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    moving = FleetRobot(
        name="moving",
        current_lm="A",
        status="IDLE",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    unrelated = FleetRobot(
        name="unrelated",
        current_lm="P",
        status="ARRIVED",
        pose={"x": 0.0, "y": 2.0, "yaw": 0.0},
    )
    manager.robots = {moving.name: moving, unrelated.name: unrelated}
    calls: list[dict] = []

    def temporal_failure(payload):
        calls.append(payload)
        return {
            "ok": False,
            "plans": [],
            "debug": {
                "reason": "no_low_level_path:moving:resource_constrained:B@7",
            },
        }

    monkeypatch.setattr(manager.planner, "plan", temporal_failure)
    request = {
        "name": moving.name,
        "startLm": "A",
        "goalLm": "C",
        "startPose": dict(moving.pose),
    }

    result = manager._plan_valid_requests(
        [request],
        {"robots": [request], "strictStationaryRobotAvoidance": True},
    )

    assert len(calls) == 2  # soft detour + no-soft causality diagnostic
    assert not result["ok"]
    assert result["debug"]["temporalResourceFailure"] is True
    assert "stationaryRobotWait" not in result["debug"]
    assert "stationaryBlockerRobots" not in result["debug"]
    assert "stationary_robot_blocks_route" not in result["debug"]["reason"]


def test_clearance_move_does_not_reverse_for_same_unchanged_waiter(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    clock = [100.0]
    monkeypatch.setattr(manager, "_now", lambda: clock[0])
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        status="WAITING",
        active_order_id="wait-order",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    waiter_order = FleetOrder(
        order_id="wait-order",
        target_lm="C",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {waiter_order.order_id: waiter_order}

    assert manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="first clearance",
    )
    state = manager._stationary_clearance_relocations[blocker.name]
    assert state["origin_lm"] == "B"
    assert state["target_lm"] == "P"
    clearance = manager.orders[state["order_id"]]
    clearance.status = "COMPLETED"
    blocker.current_lm = "P"
    blocker.pose = {"x": 0.0, "y": 2.0, "yaw": 0.0}

    clock[0] += 1.0
    manager._prune_stationary_clearance_relocations(clock[0])
    clock[0] += manager._stationary_clearance_cooldown() + 0.01

    assert not manager._queue_stationary_clearance_relocation(
        waiter,
        blocker,
        cause="unchanged failure",
    )
    assert not any(
        order.internal_kind == "traffic_clearance"
        for order in manager.orders.values()
    )
    retained = manager._stationary_clearance_relocations[blocker.name]
    assert set(retained["visited_lms"]) == {"B", "P"}


def test_quarantined_commanded_wait_chain_sink_is_still_dispatched(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        status="WAITING",
        last_reason="occupied by terminal",
        wait_for_robot="terminal",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": -2.0, "y": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
    )
    terminal = FleetRobot(
        name="terminal",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="terminal-order",
        target_lm="P",
        vehicle=terminal.name,
        assigned_robot=terminal.name,
        status="QUEUED",
        error="stationary retry",
        updated_at=0.0,
    )
    manager.robots = {waiter.name: waiter, terminal.name: terminal}
    manager.orders = {order.order_id: order}
    blocked_lms = ("B",)
    manager._stationary_order_retry_state[order.order_id] = {
        "blocked_lms": blocked_lms,
        "blocker_names": (),
        "signature": manager._stationary_blocker_signature(blocked_lms),
        "failure_count": manager._stationary_retry_failure_limit(),
    }
    assert not manager._stationary_order_retry_ready(order)
    captured: list[str] = []
    monkeypatch.setattr(
        manager,
        "_ready_simulated_order_entries",
        lambda orders: captured.extend(candidate.order_id for candidate in orders) or [],
    )

    manager._dispatch_orders()

    assert captured == [order.order_id]


def test_boxed_queued_departure_opens_vacancy_without_dropping_waiter_order(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    # The clearance selector below is intentionally stubbed, but its returned
    # routes must still be real graph routes now that the production escape
    # audit samples authored edge geometry and motion orientation.
    for source, target in (("C", "P"), ("A", "P")):
        start = manager.landmarks[source]
        goal = manager.landmarks[target]
        edge = GraphEdge(
            from_name=source,
            to_name=target,
            length=math.hypot(goal.x - start.x, goal.y - start.y),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(start.x, start.y),
                WorldPoint(goal.x, goal.y),
            ),
            properties={"direction": 2},
        )
        manager.planner.route_planner._edge_by_key[(source, target)] = edge
    now = 100.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    sink = FleetRobot(
        name="terminal",
        current_lm="B",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    east_trajectory = [
        {"t": 0.0, "x": 2.0, "y": 0.0, "yaw": math.pi, "edgeId": "C->B", "lm": "C"},
        {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": math.pi, "edgeId": "C->B", "lm": "B"},
    ]
    east = FleetRobot(
        name="east-waiter",
        current_lm="C",
        target_lm="B",
        status="WAITING",
        active_order_id="east-order",
        wait_for_robot=sink.name,
        last_reason=f"occupied by {sink.name}",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
        trajectory=list(east_trajectory),
        route_revision=7,
    )
    west = FleetRobot(
        name="west-waiter",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="west-order",
        wait_for_robot=sink.name,
        last_reason=f"occupied by {sink.name}",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": -2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B", "lm": "B"},
        ],
        route_revision=8,
    )
    sink_order = FleetOrder(
        order_id="terminal-order",
        target_lm="C",
        vehicle=sink.name,
        assigned_robot=sink.name,
        status="QUEUED",
        error="stationary_robot_blocks_route",
        dispatch_failures=73,
        updated_at=0.0,
        spatial_route_nodes=["B", "C"],
    )
    east_order = FleetOrder(
        order_id="east-order",
        target_lm="A",
        vehicle=east.name,
        assigned_robot=east.name,
        status="WAITING_TRAFFIC",
    )
    west_order = FleetOrder(
        order_id="west-order",
        target_lm="C",
        vehicle=west.name,
        assigned_robot=west.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {
        sink.name: sink,
        east.name: east,
        west.name: west,
    }
    manager.orders = {
        sink_order.order_id: sink_order,
        east_order.order_id: east_order,
        west_order.order_id: west_order,
    }
    blocked_lms = ("B",)
    manager._stationary_order_retry_state[sink_order.order_id] = {
        "blocked_lms": blocked_lms,
        "blocker_names": (),
        "signature": manager._stationary_blocker_signature(blocked_lms),
        "failure_count": manager._stationary_retry_failure_limit(),
    }
    rolling_signature = (("unrelated-rolling", "A", 99),)
    rolling_blacklist = {(rolling_signature, "unrelated-rolling", "P")}
    manager._rolling_vacancy_recovery_signature = rolling_signature
    manager._rolling_vacancy_recovery_blacklist = set(rolling_blacklist)
    monkeypatch.setattr(
        manager,
        "_stationary_clearance_route",
        lambda _sink, waiter, **_kwargs: (
            ["C", "P"] if waiter.name == east.name else ["A", "P"]
        ),
    )
    started = []
    monkeypatch.setattr(
        manager,
        "_start_async_runtime_replan",
        lambda entry: started.append(entry) or True,
    )

    manager._dispatch_orders(async_simulated=True)

    assert len(started) == 1
    _, selected, state = started[0]
    assert selected is east  # it occupies the queued sink's selected exit
    assert state["escape_route_nodes"] == ["C", "P"]
    assert state["escape_blocked_lms"] == ("B",)
    assert east.active_order_id == east_order.order_id
    assert east.trajectory == east_trajectory
    assert east_order.target_lm == "A"
    assert sink.status == "IDLE"
    assert sink_order.status == "QUEUED"
    assert manager._rolling_vacancy_recovery_signature == rolling_signature
    assert manager._rolling_vacancy_recovery_blacklist == rolling_blacklist

    manager.clear_robot_ephemeral_state(east.name)
    assert not manager._commanded_sink_vacancy_signatures
    assert not manager._commanded_sink_vacancy_blacklist
    assert manager._rolling_vacancy_recovery_signature == rolling_signature
    assert manager._rolling_vacancy_recovery_blacklist == rolling_blacklist


def test_queued_departure_vacancy_rejects_escape_toward_sink_body(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    now = 100.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    sink = FleetRobot(
        name="terminal",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    waiter = FleetRobot(
        name="waiter",
        current_lm="A",
        target_lm="B",
        status="WAITING",
        active_order_id="waiter-order",
        wait_for_robot=sink.name,
        last_reason=f"occupied by {sink.name}",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": -2.0, "y": 0.0, "yaw": 0.0, "lm": "A"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "lm": "B"},
        ],
        route_revision=7,
    )
    sink_order = FleetOrder(
        order_id="terminal-order",
        target_lm="P",
        vehicle=sink.name,
        assigned_robot=sink.name,
        status="QUEUED",
        error="no_low_level_path",
        dispatch_failures=2,
        updated_at=0.0,
        spatial_route_nodes=["B", "P"],
    )
    waiter_order = FleetOrder(
        order_id="waiter-order",
        target_lm="A",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="WAITING_TRAFFIC",
    )
    manager.robots = {sink.name: sink, waiter.name: waiter}
    manager.orders = {
        sink_order.order_id: sink_order,
        waiter_order.order_id: waiter_order,
    }
    route_calls: list[tuple[set[str], set[tuple[str, str]]]] = []

    def clearance_route(_sink, _waiter, **kwargs):
        forbidden = set(kwargs.get("forbidden_lms", set()))
        blocked_edges = set(kwargs.get("extra_blocked_edges", set()))
        route_calls.append((forbidden, blocked_edges))
        return ["A", "P"] if "B" in forbidden else ["A", "B"]

    monkeypatch.setattr(manager, "_stationary_clearance_route", clearance_route)
    monkeypatch.setattr(
        manager,
        "_graph_escape_route_current_body_blocker",
        lambda _robot, route, **_kwargs: (
            sink.name if list(route)[-1] == "B" else ""
        ),
    )
    assert manager._queue_commanded_sink_vacancy_replan(now)

    assert len(route_calls) == 2
    assert ("A", "B") in route_calls[0][1]
    assert "B" in route_calls[1][0]
    state = manager._runtime_replans[waiter.name]
    assert state["escape_route_nodes"] == ["A", "P"]
    assert any(
        owner == waiter.name and pocket == "B"
        for _, _, owner, pocket in manager._commanded_sink_vacancy_blacklist
    )


def test_commanded_sink_vacancy_episodes_are_scoped_per_sink(
    monkeypatch,
) -> None:
    manager = _clearance_manager()
    now = 100.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    first_sink = FleetRobot(
        name="sink-1",
        current_lm="A",
        status="ARRIVED",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    second_sink = FleetRobot(
        name="sink-2",
        current_lm="C",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": math.pi},
    )
    first_waiter = FleetRobot(
        name="waiter-1",
        current_lm="B",
        status="WAITING",
        active_order_id="waiter-order-1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        route_revision=11,
    )
    second_waiter = FleetRobot(
        name="waiter-2",
        current_lm="P",
        status="WAITING",
        active_order_id="waiter-order-2",
        pose={"x": 0.0, "y": 2.0, "yaw": -math.pi / 2.0},
        route_revision=12,
    )
    orders = [
        FleetOrder(
            order_id="sink-order-1",
            target_lm="C",
            vehicle=first_sink.name,
            assigned_robot=first_sink.name,
            status="QUEUED",
            error="stationary conflict",
            dispatch_failures=2,
            spatial_route_nodes=["A", "B", "C"],
        ),
        FleetOrder(
            order_id="sink-order-2",
            target_lm="A",
            vehicle=second_sink.name,
            assigned_robot=second_sink.name,
            status="QUEUED",
            error="stationary conflict",
            dispatch_failures=2,
            spatial_route_nodes=["C", "B", "A"],
        ),
        FleetOrder(
            order_id=first_waiter.active_order_id,
            target_lm="C",
            vehicle=first_waiter.name,
            assigned_robot=first_waiter.name,
            status="WAITING_TRAFFIC",
        ),
        FleetOrder(
            order_id=second_waiter.active_order_id,
            target_lm="A",
            vehicle=second_waiter.name,
            assigned_robot=second_waiter.name,
            status="WAITING_TRAFFIC",
        ),
    ]
    manager.robots = {
        robot.name: robot
        for robot in (
            first_sink,
            second_sink,
            first_waiter,
            second_waiter,
        )
    }
    manager.orders = {order.order_id: order for order in orders}
    waiters = {
        first_sink.name: [first_waiter],
        second_sink.name: [second_waiter],
    }
    monkeypatch.setattr(
        manager,
        "_live_waiters_for_stationary_sink",
        lambda sink: waiters.get(sink.name, []),
    )
    monkeypatch.setattr(
        manager,
        "_stationary_clearance_route",
        lambda _sink, waiter, **_kwargs: (
            ["B", "P"] if waiter is first_waiter else ["P", "B"]
        ),
    )
    monkeypatch.setattr(
        manager,
        "_graph_escape_route_current_body_blocker",
        lambda *_args, **_kwargs: "",
    )

    assert manager._queue_commanded_sink_vacancy_replan(now)
    assert set(manager._commanded_sink_vacancy_signatures) == {
        first_sink.name,
        second_sink.name,
    }
    first_signature = manager._commanded_sink_vacancy_signatures[first_sink.name]
    second_signature = manager._commanded_sink_vacancy_signatures[second_sink.name]
    manager._commanded_sink_vacancy_blacklist.update({
        (first_sink.name, first_signature, first_waiter.name, "P"),
        (second_sink.name, second_signature, second_waiter.name, "B"),
    })

    # Scanning the second live sink must not clear the first sink's episode.
    manager._queue_commanded_sink_vacancy_replan(now + 0.1)
    assert len(manager._commanded_sink_vacancy_blacklist) == 2

    manager.clear_robot_ephemeral_state(first_waiter.name)

    assert set(manager._commanded_sink_vacancy_signatures) == {
        second_sink.name,
    }
    assert manager._commanded_sink_vacancy_blacklist == {
        (second_sink.name, second_signature, second_waiter.name, "B"),
    }


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
        self.execute_calls: list[dict[str, object]] = []
        self.cancel_calls: list[dict[str, object]] = []
        self.cancel_error: Exception | None = None

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

    def execute_route(self, endpoint: str, payload: dict) -> dict[str, object]:
        self.execute_calls.append({"endpoint": endpoint, "payload": payload})
        return {"ok": True}

    def cancel_route(self, endpoint: str, **kwargs) -> dict[str, object]:
        self.cancel_calls.append({"endpoint": endpoint, **kwargs})
        if self.cancel_error is not None:
            raise self.cancel_error
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


def _clearance_manager() -> FleetManagerSim:
    landmarks = {
        "A": Landmark(name="A", x=-2.0, y=0.0),
        "B": Landmark(name="B", x=0.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "P": Landmark(name="P", x=0.0, y=2.0),
    }
    edges = []
    for source, target in (("A", "B"), ("B", "C"), ("B", "P")):
        start = landmarks[source]
        goal = landmarks[target]
        edges.append(GraphEdge(
            from_name=source,
            to_name=target,
            length=math.hypot(goal.x - start.x, goal.y - start.y),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(start.x, start.y),
                WorldPoint(goal.x, goal.y),
            ),
            # Traffic is directed source->target; motion orientation is free.
            properties={"direction": 2},
        ))
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "navigation": {"simulate_rotation": True},
            "fleet": {
                "parked_clearance_relocation_enabled": True,
                "parked_clearance_relocation_max_hops": 4,
            },
        },
    )


def _causal_clearance_manager() -> FleetManagerSim:
    """Graph whose only blocker pocket path crosses the waiter's LM."""
    landmarks = {
        "P": Landmark(name="P", x=-2.0, y=0.0),
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
        "G": Landmark(name="G", x=4.0, y=0.0),
    }
    edges = []
    for source, target in (
        ("A", "B"),
        ("B", "G"),
        ("B", "A"),
        ("A", "P"),
    ):
        start = landmarks[source]
        goal = landmarks[target]
        edges.append(GraphEdge(
            from_name=source,
            to_name=target,
            length=math.hypot(goal.x - start.x, goal.y - start.y),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(start.x, start.y),
                WorldPoint(goal.x, goal.y),
            ),
            properties={"direction": 2},
        ))
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "parked_clearance_relocation_enabled": True,
                "parked_clearance_relocation_max_hops": 4,
            },
        },
    )


def _stationary_cut_manager(*, staged: bool) -> FleetManagerSim:
    """Graphs for alternate-cut and two-stage parked-body recovery."""
    if staged:
        coordinates = {
            "P": (-3.0, 0.0),
            "A": (0.0, 0.0),
            "B": (3.0, 0.0),
            "G": (6.0, 0.0),
            "Q": (0.0, 3.0),
        }
        pairs = (
            ("P", "A"),
            ("A", "B"),
            ("B", "G"),
            ("A", "Q"),
        )
    else:
        coordinates = {
            "P": (-3.0, 0.0),
            "A": (0.0, 0.0),
            "B": (3.0, 0.0),
            "G": (6.0, 0.0),
            "C": (0.0, 3.0),
            "D": (3.0, 3.0),
            "Q": (3.0, 6.0),
        }
        pairs = (
            ("A", "B"),
            ("B", "G"),
            ("A", "C"),
            ("C", "D"),
            ("D", "G"),
            ("D", "Q"),
            ("A", "P"),
        )
    landmarks = {
        name: Landmark(name=name, x=x, y=y)
        for name, (x, y) in coordinates.items()
    }
    edges: list[GraphEdge] = []
    for first, second in pairs:
        for source, target in ((first, second), (second, first)):
            start = landmarks[source]
            goal = landmarks[target]
            edges.append(GraphEdge(
                from_name=source,
                to_name=target,
                length=math.hypot(goal.x - start.x, goal.y - start.y),
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(start.x, start.y),
                    WorldPoint(goal.x, goal.y),
                ),
                properties={"direction": 2},
            ))
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "parked_clearance_relocation_enabled": True,
                "parked_clearance_relocation_failures": 1,
                "parked_clearance_relocation_max_hops": 6,
                "parked_clearance_waiter_escape_attempts": 3,
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


def _remote_clearance_manager() -> FleetManagerROS:
    simulation = _clearance_manager()
    return FleetManagerROS(
        simulation.landmarks,
        simulation.edges,
        params=simulation.params,
        remote_adapter=_RemoteControlAdapter(),
    )


def test_remote_clearance_dispatch_preserves_its_explicit_spatial_route(
    monkeypatch,
) -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=1.0),
        "C": Landmark(name="C", x=1.0, y=-1.0),
        "D": Landmark(name="D", x=2.0, y=0.0),
    }
    edges = []
    for source, target in (("A", "B"), ("B", "D"), ("A", "C"), ("C", "D")):
        start = landmarks[source]
        goal = landmarks[target]
        edges.append(GraphEdge(
            from_name=source,
            to_name=target,
            length=math.hypot(goal.x - start.x, goal.y - start.y),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(start.x, start.y),
                WorldPoint(goal.x, goal.y),
            ),
            properties={"direction": 2},
        ))
    adapter = _RemoteControlAdapter()
    manager = FleetManagerROS(
        landmarks,
        edges,
        params={"planner": {"on_route_tolerance": 0.1}},
        remote_adapter=adapter,
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="IDLE",
        mode="grpc",
        base_url="grpc://robot1:50051",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="traffic-clearance-r1",
        target_lm="D",
        vehicle=robot.name,
        status="QUEUED",
        spatial_route_nodes=["A", "C", "D"],
        internal_kind="traffic_clearance",
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}

    def plan_from_requested_route(requests, payload):
        del payload
        request = requests[0]
        nodes = list(request.get("routeNodes") or ["A", "B", "D"])
        trajectory = [
            {
                "t": float(index),
                "x": landmarks[node].x,
                "y": landmarks[node].y,
                "yaw": 0.0,
                "lm": node,
                "edgeId": (
                    f"{nodes[index - 1]}->{node}" if index else "A->A"
                ),
            }
            for index, node in enumerate(nodes)
        ]
        return {
            "ok": True,
            "plans": [{
                "robot": robot.name,
                "startLm": "A",
                "goalLm": "D",
                "nodes": nodes,
                "trajectory": trajectory,
            }],
            "debug": {},
        }

    monkeypatch.setattr(manager, "_plan_valid_requests", plan_from_requested_route)

    assert manager._dispatch_order(order)
    assert len(adapter.execute_calls) == 1
    route = adapter.execute_calls[0]["payload"]["route"]
    assert route["nodes"] == ["A", "C", "D"]
    assert order.status == "EXECUTING"


def test_failed_remote_clearance_cancel_preserves_route_until_retry() -> None:
    manager = _remote_clearance_manager()
    adapter = manager.remote_adapter
    manager.params["fleet"]["parked_clearance_relocation_timeout_sec"] = 20.0
    now = manager._now()
    remote = FleetRobot(
        name="r1",
        current_lm="B",
        target_lm="P",
        status="WAITING",
        mode="grpc",
        base_url="grpc://robot1:50051",
        active_order_id="traffic-clearance-r1",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
            {"t": 1.0, "x": 0.0, "y": 0.0, "edgeId": "B->B", "lm": "B"},
        ],
        plan_nodes=["B", "P"],
        route_revision=17,
    )
    clearance = FleetOrder(
        order_id=remote.active_order_id,
        target_lm="P",
        vehicle=remote.name,
        assigned_robot=remote.name,
        status="WAITING_TRAFFIC",
        internal_kind="traffic_clearance",
    )
    state = {
        "order_id": clearance.order_id,
        "waiter_signature": (),
        "queued_at": now - 21.0,
        "cooldown_sec": manager._stationary_clearance_cooldown(),
    }
    manager.robots = {remote.name: remote}
    manager.orders = {clearance.order_id: clearance}
    manager._stationary_clearance_relocations = {remote.name: state}
    route_before = (
        remote.active_order_id,
        remote.target_lm,
        remote.status,
        list(remote.trajectory),
        list(remote.plan_nodes),
        remote.route_revision,
    )
    adapter.cancel_error = RuntimeError("cancel unavailable")

    manager._prune_stationary_clearance_relocations(now)

    assert clearance.status == "WAITING_TRAFFIC"
    assert clearance.order_id in manager.orders
    assert state["order_id"] == clearance.order_id
    assert (
        remote.active_order_id,
        remote.target_lm,
        remote.status,
        remote.trajectory,
        remote.plan_nodes,
        remote.route_revision,
    ) == route_before

    adapter.cancel_error = None
    manager._prune_stationary_clearance_relocations(now + 1.0)

    assert len(adapter.cancel_calls) == 2
    assert clearance.order_id not in manager.orders
    assert state["order_id"] == ""
    assert remote.active_order_id == ""
    assert remote.status == "IDLE"


def test_remote_stationary_plan_failure_queues_common_clearance_recovery(
    monkeypatch,
) -> None:
    manager = _remote_clearance_manager()
    manager.params["fleet"]["parked_clearance_relocation_failures"] = 1
    waiter = FleetRobot(
        name="r1",
        current_lm="A",
        status="IDLE",
        mode="grpc",
        base_url="grpc://robot1:50051",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="remote-order",
        target_lm="C",
        vehicle=waiter.name,
        status="QUEUED",
    )
    manager.robots = {waiter.name: waiter, blocker.name: blocker}
    manager.orders = {order.order_id: order}
    monkeypatch.setattr(
        manager,
        "_plan_valid_requests",
        lambda requests, payload: {
            "ok": False,
            "plans": [],
            "debug": {
                "reason": "no_low_level_path:stationary_robot_blocks_route",
                "stationaryRobotWait": True,
                "stationaryBlockerRobots": [blocker.name],
                "softBlockedLms": [blocker.current_lm],
            },
        },
    )

    assert not manager._dispatch_order(order)
    clearances = [
        candidate
        for candidate in manager.orders.values()
        if candidate.internal_kind == "traffic_clearance"
    ]
    assert len(clearances) == 1
    assert clearances[0].vehicle == blocker.name
    assert clearances[0].spatial_route_nodes == ["B", "P"]


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
