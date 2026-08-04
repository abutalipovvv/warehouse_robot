from __future__ import annotations

from fleet_manager.manager.planning import RobotPlanningState
from fleet_manager.robot.model import FleetRobot


def test_robot_defaults_are_safe_for_an_idle_local_robot() -> None:
    robot = FleetRobot(name="r1", current_lm="A")

    assert robot.status == "IDLE"
    assert robot.trajectory == []
    assert robot.plan_nodes == []
    assert robot.pending_route is None
    assert robot.remote_online is True
    assert robot.is_remote() is False


def test_robot_route_and_traffic_state_are_reported_to_clients() -> None:
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        target_lm="B",
        route_final_lm="C",
        route_revision=4,
        route_clock=1.5,
        wait_for_robot="r2",
        wait_resource="A->B",
        wait_release_at=3.0,
        trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0}],
    )

    payload = robot.to_dict()

    assert payload["targetLm"] == "C"
    assert payload["routeRevision"] == 4
    assert payload["routeClock"] == 1.5
    assert payload["waitDependency"] == {
        "robot": "r2",
        "resource": "A->B",
        "releaseAt": 3.0,
    }


def test_remote_connection_mode_is_explicit() -> None:
    local = FleetRobot(name="local", current_lm="A", mode="simulated")
    remote = FleetRobot(name="remote", current_lm="A", mode="grpc")

    assert not local.is_remote()
    assert remote.is_remote()


def test_robot_planning_snapshot_does_not_share_mutable_pose() -> None:
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        pose={"x": 1.0, "y": 2.0, "yaw": 0.0},
    )

    planning_state = RobotPlanningState.from_robot(robot)
    robot.pose["x"] = 99.0

    assert planning_state.pose.get("x") == 1.0
