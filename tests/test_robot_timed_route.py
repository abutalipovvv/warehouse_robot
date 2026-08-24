from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROBOT_PLANNER_SRC = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "robot_driver"
    / "src"
    / "robot_planner"
)
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))

from robot_planner.execution import (
    RouteControlParameters,
    RouteExecutor,
    RouteProgress,
    RouteSteeringState,
)
from robot_planner.planning import RobotTrajectoryPlanner
from robot_planner.runtime import RoutePoint


def test_robot_route_points_preserve_not_before_timestamp() -> None:
    point = RoutePoint(1.0, 2.0, 0.3, "A->B", not_before=123.5)

    restored = RoutePoint.from_dict(point.to_dict())

    assert restored.not_before == 123.5


def test_timed_segments_are_mapped_only_to_their_matching_edges() -> None:
    planner = object.__new__(RobotTrajectoryPlanner)
    trajectory = [
        RoutePoint(0.0, 0.0, 0.0, "A->B"),
        RoutePoint(1.0, 0.0, 0.0, "A->B"),
        RoutePoint(1.0, 0.0, 0.0, "B->C"),
        RoutePoint(2.0, 0.0, 0.0, "B->C"),
    ]

    planner._apply_timed_segments(
        trajectory,
        {
            "dispatchEpochSec": 100.0,
            "timedSegments": [
                {"kind": "move", "from": "A", "to": "B", "notBeforeSec": 2.0},
                {"kind": "move", "from": "B", "to": "C", "notBeforeSec": 7.0},
            ],
        },
    )

    assert [point.not_before for point in trajectory] == [102.0, 102.0, 107.0, 107.0]


def test_route_control_parameters_are_cached_until_hot_reload_replaces_payload() -> None:
    class Planner:
        def __init__(self) -> None:
            self.params = {"navigation": {"route_speed": 0.4}}

        def current_params(self) -> dict[str, object]:
            return self.params

    planner = Planner()
    executor = RouteExecutor(object(), planner, lambda *_: None)

    first = executor._route_control_parameters()
    again = executor._route_control_parameters()
    planner.params = {"navigation": {"route_speed": 0.9}}
    reloaded = executor._route_control_parameters()

    assert first is again
    assert reloaded is not first
    assert first.route_speed == pytest.approx(0.4)
    assert reloaded.route_speed == pytest.approx(0.9)


def test_backward_hard_rejoin_steers_by_path_heading() -> None:
    control = RouteControlParameters.from_payload({
        "navigation": {
            "route_speed": 0.35,
            "angular_gain": 2.0,
            "max_angular_speed": 1.0,
            "rotate_in_place_angle_deg": 20.0,
        },
    })
    progress = RouteProgress(
        distance_to_goal=2.0,
        final_yaw_error=0.0,
        remaining_distance=2.0,
    )
    steering = RouteSteeringState(
        curvature_hint=0.0,
        drive_sign=-1.0,
        path_heading_error=0.2,
        target_heading_error=0.8,
        cross_track_error=0.5,
        steering_error=0.6,
        off_route=True,
        hard_rejoin=True,
    )
    route = type("Route", (), {"goal_lm": "G"})()

    linear, angular, message = RouteExecutor._route_drive_command(
        route,
        progress,
        steering,
        control,
    )

    assert linear == 0.0
    assert angular == pytest.approx(0.4)
    assert message == "Returning to route toward G."
