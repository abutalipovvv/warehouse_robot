from __future__ import annotations

from pathlib import Path
import sys


ROBOT_PLANNER_SRC = (
    Path(__file__).resolve().parents[1]
    / "sim_robot"
    / "ws"
    / "src"
    / "robot_planner"
)
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))

from robot_planner.route_planner import RobotTrajectoryPlanner
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
