from __future__ import annotations

import math

import pytest

from fleet_manager.core.fleet.domain.models import FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.simulation.manager import FleetManagerSim


FOOTPRINT = [
    {"x": -0.60, "y": -0.05},
    {"x": 0.60, "y": -0.05},
    {"x": 0.60, "y": 0.05},
    {"x": -0.60, "y": 0.05},
]


def _manager(
    landmark_points: dict[str, tuple[float, float]],
    edge_specs: list[tuple[str, str, str]],
) -> FleetManagerSim:
    landmarks = {
        name: Landmark(name=name, x=x, y=y)
        for name, (x, y) in landmark_points.items()
    }
    edges = []
    for source, target, motion_direction in edge_specs:
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
            properties={"direction": motion_direction},
        ))
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "robot_model": {"footprint": FOOTPRINT},
            "navigation": {
                "simulate_rotation": True,
                "collision_margin": 0.001,
            },
            "fleet": {"robot_clearance_m": 0.35},
        },
    )


def _polar_pose(radius: float, angle: float, yaw: float) -> dict[str, float]:
    return {
        "x": radius * math.cos(angle),
        "y": radius * math.sin(angle),
        "yaw": yaw,
    }


def test_escape_rejects_initial_turn_collision_before_moving_away() -> None:
    manager = _manager(
        {"A": (0.0, 0.0), "B": (-2.0, 0.0)},
        [("A", "B", "forward")],
    )
    mover = FleetRobot(
        name="mover",
        current_lm="A",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": -math.pi / 6.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="P",
        status="ARRIVED",
        pose=_polar_pose(1.17, math.pi / 6.0, math.pi / 6.0),
    )
    manager.robots = {mover.name: mover, blocker.name: blocker}

    # Both endpoint orientations are physically clear and every centre-line
    # sample travels farther from the blocker.  The shortest initial turn,
    # however, sweeps the long side of the body through it.
    assert not manager.collision.footprints_overlap(mover.pose, blocker.pose)
    assert not manager.collision.footprints_overlap(
        {"x": 0.0, "y": 0.0, "yaw": math.pi},
        blocker.pose,
    )
    assert manager.collision.footprints_overlap(
        {"x": 0.0, "y": 0.0, "yaw": -5.0 * math.pi / 6.0},
        blocker.pose,
    )

    assert manager._graph_escape_route_current_body_blocker(
        mover,
        ["A", "B"],
    ) == blocker.name


@pytest.mark.parametrize("motion_direction", ["backward", "not_specified"])
def test_escape_keeps_safe_move_away_for_reverse_body_orientation(
    motion_direction: str,
) -> None:
    manager = _manager(
        {"A": (0.0, 0.0), "B": (-2.0, 0.0)},
        [("A", "B", motion_direction)],
    )
    mover = FleetRobot(
        name="mover",
        current_lm="A",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="P",
        status="ARRIVED",
        pose=_polar_pose(1.17, math.pi / 6.0, math.pi / 6.0),
    )
    manager.robots = {mover.name: mover, blocker.name: blocker}

    assert not manager.collision.footprints_overlap(mover.pose, blocker.pose)
    assert manager.collision.robot_footprints_conflict(mover.pose, blocker.pose)
    # Backward is authored explicitly in the first case.  In the second case
    # SIPP may choose the same zero-turn reverse orientation for an
    # unspecified edge.  Translation then monotonically opens the soft gap.
    assert manager._graph_escape_route_current_body_blocker(
        mover,
        ["A", "B"],
    ) == ""


def test_escape_rejects_collision_during_intermediate_lm_turn() -> None:
    manager = _manager(
        {"A": (-2.0, 0.0), "B": (0.0, 0.0), "C": (0.0, 2.0)},
        [
            ("A", "B", "forward"),
            ("B", "C", "forward"),
        ],
    )
    mover = FleetRobot(
        name="mover",
        current_lm="A",
        status="WAITING",
        pose={"x": -2.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="blocker",
        current_lm="P",
        status="ARRIVED",
        pose={
            "x": 1.17 / math.sqrt(2.0),
            "y": 1.17 / math.sqrt(2.0),
            "yaw": math.pi / 4.0,
        },
    )
    manager.robots = {mover.name: mover, blocker.name: blocker}

    for endpoint_yaw in (0.0, math.pi / 2.0):
        assert not manager.collision.footprints_overlap(
            {"x": 0.0, "y": 0.0, "yaw": endpoint_yaw},
            blocker.pose,
        )
    assert manager.collision.footprints_overlap(
        {"x": 0.0, "y": 0.0, "yaw": math.pi / 4.0},
        blocker.pose,
    )

    assert manager._graph_escape_route_current_body_blocker(
        mover,
        ["A", "B", "C"],
    ) == blocker.name


def test_rolling_failure_extracts_exact_unsafe_turn_lm() -> None:
    manager = _manager(
        {"A": (-2.0, 0.0), "B": (0.0, 0.0), "C": (0.0, 2.0)},
        [
            ("A", "B", "forward"),
            ("B", "C", "forward"),
        ],
    )
    debug = {
        "stationaryRobotWait": True,
        "stationaryTurnEnvelopeBlock": True,
        "continuousConflictEdge": "WAIT@ROTATE:B",
        "continuousUnresolvedConflicts": [
            {
                "robot": "mover",
                "other": "blocker",
                "edge": "WAIT@ROTATE:B",
            }
        ],
    }

    assert manager._stationary_failure_applies_to_robot(debug, "mover")
    assert not manager._stationary_failure_applies_to_robot(debug, "other")
    assert manager._stationary_turn_conflict_lm(debug, "mover") == "B"
