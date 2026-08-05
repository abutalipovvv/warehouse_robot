from __future__ import annotations

import math

import pytest

from fleet_manager.core.mapf.fleet.fleet_planner import FleetMapfPlanner
from fleet_manager.core.mapf.fleet.fleet_planner_backends import BackendSelector
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint


def test_backend_selector_normalizes_config_and_request_aliases() -> None:
    selector = BackendSelector()

    assert selector.from_fleet_params({"planner_backend": "sipp"}) == "rolling_sipp"
    assert selector.from_fleet_params({"plannerBackend": "SIPP+CBS"}) == "hybrid"
    assert selector.from_fleet_params({"mapf_backend": "unknown"}) == "cbs"
    assert (
        selector.from_payload(
            {"planner_backend": "rolling-sipp"},
            default="cbs",
        )
        == "rolling_sipp"
    )
    assert selector.from_payload({}, default="hybrid") == "hybrid"


def test_request_preparer_returns_typed_normalized_request() -> None:
    planner = _planner(
        params={
            "navigation": {
                "route_speed": 0.8,
                "route_acceleration": 0.4,
            },
            "fleet": {
                "planner_backend": "cbs",
                "reservation_time_step_sec": 0.5,
                "cbs_low_level_max_time": 20,
                "reserved_edge_detour_enabled": True,
                "reservation_detour_horizon_sec": 10.0,
            },
        }
    )
    prepared = planner._request_preparer.prepare(
        {
            "plannerBackend": "sipp",
            "speed": 0.6,
            "rotate": "true",
            "turnSpeed": 1.2,
            "lowLevelMaxTime": 15,
            "blockedEdges": ["C->B"],
            "reservedVertexConstraints": [{"time": 3, "node": "B"}],
            "reservedEdgeConstraints": [[4, "A", "B"]],
            "reservedVertexIntervals": [
                {"node": "B", "start": 1.0, "end": 2.1, "robot": "other"}
            ],
            "reservedEdgeIntervals": [
                {
                    "from": "A",
                    "to": "B",
                    "start": 0.5,
                    "end": 2.2,
                    "robot": "other",
                }
            ],
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "C",
                    "startPose": {"x": 0.0, "y": 0.0, "yaw": 0.25},
                    "routeNodes": ["A", "B", "C"],
                    "startNotBeforeSec": 1.1,
                    "departureNotBefore": {"B": 2.0},
                    "noWaitNodes": ["B"],
                }
            ],
        }
    )

    request = prepared.requests[0]
    assert prepared.selected_backend == "rolling_sipp"
    assert prepared.speed == pytest.approx(0.6)
    assert prepared.acceleration == pytest.approx(0.4)
    assert prepared.rotate_enabled
    assert prepared.turn_speed == pytest.approx(1.2)
    assert prepared.low_level_max_time == 15
    assert not prepared.allow_cbs_fallback
    assert request.route_nodes == ("A", "B", "C")
    assert request.start_not_before_tick == 3
    assert request.node_departure_not_before == (("B", 4),)
    assert request.no_wait_nodes == ("B",)
    assert prepared.reserved_vertex_constraints == [(3, "B")]
    assert prepared.reserved_edge_constraints == [(4, "A", "B")]
    assert prepared.reserved_vertex_intervals == [(2, 5, "B", "other")]
    assert prepared.reserved_edge_intervals == [(1, 5, "A", "B", "other")]
    assert prepared.reserved_interval_edges == {("A", "B"), ("B", "A")}
    assert prepared.detour_blocked_edges == {
        ("A", "B"),
        ("B", "A"),
        ("C", "B"),
    }


def test_motion_model_and_trajectory_builder_share_timing_math() -> None:
    planner = _planner(
        params={
            "navigation": {"route_speed": 1.0},
            "fleet": {"reservation_time_step_sec": 0.5},
        }
    )

    assert planner._motion_model.edge_tick_cost("A", "B", 0.5) == 4
    assert planner._motion_model.travel_time(1.0, 1.0, 1.0) == pytest.approx(2.0)
    assert planner._motion_model.rotation_duration(
        0.0,
        math.pi / 2.0,
        1.0,
    ) == pytest.approx(math.pi / 2.0)

    trajectory = planner._trajectory_builder.build(
        ["A", "A", "B"],
        1.0,
        [0, 2, 4],
        start_yaw=0.0,
        yaws=[0.0, math.pi / 2.0, 0.0],
        actions=["start", "rotate", "move"],
    )

    assert trajectory[1]["edgeId"] == "WAIT@ROTATE:A"
    assert trajectory[1]["motionDirection"] == "rotate"
    assert trajectory[1]["t"] == pytest.approx(1.0)
    assert trajectory[-1]["lm"] == "B"
    assert trajectory[-1]["t"] == pytest.approx(2.0)


def test_route_rotation_check_keeps_safe_reverse_option() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
        properties={"direction": 2},
    )
    planner = FleetMapfPlanner(landmarks, [edge])
    planner.set_rotation_validator(
        lambda _node, _from_yaw, _to_yaw: False
    )

    assert planner.route_rotations_are_allowed(
        ["A", "B"],
        math.pi,
        rotate_enabled=True,
    )
    assert planner.turn_safe_reachable_nodes(
        "A",
        math.pi,
        rotate_enabled=True,
    ) == {"A", "B"}
    assert not planner.route_rotations_are_allowed(
        ["A", "B"],
        math.pi / 2.0,
        rotate_enabled=True,
    )
    assert planner.turn_safe_reachable_nodes(
        "A",
        math.pi / 2.0,
        rotate_enabled=True,
    ) == {"A"}


def test_result_formatter_builds_stable_timed_segment_contract() -> None:
    planner = _planner(
        params={"fleet": {"reservation_time_step_sec": 0.5}}
    )

    segments = planner._result_formatter.timed_segments(
        ["A", "A", "B"],
        [0, 2, 5],
        ["start", "rotate", "move"],
    )

    assert segments == [
        {
            "startTick": 0,
            "endTick": 2,
            "notBeforeSec": 0.0,
            "plannedArrivalSec": 1.0,
            "kind": "rotate",
            "node": "A",
        },
        {
            "startTick": 2,
            "endTick": 5,
            "notBeforeSec": 1.0,
            "plannedArrivalSec": 2.5,
            "kind": "move",
            "from": "A",
            "to": "B",
            "motionDirection": "forward",
        },
    ]


def _planner(*, params: dict[str, object] | None = None) -> FleetMapfPlanner:
    landmarks = {
        name: Landmark(name=name, x=float(index), y=0.0)
        for index, name in enumerate(("A", "B", "C"))
    }
    edges = [
        _edge(landmarks, source, target)
        for source, target in (
            ("A", "B"),
            ("B", "A"),
            ("B", "C"),
            ("C", "B"),
        )
    ]
    return FleetMapfPlanner(landmarks, edges, params=params)


def _edge(
    landmarks: dict[str, Landmark],
    source: str,
    target: str,
) -> GraphEdge:
    return GraphEdge(
        from_name=source,
        to_name=target,
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(
            WorldPoint(landmarks[source].x, landmarks[source].y),
            WorldPoint(landmarks[target].x, landmarks[target].y),
        ),
        properties={"direction": 0},
    )
