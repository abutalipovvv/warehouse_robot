from __future__ import annotations

import math
import pytest

from fleet_manager.mapf import (
    FleetMapfPlanner,
    LmRobotRequest,
    ReservationInterval,
    ReservationTable,
    ResourceId,
    RollingSippPlanner,
    TrafficGraph,
)
from fleet_manager.route_core import GraphEdge, Landmark, WorldPoint


def test_traffic_graph_groups_reverse_lanes_as_same_resource() -> None:
    landmarks = _landmarks("A", "B")
    graph = TrafficGraph.from_route_core(
        landmarks,
        [
            _edge(landmarks, "A", "B"),
            _edge(landmarks, "B", "A"),
        ],
        default_speed_mps=1.0,
    )

    forward = graph.lane_for("A", "B")
    reverse = graph.lane_for("B", "A")

    assert forward is not None
    assert reverse is not None
    assert forward.lane_group_id == reverse.lane_group_id


def test_traffic_graph_reserves_endpoint_and_physical_clearance_resources() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=0.5, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
    }
    graph = TrafficGraph.from_route_core(
        landmarks,
        [_edge(landmarks, "A", "B")],
        default_speed_mps=1.0,
        min_robot_center_distance_m=0.83,
    )

    a_resources = set(graph.vertex_resources("A"))
    b_resources = set(graph.vertex_resources("B"))
    shared_clearance = {
        resource
        for resource in a_resources & b_resources
        if resource.kind == "clearance"
    }
    lane = graph.lane_for("A", "B")

    assert shared_clearance
    assert lane is not None
    assert ResourceId("vertex", "A") in graph.lane_resources(lane)
    assert ResourceId("vertex", "B") in graph.lane_resources(lane)


def test_adjacent_rotations_share_turn_only_resource() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.2, y=0.0),
    }
    graph = TrafficGraph.from_route_core(
        landmarks,
        [],
        default_speed_mps=1.0,
        min_robot_center_distance_m=1.15,
        rotation_min_robot_center_distance_m=1.304,
    )

    a_vertex = set(graph.vertex_resources("A"))
    a_turn = set(graph.rotation_resources("A"))
    b_turn = set(graph.rotation_resources("B"))
    shared_turn_resources = {
        resource
        for resource in a_turn & b_turn
        if resource.kind == "rotation_clearance"
    }

    assert ResourceId("vertex", "B") not in a_vertex
    assert shared_turn_resources
    assert not any(resource.kind == "rotation_clearance" for resource in a_vertex)


def test_adjacent_rotations_outside_turn_radius_do_not_share_resource() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.2, y=0.0),
    }
    graph = TrafficGraph.from_route_core(
        landmarks,
        [],
        default_speed_mps=1.0,
        min_robot_center_distance_m=1.15,
        rotation_min_robot_center_distance_m=1.10,
    )

    shared_turn_resources = {
        resource
        for resource in (
            set(graph.rotation_resources("A"))
            & set(graph.rotation_resources("B"))
        )
        if resource.kind == "rotation_clearance"
    }

    assert not shared_turn_resources


def test_fleet_cbs_uses_same_mutex_resources_as_commit_validator() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=0.0, y=2.0),
        "D": Landmark(name="D", x=1.0, y=2.0),
    }
    edges = []
    for start, goal in (("A", "B"), ("C", "D")):
        edges.append(
            GraphEdge(
                from_name=start,
                to_name=goal,
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[start].x, landmarks[start].y),
                    WorldPoint(landmarks[goal].x, landmarks[goal].y),
                ),
                properties={"direction": 2, "mutex_zone": "crossing"},
            )
        )
    planner = FleetMapfPlanner(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0},
            "fleet": {
                "planner_backend": "cbs",
                "reservation_time_step_sec": 1.0,
                "cbs_low_level_max_time": 10,
                "mapf_min_robot_center_distance_m": 0.5,
            },
        },
    )

    result = planner.plan(
        {
            "rotateEnabled": False,
            "robots": [
                {"name": "r1", "startLm": "A", "goalLm": "B"},
                {"name": "r2", "startLm": "C", "goalLm": "D"},
            ],
        }
    )

    assert result["ok"]
    assert result["debug"]["conflictsResolved"] == 1
    assert sorted(plan["times"][-1] for plan in result["plans"]) == [1, 2]


def test_locked_spatial_route_waits_instead_of_taking_free_detour() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "D": Landmark(name="D", x=1.0, y=1.0),
    }
    planner = FleetMapfPlanner(
        landmarks,
        [
            _edge(landmarks, "A", "B"),
            _edge(landmarks, "B", "C"),
            _edge(landmarks, "A", "D"),
            _edge(landmarks, "D", "C"),
        ],
        params={
            "navigation": {"route_speed": 1.0},
            "fleet": {
                "planner_backend": "rolling_sipp",
                "reservation_time_step_sec": 1.0,
                "cbs_low_level_max_time": 12,
                "mapf_min_robot_center_distance_m": 0.5,
                "reserved_edge_detour_enabled": False,
            },
        },
    )

    result = planner.plan(
        {
            "rotate": False,
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "C",
                    "routeNodes": ["A", "B", "C"],
                }
            ],
            "reserved_vertex_intervals": [
                {"node": "B", "start": 0, "end": 2, "robot": "other"},
            ],
        }
    )

    assert result["ok"]
    plan = result["plans"][0]
    assert "D" not in plan["nodes"]
    assert plan["nodes"] == ["A", "A", "A", "A", "B", "C"]


def test_rolling_sipp_rejects_overlapping_initial_robot_footprints() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=0.5, y=0.0),
        "C": Landmark(name="C", x=2.0, y=0.0),
        "D": Landmark(name="D", x=3.0, y=0.0),
    }
    params = _rolling_params()
    params["fleet"]["mapf_min_robot_center_distance_m"] = 0.83
    planner = FleetMapfPlanner(
        landmarks,
        [_edge(landmarks, "A", "C"), _edge(landmarks, "B", "D")],
        params=params,
    )

    result = planner.plan(
        {
            "robots": [
                {"name": "r1", "startLm": "A", "goalLm": "C"},
                {"name": "r2", "startLm": "B", "goalLm": "D"},
            ]
        }
    )

    assert not result["ok"]
    assert result["plans"] == []


def test_reservation_table_blocks_overlapping_resource_interval() -> None:
    table = ReservationTable()
    resource = ResourceId("vertex", "A")
    table.reserve(ReservationInterval(resource, "r1", 2, 5))

    assert not table.is_free(resource, 4, 6)
    assert table.is_free(resource, 5, 6)


def test_reservation_table_returns_safe_intervals() -> None:
    table = ReservationTable()
    resource = ResourceId("vertex", "A")
    table.reserve(ReservationInterval(resource, "r1", 2, 5))
    table.reserve(ReservationInterval(resource, "r2", 7, 8))

    intervals = table.safe_intervals_for_resources([resource], 0, 10)

    assert [(interval.start, interval.end) for interval in intervals] == [
        (0, 2),
        (5, 7),
        (8, 10),
    ]


def test_reservation_capacity_uses_concurrent_distinct_robots() -> None:
    resource = ResourceId("lane_group", "shared")
    table = ReservationTable({resource: 2})
    table.reserve(ReservationInterval(resource, "r1", 0, 2))
    table.reserve(ReservationInterval(resource, "r2", 3, 5))

    assert table.is_free(resource, 0, 5)
    assert table.safe_intervals_for_resources([resource], 0, 5) == (
        table.safe_intervals_for_resources([resource], 0, 5)[0],
    )
    assert (
        table.safe_intervals_for_resources([resource], 0, 5)[0].start,
        table.safe_intervals_for_resources([resource], 0, 5)[0].end,
    ) == (0, 5)


def test_duplicate_reservations_from_one_robot_consume_one_capacity_slot() -> None:
    resource = ResourceId("lane_group", "shared")
    table = ReservationTable({resource: 2})
    table.reserve(ReservationInterval(resource, "r1", 0, 5, reason="visit"))
    table.reserve(ReservationInterval(resource, "r1", 0, 5, reason="wait"))

    assert table.is_free(resource, 0, 5)
    intervals = table.safe_intervals_for_resources([resource], 0, 5)
    assert [(interval.start, interval.end) for interval in intervals] == [(0, 5)]


def test_rolling_sipp_never_uses_vertex_at_safe_interval_end() -> None:
    landmarks = _landmarks("A", "B", "C", "D")
    graph = TrafficGraph.from_route_core(
        landmarks,
        [
            _edge(landmarks, "A", "B"),
            _edge(landmarks, "B", "A"),
            _edge(landmarks, "B", "C"),
            _edge(landmarks, "C", "B"),
            _edge(landmarks, "B", "D"),
            _edge(landmarks, "D", "B"),
        ],
        default_speed_mps=1.0,
    )
    planner = RollingSippPlanner(graph, low_level_max_time=12)

    result = planner.plan_for_robots(
        [
            LmRobotRequest("r1", "A", "C"),
            LmRobotRequest("r2", "C", "A"),
        ]
    )

    # Prioritized SIPP may reject this solvable case and let the hybrid backend
    # repair it, but it must never return the old colliding t=2 plan.
    if result.plans:
        first = result.plans["r1"]
        second = result.plans["r2"]
        occupied = {}
        for name, plan in (("r1", first), ("r2", second)):
            for time_tick, node in zip(plan.times, plan.nodes):
                assert (time_tick, node) not in occupied
                occupied[(time_tick, node)] = name


def test_rolling_sipp_backend_waits_for_reserved_vertex_interval() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B", "C"),
        _line_edges(("A", "B"), ("B", "C")),
        params=_rolling_params(),
    )

    result = planner.plan(
        {
            "robots": [{"name": "r1", "startLm": "A", "goalLm": "C"}],
            "reserved_vertex_intervals": [
                {"node": "B", "start": 1.0, "end": 2.0, "robot": "other"},
            ],
        }
    )

    assert result["ok"]
    assert result["debug"]["plannerBackend"] == "rolling_sipp"
    assert result["debug"]["reason"].startswith("rolling_sipp:success")
    plan = result["plans"][0]
    # Traversal A->B owns B's endpoint clearance as well, so it starts only
    # after the half-open reservation [1, 3) has cleared.
    assert plan["nodes"] == ["A", "A", "A", "A", "B", "C"]
    assert plan["times"] == [0, 1, 2, 3, 4, 5]


def test_rolling_sipp_backend_waits_for_reserved_edge_interval() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B", "C"),
        _line_edges(("A", "B"), ("B", "C")),
        params=_rolling_params(),
    )

    result = planner.plan(
        {
            "robots": [{"name": "r1", "startLm": "A", "goalLm": "C"}],
            "reserved_edge_intervals": [
                {"from": "B", "to": "C", "start": 0.0, "end": 2.0, "robot": "other"},
            ],
        }
    )

    assert result["ok"]
    plan = result["plans"][0]
    assert plan["nodes"] == ["A", "A", "A", "A", "B", "C"]
    assert plan["times"] == [0, 1, 2, 3, 4, 5]


def test_rolling_sipp_does_not_wait_on_non_waitable_lm() -> None:
    landmarks = _landmarks("A", "B")
    landmarks["A"] = Landmark(name="A", x=0.0, y=0.0, properties={"can_wait": False})
    planner = FleetMapfPlanner(
        landmarks,
        [_edge(landmarks, "A", "B")],
        params=_rolling_params(),
    )

    result = planner.plan(
        {
            "robots": [{"name": "r1", "startLm": "A", "goalLm": "B"}],
            "reserved_vertex_intervals": [
                {"node": "B", "start": 0.0, "end": 2.0, "robot": "other"},
            ],
        }
    )

    assert not result["ok"]
    assert result["debug"]["reason"].startswith("cannot_wait:A")


def test_zero_tick_constraints_are_not_dropped() -> None:
    planner = FleetMapfPlanner(_landmarks("A", "B"), [], params=_rolling_params())

    assert planner._reserved_vertex_constraints(
        {"reserved_vertex_constraints": [{"time": 0, "node": "A"}]}
    ) == [(0, "A")]
    assert planner._reserved_edge_constraints(
        {"reserved_edge_constraints": [{"time": 0, "from": "A", "to": "B"}]}
    ) == [(0, "A", "B")]


def test_duplicate_robot_names_are_rejected() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B"),
        _line_edges(("A", "B")),
        params=_rolling_params(),
    )

    with pytest.raises(ValueError, match="duplicate robot name"):
        planner.plan(
            {
                "robots": [
                    {"name": "r1", "startLm": "A", "goalLm": "B"},
                    {"name": "r1", "startLm": "A", "goalLm": "B"},
                ]
            }
        )


def test_start_pose_cannot_create_an_off_graph_approach() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B"),
        _line_edges(("A", "B")),
        params=_rolling_params(),
    )

    with pytest.raises(ValueError, match="off-graph approach is forbidden"):
        planner.plan(
            {
                "robots": [
                    {
                        "name": "r1",
                        "startLm": "A",
                        "goalLm": "B",
                        "startPose": {"x": 0.5, "y": 0.5, "yaw": 0.0},
                    }
                ]
            }
        )


def test_mapf_trajectory_contains_only_real_graph_edges() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B", "C"),
        _line_edges(("A", "B"), ("B", "C")),
        params=_rolling_params(),
    )

    result = planner.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "C",
                    "startPose": {"x": 0.0, "y": 0.0, "yaw": 1.2},
                }
            ]
        }
    )

    trajectory = result["plans"][0]["trajectory"]
    moving_edges = {
        str(sample["edgeId"])
        for sample in trajectory
        if "->" in str(sample["edgeId"])
        and not str(sample["edgeId"]).startswith("WAIT@")
        and str(sample["edgeId"]).split("->", 1)[0]
        != str(sample["edgeId"]).split("->", 1)[1]
    }
    assert moving_edges == {"A->B", "B->C"}
    assert not any("CURRENT" in str(sample["edgeId"]) for sample in trajectory)
    assert any(sample.get("lm") == "B" for sample in trajectory)


def test_rotation_never_shifts_motion_outside_mapf_reservation_time() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=1.0, y=1.0),
    }
    params = _rolling_params()
    params["fleet"]["stretch_motion_to_reservation_ticks"] = True
    planner = FleetMapfPlanner(
        landmarks,
        [
            _edge(landmarks, "A", "B"),
            _edge(landmarks, "B", "C"),
        ],
        params=params,
    )

    result = planner.plan(
        {
            "robots": [{"name": "r1", "startLm": "A", "goalLm": "C"}],
            "rotate": True,
            "turnSpeed": 0.9,
            "stretchMotionToReservationTicks": True,
        }
    )

    plan = result["plans"][0]
    assert plan["arrivalTime"] == plan["times"][-1] * result["timeStepSec"]


def test_rotation_is_an_explicit_reserved_action_with_real_start_yaw() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    params = _rolling_params()
    params["fleet"]["stretch_motion_to_reservation_ticks"] = True
    planner = FleetMapfPlanner(
        landmarks,
        [_edge(landmarks, "A", "B")],
        params=params,
    )

    result = planner.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "B",
                    "startPose": {"x": 0.0, "y": 0.0, "yaw": math.pi},
                }
            ],
            "rotate": True,
            "turnSpeed": 0.9,
            "stretchMotionToReservationTicks": True,
        }
    )

    plan = result["plans"][0]
    assert plan["nodes"][:2] == ["A", "A"]
    assert plan["actions"][:3] == ["start", "rotate", "move"]
    assert plan["times"][1] == 4  # ceil(pi / 0.9) reservation seconds
    assert plan["trajectory"][0]["yaw"] == pytest.approx(math.pi)
    rotate_sample = next(
        sample
        for sample in plan["trajectory"]
        if str(sample.get("edgeId", "")).startswith("WAIT@ROTATE")
    )
    assert rotate_sample["t"] == pytest.approx(4.0)
    assert rotate_sample["x"] == pytest.approx(0.0)
    assert rotate_sample["y"] == pytest.approx(0.0)


def test_rotation_duration_uses_short_wrap_across_pi_boundary() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B"),
        [],
        params=_rolling_params(),
    )

    duration = planner._rotation_duration(
        math.radians(135.0),
        math.radians(-135.0),
        1.0,
    )

    assert duration == pytest.approx(math.pi / 2.0)


def _rolling_params() -> dict[str, object]:
    return {
        "navigation": {"route_speed": 1.0},
        "fleet": {
            "planner_backend": "rolling_sipp",
            "reservation_time_step_sec": 1.0,
            "wait_time_sec": 1.0,
            "wait_cost": 6,
            "cbs_low_level_max_time": 12,
            "reserved_edge_detour_enabled": False,
        },
    }


def _landmarks(*names: str) -> dict[str, Landmark]:
    return {
        name: Landmark(name=name, x=float(index), y=0.0)
        for index, name in enumerate(names)
    }


def _line_edges(*pairs: tuple[str, str]) -> list[GraphEdge]:
    names = sorted({name for pair in pairs for name in pair})
    landmarks = _landmarks(*names)
    return [_edge(landmarks, start, goal) for start, goal in pairs]


def _edge(
    landmarks: dict[str, Landmark],
    start: str,
    goal: str,
    *,
    length: float = 1.0,
) -> GraphEdge:
    return GraphEdge(
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
