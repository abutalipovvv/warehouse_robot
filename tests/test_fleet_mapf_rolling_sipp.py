from __future__ import annotations

import math
import pytest

from fleet_manager.core.mapf.fleet.fleet_planner import FleetMapfPlanner
from fleet_manager.core.mapf.cbs.cbs_models import LmRobotRequest
from fleet_manager.core.mapf.common.reservations import (
    ReservationInterval,
    ReservationTable,
    ResourceId,
)
from fleet_manager.core.mapf.rolling.rolling_sipp import RollingSippPlanner
from fleet_manager.core.mapf.sipp.sipp import SippPlanner
from fleet_manager.core.mapf.sipp.sipp_models import SippRobotRequest
from fleet_manager.core.mapf.graph.traffic_graph_models import TrafficGraph
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint


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


def test_traffic_graph_builds_whole_controlled_corridor_resource() -> None:
    landmarks = _landmarks("A", "B", "C", "D", "E")
    edges = [
        _edge(landmarks, start, goal)
        for start, goal in (
            ("A", "B"),
            ("B", "A"),
            ("B", "C"),
            ("C", "B"),
            ("C", "D"),
            ("D", "C"),
            ("D", "E"),
            ("E", "D"),
        )
    ]
    graph = TrafficGraph.from_route_core(
        landmarks,
        edges,
        default_speed_mps=1.0,
        controlled_corridors_enabled=True,
    )

    region_ids = graph.controlled_region_ids()

    assert len(region_ids) == 1
    assert graph.vertices["A"].can_wait
    assert graph.vertices["B"].can_wait
    assert not graph.vertices["C"].can_wait
    assert graph.vertices["D"].can_wait
    assert graph.vertices["E"].can_wait
    assert all(
        ResourceId("controlled_region", region_ids[0])
        in graph.lane_resources(graph.lane_for(start, goal))
        for start, goal in (
            ("A", "B"),
            ("B", "C"),
            ("C", "D"),
            ("D", "E"),
        )
    )
    assert (
        graph.extend_route_index_to_controlled_exit(
            ["A", "B", "C", "D", "E"],
            1,
        )
        == 4
    )


def test_controlled_corridor_auto_mode_only_enables_fully_smart_graphs() -> None:
    landmarks = _landmarks("A", "B", "C")
    smart_edges = [
        _edge(landmarks, start, goal)
        for start, goal in (
            ("A", "B"),
            ("B", "A"),
            ("B", "C"),
            ("C", "B"),
        )
    ]
    for edge in smart_edges:
        edge.properties["smart"] = True
    smart_planner = FleetMapfPlanner(
        landmarks,
        smart_edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": "auto",
                "controlled_corridor_min_edges": 2,
            }
        },
    )
    ordinary_planner = FleetMapfPlanner(
        landmarks,
        [
            _edge(landmarks, start, goal)
            for start, goal in (
                ("A", "B"),
                ("B", "A"),
                ("B", "C"),
                ("C", "B"),
            )
        ],
        params={"fleet": {"controlled_corridors_enabled": "auto"}},
    )

    assert smart_planner.controlled_corridors_enabled
    assert smart_planner.controlled_corridor_auto_detect
    assert smart_planner.controlled_corridors_mode == "auto"
    assert len(smart_planner._traffic_graph(1.0).controlled_region_ids()) == 1
    assert ordinary_planner.controlled_corridors_enabled
    assert not ordinary_planner.controlled_corridor_auto_detect
    assert ordinary_planner._traffic_graph(1.0).controlled_region_ids() == ()


def test_explicit_only_mode_does_not_turn_smart_graph_into_corridors() -> None:
    landmarks = _landmarks("A", "B", "C")
    smart_edges = [
        _edge(landmarks, start, goal)
        for start, goal in (
            ("A", "B"),
            ("B", "A"),
            ("B", "C"),
            ("C", "B"),
        )
    ]
    for edge in smart_edges:
        edge.properties["smart"] = True

    planner = FleetMapfPlanner(
        landmarks,
        smart_edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
            }
        },
    )

    graph = planner._traffic_graph(1.0)
    assert planner.controlled_corridors_enabled
    assert not planner.controlled_corridor_auto_detect
    assert graph.controlled_region_ids() == ()
    assert all(vertex.can_wait for vertex in graph.vertices.values())


def test_controlled_corridor_can_cover_single_edge_between_junctions() -> None:
    landmarks = _landmarks("A", "B")
    graph = TrafficGraph.from_route_core(
        landmarks,
        [
            _edge(landmarks, "A", "B"),
            _edge(landmarks, "B", "A"),
        ],
        default_speed_mps=1.0,
        controlled_corridors_enabled=True,
        controlled_corridor_min_edges=1,
    )

    assert graph.controlled_region_ids() == ("A<=>B",)
    assert graph.lane_for("A", "B").controlled_region_ids == ("A<=>B",)
    assert graph.lane_for("B", "A").controlled_region_ids == ("A<=>B",)


def test_corridor_horizon_spans_non_waitable_transfer_junction() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "J": Landmark(name="J", x=1.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
        "X": Landmark(name="X", x=1.0, y=1.0),
    }
    edges = [
        _edge(landmarks, src, dst)
        for first, second in (("A", "J"), ("J", "B"), ("J", "X"))
        for src, dst in ((first, second), (second, first))
    ]
    for edge in edges:
        edge.properties["smart"] = True
    planner = FleetMapfPlanner(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": "auto",
                "controlled_corridor_min_edges": 1,
            }
        },
    )

    graph = planner._traffic_graph(1.0)
    assert not graph.vertices["J"].can_wait
    single_edge_ticks = planner._edge_tick_cost("A", "J", 1.0, 1.0)
    assert planner.controlled_corridor_max_ticks(
        speed=1.0,
        acceleration=1.0,
    ) >= single_edge_ticks * 2


def test_explicit_editor_corridor_properties_work_without_auto_detection() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0, properties={"holding_point": True}),
        "B": Landmark(
            name="B",
            x=1.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": "corridor:A<=>D"},
        ),
        "C": Landmark(
            name="C",
            x=2.0,
            y=0.0,
            properties={"can_wait": False, "controlled_region": "corridor:A<=>D"},
        ),
        "D": Landmark(name="D", x=3.0, y=0.0, properties={"holding_point": True}),
    }
    edges = []
    for start, goal in (("A", "B"), ("B", "C"), ("C", "D")):
        edge = _edge(landmarks, start, goal)
        edge.properties["controlled_region"] = "corridor:A<=>D"
        edges.append(edge)
    graph = TrafficGraph.from_route_core(
        landmarks,
        edges,
        default_speed_mps=1.0,
        controlled_corridors_enabled=False,
    )

    assert graph.controlled_region_ids() == ("corridor:A<=>D",)
    assert not graph.vertices["B"].can_wait
    assert not graph.vertices["C"].can_wait
    assert graph.extend_route_index_to_controlled_exit(["A", "B", "C", "D"], 1) == 3


def test_edge_authored_corridor_infers_internal_non_waiting_vertices() -> None:
    landmarks = _landmarks("A", "B", "C", "D")
    region = "corridor:painted-zone"
    edges = []
    for first, second in (("A", "B"), ("B", "C"), ("C", "D")):
        for src, dst in ((first, second), (second, first)):
            edge = _edge(landmarks, src, dst)
            edge.properties["controlled_region"] = region
            edges.append(edge)

    graph = TrafficGraph.from_route_core(
        landmarks,
        edges,
        default_speed_mps=1.0,
        explicit_controlled_regions_enabled=True,
        controlled_corridors_enabled=False,
    )

    assert graph.vertices["A"].controlled_region_ids == ()
    assert graph.vertices["A"].can_wait
    assert graph.vertices["B"].controlled_region_ids == (region,)
    assert not graph.vertices["B"].can_wait
    assert graph.vertices["C"].controlled_region_ids == (region,)
    assert not graph.vertices["C"].can_wait
    assert graph.vertices["D"].controlled_region_ids == ()
    assert graph.vertices["D"].can_wait
    assert ResourceId("controlled_region", region) in graph.vertex_resources("B")
    assert graph.extend_route_index_to_controlled_exit(
        ["A", "B", "C", "D"],
        1,
    ) == 3


def test_rolling_sipp_serializes_head_on_controlled_corridor_without_internal_waits() -> None:
    landmarks = {
        name: Landmark(name=name, x=x, y=y)
        for name, x, y in (
            ("LS", -2.0, -1.0),
            ("LG", -2.0, 1.0),
            ("L", -1.0, 0.0),
            ("B", 0.0, 0.0),
            ("C", 1.0, 0.0),
            ("R", 2.0, 0.0),
            ("RS", 3.0, -1.0),
            ("RG", 3.0, 1.0),
        )
    }
    pairs = (
        ("LS", "L"),
        ("LG", "L"),
        ("L", "B"),
        ("B", "C"),
        ("C", "R"),
        ("R", "RS"),
        ("R", "RG"),
    )
    edges = [
        _edge(landmarks, start, goal)
        for first, second in pairs
        for start, goal in ((first, second), (second, first))
    ]
    planner = FleetMapfPlanner(
        landmarks,
        edges,
        params={
            "navigation": {"route_speed": 1.0},
            "fleet": {
                "planner_backend": "rolling_sipp",
                "reservation_time_step_sec": 1.0,
                "cbs_low_level_max_time": 24,
                "mapf_min_robot_center_distance_m": 0.1,
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": True,
            },
        },
    )

    result = planner.plan(
        {
            "rotate": False,
            "robots": [
                {"name": "r1", "startLm": "LS", "goalLm": "RG"},
                {"name": "r2", "startLm": "RS", "goalLm": "LG"},
            ],
        }
    )

    assert result["ok"], result["debug"]
    assert result["debug"]["controlledCorridors"] == 1
    for plan in result["plans"]:
        for start, end in zip(plan["nodes"], plan["nodes"][1:]):
            assert not (start == end and start in {"B", "C"})


def test_cbs_branches_on_whole_controlled_corridor_occupation() -> None:
    landmarks = {
        name: Landmark(name=name, x=x, y=y)
        for name, x, y in (
            ("LS", -2.0, -1.0),
            ("LG", -2.0, 1.0),
            ("L", -1.0, 0.0),
            ("B", 0.0, 0.0),
            ("C", 1.0, 0.0),
            ("R", 2.0, 0.0),
            ("RS", 3.0, -1.0),
            ("RG", 3.0, 1.0),
        )
    }
    pairs = (
        ("LS", "L"),
        ("LG", "L"),
        ("L", "B"),
        ("B", "C"),
        ("C", "R"),
        ("R", "RS"),
        ("R", "RG"),
    )
    planner = FleetMapfPlanner(
        landmarks,
        [
            _edge(landmarks, start, goal)
            for first, second in pairs
            for start, goal in ((first, second), (second, first))
        ],
        params={
            "navigation": {"route_speed": 1.0},
            "fleet": {
                "planner_backend": "cbs",
                "reservation_time_step_sec": 1.0,
                "cbs_low_level_max_time": 24,
                "cbs_max_high_level_nodes": 200,
                "cbs_max_planning_time_sec": 2.0,
                "mapf_min_robot_center_distance_m": 0.1,
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": True,
            },
        },
    )

    result = planner.plan(
        {
            "rotate": False,
            "robots": [
                {"name": "r1", "startLm": "LS", "goalLm": "RG"},
                {"name": "r2", "startLm": "RS", "goalLm": "LG"},
            ],
        }
    )

    assert result["ok"], result["debug"]
    assert result["debug"]["conflictsResolved"] >= 1
    assert result["debug"]["highLevelNodes"] < 20
    for plan in result["plans"]:
        for start, end in zip(plan["nodes"], plan["nodes"][1:]):
            assert not (start == end and start in {"B", "C"})


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


def test_rolling_sipp_observes_the_runtime_planning_deadline() -> None:
    landmarks = _landmarks("A", "B")
    graph = TrafficGraph.from_route_core(
        landmarks,
        [_edge(landmarks, "A", "B")],
        default_speed_mps=1.0,
    )
    planner = RollingSippPlanner(
        graph,
        low_level_max_time=12,
        max_planning_time_sec=0.0,
    )

    result = planner.plan_for_robots(
        [LmRobotRequest("r1", "A", "B")]
    )

    assert result.plans == {}
    assert result.debug.reason == "rolling_sipp:planning_timeout"


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


def test_hybrid_failure_preserves_external_reservation_owner() -> None:
    params = _rolling_params()
    params["fleet"]["planner_backend"] = "hybrid"
    planner = FleetMapfPlanner(
        _landmarks("A", "B"),
        _line_edges(("A", "B")),
        params=params,
    )

    result = planner.plan(
        {
            "robots": [{"name": "waiter", "startLm": "A", "goalLm": "B"}],
            "reserved_vertex_intervals": [
                {
                    "node": "B",
                    "start": 0.0,
                    "end": 20.0,
                    "robot": "resource-owner",
                },
            ],
            "allowCbsFallback": True,
        }
    )

    assert not result["ok"]
    assert result["debug"]["reservationBlockerRobots"] == [
        "resource-owner"
    ]
    assert result["debug"]["reservationBlockers"] == [
        {
            "robot": "resource-owner",
            "resource": "vertex:B",
        }
    ]
    assert "rolling_sipp:" in result["debug"]["reservedFallbackReason"]
    assert "cbs:" in result["debug"]["reservedFallbackReason"]


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
    assert "cannot_wait:A" in result["debug"]["reason"]


def test_rolling_sipp_moves_downstream_wait_to_safe_corridor_staging() -> None:
    landmarks = {
        name: Landmark(
            name=name,
            x=float(index * 3),
            y=0.0,
            properties={"can_wait": name in {"A", "D"}},
        )
        for index, name in enumerate(("A", "B", "C", "D"))
    }
    params = _rolling_params()
    params["fleet"]["cbs_low_level_max_time"] = 40
    planner = FleetMapfPlanner(
        landmarks,
        [
            _edge(landmarks, start, goal, length=3.0)
            for start, goal in (
                ("A", "B"),
                ("B", "C"),
                ("C", "D"),
            )
        ],
        params=params,
    )

    result = planner.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "D",
                    "routeNodes": ["A", "B", "C", "D"],
                }
            ],
            "reserved_edge_intervals": [
                {
                    "from": "C",
                    "to": "D",
                    "start": 0.0,
                    "end": 8.0,
                    "robot": "other",
                },
            ],
        }
    )

    assert result["ok"]
    assert "staging_wait_repairs=" in result["debug"]["reason"]
    plan = result["plans"][0]
    assert plan["nodes"][:4] == ["A", "A", "A", "A"]
    assert "B" in plan["nodes"]
    assert plan["nodes"][-1] == "D"
    assert all(
        not (
            node in {"B", "C"}
            and action == "wait"
        )
        for node, action in zip(plan["nodes"], plan["actions"])
    )


def test_rolling_sipp_honours_central_corridor_start_not_before() -> None:
    planner = FleetMapfPlanner(
        _landmarks("A", "B"),
        _line_edges(("A", "B")),
        params=_rolling_params(),
    )

    result = planner.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "B",
                    "startNotBeforeSec": 3.0,
                }
            ],
            "allowCbsFallback": True,
        }
    )

    assert result["ok"]
    assert not result["debug"]["cbsFallbackAllowed"]
    plan = result["plans"][0]
    assert plan["nodes"][:4] == ["A", "A", "A", "A"]
    assert plan["actions"][:4] == ["start", "wait", "wait", "wait"]
    assert plan["times"][-1] >= 4


def test_rolling_sipp_waits_at_corridor_staging_not_route_start() -> None:
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
                    "routeNodes": ["A", "B", "C"],
                    "departureNotBefore": [
                        {"node": "B", "timeSec": 5.0},
                    ],
                }
            ],
            "allowCbsFallback": True,
        }
    )

    assert result["ok"]
    assert not result["debug"]["cbsFallbackAllowed"]
    plan = result["plans"][0]
    assert plan["nodes"][0] == "A"
    assert plan["nodes"][1] == "B"
    assert plan["times"][1] == 1
    assert plan["nodes"][1:6] == ["B", "B", "B", "B", "B"]
    assert plan["actions"][2:6] == ["wait", "wait", "wait", "wait"]
    assert plan["nodes"][-1] == "C"
    assert plan["times"][-1] >= 6


def test_request_scoped_no_wait_moves_clearance_wait_before_stop_line() -> None:
    """A backed-off corridor passage may not pause after its stop line."""
    landmarks = _landmarks("A", "B", "C", "D")
    params = _rolling_params()
    params["fleet"]["cbs_low_level_max_time"] = 40
    planner = FleetMapfPlanner(
        landmarks,
        _line_edges(("A", "B"), ("B", "C"), ("C", "D")),
        params=params,
    )

    result = planner.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "D",
                    "routeNodes": ["A", "B", "C", "D"],
                    "departureNotBefore": [
                        {"node": "A", "timeSec": 0.0},
                    ],
                    # B and C are ordinarily waitable map LMs, but for this
                    # admitted atomic passage they lie beyond its stop line.
                    "noWaitNodes": ["B", "C"],
                }
            ],
            "reserved_edge_intervals": [
                {
                    "from": "C",
                    "to": "D",
                    "start": 0.0,
                    "end": 8.0,
                    "robot": "other",
                },
            ],
        }
    )

    assert result["ok"], result["debug"]
    plan = result["plans"][0]
    assert "staging_wait_repairs=" in result["debug"]["reason"]
    assert any(
        node == "A" and action == "wait"
        for node, action in zip(plan["nodes"], plan["actions"])
    )
    assert all(
        not (
            node in {"B", "C"}
            and action == "wait"
        )
        for node, action in zip(plan["nodes"], plan["actions"])
    )


def test_central_authority_pipelines_same_direction_corridor_convoy() -> None:
    landmarks = {
        name: Landmark(name=name, x=x, y=y)
        for name, x, y in (
            ("L2", -2.0, 0.0),
            ("L1", -1.0, 0.0),
            ("A", 0.0, 0.0),
            ("B", 1.0, 0.0),
            ("C", 2.0, 0.0),
            ("D", 3.0, 0.0),
            ("G1", 4.0, 1.0),
            ("G2", 4.0, -1.0),
        )
    }
    region = "corridor:convoy"
    edges = [
        _edge(landmarks, src, dst)
        for src, dst in (
            ("L2", "L1"),
            ("L1", "A"),
            ("A", "B"),
            ("B", "C"),
            ("C", "D"),
            ("D", "G1"),
            ("D", "G2"),
        )
    ]
    for edge in edges:
        if (edge.from_name, edge.to_name) in {
            ("A", "B"),
            ("B", "C"),
            ("C", "D"),
        }:
            edge.properties["controlled_region"] = region
    params = _rolling_params()
    params["fleet"]["cbs_low_level_max_time"] = 24
    planner = FleetMapfPlanner(landmarks, edges, params=params)
    requests = [
        {
            "name": "leader",
            "startLm": "L1",
            "goalLm": "G1",
            "routeNodes": ["L1", "A", "B", "C", "D", "G1"],
            "departureNotBefore": [{"node": "A", "timeSec": 1.0}],
            "authorizedControlledRegions": [region],
        },
        {
            "name": "follower",
            "startLm": "L2",
            "goalLm": "G2",
            "routeNodes": [
                "L2",
                "L1",
                "A",
                "B",
                "C",
                "D",
                "G2",
            ],
            "departureNotBefore": [{"node": "A", "timeSec": 3.0}],
            "authorizedControlledRegions": [region],
        },
    ]

    result = planner.plan({"rotate": False, "robots": requests})

    assert result["ok"], result["debug"]
    plans = {plan["robot"]: plan for plan in result["plans"]}

    def transition_time(plan: dict[str, object], src: str, dst: str) -> int:
        nodes = plan["nodes"]
        times = plan["times"]
        return next(
            int(times[index - 1])
            for index in range(1, len(nodes))
            if nodes[index - 1] == src and nodes[index] == dst
        )

    leader_exit = transition_time(plans["leader"], "D", "G1")
    follower_entry = transition_time(plans["follower"], "A", "B")
    assert follower_entry < leader_exit
    # The whole-region mutex is bypassed, but ordinary graph resources still
    # keep the follower at least one complete edge behind the leader.
    assert follower_entry >= transition_time(plans["leader"], "A", "B") + 1


def test_central_authority_is_scoped_to_exact_controlled_region_ids() -> None:
    landmarks = _landmarks("A", "B", "C")
    first_region = "corridor:first"
    second_region = "corridor:second"
    first = _edge(landmarks, "A", "B")
    first.properties["controlled_region"] = first_region
    second = _edge(landmarks, "B", "C")
    second.properties["controlled_region"] = second_region
    graph = TrafficGraph.from_route_core(
        landmarks,
        [first, second],
        default_speed_mps=1.0,
    )
    reservations = ReservationTable()
    reservations.reserve(
        ReservationInterval(
            ResourceId("controlled_region", first_region),
            "same-convoy",
            0,
            10,
        )
    )
    reservations.reserve(
        ReservationInterval(
            ResourceId("controlled_region", second_region),
            "other-phase",
            0,
            5,
        )
    )
    planner = SippPlanner(graph, low_level_max_time=12)

    path = planner.plan(
        SippRobotRequest(
            "robot",
            "A",
            "C",
            route_nodes=("A", "B", "C"),
            authorized_controlled_regions=(first_region,),
        ),
        reservations,
    )

    assert path is not None
    assert path.nodes[0:2] == ["A", "B"]
    assert path.times[1] == 1
    assert path.nodes[-1] == "C"
    assert path.times[-1] >= 6


def test_authorized_region_is_not_reported_as_the_exact_blocker() -> None:
    landmarks = _landmarks("A", "B")
    region = "corridor:convoy"
    edge = _edge(landmarks, "A", "B")
    edge.properties["controlled_region"] = region
    graph = TrafficGraph.from_route_core(
        landmarks,
        [edge],
        default_speed_mps=1.0,
    )
    planner = RollingSippPlanner(graph, low_level_max_time=12)
    reservations = ReservationTable()
    reservations.reserve(
        ReservationInterval(
            ResourceId("controlled_region", region),
            "compatible-convoy",
            0,
            2,
        )
    )
    reservations.reserve(
        ReservationInterval(
            ResourceId("lane", "A->B"),
            "actual-lane-blocker",
            0,
            2,
        )
    )

    owners = planner._blocking_plan_owners(
        "reserved_edge:A->B@0-1",
        reservations,
        authorized_controlled_regions=(region,),
    )

    assert owners == {"actual-lane-blocker"}


def test_corridor_authority_requires_a_matching_route_and_departure_gate() -> None:
    landmarks = _landmarks("A", "B")
    region = "corridor:only"
    edge = _edge(landmarks, "A", "B")
    edge.properties["controlled_region"] = region
    planner = FleetMapfPlanner(
        landmarks,
        [edge],
        params=_rolling_params(),
    )

    with pytest.raises(ValueError, match="requires departureNotBefore"):
        planner.plan(
            {
                "robots": [
                    {
                        "name": "robot",
                        "startLm": "A",
                        "goalLm": "B",
                        "routeNodes": ["A", "B"],
                        "authorizedControlledRegions": [region],
                    }
                ]
            }
        )
    with pytest.raises(ValueError, match="outside routeNodes"):
        planner.plan(
            {
                "robots": [
                    {
                        "name": "robot",
                        "startLm": "A",
                        "goalLm": "B",
                        "routeNodes": ["A", "B"],
                        "departureNotBefore": [
                            {"node": "A", "timeSec": 1.0},
                        ],
                        "authorizedControlledRegions": [
                            "corridor:not-on-route",
                        ],
                    }
                ]
            }
        )


def test_unspecified_motion_cannot_hide_wait_as_rotation_on_corridor_lm() -> None:
    landmarks = _landmarks("A", "B")
    landmarks["A"] = Landmark(
        name="A",
        x=0.0,
        y=0.0,
        properties={"can_wait": False},
    )
    planner = FleetMapfPlanner(
        landmarks,
        [_edge(landmarks, "A", "B", direction=2)],
        params=_rolling_params(),
    )

    result = planner.plan(
        {
            "robots": [
                {
                    "name": "r1",
                    "startLm": "A",
                    "goalLm": "B",
                    "startPose": {
                        "x": 0.0,
                        "y": 0.0,
                        "yaw": 0.0,
                    },
                }
            ],
            "rotate": True,
            "turnSpeed": 0.9,
            "allowCbsFallback": True,
            "reserved_vertex_intervals": [
                {
                    "node": "B",
                    "start": 0.0,
                    "end": 4.0,
                    "robot": "other",
                },
            ],
        }
    )

    assert not result["ok"]
    assert "cannot_wait:A" in result["debug"]["reason"]


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
        [_edge(landmarks, "A", "B", direction=0)],
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


def test_unspecified_motion_keeps_yaw_and_drives_backward_without_rotation() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    params = _rolling_params()
    params["fleet"]["stretch_motion_to_reservation_ticks"] = True
    planner = FleetMapfPlanner(
        landmarks,
        [_edge(landmarks, "A", "B", direction=2)],
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
    assert "rotate" not in plan["actions"]
    assert not any(
        str(sample.get("edgeId", "")).startswith("WAIT@ROTATE")
        for sample in plan["trajectory"]
    )
    moving = [
        sample
        for sample in plan["trajectory"]
        if sample.get("edgeId") == "A->B"
    ]
    assert moving
    assert all(sample["yaw"] == pytest.approx(math.pi) for sample in moving)
    assert all(sample["motionDirection"] == "backward" for sample in moving)


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
    direction: int = 2,
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
        properties={"direction": direction},
    )
