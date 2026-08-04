from __future__ import annotations

from fleet_manager.core.mapf.graph.traffic_graph import (
    TrafficGraph,
    TrafficLane,
    TrafficVertex,
    lane_id,
)
from fleet_manager.core.mapf.graph.traffic_graph_builder import TrafficGraphBuilder
from fleet_manager.core.mapf.graph.traffic_graph_models import (
    TrafficGraph as TrafficGraphModel,
)
from fleet_manager.core.mapping.maps.models import (
    EdgeGeometry,
    GraphEdge,
    Landmark,
    WorldPoint,
)


def _edge(
    landmarks: dict[str, Landmark],
    start: str,
    end: str,
    *,
    properties: dict[str, object] | None = None,
) -> GraphEdge:
    return GraphEdge(
        from_name=start,
        to_name=end,
        length=1.0,
        kind="line",
        edge_type="line",
        world_points=(
            landmarks[start].to_point(),
            landmarks[end].to_point(),
        ),
        properties=properties or {},
    )


def test_traffic_graph_module_remains_a_stable_facade() -> None:
    assert TrafficGraph is TrafficGraphModel
    assert TrafficVertex.__name__ == "TrafficVertex"
    assert TrafficLane.__name__ == "TrafficLane"
    assert lane_id("A", "B") == "A->B"


def test_builder_matches_classmethod_and_preserves_lane_order() -> None:
    landmarks = {
        "A": Landmark("A", 0.0, 0.0),
        "B": Landmark("B", 1.0, 0.0),
        "C": Landmark("C", 0.0, 1.0),
    }
    edges = [
        _edge(landmarks, "A", "C"),
        _edge(landmarks, "C", "A"),
        _edge(landmarks, "A", "B", properties={"capacity": "2"}),
        _edge(landmarks, "B", "A"),
    ]
    direct = TrafficGraphBuilder(
        landmarks=landmarks,
        edges=edges,
        default_speed_mps=1.0,
    ).build()
    compatible = TrafficGraph.from_route_core(
        landmarks,
        edges,
        default_speed_mps=1.0,
    )

    assert direct == compatible
    assert direct.outgoing["A"] == ["A->B", "A->C"]
    assert direct.lane_for("A", "B").capacity == 2
    assert direct.lane_for("A", "B").lane_group_id == "A<->B"


def test_builder_infers_internal_explicit_region_but_keeps_boundaries() -> None:
    landmarks = {
        "A": Landmark("A", 0.0, 0.0, {"holding_point": True}),
        "B": Landmark("B", 1.0, 0.0),
        "C": Landmark("C", 2.0, 0.0, {"holding_point": True}),
    }
    region = {"controlled_region": "narrow"}
    edges = [
        _edge(landmarks, "A", "B", properties=region),
        _edge(landmarks, "B", "A", properties=region),
        _edge(landmarks, "B", "C", properties=region),
        _edge(landmarks, "C", "B", properties=region),
    ]

    graph = TrafficGraph.from_route_core(
        landmarks,
        edges,
        default_speed_mps=1.0,
    )

    assert graph.vertices["A"].controlled_region_ids == ()
    assert graph.vertices["B"].controlled_region_ids == ("narrow",)
    assert graph.vertices["C"].controlled_region_ids == ()
    assert not graph.vertices["B"].can_wait


def test_builder_uses_curved_centerline_for_lane_clearance() -> None:
    landmarks = {
        "A": Landmark("A", 0.0, 0.0),
        "B": Landmark("B", 2.0, 0.0),
        "N": Landmark("N", 1.0, 0.75),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=3.0,
        kind="bezier",
        edge_type="bezier",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(2.0, 0.0)),
        geometry=EdgeGeometry(
            geometry="bezier",
            control_points=(
                WorldPoint(0.0, 0.0),
                WorldPoint(0.0, 1.5),
                WorldPoint(2.0, 1.5),
                WorldPoint(2.0, 0.0),
            ),
            curve_type="cubic",
        ),
    )

    graph = TrafficGraph.from_route_core(
        landmarks,
        [edge],
        default_speed_mps=1.0,
        min_robot_center_distance_m=0.5,
    )

    assert graph.lanes["A->B"].clearance_zone_ids == ("A->B<->N",)
    assert "A->B<->N" in graph.vertices["N"].clearance_zone_ids
