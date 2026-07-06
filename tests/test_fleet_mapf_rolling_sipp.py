from __future__ import annotations

from fleet_manager.mapf import FleetMapfPlanner, ReservationInterval, ReservationTable, ResourceId, TrafficGraph
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
    assert plan["nodes"] == ["A", "A", "A", "B", "C"]
    assert plan["times"] == [0, 1, 2, 3, 4]


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
                {"from": "A", "to": "B", "start": 0.0, "end": 2.0, "robot": "other"},
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
            "reserved_edge_intervals": [
                {"from": "A", "to": "B", "start": 0.0, "end": 2.0, "robot": "other"},
            ],
        }
    )

    assert not result["ok"]
    assert result["debug"]["reason"].startswith("cannot_wait:A")


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
