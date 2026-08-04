from __future__ import annotations

from time import monotonic

from fleet_manager.core.mapf.common.reservations import (
    ReservationInterval,
    ReservationTable,
    ResourceId,
)
from fleet_manager.core.mapf.sipp.sipp import SippPlanner
from fleet_manager.core.mapf.sipp.sipp_models import (
    SippRobotRequest,
    SippState,
    TimedPath,
    TimedState,
)
from fleet_manager.core.mapf.graph.traffic_graph_models import TrafficGraph
from fleet_manager.core.mapping.maps.models import (
    GraphEdge,
    Landmark,
    WorldPoint,
)


def _line_graph() -> TrafficGraph:
    landmarks = {
        "A": Landmark("A", 0.0, 0.0),
        "B": Landmark("B", 1.0, 0.0),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="line",
        world_points=(
            WorldPoint(0.0, 0.0),
            WorldPoint(1.0, 0.0),
        ),
    )
    return TrafficGraph.from_route_core(
        landmarks,
        [edge],
        default_speed_mps=1.0,
    )


def test_sipp_models_have_direct_stable_modules() -> None:
    assert SippState.__module__.endswith("sipp_models")
    assert TimedState.__module__.endswith("sipp_models")
    assert TimedPath.__module__.endswith("sipp_models")


def test_sipp_facade_returns_the_legacy_timed_action_path() -> None:
    planner = SippPlanner(_line_graph(), low_level_max_time=5)

    path = planner.plan(
        SippRobotRequest("robot", "A", "B"),
        ReservationTable(),
    )

    assert path is not None
    assert path.nodes == ["A", "B"]
    assert path.times == [0, 1]
    assert path.actions == ["start", "move"]
    assert planner.expanded_nodes == 2


def test_sipp_deadline_is_reported_by_the_shared_search_loop() -> None:
    planner = SippPlanner(_line_graph(), low_level_max_time=5)

    path = planner.plan(
        SippRobotRequest("robot", "A", "B"),
        ReservationTable(),
        planning_deadline=monotonic() - 1.0,
    )

    assert path is None
    assert planner.expanded_nodes == 0
    assert planner.last_failure == "planning_timeout:robot"


def test_initial_state_validation_precedes_deadline_check() -> None:
    planner = SippPlanner(_line_graph(), low_level_max_time=5)

    path = planner.plan(
        SippRobotRequest("robot", "missing", "B"),
        ReservationTable(),
        planning_deadline=monotonic() - 1.0,
    )

    assert path is None
    assert planner.expanded_nodes == 0
    assert planner.last_failure == "unknown_node:missing"


def test_sipp_facade_preserves_exact_blocking_diagnostics() -> None:
    planner = SippPlanner(_line_graph(), low_level_max_time=5)
    reservations = ReservationTable()
    reservations.reserve(
        ReservationInterval(
            ResourceId("vertex", "B"),
            "blocking-robot",
            0,
            6,
        )
    )

    path = planner.plan(
        SippRobotRequest("robot", "A", "B"),
        reservations,
    )

    assert path is None
    assert planner.last_failure == "no_sipp_path:robot:A->B"
    assert planner.blocking_robot_names == {"blocking-robot"}
    assert planner.blocking_resources_by_robot == {
        "blocking-robot": {ResourceId("vertex", "B")}
    }
