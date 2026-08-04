from __future__ import annotations

from fleet_manager.core.mapf.cbs.cbs_conflicts import (
    CbsConflictAnalyzer,
)
from fleet_manager.core.mapf.cbs.cbs_low_level import (
    LmCBSEnvironment,
)
from fleet_manager.core.mapf.cbs.cbs_models import (
    Conflict,
    Constraints,
    EdgeIntervalConstraint,
    LmRobotRequest,
    ResourceIntervalConstraint,
    State,
    VertexConstraint,
    VertexIntervalConstraint,
)
from fleet_manager.core.mapf.common.reservations import ResourceId


def _analyzer(
    *,
    with_resources: bool = False,
) -> CbsConflictAnalyzer:
    return CbsConflictAnalyzer(
        transition_parts=lambda _start, _end: (0, 1),
        vertex_resources_fn=(
            (
                lambda node: (
                    ResourceId("vertex", node),
                )
            )
            if with_resources
            else None
        ),
        rotation_resources_fn=(
            (
                lambda node: (
                    ResourceId("vertex", node),
                )
            )
            if with_resources
            else None
        ),
        lane_resources_fn=(
            (
                lambda _source, _target: (
                    ResourceId("lane_group", "shared"),
                )
            )
            if with_resources
            else None
        ),
    )


def test_cbs_types_live_in_their_focused_modules() -> None:
    assert State.__module__.endswith("cbs_models")
    assert LmCBSEnvironment.__module__.endswith("cbs_low_level")


def test_constraints_merge_without_sharing_default_sets() -> None:
    first = Constraints()
    second = Constraints(
        vertex_constraints={VertexConstraint(2, "B")},
        edge_interval_constraints={
            EdgeIntervalConstraint(3, 5, "A", "B")
        },
    )

    first.add_constraint(second)

    assert first.vertex_constraints == {
        VertexConstraint(2, "B")
    }
    assert first.edge_interval_constraints == {
        EdgeIntervalConstraint(3, 5, "A", "B")
    }
    second.vertex_constraints.add(
        VertexConstraint(4, "C")
    )
    assert VertexConstraint(4, "C") not in (
        first.vertex_constraints
    )


def test_conflict_analyzer_finds_the_first_vertex_conflict() -> None:
    analyzer = _analyzer()
    solution = {
        "first": [
            State(0, "A"),
            State(1, "B"),
        ],
        "second": [
            State(0, "C"),
            State(1, "B"),
        ],
    }

    conflict = analyzer.first_conflict(solution)

    assert conflict is not None
    assert conflict.type == Conflict.VERTEX
    assert conflict.time == 1
    assert conflict.node_1 == "B"
    assert (conflict.agent_1, conflict.agent_2) == (
        "first",
        "second",
    )


def test_conflict_analyzer_detects_reverse_edge_overlap() -> None:
    analyzer = _analyzer()
    solution = {
        "first": [
            State(0, "A"),
            State(1, "B"),
        ],
        "second": [
            State(0, "B"),
            State(1, "A"),
        ],
    }

    conflict = analyzer.first_conflict(solution)

    assert conflict is not None
    assert conflict.type == Conflict.EDGE
    assert conflict.time == 0
    assert conflict.end_time == 1
    assert (
        conflict.agent_1_from,
        conflict.agent_1_to,
    ) == ("A", "B")
    assert (
        conflict.agent_2_from,
        conflict.agent_2_to,
    ) == ("B", "A")


def test_conflict_analyzer_detects_shared_graph_resource() -> None:
    analyzer = _analyzer(with_resources=True)
    solution = {
        "first": [
            State(0, "A"),
            State(1, "B"),
        ],
        "second": [
            State(0, "C"),
            State(1, "D"),
        ],
    }

    conflict = analyzer.first_conflict(solution)

    assert conflict is not None
    assert conflict.type == Conflict.RESOURCE
    assert conflict.time == 0
    assert conflict.resource == ResourceId(
        "lane_group",
        "shared",
    )


def test_controlled_resource_conflict_builds_scoped_constraints() -> None:
    analyzer = _analyzer(with_resources=True)
    resource = ResourceId(
        "controlled_region",
        "corridor",
    )
    conflict = Conflict(
        time=4,
        end_time=6,
        type=Conflict.RESOURCE,
        agent_1="first",
        agent_2="second",
        agent_1_resource_start=2,
        agent_1_resource_end=8,
        agent_2_resource_start=4,
        agent_2_resource_end=6,
        agent_1_resource_entry="A",
        agent_2_resource_entry="D",
        resource=resource,
    )

    constraints = analyzer.constraints_from_conflict(
        conflict
    )

    assert constraints[
        "first"
    ].resource_interval_constraints == {
        ResourceIntervalConstraint(0, 6, resource)
    }
    assert constraints[
        "second"
    ].resource_interval_constraints == {
        ResourceIntervalConstraint(0, 8, resource)
    }
    assert constraints[
        "first"
    ].vertex_interval_constraints == {
        VertexIntervalConstraint(0, 7, "A")
    }
    assert constraints[
        "second"
    ].vertex_interval_constraints == {
        VertexIntervalConstraint(0, 9, "D")
    }


def test_low_level_search_waits_for_reserved_vertex_interval() -> None:
    request = LmRobotRequest("robot", "A", "C")
    environment = LmCBSEnvironment(
        {
            "A": ["B"],
            "B": ["C"],
            "C": [],
        },
        [request],
        global_vertex_intervals=[
            VertexIntervalConstraint(
                1,
                2,
                "B",
                "external",
            )
        ],
        low_level_max_time=10,
    )
    environment.constraint_dict = {
        "robot": Constraints()
    }

    path = environment.low_level_search("robot", 10)

    assert path is not None
    assert [(state.time, state.node) for state in path] == [
        (0, "A"),
        (1, "A"),
        (2, "A"),
        (3, "B"),
        (4, "C"),
    ]
