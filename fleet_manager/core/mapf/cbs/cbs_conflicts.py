"""Conflict detection and constraint generation for CBS."""

from __future__ import annotations

from itertools import combinations
from typing import Callable

from fleet_manager.core.math.intervals import closed_intervals_overlap

from .cbs_models import (
    Conflict,
    Constraints,
    EdgeIntervalConstraint,
    NodeName,
    PathEdgeInterval,
    PathResourceInterval,
    PathVertexInterval,
    ResourceIntervalConstraint,
    State,
    VertexIntervalConstraint,
)


class CbsConflictAnalyzer:
    """Convert timed paths into their earliest CBS conflict."""

    def __init__(
        self,
        *,
        transition_parts: Callable[
            [State, State],
            tuple[int, int],
        ],
        vertex_resources_fn: (
            Callable[[NodeName], tuple[object, ...]] | None
        ),
        rotation_resources_fn: (
            Callable[[NodeName], tuple[object, ...]] | None
        ),
        lane_resources_fn: (
            Callable[
                [NodeName, NodeName],
                tuple[object, ...],
            ]
            | None
        ),
    ) -> None:
        self.transition_parts = transition_parts
        self.vertex_resources_fn = vertex_resources_fn
        self.rotation_resources_fn = rotation_resources_fn
        self.lane_resources_fn = lane_resources_fn

    def first_conflict(
        self,
        solution: dict[str, list[State]],
    ) -> Conflict | None:
        if not solution:
            return None

        horizon = max(
            path[-1].time
            for path in solution.values()
            if path
        )
        vertex_intervals: dict[
            str,
            list[PathVertexInterval],
        ] = {}
        edge_intervals: dict[
            str,
            list[PathEdgeInterval],
        ] = {}
        resource_intervals: dict[
            str,
            list[PathResourceInterval],
        ] = {}

        for agent_name, path in solution.items():
            vertices, edges = self._path_intervals(
                agent_name,
                path,
                horizon,
            )
            vertex_intervals[agent_name] = vertices
            edge_intervals[agent_name] = edges
            if (
                self.vertex_resources_fn is not None
                and self.lane_resources_fn is not None
            ):
                resource_intervals[agent_name] = (
                    self._path_resource_intervals(
                        agent_name,
                        path,
                        horizon,
                    )
                )

        first_conflict: Conflict | None = None
        for agent_1, agent_2 in combinations(
            solution.keys(),
            2,
        ):
            conflict = self._first_vertex_interval_conflict(
                vertex_intervals[agent_1],
                vertex_intervals[agent_2],
            )
            first_conflict = self._earlier_conflict(
                first_conflict,
                conflict,
            )

            conflict = self._first_edge_interval_conflict(
                edge_intervals[agent_1],
                edge_intervals[agent_2],
            )
            first_conflict = self._earlier_conflict(
                first_conflict,
                conflict,
            )

            if resource_intervals:
                conflict = self._first_resource_interval_conflict(
                    resource_intervals[agent_1],
                    resource_intervals[agent_2],
                )
                first_conflict = self._earlier_conflict(
                    first_conflict,
                    conflict,
                )
        return first_conflict

    def constraints_from_conflict(
        self,
        conflict: Conflict,
    ) -> dict[str, Constraints]:
        if conflict.type == Conflict.VERTEX:
            assert conflict.node_1 is not None
            constraint = Constraints()
            constraint.vertex_interval_constraints.add(
                VertexIntervalConstraint(
                    start_time=conflict.time,
                    end_time=max(
                        conflict.time,
                        conflict.end_time,
                    ),
                    node=conflict.node_1,
                )
            )
            return {
                conflict.agent_1: constraint,
                conflict.agent_2: constraint,
            }

        if conflict.type == Conflict.RESOURCE:
            return self._resource_constraints(conflict)
        return self._edge_constraints(conflict)

    def _path_resource_intervals(
        self,
        agent_name: str,
        path: list[State],
        horizon: int,
    ) -> list[PathResourceInterval]:
        if (
            not path
            or self.vertex_resources_fn is None
            or self.lane_resources_fn is None
        ):
            return []

        intervals: set[PathResourceInterval] = set()

        def add_vertex(
            node: NodeName,
            start: int,
            end: int,
            *,
            rotation: bool = False,
        ) -> None:
            resource_fn = (
                self.rotation_resources_fn
                if (
                    rotation
                    and self.rotation_resources_fn is not None
                )
                else self.vertex_resources_fn
            )
            if resource_fn is None:
                return
            for resource in resource_fn(node):
                intervals.add(
                    PathResourceInterval(
                        agent_name,
                        start,
                        max(start + 1, end),
                        resource,
                        "vertex",
                        node=node,
                    )
                )

        for state in path:
            add_vertex(
                state.node,
                state.time,
                state.time + 1,
            )

        for start, end in zip(path, path[1:]):
            if start.node == end.node:
                add_vertex(
                    start.node,
                    start.time,
                    end.time + 1,
                )
                continue

            turn_ticks, _ = self.transition_parts(start, end)
            move_start = start.time + turn_ticks
            if turn_ticks:
                add_vertex(
                    start.node,
                    start.time,
                    move_start + 1,
                    rotation=True,
                )
            for resource in self.lane_resources_fn(
                start.node,
                end.node,
            ):
                intervals.add(
                    PathResourceInterval(
                        agent_name,
                        move_start,
                        max(move_start + 1, end.time),
                        resource,
                        "edge",
                        from_node=start.node,
                        to_node=end.node,
                    )
                )

        final = path[-1]
        add_vertex(
            final.node,
            final.time,
            horizon + 1,
        )
        return sorted(
            intervals,
            key=lambda item: (
                item.start_time,
                item.end_time,
                str(item.resource),
                item.kind,
            ),
        )

    def _first_resource_interval_conflict(
        self,
        first: list[PathResourceInterval],
        second: list[PathResourceInterval],
    ) -> Conflict | None:
        best: Conflict | None = None
        second_by_resource: dict[
            object,
            list[PathResourceInterval],
        ] = {}
        for interval in second:
            second_by_resource.setdefault(
                interval.resource,
                [],
            ).append(interval)

        for interval_1 in first:
            for interval_2 in second_by_resource.get(
                interval_1.resource,
                [],
            ):
                span_1 = (
                    interval_1.start_time,
                    interval_1.end_time,
                )
                span_2 = (
                    interval_2.start_time,
                    interval_2.end_time,
                )
                entry_1: NodeName | None = None
                entry_2: NodeName | None = None
                if (
                    getattr(
                        interval_1.resource,
                        "kind",
                        "",
                    )
                    == "controlled_region"
                ):
                    span_1 = self._resource_occupation_span(
                        first,
                        interval_1,
                    )
                    span_2 = self._resource_occupation_span(
                        second,
                        interval_2,
                    )
                    entry_1 = self._resource_entry_node(
                        first,
                        interval_1.resource,
                        span_1,
                    )
                    entry_2 = self._resource_entry_node(
                        second,
                        interval_2.resource,
                        span_2,
                    )

                start = max(span_1[0], span_2[0])
                end = min(span_1[1], span_2[1])
                if start >= end:
                    continue

                conflict = Conflict(
                    time=start,
                    end_time=end - 1,
                    type=Conflict.RESOURCE,
                    agent_1=interval_1.agent,
                    agent_2=interval_2.agent,
                    node_1=interval_1.node,
                    node_2=interval_2.node,
                    agent_1_from=interval_1.from_node,
                    agent_1_to=interval_1.to_node,
                    agent_2_from=interval_2.from_node,
                    agent_2_to=interval_2.to_node,
                    agent_1_resource_kind=interval_1.kind,
                    agent_2_resource_kind=interval_2.kind,
                    agent_1_resource_start=span_1[0],
                    agent_1_resource_end=span_1[1] - 1,
                    agent_2_resource_start=span_2[0],
                    agent_2_resource_end=span_2[1] - 1,
                    agent_1_resource_entry=entry_1,
                    agent_2_resource_entry=entry_2,
                    resource=interval_1.resource,
                )
                best = self._earlier_conflict(
                    best,
                    conflict,
                )
        return best

    def _resource_occupation_span(
        self,
        intervals: list[PathResourceInterval],
        seed: PathResourceInterval,
    ) -> tuple[int, int]:
        """Merge touching reservations for one controlled region."""

        start = seed.start_time
        end = seed.end_time
        changed = True
        while changed:
            changed = False
            for candidate in intervals:
                if candidate.resource != seed.resource:
                    continue
                if (
                    candidate.start_time > end
                    or candidate.end_time < start
                ):
                    continue
                next_start = min(start, candidate.start_time)
                next_end = max(end, candidate.end_time)
                if next_start != start or next_end != end:
                    start = next_start
                    end = next_end
                    changed = True
        return start, end

    @staticmethod
    def _resource_entry_node(
        intervals: list[PathResourceInterval],
        resource: object,
        span: tuple[int, int],
    ) -> NodeName | None:
        edge_intervals = sorted(
            (
                interval
                for interval in intervals
                if interval.resource == resource
                and interval.kind == "edge"
                and interval.from_node
                and interval.start_time >= span[0]
                and interval.end_time <= span[1]
            ),
            key=lambda interval: (
                interval.start_time,
                interval.end_time,
            ),
        )
        return (
            edge_intervals[0].from_node
            if edge_intervals
            else None
        )

    def _path_intervals(
        self,
        agent_name: str,
        path: list[State],
        horizon: int,
    ) -> tuple[
        list[PathVertexInterval],
        list[PathEdgeInterval],
    ]:
        if not path:
            return [], []

        vertex_intervals: list[PathVertexInterval] = []
        edge_intervals: list[PathEdgeInterval] = []
        for state in path:
            vertex_intervals.append(
                PathVertexInterval(
                    agent_name,
                    state.time,
                    state.time,
                    state.node,
                )
            )

        for start, end in zip(path, path[1:]):
            if start.node == end.node:
                vertex_intervals.append(
                    PathVertexInterval(
                        agent_name,
                        start.time,
                        end.time,
                        start.node,
                    )
                )
                continue

            turn_ticks, _ = self.transition_parts(start, end)
            if turn_ticks:
                vertex_intervals.append(
                    PathVertexInterval(
                        agent_name,
                        start.time,
                        start.time + turn_ticks,
                        start.node,
                    )
                )
            edge_intervals.append(
                PathEdgeInterval(
                    agent=agent_name,
                    start_time=start.time + turn_ticks,
                    end_time=end.time,
                    from_node=start.node,
                    to_node=end.node,
                )
            )

        final = path[-1]
        if final.time <= horizon:
            vertex_intervals.append(
                PathVertexInterval(
                    agent_name,
                    final.time,
                    horizon,
                    final.node,
                )
            )
        return vertex_intervals, edge_intervals

    def _first_vertex_interval_conflict(
        self,
        first: list[PathVertexInterval],
        second: list[PathVertexInterval],
    ) -> Conflict | None:
        best: Conflict | None = None
        for interval_1 in first:
            for interval_2 in second:
                if interval_1.node != interval_2.node:
                    continue
                if not self._intervals_overlap(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                ):
                    continue
                start_time = self._interval_overlap_start(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                )
                end_time = min(
                    interval_1.end_time,
                    interval_2.end_time,
                )
                conflict = Conflict(
                    time=start_time,
                    end_time=end_time,
                    type=Conflict.VERTEX,
                    agent_1=interval_1.agent,
                    agent_2=interval_2.agent,
                    node_1=interval_1.node,
                )
                best = self._earlier_conflict(
                    best,
                    conflict,
                )
        return best

    def _first_edge_interval_conflict(
        self,
        first: list[PathEdgeInterval],
        second: list[PathEdgeInterval],
    ) -> Conflict | None:
        best: Conflict | None = None
        for interval_1 in first:
            for interval_2 in second:
                same_edge = (
                    interval_1.from_node
                    == interval_2.from_node
                    and interval_1.to_node
                    == interval_2.to_node
                )
                reverse_edge = (
                    interval_1.from_node
                    == interval_2.to_node
                    and interval_1.to_node
                    == interval_2.from_node
                )
                if not same_edge and not reverse_edge:
                    continue
                if not self._intervals_overlap(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                ):
                    continue
                start_time = self._interval_overlap_start(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                )
                end_time = min(
                    interval_1.end_time,
                    interval_2.end_time,
                )
                conflict = Conflict(
                    time=start_time,
                    end_time=end_time,
                    type=Conflict.EDGE,
                    agent_1=interval_1.agent,
                    agent_2=interval_2.agent,
                    node_1=interval_1.from_node,
                    node_2=interval_1.to_node,
                    agent_1_from=interval_1.from_node,
                    agent_1_to=interval_1.to_node,
                    agent_1_time=interval_1.start_time,
                    agent_2_from=interval_2.from_node,
                    agent_2_to=interval_2.to_node,
                    agent_2_time=interval_2.start_time,
                )
                best = self._earlier_conflict(
                    best,
                    conflict,
                )
        return best

    @staticmethod
    def _earlier_conflict(
        current: Conflict | None,
        candidate: Conflict | None,
    ) -> Conflict | None:
        if candidate is None:
            return current
        if current is None:
            return candidate
        if candidate.time != current.time:
            return (
                candidate
                if candidate.time < current.time
                else current
            )
        return (
            candidate
            if candidate.type < current.type
            else current
        )

    @staticmethod
    def _intervals_overlap(
        start_a: int,
        end_a: int,
        start_b: int,
        end_b: int,
    ) -> bool:
        return closed_intervals_overlap(start_a, end_a, start_b, end_b)

    @staticmethod
    def _interval_overlap_start(
        start_a: int,
        _end_a: int,
        start_b: int,
        _end_b: int,
    ) -> int:
        return max(start_a, start_b)

    def _resource_constraints(
        self,
        conflict: Conflict,
    ) -> dict[str, Constraints]:
        assert conflict.resource is not None
        constraint_1 = Constraints()
        constraint_2 = Constraints()
        controlled = (
            getattr(conflict.resource, "kind", "")
            == "controlled_region"
        )
        constraint_1.resource_interval_constraints.add(
            ResourceIntervalConstraint(
                start_time=(
                    0
                    if (
                        controlled
                        and conflict.agent_2_resource_start >= 0
                    )
                    else conflict.time
                ),
                end_time=(
                    conflict.agent_2_resource_end
                    if (
                        controlled
                        and conflict.agent_2_resource_end >= 0
                    )
                    else max(
                        conflict.time,
                        conflict.end_time,
                    )
                ),
                resource=conflict.resource,
            )
        )
        constraint_2.resource_interval_constraints.add(
            ResourceIntervalConstraint(
                start_time=(
                    0
                    if (
                        controlled
                        and conflict.agent_1_resource_start >= 0
                    )
                    else conflict.time
                ),
                end_time=(
                    conflict.agent_1_resource_end
                    if (
                        controlled
                        and conflict.agent_1_resource_end >= 0
                    )
                    else max(
                        conflict.time,
                        conflict.end_time,
                    )
                ),
                resource=conflict.resource,
            )
        )

        if (
            controlled
            and conflict.agent_1_resource_entry
        ):
            constraint_1.vertex_interval_constraints.add(
                VertexIntervalConstraint(
                    start_time=0,
                    end_time=max(
                        0,
                        conflict.agent_2_resource_end + 1,
                    ),
                    node=conflict.agent_1_resource_entry,
                )
            )
        if (
            controlled
            and conflict.agent_2_resource_entry
        ):
            constraint_2.vertex_interval_constraints.add(
                VertexIntervalConstraint(
                    start_time=0,
                    end_time=max(
                        0,
                        conflict.agent_1_resource_end + 1,
                    ),
                    node=conflict.agent_2_resource_entry,
                )
            )
        return {
            conflict.agent_1: constraint_1,
            conflict.agent_2: constraint_2,
        }

    @staticmethod
    def _edge_constraints(
        conflict: Conflict,
    ) -> dict[str, Constraints]:
        assert conflict.node_1 is not None
        assert conflict.node_2 is not None
        constraint_1 = Constraints()
        constraint_2 = Constraints()
        agent_1_from = (
            conflict.agent_1_from or conflict.node_1
        )
        agent_1_to = (
            conflict.agent_1_to or conflict.node_2
        )
        agent_2_from = (
            conflict.agent_2_from or conflict.node_2
        )
        agent_2_to = (
            conflict.agent_2_to or conflict.node_1
        )
        constraint_1.edge_interval_constraints.add(
            EdgeIntervalConstraint(
                start_time=conflict.time,
                end_time=max(
                    conflict.time,
                    conflict.end_time,
                ),
                from_node=agent_1_from,
                to_node=agent_1_to,
            )
        )
        constraint_2.edge_interval_constraints.add(
            EdgeIntervalConstraint(
                start_time=conflict.time,
                end_time=max(
                    conflict.time,
                    conflict.end_time,
                ),
                from_node=agent_2_from,
                to_node=agent_2_to,
            )
        )
        return {
            conflict.agent_1: constraint_1,
            conflict.agent_2: constraint_2,
        }
