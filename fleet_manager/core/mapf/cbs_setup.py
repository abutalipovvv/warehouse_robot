"""Validated inputs for Conflict-Based Search.

The public planner accepts lists because they are convenient at integration
boundaries.  CBS itself works with typed limits and normalized constraints so
that the search loop only has to deal with the algorithm.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass

from .cbs_models import (
    EdgeConstraint,
    EdgeIntervalConstraint,
    NodeName,
    ResourceIntervalConstraint,
    VertexConstraint,
    VertexIntervalConstraint,
)


Clock = Callable[[], float]


@dataclass(frozen=True, slots=True)
class CbsPlanningLimits:
    """The bounded resources available to one CBS invocation."""

    low_level_max_time: int
    high_level_max_nodes: int
    time_budget_seconds: float
    started_at: float
    clock: Clock

    @property
    def deadline(self) -> float:
        return self.started_at + self.time_budget_seconds

    @property
    def timeout_reason(self) -> str:
        return f"planning_timeout:{self.time_budget_seconds:.3f}s"

    def timed_out(self) -> bool:
        return (
            self.clock() - self.started_at
            >= self.time_budget_seconds
        )


@dataclass(frozen=True, slots=True)
class CbsGlobalReservations:
    """Constraints contributed by plans outside the local CBS component."""

    vertices: frozenset[VertexConstraint]
    edges: frozenset[EdgeConstraint]
    vertex_intervals: tuple[VertexIntervalConstraint, ...]
    edge_intervals: tuple[EdgeIntervalConstraint, ...]
    resource_intervals: tuple[ResourceIntervalConstraint, ...]

    @classmethod
    def from_raw(
        cls,
        *,
        vertices: Iterable[tuple[int, NodeName]] = (),
        edges: Iterable[tuple[int, NodeName, NodeName]] = (),
        vertex_intervals: Iterable[
            tuple[int, int, NodeName, str]
        ] = (),
        edge_intervals: Iterable[
            tuple[int, int, NodeName, NodeName, str]
        ] = (),
        resource_intervals: Iterable[
            tuple[int, int, object]
        ] = (),
    ) -> CbsGlobalReservations:
        return cls(
            vertices=frozenset(
                VertexConstraint(time=time_tick, node=node)
                for time_tick, node in vertices
            ),
            edges=frozenset(
                EdgeConstraint(
                    time=time_tick,
                    from_node=source,
                    to_node=target,
                )
                for time_tick, source, target in edges
            ),
            vertex_intervals=tuple(
                VertexIntervalConstraint(
                    start_time=max(0, int(start)),
                    end_time=max(0, int(end)),
                    node=node,
                    owner=owner,
                )
                for start, end, node, owner in vertex_intervals
                if node
            ),
            edge_intervals=tuple(
                EdgeIntervalConstraint(
                    start_time=max(0, int(start)),
                    end_time=max(0, int(end)),
                    from_node=source,
                    to_node=target,
                    owner=owner,
                )
                for start, end, source, target, owner in edge_intervals
                if source and target
            ),
            resource_intervals=tuple(
                ResourceIntervalConstraint(
                    start_time=max(0, int(start)),
                    end_time=max(0, int(end)),
                    resource=resource,
                )
                for start, end, resource in resource_intervals
                if resource is not None
            ),
        )


__all__ = ["CbsGlobalReservations", "CbsPlanningLimits", "Clock"]
