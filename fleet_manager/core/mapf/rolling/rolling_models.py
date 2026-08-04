"""Small data objects shared by rolling SIPP components."""

from __future__ import annotations

from dataclasses import dataclass

from ..cbs.lm_cbs import NodeName


VertexConstraintInput = tuple[int, NodeName]
EdgeConstraintInput = tuple[int, NodeName, NodeName]
VertexIntervalInput = tuple[int, int, NodeName, str]
EdgeIntervalInput = tuple[
    int,
    int,
    NodeName,
    NodeName,
    str,
]


@dataclass(frozen=True, slots=True)
class StaticReservations:
    """External constraints copied into each planning attempt."""

    vertex_constraints: tuple[VertexConstraintInput, ...] = ()
    edge_constraints: tuple[EdgeConstraintInput, ...] = ()
    vertex_intervals: tuple[VertexIntervalInput, ...] = ()
    edge_intervals: tuple[EdgeIntervalInput, ...] = ()


@dataclass(slots=True)
class RollingPlanningMetrics:
    expanded_nodes: int = 0
    staging_wait_repairs: int = 0
