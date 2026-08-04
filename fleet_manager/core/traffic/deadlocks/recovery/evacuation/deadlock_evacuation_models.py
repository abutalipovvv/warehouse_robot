"""Immutable value objects used during deadlock evacuation."""

from __future__ import annotations

from dataclasses import dataclass, field

from fleet_manager.core.fleet.domain.models import FleetRobot


@dataclass(frozen=True, order=True, slots=True)
class _EvacuationCandidate:
    """One safe recovery option, ordered by cost then stable robot identity."""

    distance: float
    priority: int
    robot_name: str
    robot: FleetRobot = field(compare=False)
    target_clock: float = field(compare=False)
    target_lm: str = field(compare=False)
    retreat_is_noop_at_current_lm: bool = field(compare=False)
    graph_escape_route: list[str] = field(compare=False)
    portal_blocked_edges: list[tuple[str, str]] = field(compare=False)
