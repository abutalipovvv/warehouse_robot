"""Mutable analysis state for one deadlock evacuation candidate search."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from fleet_manager.core.models import FleetRobot


@dataclass(frozen=True, slots=True)
class _EvacuationSearchContext:
    """Fleet component and portal facts shared by every candidate."""

    robots: list[FleetRobot]
    winner: FleetRobot
    winner_regions: set[str]
    portal_queue_depths: dict[str, int]


@dataclass(slots=True)
class _EvacuationCandidateProbe:
    """Progressive geometric analysis for one possible evacuating robot."""

    robot: FleetRobot
    target_clock: float
    target_lm: str
    portal_queue_depth: int
    robot_regions: set[str]
    upcoming: dict[str, Any] | None
    retreats_from_occupied_portal: bool
    retreat_is_noop_at_current_lm: bool
    reciprocal_blocker: bool
    graph_escape_required: bool
    historical_retreat_blocker: str = ""
    graph_escape_route: list[str] = field(default_factory=list)
    portal_blocked_edges: list[tuple[str, str]] = field(
        default_factory=list
    )


__all__ = [
    "_EvacuationCandidateProbe",
    "_EvacuationSearchContext",
]
