"""Internal value objects for one corridor-calendar build."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import NamedTuple

from fleet_manager.core.traffic.corridor_calendar import CorridorCalendar
from fleet_manager.core.traffic.corridor_models import (
    CorridorDecision,
    CorridorOccupancy,
    CorridorRequest,
    CorridorSchedulerConfig,
    CorridorSlot,
)


@dataclass(slots=True)
class _PlanningContext:
    """Mutable state owned by one side-effect-free build call."""

    policy: CorridorSchedulerConfig
    calendar: CorridorCalendar
    now: float
    horizon_end: float
    explicit_regions: frozenset[str]
    request_by_robot: dict[str, CorridorRequest]
    occupancy_snapshot_provided: bool
    occupancy_by_robot: dict[str, CorridorOccupancy]
    pinned_by_robot: dict[str, CorridorSlot]
    physically_occupied_robots: frozenset[str]
    slots: list[CorridorSlot] = field(default_factory=list)
    decisions: dict[str, CorridorDecision] = field(default_factory=dict)
    cleared_passage_robots: set[str] = field(default_factory=set)


class _PlacementProposal(NamedTuple):
    """One schedulable request and its exact deterministic rank."""

    rank: tuple[object, ...]
    request: CorridorRequest
    placement: tuple[float, float, int]


__all__ = [
    "_PlacementProposal",
    "_PlanningContext",
]
