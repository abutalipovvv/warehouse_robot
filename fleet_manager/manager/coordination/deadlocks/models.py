"""Decision context for one runtime wait-cycle arbitration."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from fleet_manager.robot.model import FleetRobot


@dataclass(frozen=True, slots=True)
class _WaitCycleDecisionContext:
    """Stable facts shared by recovery escalation and lease installation."""

    robots: list[FleetRobot]
    now: float
    new_episode: bool
    corridor_owner: FleetRobot | None
    winner: FleetRobot
    corridor_handoff_required: bool
    cycle_key: tuple[str, ...]
    grant_already_failed: bool
    cycle_wait: float


@dataclass(frozen=True, order=True, slots=True)
class _EvacuationCandidate:
    """One safe recovery option, ordered by cost and robot identity."""

    distance: float
    priority: int
    robot_name: str
    robot: FleetRobot = field(compare=False)
    target_clock: float = field(compare=False)
    target_lm: str = field(compare=False)
    retreat_is_noop_at_current_lm: bool = field(compare=False)
    graph_escape_route: list[str] = field(compare=False)
    portal_blocked_edges: list[tuple[str, str]] = field(compare=False)


@dataclass(frozen=True, slots=True)
class _EvacuationSearchContext:
    """Fleet component and portal facts shared by every candidate."""

    robots: list[FleetRobot]
    winner: FleetRobot
    winner_regions: set[str]
    portal_queue_depths: dict[str, int]


@dataclass(slots=True)
class _EvacuationCandidateProbe:
    """Progressive analysis for one possible evacuating robot."""

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
