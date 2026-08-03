"""Decision context for one runtime wait-cycle arbitration."""

from __future__ import annotations

from dataclasses import dataclass

from fleet_manager.core.domain.models import FleetRobot


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


__all__ = ["_WaitCycleDecisionContext"]
