"""Small typed values passed between corridor-prefetch stages."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from fleet_manager.core.traffic.corridors.scheduling.corridor_scheduler import (
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSlot,
)


@dataclass(frozen=True, slots=True)
class _CorridorRouteDraft:
    """Nominal route and pose prepared before passage discovery."""

    route_nodes: list[str]
    start_lm: str
    trajectory: list[dict[str, Any]]
    pose: dict[str, float]


@dataclass(frozen=True, slots=True)
class _CorridorIntentDraft:
    """First authored passage and its future calendar timing."""

    route: _CorridorRouteDraft
    entry: dict[str, Any]
    regions: tuple[str, ...]
    direction: str
    staging_lm: str
    exit_lm: str
    staging_clock: float
    exit_clock: float
    kind: str
    signature: tuple[Any, ...]
    handoff_at: float
    earliest_entry: float


@dataclass(frozen=True, slots=True)
class _CorridorValidationContext:
    """Current intent and slot captured before MAPF result inspection."""

    intent: dict[str, Any]
    request: CorridorRequest
    slot: CorridorSlot


@dataclass(frozen=True, slots=True)
class _CorridorPlannedPassage:
    """Authored passage reconstructed from one completed MAPF result."""

    entry: dict[str, Any]
    regions: tuple[str, ...]
    resource_windows: tuple[CorridorResourceWindow, ...]
    actual_staging_at: float
    staging_clock: float
    exit_clock: float
