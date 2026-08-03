"""State accumulated during one central admission snapshot."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from fleet_manager.core.traffic.corridor_scheduler import (
    CorridorRequest,
)


@dataclass(slots=True)
class _CentralCorridorBuild:
    """Mutable data produced during one central admission snapshot."""

    now: float
    scheduler: Any
    old_schedule: Any | None
    physical_by_robot: dict[str, set[str]] = field(default_factory=dict)
    occupancy_by_region: dict[str, set[str]] = field(default_factory=dict)
    requests: list[CorridorRequest] = field(default_factory=list)
    entries_by_robot: dict[str, dict[str, Any]] = field(default_factory=dict)
    scheduled_intent_names: set[str] = field(default_factory=set)
    active_wait_keys: set[tuple[str, str, int, str]] = field(default_factory=set)
    downstream_blockers: dict[str, str] = field(default_factory=dict)
    starvation: float = 0.0


@dataclass(frozen=True, slots=True)
class _CentralCorridorWaitContext:
    """Validated calendar state for one external wait decision."""

    schedule: Any
    entry: dict[str, Any]
    regions: tuple[str, ...]
    direction: str
    admission_wait: bool
    decision: Any
    blocker_name: str


@dataclass(slots=True)
class _CentralCorridorPublication:
    """Mutable runtime projection produced from one calendar."""

    immediate_window: float
    passages: dict[str, dict[str, Any]] = field(default_factory=dict)
    leases: dict[str, tuple[str, float]] = field(default_factory=dict)
    winners: dict[str, str] = field(default_factory=dict)
    queues: dict[str, list[tuple[float, str]]] = field(
        default_factory=dict
    )


__all__ = [
    "_CentralCorridorBuild",
    "_CentralCorridorPublication",
    "_CentralCorridorWaitContext",
]
