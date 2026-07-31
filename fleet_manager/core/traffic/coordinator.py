"""Compatibility facade for runtime traffic coordination."""

from __future__ import annotations

from fleet_manager.core.traffic.deadlock_arbitration import (
    DeadlockArbitrationMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation import (
    DeadlockEvacuationMixin,
)
from fleet_manager.core.traffic.runtime_conflicts import RuntimeConflictMixin


class TrafficCoordinatorMixin(
    DeadlockArbitrationMixin,
    DeadlockEvacuationMixin,
    RuntimeConflictMixin,
):
    """Preserve the manager mixin API while composing focused components."""


__all__ = ["TrafficCoordinatorMixin"]
