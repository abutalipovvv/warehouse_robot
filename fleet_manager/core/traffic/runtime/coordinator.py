"""Compatibility facade for runtime traffic coordination."""

from __future__ import annotations

from fleet_manager.core.traffic.deadlocks.arbitration.deadlock_arbitration import (
    DeadlockArbitrationMixin,
)
from fleet_manager.core.traffic.deadlocks.recovery.evacuation.deadlock_evacuation import (
    DeadlockEvacuationMixin,
)
from fleet_manager.core.traffic.runtime.runtime_conflicts import RuntimeConflictMixin


class TrafficCoordinatorMixin(
    DeadlockArbitrationMixin,
    DeadlockEvacuationMixin,
    RuntimeConflictMixin,
):
    """Preserve the manager mixin API while composing focused components."""
