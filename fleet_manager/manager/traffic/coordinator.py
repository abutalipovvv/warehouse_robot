"""Composition of runtime traffic coordination capabilities."""

from __future__ import annotations

from fleet_manager.manager.traffic.deadlocks.arbitration.deadlock_arbitration import (
    DeadlockArbitrationMixin,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation import (
    DeadlockEvacuationMixin,
)
from fleet_manager.manager.traffic.runtime_conflicts import RuntimeConflictMixin


class TrafficCoordinatorMixin(
    DeadlockArbitrationMixin,
    DeadlockEvacuationMixin,
    RuntimeConflictMixin,
):
    """Preserve the manager mixin API while composing focused components."""
