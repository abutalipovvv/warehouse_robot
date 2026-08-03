"""Compatibility facade for runtime wait-cycle arbitration."""

from __future__ import annotations

from fleet_manager.core.traffic.deadlock_corridor_ownership import (
    CorridorOwnershipMixin,
)
from fleet_manager.core.traffic.deadlock_cycle_recovery import (
    WaitCycleRecoveryMixin,
)
from fleet_manager.core.traffic.deadlock_leases import DeadlockLeaseMixin
from fleet_manager.core.traffic.deadlock_policy import DeadlockPolicyMixin
from fleet_manager.core.traffic.deadlock_priority import DeadlockPriorityMixin
from fleet_manager.core.traffic.deadlock_wait_detection import (
    WaitCycleDetectionMixin,
    _RuntimeWaitSnapshot,
)


class DeadlockArbitrationMixin(
    WaitCycleDetectionMixin,
    DeadlockPriorityMixin,
    WaitCycleRecoveryMixin,
    CorridorOwnershipMixin,
    DeadlockLeaseMixin,
    DeadlockPolicyMixin,
):
    """Compose detection, priority, corridor, lease and policy capabilities."""


__all__ = ["DeadlockArbitrationMixin"]
