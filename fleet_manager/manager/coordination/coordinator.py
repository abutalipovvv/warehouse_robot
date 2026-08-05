"""Composition of runtime traffic coordination capabilities."""

from __future__ import annotations

from fleet_manager.manager.coordination.deadlocks.corridor_ownership import (
    CorridorOwnershipMixin,
)
from fleet_manager.manager.coordination.deadlocks.detection import (
    WaitCycleDetectionMixin,
)
from fleet_manager.manager.coordination.deadlocks.escape_install import (
    GraphEscapeInstallMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation import (
    EvacuationActivationMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation_candidates import (
    EvacuationCandidateMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation_geometry import (
    EvacuationGeometryMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation_latches import (
    EvacuationLatchMixin,
)
from fleet_manager.manager.coordination.deadlocks.leases import (
    DeadlockLeaseMixin,
)
from fleet_manager.manager.coordination.deadlocks.policy import (
    DeadlockPolicyMixin,
)
from fleet_manager.manager.coordination.deadlocks.priority import (
    DeadlockPriorityMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery import (
    WaitCycleRecoveryMixin,
)
from fleet_manager.manager.coordination.runtime_conflicts import RuntimeConflictMixin


class TrafficCoordinatorMixin(
    WaitCycleDetectionMixin,
    DeadlockPriorityMixin,
    WaitCycleRecoveryMixin,
    CorridorOwnershipMixin,
    DeadlockLeaseMixin,
    DeadlockPolicyMixin,
    EvacuationGeometryMixin,
    EvacuationLatchMixin,
    EvacuationCandidateMixin,
    EvacuationActivationMixin,
    GraphEscapeInstallMixin,
    RuntimeConflictMixin,
):
    """Preserve the manager mixin API while composing focused components."""
