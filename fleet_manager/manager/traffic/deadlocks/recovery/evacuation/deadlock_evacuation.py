"""Composition of graph-safe deadlock evacuation stages."""

from __future__ import annotations

from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_escape_install import (
    GraphEscapeInstallMixin,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_activation import (
    EvacuationActivationMixin,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_candidates import (
    EvacuationCandidateMixin,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_geometry import (
    EvacuationGeometryMixin,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_latches import (
    EvacuationLatchMixin,
)
class DeadlockEvacuationMixin(
    EvacuationGeometryMixin,
    EvacuationLatchMixin,
    EvacuationCandidateMixin,
    EvacuationActivationMixin,
    GraphEscapeInstallMixin,
):
    """Compose evacuation geometry, candidates, activation and installation."""
