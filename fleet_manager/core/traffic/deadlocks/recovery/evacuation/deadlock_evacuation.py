"""Compatibility facade for graph-safe deadlock evacuation."""

from __future__ import annotations

from fleet_manager.core.traffic.deadlock_escape_install import (
    GraphEscapeInstallMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation_activation import (
    EvacuationActivationMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation_candidates import (
    EvacuationCandidateMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation_geometry import (
    EvacuationGeometryMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation_latches import (
    EvacuationLatchMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation_models import (
    _EvacuationCandidate,
)


class DeadlockEvacuationMixin(
    EvacuationGeometryMixin,
    EvacuationLatchMixin,
    EvacuationCandidateMixin,
    EvacuationActivationMixin,
    GraphEscapeInstallMixin,
):
    """Compose evacuation geometry, candidates, activation and installation."""


__all__ = ["DeadlockEvacuationMixin"]
