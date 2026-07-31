"""Composition root for central controlled-corridor admission."""

from __future__ import annotations

from .controlled_corridor_admission_decisions import (
    ControlledCorridorAdmissionDecisionMixin,
)
from .controlled_corridor_admission_requests import (
    ControlledCorridorRequestCollectionMixin,
)
from .controlled_corridor_admission_runtime import (
    ControlledCorridorRuntimePublicationMixin,
)


class ControlledCorridorAdmissionMixin(
    ControlledCorridorRuntimePublicationMixin,
    ControlledCorridorRequestCollectionMixin,
    ControlledCorridorAdmissionDecisionMixin,
):
    """Compose request collection, admission decisions and publication."""


__all__ = ["ControlledCorridorAdmissionMixin"]
