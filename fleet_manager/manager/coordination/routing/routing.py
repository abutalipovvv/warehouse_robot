"""Composition of runtime traffic routing capabilities."""

from __future__ import annotations

from fleet_manager.manager.coordination.corridors.admission import (
    ControlledCorridorAdmissionDecisionMixin,
)
from fleet_manager.manager.coordination.corridors.passage import (
    ControlledCorridorPassageMixin,
)
from fleet_manager.manager.coordination.corridors.intent import (
    ControlledCorridorPrefetchIntentMixin,
)
from fleet_manager.manager.coordination.corridors.prefetch import (
    ControlledCorridorPrefetchGateMixin,
)
from fleet_manager.manager.coordination.corridors.publication import (
    ControlledCorridorRuntimePublicationMixin,
)
from fleet_manager.manager.coordination.corridors.requests import (
    ControlledCorridorRequestCollectionMixin,
)
from fleet_manager.manager.coordination.corridors.validation import (
    ControlledCorridorPrefetchValidationMixin,
)
from fleet_manager.manager.coordination.routing.rolling import RollingRouteMixin
from fleet_manager.manager.coordination.routing.spatial import SpatialDetourMixin
from fleet_manager.manager.coordination.zones import (
    TrafficZoneAdmissionMixin,
)


class TrafficRoutingMixin(
    SpatialDetourMixin,
    ControlledCorridorPassageMixin,
    ControlledCorridorPrefetchGateMixin,
    ControlledCorridorPrefetchIntentMixin,
    ControlledCorridorPrefetchValidationMixin,
    ControlledCorridorRuntimePublicationMixin,
    ControlledCorridorRequestCollectionMixin,
    ControlledCorridorAdmissionDecisionMixin,
    TrafficZoneAdmissionMixin,
    RollingRouteMixin,
):
    """Preserve routing hooks while composing focused traffic components."""
