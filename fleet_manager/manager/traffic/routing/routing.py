"""Composition of runtime traffic routing capabilities."""

from __future__ import annotations

from fleet_manager.manager.traffic.corridors.admission.controlled_corridor_admission import (
    ControlledCorridorAdmissionMixin,
)
from fleet_manager.manager.traffic.corridors.passage import (
    ControlledCorridorPassageMixin,
)
from fleet_manager.manager.traffic.corridors.prefetch.controlled_corridor_prefetch import (
    ControlledCorridorPrefetchMixin,
)
from fleet_manager.manager.traffic.routing.rolling_route_helpers import RollingRouteMixin
from fleet_manager.manager.traffic.routing.spatial_detours import SpatialDetourMixin
from fleet_manager.manager.traffic.zones import (
    TrafficZoneAdmissionMixin,
)


class TrafficRoutingMixin(
    SpatialDetourMixin,
    ControlledCorridorPassageMixin,
    ControlledCorridorPrefetchMixin,
    ControlledCorridorAdmissionMixin,
    TrafficZoneAdmissionMixin,
    RollingRouteMixin,
):
    """Preserve routing hooks while composing focused traffic components."""
