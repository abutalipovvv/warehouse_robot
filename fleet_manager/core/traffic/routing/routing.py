"""Compatibility facade for runtime traffic routing."""

from __future__ import annotations

from fleet_manager.core.traffic.corridors.admission.controlled_corridor_admission import (
    ControlledCorridorAdmissionMixin,
)
from fleet_manager.core.traffic.corridors.controlled_corridor_passage import (
    ControlledCorridorPassageMixin,
)
from fleet_manager.core.traffic.corridors.prefetch.controlled_corridor_prefetch import (
    ControlledCorridorPrefetchMixin,
)
from fleet_manager.core.traffic.routing.rolling_route_helpers import RollingRouteMixin
from fleet_manager.core.traffic.routing.spatial_detours import SpatialDetourMixin
from fleet_manager.core.traffic.routing.traffic_zone_admission import (
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
