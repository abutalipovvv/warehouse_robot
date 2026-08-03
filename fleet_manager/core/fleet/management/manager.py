"""Readable composition root for fleet runtime capabilities."""

from __future__ import annotations

from fleet_manager.core.geometry.collision import FleetCollisionChecker
from fleet_manager.core.management.commands import FleetManagerCommandMixin
from fleet_manager.core.management.remote_control import (
    FleetManagerRemoteControlMixin,
)
from fleet_manager.core.management.robot_lifecycle import (
    FleetManagerRobotLifecycleMixin,
)
from fleet_manager.core.management.route_metadata import (
    FleetManagerRouteMetadataMixin,
)
from fleet_manager.core.management.snapshots import FleetManagerSnapshotMixin
from fleet_manager.core.management.state import FleetManagerRuntimeStateMixin
from fleet_manager.core.domain.models import FleetEvent, FleetOrder, FleetRobot
from fleet_manager.core.movement.motion import FleetMotionRuntimeMixin
from fleet_manager.core.tasks.dispatch import FleetTaskDispatchMixin
from fleet_manager.core.traffic.coordinator import TrafficCoordinatorMixin
from fleet_manager.core.traffic.planning import TrafficPlanningMixin
from fleet_manager.core.traffic.routing import TrafficRoutingMixin


class FleetManagerCore(
    FleetManagerRuntimeStateMixin,
    FleetManagerSnapshotMixin,
    FleetManagerCommandMixin,
    FleetManagerRobotLifecycleMixin,
    FleetManagerRemoteControlMixin,
    FleetManagerRouteMetadataMixin,
    FleetMotionRuntimeMixin,
    TrafficCoordinatorMixin,
    TrafficRoutingMixin,
    TrafficPlanningMixin,
    FleetTaskDispatchMixin,
):
    """Compose fleet state, commands, traffic policy and transport hooks."""

    MAX_SIMULATION_TIME_SCALE = 4.0
    runtime_kind = "core"


__all__ = [
    "FleetCollisionChecker",
    "FleetEvent",
    "FleetManagerCore",
    "FleetOrder",
    "FleetRobot",
]
