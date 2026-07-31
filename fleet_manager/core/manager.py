"""Readable composition root for fleet runtime capabilities."""

from __future__ import annotations

from time import time

from fleet_manager.core.geometry.collision import FleetCollisionChecker
from fleet_manager.core.manager_commands import FleetManagerCommandMixin
from fleet_manager.core.manager_remote import FleetManagerRemoteControlMixin
from fleet_manager.core.manager_robots import FleetManagerRobotLifecycleMixin
from fleet_manager.core.manager_routes import FleetManagerRouteMetadataMixin
from fleet_manager.core.manager_snapshots import FleetManagerSnapshotMixin
from fleet_manager.core.manager_state import FleetManagerRuntimeStateMixin
from fleet_manager.core.models import FleetEvent, FleetOrder, FleetRobot
from fleet_manager.core.motion import FleetMotionRuntimeMixin
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
