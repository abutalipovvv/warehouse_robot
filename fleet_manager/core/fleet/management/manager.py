"""Readable composition root for fleet runtime capabilities."""

from __future__ import annotations

from time import time as _system_time

from fleet_manager.core.fleet.safety.collision import FleetCollisionChecker
from fleet_manager.core.fleet.management.commands import FleetManagerCommandMixin
from fleet_manager.core.fleet.management.remote_control import (
    FleetManagerRemoteControlMixin,
)
from fleet_manager.core.fleet.management.robot_lifecycle import (
    FleetManagerRobotLifecycleMixin,
)
from fleet_manager.core.fleet.management.route_metadata import (
    FleetManagerRouteMetadataMixin,
)
from fleet_manager.core.fleet.management.snapshots import FleetManagerSnapshotMixin
from fleet_manager.core.fleet.management.state import FleetManagerRuntimeStateMixin
from fleet_manager.core.fleet.domain.models import FleetEvent, FleetOrder, FleetRobot
from fleet_manager.core.fleet.movement.motion import FleetMotionRuntimeMixin
from fleet_manager.core.tasks.dispatch import FleetTaskDispatchMixin
from fleet_manager.core.traffic.runtime.coordinator import TrafficCoordinatorMixin
from fleet_manager.core.traffic.planning.planning import TrafficPlanningMixin
from fleet_manager.core.traffic.routing.routing import TrafficRoutingMixin


# Tests and embedded runtimes may replace this clock before manager creation.
time = _system_time


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
