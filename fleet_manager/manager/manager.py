"""Readable composition root for fleet runtime capabilities."""

from __future__ import annotations

from time import time as _system_time

from fleet_manager.manager.commands import FleetManagerCommandMixin
from fleet_manager.manager.remote_control import (
    FleetManagerRemoteControlMixin,
)
from fleet_manager.manager.robot_lifecycle import (
    FleetManagerRobotLifecycleMixin,
)
from fleet_manager.manager.route_metadata import (
    FleetManagerRouteMetadataMixin,
)
from fleet_manager.manager.snapshots import FleetManagerSnapshotMixin
from fleet_manager.manager.runtime_state import FleetManagerRuntimeStateMixin
from fleet_manager.manager.movement.motion import FleetMotionRuntimeMixin
from fleet_manager.manager.tasks.dispatch import FleetTaskDispatchMixin
from fleet_manager.manager.traffic.coordinator import TrafficCoordinatorMixin
from fleet_manager.manager.traffic.planning.planning import TrafficPlanningMixin
from fleet_manager.manager.traffic.routing.routing import TrafficRoutingMixin


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

    def _wall_time(self) -> float:
        """Return the injectable wall clock used by manager state."""

        return float(time())
