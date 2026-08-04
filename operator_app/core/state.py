"""Composition root for operator application state capabilities."""

from __future__ import annotations

from .state_runtime import RuntimeOwnershipMixin
from .state_robot_registry import RobotRegistryProbeMixin
from .state_fleet_api import FleetApiRoutingMixin
from .state_fleet_maps import FleetMapSyncMixin
from .state_robot_maps import RobotMapSyncMixin
from .state_robot_control import RobotControlProxyMixin


class OperatorAppState(
    RuntimeOwnershipMixin,
    RobotRegistryProbeMixin,
    FleetApiRoutingMixin,
    FleetMapSyncMixin,
    RobotMapSyncMixin,
    RobotControlProxyMixin,
):
    """Compose runtime, registry, map and robot-control capabilities."""
