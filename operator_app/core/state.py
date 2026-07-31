"""Compatibility facade for operator application state capabilities."""

from __future__ import annotations

from fleet_manager.runtime.loop import RuntimeLoop, RuntimeLoopFailure

from .fleet_manager import OperatorFleetManager
from .grpc.client import GrpcRobotAdapter
from .state_common import (
    OPERATOR_CONTROL_OWNER_ID,
    OPERATOR_CONTROL_OWNER_NAME,
    RobotProbeError,
    utc_now,
)
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


__all__ = [
    "OPERATOR_CONTROL_OWNER_ID",
    "OPERATOR_CONTROL_OWNER_NAME",
    "GrpcRobotAdapter",
    "OperatorAppState",
    "OperatorFleetManager",
    "RobotProbeError",
    "RuntimeLoop",
    "RuntimeLoopFailure",
    "utc_now",
]
