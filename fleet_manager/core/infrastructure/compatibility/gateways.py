"""Compatibility import for the robot gateway public API."""

from fleet_manager.core.transport.gateways import (
    RemoteRobotGateway,
    RobotGateway,
    UnavailableRobotGateway,
)

__all__ = [
    "RemoteRobotGateway",
    "RobotGateway",
    "UnavailableRobotGateway",
]
