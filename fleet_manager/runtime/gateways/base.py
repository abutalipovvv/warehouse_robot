"""Compatibility imports for the transport-neutral gateway ports."""

from fleet_manager.core.gateways import (
    RemoteRobotGateway,
    RobotGateway,
    UnavailableRobotGateway,
)

__all__ = [
    "RemoteRobotGateway",
    "RobotGateway",
    "UnavailableRobotGateway",
]
