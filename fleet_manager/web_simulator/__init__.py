from .manager import (
    FleetCollisionChecker,
    FleetEvent,
    FleetOrder,
    FleetRobot,
    WebFleetManager,
)

FleetManager = WebFleetManager

__all__ = [
    "FleetCollisionChecker",
    "FleetEvent",
    "FleetManager",
    "FleetOrder",
    "FleetRobot",
    "WebFleetManager",
]
