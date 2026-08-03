"""Compatibility entry point for the Fleet Manager composition root."""

from time import time

from fleet_manager.core.management.manager import (
    FleetCollisionChecker,
    FleetEvent,
    FleetManagerCore,
    FleetOrder,
    FleetRobot,
)

__all__ = [
    "FleetCollisionChecker",
    "FleetEvent",
    "FleetManagerCore",
    "FleetOrder",
    "FleetRobot",
]
