"""Compatibility import for the fleet domain models public API."""

from fleet_manager.core.domain.models import FleetEvent, FleetOrder, FleetRobot

__all__ = ["FleetEvent", "FleetOrder", "FleetRobot"]
