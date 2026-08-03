"""Simulation specialization of the shared fleet runtime."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.manager import FleetManagerCore
from ..gateways.simulation import SimulationRobotGateway


class FleetManagerSim(FleetManagerCore):
    """Run production fleet policy against in-memory simulated robots."""

    runtime_kind = "simulation"

    def __init__(self, *args: Any, simulation_gateway=None, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        gateway = simulation_gateway or SimulationRobotGateway()
        gateway.bind(self)
        self.robot_gateway = gateway
        self.set_active_robot_modes({"simulated"})


__all__ = ["FleetManagerSim"]
