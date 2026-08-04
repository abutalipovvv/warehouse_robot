"""In-memory execution boundary for Fleet Manager Sim."""

from __future__ import annotations

from typing import Any


class SimulationRobotGateway:
    """Connect simulation tools to the same fleet runtime instance.

    Motion remains advanced by ``FleetManagerCore.advance_runtime``. The
    gateway owns only the transport boundary so benchmark/web code no longer
    needs to pretend simulated robots are gRPC endpoints.
    """

    transport = "simulation"

    def __init__(self) -> None:
        self._manager: Any | None = None

    def bind(self, manager: Any) -> None:
        self._manager = manager

    @property
    def manager(self) -> Any:
        if self._manager is None:
            raise RuntimeError("simulation gateway is not bound")
        return self._manager

    def add_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.manager.add_robot({**payload, "mode": "simulated"})

    def update_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.manager.update_robot(payload)

    def stop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.manager.stop_robot(payload)
