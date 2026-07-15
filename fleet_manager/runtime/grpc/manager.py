"""gRPC specialization of the shared fleet runtime."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.manager import FleetManagerCore
from fleet_manager.runtime.gateways.grpc import GrpcRobotGateway
from fleet_manager.runtime.grpc.mixin import GrpcRobotRuntimeMixin


class FleetManagerROS(GrpcRobotRuntimeMixin, FleetManagerCore):
    """Dispatch real ROS 2 robots through their existing gRPC tunnels."""

    runtime_kind = "grpc"

    def __init__(self, *args: Any, remote_adapter=None, **kwargs: Any) -> None:
        gateway = remote_adapter
        super().__init__(*args, remote_adapter=gateway, **kwargs)
        if gateway is None:
            self._configure_robot_gateway()
        else:
            self.robot_gateway = gateway
        self.set_active_robot_modes({"remote"})

    def _configure_robot_gateway(self) -> None:
        gateway = GrpcRobotGateway(timeout=self._remote_timeout())
        self.remote_adapter = gateway
        self.robot_gateway = gateway


# Technical alias: FleetManagerROS communicates through gRPC; it does not
# publish ROS topics directly from the operator process.
FleetManagerGrpc = FleetManagerROS

__all__ = ["FleetManagerGrpc", "FleetManagerROS"]
