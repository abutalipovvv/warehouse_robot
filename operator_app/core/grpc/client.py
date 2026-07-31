"""Operator-facing compatibility layer for the shared robot gRPC client.

The wire protocol and RPC implementation live in ``fleet_manager.runtime``.
The operator only specializes how an unavailable pose is represented in its
JSON API.
"""

from __future__ import annotations

from typing import Any

from fleet_manager.runtime.grpc.api.client import (
    GrpcRobotAdapter as _SharedGrpcRobotAdapter,
    GrpcRobotClient as _SharedGrpcRobotClient,
    GrpcRobotError,
)

from .contracts import DEFAULT_GRPC_PORT


class GrpcRobotClient(_SharedGrpcRobotClient):
    """Shared client with the operator's nullable-pose presentation rule."""

    def status(self, endpoint: str) -> dict[str, Any]:
        payload = super().status(endpoint)
        robot = payload.get("robot")
        if isinstance(robot, dict) and (
            not bool(robot.get("connected"))
            or not bool(robot.get("localizationOk"))
        ):
            robot["pose"] = None
        return payload


class GrpcRobotAdapter(_SharedGrpcRobotAdapter):
    """Adapter that constructs the operator-specific client above."""

    def __init__(
        self,
        *,
        timeout: float = 1.5,
        default_port: int = DEFAULT_GRPC_PORT,
    ) -> None:
        self.client = GrpcRobotClient(
            timeout=timeout,
            default_port=default_port,
        )


__all__ = ["GrpcRobotAdapter", "GrpcRobotClient", "GrpcRobotError"]
