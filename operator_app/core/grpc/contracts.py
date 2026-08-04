"""Operator-specific JSON conversion for the canonical robot API."""

from __future__ import annotations

from typing import Any

from fleet_manager.runtime.grpc.api.contracts import (
    robot_status_to_json as _shared_robot_status_to_json,
)

from fleet_manager.runtime.grpc.api.proto import robot_api_pb2


def robot_status_to_json(
    status: robot_api_pb2.RobotStatus | None,
) -> dict[str, Any]:
    """Convert a status and hide coordinates until localization is valid."""

    payload = _shared_robot_status_to_json(status)
    if payload and (
        not bool(payload.get("connected"))
        or not bool(payload.get("localizationOk"))
    ):
        payload["pose"] = None
    return payload
