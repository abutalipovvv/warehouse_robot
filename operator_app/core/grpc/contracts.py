"""Operator compatibility exports for the canonical robot API contract."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.transport.endpoints import (
    DEFAULT_GRPC_PORT,
    EndpointError as RobotApiError,
    RobotEndpoint,
    build_grpc_endpoint,
    normalize_grpc_endpoint,
    parse_grpc_endpoint,
)
from fleet_manager.runtime.grpc.api.contracts import (
    API_VERSION,
    DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC,
    DEFAULT_GRPC_MAX_MESSAGE_BYTES,
    GRPC_CHANNEL_OPTIONS,
    json_dumps,
    json_loads_object,
    robot_status_from_json,
    robot_status_to_json as _shared_robot_status_to_json,
)

from .proto import robot_api_pb2


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


__all__ = [
    "API_VERSION",
    "DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC",
    "DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC",
    "DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC",
    "DEFAULT_GRPC_MAX_MESSAGE_BYTES",
    "DEFAULT_GRPC_PORT",
    "GRPC_CHANNEL_OPTIONS",
    "RobotApiError",
    "RobotEndpoint",
    "build_grpc_endpoint",
    "json_dumps",
    "json_loads_object",
    "normalize_grpc_endpoint",
    "parse_grpc_endpoint",
    "robot_status_from_json",
    "robot_status_to_json",
]
