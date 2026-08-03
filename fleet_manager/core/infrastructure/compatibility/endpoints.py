"""Compatibility import for the transport endpoint public API."""

from fleet_manager.core.transport.endpoints import (
    DEFAULT_GRPC_PORT,
    EndpointError,
    RobotEndpoint,
    build_grpc_endpoint,
    normalize_grpc_endpoint,
    parse_grpc_endpoint,
)

__all__ = [
    "DEFAULT_GRPC_PORT",
    "EndpointError",
    "RobotEndpoint",
    "build_grpc_endpoint",
    "normalize_grpc_endpoint",
    "parse_grpc_endpoint",
]
