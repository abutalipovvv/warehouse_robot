"""Transport endpoint values that do not depend on gRPC or protobuf."""

from __future__ import annotations

from dataclasses import dataclass
from urllib.parse import parse_qs, urlencode, urlparse, urlunparse


DEFAULT_GRPC_PORT = 50051


class EndpointError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class RobotEndpoint:
    host: str
    port: int = DEFAULT_GRPC_PORT
    robot_id: str = ""
    name: str = ""
    secure: bool = False

    @property
    def target(self) -> str:
        host = (
            f"[{self.host}]"
            if ":" in self.host and not self.host.startswith("[")
            else self.host
        )
        return f"{host}:{self.port}"

    @property
    def url(self) -> str:
        query: dict[str, str] = {}
        if self.robot_id:
            query["robotId"] = self.robot_id
        if self.name:
            query["name"] = self.name
        return build_grpc_endpoint(
            self.host,
            self.port,
            secure=self.secure,
            query=query,
        )


def build_grpc_endpoint(
    host: str,
    port: int = DEFAULT_GRPC_PORT,
    *,
    secure: bool = False,
    query: dict[str, str] | None = None,
) -> str:
    clean_host = str(host or "").strip()
    if not clean_host:
        raise EndpointError("robot host is required")
    try:
        clean_port = int(port)
    except (TypeError, ValueError) as exc:
        raise EndpointError("robot gRPC port must be an integer") from exc
    if clean_port < 1 or clean_port > 65535:
        raise EndpointError("robot gRPC port must be in range 1..65535")

    netloc = (
        f"[{clean_host}]"
        if ":" in clean_host and not clean_host.startswith("[")
        else clean_host
    )
    return urlunparse(
        (
            "grpcs" if secure else "grpc",
            f"{netloc}:{clean_port}",
            "",
            "",
            urlencode(query or {}),
            "",
        )
    )


def parse_grpc_endpoint(
    value: str,
    *,
    default_port: int = DEFAULT_GRPC_PORT,
) -> RobotEndpoint:
    raw = str(value or "").strip().rstrip("/")
    if not raw:
        raise EndpointError("robot gRPC endpoint is required")
    if "://" not in raw:
        raw = f"grpc://{raw}"

    parsed = urlparse(raw)
    if parsed.scheme not in {"grpc", "grpcs"} or not parsed.hostname:
        raise EndpointError(f"invalid robot gRPC endpoint: {value}")
    try:
        port = int(parsed.port or default_port)
    except ValueError as exc:
        raise EndpointError(f"invalid robot gRPC port: {value}") from exc
    if port < 1 or port > 65535:
        raise EndpointError("robot gRPC port must be in range 1..65535")

    query = parse_qs(parsed.query)
    return RobotEndpoint(
        host=parsed.hostname,
        port=port,
        robot_id=_first(query, "robotId") or _first(query, "robot_id"),
        name=_first(query, "name"),
        secure=parsed.scheme == "grpcs",
    )


def normalize_grpc_endpoint(
    value: str,
    *,
    default_port: int = DEFAULT_GRPC_PORT,
) -> str:
    return parse_grpc_endpoint(value, default_port=default_port).url


def _first(query: dict[str, list[str]], key: str) -> str:
    values = query.get(key)
    return str(values[0]) if values else ""


__all__ = [
    "DEFAULT_GRPC_PORT",
    "EndpointError",
    "RobotEndpoint",
    "build_grpc_endpoint",
    "normalize_grpc_endpoint",
    "parse_grpc_endpoint",
]
