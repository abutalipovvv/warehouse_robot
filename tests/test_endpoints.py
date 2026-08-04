from __future__ import annotations

import pytest

from fleet_manager.manager.endpoints import (
    EndpointError,
    RobotEndpoint,
    build_grpc_endpoint,
    normalize_grpc_endpoint,
    parse_grpc_endpoint,
)
from fleet_manager.runtime.grpc.api.contracts import json_loads_object


def test_endpoint_round_trip_preserves_identity_and_security() -> None:
    endpoint = RobotEndpoint(
        host="2001:db8::1",
        port=55051,
        robot_id="robot 1",
        name="Loader #1",
        secure=True,
    )

    parsed = parse_grpc_endpoint(endpoint.url)

    assert endpoint.target == "[2001:db8::1]:55051"
    assert parsed == endpoint


def test_normalize_adds_scheme_and_default_port() -> None:
    assert normalize_grpc_endpoint("warehouse.local") == (
        "grpc://warehouse.local:50051"
    )
    assert normalize_grpc_endpoint(
        "warehouse.local",
        default_port=60000,
    ) == "grpc://warehouse.local:60000"


@pytest.mark.parametrize(
    "value",
    [
        "",
        "http://warehouse.local:50051",
        "grpc://warehouse.local:not-a-port",
        "grpc://warehouse.local:70000",
    ],
)
def test_invalid_endpoint_is_rejected(value: str) -> None:
    with pytest.raises(EndpointError):
        parse_grpc_endpoint(value)


def test_build_rejects_invalid_host_and_port() -> None:
    with pytest.raises(EndpointError, match="host"):
        build_grpc_endpoint("", 50051)
    with pytest.raises(EndpointError, match="integer"):
        build_grpc_endpoint("localhost", "bad")  # type: ignore[arg-type]
    with pytest.raises(EndpointError, match="1..65535"):
        build_grpc_endpoint("localhost", 0)


def test_runtime_json_contract_uses_the_neutral_endpoint_error() -> None:
    with pytest.raises(EndpointError, match="invalid JSON"):
        json_loads_object("not-json")
