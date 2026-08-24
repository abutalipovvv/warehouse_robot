from __future__ import annotations

from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SERVER_API = ROOT / "fleet_manager" / "runtime" / "grpc" / "api"
ROBOT_API = (
    ROOT
    / "robot"
    / "robot_driver"
    / "src"
    / "robot_grpc_api"
    / "robot_grpc_api"
)


@pytest.mark.parametrize(
    "relative_path",
    [
        Path("client.py"),
        Path("proto/robot_api.proto"),
        Path("proto/robot_api_pb2.py"),
        Path("proto/robot_api_pb2_grpc.py"),
    ],
)
def test_independently_deployed_robot_api_sources_stay_in_sync(
    relative_path: Path,
) -> None:
    """The ROS package is standalone, so CI guards its deliberate copies."""

    assert (SERVER_API / relative_path).read_bytes() == (
        ROBOT_API / relative_path
    ).read_bytes()
