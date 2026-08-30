from __future__ import annotations

from pathlib import Path
import importlib.util
import sys

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "tools"
    / "validate_robot_identity.py"
)
SPEC = importlib.util.spec_from_file_location("validate_robot_identity", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

NETWORK_IDENTITY = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "robot_driver"
    / "src"
    / "robot_grpc_api"
    / "robot_grpc_api"
    / "network_identity.py"
)
NETWORK_SPEC = importlib.util.spec_from_file_location(
    "robot_network_identity",
    NETWORK_IDENTITY,
)
assert NETWORK_SPEC is not None and NETWORK_SPEC.loader is not None
NETWORK_MODULE = importlib.util.module_from_spec(NETWORK_SPEC)
sys.modules[NETWORK_SPEC.name] = NETWORK_MODULE
NETWORK_SPEC.loader.exec_module(NETWORK_MODULE)


def test_physical_robot_identity_is_persistent_and_complete(
    tmp_path: Path,
) -> None:
    cyclone = tmp_path / "cyclonedds.xml"
    cyclone.write_text("<CycloneDDS />", encoding="utf-8")
    identity = tmp_path / "robot.env"
    identity.write_text(
        "\n".join(
            [
                "ROBOT_ID=warehouse-r12",
                "ROS_NAMESPACE=warehouse_r12",
                "ROS_DOMAIN_ID=42",
                "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
                f"CYCLONEDDS_URI=file://{cyclone}",
            ]
        ),
        encoding="utf-8",
    )

    result = MODULE.validate_identity(MODULE.load_env_file(identity))

    assert result["robotId"] == "warehouse-r12"
    assert result["namespace"] == "warehouse_r12"
    assert result["domainId"] == 42


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("ROBOT_ID", "", "ROBOT_ID"),
        ("ROS_NAMESPACE", "robot-with-hyphen", "ROS_NAMESPACE"),
        ("ROS_DOMAIN_ID", "999", "ROS_DOMAIN_ID"),
        ("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", "RMW_IMPLEMENTATION"),
    ],
)
def test_invalid_robot_identity_is_rejected(
    tmp_path: Path,
    key: str,
    value: str,
    message: str,
) -> None:
    cyclone = tmp_path / "cyclonedds.xml"
    cyclone.write_text("<CycloneDDS />", encoding="utf-8")
    payload = {
        "ROBOT_ID": "warehouse-r01",
        "ROS_NAMESPACE": "warehouse_r01",
        "ROS_DOMAIN_ID": "31",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": f"file://{cyclone}",
    }
    payload[key] = value

    with pytest.raises(ValueError, match=message):
        MODULE.validate_identity(payload)


@pytest.mark.parametrize(
    ("ipv4", "robot_id", "domain_id"),
    [
        ("192.168.1.1", "robot1", 1),
        ("192.168.1.6", "robot6", 6),
        ("10.20.30.254", "robot254", 21),
    ],
)
def test_network_identity_is_derived_from_last_ipv4_octet(
    ipv4: str,
    robot_id: str,
    domain_id: int,
) -> None:
    identity = NETWORK_MODULE.resolve_network_identity(
        selected_network=("wlan0", ipv4),
    )

    assert identity.robot_id == robot_id
    assert identity.domain_id == domain_id
    assert identity.ipv4 == ipv4
    assert identity.interface == "wlan0"


def test_wifi_address_is_preferred_over_wired_default_route() -> None:
    addresses = {
        "enp2s0": "192.168.1.4",
        "wlan0": "192.168.1.6",
        "docker0": "172.17.0.1",
    }

    interface, ipv4 = NETWORK_MODULE.select_robot_ipv4(
        interfaces=addresses,
        address_for=addresses.get,
        wireless_for=lambda name: name == "wlan0",
        default_interface="enp2s0",
    )

    assert (interface, ipv4) == ("wlan0", "192.168.1.6")


def test_explicit_identity_override_does_not_require_network() -> None:
    identity = NETWORK_MODULE.resolve_network_identity("warehouse-r9", "42")

    assert identity.robot_id == "warehouse-r9"
    assert identity.domain_id == 42
    assert identity.interface == "explicit"
