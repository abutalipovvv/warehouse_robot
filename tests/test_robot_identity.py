from __future__ import annotations

from pathlib import Path
import importlib.util

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
