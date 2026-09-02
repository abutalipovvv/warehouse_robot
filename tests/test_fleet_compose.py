from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DOCKER_DIR = ROOT / "robot" / "simulation" / "docker"
COMPOSE_PATH = DOCKER_DIR / "compose.yaml"


def _compose() -> dict:
    return yaml.safe_load(COMPOSE_PATH.read_text(encoding="utf-8"))


def test_compose_has_one_explicit_service_per_fleet_robot() -> None:
    services = _compose()["services"]

    assert list(services) == ["robot11", "robot12", "robot13", "robot14"]
    for suffix, service_name in enumerate(services, start=11):
        service = services[service_name]
        environment = service["environment"]
        assert service["container_name"] == f"warehouse-robot{suffix}"
        assert service["network_mode"] == "host"
        assert service["ipc"] == "host"
        assert environment["ROBOT_ID"] == f"robot{suffix}"
        assert str(environment["ROS_DOMAIN_ID"]) == str(suffix)
        assert environment["ROS_NAMESPACE"] == ""
        assert environment["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
        assert environment["ROBOT_API_HOST"] == f"127.0.0.{suffix}"
        assert environment["MAP_YAML"].endswith("22.05.26_smap.yaml")
        assert f"robot{suffix}-state:/var/lib/warehouse_robot" in service[
            "volumes"
        ]


def test_image_is_built_offline_from_prebuilt_and_only_robot_driver() -> None:
    dockerfile = (DOCKER_DIR / "Dockerfile.robot-stack").read_text(
        encoding="utf-8"
    )

    assert "apt-get" not in dockerfile
    assert "container-runtime.tar.zst" in dockerfile
    assert "ros2_libs-install.tar.zst" in dockerfile
    assert "COPY robot/robot_driver/" in dockerfile
    assert "colcon build" in dockerfile
    assert "--symlink-install" in dockerfile
    assert "simulation/install" not in dockerfile


def test_compose_uses_container_launch_without_legacy_orchestrators() -> None:
    rendered = COMPOSE_PATH.read_text(encoding="utf-8")
    container_launch = (
        ROOT
        / "robot"
        / "robot_driver"
        / "src"
        / "launch"
        / "launch"
        / "container.launch.py"
    )

    assert "robot_launch container.launch.py" in rendered
    assert container_launch.is_file()
    assert not (DOCKER_DIR / "fleet_containers.py").exists()
    assert not (DOCKER_DIR / "robot-stack-entrypoint.sh").exists()
