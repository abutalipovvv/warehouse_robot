from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Any

from .models import KnownRobot
from .robot_client import RobotClient, RobotProbeError


def safe_robot_key(value: str) -> str:
    clean = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in value.strip())
    while "__" in clean:
        clean = clean.replace("__", "_")
    return clean.strip("._-") or "robot"


def free_local_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def write_cyclonedds_config(robot: KnownRobot, root: Path = Path("/tmp/warehouse_operator_cyclonedds")) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    config_path = root / f"{safe_robot_key(robot.id)}_{safe_robot_key(robot.host)}_d{robot.domain_id}.xml"
    config_path.write_text(
        "\n".join(
            [
                '<?xml version="1.0" encoding="UTF-8" ?>',
                '<CycloneDDS xmlns="https://cdds.io/config"',
                '            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"',
                '            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">',
                '  <Domain Id="any">',
                "    <General>",
                "      <Interfaces>",
                '        <NetworkInterface autodetermine="true" />',
                "      </Interfaces>",
                "      <AllowMulticast>false</AllowMulticast>",
                "    </General>",
                "    <Discovery>",
                "      <Peers>",
                f'        <Peer Address="{robot.host}" />',
                "      </Peers>",
                "    </Discovery>",
                "  </Domain>",
                "</CycloneDDS>",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return config_path


@dataclass
class RosRobotLink:
    robot: KnownRobot
    base_url: str
    process: subprocess.Popen
    log_path: Path
    _log_file: Any
    timeout: float = 1.5

    def is_running(self) -> bool:
        return self.process.poll() is None

    def close(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.5)
        try:
            self._log_file.close()
        except Exception:
            pass

    def health(self) -> dict[str, Any]:
        return self.request_json("/health", timeout=0.5)

    def sidebar_payload(self) -> dict[str, Any]:
        return self.request_json("/api/robot/sidebar")

    def identity_payload(self) -> dict[str, Any]:
        return self.request_json("/api/robot/identity")

    def status_payload(self) -> dict[str, Any]:
        return self.request_json("/api/robot/status")

    def request(
        self,
        path: str,
        *,
        method: str = "GET",
        headers: dict[str, str] | None = None,
        body: bytes | None = None,
        timeout: float | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        client = RobotClient(timeout=self.timeout if timeout is None else timeout)
        return client.request(self.base_url, path, method=method, headers=headers, body=body, timeout=timeout)

    def request_json(
        self,
        path: str,
        *,
        method: str = "GET",
        payload: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        client = RobotClient(timeout=self.timeout if timeout is None else timeout)
        return client.request_json(self.base_url, path, method=method, payload=payload, timeout=timeout)

    def log_tail(self, max_chars: int = 2000) -> str:
        try:
            raw = self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        return raw[-max_chars:]


class RosRobotLinkManager:
    def __init__(self, *, timeout: float = 1.5, log_dir: Path = Path("/tmp/warehouse_operator_ros_links")) -> None:
        self.timeout = max(0.5, float(timeout))
        self.log_dir = log_dir
        self.links: dict[str, RosRobotLink] = {}

    def get(self, robot: KnownRobot) -> RosRobotLink:
        link = self.links.get(robot.id)
        if link is not None:
            if link.is_running():
                return link
            link.close()
            self.links.pop(robot.id, None)
        link = self._start(robot)
        self.links[robot.id] = link
        return link

    def remove(self, robot_id: str) -> None:
        link = self.links.pop(robot_id, None)
        if link is not None:
            link.close()

    def close(self) -> None:
        for link in list(self.links.values()):
            link.close()
        self.links.clear()

    def _start(self, robot: KnownRobot) -> RosRobotLink:
        port = free_local_port()
        config_path = write_cyclonedds_config(robot)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.log_dir / f"{safe_robot_key(robot.id)}.log"
        log_file = log_path.open("ab")
        env = os.environ.copy()
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        env["ROS_DOMAIN_ID"] = str(robot.domain_id)
        env["CYCLONEDDS_URI"] = f"file://{config_path}"
        env["PYTHONUNBUFFERED"] = "1"
        env.pop("ROS_STATIC_PEERS", None)
        env.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
        env.pop("ROS_LOCALHOST_ONLY", None)
        command = [
            sys.executable,
            "-m",
            "operator_app.ros_robot_link",
            "--bind-host",
            "127.0.0.1",
            "--port",
            str(port),
            "--robot-id",
            robot.id,
            "--robot-name",
            robot.name or robot.id,
            "--host",
            robot.host,
            "--domain-id",
            str(robot.domain_id),
            "--namespace",
            robot.namespace,
            "--status-topic",
            robot.status_topic,
            "--cmd-vel-topic",
            robot.cmd_vel_topic,
            "--go-to-lm-topic",
            robot.go_to_lm_topic,
        ]
        process = subprocess.Popen(
            command,
            cwd=str(Path(__file__).resolve().parents[1]),
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
        )
        link = RosRobotLink(
            robot=robot,
            base_url=f"http://127.0.0.1:{port}",
            process=process,
            log_path=log_path,
            _log_file=log_file,
            timeout=self.timeout,
        )
        self._wait_until_ready(link)
        return link

    def _wait_until_ready(self, link: RosRobotLink) -> None:
        deadline = time.monotonic() + self.timeout
        last_error = ""
        while time.monotonic() < deadline:
            if not link.is_running():
                tail = link.log_tail()
                raise RobotProbeError(f"ROS2 link exited early: {tail or 'no log output'}")
            try:
                link.health()
                return
            except RobotProbeError as exc:
                last_error = str(exc)
                time.sleep(0.05)
        tail = link.log_tail()
        message = last_error or "timed out waiting for ROS2 link"
        if tail:
            message = f"{message}; log: {tail}"
        raise RobotProbeError(message)
