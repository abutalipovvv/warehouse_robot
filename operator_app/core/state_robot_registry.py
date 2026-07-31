"""Register, probe and describe robot endpoints."""

from __future__ import annotations

import re
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from time import perf_counter
from typing import Any

from .config import GRPC_ROBOT_TYPES
from .fleet_manager import FLEET_MANAGER_IDS
from .models import KnownRobot
from .grpc.client import GrpcRobotAdapter
from .grpc.contracts import DEFAULT_GRPC_PORT
from .state_common import (
    OPERATOR_CONTROL_OWNER_ID,
    OPERATOR_CONTROL_OWNER_NAME,
    RobotProbeError,
    utc_now,
)


class RobotRegistryProbeMixin:
    """Register, probe and describe robot endpoints."""

    def list_robots_payload(self, probe_robots: bool = True) -> dict[str, Any]:
        with self._lock:
            robots = self.registry.load()
        fleet_payloads = self._fleet_sidebar_payloads(
            include_runtime=probe_robots
        )
        items = [self._robot_payload(robot, allow_probe=probe_robots) for robot in robots]
        items.extend(fleet_payloads)
        return {"ok": True, "robots": items}

    def robot_pings_payload(self) -> dict[str, Any]:
        with self._lock:
            robots = [robot for robot in self.registry.load() if robot.is_grpc]
        if not robots:
            return {"ok": True, "robots": []}

        results: list[dict[str, Any] | None] = [None] * len(robots)
        max_workers = min(16, max(1, len(robots)))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                executor.submit(self._robot_ping_payload, robot): index
                for index, robot in enumerate(robots)
            }
            for future in as_completed(futures):
                index = futures[future]
                try:
                    results[index] = future.result()
                except Exception as exc:
                    robot = robots[index]
                    results[index] = {
                        "id": robot.id,
                        "online": False,
                        "error": str(exc),
                        "probed": True,
                        "pingOk": False,
                        "pingMs": None,
                        "pingError": str(exc),
                    }
        return {"ok": True, "robots": [item for item in results if item is not None]}

    def probe_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        host = self._require_host(payload)
        robot_type = self._payload_robot_type(payload)
        if robot_type not in GRPC_ROBOT_TYPES:
            raise ValueError("robot transport must be grpc")
        port = self._require_port(payload, default=DEFAULT_GRPC_PORT)
        result = self._probe_grpc_robot(host, port)
        return {"ok": True, "probe": result}

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        host = self._require_host(payload)
        robot_type = self._payload_robot_type(payload)
        if robot_type in GRPC_ROBOT_TYPES:
            port = self._require_port(payload, default=DEFAULT_GRPC_PORT)
            try:
                probe = self._probe_grpc_robot(host, port)
            except Exception as exc:
                probe = self._offline_grpc_probe(host, port, str(exc))
            identity = probe.get("identity", {})
            if not isinstance(identity, dict):
                identity = {}
            status = probe.get("status", {}) if isinstance(probe.get("status"), dict) else {}
            with self._lock:
                existing = self._registered_robot_for_endpoint(host, port)
            existing_identity = existing.last_identity if existing and isinstance(existing.last_identity, dict) else {}
            existing_name = str(existing.name or "").strip() if existing else ""
            generated_existing_name = existing_name in {"", f"robot@{host}", self._grpc_robot_key(host, port)}
            robot_name = str(
                payload.get("name")
                or ("" if generated_existing_name else existing_name)
                or identity.get("robotId")
                or identity.get("name")
                or existing_identity.get("robotId")
                or existing_identity.get("name")
                or existing_name
                or status.get("robotId")
                or f"robot@{host}"
            ).strip()
            robot_id = existing.id if existing else self._grpc_robot_key(host, port)
            robot = KnownRobot(
                id=robot_id,
                name=robot_name,
                host=host,
                port=port,
                type="grpc",
                last_seen=utc_now() if probe.get("ok") else (existing.last_seen if existing else ""),
                last_identity=identity if isinstance(identity, dict) and identity else (existing.last_identity if existing else None),
            )
            with self._lock:
                self.registry.upsert(robot)
            if probe.get("ok"):
                cache = self._bootstrap_robot_workspace(robot, robot.base_url)
            else:
                cache = {
                    "workspace": self._ensure_robot_workspace(robot),
                    "cachedMaps": [],
                    "activeMapName": "",
                    "warnings": [str(probe.get("error") or "robot is offline")],
                }
            return {
                "ok": True,
                "robot": self._robot_payload(robot, probe=probe),
                "cache": cache,
            }
        raise ValueError("robot transport must be grpc")

    def _registered_robot_for_endpoint(self, host: str, port: int) -> KnownRobot | None:
        for robot in self.registry.load():
            if robot.is_grpc and robot.host == host and int(robot.port) == int(port):
                return robot
        return None

    @staticmethod
    def _grpc_robot_key(host: str, port: int) -> str:
        safe_host = re.sub(r"[^A-Za-z0-9_.-]+", "_", host.strip().lower()).strip("._") or "robot"
        return f"grpc_{safe_host}_{int(port)}"

    def delete_robot_payload(self, robot_id: str) -> dict[str, Any]:
        if robot_id in FLEET_MANAGER_IDS:
            raise ValueError("system robot entries cannot be removed")
        with self._lock:
            deleted = self.registry.remove(robot_id)
        if not deleted:
            raise ValueError(f"unknown robot: {robot_id}")
        return {"ok": True, "deleted": robot_id}

    def get_robot(self, robot_id: str) -> KnownRobot:
        with self._lock:
            robots = self.registry.load()
        for robot in robots:
            if robot.id == robot_id:
                return robot
        raise ValueError(f"unknown robot: {robot_id}")

    def _robot_payload(
        self,
        robot: KnownRobot,
        probe: dict[str, Any] | None = None,
        *,
        allow_probe: bool = True,
    ) -> dict[str, Any]:
        if robot.is_grpc:
            if probe is None and allow_probe:
                try:
                    probe = self._probe_grpc_robot(robot.host, robot.port)
                except Exception as exc:
                    probe = self._offline_grpc_probe(robot.host, robot.port, str(exc), identity=robot.last_identity or None)
            if probe is None:
                probe = {
                    "ok": False,
                    "baseUrl": robot.base_url,
                    "online": False,
                    "identity": robot.last_identity or None,
                    "status": None,
                    "error": "",
                    "probed": False,
                    "pingOk": False,
                    "pingMs": None,
                    "pingError": "",
                }
            payload = robot.to_dict()
            payload.update(
                {
                    "online": bool(probe.get("ok", False)),
                    "baseUrl": str(probe.get("baseUrl") or robot.base_url),
                    "identity": probe.get("identity") or robot.last_identity or None,
                    "status": probe.get("status") if isinstance(probe.get("status"), dict) else None,
                    "error": str(probe.get("error") or "").strip(),
                    "probed": bool(probe.get("probed", True)),
                    "pingOk": bool(probe.get("pingOk")),
                    "pingMs": probe.get("pingMs") if isinstance(probe.get("pingMs"), (int, float)) else None,
                    "pingError": str(probe.get("pingError") or "").strip(),
                    "workspace": self.workspace.workspace_payload(robot),
                }
            )
            return payload

        payload = robot.to_dict()
        payload.update(
            {
                "online": False,
                "baseUrl": "",
                "identity": robot.last_identity or None,
                "status": None,
                "error": "unsupported robot transport; use grpc",
                "probed": bool(allow_probe),
            }
        )
        return payload

    def _ping_robot_host(self, host: str) -> dict[str, Any]:
        started = perf_counter()
        try:
            completed = subprocess.run(
                ["ping", "-c", "1", "-W", "1", host],
                capture_output=True,
                check=False,
                text=True,
                timeout=1.5,
            )
        except FileNotFoundError:
            return {"pingOk": False, "pingMs": None, "pingError": "ping command is not installed"}
        except subprocess.TimeoutExpired:
            return {"pingOk": False, "pingMs": None, "pingError": "ping timeout"}
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "ping timeout").strip().splitlines()
            return {"pingOk": False, "pingMs": None, "pingError": detail[-1] if detail else "ping timeout"}
        match = re.search(r"time[=<]([0-9.]+)\s*ms", completed.stdout)
        if match:
            return {"pingOk": True, "pingMs": float(match.group(1)), "pingError": ""}
        return {"pingOk": True, "pingMs": round((perf_counter() - started) * 1000.0, 1), "pingError": ""}

    @staticmethod
    def _grpc_latency_ping(started: float) -> dict[str, Any]:
        return {
            "pingOk": True,
            "pingMs": round((perf_counter() - started) * 1000.0, 1),
            "pingError": "",
            "pingTransport": "grpc",
        }

    def _robot_ping_payload(self, robot: KnownRobot) -> dict[str, Any]:
        endpoint = robot.base_url
        adapter = GrpcRobotAdapter(timeout=min(0.75, max(0.2, float(self.grpc_adapter.client.timeout))))
        try:
            started = perf_counter()
            adapter.client.health(endpoint)
            return {
                "id": robot.id,
                "online": True,
                "error": "",
                "probed": True,
                **self._grpc_latency_ping(started),
            }
        except Exception as exc:
            ping = self._ping_robot_host(robot.host)
            return {
                "id": robot.id,
                "online": False,
                "error": str(exc),
                "probed": True,
                **ping,
            }
        finally:
            adapter.client.close()

    def _offline_grpc_probe(
        self,
        host: str,
        port: int,
        error: str,
        *,
        identity: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        ping = self._ping_robot_host(host)
        return {
            "ok": False,
            "online": False,
            "baseUrl": f"grpc://{host}:{port}",
            "identity": identity if isinstance(identity, dict) else None,
            "status": None,
            "error": str(error or "robot is offline"),
            "probed": True,
            **ping,
        }

    def _is_grpc_robot_id(self, robot_id: str) -> bool:
        try:
            return self.get_robot(robot_id).is_grpc
        except ValueError:
            return False

    def _grpc_endpoint(self, robot: KnownRobot) -> str:
        if not robot.is_grpc:
            raise ValueError(f"robot is not gRPC-backed: {robot.id}")
        return robot.base_url

    def _note_fleet_external_control_takeover(self, endpoint: str) -> None:
        fleet_manager = getattr(self, "fleet_manager", None)
        fleet_lock = getattr(self, "_fleet_lock", None)
        if fleet_manager is None or fleet_lock is None:
            return
        with fleet_lock:
            fleet_manager.note_external_control_takeover(
                endpoint,
                owner_id=OPERATOR_CONTROL_OWNER_ID,
                owner_name=OPERATOR_CONTROL_OWNER_NAME,
            )

    def _probe_grpc_robot(self, host: str, port: int) -> dict[str, Any]:
        endpoint = f"grpc://{host}:{port}"
        try:
            health_started = perf_counter()
            self.grpc_adapter.client.health(endpoint)
            ping = self._grpc_latency_ping(health_started)
            identity = self.grpc_adapter.identity(endpoint)
            status_payload = self.grpc_adapter.status(endpoint)
        except Exception as exc:
            raise RobotProbeError(f"{endpoint} is not reachable: {exc}") from exc
        robot_status = status_payload.get("robot", {})
        if not isinstance(robot_status, dict):
            robot_status = {}
        return {
            "ok": True,
            "online": True,
            "baseUrl": endpoint,
            "identity": identity,
            "status": robot_status,
            "probed": True,
            **ping,
        }


__all__ = ["RobotRegistryProbeMixin"]
