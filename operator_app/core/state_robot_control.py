"""Proxy robot SLAM, teleop and validated gRPC controls."""

from __future__ import annotations

import ipaddress
import json
from typing import Any
from urllib.parse import urlparse

from fleet_manager.manager.endpoints import DEFAULT_GRPC_PORT
from fleet_manager.runtime.grpc.api.client import GrpcRobotError
from .state_common import (
    OPERATOR_CONTROL_OWNER_ID,
    OPERATOR_CONTROL_OWNER_NAME,
)


class RobotControlProxyMixin:
    """Proxy robot SLAM, teleop and validated gRPC controls."""

    def watch_robot_laser_scan(
        self,
        robot_id: str,
        *,
        topic: str = "/scan",
        hz: float = 1.0,
        include_intensities: bool = False,
    ) -> Any:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        return self.grpc_adapter.watch_laser_scan(
            self._grpc_endpoint(robot),
            topic=topic,
            hz=hz,
            include_intensities=include_intensities,
        )

    def robot_slam_defaults_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        result = self.grpc_adapter.get_slam_defaults(self._grpc_endpoint(robot))
        result["robotId"] = robot_id
        return result

    def robot_slam_state_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        result = self.grpc_adapter.get_slam_state(self._grpc_endpoint(robot))
        result["robotId"] = robot_id
        return result

    def start_robot_slam_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        params = payload.get("params") if isinstance(payload.get("params"), dict) else payload
        result = self.grpc_adapter.start_slam(
            self._grpc_endpoint(robot),
            params if isinstance(params, dict) else {},
            use_sim_time=bool(payload.get("useSimTime", True)),
        )
        result["robotId"] = robot_id
        return result

    def finish_robot_slam_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        map_name = str(payload.get("mapName") or payload.get("map_name") or "").strip()
        activate = bool(payload.get("activate", True))
        result = self.grpc_adapter.finish_slam(
            self._grpc_endpoint(robot),
            map_name=map_name,
            activate=activate,
        )
        bundle = result.get("bundle") if isinstance(result.get("bundle"), dict) else None
        if isinstance(bundle, dict):
            if activate:
                remote = self.grpc_adapter.get_map_bundle(
                    self._grpc_endpoint(robot),
                    str(result.get("mapName") or map_name),
                )
                expected_signature = str(bundle.get("signature") or "").strip()
                remote_signature = str(remote.get("signature") or "").strip()
                if not expected_signature or remote_signature != expected_signature:
                    raise ValueError(
                        "saved SLAM map failed robot read-back verification: "
                        f"expected {expected_signature or '-'}, got {remote_signature or '-'}"
                    )
                active = self.grpc_adapter.active_map(self._grpc_endpoint(robot))
                if (
                    str(active.get("mapName") or "").strip() != str(result.get("mapName") or map_name).strip()
                    or str(active.get("signature") or "").strip() != expected_signature
                ):
                    raise ValueError("saved SLAM map is not the verified active robot map")
                cached = self.map_cache.save_pulled_map(robot.id, bundle, activate=True)
            else:
                cached = self.map_cache.save_local_bundle(robot.id, bundle, activate=False)
            result["local"] = cached
            if activate:
                self.workspace.save_active_map_meta(
                    robot,
                    {
                        "ok": True,
                        "mapName": str(result.get("mapName") or map_name),
                        "mapDir": str(result.get("mapDir") or ""),
                        "mapId": str(result.get("mapId") or ""),
                        "signature": str(result.get("signature") or ""),
                    },
                )
            try:
                self.workspace.save_map_index(robot, self.grpc_adapter.list_maps(self._grpc_endpoint(robot)))
            except Exception:
                pass
        result["robotId"] = robot_id
        return result

    def cancel_robot_slam_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        result = self.grpc_adapter.cancel_slam(
            self._grpc_endpoint(robot),
            reason=str(payload.get("reason") or "SLAM canceled by operator."),
        )
        result["robotId"] = robot_id
        return result

    def watch_robot_slam_map(
        self,
        robot_id: str,
        *,
        hz: float = 1.0,
        include_cells: bool = True,
    ) -> Any:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        return self.grpc_adapter.watch_slam_map(
            self._grpc_endpoint(robot),
            hz=hz,
            include_cells=include_cells,
        )

    def robot_teleop_stream(self, robot_id: str, commands) -> Any:
        robot = self.get_robot(robot_id)
        if not robot.is_grpc:
            raise ValueError("unsupported robot transport; use grpc")
        return self.grpc_adapter.teleop_stream(
            self._grpc_endpoint(robot),
            commands,
            owner_id=OPERATOR_CONTROL_OWNER_ID,
        )

    def _proxy_grpc_robot_request(self, robot_id: str, method: str, path: str, *, body: bytes | None) -> tuple[int, dict[str, str], bytes]:
        robot = self.get_robot(robot_id)
        endpoint = self._grpc_endpoint(robot)
        parsed = urlparse(path)
        route = parsed.path.rstrip("/") or "/"
        payload: dict[str, Any] = {}
        if body:
            try:
                decoded = json.loads(body.decode("utf-8"))
            except json.JSONDecodeError as exc:
                raise ValueError("invalid JSON") from exc
            if isinstance(decoded, dict):
                payload = decoded
        method = method.upper()
        try:
            if method == "GET" and route == "/health":
                return self._json_response_tuple(self.grpc_adapter.client.health(endpoint))
            if method == "GET" and route == "/api/robot/identity":
                return self._json_response_tuple(self.grpc_adapter.identity(endpoint))
            if method == "GET" and route == "/api/robot/status":
                return self._json_response_tuple(self.grpc_adapter.status(endpoint))
            if method == "POST" and route == "/api/robot/teleop":
                return self._json_response_tuple(
                    self.grpc_adapter.teleop(
                        endpoint,
                        linear=float(payload.get("linear", 0.0) or 0.0),
                        angular=float(payload.get("angular", 0.0) or 0.0),
                        timeout_ms=int(payload.get("timeoutMs", 350) or 350),
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                    )
                )
            if method == "POST" and route == "/api/robot/teleop/stop":
                return self._json_response_tuple(self.grpc_adapter.teleop_stop(endpoint, owner_id=OPERATOR_CONTROL_OWNER_ID))
            if method == "POST" and route == "/api/robot/stop":
                return self._json_response_tuple(self.grpc_adapter.stop(endpoint, owner_id=OPERATOR_CONTROL_OWNER_ID))
            if method == "POST" and route == "/api/robot/control/acquire":
                result = self.grpc_adapter.acquire_control(
                    endpoint,
                    owner_id=OPERATOR_CONTROL_OWNER_ID,
                    owner_name=OPERATOR_CONTROL_OWNER_NAME,
                    force=bool(payload.get("force")),
                    lease_ms=int(payload.get("leaseMs", payload.get("lease_ms", 0)) or 0),
                )
                if bool(payload.get("stopNavigation") or payload.get("stop_navigation")):
                    # A control handoff must not leave the previous Fleet
                    # Manager route publishing motion behind the operator.
                    stopped = self.grpc_adapter.stop(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                    )
                    if isinstance(stopped.get("status"), dict):
                        result["status"] = stopped["status"]
                    result["navigationStopped"] = True
                    self._note_fleet_external_control_takeover(endpoint)
                return self._json_response_tuple(result)
            if method == "POST" and route == "/api/robot/control/release":
                return self._json_response_tuple(
                    self.grpc_adapter.release_control(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        force=bool(payload.get("force")),
                    )
                )
            if method == "POST" and route == "/api/robot/relocate":
                pose = payload.get("pose") if isinstance(payload.get("pose"), dict) else payload
                return self._json_response_tuple(
                    self.grpc_adapter.relocate(
                        endpoint,
                        x=float(pose.get("x", 0.0) or 0.0),
                        y=float(pose.get("y", 0.0) or 0.0),
                        yaw=float(pose.get("yaw", pose.get("theta", 0.0)) or 0.0),
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        frame_id=str(payload.get("frameId") or payload.get("frame_id") or "map"),
                        covariance=payload.get("covariance") if isinstance(payload.get("covariance"), list) else None,
                        confirm=bool(payload.get("confirm")),
                    )
                )
            if method == "POST" and route == "/api/robot/route/cancel":
                return self._json_response_tuple(self.grpc_adapter.cancel_route(endpoint, owner_id=OPERATOR_CONTROL_OWNER_ID))
            if method == "POST" and route == "/api/robot/route/pause":
                return self._json_response_tuple(
                    self.grpc_adapter.pause_route(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        message=str(payload.get("message") or ""),
                    )
                )
            if method == "POST" and route == "/api/robot/route/resume":
                return self._json_response_tuple(
                    self.grpc_adapter.resume_route(
                        endpoint,
                        owner_id=OPERATOR_CONTROL_OWNER_ID,
                        message=str(payload.get("message") or ""),
                    )
                )
            if method == "POST" and route == "/api/robot/route/execute":
                payload.setdefault("ownerId", OPERATOR_CONTROL_OWNER_ID)
                return self._json_response_tuple(self.grpc_adapter.execute_route(endpoint, payload))
        except GrpcRobotError as exc:
            return self._json_response_tuple({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:
            return self._json_response_tuple({"ok": False, "error": str(exc)}, status=500)
        return self._json_response_tuple({"ok": False, "error": f"unsupported gRPC robot path: {method} {route}"}, status=404)

    def _json_response_tuple(self, payload: dict[str, Any], *, status: int = 200) -> tuple[int, dict[str, str], bytes]:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        return status, {"Content-Type": "application/json; charset=utf-8"}, encoded

    @staticmethod
    def _require_host(payload: dict[str, Any]) -> str:
        host = str(payload.get("host") or "").strip()
        if not host:
            raise ValueError("host is required")
        if host in {"0.0.0.0", "::"}:
            raise ValueError(
                f"{host} is a listen/bind address, not a robot address. "
                "Use 127.0.0.1 on the same PC or the robot LAN IP like 192.168.x.x."
            )
        if any(ch.isspace() for ch in host):
            raise ValueError("robot host must not contain whitespace")
        try:
            ipaddress.ip_address(host)
        except ValueError:
            if not all(ch.isalnum() or ch in ".-_" for ch in host):
                raise ValueError("robot host must be an IP address or DNS name")
        return host

    @staticmethod
    def _require_port(payload: dict[str, Any], *, default: int = DEFAULT_GRPC_PORT) -> int:
        raw = payload.get("port", default)
        try:
            port = int(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("port must be an integer") from exc
        if port < 1 or port > 65535:
            raise ValueError("port must be in range 1..65535")
        return port

    @staticmethod
    def _payload_robot_type(payload: dict[str, Any]) -> str:
        return str(payload.get("type") or payload.get("mode") or "grpc").strip().lower() or "grpc"
