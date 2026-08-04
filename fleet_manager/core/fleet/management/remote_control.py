"""Remote robot identity, endpoint and manual-control capabilities."""

from __future__ import annotations

from typing import Any
from urllib.parse import urlparse

from fleet_manager.core.transport.endpoints import (
    DEFAULT_GRPC_PORT,
    normalize_grpc_endpoint,
)
from fleet_manager.core.fleet.domain.constants import (
    EXTERNAL_CONTROL_PAUSE_PREFIX,
    FLEET_CONTROL_OWNER_ID,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.fleet.domain.models import FleetRobot

from .state import runtime_command


class FleetManagerRemoteControlMixin:
    """Normalize remote identities and control transport-backed robots."""

    def _cancel_remote_route(self, robot: FleetRobot, reason: str) -> bool:
        """Cancel a transport route and report whether it is safe to retire.

        Simulation has no independent transport state, so cancellation is
        acknowledged immediately.  Remote runtimes override this hook and
        must return ``False`` while the physical robot may still be executing
        the route.
        """
        del robot, reason
        return True

    def _stop_remote_robot(self, robot: FleetRobot) -> None:
        """No-op transport hook overridden by the gRPC runtime."""
        del robot


    @runtime_command
    def teleop_robot(
        self,
        payload: dict[str, Any],
        *,
        include_state: bool = True,
    ) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            raise ValueError(f"unknown robot: {name}")
        if not robot.is_remote() or not robot.base_url:
            raise ValueError(f"{name} is not a remote robot")

        linear = float(payload.get("linear", 0.0) or 0.0)
        angular = float(payload.get("angular", 0.0) or 0.0)
        timeout_ms = max(80, int(payload.get("timeoutMs", 350) or 350))
        if robot.active_order_id:
            self._cancel_active_order_for_robot(robot, "manual control takeover")
        try:
            self._ensure_remote_control(robot, "manual control")
            response = self.remote_adapter.teleop(
                robot.base_url,
                linear=linear,
                angular=angular,
                timeout_ms=timeout_ms,
                owner_id=FLEET_CONTROL_OWNER_ID,
            )
            robot.remote_online = True
            robot.remote_error = ""
            status = response.get("status")
            if isinstance(status, dict):
                self._apply_remote_status(robot, status, self._now())
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote teleop failed: {exc}"
            robot.updated_at = self._now()
            raise ValueError(robot.last_reason) from exc

        robot.status = "MANUAL"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.trajectory_dirty = True
        robot.last_reason = "manual control active"
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()
        self._advance_planning_revision(f"manual control started: {name}")
        return {
            "ok": True,
            "robot": robot.to_dict(),
            "state": self.state() if include_state else None,
        }

    @runtime_command
    def teleop_stop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            raise ValueError(f"unknown robot: {name}")
        if not robot.is_remote() or not robot.base_url:
            raise ValueError(f"{name} is not a remote robot")
        try:
            self._ensure_remote_control(robot, "manual stop")
            response = self.remote_adapter.teleop_stop(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
            status = response.get("status")
            if isinstance(status, dict):
                self._apply_remote_status(robot, status, self._now())
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote teleop stop failed: {exc}"
            robot.updated_at = self._now()
            raise ValueError(robot.last_reason) from exc

        if robot.status == "MANUAL":
            robot.status = "IDLE"
            robot.last_reason = "manual control released"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.trajectory_dirty = True
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()
        self._advance_planning_revision(f"manual control stopped: {name}")
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    @runtime_command
    def note_external_control_takeover(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
    ) -> bool:
        """Mirror a direct robot takeover into Fleet Manager immediately.

        The robot remains the source of truth for ownership.  This local note
        closes the short race where an operator can acquire and release again
        before the next remote status poll observes the foreign owner.
        """
        try:
            normalized = normalize_grpc_endpoint(endpoint, default_port=DEFAULT_GRPC_PORT)
        except Exception:
            normalized = str(endpoint or "").strip()
        parsed_endpoint = urlparse(normalized)
        endpoint_key = (
            str(parsed_endpoint.hostname or "").lower(),
            int(parsed_endpoint.port or DEFAULT_GRPC_PORT),
        )
        robot = next(
            (
                candidate
                for candidate in self.robots.values()
                if candidate.is_remote()
                and (
                    candidate.base_url == normalized
                    or (
                        str(urlparse(candidate.base_url).hostname or "").lower(),
                        int(urlparse(candidate.base_url).port or DEFAULT_GRPC_PORT),
                    ) == endpoint_key
                )
            ),
            None,
        )
        if robot is None:
            return False

        status = dict(robot.remote_status) if isinstance(robot.remote_status, dict) else {}
        control = dict(status.get("control")) if isinstance(status.get("control"), dict) else {}
        control.update({"state": "OWNED", "ownerId": owner_id, "ownerName": owner_name})
        status.update(
            {
                "control": control,
                "controlState": "OWNED",
                "controlOwner": owner_id,
                "controlOwnerName": owner_name,
            }
        )
        robot.remote_status = status
        now = self._now()
        if robot.active_order_id:
            order = self.orders.get(robot.active_order_id)
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                self._pause_order_for_external_control(robot, order, now, owner_name or owner_id)
                self._advance_planning_revision(
                    f"external control takeover: {robot.name}"
                )
                return True
        robot.status = "MANUAL"
        robot.last_reason = f"{EXTERNAL_CONTROL_PAUSE_PREFIX} {owner_name or owner_id}"
        robot.updated_at = now
        self._advance_planning_revision(
            f"external control takeover: {robot.name}"
        )
        return True


    def _robot_mode_from_payload(self, payload: dict[str, Any]) -> str:
        raw = str(payload.get("mode") or payload.get("type") or payload.get("robotMode") or "simulated").strip().lower()
        if raw in {"remote", "robot", "real", "grpc", "aivison_grpc", "real_grpc"}:
            return "remote"
        return "simulated"

    def _robot_name_from_payload(self, payload: dict[str, Any]) -> str:
        return str(
            payload.get("name")
            or payload.get("robotName")
            or payload.get("robot_name")
            or payload.get("alias")
            or ""
        ).strip()

    def _remote_base_url_from_payload(self, payload: dict[str, Any]) -> str:
        value = str(
            payload.get("baseUrl")
            or payload.get("url")
            or payload.get("host")
            or payload.get("ip")
            or payload.get("address")
            or ""
        ).strip()
        if not value:
            return ""
        if getattr(self.remote_adapter, "transport", "") == "grpc":
            if value.startswith("grpc://") or value.startswith("grpcs://"):
                return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
            if "://" in value:
                raise ValueError("unsupported robot endpoint scheme; use grpc://host:port")
            port_raw = payload.get("port") or payload.get("grpcPort") or payload.get("grpc_port")
            if port_raw is None:
                return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
            try:
                port = int(port_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid robot gRPC port") from exc
            return normalize_grpc_endpoint(f"grpc://{value}:{port}", default_port=DEFAULT_GRPC_PORT)
        if "://" in value:
            raise ValueError("unsupported robot endpoint scheme; use grpc://host:port")
        port_raw = payload.get("port") or payload.get("grpcPort") or payload.get("grpc_port")
        if port_raw is None:
            return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
        try:
            port = int(port_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid robot gRPC port") from exc
        return normalize_grpc_endpoint(f"grpc://{value}:{port}", default_port=DEFAULT_GRPC_PORT)

    def _remote_robot_name(
        self,
        identity_payload: dict[str, Any] | None,
        status_payload: dict[str, Any] | None,
        base_url: str,
    ) -> str:
        candidates: list[Any] = []

        def collect(payload: dict[str, Any] | None) -> None:
            if not isinstance(payload, dict):
                return
            for key in ("robotId", "robot_id", "name", "id", "vehicleId", "vehicle_id", "uuid", "serial"):
                candidates.append(payload.get(key))
            for nested_key in ("identity", "robot", "basic_info", "basicInfo", "robot_report", "robotReport"):
                nested = payload.get(nested_key)
                if isinstance(nested, dict):
                    for key in ("robotId", "robot_id", "name", "id", "vehicleId", "vehicle_id", "uuid", "serial"):
                        candidates.append(nested.get(key))

        collect(identity_payload)
        collect(status_payload)
        for value in candidates:
            text = str(value or "").strip()
            if text and text.lower() not in {"none", "null", "unknown", "-"}:
                return text
        parsed = urlparse(base_url)
        return str(parsed.hostname or parsed.netloc or "").strip()

    def _remote_unique_robot_name(self, name: str, base_url: str) -> str:
        clean_name = str(name or "").strip() or self._remote_name_from_endpoint(base_url)
        for existing in self.robots.values():
            if existing.is_remote() and existing.base_url == base_url:
                return existing.name
        existing = self.robots.get(clean_name)
        if existing is None or (existing.is_remote() and existing.base_url == base_url):
            return clean_name

        suffix = self._remote_name_from_endpoint(base_url)
        candidate = f"{clean_name}-{suffix}" if suffix and suffix != clean_name else f"{clean_name}-remote"
        index = 2
        while candidate in self.robots:
            candidate = f"{clean_name}-{suffix or 'remote'}-{index}"
            index += 1
        return candidate

    def _remote_name_from_endpoint(self, base_url: str) -> str:
        parsed = urlparse(base_url)
        host = str(parsed.hostname or parsed.netloc or "").strip()
        if not host:
            return "remote"
        parts = [part for part in host.replace(":", ".").split(".") if part]
        if len(parts) >= 4 and all(part.isdigit() for part in parts[-4:]):
            return f"robot-{parts[-1]}"
        return host.replace(".", "-")

    def _remote_identity_id(self, identity_payload: dict[str, Any] | None) -> str:
        if not isinstance(identity_payload, dict):
            return ""
        identity = identity_payload.get("identity")
        if isinstance(identity, dict):
            value = identity.get("robotId") or identity.get("id")
        else:
            value = identity_payload.get("robotId") or identity_payload.get("id")
        return str(value or "").strip()

    def _remote_status_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        robot_payload = payload.get("robot")
        if isinstance(robot_payload, dict):
            return robot_payload
        return payload if isinstance(payload, dict) else {}

    def _remote_pose_from_status(self, status_payload: dict[str, Any]) -> dict[str, float] | None:
        pose = status_payload.get("pose")
        if isinstance(pose, dict):
            try:
                return {
                    "x": float(pose.get("x", 0.0) or 0.0),
                    "y": float(pose.get("y", 0.0) or 0.0),
                    "yaw": float(pose.get("yaw", 0.0) or pose.get("angle", 0.0) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        if "x" in status_payload and "y" in status_payload:
            try:
                return {
                    "x": float(status_payload.get("x", 0.0) or 0.0),
                    "y": float(status_payload.get("y", 0.0) or 0.0),
                    "yaw": float(status_payload.get("yaw", status_payload.get("angle", 0.0)) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        robot_report = status_payload.get("robot_report") or status_payload.get("robotReport")
        if isinstance(robot_report, dict) and "x" in robot_report and "y" in robot_report:
            try:
                return {
                    "x": float(robot_report.get("x", 0.0) or 0.0),
                    "y": float(robot_report.get("y", 0.0) or 0.0),
                    "yaw": float(robot_report.get("angle", robot_report.get("yaw", 0.0)) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        return None

    def _remote_timeout(self) -> float:
        return self.settings.fleet.number(
            "remote_timeout_sec",
            0.8,
            minimum=0.2,
            default_if_falsy=True,
        )
