"""Expose status, navigation, teleoperation and control leases."""

from __future__ import annotations

import json
import math
import re
import time
from pathlib import Path
from time import monotonic
from typing import Any

from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.maps.models import WorldPoint

class RosRuntimeControlMixin:
    """Expose status, navigation, teleoperation and control leases."""

    def identity_payload(self) -> dict[str, Any]:
        status = self._message_to_robot_payload(self._latest_message())
        return {
            "ok": True,
            "robotId": status.get("robotId") or self.robot_name,
            "mapId": status.get("mapId") or "",
            "type": "ros2",
            "host": self.host,
            "domainId": self.domain_id,
            "namespace": self.namespace,
            "statusTopic": self.status_topic,
            "cmdVelTopic": self.cmd_vel_topic,
            "initialPoseTopic": self.initial_pose_topic,
            "scanTopic": self.scan_topic,
            "goToLmTopic": self.go_to_lm_topic,
            "planService": self.plan_service_name,
            "executeService": self.execute_service_name,
            "cancelService": self.cancel_service_name,
            "routePauseService": self.route_pause_service_name,
            "routeLoadMapService": self.route_load_map_service_name,
            "statusLoadMapService": self.status_load_map_service_name,
            "mapStateService": self.map_state_service_name,
            "mapLoadService": self.map_load_service_name,
            "mapListService": self.map_list_service_name,
            "mapGetBundleService": self.map_get_bundle_service_name,
            "mapPutBundleService": self.map_put_bundle_service_name,
            "mapTopic": self.map_topic,
            "slamSaveMapService": self.slam_save_map_service_name,
            "slamParamsFile": str(self.slam_params_file),
            "slamLaunchFile": str(self.slam_launch_file),
            "paramsPath": str(self.params_path or ""),
            "available": self._available,
            "error": self._error,
        }

    def sidebar_payload(self) -> dict[str, Any]:
        status_payload = self.status_payload()
        robot_status = status_payload.get("robot") if isinstance(status_payload.get("robot"), dict) else {}
        identity = self.identity_payload()
        return {
            "id": self.robot_id,
            "name": self.robot_name,
            "host": self.host or "DDS",
            "port": 0,
            "baseUrl": "",
            "type": "ros2",
            "mode": "ros2",
            "domainId": self.domain_id,
            "namespace": self.namespace,
            "online": bool(robot_status.get("connected")),
            "identity": identity,
            "lastIdentity": identity,
            "status": robot_status,
            "error": "" if bool(robot_status.get("connected")) else (self._error or str(robot_status.get("message") or "")),
            "probed": True,
        }

    def status_payload(self) -> dict[str, Any]:
        message = self._latest_message()
        robot = self._message_to_robot_payload(message)
        robot.update(self._control_payload(robot))
        return {
            "ok": True,
            "robot": robot,
            "events": list(self._events[-120:]),
            "route": self._route_payload(robot),
        }

    def status_robot_payload(self) -> dict[str, Any]:
        return self.status_payload()["robot"]

    def teleop(self, *, linear: float, angular: float, timeout_ms: int = 350, owner_id: str = "") -> dict[str, Any]:
        del timeout_ms
        self._ensure_control_owner(owner_id, action="manual control")
        self._publish_twist(linear, angular)
        return {"ok": True, "linear": float(linear), "angular": float(angular)}

    def teleop_stop(self) -> dict[str, Any]:
        self._publish_twist(0.0, 0.0)
        return {"ok": True}

    def laser_scan_payload(self, *, topic: str = "/scan", include_intensities: bool = False) -> dict[str, Any]:
        requested_topic = self._topic(topic or self.scan_topic)
        if requested_topic != self.scan_topic:
            raise ValueError(f"scan topic {requested_topic} is not configured; use {self.scan_topic}")
        with self._lock:
            message = self._latest_scan
            received_at = self._latest_scan_at
        if message is None:
            raise ValueError(f"Waiting for LaserScan on {self.scan_topic}.")

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        stamp_sec = float(getattr(stamp, "sec", 0.0) or 0.0) + float(getattr(stamp, "nanosec", 0.0) or 0.0) / 1e9
        ranges = [float(item) for item in list(getattr(message, "ranges", []) or [])]
        intensities = []
        if include_intensities:
            intensities = [float(item) for item in list(getattr(message, "intensities", []) or [])]
        return {
            "ok": True,
            "robotId": self.robot_id,
            "topic": self.scan_topic,
            "frameId": str(getattr(header, "frame_id", "") or ""),
            "stampSec": stamp_sec,
            "receivedAgeSec": 9999.0 if received_at is None else max(0.0, monotonic() - received_at),
            "angleMin": float(getattr(message, "angle_min", 0.0)),
            "angleMax": float(getattr(message, "angle_max", 0.0)),
            "angleIncrement": float(getattr(message, "angle_increment", 0.0)),
            "timeIncrement": float(getattr(message, "time_increment", 0.0)),
            "scanTime": float(getattr(message, "scan_time", 0.0)),
            "rangeMin": float(getattr(message, "range_min", 0.0)),
            "rangeMax": float(getattr(message, "range_max", 0.0)),
            "ranges": ranges,
            "intensities": intensities,
        }

    def stop(self, *, owner_id: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="stop")
        self._publish_twist(0.0, 0.0)
        try:
            self.cancel_route(owner_id=owner_id)
        except Exception:
            self._publish_go_to_lm("cancel")
        return {"ok": True}

    def cancel_route(self, *, owner_id: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="cancel route")
        if self._cancel_route_client is not None and self._service_available(self._cancel_route_client, 0.05):
            request = self._cancel_route_client.srv_type.Request()
            request.message = "Route canceled."
            response = self._call_service(self._cancel_route_client, request, "route cancel")
            if not bool(response.ok):
                raise ValueError(str(response.error or "route cancel failed"))
        else:
            self._publish_go_to_lm("cancel")
        self._navigation_paused = False
        return {"ok": True}

    def execute_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._ensure_control_owner(str(payload.get("ownerId") or payload.get("owner_id") or ""), action="execute route")
        goal_pose = self._goal_pose_payload(payload)
        if goal_pose is not None:
            return self._execute_pose_route(goal_pose, payload)

        if self._execute_route_client is not None and self._service_available(self._execute_route_client, 0.05):
            route_payload = self._route_payload_from_request(payload)
            request = self._execute_route_client.srv_type.Request()
            request.route_json = json.dumps(route_payload, ensure_ascii=False)
            response = self._call_service(self._execute_route_client, request, "route execute")
            if not bool(response.ok):
                raise ValueError(str(response.error or "route execute failed"))
            self._navigation_paused = False
            return {"ok": True, "route": route_payload}

        goal_lm = str(
            payload.get("goalLm")
            or payload.get("goal_lm")
            or payload.get("targetLm")
            or payload.get("target_lm")
            or payload.get("id")
            or ""
        ).strip()
        if not goal_lm:
            raise ValueError("ROS2 local robot currently needs goalLm/id for /go_to_lm navigation")
        source_lm = str(payload.get("startLm") or payload.get("start_lm") or "").strip()
        command: dict[str, Any] = {"id": goal_lm}
        if source_lm:
            command["source_id"] = source_lm
        self._publish_go_to_lm(json.dumps(command, ensure_ascii=False))
        self._navigation_paused = False
        route = {
            "routeId": f"ros2-{goal_lm}",
            "goalLm": goal_lm,
            "startLm": source_lm,
            "nodes": [item for item in [source_lm, goal_lm] if item],
            "trajectory": [],
        }
        return {"ok": True, "route": route}

    def acquire_control(
        self,
        *,
        owner_id: str,
        owner_name: str = "",
        force: bool = False,
        lease_ms: int = 0,
    ) -> dict[str, Any]:
        owner = self._clean_owner_id(owner_id)
        if not owner:
            raise ValueError("owner_id is required")
        with self._lock:
            self._expire_control_owner_locked()
            if self._control_owner_id and self._control_owner_id != owner and not force:
                raise ValueError(f"control is owned by {self._control_owner_name or self._control_owner_id}")
            self._control_owner_id = owner
            self._control_owner_name = str(owner_name or owner)
            self._control_acquired_at = monotonic()
            self._control_lease_ms = max(0, int(lease_ms or 0))
        return {"ok": True, "control": self._control_state_payload()}

    def release_control(self, *, owner_id: str, force: bool = False) -> dict[str, Any]:
        owner = self._clean_owner_id(owner_id)
        with self._lock:
            self._expire_control_owner_locked()
            if self._control_owner_id and self._control_owner_id != owner and not force:
                raise ValueError(f"control is owned by {self._control_owner_name or self._control_owner_id}")
            self._control_owner_id = ""
            self._control_owner_name = ""
            self._control_acquired_at = None
            self._control_lease_ms = 0
        return {"ok": True, "control": self._control_state_payload()}

    def relocate(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        owner_id: str = "",
        frame_id: str = "map",
        covariance_json: str = "",
        confirm: bool = False,
    ) -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="relocate")
        if self._initial_pose_pub is None or self._pose_with_covariance_type is None:
            raise ValueError(self._error or "initial pose publisher is not available")
        ros_x, ros_y, ros_yaw = self._map_pose_to_ros_pose(float(x), float(y), float(yaw))
        message = self._pose_with_covariance_type()
        message.header.frame_id = str(frame_id or "map")
        if self._node is not None:
            message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = ros_x
        message.pose.pose.position.y = ros_y
        message.pose.pose.position.z = 0.0
        qz = math.sin(ros_yaw * 0.5)
        qw = math.cos(ros_yaw * 0.5)
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        covariance = self._covariance_from_json(covariance_json)
        if covariance:
            message.pose.covariance = covariance
        else:
            message.pose.covariance[0] = 0.25
            message.pose.covariance[7] = 0.25
            message.pose.covariance[35] = 0.06853891945200942
        self._initial_pose_pub.publish(message)
        self._publish_twist(0.0, 0.0)
        with self._lock:
            self._localization_confirmed = bool(confirm)
            self._relocation_requested_at = monotonic()
        return {"ok": True, "relocate": {"x": float(x), "y": float(y), "yaw": float(yaw), "frameId": message.header.frame_id}}

    def _map_pose_to_ros_pose(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        active = self.active_map_payload()
        map_dir = Path(str(active.get("mapDir") or "")).resolve()
        if not map_dir.is_dir():
            raise ValueError(f"active map directory is not available: {map_dir}")
        loaded_map = WarehouseMapLoader(map_dir).load()
        point = loaded_map.map_metadata.map_to_ros_point(WorldPoint(x=float(x), y=float(y)))
        return (
            float(point.x),
            float(point.y),
            float(loaded_map.map_metadata.map_yaw_to_ros(float(yaw))),
        )

    def confirm_localization(self, *, accepted: bool = True, message: str = "", owner_id: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="confirm localization")
        with self._lock:
            self._localization_confirmed = bool(accepted)
            if accepted:
                self._relocation_requested_at = None
        return {
            "ok": True,
            "localizationConfirmed": bool(accepted),
            "message": message or ("Localization confirmed." if accepted else "Localization rejected."),
        }

    def pause_route(self, *, owner_id: str = "", message: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="pause route")
        self._set_route_paused(True, message or "Route paused by operator.")
        return {"ok": True, "navigationPaused": True}

    def resume_route(self, *, owner_id: str = "", message: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="resume route")
        self._set_route_paused(False, message or "Route resumed by operator.")
        return {"ok": True, "navigationPaused": False}

    def _execute_pose_route(self, pose: dict[str, float], payload: dict[str, Any]) -> dict[str, Any]:
        script_args: dict[str, Any] = {
            "x": pose["x"],
            "y": pose["y"],
            "theta": pose["yaw"],
            "coordinate": "world",
        }
        for key in ("reachAngle", "reachDist", "backMode", "useOdo"):
            if key in payload:
                script_args[key] = payload[key]
        command = {
            "id": "SELF_POSITION",
            "source_id": "SELF_POSITION",
            "operation": "Script",
            "script_name": "syspy/goPath.py",
            "script_args": script_args,
        }
        self._publish_go_to_lm(json.dumps(command, ensure_ascii=False))
        label = f"pose({pose['x']:.3f},{pose['y']:.3f},{pose['yaw']:.3f})"
        route = {
            "routeId": f"ros2-{label}",
            "goalPose": pose,
            "goalLm": "",
            "nodes": [label],
            "trajectory": [],
        }
        return {"ok": True, "route": route}

    def _route_payload_from_request(self, payload: dict[str, Any]) -> dict[str, Any]:
        route_payload = payload.get("route")
        if isinstance(route_payload, dict):
            return dict(route_payload)

        # ExecuteRoute transports route_json as a top-level object and appends
        # command/ownership fields.  A supplied MAPF route must reach the robot
        # planner unchanged; only goal-only requests are planned locally.
        protocol = str(payload.get("protocol") or payload.get("routeProtocol") or "").strip().lower()
        nodes = payload.get("nodes") or payload.get("routeNodes") or payload.get("route_nodes")
        if protocol in {"lm_route", "lm-route", "lmroute"} or isinstance(nodes, list):
            explicit_route = dict(payload)
            for key in ("ownerId", "owner_id", "commandId", "command_id", "order"):
                explicit_route.pop(key, None)
            return explicit_route

        goal_lm = str(payload.get("goalLm") or payload.get("targetLm") or "").strip()
        if not goal_lm:
            raise ValueError("goalLm is required")
        if self._plan_route_client is None or not self._service_available(self._plan_route_client, 0.05):
            return {
                "protocol": "lm_route",
                "goalLm": goal_lm,
                "startLm": str(payload.get("startLm") or "").strip(),
                "nodes": [item for item in [str(payload.get("startLm") or "").strip(), goal_lm] if item],
            }

        pose_payload = payload.get("startPose")
        if isinstance(pose_payload, dict):
            pose = {
                "x": float(pose_payload.get("x", 0.0) or 0.0),
                "y": float(pose_payload.get("y", 0.0) or 0.0),
                "yaw": float(pose_payload.get("yaw", 0.0) or 0.0),
            }
        else:
            status = self.status_robot_payload()
            pose = status.get("pose") if isinstance(status.get("pose"), dict) else None
            if not isinstance(pose, dict):
                raise ValueError("robot pose is not available yet")

        request = self._plan_route_client.srv_type.Request()
        request.goal_lm = goal_lm
        request.start_lm = str(payload.get("startLm") or "").strip()
        request.use_start_pose = True
        request.start_x = float(pose.get("x", 0.0) or 0.0)
        request.start_y = float(pose.get("y", 0.0) or 0.0)
        request.start_yaw = float(pose.get("yaw", 0.0) or 0.0)
        response = self._call_service(self._plan_route_client, request, "route plan")
        if not bool(response.ok):
            raise ValueError(str(response.error or "route planning failed"))
        try:
            route = json.loads(str(response.route_json or "{}"))
        except json.JSONDecodeError as exc:
            raise ValueError("route planner returned invalid JSON") from exc
        if not isinstance(route, dict):
            raise ValueError("route planner returned invalid route payload")
        return route

    def _goal_pose_payload(self, payload: dict[str, Any]) -> dict[str, float] | None:
        source = (
            payload.get("goalPose")
            or payload.get("goal_pose")
            or payload.get("targetPose")
            or payload.get("target_pose")
            or payload.get("pose")
        )
        if source is None and ("x" in payload or "y" in payload):
            source = payload
        if not isinstance(source, dict):
            return None
        if "x" not in source or "y" not in source:
            return None
        try:
            x = float(source.get("x", 0.0) or 0.0)
            y = float(source.get("y", 0.0) or 0.0)
            yaw = float(source.get("yaw", source.get("theta", 0.0)) or 0.0)
        except (TypeError, ValueError) as exc:
            raise ValueError("goalPose x/y/yaw must be numeric") from exc
        return {"x": x, "y": y, "yaw": yaw}

    def _current_pose_payload(self) -> dict[str, float]:
        status = self._message_to_robot_payload(self._latest_message())
        pose = status.get("pose") if isinstance(status.get("pose"), dict) else {}
        if not isinstance(pose, dict):
            pose = {}
        return {
            "x": float(pose.get("x", 0.0) or 0.0),
            "y": float(pose.get("y", 0.0) or 0.0),
            "yaw": float(pose.get("yaw", 0.0) or 0.0),
            "stampSec": time.time(),
        }

    def _yaw_from_quaternion(self, orientation: Any) -> float:
        if orientation is None:
            return 0.0
        x = float(getattr(orientation, "x", 0.0) or 0.0)
        y = float(getattr(orientation, "y", 0.0) or 0.0)
        z = float(getattr(orientation, "z", 0.0) or 0.0)
        w = float(getattr(orientation, "w", 1.0) or 1.0)
        return math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))

    def _set_route_paused(self, paused: bool, message: str) -> None:
        if self._route_pause_client is not None and self._service_available(self._route_pause_client, 0.05):
            request = self._route_pause_client.srv_type.Request()
            request.paused = bool(paused)
            request.message = str(message or "")
            response = self._call_service(self._route_pause_client, request, "route pause", timeout_sec=3.0)
            if not bool(response.ok):
                raise ValueError(str(response.error or "route pause failed"))
        else:
            if paused:
                self._publish_twist(0.0, 0.0)
            if self._route_pause_client is None:
                raise ValueError("route pause service is not configured")
            raise ValueError("route pause service is not available")
        self._navigation_paused = bool(paused)

    def _control_payload(self, robot: dict[str, Any]) -> dict[str, Any]:
        control = self._control_state_payload()
        state = str(robot.get("state") or "").upper()
        return {
            "control": control,
            "controlOwner": control["ownerId"],
            "controlOwnerName": control["ownerName"],
            "controlState": control["state"],
            "navigationPaused": bool(self._navigation_paused or state == "PAUSED"),
            "localizationConfirmed": bool(self._localization_confirmed),
            "relocationRequested": self._relocation_requested_at is not None,
        }

    def _control_state_payload(self) -> dict[str, Any]:
        with self._lock:
            self._expire_control_owner_locked()
            acquired_age = 0.0
            if self._control_acquired_at is not None:
                acquired_age = max(0.0, monotonic() - self._control_acquired_at)
            return {
                "state": "OWNED" if self._control_owner_id else "FREE",
                "ownerId": self._control_owner_id,
                "ownerName": self._control_owner_name,
                "leaseMs": self._control_lease_ms,
                "acquiredAgeSec": acquired_age,
            }

    def _ensure_control_owner(self, owner_id: str, *, action: str) -> None:
        owner = self._clean_owner_id(owner_id)
        with self._lock:
            self._expire_control_owner_locked()
            if not self._control_owner_id:
                raise ValueError(
                    f"cannot {action}: control is free; use Seize Control first"
                )
            if owner and owner == self._control_owner_id:
                self._control_acquired_at = self._control_acquired_at or monotonic()
                return
            if not owner:
                raise ValueError(f"cannot {action}: control is owned by {self._control_owner_name or self._control_owner_id}")
            raise ValueError(f"cannot {action}: control is owned by {self._control_owner_name or self._control_owner_id}")

    def _expire_control_owner_locked(self) -> None:
        if not self._control_owner_id or not self._control_lease_ms or self._control_acquired_at is None:
            return
        if (monotonic() - self._control_acquired_at) * 1000.0 <= float(self._control_lease_ms):
            return
        self._control_owner_id = ""
        self._control_owner_name = ""
        self._control_acquired_at = None
        self._control_lease_ms = 0

    @staticmethod
    def _clean_owner_id(owner_id: str) -> str:
        return re.sub(r"\s+", " ", str(owner_id or "").strip())[:120]

    @staticmethod
    def _covariance_from_json(value: str) -> list[float]:
        if not value:
            return []
        try:
            payload = json.loads(value)
        except json.JSONDecodeError:
            return []
        if not isinstance(payload, list) or len(payload) != 36:
            return []
        try:
            return [float(item) for item in payload]
        except (TypeError, ValueError):
            return []

    def _route_payload(self, robot: dict[str, Any]) -> dict[str, Any] | None:
        reported_route = robot.get("route")
        if isinstance(reported_route, dict):
            return dict(reported_route)
        target = str(robot.get("targetLm") or "").strip()
        route_id = str(robot.get("routeId") or "").strip()
        if not target and not route_id:
            return None
        return {
            "routeId": route_id or f"ros2-{target}",
            "goalLm": target,
            "nodes": [target] if target else [],
            "trajectory": [],
        }

    def _persist_event(self, previous: Any | None, current: Any) -> None:
        state = str(getattr(current, "state", "") or "UNKNOWN")
        message = str(getattr(current, "message", "") or "")
        previous_state = str(getattr(previous, "state", "") or "") if previous is not None else ""
        previous_message = str(getattr(previous, "message", "") or "") if previous is not None else ""
        if state == previous_state and message == previous_message:
            return
        level = "error" if state == "ERROR" else ("warn" if state in {"DISCONNECTED", "LOCALIZING"} else "info")
        self._events.append({"stamp": monotonic(), "level": level, "message": message or state})
        self._events = self._events[-120:]
