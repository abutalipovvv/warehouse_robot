"""Convert ROS messages and provide publisher/service primitives."""

from __future__ import annotations

import math
from time import monotonic, sleep
from typing import Any

class RosRuntimeMessageServiceMixin:
    """Convert ROS messages and provide publisher/service primitives."""

    def _publish_twist(self, linear: float, angular: float) -> None:
        if self._cmd_vel_pub is None or self._twist_type is None:
            raise ValueError(self._error or "ROS2 runtime is not available")
        message = self._twist_type()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._cmd_vel_pub.publish(message)

    def _publish_go_to_lm(self, data: str) -> None:
        if self._go_to_lm_pub is None or self._string_type is None:
            raise ValueError(self._error or "ROS2 runtime is not available")
        message = self._string_type()
        message.data = str(data)
        self._go_to_lm_pub.publish(message)

    def _service_available(self, client: Any, timeout_sec: float = 0.05) -> bool:
        try:
            return bool(client.wait_for_service(timeout_sec=max(0.0, float(timeout_sec))))
        except Exception:
            return False

    def _call_service(self, client: Any, request: Any, service_label: str, *, timeout_sec: float = 3.0) -> Any:
        if not client.wait_for_service(timeout_sec=min(1.0, max(0.05, float(timeout_sec)))):
            raise ValueError(f"{service_label} service is not available")
        future = client.call_async(request)
        deadline = monotonic() + max(0.2, float(timeout_sec))
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            raise ValueError(f"{service_label} service timed out")
        if future.exception() is not None:
            raise ValueError(f"{service_label} service failed: {future.exception()}")
        response = future.result()
        if response is None:
            raise ValueError(f"{service_label} returned no response")
        return response

    def _message_to_robot_payload(self, message: Any | None) -> dict[str, Any]:
        if message is None:
            return {
                "robotId": self.robot_name,
                "mapId": "",
                "connected": False,
                "localizationOk": False,
                "localizationAgeSec": 9999.0,
                "state": "DISCONNECTED",
            "message": self._error or f"Waiting for {self.status_topic}.",
                "targetLm": "",
                "nearestLm": "",
                "currentEdgeId": "",
                "routeId": "",
                "routeProgress": 0.0,
                "pose": None,
                "velocity": {"linear": 0.0, "angular": 0.0},
                "acceleration": {"linear": 0.0, "angular": 0.0},
                "battery": None,
            }

        connected = bool(getattr(message, "connected", False))
        localization_ok = bool(getattr(message, "localization_ok", False))
        pose = {
            "x": float(getattr(message, "pose_x", 0.0)),
            "y": float(getattr(message, "pose_y", 0.0)),
            "yaw": float(getattr(message, "pose_yaw", 0.0)),
        } if connected and localization_ok else None
        route_nodes = [
            str(node)
            for node in list(getattr(message, "route_nodes", []) or [])
            if str(node)
        ]
        target_lm = str(getattr(message, "target_lm", "") or "")
        route_id = str(getattr(message, "route_id", "") or "")
        battery_level = float(getattr(message, "battery_level", float("nan")))
        battery = None
        if math.isfinite(battery_level) and battery_level >= 0.0:
            voltage = float(getattr(message, "battery_voltage", float("nan")))
            current = float(getattr(message, "battery_current", float("nan")))
            temperature = float(getattr(message, "battery_temperature", float("nan")))
            battery = {
                "level": battery_level,
                "voltage": voltage if math.isfinite(voltage) else 0.0,
                "current": current if math.isfinite(current) else 0.0,
                "temperature": temperature if math.isfinite(temperature) else 0.0,
                "charging": bool(getattr(message, "battery_charging", False)),
            }
        state = str(getattr(message, "state", "") or "UNKNOWN")
        route_active = bool(
            getattr(
                message,
                "route_active",
                state.upper() in {"EXECUTING_ROUTE", "MOVING", "WAITING", "PAUSED"},
            )
        )
        payload = {
            "robotId": str(getattr(message, "robot_id", "") or self.robot_name),
            "mapId": str(getattr(message, "map_id", "") or ""),
            "connected": connected,
            "localizationOk": localization_ok,
            "localizationAgeSec": float(getattr(message, "localization_age_sec", 9999.0)),
            "state": state,
            "message": str(getattr(message, "message", "") or ""),
            "targetLm": target_lm,
            "nearestLm": str(getattr(message, "nearest_lm", "") or ""),
            "currentEdgeId": str(getattr(message, "current_edge_id", "") or ""),
            "routeId": route_id,
            "routeProgress": float(getattr(message, "route_progress", 0.0)),
            "pose": pose,
            "velocity": {
                "linear": float(getattr(message, "linear_velocity", 0.0)),
                "angular": float(getattr(message, "angular_velocity", 0.0)),
            },
            "acceleration": {
                "linear": float(getattr(message, "linear_acceleration", 0.0)),
                "angular": float(getattr(message, "angular_acceleration", 0.0)),
            },
            "battery": battery,
        }
        if route_active or route_id or route_nodes or target_lm:
            payload["route"] = {
                "active": route_active,
                "routeId": route_id,
                "goalLm": target_lm,
                "finalGoalLm": str(getattr(message, "final_goal_lm", "") or target_lm),
                "nodes": route_nodes or ([target_lm] if target_lm else []),
                "progress": float(getattr(message, "route_progress", 0.0)),
                "paused": bool(getattr(message, "route_paused", False)),
                "trajectory": [],
            }
        return payload
