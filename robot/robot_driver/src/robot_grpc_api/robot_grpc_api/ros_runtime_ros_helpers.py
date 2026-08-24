"""Convert ROS messages and provide publisher/service primitives."""

from __future__ import annotations

import json
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

    def _publish_motion_mode(self, mode: str, reason: str = "") -> None:
        if self._motion_mode_pub is None or self._string_type is None:
            return
        message = self._string_type()
        message.data = json.dumps(
            {
                "mode": str(mode or "IDLE").strip().upper(),
                "reason": str(reason or ""),
            },
            separators=(",", ":"),
        )
        self._motion_mode_pub.publish(message)

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
            status_age = self._status_age_sec()
            if status_age is None:
                detail = f"Waiting for {self.status_topic}."
            else:
                detail = f"Robot status is stale ({status_age:.1f}s). Waiting for {self.status_topic}."
            return {
                "robotId": self.robot_name,
                "mapId": "",
                "connected": False,
                "localizationOk": False,
                "localizationAgeSec": 9999.0,
                "statusAgeSec": 9999.0 if status_age is None else status_age,
                "state": "DISCONNECTED",
                "message": self._error or detail,
                "targetLm": "",
                "nearestLm": "",
                "currentEdgeId": "",
                "routeId": "",
                "routeProgress": 0.0,
                "tracking": {},
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
        state = self._message_state(message)
        route_nodes = [
            str(node)
            for node in list(getattr(message, "route_nodes", []) or [])
            if str(node)
        ]
        final_goal_lm = str(getattr(message, "final_goal_lm", "") or "")
        route_active = bool(
            getattr(
                message,
                "route_active",
                state in {"EXECUTING_ROUTE", "MOVING", "WAITING", "PAUSED"},
            )
        )
        route_id = str(getattr(message, "route_id", "") or "")
        target_lm = str(getattr(message, "target_lm", "") or "")
        route = None
        if route_active or route_id or route_nodes or target_lm:
            route = {
                "active": route_active,
                "routeId": route_id,
                "goalLm": target_lm,
                "finalGoalLm": final_goal_lm or target_lm,
                "nodes": route_nodes or ([target_lm] if target_lm else []),
                "progress": float(getattr(message, "route_progress", 0.0)),
                "paused": bool(getattr(message, "route_paused", False)),
                "trajectory": [],
            }
        payload = {
            "robotId": str(getattr(message, "robot_id", "") or self.robot_name),
            "mapId": str(getattr(message, "map_id", "") or ""),
            "connected": connected,
            "localizationOk": localization_ok,
            "localizationAgeSec": float(getattr(message, "localization_age_sec", 9999.0)),
            "statusAgeSec": float(self._status_age_sec() or 0.0),
            "state": state,
            "message": str(getattr(message, "message", "") or ""),
            "targetLm": target_lm,
            "nearestLm": str(getattr(message, "nearest_lm", "") or ""),
            "currentEdgeId": str(getattr(message, "current_edge_id", "") or ""),
            "routeId": route_id,
            "routeProgress": float(getattr(message, "route_progress", 0.0)),
            "tracking": {
                "crossTrackError": float(
                    getattr(message, "cross_track_error", 0.0)
                ),
                "headingError": float(getattr(message, "heading_error", 0.0)),
                "remainingDistance": float(
                    getattr(message, "remaining_distance", 0.0)
                ),
                "goalPositionError": float(
                    getattr(message, "goal_position_error", 0.0)
                ),
                "goalYawError": float(
                    getattr(message, "goal_yaw_error", 0.0)
                ),
                "commandedLinear": float(
                    getattr(message, "commanded_linear", 0.0)
                ),
                "commandedAngular": float(
                    getattr(message, "commanded_angular", 0.0)
                ),
                "maxCrossTrackError": float(
                    getattr(message, "max_cross_track_error", 0.0)
                ),
                "meanCrossTrackError": float(
                    getattr(message, "mean_cross_track_error", 0.0)
                ),
                "samples": int(getattr(message, "tracking_samples", 0)),
                "arrivalStableCycles": int(
                    getattr(message, "arrival_stable_cycles", 0)
                ),
                "arrivalRequiredCycles": int(
                    getattr(message, "arrival_required_cycles", 0)
                ),
            },
            "pose": pose,
            "velocity": {
                "linear": float(getattr(message, "linear_velocity", 0.0)),
                "angular": float(getattr(message, "angular_velocity", 0.0)),
            },
            "acceleration": {
                "linear": float(getattr(message, "linear_acceleration", 0.0)),
                "angular": float(getattr(message, "angular_acceleration", 0.0)),
            },
            "battery": self._battery_payload(message),
        }
        if route is not None:
            payload["route"] = route
        return payload

    @staticmethod
    def _battery_payload(message: Any) -> dict[str, float | bool] | None:
        level = float(getattr(message, "battery_level", float("nan")))
        if not math.isfinite(level) or level < 0.0:
            return None
        def finite_metric(name: str) -> float:
            value = float(getattr(message, name, float("nan")))
            return value if math.isfinite(value) else 0.0

        return {
            "level": level,
            "voltage": finite_metric("battery_voltage"),
            "current": finite_metric("battery_current"),
            "temperature": finite_metric("battery_temperature"),
            "charging": bool(getattr(message, "battery_charging", False)),
        }

__all__ = ["RosRuntimeMessageServiceMixin"]
