"""Convert ROS messages and provide publisher/service primitives."""

from __future__ import annotations

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
        payload = {
            "robotId": str(getattr(message, "robot_id", "") or self.robot_name),
            "mapId": str(getattr(message, "map_id", "") or ""),
            "connected": connected,
            "localizationOk": localization_ok,
            "localizationAgeSec": float(getattr(message, "localization_age_sec", 9999.0)),
            "statusAgeSec": float(self._status_age_sec() or 0.0),
            "state": state,
            "message": str(getattr(message, "message", "") or ""),
            "targetLm": str(getattr(message, "target_lm", "") or ""),
            "nearestLm": str(getattr(message, "nearest_lm", "") or ""),
            "currentEdgeId": str(getattr(message, "current_edge_id", "") or ""),
            "routeId": str(getattr(message, "route_id", "") or ""),
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
            "battery": {
                "level": float(getattr(message, "battery_level", 0.0)),
                "voltage": float(getattr(message, "battery_voltage", 0.0)),
                "current": float(getattr(message, "battery_current", 0.0)),
                "temperature": float(getattr(message, "battery_temperature", 0.0)),
                "charging": bool(getattr(message, "battery_charging", False)),
            },
        }
        return payload

__all__ = ["RosRuntimeMessageServiceMixin"]
