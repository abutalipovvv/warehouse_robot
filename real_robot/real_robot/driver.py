from __future__ import annotations

import json
import math
import time
from time import monotonic
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from robot_msgs.msg import ExecutorState, RobotStatus

from .robokit_protocol import (
    CONFIG_PORT,
    CONTROL_PORT,
    NAVIGATION_PORT,
    STATUS_PORT,
    RobokitApiError,
    RobokitClient,
    RobokitError,
)


STATUS_KEYS = [
    "mode",
    "x",
    "y",
    "angle",
    "confidence",
    "conidence",
    "current_station",
    "last_station",
    "vx",
    "vy",
    "w",
    "r_vx",
    "r_vy",
    "r_w",
    "is_stop",
    "blocked",
    "block_reason",
    "slowed",
    "slow_reason",
    "battery_level",
    "battery_temp",
    "charging",
    "voltage",
    "current",
    "max_charge_voltage",
    "max_charge_current",
    "manual_charge",
    "auto_charge",
    "battery_cycle",
    "emergency",
    "driver_emc",
    "task_status",
    "task_type",
    "target_id",
    "target_point",
    "finished_path",
    "unfinished_path",
    "reloc_status",
    "loadmap_status",
    "fatals",
    "errors",
    "warnings",
    "notices",
    "current_map",
    "current_map_md5",
    "move_status_info",
    "vehicle_id",
]

OPTIONAL_GOTO_FIELDS = {
    "angle",
    "method",
    "operation",
    "direction",
    "recognize",
    "layer",
    "jack_height",
    "start_height",
    "end_height",
    "fork_dist",
    "use_pgv",
    "max_speed",
    "max_wspeed",
    "max_acc",
    "max_wacc",
    "duration",
    "orientation",
    "spin",
    "delay",
    "rot_dir",
    "reach_list",
    "reach_angle",
    "skill_name",
}

TASK_STATUS_NAMES = {
    0: "NONE",
    1: "WAITING",
    2: "RUNNING",
    3: "SUSPENDED",
    4: "COMPLETED",
    5: "FAILED",
    6: "CANCELED",
}

BLOCK_REASON_NAMES = {
    0: "ultrasonic",
    1: "laser",
    2: "fallingdown",
    3: "collision",
    4: "infrared",
    5: "lock",
    6: "dynamic obstacle",
    7: "virtual laser",
    8: "3D camera",
}


def _as_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _as_int(value: Any, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "on"}:
        return True
    if text in {"false", "0", "no", "off"}:
        return False
    return default


def _clean_text(value: Any) -> str:
    if value is None:
        return ""
    text = str(value).strip()
    if text.lower() in {"", "none", "null", "nan"}:
        return ""
    return text


def _clean_station(value: Any) -> str:
    text = _clean_text(value)
    if text.upper() == "SELF_POSITION":
        return ""
    return text


def _as_string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [_clean_text(item) for item in value if _clean_text(item)]


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(float(value), minimum), maximum)


class RobotDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_driver")

        self.robot_ip = str(self._param("robot_ip", "192.168.192.5")).strip()
        self.robot_id = str(self._param("robot_id", "robot1")).strip() or "robot1"
        self.map_id = str(self._param("map_id", "")).strip()
        self.status_rate_hz = max(0.5, float(self._param("status_rate_hz", 10.0)))
        self.command_duration_ms = max(50, int(self._param("command_duration_ms", 350)))
        self.socket_timeout_sec = max(0.1, float(self._param("socket_timeout_sec", 0.8)))
        self.localization_confidence_min = _clamp(
            float(self._param("localization_confidence_min", 0.2)),
            0.0,
            1.0,
        )
        self.acquire_control_on_start = bool(self._param("acquire_control_on_start", False))
        self.acquire_control_before_command = bool(self._param("acquire_control_before_command", False))
        self.release_control_on_shutdown = bool(self._param("release_control_on_shutdown", True))
        self.control_nick_name = str(self._param("control_nick_name", "warehouse_robot_driver")).strip()
        self.default_source_id = str(self._param("default_source_id", "")).strip()
        self.odom_frame_id = str(self._param("odom_frame_id", "map")).strip() or "map"
        self.base_frame_id = str(self._param("base_frame_id", "base_link")).strip() or "base_link"

        status_port = int(self._param("status_port", STATUS_PORT))
        control_port = int(self._param("control_port", CONTROL_PORT))
        navigation_port = int(self._param("navigation_port", NAVIGATION_PORT))
        config_port = int(self._param("config_port", CONFIG_PORT))

        odom_topic = str(self._param("odom_topic", "/odom"))
        cmd_vel_topic = str(self._param("cmd_vel_topic", "/cmd_vel"))
        status_topic = str(self._param("status_topic", "/robot_status"))
        bms_topic = str(self._param("bms_topic", "/bms"))
        go_to_lm_topic = str(self._param("go_to_lm_topic", "/go_to_lm"))
        navigate_status_topic = str(self._param("navigate_status_topic", "/navigate_status"))

        self.client = RobokitClient(
            self.robot_ip,
            status_port=status_port,
            control_port=control_port,
            navigation_port=navigation_port,
            config_port=config_port,
            timeout_sec=self.socket_timeout_sec,
        )
        self._control_acquired = False
        self._last_error_log_at: dict[str, float] = {}
        self._latest_payload: dict[str, Any] = {}
        self._last_status_at: float | None = None
        self._last_cmd_vel_at: float | None = None
        self._last_current_station = ""
        self._last_station = ""
        self._active_task_id = ""
        self._last_target_lm = ""
        self._last_nav_command_at: float | None = None
        self._last_nav_result: tuple[int, str] | None = None
        self._shutdown_done = False

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.status_pub = self.create_publisher(RobotStatus, status_topic, 10)
        self.bms_pub = self.create_publisher(BatteryState, bms_topic, 10)
        self.navigate_status_pub = self.create_publisher(ExecutorState, navigate_status_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(String, go_to_lm_topic, self._on_go_to_lm, 10)
        self.create_timer(1.0 / self.status_rate_hz, self._poll_status)

        if self.acquire_control_on_start:
            self._ensure_control("startup")

        self.get_logger().info(
            f"AIvison robot driver connected to {self.robot_ip}; "
            f"status_rate={self.status_rate_hz:.1f} Hz"
        )

    def _param(self, name: str, default: Any) -> Any:
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return default if value is None else value

    def destroy_node(self) -> bool:
        self._shutdown()
        return super().destroy_node()

    def _shutdown(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True
        if self.release_control_on_shutdown and self._control_acquired:
            try:
                self.client.release_control()
            except RobokitError as exc:
                self.get_logger().warn(f"failed to release AIvison control: {exc}")
        self.client.close()

    def _poll_status(self) -> None:
        try:
            payload = self.client.read_all_status(STATUS_KEYS)
        except RobokitError as exc:
            self._log_warn_throttled("status", f"AIvison status request failed: {exc}")
            self._publish_disconnected(str(exc))
            return

        self._latest_payload = payload
        self._last_status_at = monotonic()
        self._last_current_station = _clean_station(payload.get("current_station"))
        self._last_station = _clean_station(payload.get("last_station"))
        self._publish_from_payload(payload)

    def _publish_disconnected(self, reason: str) -> None:
        stamp = self.get_clock().now().to_msg()
        status = RobotStatus()
        status.stamp = stamp
        status.robot_id = self.robot_id
        status.map_id = self.map_id
        status.connected = False
        status.localization_ok = False
        status.localization_age_sec = 9999.0
        status.state = "DISCONNECTED"
        status.message = f"AIvison API disconnected: {reason}"
        status.target_lm = self._last_target_lm
        status.nearest_lm = self._last_current_station or self._last_station
        self.status_pub.publish(status)

        nav = ExecutorState()
        nav.stamp = stamp
        nav.robot_id = self.robot_id
        nav.map_id = self.map_id
        nav.route_active = False
        nav.state = "DISCONNECTED"
        nav.message = status.message
        nav.target_lm = self._last_target_lm
        nav.route_id = self._active_task_id
        self.navigate_status_pub.publish(nav)

    def _publish_from_payload(self, payload: dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(payload, stamp)
        self._publish_bms(payload, stamp)

        state, message, route_active, route_progress = self._derive_state(payload)
        map_id = _clean_text(payload.get("current_map")) or self.map_id
        target_lm = _clean_text(payload.get("target_id")) or self._last_target_lm
        task_status = _as_int(payload.get("task_status"), 0)
        route_id = self._active_task_id if route_active else ""
        nearest_lm = self._last_current_station or self._last_station

        status = RobotStatus()
        status.stamp = stamp
        status.robot_id = self.robot_id
        status.map_id = map_id
        status.connected = True
        status.localization_ok = self._localization_ok(payload)
        status.localization_age_sec = self._localization_age_sec()
        status.state = state
        status.message = message
        status.target_lm = target_lm
        status.nearest_lm = nearest_lm
        status.current_edge_id = self._current_edge_id(payload)
        status.route_id = route_id
        status.route_progress = float(route_progress)
        if self._has_pose(payload):
            status.pose_x = _as_float(payload.get("x"))
            status.pose_y = _as_float(payload.get("y"))
            status.pose_yaw = self._pose_yaw(payload)
        status.linear_velocity = _as_float(payload.get("vx"))
        status.angular_velocity = _as_float(payload.get("w"))
        self.status_pub.publish(status)

        nav = ExecutorState()
        nav.stamp = stamp
        nav.robot_id = self.robot_id
        nav.map_id = map_id
        nav.route_active = route_active
        nav.state = state
        nav.message = message
        nav.target_lm = target_lm
        nav.current_edge_id = status.current_edge_id
        nav.route_id = route_id
        nav.route_progress = float(route_progress)
        self.navigate_status_pub.publish(nav)

        if task_status in {4, 5, 6}:
            self._last_nav_result = (task_status, target_lm)
            self._active_task_id = ""

    def _publish_odom(self, payload: dict[str, Any], stamp: Any) -> None:
        if not self._has_pose(payload):
            return
        yaw = self._pose_yaw(payload)
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = _as_float(payload.get("x"))
        odom.pose.pose.position.y = _as_float(payload.get("y"))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = _as_float(payload.get("vx"))
        odom.twist.twist.linear.y = _as_float(payload.get("vy"))
        odom.twist.twist.angular.z = _as_float(payload.get("w"))
        self.odom_pub.publish(odom)

    def _publish_bms(self, payload: dict[str, Any], stamp: Any) -> None:
        if "battery_level" not in payload:
            return
        percentage = _clamp(_as_float(payload.get("battery_level"), 0.0), 0.0, 1.0)
        battery = BatteryState()
        battery.header.stamp = stamp
        battery.percentage = percentage
        battery.temperature = _as_float(payload.get("battery_temp"))
        battery.voltage = _as_float(payload.get("voltage"))
        battery.current = _as_float(payload.get("current"))
        battery.present = True
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        if _as_bool(payload.get("charging")):
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif percentage >= 0.995:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.bms_pub.publish(battery)

    def _on_cmd_vel(self, message: Twist) -> None:
        vx = float(message.linear.x)
        vy = float(message.linear.y)
        w = float(message.angular.z)
        if self.acquire_control_before_command:
            self._ensure_control("cmd_vel")
        try:
            self.client.send_motion(vx=vx, vy=vy, w=w, duration_ms=self.command_duration_ms)
            if abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(w) > 1e-4:
                self._last_cmd_vel_at = monotonic()
        except RobokitError as exc:
            self._log_warn_throttled("cmd_vel", f"AIvison cmd_vel failed: {exc}")

    def _on_go_to_lm(self, message: String) -> None:
        raw = str(message.data or "").strip()
        if not raw:
            self.get_logger().warn("/go_to_lm ignored: empty target")
            return
        if raw.lower() in {"cancel", "stop"}:
            self._cancel_navigation()
            return

        try:
            payload = self._go_to_lm_payload(raw)
        except ValueError as exc:
            self.get_logger().warn(f"/go_to_lm ignored: {exc}")
            return

        if self.acquire_control_before_command:
            self._ensure_control("go_to_lm")
        try:
            self.client.goto_target(payload)
        except RobokitApiError as exc:
            self._log_warn_throttled("go_to_lm", f"AIvison go_to_lm rejected: {exc}")
            return
        except RobokitError as exc:
            self._log_warn_throttled("go_to_lm", f"AIvison go_to_lm failed: {exc}")
            return

        self._active_task_id = _clean_text(payload.get("task_id"))
        self._last_target_lm = _clean_text(payload.get("id"))
        self._last_nav_command_at = monotonic()
        self.get_logger().info(
            f"sent AIvison navigation task {self._active_task_id}: "
            f"{payload.get('source_id')} -> {payload.get('id')}"
        )

    def _go_to_lm_payload(self, raw: str) -> dict[str, Any]:
        if raw.startswith("{"):
            try:
                decoded = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError("invalid JSON payload") from exc
            if not isinstance(decoded, dict):
                raise ValueError("JSON payload must be an object")
            source = decoded
        else:
            source = {"id": raw}

        target = (
            _clean_text(source.get("id"))
            or _clean_text(source.get("target_id"))
            or _clean_text(source.get("target_lm"))
            or _clean_text(source.get("targetLm"))
        )
        if not target:
            raise ValueError("target LM is required")

        source_id = (
            _clean_text(source.get("source_id"))
            or _clean_text(source.get("source_lm"))
            or _clean_text(source.get("sourceLm"))
            or self.default_source_id
            or self._last_current_station
            or self._last_station
            or "SELF_POSITION"
        )
        task_id = _clean_text(source.get("task_id")) or _clean_text(source.get("taskId"))
        if not task_id:
            task_id = f"{self.robot_id}-{time.time_ns()}"

        payload: dict[str, Any] = {
            "source_id": source_id,
            "id": target,
            "task_id": task_id,
        }
        for key in OPTIONAL_GOTO_FIELDS:
            if key in source:
                payload[key] = source[key]
        return payload

    def _cancel_navigation(self) -> None:
        try:
            self.client.cancel_navigation()
        except RobokitError as exc:
            self._log_warn_throttled("cancel_nav", f"AIvison cancel navigation failed: {exc}")
            return
        self._active_task_id = ""
        self.get_logger().info("sent AIvison navigation cancel")

    def _ensure_control(self, reason: str) -> None:
        if self._control_acquired:
            return
        try:
            self.client.acquire_control(self.control_nick_name)
            self._control_acquired = True
        except RobokitError as exc:
            self._log_warn_throttled(
                "control",
                f"failed to acquire AIvison control for {reason}: {exc}",
            )

    def _derive_state(self, payload: dict[str, Any]) -> tuple[str, str, bool, float]:
        task_status = _as_int(payload.get("task_status"), 0)
        target_lm = _clean_text(payload.get("target_id")) or self._last_target_lm
        route_active = self._route_active(task_status)
        route_progress = self._route_progress(payload, task_status)

        error_summary = self._alarm_summary(payload, ("fatals", "errors"))
        if _as_bool(payload.get("emergency")):
            return "ERROR", "Emergency stop is active.", route_active, route_progress
        if _as_bool(payload.get("driver_emc")):
            return "ERROR", "Motor driver emergency stop is active.", route_active, route_progress
        if error_summary:
            return "ERROR", error_summary, route_active, route_progress

        loadmap_status = _as_int(payload.get("loadmap_status"), 1)
        if loadmap_status == 0:
            return "ERROR", "Map loading failed.", route_active, route_progress
        if loadmap_status == 2:
            return "LOCALIZING", "Map is loading.", route_active, route_progress

        reloc_status = _as_int(payload.get("reloc_status"), 1)
        if reloc_status == 0:
            return "ERROR", "Localization failed.", route_active, route_progress
        if reloc_status == 2:
            return "LOCALIZING", "Relocation in progress.", route_active, route_progress

        if not self._localization_ok(payload):
            confidence = self._confidence(payload)
            return (
                "LOCALIZING",
                f"Waiting for valid localization, confidence={confidence:.3f}.",
                route_active,
                route_progress,
            )

        move_info = _clean_text(payload.get("move_status_info"))
        if task_status == 1:
            message = f"Navigation waiting for {target_lm}." if target_lm else "Navigation waiting."
            return "EXECUTING_ROUTE", move_info or message, True, route_progress
        if task_status == 2:
            if _as_bool(payload.get("blocked")):
                reason = BLOCK_REASON_NAMES.get(_as_int(payload.get("block_reason"), -1), "unknown")
                return "EXECUTING_ROUTE", f"Navigation blocked by {reason}.", True, route_progress
            message = f"Navigating to {target_lm}." if target_lm else "Navigation running."
            return "EXECUTING_ROUTE", move_info or message, True, route_progress
        if task_status == 3:
            message = f"Navigation to {target_lm} is suspended." if target_lm else "Navigation suspended."
            return "EXECUTING_ROUTE", move_info or message, True, route_progress
        if task_status == 4:
            message = f"Arrived at {target_lm}." if target_lm else "Navigation completed."
            return "ARRIVED", move_info or message, False, 1.0
        if task_status == 5:
            message = f"Navigation to {target_lm} failed." if target_lm else "Navigation failed."
            return "ERROR", move_info or message, False, route_progress
        if task_status == 6:
            message = f"Navigation to {target_lm} canceled." if target_lm else "Navigation canceled."
            return "IDLE", move_info or message, False, route_progress

        if self._manual_active():
            return "MANUAL", "Manual velocity command active.", False, 0.0
        if _as_bool(payload.get("blocked")):
            reason = BLOCK_REASON_NAMES.get(_as_int(payload.get("block_reason"), -1), "unknown")
            return "IDLE", f"Robot is blocked by {reason}.", False, 0.0
        warning_summary = self._alarm_summary(payload, ("warnings",))
        return "IDLE", warning_summary or "Robot is ready.", False, 0.0

    def _route_active(self, task_status: int) -> bool:
        if task_status in {1, 2, 3}:
            return True
        if task_status == 0 and self._active_task_id and self._last_nav_command_at is not None:
            return (monotonic() - self._last_nav_command_at) <= 3.0
        return False

    def _route_progress(self, payload: dict[str, Any], task_status: int) -> float:
        if task_status == 4:
            return 1.0
        finished = _as_string_list(payload.get("finished_path"))
        unfinished = _as_string_list(payload.get("unfinished_path"))
        total = len(finished) + len(unfinished)
        if total > 0:
            return _clamp(len(finished) / total, 0.0, 1.0)
        if self._route_active(task_status):
            return 0.0
        return 0.0

    def _current_edge_id(self, payload: dict[str, Any]) -> str:
        finished = _as_string_list(payload.get("finished_path"))
        unfinished = _as_string_list(payload.get("unfinished_path"))
        if finished and unfinished:
            return f"{finished[-1]}->{unfinished[0]}"
        if unfinished:
            start = self._last_current_station or self._last_station or "SELF_POSITION"
            return f"{start}->{unfinished[0]}"
        return ""

    def _localization_ok(self, payload: dict[str, Any]) -> bool:
        if not self._has_pose(payload):
            return False
        confidence = self._confidence(payload)
        if confidence < self.localization_confidence_min:
            return False
        loadmap_status = _as_int(payload.get("loadmap_status"), 1)
        reloc_status = _as_int(payload.get("reloc_status"), 1)
        return loadmap_status == 1 and reloc_status not in {0, 2}

    def _localization_age_sec(self) -> float:
        if self._last_status_at is None:
            return 9999.0
        return max(0.0, monotonic() - self._last_status_at)

    def _has_pose(self, payload: dict[str, Any]) -> bool:
        return payload.get("x") is not None and payload.get("y") is not None and (
            payload.get("angle") is not None or payload.get("yaw") is not None
        )

    def _pose_yaw(self, payload: dict[str, Any]) -> float:
        if payload.get("angle") is not None:
            return _as_float(payload.get("angle"))
        return _as_float(payload.get("yaw"))

    def _confidence(self, payload: dict[str, Any]) -> float:
        if payload.get("confidence") is not None:
            return _as_float(payload.get("confidence"), 0.0)
        if payload.get("conidence") is not None:
            return _as_float(payload.get("conidence"), 0.0)
        return 1.0

    def _manual_active(self) -> bool:
        if self._last_cmd_vel_at is None:
            return False
        return (monotonic() - self._last_cmd_vel_at) <= (self.command_duration_ms / 1000.0)

    def _alarm_summary(self, payload: dict[str, Any], fields: tuple[str, ...]) -> str:
        parts: list[str] = []
        for field in fields:
            value = payload.get(field)
            if not isinstance(value, list):
                continue
            for alarm in value:
                text = self._format_alarm(alarm)
                if text:
                    parts.append(text)
                if len(parts) >= 3:
                    break
            if len(parts) >= 3:
                break
        if not parts:
            return ""
        prefix = "Robot alarm" if len(parts) == 1 else "Robot alarms"
        return f"{prefix}: {'; '.join(parts)}"

    def _format_alarm(self, alarm: Any) -> str:
        if isinstance(alarm, dict):
            for key in ("desc", "description", "message", "msg", "err_msg", "name", "code", "id"):
                text = _clean_text(alarm.get(key))
                if text:
                    return text
            return _clean_text(alarm)
        return _clean_text(alarm)

    def _log_warn_throttled(self, key: str, message: str, period_sec: float = 5.0) -> None:
        now = monotonic()
        last = self._last_error_log_at.get(key, 0.0)
        if now - last < period_sec:
            return
        self._last_error_log_at[key] = now
        self.get_logger().warn(message)


def main() -> None:
    rclpy.init(args=None)
    node = RobotDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
