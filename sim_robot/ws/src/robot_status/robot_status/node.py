from __future__ import annotations

import math
from pathlib import Path
from time import monotonic

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_msgs.msg import ExecutorState, RobotStatus
from robot_msgs.srv import LoadRobotMap
from robot_planner import RobotTrajectoryPlanner

AMCL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


class RobotStatusNode(Node):
    def __init__(
        self,
        *,
        robot_id: str,
        route_planner: RobotTrajectoryPlanner,
        amcl_topic: str,
        odom_topic: str,
        cmd_vel_topic: str,
        status_topic: str,
        executor_status_topic: str,
        load_map_service_name: str,
    ) -> None:
        super().__init__("robot_status")
        self.robot_id = robot_id
        self.map_id = route_planner.map_id
        self.route_planner = route_planner
        self._map_frame = "map"
        self._base_frame = "base_link"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._pose: dict[str, float] | None = None
        self._pose_updated_at: float | None = None
        self._last_tf_pose_at: float | None = None
        self._velocity = {"linear": 0.0, "angular": 0.0}
        self._last_localization_fix_at: float | None = None
        self._last_manual_cmd_at: float | None = None
        self._executor_state = self._default_executor_state()

        self.status_pub = self.create_publisher(RobotStatus, status_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._on_amcl_pose, AMCL_QOS)
        self.create_subscription(Odometry, odom_topic, self._on_odom, ODOM_QOS)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_subscription(ExecutorState, executor_status_topic, self._on_executor_state, 20)
        self.create_service(LoadRobotMap, load_map_service_name, self._handle_load_map)
        self.create_timer(0.03, self._publish_status)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._last_localization_fix_at = monotonic()
        pose = message.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._set_pose(pose.position.x, pose.position.y, yaw)
        self._update_pose_from_tf()

    def _on_odom(self, message: Odometry) -> None:
        self._velocity = {
            "linear": float(message.twist.twist.linear.x),
            "angular": float(message.twist.twist.angular.z),
        }
        self._update_pose_from_tf()

    def _on_cmd_vel(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if abs(linear) <= 1e-4 and abs(angular) <= 1e-4:
            return
        self._last_manual_cmd_at = monotonic()

    def _on_executor_state(self, message: ExecutorState) -> None:
        self._executor_state = {
            "routeActive": bool(message.route_active),
            "routePaused": bool(message.route_paused),
            "state": str(message.state or ""),
            "message": str(message.message or ""),
            "targetLm": str(message.target_lm or ""),
            "currentEdgeId": str(message.current_edge_id or ""),
            "routeId": str(message.route_id or ""),
            "routeProgress": float(message.route_progress),
        }

    def _publish_status(self) -> None:
        self.route_planner.reload_params_from_disk()
        self._update_pose_from_tf()
        pose = self._pose
        pose_age = self._pose_age()
        pose_timeout = self._localization_timeout()
        amcl_age = self._localization_age()
        amcl_correction_timeout = self._amcl_correction_timeout()
        has_amcl_fix = self._last_localization_fix_at is not None
        pose_fresh = pose is not None and pose_age <= pose_timeout
        has_tf_pose = self._last_tf_pose_at is not None and pose_fresh
        localization_ok = pose is not None and pose_fresh and (has_amcl_fix or has_tf_pose)
        stationary = self._is_stationary()

        executor_state = self._executor_state
        route_active = bool(executor_state.get("routeActive"))
        route_paused = bool(executor_state.get("routePaused"))
        manual_active = self._manual_active() and not route_active
        state = str(executor_state.get("state") or "IDLE")
        message = str(executor_state.get("message") or "")

        if pose is None:
            if route_active or manual_active:
                state = "ERROR"
                message = "Waiting for map->base_link pose."
            else:
                state = "LOCALIZING"
                message = "Waiting for map->base_link pose."
        elif not pose_fresh:
            if route_active or manual_active or state == "ERROR":
                state = "ERROR"
                if not message:
                    message = f"Localization transform timeout: pose is stale for {pose_age:.2f}s"
            else:
                state = "LOCALIZING"
                message = f"Localization transform timeout: pose is stale for {pose_age:.2f}s"
        elif not has_amcl_fix and has_tf_pose:
            if manual_active:
                state = "MANUAL"
                message = "Manual control active. Using map->base_link TF pose."
            elif route_active:
                state = state or "EXECUTING_ROUTE"
                if not message:
                    target_lm = str(executor_state.get("targetLm") or "").strip()
                    message = (
                        f"Driving to {target_lm}. Using map->base_link TF pose."
                        if target_lm
                        else "Executing route. Using map->base_link TF pose."
                    )
            else:
                state = "IDLE"
                message = "Localized from map->base_link TF. Waiting for AMCL correction."
        elif route_active and route_paused:
            state = "PAUSED"
            if not message:
                target_lm = str(executor_state.get("targetLm") or "").strip()
                message = f"Route to {target_lm} paused." if target_lm else "Route paused."
        elif manual_active:
            state = "MANUAL"
            message = "Manual control active."
        elif route_active:
            state = state or "EXECUTING_ROUTE"
            if not message:
                target_lm = str(executor_state.get("targetLm") or "").strip()
                message = f"Driving to {target_lm}." if target_lm else "Executing route."
        elif amcl_age > amcl_correction_timeout:
            if state == "LOCALIZING":
                state = "IDLE"
            if stationary and self._accept_stale_pose_when_stationary():
                message = f"Localized. AMCL pose is cached while robot is stationary ({amcl_age:.2f}s old)."
            else:
                message = (
                    f"Localized. AMCL correction is {amcl_age:.2f}s old, "
                    "tracking continues on map->base_link TF."
                )
            if state not in {"ARRIVED", "ERROR"}:
                state = "IDLE"
        elif state in {"", "LOCALIZING"}:
            state = "IDLE"
            message = "Localized."

        nearest_name = ""
        if pose is not None:
            nearest, _ = self.route_planner.planner.nearest_landmark(pose["x"], pose["y"])
            nearest_name = nearest.name

        status = RobotStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.robot_id = self.robot_id
        status.map_id = self.map_id
        status.connected = True
        status.localization_ok = localization_ok
        status.localization_age_sec = float(amcl_age if math.isfinite(amcl_age) else 9999.0)
        status.state = state
        status.message = message
        status.target_lm = str(executor_state.get("targetLm") or "")
        status.nearest_lm = nearest_name
        status.current_edge_id = str(executor_state.get("currentEdgeId") or "")
        status.route_id = str(executor_state.get("routeId") or "")
        status.route_progress = float(executor_state.get("routeProgress", 0.0) or 0.0)
        if pose is not None:
            status.pose_x = float(pose["x"])
            status.pose_y = float(pose["y"])
            status.pose_yaw = float(pose["yaw"])
        status.linear_velocity = float(self._velocity.get("linear", 0.0) or 0.0)
        status.angular_velocity = float(self._velocity.get("angular", 0.0) or 0.0)
        self.status_pub.publish(status)

    def _default_executor_state(self) -> dict[str, object]:
        return {
            "routeActive": False,
            "routePaused": False,
            "state": "LOCALIZING",
            "message": "Waiting for amcl pose.",
            "targetLm": "",
            "currentEdgeId": "",
            "routeId": "",
            "routeProgress": 0.0,
        }

    def _handle_load_map(self, request, response):
        try:
            map_dir = Path(str(request.map_dir or "")).resolve()
            if not map_dir.is_dir():
                raise ValueError(f"map_dir does not exist: {map_dir}")
            self.route_planner.reload_map(map_dir)
            self.map_id = self.route_planner.map_id
            self._pose = None
            self._pose_updated_at = None
            self._last_tf_pose_at = None
            self._last_localization_fix_at = None
            self._executor_state = self._default_executor_state()
            response.ok = True
            response.error = ""
            response.map_name = str(request.map_name or self.map_id)
            response.map_dir = str(self.route_planner.map_dir)
            response.map_id = self.map_id
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.map_name = ""
            response.map_dir = ""
            response.map_id = ""
        return response

    def _set_pose(self, x: float, y: float, yaw: float) -> None:
        self._pose = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }
        self._pose_updated_at = monotonic()

    def _pose_age(self) -> float:
        if self._pose_updated_at is None:
            return float("inf")
        return max(0.0, monotonic() - self._pose_updated_at)

    def _manual_active(self) -> bool:
        if self._last_manual_cmd_at is None:
            return False
        params = self.route_planner.current_params()
        navigation = params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        timeout_ms = max(80, int(navigation.get("manual_control_timeout_ms", 300) or 300))
        return (monotonic() - self._last_manual_cmd_at) <= (timeout_ms / 1000.0)

    def _update_pose_from_tf(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        self._set_pose(float(translation.x), float(translation.y), float(yaw))
        self._last_tf_pose_at = monotonic()

    def _localization_timeout(self) -> float:
        params = self.route_planner.current_params()
        localization = params.get("localization", {})
        if not isinstance(localization, dict):
            localization = {}
        return max(0.1, float(localization.get("localization_timeout", 0.5) or 0.5))

    def _amcl_correction_timeout(self) -> float:
        params = self.route_planner.current_params()
        localization = params.get("localization", {})
        if not isinstance(localization, dict):
            localization = {}
        return max(0.5, float(localization.get("amcl_correction_timeout", 5.0) or 5.0))

    def _localization_age(self) -> float:
        if self._last_localization_fix_at is None:
            return float("inf")
        return max(0.0, monotonic() - self._last_localization_fix_at)

    def _accept_stale_pose_when_stationary(self) -> bool:
        params = self.route_planner.current_params()
        localization = params.get("localization", {})
        if not isinstance(localization, dict):
            localization = {}
        return bool(localization.get("accept_stale_pose_when_stationary", True))

    def _is_stationary(self) -> bool:
        params = self.route_planner.current_params()
        localization = params.get("localization", {})
        if not isinstance(localization, dict):
            localization = {}
        linear_epsilon = max(
            0.001,
            float(localization.get("stationary_linear_velocity_epsilon", 0.02) or 0.02),
        )
        angular_epsilon = max(
            0.001,
            float(localization.get("stationary_angular_velocity_epsilon", 0.05) or 0.05),
        )
        linear = abs(float(self._velocity.get("linear", 0.0) or 0.0))
        angular = abs(float(self._velocity.get("angular", 0.0) or 0.0))
        return linear <= linear_epsilon and angular <= angular_epsilon
