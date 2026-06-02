from __future__ import annotations

import math
from time import monotonic
from typing import Any

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_msgs.msg import RobotStatus
from robot_planner import RobotRuntime, RobotTrajectoryPlanner

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
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        amcl_topic: str,
        odom_topic: str,
        cmd_vel_topic: str,
        status_topic: str,
    ) -> None:
        super().__init__("robot_status")
        self.runtime = runtime
        self.route_planner = route_planner
        self._map_frame = "map"
        self._base_frame = "base_link"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._last_localization_fix_at: float | None = None
        self._last_manual_cmd_at: float | None = None
        self._last_manual_linear = 0.0
        self._last_manual_angular = 0.0
        self.status_pub = self.create_publisher(RobotStatus, status_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._on_amcl_pose, AMCL_QOS)
        self.create_subscription(Odometry, odom_topic, self._on_odom, ODOM_QOS)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 20)
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
        self.runtime.set_pose(pose.position.x, pose.position.y, yaw)
        self._update_pose_from_tf()

    def _on_odom(self, message: Odometry) -> None:
        self.runtime.set_velocity(
            linear=message.twist.twist.linear.x,
            angular=message.twist.twist.angular.z,
        )
        self._update_pose_from_tf()

    def _on_cmd_vel(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if abs(linear) <= 1e-4 and abs(angular) <= 1e-4:
            self._last_manual_linear = 0.0
            self._last_manual_angular = 0.0
            return
        self._last_manual_cmd_at = monotonic()
        self._last_manual_linear = linear
        self._last_manual_angular = angular

    def _publish_status(self) -> None:
        self._update_pose_from_tf()
        snapshot = self.runtime.snapshot()
        pose = snapshot.get("pose")
        state = str(snapshot.get("state") or "LOCALIZING")
        message = str(snapshot.get("message") or "")
        amcl_age = self._localization_age()
        pose_age = self.runtime.localization_age()
        pose_timeout = self._localization_timeout()
        amcl_correction_timeout = self._amcl_correction_timeout()
        has_amcl_fix = self._last_localization_fix_at is not None
        pose_fresh = pose is not None and pose_age <= pose_timeout
        localization_ok = pose is not None and has_amcl_fix and pose_fresh
        stationary = self._is_stationary(snapshot)

        route = snapshot.get("route") if isinstance(snapshot.get("route"), dict) else {}
        route_active = isinstance(route, dict) and bool(route.get("routeId"))
        manual_active = self._manual_active() and not route_active

        if pose is None or not has_amcl_fix:
            if state != "ERROR":
                state = "LOCALIZING"
                message = "Waiting for amcl pose."
        elif not pose_fresh:
            if state in {"MANUAL", "EXECUTING_ROUTE", "ERROR"} or manual_active:
                state = "ERROR"
                if not message or message == "Localized.":
                    message = f"Localization transform timeout: pose is stale for {pose_age:.2f}s"
            else:
                state = "LOCALIZING"
                message = f"Localization transform timeout: pose is stale for {pose_age:.2f}s"
        elif manual_active:
            state = "MANUAL"
            message = "Manual control active."
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
        elif state == "LOCALIZING":
            state = "IDLE"
            message = "Localized."
        elif not route_active and state == "ARRIVED":
            state = "IDLE"

        nearest_name = ""
        if pose is not None:
            nearest, _ = self.route_planner.planner.nearest_landmark(pose["x"], pose["y"])
            nearest_name = nearest.name
            self.runtime.set_nearest_lm(nearest_name)

        status = RobotStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.robot_id = self.runtime.robot_id
        status.map_id = self.runtime.map_id
        status.connected = True
        status.localization_ok = localization_ok
        status.localization_age_sec = float(amcl_age if math.isfinite(amcl_age) else 9999.0)
        status.state = state
        status.message = message
        status.target_lm = str(snapshot.get("targetLm") or "")
        status.nearest_lm = nearest_name
        status.current_edge_id = str(snapshot.get("currentEdgeId") or "")
        status.route_id = str(route.get("routeId") or "")
        status.route_progress = float(snapshot.get("routeProgress", 0.0) or 0.0)
        if pose is not None:
            status.pose_x = float(pose["x"])
            status.pose_y = float(pose["y"])
            status.pose_yaw = float(pose["yaw"])
        velocity = snapshot.get("velocity") if isinstance(snapshot.get("velocity"), dict) else {}
        status.linear_velocity = float(velocity.get("linear", 0.0) or 0.0)
        status.angular_velocity = float(velocity.get("angular", 0.0) or 0.0)
        self.status_pub.publish(status)

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
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
            )
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        self.runtime.set_pose(
            float(translation.x),
            float(translation.y),
            float(yaw),
        )

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

    def _is_stationary(self, snapshot: dict[str, Any]) -> bool:
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
        velocity = snapshot.get("velocity", {})
        if not isinstance(velocity, dict):
            return False
        linear = abs(float(velocity.get("linear", 0.0) or 0.0))
        angular = abs(float(velocity.get("angular", 0.0) or 0.0))
        return linear <= linear_epsilon and angular <= angular_epsilon
