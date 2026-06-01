from __future__ import annotations

import argparse
import json
import math
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
from time import monotonic
from typing import Any
from urllib.parse import urlparse
import webbrowser

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from route_core import DEFAULT_PARAMS_PATH, LmRoutePlanner, WarehouseMapLoader, load_route_params, save_route_params

from .runtime import PlannedRobotRoute, Pose2D, RobotRuntime, RoutePoint

try:
    from robot_msgs.msg import RobotStatus
except ImportError as exc:  # pragma: no cover - guarded at runtime
    RobotStatus = None
    ROBOT_STATUS_IMPORT_ERROR = exc
else:
    ROBOT_STATUS_IMPORT_ERROR = None


DEFAULT_ROBOT_MAP_DIR = Path(__file__).resolve().parents[1] / "map_data" / "maps_out" / "22.05.26_smap.smap"
DEFAULT_STATIC_DIR = Path(__file__).resolve().parent / "static"

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


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


class RobotTrajectoryPlanner:
    def __init__(
        self,
        map_dir: Path,
        params_path: Path,
    ) -> None:
        self.loaded_map = WarehouseMapLoader(map_dir).load()
        self.params_path = params_path
        self.params = load_route_params(params_path, create=True)
        self.planner = LmRoutePlanner(
            self.loaded_map.landmarks,
            self.loaded_map.edges,
            params=self.params,
        )

    @property
    def map_id(self) -> str:
        return self.loaded_map.map_metadata.map_name

    def update_params(self, params: dict[str, Any]) -> None:
        self.params = params
        self.planner = LmRoutePlanner(
            self.loaded_map.landmarks,
            self.loaded_map.edges,
            params=self.params,
        )

    def site_payload(self, robot_id: str) -> dict[str, Any]:
        landmarks = [self.loaded_map.landmarks[name] for name in sorted(self.loaded_map.landmarks)]
        return {
            "title": "Warehouse Robot Control",
            "robotId": robot_id,
            "mapName": self.loaded_map.map_metadata.map_name,
            "map": self.loaded_map.map_metadata.to_dict(),
            "lms": [item.to_dict() for item in landmarks],
            "edges": [edge.to_dict() for edge in self.loaded_map.edges],
            "params": self.params,
            "defaultGoal": landmarks[-1].name if landmarks else "",
        }

    def current_params(self) -> dict[str, Any]:
        return self.params

    def plan_from_pose(self, pose: Pose2D, goal_lm: str, start_lm: str | None = None) -> PlannedRobotRoute:
        goal = str(goal_lm).strip()
        if goal not in self.loaded_map.landmarks:
            raise ValueError(f"unknown goal LM: {goal}")

        planner_params = self.params.get("planner", {})
        if not isinstance(planner_params, dict):
            planner_params = {}
        sample_distance = max(0.02, float(planner_params.get("trajectory_sample_distance", 0.05) or 0.05))
        tolerance = max(0.01, float(planner_params.get("nearest_lm_tolerance", 0.05) or 0.05))
        on_route_tolerance = max(0.02, float(planner_params.get("on_route_tolerance", 0.12) or 0.12))

        nearest_name = str(start_lm or "").strip()
        connector_length = 0.0
        connector_points: list[RoutePoint] = []
        nodes: list[str] = []

        if nearest_name:
            if nearest_name not in self.loaded_map.landmarks:
                raise ValueError(f"unknown start LM: {nearest_name}")
            nearest = self.loaded_map.landmarks[nearest_name]
            distance = math.hypot(nearest.x - pose.x, nearest.y - pose.y)
        else:
            nearest, distance = self.planner.nearest_landmark(pose.x, pose.y)
            nearest_name = nearest.name

        if not start_lm and distance > tolerance:
            route = self._plan_from_current_edge(
                pose=pose,
                goal_lm=goal,
                sample_distance=sample_distance,
                on_route_tolerance=on_route_tolerance,
            )
            if route is not None:
                return route

        if distance > tolerance:
            connector_points = self._sample_line(
                pose,
                Pose2D(x=nearest.x, y=nearest.y, yaw=pose.yaw),
                sample_distance,
                edge_id=f"CURRENT_POSE->{nearest_name}",
            )
            connector_length = distance
            nodes.extend(["CURRENT_POSE", nearest_name])
        else:
            nodes.append(nearest_name)

        route = self.planner.find_route(nearest_name, goal)
        route_points = self._route_points_from_graph_route(route, sample_distance)
        if connector_points and route_points:
            route_points = route_points[1:]
        trajectory = connector_points + route_points

        if len(nodes) == 1:
            nodes = list(route.nodes)
        elif route.nodes:
            nodes.extend(route.nodes[1:])

        if not trajectory:
            trajectory = [RoutePoint(x=pose.x, y=pose.y, yaw=pose.yaw, edge_id=f"{goal}->{goal}")]

        return PlannedRobotRoute.create(
            start_lm=nearest_name,
            goal_lm=goal,
            nodes=nodes,
            trajectory=trajectory,
            length=connector_length + route.length,
        )

    def _plan_from_current_edge(
        self,
        pose: Pose2D,
        goal_lm: str,
        sample_distance: float,
        on_route_tolerance: float,
    ) -> PlannedRobotRoute | None:
        best: dict[str, Any] | None = None
        nearest_name, _ = self.planner.nearest_landmark(pose.x, pose.y)

        for edge in self.loaded_map.edges:
            sampled = self._sample_edge(edge, sample_distance)
            if len(sampled) < 2:
                continue

            projection = self._project_pose_to_samples(pose, sampled)
            if projection is None or float(projection["distance"]) > on_route_tolerance:
                continue

            if edge.to_name == goal_lm:
                route_nodes = [edge.to_name]
                route_length = 0.0
                route_points: list[RoutePoint] = []
            else:
                try:
                    graph_route = self.planner.find_route(edge.to_name, goal_lm)
                except ValueError:
                    continue
                route_nodes = list(graph_route.nodes)
                route_length = float(graph_route.length)
                route_points = self._route_points_from_graph_route(graph_route, sample_distance)

            remaining_path = self._remaining_edge_path(sampled, projection)
            remaining_length = self._path_length(remaining_path)
            total_length = remaining_length + route_length
            candidate = {
                "edge_id": f"{edge.from_name}->{edge.to_name}",
                "edge_to": edge.to_name,
                "nearest_name": nearest_name.name,
                "remaining_path": remaining_path,
                "remaining_length": remaining_length,
                "route_nodes": route_nodes,
                "route_points": route_points,
                "total_length": total_length,
            }

            if best is None or float(candidate["total_length"]) < float(best["total_length"]):
                best = candidate

        if best is None:
            return None

        trajectory = list(best["remaining_path"])
        route_points = list(best["route_points"])
        if trajectory and route_points:
            route_points = route_points[1:]
        trajectory.extend(route_points)
        if not trajectory:
            return None

        edge_to = str(best["edge_to"])
        edge_id = str(best["edge_id"])
        nodes = [f"CURRENT_EDGE {edge_id}", edge_to]
        route_nodes = [str(item) for item in best["route_nodes"] if str(item)]
        if route_nodes:
            if route_nodes[0] == edge_to:
                nodes.extend(route_nodes[1:])
            else:
                nodes.extend(route_nodes)

        return PlannedRobotRoute.create(
            start_lm=str(best["nearest_name"]),
            goal_lm=goal_lm,
            nodes=nodes,
            trajectory=trajectory,
            length=float(best["total_length"]),
        )

    def _sample_line(
        self,
        start: Pose2D,
        goal: Pose2D,
        sample_distance: float,
        edge_id: str,
    ) -> list[RoutePoint]:
        dx = goal.x - start.x
        dy = goal.y - start.y
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return [RoutePoint(x=start.x, y=start.y, yaw=start.yaw, edge_id=edge_id)]
        steps = max(1, math.ceil(length / sample_distance))
        yaw = math.atan2(dy, dx)
        return [
            RoutePoint(
                x=start.x + (dx * (step / steps)),
                y=start.y + (dy * (step / steps)),
                yaw=yaw,
                edge_id=edge_id,
                motion_direction="forward",
            )
            for step in range(steps + 1)
        ]

    def _route_points_from_graph_route(
        self,
        route,
        sample_distance: float,
    ) -> list[RoutePoint]:
        return [
            RoutePoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point.get("yaw", 0.0) or 0.0),
                edge_id=str(point.get("edgeId") or ""),
                motion_direction=str(point.get("motionDirection") or "forward"),
            )
            for point in self.planner.sample_route(route, sample_distance=sample_distance)
        ]

    def _sample_edge(self, edge, sample_distance: float) -> list[RoutePoint]:
        return [
            RoutePoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point.get("yaw", 0.0) or 0.0),
                edge_id=str(point.get("edgeId") or ""),
                motion_direction=str(point.get("motionDirection") or "forward"),
            )
            for point in self.planner._sample_edge(edge, sample_distance)
        ]

    def _project_pose_to_samples(
        self,
        pose: Pose2D,
        samples: list[RoutePoint],
    ) -> dict[str, Any] | None:
        best: dict[str, Any] | None = None
        for index in range(0, len(samples) - 1):
            first = samples[index]
            second = samples[index + 1]
            dx = second.x - first.x
            dy = second.y - first.y
            length_sq = (dx * dx) + (dy * dy)
            if length_sq <= 1e-9:
                continue
            ratio = clamp((((pose.x - first.x) * dx) + ((pose.y - first.y) * dy)) / length_sq, 0.0, 1.0)
            projected_x = first.x + (dx * ratio)
            projected_y = first.y + (dy * ratio)
            projected_yaw = normalize_angle(first.yaw + (normalize_angle(second.yaw - first.yaw) * ratio))
            distance = math.hypot(pose.x - projected_x, pose.y - projected_y)
            if best is None or distance < float(best["distance"]):
                best = {
                    "x": projected_x,
                    "y": projected_y,
                    "yaw": projected_yaw,
                    "distance": distance,
                    "segment_index": index,
                    "edge_id": first.edge_id,
                    "motion_direction": first.motion_direction,
                }
        return best

    def _remaining_edge_path(
        self,
        samples: list[RoutePoint],
        projection: dict[str, Any],
    ) -> list[RoutePoint]:
        remaining = [
            RoutePoint(
                x=float(projection["x"]),
                y=float(projection["y"]),
                yaw=float(projection["yaw"]),
                edge_id=str(projection["edge_id"]),
                motion_direction=str(projection.get("motion_direction") or "forward"),
            )
        ]
        segment_index = int(projection["segment_index"])
        for index in range(segment_index + 1, len(samples)):
            remaining.append(samples[index])
        return remaining

    def _path_length(self, points: list[RoutePoint]) -> float:
        length = 0.0
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            length += math.hypot(current.x - previous.x, current.y - previous.y)
        return length


class RobotStatusPublisher(Node):
    def __init__(
        self,
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        amcl_topic: str,
        odom_topic: str,
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
        self.status_pub = self.create_publisher(RobotStatus, status_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._on_amcl_pose, AMCL_QOS)
        self.create_subscription(Odometry, odom_topic, self._on_odom, ODOM_QOS)
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

    def _publish_status(self) -> None:
        self._update_pose_from_tf()
        snapshot = self.runtime.snapshot()
        pose = snapshot.get("pose")
        state = str(snapshot.get("state") or "LOCALIZING")
        message = str(snapshot.get("message") or "")
        age = self._localization_age()
        localization_timeout = self._localization_timeout()
        localization_ok = pose is not None and age <= localization_timeout
        stationary = self._is_stationary(snapshot)
        if pose is not None and not localization_ok and stationary and self._accept_stale_pose_when_stationary():
            localization_ok = True

        if pose is None:
            if state != "ERROR":
                state = "LOCALIZING"
                message = "Waiting for amcl pose."
        elif not localization_ok:
            if state in {"MANUAL", "EXECUTING_ROUTE", "ERROR"}:
                state = "ERROR"
                if not message or message == "Localized.":
                    message = f"Localization timeout: {age:.2f}s"
            else:
                state = "LOCALIZING"
                message = f"Localization timeout: {age:.2f}s"
        elif stationary and age > localization_timeout:
            if state == "LOCALIZING":
                state = "IDLE"
            message = f"Localized. AMCL pose is cached while robot is stationary ({age:.2f}s old)."
        elif state == "LOCALIZING":
            state = "IDLE"
            message = "Localized."

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
        status.localization_age_sec = float(age if math.isfinite(age) else 9999.0)
        status.state = state
        status.message = message
        status.target_lm = str(snapshot.get("targetLm") or "")
        status.nearest_lm = nearest_name
        status.current_edge_id = str(snapshot.get("currentEdgeId") or "")
        route = snapshot.get("route") if isinstance(snapshot.get("route"), dict) else {}
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


class RobotHttpApiNode(Node):
    def __init__(
        self,
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        params_path: Path,
        cmd_vel_topic: str,
        status_topic: str,
    ) -> None:
        super().__init__("robot_http_api")
        self.runtime = runtime
        self.route_planner = route_planner
        self.params_path = params_path
        self._status_lock = Lock()
        self._latest_status: RobotStatus | None = None
        self._last_status_event_key: tuple[str, str] | None = None
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 20)
        self.create_subscription(RobotStatus, status_topic, self._on_robot_status, 20)
        self.create_timer(0.05, self._control_step)

    def identity_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "robotId": self.runtime.robot_id,
            "mapId": self.runtime.map_id,
            "api": "robot_http_api",
            "version": 1,
        }

    def params_payload(self) -> dict[str, Any]:
        return load_route_params(self.params_path, create=True)

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        saved_path = save_route_params(payload, self.params_path)
        params = load_route_params(self.params_path, create=True)
        self.route_planner.update_params(params)
        self.runtime.add_event("info", f"params saved: {saved_path}")
        return {
            "ok": True,
            "path": str(saved_path),
            "params": params,
        }

    def status_payload(self) -> dict[str, Any]:
        status = self._latest_status_payload()
        snapshot = self.runtime.snapshot()
        return {
            "ok": True,
            "robot": status,
            "route": snapshot.get("route"),
            "events": snapshot.get("events", []),
        }

    def plan_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_request(payload)
        self.runtime.add_event("info", f"planned route to {route.goal_lm}")
        return {
            "ok": True,
            "route": route.to_dict(),
        }

    def execute_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_request(payload)
        self.runtime.set_route(route)
        self.runtime.add_event("info", f"executing route {route.route_id} -> {route.goal_lm}")
        return {
            "ok": True,
            "route": route.to_dict(),
            "status": self.status_payload(),
        }

    def cancel_route_payload(self) -> dict[str, Any]:
        self.runtime.cancel_route("Route canceled.")
        self.runtime.clear_manual()
        self._publish_cmd_vel(0.0, 0.0)
        self.runtime.add_event("warn", "route canceled")
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def teleop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        linear = float(payload.get("linear", 0.0) or 0.0)
        angular = float(payload.get("angular", 0.0) or 0.0)
        timeout_ms = max(80, int(payload.get("timeoutMs", 300) or 300))
        snapshot = self.runtime.snapshot()
        self.runtime.set_manual_command(linear=linear, angular=angular, timeout_sec=timeout_ms / 1000.0)
        if snapshot.get("state") != "MANUAL":
            self.runtime.add_event("info", "manual control engaged")
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def teleop_stop_payload(self) -> dict[str, Any]:
        self.runtime.clear_manual()
        self._publish_cmd_vel(0.0, 0.0)
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def stop_payload(self) -> dict[str, Any]:
        self.runtime.clear_manual()
        self.runtime.cancel_route("Stopped.")
        self._publish_cmd_vel(0.0, 0.0)
        self.runtime.add_event("warn", "robot stopped")
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def _latest_status_payload(self) -> dict[str, Any]:
        with self._status_lock:
            message = self._latest_status
        if message is None:
            snapshot = self.runtime.snapshot()
            pose = snapshot.get("pose") if isinstance(snapshot.get("pose"), dict) else {}
            velocity = snapshot.get("velocity") if isinstance(snapshot.get("velocity"), dict) else {}
            route = snapshot.get("route") if isinstance(snapshot.get("route"), dict) else {}
            return {
                "robotId": self.runtime.robot_id,
                "mapId": self.runtime.map_id,
                "connected": True,
                "localizationOk": False,
                "localizationAgeSec": float(snapshot.get("localizationAgeSec", 9999.0) or 9999.0),
                "state": str(snapshot.get("state") or "LOCALIZING"),
                "message": str(snapshot.get("message") or ""),
                "targetLm": str(snapshot.get("targetLm") or ""),
                "nearestLm": str(snapshot.get("nearestLm") or ""),
                "currentEdgeId": str(snapshot.get("currentEdgeId") or ""),
                "routeId": str(route.get("routeId") or ""),
                "routeProgress": float(snapshot.get("routeProgress", 0.0) or 0.0),
                "pose": {
                    "x": float(pose.get("x", 0.0) or 0.0),
                    "y": float(pose.get("y", 0.0) or 0.0),
                    "yaw": float(pose.get("yaw", 0.0) or 0.0),
                } if pose else None,
                "velocity": {
                    "linear": float(velocity.get("linear", 0.0) or 0.0),
                    "angular": float(velocity.get("angular", 0.0) or 0.0),
                },
            }

        return {
            "robotId": message.robot_id,
            "mapId": message.map_id,
            "connected": bool(message.connected),
            "localizationOk": bool(message.localization_ok),
            "localizationAgeSec": float(message.localization_age_sec),
            "state": message.state,
            "message": message.message,
            "targetLm": message.target_lm,
            "nearestLm": message.nearest_lm,
            "currentEdgeId": message.current_edge_id,
            "routeId": message.route_id,
            "routeProgress": float(message.route_progress),
            "pose": {
                "x": float(message.pose_x),
                "y": float(message.pose_y),
                "yaw": float(message.pose_yaw),
            } if message.localization_ok else None,
            "velocity": {
                "linear": float(message.linear_velocity),
                "angular": float(message.angular_velocity),
            },
        }

    def _route_from_request(self, payload: dict[str, Any]) -> PlannedRobotRoute:
        route_payload = payload.get("route")
        if isinstance(route_payload, dict):
            route = PlannedRobotRoute.from_dict(route_payload)
            if not route.goal_lm:
                raise ValueError("route.goalLm is required")
            return route

        goal_lm = str(payload.get("goalLm") or payload.get("targetLm") or "").strip()
        if not goal_lm:
            raise ValueError("goalLm is required")

        pose_payload = payload.get("startPose")
        pose = None
        if isinstance(pose_payload, dict):
            pose = Pose2D(
                x=float(pose_payload.get("x", 0.0) or 0.0),
                y=float(pose_payload.get("y", 0.0) or 0.0),
                yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
            )
        if pose is None:
            pose = self._best_available_pose()
        if pose is None:
            raise ValueError("robot pose is not available yet")
        start_lm = str(payload.get("startLm") or "").strip() or None
        return self.route_planner.plan_from_pose(pose=pose, goal_lm=goal_lm, start_lm=start_lm)

    def _best_available_pose(self) -> Pose2D | None:
        pose = self.runtime.latest_pose()
        if pose is not None:
            return pose

        snapshot = self.runtime.snapshot()
        pose_payload = snapshot.get("pose")
        if isinstance(pose_payload, dict):
            return Pose2D(
                x=float(pose_payload.get("x", 0.0) or 0.0),
                y=float(pose_payload.get("y", 0.0) or 0.0),
                yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
            )

        with self._status_lock:
            message = self._latest_status
        if message is None:
            return None

        state = str(message.state or "")
        if not bool(message.localization_ok) and state in {"", "LOCALIZING", "ERROR"}:
            return None

        return Pose2D(
            x=float(message.pose_x),
            y=float(message.pose_y),
            yaw=float(message.pose_yaw),
        )

    def _on_robot_status(self, message: RobotStatus) -> None:
        with self._status_lock:
            previous = self._latest_status
            self._latest_status = message
        self._persist_status_event(previous, message)

    def _persist_status_event(self, previous: RobotStatus | None, current: RobotStatus) -> None:
        state = str(current.state or "").strip() or "UNKNOWN"
        raw_message = str(current.message or "").strip()
        previous_state = str(previous.state or "").strip() if previous is not None else ""
        previous_message = str(previous.message or "").strip() if previous is not None else ""

        if state == previous_state and raw_message == previous_message:
            return

        level = "info"
        persist = False
        if state == "ERROR":
            level = "error"
            persist = True
        elif state == "LOCALIZING" and ("timeout" in raw_message.lower() or "waiting" in raw_message.lower()):
            level = "warn"
            persist = True
        elif previous_state == "ERROR" and state != "ERROR":
            level = "info"
            persist = True

        if not persist:
            return

        event_key = (state, raw_message)
        if event_key == self._last_status_event_key:
            return
        self._last_status_event_key = event_key
        self.runtime.add_event(level, self._humanize_status_event(state, raw_message))

    def _humanize_status_event(self, state: str, message: str) -> str:
        text = message.strip() or state
        lowered = text.lower()

        if state == "ERROR" and "localization timeout" in lowered:
            return f"Localization error: {text}. Robot pose became stale. Check /scan, /amcl_pose, /tf, and map alignment."
        if state == "LOCALIZING" and "waiting for amcl pose" in lowered:
            return "Localization waiting: AMCL pose has not been received yet. Set initial pose and verify /amcl_pose."
        if state == "LOCALIZING" and "timeout" in lowered:
            return f"Localization warning: {text}. The last AMCL update is too old."
        if state == "ERROR" and "robot pose is not available" in lowered:
            return "Route execution error: robot pose is not available for planning."
        if state != "ERROR" and state != "LOCALIZING":
            return f"Recovered from error: {text}."
        return f"{state}: {text}"

    def _control_step(self) -> None:
        now = monotonic()
        manual = self.runtime.manual_command(now=now)
        status = self._latest_status_payload()
        if manual is not None:
            if not status.get("localizationOk", False):
                self.runtime.clear_manual()
                self.runtime.set_state("ERROR", "Localization timeout during manual control.")
                self.runtime.add_event(
                    "error",
                    "Manual control error: localization timed out while teleop was active. Robot was stopped."
                )
                self._publish_cmd_vel(0.0, 0.0)
                return
            self.runtime.set_state("MANUAL", "Manual control active.")
            self._publish_cmd_vel(manual.linear, manual.angular)
            return

        route = self.runtime.active_route()
        if route is None:
            if status.get("localizationOk", False) and status.get("state") in {"ARRIVED", "MANUAL", "EXECUTING_ROUTE"}:
                self.runtime.set_state("IDLE", "Ready.")
            return

        pose_payload = status.get("pose")
        if not isinstance(pose_payload, dict):
            self.runtime.finish_route(False, "Robot pose is not available.")
            self.runtime.add_event(
                "error",
                "Route execution error: robot pose is not available. Planning or tracking cannot continue."
            )
            self._publish_cmd_vel(0.0, 0.0)
            return

        pose = Pose2D(
            x=float(pose_payload.get("x", 0.0) or 0.0),
            y=float(pose_payload.get("y", 0.0) or 0.0),
            yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
        )
        self._follow_route(route, pose)

    def _follow_route(self, route: PlannedRobotRoute, pose: Pose2D) -> None:
        params = self.route_planner.current_params()
        navigation = params.get("navigation", {})
        planner = params.get("planner", {})
        localization = params.get("localization", {})
        if not isinstance(navigation, dict):
            navigation = {}
        if not isinstance(planner, dict):
            planner = {}
        if not isinstance(localization, dict):
            localization = {}

        route_speed = max(0.05, float(navigation.get("route_speed", 0.35) or 0.35))
        lookahead = max(0.10, float(navigation.get("footprint_lookahead", 0.8) or 0.8))
        stop_distance = max(0.04, float(navigation.get("stop_distance", 0.1) or 0.1))
        angular_gain = max(0.4, float(navigation.get("angular_gain", 2.2) or 2.2))
        max_angular = max(0.35, float(navigation.get("max_angular_speed", 0.9) or 0.9))
        rotate_in_place_angle = math.radians(
            max(10.0, float(navigation.get("rotate_in_place_angle_deg", 32.0) or 32.0))
        )
        curve_speed_limit = max(0.05, float(navigation.get("curve_speed_limit", 0.25) or 0.25))
        rejoin_speed_limit = max(0.05, float(navigation.get("rejoin_speed_limit", 0.16) or 0.16))
        hard_rejoin_speed_limit = max(0.04, float(navigation.get("hard_rejoin_speed_limit", 0.06) or 0.06))
        on_route_tolerance = max(0.05, float(planner.get("on_route_tolerance", 0.12) or 0.12))
        yaw_tolerance = math.radians(max(0.5, float(localization.get("allowed_yaw_error_deg", 4.0) or 4.0)))

        if not route.trajectory:
            self.runtime.finish_route(False, "Route is empty.")
            self.runtime.add_event(
                "error",
                "Route execution error: the planned route is empty."
            )
            self._publish_cmd_vel(0.0, 0.0)
            return

        points = route.trajectory
        distances = self._path_distances(points)
        projection = self._project_pose_to_path(points, distances, pose, route.current_index)
        current_index = int(projection["index"])
        total_length = max(1e-6, distances[-1])
        progress = float(projection["s"]) / total_length
        self.runtime.update_route_progress(current_index, str(projection["edge_id"]), progress)

        final_point = points[-1]
        distance_to_goal = math.hypot(final_point.x - pose.x, final_point.y - pose.y)
        desired_final_yaw = normalize_angle(final_point.yaw)
        final_yaw_error = normalize_angle(desired_final_yaw - pose.yaw)
        remaining_distance = max(0.0, total_length - float(projection["s"]))
        if remaining_distance <= stop_distance and distance_to_goal <= stop_distance and abs(final_yaw_error) <= yaw_tolerance:
            self._publish_cmd_vel(0.0, 0.0)
            self.runtime.finish_route(True, f"Arrived at {route.goal_lm}.")
            self.runtime.add_event("info", f"arrived at {route.goal_lm}")
            return

        preview_target = self._interpolate_path_point(
            points,
            distances,
            min(total_length, float(projection["s"]) + lookahead),
            current_index,
        )
        curvature_hint = abs(normalize_angle(preview_target.yaw - float(projection["yaw"])))
        effective_lookahead = clamp(
            lookahead
            * (
                1.0
                - (0.35 * min(1.0, curvature_hint / 1.1))
                - (0.30 * min(1.0, abs(float(projection["cross_track"])) / max(0.2, on_route_tolerance * 2.0)))
            ),
            0.12,
            lookahead,
        )
        target = self._interpolate_path_point(
            points,
            distances,
            min(total_length, float(projection["s"]) + effective_lookahead),
            current_index,
        )
        drive_sign = -1.0 if target.motion_direction == "backward" else 1.0
        path_heading_error = normalize_angle(target.yaw - pose.yaw)
        target_bearing = math.atan2(target.y - pose.y, target.x - pose.x)
        target_heading_error = path_heading_error if drive_sign < 0.0 else normalize_angle(target_bearing - pose.yaw)
        cross_track_error = float(projection["cross_track"])
        cross_track_term = math.atan2(1.9 * cross_track_error, max(route_speed, 0.10))
        steering_error = (
            path_heading_error - cross_track_term
            if drive_sign < 0.0
            else ((0.45 * path_heading_error) + (0.55 * target_heading_error) - cross_track_term)
        )

        linear = drive_sign * route_speed
        heading_penalty = min(abs(steering_error) / 1.25, 0.90)
        lateral_penalty = min(abs(cross_track_error) / max(0.32, on_route_tolerance * 2.5), 0.85)
        curvature_penalty = min(curvature_hint / 1.10, 0.70)
        linear_scale = max(
            0.12,
            1.0 - (0.45 * heading_penalty) - (0.35 * lateral_penalty) - (0.25 * curvature_penalty),
        )
        linear *= linear_scale

        off_route = abs(cross_track_error) > on_route_tolerance
        hard_rejoin = abs(cross_track_error) > max(0.32, on_route_tolerance * 2.5)
        if off_route:
            linear = drive_sign * min(abs(linear), max(0.08, route_speed * 0.55))
        if hard_rejoin and abs(target_heading_error) > 0.45:
            linear = drive_sign * min(abs(linear), 0.06)
        if abs(steering_error) > 1.25:
            linear = drive_sign * min(abs(linear), 0.05)
        if distance_to_goal < 0.50:
            linear = drive_sign * min(abs(linear), max(0.08, distance_to_goal))
        if distance_to_goal < stop_distance:
            linear = drive_sign * min(abs(linear), 0.05)
        if distance_to_goal <= stop_distance and abs(final_yaw_error) > yaw_tolerance:
            linear = 0.0
            steering_error = final_yaw_error
        elif hard_rejoin and abs(target_heading_error) > rotate_in_place_angle:
            linear = 0.0
            steering_error = path_heading_error if drive_sign < 0.0 else target_heading_error
        elif off_route and abs(steering_error) > (rotate_in_place_angle * 1.15):
            linear = 0.0

        angular = clamp(angular_gain * steering_error, -max_angular, max_angular)
        if hard_rejoin:
            message = f"Returning to route toward {route.goal_lm}."
        elif off_route:
            message = f"Rejoining route to {route.goal_lm}."
        else:
            message = f"Driving to {route.goal_lm}."
        if curvature_hint > 0.38:
            linear = drive_sign * min(abs(linear), curve_speed_limit)
        self.runtime.set_state("EXECUTING_ROUTE", message)
        if off_route:
            linear = drive_sign * min(abs(linear), rejoin_speed_limit)
        if hard_rejoin:
            linear = drive_sign * min(abs(linear), hard_rejoin_speed_limit)
        self._publish_cmd_vel(linear, angular)

    def _path_distances(self, points: list[RoutePoint]) -> list[float]:
        distances = [0.0]
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            distances.append(
                distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
            )
        return distances

    def _project_pose_to_path(
        self,
        points: list[RoutePoint],
        distances: list[float],
        pose: Pose2D,
        hint_index: int,
    ) -> dict[str, Any]:
        if len(points) == 1:
            point = points[0]
            return {
                "index": 0,
                "s": 0.0,
                "x": point.x,
                "y": point.y,
                "yaw": point.yaw,
                "cross_track": math.hypot(point.x - pose.x, point.y - pose.y),
                "edge_id": point.edge_id,
            }

        best = self._project_pose_to_path_range(
            points=points,
            distances=distances,
            pose=pose,
            start_index=max(0, hint_index - 4),
            stop_index=min(len(points) - 1, hint_index + 72),
        )
        if best is None or abs(float(best["cross_track"])) > 0.75:
            fallback = self._project_pose_to_path_range(
                points=points,
                distances=distances,
                pose=pose,
                start_index=0,
                stop_index=len(points) - 1,
            )
            if fallback is not None:
                best = fallback

        return best or {
            "index": 0,
            "s": 0.0,
            "x": points[0].x,
            "y": points[0].y,
            "yaw": points[0].yaw,
            "cross_track": math.hypot(points[0].x - pose.x, points[0].y - pose.y),
            "edge_id": points[0].edge_id,
        }

    def _project_pose_to_path_range(
        self,
        points: list[RoutePoint],
        distances: list[float],
        pose: Pose2D,
        start_index: int,
        stop_index: int,
    ) -> dict[str, Any] | None:
        segment_count = max(0, len(points) - 1)
        if segment_count == 0:
            return None
        start = min(max(0, start_index), segment_count - 1)
        stop = min(max(start + 1, stop_index), segment_count)
        best: dict[str, Any] | None = None
        best_distance = float("inf")
        for index in range(start, stop):
            first = points[index]
            second = points[index + 1]
            dx = second.x - first.x
            dy = second.y - first.y
            seg_len_sq = (dx * dx) + (dy * dy)
            if seg_len_sq <= 1e-9:
                ratio = 0.0
                proj_x = first.x
                proj_y = first.y
            else:
                ratio = clamp((((pose.x - first.x) * dx) + ((pose.y - first.y) * dy)) / seg_len_sq, 0.0, 1.0)
                proj_x = first.x + (dx * ratio)
                proj_y = first.y + (dy * ratio)
            interpolated_yaw = normalize_angle(first.yaw + (normalize_angle(second.yaw - first.yaw) * ratio))
            cross_track = (-math.sin(interpolated_yaw) * (pose.x - proj_x)) + (math.cos(interpolated_yaw) * (pose.y - proj_y))
            distance = math.hypot(pose.x - proj_x, pose.y - proj_y)
            if distance < best_distance:
                best_distance = distance
                best = {
                    "index": index if ratio < 0.5 else min(index + 1, len(points) - 1),
                    "s": distances[index] + (math.sqrt(seg_len_sq) * ratio),
                    "x": proj_x,
                    "y": proj_y,
                    "yaw": interpolated_yaw,
                    "cross_track": cross_track,
                    "edge_id": second.edge_id if ratio > 0.5 and second.edge_id else first.edge_id,
                }
        return best

    def _interpolate_path_point(
        self,
        points: list[RoutePoint],
        distances: list[float],
        target_s: float,
        hint_index: int = 0,
    ) -> RoutePoint:
        if target_s <= 0.0 or len(points) == 1:
            return points[0]
        if target_s >= distances[-1]:
            return points[-1]

        index = max(0, min(hint_index, len(points) - 2))
        if distances[index] > target_s:
            while index > 0 and distances[index] > target_s:
                index -= 1
        else:
            while index < len(points) - 2 and distances[index + 1] < target_s:
                index += 1

        first = points[index]
        second = points[index + 1]
        span = max(1e-6, distances[index + 1] - distances[index])
        ratio = clamp((target_s - distances[index]) / span, 0.0, 1.0)
        return RoutePoint(
            x=first.x + ((second.x - first.x) * ratio),
            y=first.y + ((second.y - first.y) * ratio),
            yaw=normalize_angle(first.yaw + (normalize_angle(second.yaw - first.yaw) * ratio)),
            edge_id=second.edge_id if ratio > 0.5 and second.edge_id else first.edge_id,
            motion_direction=second.motion_direction if ratio > 0.5 else first.motion_direction,
        )

    def _closest_trajectory_index(
        self,
        points: list[RoutePoint],
        pose: Pose2D,
        hint_index: int,
    ) -> int:
        if len(points) == 1:
            return 0
        start = max(0, hint_index - 5)
        stop = min(len(points), hint_index + 80) if hint_index else min(len(points), 80)
        best_index = start
        best_distance = float("inf")
        for index in range(start, stop):
            point = points[index]
            distance = math.hypot(point.x - pose.x, point.y - pose.y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def _lookahead_point(self, points: list[RoutePoint], start_index: int, lookahead: float) -> RoutePoint:
        if start_index >= len(points) - 1:
            return points[-1]
        distance = 0.0
        previous = points[start_index]
        for index in range(start_index + 1, len(points)):
            current = points[index]
            distance += math.hypot(current.x - previous.x, current.y - previous.y)
            if distance >= lookahead:
                return current
            previous = current
        return points[-1]

    def _publish_cmd_vel(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._cmd_vel_pub.publish(message)


class RobotWebRequestHandler(SimpleHTTPRequestHandler):
    bridge: RobotHttpApiNode | None = None
    params_path: Path = DEFAULT_PARAMS_PATH
    site_data_script: str = ""

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/health":
            self._send_json({"ok": True})
            return
        if path == "/demo-data.js":
            self._send_script(self.site_data_script)
            return
        if path == "/api/robot/identity":
            self._send_json(self._require_bridge().identity_payload())
            return
        if path == "/api/robot/status":
            self._send_json(self._require_bridge().status_payload())
            return
        if path == "/api/params":
            self._send_json(self._require_bridge().params_payload())
            return
        super().do_GET()

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/params":
            self._handle_json(self._handle_save_params)
            return
        if path == "/api/robot/teleop":
            self._handle_json(self._handle_teleop)
            return
        if path == "/api/robot/teleop/stop":
            self._handle_json(self._handle_teleop_stop)
            return
        if path == "/api/robot/route/plan":
            self._handle_json(self._handle_plan_route)
            return
        if path == "/api/robot/route/execute":
            self._handle_json(self._handle_execute_route)
            return
        if path == "/api/robot/route/cancel":
            self._handle_json(self._handle_cancel_route)
            return
        if path == "/api/robot/stop":
            self._handle_json(self._handle_stop)
            return
        self.send_error(404, "Not found")

    def _handle_save_params(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().save_params_payload(payload)

    def _handle_teleop(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().teleop_payload(payload)

    def _handle_teleop_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        return self._require_bridge().teleop_stop_payload()

    def _handle_plan_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().plan_route_payload(payload)

    def _handle_execute_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().execute_route_payload(payload)

    def _handle_cancel_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        return self._require_bridge().cancel_route_payload()

    def _handle_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        return self._require_bridge().stop_payload()

    def _handle_json(self, callback) -> None:
        try:
            payload = self._read_json_payload()
            if payload is None:
                return
            if not isinstance(payload, dict):
                self.send_error(400, "Expected object")
                return
            self._send_json(callback(payload))
        except ValueError as exc:
            self.send_error(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self.send_error(500, str(exc))

    def _read_json_payload(self) -> object | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        if not raw_body:
            return {}
        try:
            return json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
            return None

    def _send_json(self, payload: object) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_script(self, content: str) -> None:
        encoded = content.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/javascript; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _require_bridge(self) -> RobotHttpApiNode:
        if self.bridge is None:
            raise RuntimeError("robot_http_api bridge is not ready")
        return self.bridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve single robot HTTP API and web UI.")
    parser.add_argument("--map-dir", default=DEFAULT_ROBOT_MAP_DIR, type=Path)
    parser.add_argument("--params", default=DEFAULT_PARAMS_PATH, type=Path)
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8790, type=int)
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--amcl-topic", default="/amcl_pose")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--status-topic", default="/robot_status")
    return parser.parse_args()


def resolve_map_dir(map_dir: Path) -> Path:
    if map_dir.exists():
        return map_dir
    if map_dir.is_absolute():
        return map_dir

    project_root = Path(__file__).resolve().parents[1]
    relocated = project_root / "map_data" / map_dir
    if relocated.exists():
        return relocated
    return map_dir


def main() -> None:
    if RobotStatus is None:
        raise RuntimeError(
            "robot_msgs is not available. Build and source the ROS workspace first: "
            "`cd robot && source /opt/ros/jazzy/setup.bash && colcon build --packages-select robot_msgs && source install/setup.bash`"
        ) from ROBOT_STATUS_IMPORT_ERROR

    args = parse_args()
    map_dir = resolve_map_dir(args.map_dir)
    params_path = args.params.resolve()
    route_planner = RobotTrajectoryPlanner(map_dir=map_dir, params_path=params_path)
    runtime = RobotRuntime(robot_id=args.robot_id, map_id=route_planner.map_id)

    rclpy.init(args=None)
    status_node = RobotStatusPublisher(
        runtime=runtime,
        route_planner=route_planner,
        amcl_topic=args.amcl_topic,
        odom_topic=args.odom_topic,
        status_topic=args.status_topic,
    )
    bridge_node = RobotHttpApiNode(
        runtime=runtime,
        route_planner=route_planner,
        params_path=params_path,
        cmd_vel_topic=args.cmd_vel_topic,
        status_topic=args.status_topic,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(status_node)
    executor.add_node(bridge_node)
    spinner = Thread(target=executor.spin, daemon=True)
    spinner.start()

    site_payload = json.dumps(route_planner.site_payload(args.robot_id), ensure_ascii=False).replace(
        "</script>",
        "<\\/script>",
    )
    RobotWebRequestHandler.bridge = bridge_node
    RobotWebRequestHandler.params_path = params_path
    RobotWebRequestHandler.site_data_script = f"window.ROBOT_WEB_DATA = {site_payload};\n"
    handler = partial(RobotWebRequestHandler, directory=str(DEFAULT_STATIC_DIR.resolve()))
    server = ThreadingHTTPServer((args.host, args.port), handler)
    url = f"http://{args.host}:{args.port}/"
    print(f"Serving single robot UI: {url}")
    print(f"Robot id: {args.robot_id}")
    print(f"Map dir: {map_dir}")
    print(f"Params path: {params_path}")
    if args.open:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()
        executor.shutdown()
        status_node.destroy_node()
        bridge_node.destroy_node()
        rclpy.shutdown()
