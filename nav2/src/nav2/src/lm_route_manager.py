#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import queue
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from route_core import Landmark, LmRoutePlanner, PlannedRoute, WarehouseMapLoader


class LmRouteGraph:
    def __init__(self, map_dir: Path) -> None:
        loaded_map = WarehouseMapLoader(map_dir).load()
        self.map_dir = loaded_map.map_dir
        self.map_metadata = loaded_map.map_metadata
        self.landmarks = loaded_map.landmarks
        self.edges = loaded_map.edges
        self._planner = LmRoutePlanner(self.landmarks, self.edges)

    def nearest_landmark(self, x: float, y: float) -> tuple[Landmark, float]:
        return self._planner.nearest_landmark(x, y)

    def find_route(self, start: str, goal: str) -> PlannedRoute:
        return self._planner.find_route(start, goal)

    def sample_path(
        self,
        route: PlannedRoute,
        sample_distance: float,
    ) -> list[dict[str, float | str]]:
        return self._planner.sample_route(route, sample_distance)


class LmRouteManager(Node):
    def __init__(self) -> None:
        super().__init__("lm_route_manager")
        self.declare_parameter("map_dir", "")
        self.declare_parameter("path_topic", "/lm_route_path")
        self.declare_parameter("goal_topic", "/lm_goal")
        self.declare_parameter("sample_distance", 0.05)
        self.declare_parameter("default_goal_lm", "")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("lm_position_tolerance", 0.05)
        self.declare_parameter("rotate_yaw_tolerance", 0.05)
        self.declare_parameter("web_host", "127.0.0.1")
        self.declare_parameter("web_port", 8765)
        self.declare_parameter("enable_web_server", True)

        map_dir = Path(self.get_parameter("map_dir").value)
        if not map_dir.exists():
            raise FileNotFoundError(f"LM map directory does not exist: {map_dir}")

        self.graph = LmRouteGraph(map_dir)
        self.sample_distance = float(self.get_parameter("sample_distance").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self.progress_checker_id = str(self.get_parameter("progress_checker_id").value)
        self.default_goal_lm = str(self.get_parameter("default_goal_lm").value).strip()
        self.lm_position_tolerance = float(self.get_parameter("lm_position_tolerance").value)
        self.rotate_yaw_tolerance = float(self.get_parameter("rotate_yaw_tolerance").value)
        self.web_host = str(self.get_parameter("web_host").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.enable_web_server = bool(self.get_parameter("enable_web_server").value)
        self.web_dir = Path(get_package_share_directory("nav2")) / "web"

        self.path_topic = str(self.get_parameter("path_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)

        self.path_publisher = self.create_publisher(NavPath, self.path_topic, 10)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._handle_pose,
            10,
        )
        self.goal_subscription = self.create_subscription(
            String,
            self.goal_topic,
            self._handle_goal_lm,
            10,
        )

        self.follow_path_client = ActionClient(self, FollowPath, "/follow_path")
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.command_queue: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self.command_timer = self.create_timer(0.1, self._process_commands)
        self.default_goal_timer = self.create_timer(1.0, self._maybe_run_default_goal)

        self.state_lock = threading.Lock()
        self.latest_pose: dict[str, float] | None = None
        self.mode = "IDLE"
        self.status_text = "Ready."
        self.active_goal_lm = ""
        self.pending_route_goal_lm = ""
        self.current_route_nodes: list[str] = []
        self.current_route_path: list[dict[str, float | str]] = []
        self.last_route_start_lm = ""
        self.last_nearest_lm = ""
        self.last_nearest_lm_distance = math.inf
        self.navigate_goal_handle = None
        self.follow_goal_handle = None
        self.pending_navigate_mode = ""
        self.pending_rotate_yaw: float | None = None
        self.pending_follow_route_path: list[dict[str, float | str]] = []
        self.web_server: ThreadingHTTPServer | None = None
        self.web_thread: threading.Thread | None = None

        if self.enable_web_server:
            self._start_web_server()

        self.get_logger().info(
            f"Loaded LM graph from {map_dir} with "
            f"{len(self.graph.landmarks)} landmarks and {len(self.graph.edges)} directed edges"
        )

    def config_payload(self) -> dict[str, object]:
        return {
            "mapName": self.graph.map_metadata.map_name,
            "map": self.graph.map_metadata.to_dict(),
            "lms": [self.graph.landmarks[name].to_dict() for name in sorted(self.graph.landmarks)],
            "edges": [edge.to_dict() for edge in self.graph.edges],
        }

    def state_payload(self) -> dict[str, object]:
        with self.state_lock:
            return {
                "mode": self.mode,
                "statusText": self.status_text,
                "activeGoalLm": self.active_goal_lm,
                "pendingRouteGoalLm": self.pending_route_goal_lm,
                "routeNodes": list(self.current_route_nodes),
                "routePath": list(self.current_route_path),
                "nearestLm": self.last_nearest_lm,
                "nearestLmDistance": (
                    None if math.isinf(self.last_nearest_lm_distance) else self.last_nearest_lm_distance
                ),
                "isOnLandmark": (
                    False
                    if math.isinf(self.last_nearest_lm_distance)
                    else self.last_nearest_lm_distance <= self.lm_position_tolerance
                ),
                "robotPose": dict(self.latest_pose) if self.latest_pose is not None else None,
                "lmPositionTolerance": self.lm_position_tolerance,
                "rotateYawTolerance": self.rotate_yaw_tolerance,
            }

    def enqueue_goal(self, goal_lm: str) -> None:
        self.command_queue.put(("goal_lm", {"goal_lm": goal_lm}))

    def enqueue_stop(self) -> None:
        self.command_queue.put(("stop", {}))

    def _handle_pose(self, msg: PoseWithCovarianceStamped) -> None:
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        pose = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": yaw,
        }
        nearest, distance = self.graph.nearest_landmark(pose["x"], pose["y"])
        with self.state_lock:
            self.latest_pose = pose
            self.last_nearest_lm = nearest.name
            self.last_nearest_lm_distance = distance

    def _handle_goal_lm(self, msg: String) -> None:
        goal_lm = msg.data.strip()
        if goal_lm:
            self.enqueue_goal(goal_lm)

    def _maybe_run_default_goal(self) -> None:
        if not self.default_goal_lm or self.latest_pose is None:
            return
        goal_lm = self.default_goal_lm
        self.default_goal_lm = ""
        self.enqueue_goal(goal_lm)

    def _process_commands(self) -> None:
        while True:
            try:
                command, payload = self.command_queue.get_nowait()
            except queue.Empty:
                break

            if command == "goal_lm":
                self._start_goal_request(str(payload["goal_lm"]))
            elif command == "stop":
                self._stop_active_motion("Stopped by operator.")

    def _start_goal_request(self, goal_lm: str) -> None:
        if self.latest_pose is None:
            self._set_state("ERROR", "Cannot start route: /amcl_pose is not available.")
            return

        if goal_lm not in self.graph.landmarks:
            self._set_state("ERROR", f"Unknown goal LM: {goal_lm}")
            return

        self._cancel_active_goals()
        nearest, distance = self.graph.nearest_landmark(self.latest_pose["x"], self.latest_pose["y"])

        with self.state_lock:
            self.active_goal_lm = goal_lm
            self.pending_route_goal_lm = goal_lm

        if distance > self.lm_position_tolerance:
            self._set_state(
                "NAV_TO_NEAREST_LM",
                f"Robot is {distance:.2f} m from {nearest.name}. Navigating to nearest LM first.",
            )
            self._send_navigate_to_pose(nearest, mode="TO_NEAREST_LM")
            return

        self._set_state("AT_LM", f"Robot is on {nearest.name}. Building strict LM route.")
        self._start_strict_route(nearest.name, goal_lm)

    def _send_navigate_to_pose(self, target_lm: Landmark, mode: str) -> None:
        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self._set_state("ERROR", "/navigate_to_pose action server is not available.")
            return

        yaw = 0.0
        if self.latest_pose is not None:
            yaw = math.atan2(target_lm.y - self.latest_pose["y"], target_lm.x - self.latest_pose["x"])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target_lm.x
        goal.pose.pose.position.y = target_lm.y
        goal.pose.pose.orientation = self._quaternion_from_yaw(yaw)

        self.pending_navigate_mode = mode
        future = self.navigate_to_pose_client.send_goal_async(goal)
        future.add_done_callback(self._handle_navigate_response)

    def _send_rotate_to_route(self, target_yaw: float) -> None:
        if self.latest_pose is None:
            self._set_state("ERROR", "Cannot rotate to route: robot pose is unavailable.")
            return
        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self._set_state("ERROR", "/navigate_to_pose action server is not available.")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.latest_pose["x"]
        goal.pose.pose.position.y = self.latest_pose["y"]
        goal.pose.pose.orientation = self._quaternion_from_yaw(target_yaw)

        self.pending_navigate_mode = "ROTATE_TO_ROUTE"
        self.pending_rotate_yaw = target_yaw
        future = self.navigate_to_pose_client.send_goal_async(goal)
        future.add_done_callback(self._handle_navigate_response)

    def _handle_navigate_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._set_state("ERROR", "NavigateToPose goal was rejected.")
            return

        self.navigate_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_navigate_result)

    def _handle_navigate_result(self, future) -> None:
        result = future.result()
        self.navigate_goal_handle = None
        if result is None:
            self._set_state("ERROR", "NavigateToPose returned no result.")
            return

        status = int(result.status)
        if status == 5:
            self._set_state("IDLE", "NavigateToPose canceled.")
            self.pending_navigate_mode = ""
            self.pending_rotate_yaw = None
            return
        if status != 4:
            self._set_state("ERROR", f"NavigateToPose failed with status={status}.")
            self.pending_navigate_mode = ""
            self.pending_rotate_yaw = None
            return

        if self.latest_pose is None:
            self._set_state("ERROR", "NavigateToPose succeeded but robot pose is unavailable.")
            self.pending_navigate_mode = ""
            self.pending_rotate_yaw = None
            return

        navigate_mode = self.pending_navigate_mode
        self.pending_navigate_mode = ""

        if navigate_mode == "ROTATE_TO_ROUTE":
            route_path = list(self.pending_follow_route_path)
            self.pending_rotate_yaw = None
            if not route_path:
                self._set_state("ERROR", "RotateToRoute succeeded but no pending route exists.")
                return
            self._set_state("FOLLOW_LM_ROUTE", "Rotation to route heading completed. Starting path following.")
            self._send_follow_path(route_path)
            return

        nearest, distance = self.graph.nearest_landmark(self.latest_pose["x"], self.latest_pose["y"])
        goal_lm = self.pending_route_goal_lm
        if not goal_lm:
            self._set_state("IDLE", "NavigateToPose finished.")
            return

        if distance > self.lm_position_tolerance:
            self._set_state(
                "ERROR",
                f"Robot still not on LM after NavigateToPose: nearest {nearest.name}, distance {distance:.2f} m.",
            )
            return

        self._set_state("AT_LM", f"Reached {nearest.name}. Building strict LM route.")
        self._start_strict_route(nearest.name, goal_lm)

    def _start_strict_route(self, start_lm: str, goal_lm: str) -> None:
        try:
            route = self.graph.find_route(start_lm, goal_lm)
        except ValueError as exc:
            self._set_state("ERROR", str(exc))
            return

        route_nodes = route.nodes
        route_path = self.graph.sample_path(route, self.sample_distance)

        self._publish_route(route_path)
        with self.state_lock:
            self.current_route_nodes = route_nodes
            self.current_route_path = route_path
            self.last_route_start_lm = start_lm
        self.pending_follow_route_path = list(route_path)

        if start_lm == goal_lm:
            self.pending_route_goal_lm = ""
            self._set_state("ARRIVED", f"Robot is already at {goal_lm}.")
            return

        initial_yaw = float(route_path[0]["yaw"])
        if self.latest_pose is not None:
            yaw_error = self._normalize_angle(initial_yaw - self.latest_pose["yaw"])
        else:
            yaw_error = 0.0

        if abs(yaw_error) > self.rotate_yaw_tolerance:
            self._set_state(
                "ROTATE_TO_ROUTE",
                f"Rotating to first edge heading before motion: yaw error {abs(yaw_error):.3f} rad.",
            )
            self._send_rotate_to_route(initial_yaw)
            return

        self._set_state("FOLLOW_LM_ROUTE", f"Following strict LM route: {' -> '.join(route_nodes)}")
        self._send_follow_path(route_path)

    def _publish_route(self, route_path: list[dict[str, float | str]]) -> None:
        self.path_publisher.publish(self._build_path_msg(route_path))

    def _send_follow_path(self, route_path: list[dict[str, float | str]]) -> None:
        if not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self._set_state("ERROR", "/follow_path action server is not available.")
            return

        goal = FollowPath.Goal()
        goal.path = self._build_path_msg(route_path)
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id

        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self._handle_follow_path_response)

    def _handle_follow_path_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._set_state("ERROR", "FollowPath goal was rejected.")
            return

        self.follow_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_follow_path_result)

    def _handle_follow_path_result(self, future) -> None:
        result = future.result()
        self.follow_goal_handle = None
        if result is None:
            self._set_state("ERROR", "FollowPath returned no result.")
            return

        status = int(result.status)
        if status == 5:
            self._set_state("IDLE", "FollowPath canceled.")
            return
        if status != 4:
            self._set_state("ERROR", f"FollowPath failed with status={status}.")
            return

        goal_lm = self.active_goal_lm or self.pending_route_goal_lm
        with self.state_lock:
            self.pending_route_goal_lm = ""
        self._set_state("ARRIVED", f"Arrived at {goal_lm}.")

    def _build_path_msg(self, route_path: list[dict[str, float | str]]) -> NavPath:
        stamp = self.get_clock().now().to_msg()
        path_msg = NavPath()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = stamp

        for point in route_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = stamp
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            pose.pose.orientation = self._quaternion_from_yaw(float(point["yaw"]))
            path_msg.poses.append(pose)
        return path_msg

    def _stop_active_motion(self, reason: str) -> None:
        self._cancel_active_goals()
        with self.state_lock:
            self.pending_route_goal_lm = ""
            self.active_goal_lm = ""
        self._set_state("IDLE", reason)

    def _cancel_active_goals(self) -> None:
        if self.navigate_goal_handle is not None:
            self.navigate_goal_handle.cancel_goal_async()
            self.navigate_goal_handle = None
        if self.follow_goal_handle is not None:
            self.follow_goal_handle.cancel_goal_async()
            self.follow_goal_handle = None
        self.pending_navigate_mode = ""
        self.pending_rotate_yaw = None
        self.pending_follow_route_path = []

    def _set_state(self, mode: str, status_text: str) -> None:
        with self.state_lock:
            self.mode = mode
            self.status_text = status_text
        self.get_logger().info(status_text)

    def _start_web_server(self) -> None:
        node = self

        class RequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
                node.get_logger().debug(format % args)

            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                if parsed.path == "/api/config":
                    self._send_json(node.config_payload())
                    return
                if parsed.path == "/api/state":
                    self._send_json(node.state_payload())
                    return
                if parsed.path == "/" or parsed.path == "/index.html":
                    self._send_static("index.html", "text/html; charset=utf-8")
                    return
                if parsed.path == "/styles.css":
                    self._send_static("styles.css", "text/css; charset=utf-8")
                    return
                if parsed.path == "/app.js":
                    self._send_static("app.js", "application/javascript; charset=utf-8")
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                if parsed.path == "/api/goal-lm":
                    payload = self._read_json_body()
                    goal_lm = str(payload.get("goalLm", "")).strip()
                    if not goal_lm:
                        self._send_json({"ok": False, "error": "goalLm is required"}, status=HTTPStatus.BAD_REQUEST)
                        return
                    node.enqueue_goal(goal_lm)
                    self._send_json({"ok": True})
                    return
                if parsed.path == "/api/stop":
                    node.enqueue_stop()
                    self._send_json({"ok": True})
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def _read_json_body(self) -> dict[str, Any]:
                content_length = int(self.headers.get("Content-Length", "0"))
                if content_length <= 0:
                    return {}
                raw = self.rfile.read(content_length)
                return json.loads(raw.decode("utf-8"))

            def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_static(self, filename: str, content_type: str) -> None:
                path = node.web_dir / filename
                if not path.exists():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                body = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        self.web_server = ThreadingHTTPServer((self.web_host, self.web_port), RequestHandler)
        self.web_thread = threading.Thread(target=self.web_server.serve_forever, daemon=True)
        self.web_thread.start()
        self.get_logger().info(f"Operator panel: http://{self.web_host}:{self.web_port}")

    def destroy_node(self) -> bool:
        if self.web_server is not None:
            self.web_server.shutdown()
            self.web_server.server_close()
            self.web_server = None
        return super().destroy_node()

    def _yaw_from_quaternion(self, quaternion: Quaternion) -> float:
        siny_cosp = 2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y))
        cosy_cosp = 1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)))
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _quaternion_from_yaw(self, yaw: float) -> Quaternion:
        half_yaw = yaw * 0.5
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half_yaw),
            w=math.cos(half_yaw),
        )


def main() -> None:
    rclpy.init()
    node = LmRouteManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
