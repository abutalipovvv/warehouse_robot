from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_msgs.msg import ExecutorState, RobotStatus
from robot_msgs.srv import CancelRoute, ExecuteRoute, LoadRobotMap, PlanRoute

from .executor import RouteExecutor
from .route_planner import RobotTrajectoryPlanner
from .runtime import PlannedRobotRoute, Pose2D, RobotRuntime, route_update_is_stale


class RobotRouteNode(Node):
    def __init__(
        self,
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        *,
        cmd_vel_topic: str,
        status_topic: str,
        executor_status_topic: str,
        plan_service_name: str,
        execute_service_name: str,
        cancel_service_name: str,
        load_map_service_name: str,
    ) -> None:
        super().__init__("robot_route")
        self.runtime = runtime
        self.route_planner = route_planner
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 20)
        self._executor_state_pub = self.create_publisher(ExecutorState, executor_status_topic, 20)
        self._executor = RouteExecutor(runtime, route_planner, self._publish_cmd_vel)
        self._latest_status = self._default_status_payload()
        self._last_motion_command = False
        self.create_subscription(RobotStatus, status_topic, self._on_robot_status, 20)
        self.create_service(PlanRoute, plan_service_name, self._handle_plan_route)
        self.create_service(ExecuteRoute, execute_service_name, self._handle_execute_route)
        self.create_service(CancelRoute, cancel_service_name, self._handle_cancel_route)
        self.create_service(LoadRobotMap, load_map_service_name, self._handle_load_map)
        self.create_timer(0.05, self._control_step)

    def _on_robot_status(self, message: RobotStatus) -> None:
        self._latest_status = {
            "robotId": message.robot_id,
            "mapId": message.map_id,
            "connected": bool(message.connected),
            "localizationOk": bool(message.localization_ok),
            "localizationAgeSec": float(message.localization_age_sec),
            "state": str(message.state or ""),
            "message": str(message.message or ""),
            "targetLm": str(message.target_lm or ""),
            "nearestLm": str(message.nearest_lm or ""),
            "currentEdgeId": str(message.current_edge_id or ""),
            "routeId": str(message.route_id or ""),
            "routeProgress": float(message.route_progress),
            "pose": {
                "x": float(message.pose_x),
                "y": float(message.pose_y),
                "yaw": float(message.pose_yaw),
            } if bool(message.localization_ok) else None,
            "velocity": {
                "linear": float(message.linear_velocity),
                "angular": float(message.angular_velocity),
            },
        }

    def _control_step(self) -> None:
        had_motion = self._last_motion_command
        self._executor.control_step(self._latest_status)
        if had_motion and not self._command_is_active():
            self._publish_cmd_vel(0.0, 0.0)
        self._publish_executor_state()

    def _command_is_active(self) -> bool:
        snapshot = self.runtime.snapshot()
        route = snapshot.get("route")
        return bool(snapshot.get("targetLm") or route is not None)

    def _handle_plan_route(self, request, response):
        try:
            if not bool(request.use_start_pose):
                raise ValueError("start pose is required")
            pose = Pose2D(
                x=float(request.start_x),
                y=float(request.start_y),
                yaw=float(request.start_yaw),
            )
            start_lm = str(request.start_lm or "").strip() or None
            route = self.route_planner.plan_from_pose(
                pose=pose,
                goal_lm=str(request.goal_lm or "").strip(),
                start_lm=start_lm,
            )
            response.ok = True
            response.error = ""
            response.route_json = json.dumps(route.to_dict(), ensure_ascii=False)
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.route_json = ""
        return response

    def _handle_execute_route(self, request, response):
        try:
            payload = json.loads(str(request.route_json or ""))
            if not isinstance(payload, dict):
                raise ValueError("route_json must contain an object")
            route = self._route_from_execute_payload(payload)
            if not route.goal_lm:
                raise ValueError("route.goalLm is required")
            active_route = self.runtime.active_route()
            if route_update_is_stale(active_route, route):
                raise ValueError(
                    f"stale route revision: {route.route_id} rev {route.revision} "
                    f"<= active rev {active_route.revision if active_route else 0}"
                )
            replacing = active_route is not None and (
                active_route.route_id != route.route_id
                or active_route.revision != route.revision
            )
            if replacing and route.replace_mode == "immediate":
                self._publish_cmd_vel(0.0, 0.0)
            self.runtime.set_route(route)
            verb = "replacing" if replacing else "executing"
            self.runtime.add_event(
                "info",
                f"{verb} route {route.route_id} rev {route.revision} -> {route.goal_lm}",
            )
            response.ok = True
            response.error = ""
            self._publish_executor_state()
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response

    def _route_from_execute_payload(self, payload: dict[str, object]) -> PlannedRobotRoute:
        if self._is_lm_route_payload(payload):
            pose_payload = self._latest_status.get("pose")
            if not isinstance(pose_payload, dict):
                raise ValueError("robot pose is not available yet")
            pose = Pose2D(
                x=float(pose_payload.get("x", 0.0) or 0.0),
                y=float(pose_payload.get("y", 0.0) or 0.0),
                yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
            )
            return self.route_planner.plan_from_lm_route(pose, payload)
        return PlannedRobotRoute.from_dict(payload)

    def _is_lm_route_payload(self, payload: dict[str, object]) -> bool:
        protocol = str(payload.get("protocol") or payload.get("routeProtocol") or "").strip().lower()
        if protocol in {"lm_route", "lm-route", "lmroute"}:
            return True
        trajectory = payload.get("trajectory")
        nodes = payload.get("nodes") or payload.get("routeNodes") or payload.get("route_nodes")
        return not isinstance(trajectory, list) and isinstance(nodes, list)

    def _handle_cancel_route(self, request, response):
        try:
            message = str(request.message or "").strip() or "Route canceled."
            self.runtime.cancel_route(message)
            self._publish_cmd_vel(0.0, 0.0)
            self.runtime.add_event("warn", "route canceled")
            response.ok = True
            response.error = ""
            self._publish_executor_state()
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response

    def _handle_load_map(self, request, response):
        try:
            map_dir = Path(str(request.map_dir or "")).resolve()
            if not map_dir.is_dir():
                raise ValueError(f"map_dir does not exist: {map_dir}")
            same_map = map_dir == self.route_planner.map_dir
            if not same_map:
                self.runtime.cancel_route("Map changed.")
                self._publish_cmd_vel(0.0, 0.0)
            self.route_planner.reload_params_from_disk()
            if not same_map:
                self.route_planner.reload_map(map_dir)
            self.runtime.set_map(self.route_planner.map_id)
            event = "params reloaded" if same_map else f"map reloaded: {self.route_planner.map_id}"
            self.runtime.add_event("warn", event)
            response.ok = True
            response.error = ""
            response.map_name = str(request.map_name or self.route_planner.map_id)
            response.map_dir = str(self.route_planner.map_dir)
            response.map_id = self.route_planner.map_id
            self._publish_executor_state()
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.map_name = ""
            response.map_dir = ""
            response.map_id = ""
        return response

    def _publish_cmd_vel(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._cmd_vel_pub.publish(message)
        self._last_motion_command = abs(float(linear)) > 1e-6 or abs(float(angular)) > 1e-6

    def _publish_executor_state(self) -> None:
        snapshot = self.runtime.snapshot()
        route = snapshot.get("route") if isinstance(snapshot.get("route"), dict) else None
        message = ExecutorState()
        message.stamp = self.get_clock().now().to_msg()
        message.robot_id = self.runtime.robot_id
        message.map_id = self.runtime.map_id
        message.route_active = route is not None
        message.state = str(snapshot.get("state") or "IDLE")
        message.message = str(snapshot.get("message") or "")
        message.target_lm = str(snapshot.get("targetLm") or "")
        message.current_edge_id = str(snapshot.get("currentEdgeId") or "")
        message.route_id = str(route.get("routeId") or "") if isinstance(route, dict) else ""
        message.route_progress = float(snapshot.get("routeProgress", 0.0) or 0.0)
        self._executor_state_pub.publish(message)

    def _default_status_payload(self) -> dict[str, object]:
        return {
            "robotId": self.runtime.robot_id,
            "mapId": self.runtime.map_id,
            "connected": True,
            "localizationOk": False,
            "localizationAgeSec": 9999.0,
            "state": "LOCALIZING",
            "message": "Waiting for amcl pose.",
            "targetLm": "",
            "nearestLm": "",
            "currentEdgeId": "",
            "routeId": "",
            "routeProgress": 0.0,
            "pose": None,
            "velocity": {"linear": 0.0, "angular": 0.0},
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run robot route planner/executor node.")
    parser.add_argument("--map-dir", required=True, type=Path)
    parser.add_argument("--params", required=True, type=Path)
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--status-topic", default="/robot_status")
    parser.add_argument("--executor-status-topic", default="/route/executor_state")
    parser.add_argument("--plan-service", default="/route/plan")
    parser.add_argument("--execute-service", default="/route/execute")
    parser.add_argument("--cancel-service", default="/route/cancel")
    parser.add_argument("--load-map-service", default="/route/load_map")
    args, _unknown = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    route_planner = RobotTrajectoryPlanner(
        map_dir=args.map_dir,
        params_path=args.params,
    )
    runtime = RobotRuntime(robot_id=args.robot_id, map_id=route_planner.map_id)
    rclpy.init(args=None)
    node = RobotRouteNode(
        runtime=runtime,
        route_planner=route_planner,
        cmd_vel_topic=args.cmd_vel_topic,
        status_topic=args.status_topic,
        executor_status_topic=args.executor_status_topic,
        plan_service_name=args.plan_service,
        execute_service_name=args.execute_service,
        cancel_service_name=args.cancel_service,
        load_map_service_name=args.load_map_service,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
