from __future__ import annotations

import json

import rclpy
from rclpy.node import Node

from robot_msgs.srv import CancelRoute, ExecuteRoute, ReleaseManual, SetTeleop, StopRobot
from robot_planner import PlannedRobotRoute, RobotRuntime


class RobotCommandNode(Node):
    def __init__(
        self,
        runtime: RobotRuntime,
        execute_service_name: str,
        cancel_service_name: str,
        teleop_service_name: str,
        release_manual_service_name: str,
        stop_service_name: str,
    ) -> None:
        super().__init__("robot_command")
        self.runtime = runtime
        self.create_service(ExecuteRoute, execute_service_name, self._handle_execute_route)
        self.create_service(CancelRoute, cancel_service_name, self._handle_cancel_route)
        self.create_service(SetTeleop, teleop_service_name, self._handle_set_teleop)
        self.create_service(ReleaseManual, release_manual_service_name, self._handle_release_manual)
        self.create_service(StopRobot, stop_service_name, self._handle_stop_robot)

    def _handle_execute_route(self, request, response):
        try:
            payload = json.loads(str(request.route_json or ""))
            if not isinstance(payload, dict):
                raise ValueError("route_json must contain an object")
            route = PlannedRobotRoute.from_dict(payload)
            if not route.goal_lm:
                raise ValueError("route.goalLm is required")
            self.runtime.set_route(route)
            self.runtime.add_event("info", f"executing route {route.route_id} -> {route.goal_lm}")
            response.ok = True
            response.error = ""
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response

    def _handle_cancel_route(self, request, response):
        try:
            message = str(request.message or "").strip() or "Route canceled."
            self.runtime.cancel_route(message)
            self.runtime.clear_manual()
            self.runtime.add_event("warn", "route canceled")
            response.ok = True
            response.error = ""
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response

    def _handle_set_teleop(self, request, response):
        try:
            snapshot = self.runtime.snapshot()
            timeout_sec = max(0.08, int(request.timeout_ms) / 1000.0)
            self.runtime.set_manual_command(
                linear=float(request.linear),
                angular=float(request.angular),
                timeout_sec=timeout_sec,
            )
            if snapshot.get("state") != "MANUAL":
                self.runtime.add_event("info", "manual control engaged")
            response.ok = True
            response.error = ""
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response

    def _handle_release_manual(self, request, response):
        del request
        try:
            self.runtime.clear_manual()
            response.ok = True
            response.error = ""
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response

    def _handle_stop_robot(self, request, response):
        try:
            message = str(request.message or "").strip() or "Stopped."
            self.runtime.clear_manual()
            self.runtime.cancel_route(message)
            self.runtime.add_event("warn", "robot stopped")
            response.ok = True
            response.error = ""
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
        return response


def main() -> None:
    rclpy.init(args=None)
    node = RobotCommandNode(
        runtime=RobotRuntime(robot_id="robot", map_id="map"),
        execute_service_name="/route/execute",
        cancel_service_name="/route/cancel",
        teleop_service_name="/robot/teleop/set",
        release_manual_service_name="/robot/teleop/release",
        stop_service_name="/robot/stop",
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
