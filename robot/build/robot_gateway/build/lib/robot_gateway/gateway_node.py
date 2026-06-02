from __future__ import annotations

import json
from threading import Lock
from time import monotonic, sleep
from typing import Any

import rclpy
from rclpy.node import Node

from robot_msgs.msg import RobotStatus
from robot_msgs.srv import CancelRoute, ExecuteRoute, PlanRoute, ReleaseManual, SetTeleop, StopRobot
from robot_planner import PlannedRobotRoute, Pose2D, RobotRuntime


class RobotRosBridgeNode(Node):
    def __init__(
        self,
        runtime: RobotRuntime,
        status_topic: str,
        plan_service_name: str,
        execute_service_name: str,
        cancel_service_name: str,
        teleop_service_name: str,
        release_manual_service_name: str,
        stop_service_name: str,
    ) -> None:
        super().__init__("robot_gateway")
        self.runtime = runtime
        self._status_lock = Lock()
        self._latest_status: RobotStatus | None = None
        self._last_status_event_key: tuple[str, str] | None = None
        self._plan_route_client = self.create_client(PlanRoute, plan_service_name)
        self._execute_route_client = self.create_client(ExecuteRoute, execute_service_name)
        self._cancel_route_client = self.create_client(CancelRoute, cancel_service_name)
        self._teleop_client = self.create_client(SetTeleop, teleop_service_name)
        self._release_manual_client = self.create_client(ReleaseManual, release_manual_service_name)
        self._stop_client = self.create_client(StopRobot, stop_service_name)
        self.create_subscription(RobotStatus, status_topic, self._on_robot_status, 20)

    def latest_status_payload(self) -> dict[str, Any]:
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

    def latest_pose(self) -> Pose2D | None:
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

    def plan_route_via_service(
        self,
        pose: Pose2D,
        goal_lm: str,
        start_lm: str | None,
    ) -> PlannedRobotRoute:
        request = PlanRoute.Request()
        request.goal_lm = str(goal_lm)
        request.start_lm = str(start_lm or "")
        request.use_start_pose = True
        request.start_x = float(pose.x)
        request.start_y = float(pose.y)
        request.start_yaw = float(pose.yaw)
        response = self._call_service(
            client=self._plan_route_client,
            request=request,
            service_label="route planner",
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "route planning failed"))

        try:
            payload = json.loads(str(response.route_json or ""))
        except json.JSONDecodeError as exc:
            raise ValueError("route planner returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("route planner returned invalid route payload")
        return PlannedRobotRoute.from_dict(payload)

    def execute_route(self, route: PlannedRobotRoute) -> None:
        request = ExecuteRoute.Request()
        request.route_json = json.dumps(route.to_dict(), ensure_ascii=False)
        response = self._call_service(
            client=self._execute_route_client,
            request=request,
            service_label="route execute",
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "route execute failed"))

    def cancel_route(self, message: str = "Route canceled.") -> None:
        request = CancelRoute.Request()
        request.message = str(message)
        response = self._call_service(
            client=self._cancel_route_client,
            request=request,
            service_label="route cancel",
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "route cancel failed"))

    def set_teleop(self, linear: float, angular: float, timeout_ms: int) -> None:
        request = SetTeleop.Request()
        request.linear = float(linear)
        request.angular = float(angular)
        request.timeout_ms = int(timeout_ms)
        response = self._call_service(
            client=self._teleop_client,
            request=request,
            service_label="teleop",
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "teleop failed"))

    def release_manual(self) -> None:
        request = ReleaseManual.Request()
        response = self._call_service(
            client=self._release_manual_client,
            request=request,
            service_label="release manual",
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "release manual failed"))

    def stop_robot(self, message: str = "Stopped.") -> None:
        request = StopRobot.Request()
        request.message = str(message)
        response = self._call_service(
            client=self._stop_client,
            request=request,
            service_label="stop robot",
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "stop robot failed"))

    def _call_service(self, client, request, service_label: str):
        if not client.wait_for_service(timeout_sec=1.0):
            raise ValueError(f"{service_label} service is not available")

        future = client.call_async(request)
        deadline = monotonic() + 3.0
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

        if state == "ERROR" and "localization transform timeout" in lowered:
            return (
                f"Localization error: {text}. Robot pose from map->base_link became stale. "
                "Check /tf, /odom, /amcl_pose, and map alignment."
            )
        if state == "ERROR" and "localization timeout" in lowered:
            return (
                f"Localization error: {text}. Robot pose became stale. "
                "Check /scan, /amcl_pose, /tf, and map alignment."
            )
        if state == "LOCALIZING" and "waiting for amcl pose" in lowered:
            return "Localization waiting: AMCL pose has not been received yet. Set initial pose and verify /amcl_pose."
        if state != "ERROR" and "amcl correction is" in lowered:
            return f"Localization warning: {text}"
        if state == "LOCALIZING" and "timeout" in lowered:
            return f"Localization warning: {text}. The last AMCL update is too old."
        if state == "ERROR" and "robot pose is not available" in lowered:
            return "Route execution error: robot pose is not available for planning."
        if state != "ERROR" and state != "LOCALIZING":
            return f"Recovered from error: {text}."
        return f"{state}: {text}"


def main() -> None:
    rclpy.init(args=None)
    node = RobotRosBridgeNode(
        runtime=RobotRuntime(robot_id="robot", map_id="map"),
        status_topic="/robot_status",
        plan_service_name="/route/plan",
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
