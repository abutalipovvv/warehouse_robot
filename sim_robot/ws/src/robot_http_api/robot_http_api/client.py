from __future__ import annotations

import json
from pathlib import Path
from threading import Lock
from time import monotonic, sleep, time
from typing import Any

from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_msgs.msg import RobotStatus
from robot_msgs.srv import CancelRoute, ExecuteRoute, GetRobotMapState, LoadRobotMap, PlanRoute
from robot_planner import (
    PlannedRobotRoute,
    Pose2D,
    RobotTrajectoryPlanner,
    load_route_params,
    route_update_is_stale,
    save_route_params,
)


class RobotRosClient:
    def __init__(
        self,
        *,
        robot_id: str,
        map_dir: Path,
        params_path: Path,
        status_topic: str,
        cmd_vel_topic: str,
        plan_service_name: str,
        execute_service_name: str,
        cancel_service_name: str,
        map_state_service_name: str,
        map_load_service_name: str,
    ) -> None:
        self.robot_id = robot_id
        self.params_path = Path(params_path).resolve()
        self.route_planner = RobotTrajectoryPlanner(
            map_dir=Path(map_dir).resolve(),
            params_path=self.params_path,
        )
        self.map_id = self.route_planner.map_id
        self.node = Node("robot_http_api_client")
        self._status_lock = Lock()
        self._latest_status: RobotStatus | None = None
        self._last_status_event_key: tuple[str, str] | None = None
        self._teleop_active = False
        self._teleop_deadline: float | None = None
        self._events: list[dict[str, Any]] = []
        self._active_route: PlannedRobotRoute | None = None

        self._plan_route_client = self.node.create_client(PlanRoute, plan_service_name)
        self._execute_route_client = self.node.create_client(ExecuteRoute, execute_service_name)
        self._cancel_route_client = self.node.create_client(CancelRoute, cancel_service_name)
        self._map_state_client = self.node.create_client(GetRobotMapState, map_state_service_name)
        self._map_load_client = self.node.create_client(LoadRobotMap, map_load_service_name)
        self._cmd_vel_pub = self.node.create_publisher(Twist, cmd_vel_topic, 20)
        self.node.create_subscription(RobotStatus, status_topic, self._on_robot_status, 20)
        self.node.create_timer(0.05, self._teleop_watchdog)

    def destroy(self) -> None:
        self.node.destroy_node()

    def site_payload(self) -> dict[str, Any]:
        return self.route_planner.site_payload(self.robot_id)

    def active_map_payload(self) -> dict[str, Any]:
        request = GetRobotMapState.Request()
        response = self._call_service(self._map_state_client, request, "map state", timeout_sec=5.0)
        if not bool(response.ok):
            raise ValueError(str(response.error or "map state failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
        }

    def sync_active_map_context(self) -> None:
        active = self.active_map_payload()
        map_dir = str(active.get("mapDir") or "").strip()
        if not map_dir:
            return
        self.reload_map_context(Path(map_dir))

    def params_payload(self) -> dict[str, Any]:
        return load_route_params(self.params_path, create=True)

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        saved_path = save_route_params(payload, self.params_path)
        params = load_route_params(self.params_path, create=True)
        self.route_planner.update_params(params)
        self.add_event("info", f"params saved: {saved_path}")
        return {
            "ok": True,
            "path": str(saved_path),
            "params": params,
        }

    def latest_status_payload(self) -> dict[str, Any]:
        with self._status_lock:
            message = self._latest_status
        if message is None:
            return {
                "robotId": self.robot_id,
                "mapId": self.map_id,
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
                "battery": {
                    "level": 0.0,
                    "voltage": 0.0,
                    "current": 0.0,
                    "temperature": 0.0,
                    "charging": False,
                },
            }

        route_id = str(message.route_id or "")
        if not route_id:
            self._active_route = None
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
            "routeId": route_id,
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
            "battery": {
                "level": float(message.battery_level),
                "voltage": float(message.battery_voltage),
                "current": float(message.battery_current),
                "temperature": float(message.battery_temperature),
                "charging": bool(message.battery_charging),
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

    def plan_route(self, pose: Pose2D, goal_lm: str, start_lm: str | None) -> PlannedRobotRoute:
        request = PlanRoute.Request()
        request.goal_lm = str(goal_lm)
        request.start_lm = str(start_lm or "")
        request.use_start_pose = True
        request.start_x = float(pose.x)
        request.start_y = float(pose.y)
        request.start_yaw = float(pose.yaw)
        response = self._call_service(self._plan_route_client, request, "route planner")
        if not bool(response.ok):
            raise ValueError(str(response.error or "route planning failed"))
        try:
            payload = json.loads(str(response.route_json or ""))
        except json.JSONDecodeError as exc:
            raise ValueError("route planner returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("route planner returned invalid route payload")
        route = PlannedRobotRoute.from_dict(payload)
        self._active_route = route
        return route

    def plan_route_payload(self, *, pose: dict[str, float], goal_lm: str, start_lm: str | None) -> dict[str, Any]:
        route = self.plan_route(
            pose=Pose2D(
                x=float(pose.get("x", 0.0) or 0.0),
                y=float(pose.get("y", 0.0) or 0.0),
                yaw=float(pose.get("yaw", 0.0) or 0.0),
            ),
            goal_lm=goal_lm,
            start_lm=start_lm,
        )
        return route.to_dict()

    def execute_route(self, route: PlannedRobotRoute) -> None:
        request = ExecuteRoute.Request()
        request.route_json = json.dumps(route.to_dict(), ensure_ascii=False)
        response = self._call_service(self._execute_route_client, request, "route execute")
        if not bool(response.ok):
            raise ValueError(str(response.error or "route execute failed"))
        self._active_route = route

    def execute_route_payload(self, route_payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_execute_payload(route_payload)
        if not route.goal_lm:
            raise ValueError("route.goalLm is required")
        self._reject_stale_route_update(route)
        self.execute_route(route)
        return route.to_dict()

    def _reject_stale_route_update(self, route: PlannedRobotRoute) -> None:
        status = self.latest_status_payload()
        active = self._active_route if status.get("routeId") else None
        if route_update_is_stale(active, route):
            raise ValueError(
                f"stale route revision: {route.route_id} rev {route.revision} "
                f"<= active rev {active.revision if active else 0}"
            )

    def _route_from_execute_payload(self, route_payload: dict[str, Any]) -> PlannedRobotRoute:
        if self._is_lm_route_payload(route_payload):
            pose = self.latest_pose()
            if pose is None:
                raise ValueError("robot pose is not available yet")
            return self.route_planner.plan_from_lm_route(pose, route_payload)
        return PlannedRobotRoute.from_dict(route_payload)

    def _is_lm_route_payload(self, route_payload: dict[str, Any]) -> bool:
        protocol = str(route_payload.get("protocol") or route_payload.get("routeProtocol") or "").strip().lower()
        if protocol in {"lm_route", "lm-route", "lmroute"}:
            return True
        trajectory = route_payload.get("trajectory")
        nodes = route_payload.get("nodes") or route_payload.get("routeNodes") or route_payload.get("route_nodes")
        return not isinstance(trajectory, list) and isinstance(nodes, list)

    def load_map(self, map_name: str) -> dict[str, Any]:
        request = LoadRobotMap.Request()
        request.map_name = str(map_name)
        request.map_dir = ""
        response = self._call_service(self._map_load_client, request, "map load", timeout_sec=20.0)
        if not bool(response.ok):
            raise ValueError(str(response.error or "map load failed"))
        self.reload_map_context(Path(str(response.map_dir)))
        self._active_route = None
        self.add_event("warn", f"active map changed: {response.map_name}")
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
        }

    def reload_map_context(self, map_dir: Path) -> None:
        self.route_planner.reload_map(Path(map_dir).resolve())
        self.map_id = self.route_planner.map_id

    def cancel_route(self, message: str = "Route canceled.") -> None:
        request = CancelRoute.Request()
        request.message = str(message)
        response = self._call_service(self._cancel_route_client, request, "route cancel")
        if not bool(response.ok):
            raise ValueError(str(response.error or "route cancel failed"))
        self._active_route = None

    def teleop(self, linear: float, angular: float, timeout_ms: int) -> None:
        if self._route_is_active():
            self.cancel_route("Manual control takeover.")
        self._publish_cmd_vel(linear, angular)
        self._teleop_active = True
        self._teleop_deadline = monotonic() + max(0.08, timeout_ms / 1000.0)

    def teleop_stop(self) -> None:
        self._teleop_active = False
        self._teleop_deadline = None
        self._publish_cmd_vel(0.0, 0.0)

    def stop(self) -> None:
        if self._route_is_active():
            self.cancel_route("Stopped.")
        self._teleop_active = False
        self._teleop_deadline = None
        self._publish_cmd_vel(0.0, 0.0)

    def add_event(self, level: str, message: str) -> None:
        self._events.append({"stamp": time(), "level": level, "message": message})
        self._events = self._events[-120:]

    def events_payload(self) -> list[dict[str, Any]]:
        return list(self._events)

    def active_route_payload(self) -> dict[str, Any] | None:
        if self._active_route is None:
            return None
        return self._active_route.to_dict()

    def _route_is_active(self) -> bool:
        status = self.latest_status_payload()
        return bool(status.get("routeId")) or self._active_route is not None

    def _publish_cmd_vel(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._cmd_vel_pub.publish(message)

    def _teleop_watchdog(self) -> None:
        if not self._teleop_active or self._teleop_deadline is None:
            return
        if monotonic() <= self._teleop_deadline:
            return
        self._teleop_active = False
        self._teleop_deadline = None
        self._publish_cmd_vel(0.0, 0.0)

    def _call_service(self, client, request, service_label: str, *, timeout_sec: float = 3.0, wait_for_service_sec: float = 1.0):
        if not client.wait_for_service(timeout_sec=wait_for_service_sec):
            raise ValueError(f"{service_label} service is not available")
        future = client.call_async(request)
        deadline = monotonic() + max(0.5, float(timeout_sec))
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
        self.add_event(level, self._humanize_status_event(state, raw_message))

    def _humanize_status_event(self, state: str, message: str) -> str:
        text = message.strip() or state
        lowered = text.lower()
        if state == "ERROR" and "localization transform timeout" in lowered:
            return (
                f"Localization error: {text}. Robot pose from map->base_link became stale. "
                "Check /tf, /odom, /amcl_pose, and map alignment."
            )
        if state == "ERROR" and "waiting for amcl pose" in lowered:
            return "Localization error: amcl pose is missing. Set initial pose and verify /amcl_pose."
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
