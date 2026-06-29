from __future__ import annotations

import json
import re
import threading
from time import monotonic, sleep
from typing import Any


def _clean_node_suffix(value: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_]", "_", value.strip())
    clean = re.sub(r"_+", "_", clean).strip("_")
    if not clean or clean[0].isdigit():
        clean = f"robot_{clean or 'api'}"
    return clean[:64]


class RosRobotRuntime:
    def __init__(
        self,
        *,
        robot_id: str,
        robot_name: str,
        host: str = "",
        domain_id: int | None = None,
        namespace: str = "",
        status_topic: str = "/robot_status",
        cmd_vel_topic: str = "/cmd_vel",
        go_to_lm_topic: str = "/go_to_lm",
        plan_service_name: str = "/route/plan",
        execute_service_name: str = "/route/execute",
        cancel_service_name: str = "/route/cancel",
        map_state_service_name: str = "/robot/maps/state",
        map_load_service_name: str = "/robot/maps/load",
        map_list_service_name: str = "/robot/maps/list",
        map_get_bundle_service_name: str = "/robot/maps/get_bundle",
        map_put_bundle_service_name: str = "/robot/maps/put_bundle",
    ) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_name
        self.host = host
        self.domain_id = domain_id
        self.namespace = namespace.strip().strip("/")
        self.status_topic = self._topic(status_topic)
        self.cmd_vel_topic = self._topic(cmd_vel_topic)
        self.go_to_lm_topic = self._topic(go_to_lm_topic)
        self.plan_service_name = self._topic(plan_service_name)
        self.execute_service_name = self._topic(execute_service_name)
        self.cancel_service_name = self._topic(cancel_service_name)
        self.map_state_service_name = self._topic(map_state_service_name)
        self.map_load_service_name = self._topic(map_load_service_name)
        self.map_list_service_name = self._topic(map_list_service_name)
        self.map_get_bundle_service_name = self._topic(map_get_bundle_service_name)
        self.map_put_bundle_service_name = self._topic(map_put_bundle_service_name)
        self._lock = threading.Lock()
        self._latest_status: Any | None = None
        self._latest_status_at: float | None = None
        self._events: list[dict[str, Any]] = []
        self._available = False
        self._error = ""
        self._node = None
        self._rclpy = None
        self._context = None
        self._twist_type = None
        self._string_type = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._cmd_vel_pub = None
        self._go_to_lm_pub = None
        self._plan_route_client = None
        self._execute_route_client = None
        self._cancel_route_client = None
        self._map_state_client = None
        self._map_load_client = None
        self._map_list_client = None
        self._map_get_bundle_client = None
        self._map_put_bundle_client = None
        self._start()

    @property
    def available(self) -> bool:
        return self._available

    @property
    def error(self) -> str:
        return self._error

    def _topic(self, topic: str) -> str:
        raw = str(topic or "").strip() or "/"
        if not self.namespace:
            return raw if raw.startswith("/") else f"/{raw}"
        return f"/{self.namespace}/{raw.strip('/')}"

    def close(self) -> None:
        executor = self._executor
        node = self._node
        context = self._context
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:
                pass

    def identity_payload(self) -> dict[str, Any]:
        status = self._message_to_robot_payload(self._latest_message())
        return {
            "ok": True,
            "robotId": status.get("robotId") or self.robot_name,
            "mapId": status.get("mapId") or "",
            "type": "ros2",
            "host": self.host,
            "domainId": self.domain_id,
            "namespace": self.namespace,
            "statusTopic": self.status_topic,
            "cmdVelTopic": self.cmd_vel_topic,
            "goToLmTopic": self.go_to_lm_topic,
            "planService": self.plan_service_name,
            "executeService": self.execute_service_name,
            "cancelService": self.cancel_service_name,
            "mapStateService": self.map_state_service_name,
            "mapLoadService": self.map_load_service_name,
            "mapListService": self.map_list_service_name,
            "mapGetBundleService": self.map_get_bundle_service_name,
            "mapPutBundleService": self.map_put_bundle_service_name,
            "available": self._available,
            "error": self._error,
        }

    def sidebar_payload(self) -> dict[str, Any]:
        status_payload = self.status_payload()
        robot_status = status_payload.get("robot") if isinstance(status_payload.get("robot"), dict) else {}
        identity = self.identity_payload()
        return {
            "id": self.robot_id,
            "name": self.robot_name,
            "host": self.host or "DDS",
            "port": 0,
            "baseUrl": "",
            "type": "ros2",
            "mode": "ros2",
            "domainId": self.domain_id,
            "namespace": self.namespace,
            "online": bool(robot_status.get("connected")),
            "identity": identity,
            "lastIdentity": identity,
            "status": robot_status,
            "error": "" if bool(robot_status.get("connected")) else (self._error or str(robot_status.get("message") or "")),
            "probed": True,
        }

    def status_payload(self) -> dict[str, Any]:
        message = self._latest_message()
        robot = self._message_to_robot_payload(message)
        return {
            "ok": True,
            "robot": robot,
            "events": list(self._events[-120:]),
            "route": self._route_payload(robot),
        }

    def status_robot_payload(self) -> dict[str, Any]:
        return self.status_payload()["robot"]

    def teleop(self, *, linear: float, angular: float, timeout_ms: int = 350) -> dict[str, Any]:
        del timeout_ms
        self._publish_twist(linear, angular)
        return {"ok": True, "linear": float(linear), "angular": float(angular)}

    def teleop_stop(self) -> dict[str, Any]:
        self._publish_twist(0.0, 0.0)
        return {"ok": True}

    def stop(self) -> dict[str, Any]:
        self._publish_twist(0.0, 0.0)
        try:
            self.cancel_route()
        except Exception:
            self._publish_go_to_lm("cancel")
        return {"ok": True}

    def cancel_route(self) -> dict[str, Any]:
        if self._cancel_route_client is not None and self._service_available(self._cancel_route_client, 0.05):
            request = self._cancel_route_client.srv_type.Request()
            request.message = "Route canceled."
            response = self._call_service(self._cancel_route_client, request, "route cancel")
            if not bool(response.ok):
                raise ValueError(str(response.error or "route cancel failed"))
        else:
            self._publish_go_to_lm("cancel")
        return {"ok": True}

    def execute_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        goal_pose = self._goal_pose_payload(payload)
        if goal_pose is not None:
            return self._execute_pose_route(goal_pose, payload)

        if self._execute_route_client is not None and self._service_available(self._execute_route_client, 0.05):
            route_payload = self._route_payload_from_request(payload)
            request = self._execute_route_client.srv_type.Request()
            request.route_json = json.dumps(route_payload, ensure_ascii=False)
            response = self._call_service(self._execute_route_client, request, "route execute")
            if not bool(response.ok):
                raise ValueError(str(response.error or "route execute failed"))
            return {"ok": True, "route": route_payload}

        goal_lm = str(
            payload.get("goalLm")
            or payload.get("goal_lm")
            or payload.get("targetLm")
            or payload.get("target_lm")
            or payload.get("id")
            or ""
        ).strip()
        if not goal_lm:
            raise ValueError("ROS2 local robot currently needs goalLm/id for /go_to_lm navigation")
        source_lm = str(payload.get("startLm") or payload.get("start_lm") or "").strip()
        command: dict[str, Any] = {"id": goal_lm}
        if source_lm:
            command["source_id"] = source_lm
        self._publish_go_to_lm(json.dumps(command, ensure_ascii=False))
        route = {
            "routeId": f"ros2-{goal_lm}",
            "goalLm": goal_lm,
            "startLm": source_lm,
            "nodes": [item for item in [source_lm, goal_lm] if item],
            "trajectory": [],
        }
        return {"ok": True, "route": route}

    def active_map_payload(self) -> dict[str, Any]:
        if self._map_state_client is None:
            raise ValueError("map state service is not configured")
        request = self._map_state_client.srv_type.Request()
        response = self._call_service(self._map_state_client, request, "map state", timeout_sec=5.0)
        if not bool(response.ok):
            raise ValueError(str(response.error or "map state failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
        }

    def list_maps_payload(self) -> dict[str, Any]:
        if self._map_list_client is None:
            raise ValueError("map list service is not configured")
        response = self._call_service(
            self._map_list_client,
            self._map_list_client.srv_type.Request(),
            "map list",
            timeout_sec=5.0,
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "map list failed"))
        maps = []
        for name, map_dir, map_id in zip(response.map_names, response.map_dirs, response.map_ids):
            maps.append(
                {
                    "name": str(name),
                    "folder": f"{name}.smap" if str(name) and not str(name).endswith(".smap") else str(name),
                    "mapDir": str(map_dir),
                    "mapId": str(map_id),
                    "active": str(name) == str(response.active_map_name),
                }
            )
        return {
            "ok": True,
            "active": str(response.active_map_name or ""),
            "activeMapDir": str(response.active_map_dir or ""),
            "activeMapId": str(response.active_map_id or ""),
            "maps": maps,
        }

    def pull_map_bundle_payload(self, map_name: str = "") -> dict[str, Any]:
        if self._map_get_bundle_client is None:
            raise ValueError("map bundle service is not configured")
        request = self._map_get_bundle_client.srv_type.Request()
        request.map_name = str(map_name or "")
        response = self._call_service(self._map_get_bundle_client, request, "map bundle pull", timeout_sec=20.0)
        if not bool(response.ok):
            raise ValueError(str(response.error or "map bundle pull failed"))
        try:
            payload = json.loads(str(response.bundle_json or "{}"))
        except json.JSONDecodeError as exc:
            raise ValueError("map bundle service returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("map bundle service returned invalid payload")
        payload.setdefault("ok", True)
        payload.setdefault("mapName", str(response.map_name or ""))
        payload.setdefault("mapDir", str(response.map_dir or ""))
        payload.setdefault("signature", str(response.signature or ""))
        return payload

    def push_map_bundle_payload(
        self,
        bundle_payload: dict[str, Any],
        *,
        map_name: str = "",
        activate: bool = False,
    ) -> dict[str, Any]:
        if self._map_put_bundle_client is None:
            raise ValueError("map bundle push service is not configured")
        request = self._map_put_bundle_client.srv_type.Request()
        request.map_name = str(map_name or bundle_payload.get("mapName") or "")
        request.bundle_json = json.dumps(bundle_payload, ensure_ascii=False)
        request.activate = bool(activate)
        response = self._call_service(self._map_put_bundle_client, request, "map bundle push", timeout_sec=30.0)
        if not bool(response.ok):
            raise ValueError(str(response.error or "map bundle push failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
            "signature": str(response.signature or ""),
        }

    def load_map(self, map_name: str) -> dict[str, Any]:
        if self._map_load_client is None:
            raise ValueError("map load service is not configured")
        request = self._map_load_client.srv_type.Request()
        request.map_name = str(map_name)
        request.map_dir = ""
        response = self._call_service(self._map_load_client, request, "map load", timeout_sec=20.0)
        if not bool(response.ok):
            raise ValueError(str(response.error or "map load failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
        }

    def _execute_pose_route(self, pose: dict[str, float], payload: dict[str, Any]) -> dict[str, Any]:
        script_args: dict[str, Any] = {
            "x": pose["x"],
            "y": pose["y"],
            "theta": pose["yaw"],
            "coordinate": "world",
        }
        for key in ("reachAngle", "reachDist", "backMode", "useOdo"):
            if key in payload:
                script_args[key] = payload[key]
        command = {
            "id": "SELF_POSITION",
            "source_id": "SELF_POSITION",
            "operation": "Script",
            "script_name": "syspy/goPath.py",
            "script_args": script_args,
        }
        self._publish_go_to_lm(json.dumps(command, ensure_ascii=False))
        label = f"pose({pose['x']:.3f},{pose['y']:.3f},{pose['yaw']:.3f})"
        route = {
            "routeId": f"ros2-{label}",
            "goalPose": pose,
            "goalLm": "",
            "nodes": [label],
            "trajectory": [],
        }
        return {"ok": True, "route": route}

    def _route_payload_from_request(self, payload: dict[str, Any]) -> dict[str, Any]:
        route_payload = payload.get("route")
        if isinstance(route_payload, dict):
            return route_payload

        goal_lm = str(payload.get("goalLm") or payload.get("targetLm") or "").strip()
        if not goal_lm:
            raise ValueError("goalLm is required")
        if self._plan_route_client is None or not self._service_available(self._plan_route_client, 0.05):
            return {
                "protocol": "lm_route",
                "goalLm": goal_lm,
                "startLm": str(payload.get("startLm") or "").strip(),
                "nodes": [item for item in [str(payload.get("startLm") or "").strip(), goal_lm] if item],
            }

        pose_payload = payload.get("startPose")
        if isinstance(pose_payload, dict):
            pose = {
                "x": float(pose_payload.get("x", 0.0) or 0.0),
                "y": float(pose_payload.get("y", 0.0) or 0.0),
                "yaw": float(pose_payload.get("yaw", 0.0) or 0.0),
            }
        else:
            status = self.status_robot_payload()
            pose = status.get("pose") if isinstance(status.get("pose"), dict) else None
            if not isinstance(pose, dict):
                raise ValueError("robot pose is not available yet")

        request = self._plan_route_client.srv_type.Request()
        request.goal_lm = goal_lm
        request.start_lm = str(payload.get("startLm") or "").strip()
        request.use_start_pose = True
        request.start_x = float(pose.get("x", 0.0) or 0.0)
        request.start_y = float(pose.get("y", 0.0) or 0.0)
        request.start_yaw = float(pose.get("yaw", 0.0) or 0.0)
        response = self._call_service(self._plan_route_client, request, "route plan")
        if not bool(response.ok):
            raise ValueError(str(response.error or "route planning failed"))
        try:
            route = json.loads(str(response.route_json or "{}"))
        except json.JSONDecodeError as exc:
            raise ValueError("route planner returned invalid JSON") from exc
        if not isinstance(route, dict):
            raise ValueError("route planner returned invalid route payload")
        return route

    def _goal_pose_payload(self, payload: dict[str, Any]) -> dict[str, float] | None:
        source = (
            payload.get("goalPose")
            or payload.get("goal_pose")
            or payload.get("targetPose")
            or payload.get("target_pose")
            or payload.get("pose")
        )
        if source is None and ("x" in payload or "y" in payload):
            source = payload
        if not isinstance(source, dict):
            return None
        if "x" not in source or "y" not in source:
            return None
        try:
            x = float(source.get("x", 0.0) or 0.0)
            y = float(source.get("y", 0.0) or 0.0)
            yaw = float(source.get("yaw", source.get("theta", 0.0)) or 0.0)
        except (TypeError, ValueError) as exc:
            raise ValueError("goalPose x/y/yaw must be numeric") from exc
        return {"x": x, "y": y, "yaw": yaw}

    def _start(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from geometry_msgs.msg import Twist
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from robot_msgs.msg import RobotStatus
            from robot_msgs.srv import (
                CancelRoute,
                ExecuteRoute,
                GetRobotMapBundle,
                GetRobotMapState,
                ListRobotMaps,
                LoadRobotMap,
                PlanRoute,
                PutRobotMapBundle,
            )
            from std_msgs.msg import String
        except Exception as exc:
            self._error = f"ROS2 Python imports failed: {exc}"
            return

        try:
            context = Context()
            init_kwargs: dict[str, Any] = {"args": None, "context": context}
            if self.domain_id is not None:
                init_kwargs["domain_id"] = int(self.domain_id)
            rclpy.init(**init_kwargs)
            node_name = f"robot_api_ros_runtime_{_clean_node_suffix(self.robot_id)}"
            node = Node(node_name, context=context)
            self._cmd_vel_pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)
            self._go_to_lm_pub = node.create_publisher(String, self.go_to_lm_topic, 10)
            self._plan_route_client = node.create_client(PlanRoute, self.plan_service_name)
            self._execute_route_client = node.create_client(ExecuteRoute, self.execute_service_name)
            self._cancel_route_client = node.create_client(CancelRoute, self.cancel_service_name)
            self._map_state_client = node.create_client(GetRobotMapState, self.map_state_service_name)
            self._map_load_client = node.create_client(LoadRobotMap, self.map_load_service_name)
            self._map_list_client = node.create_client(ListRobotMaps, self.map_list_service_name)
            self._map_get_bundle_client = node.create_client(GetRobotMapBundle, self.map_get_bundle_service_name)
            self._map_put_bundle_client = node.create_client(PutRobotMapBundle, self.map_put_bundle_service_name)
            node.create_subscription(RobotStatus, self.status_topic, self._on_status, 10)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            thread = threading.Thread(
                target=self._spin_executor,
                args=(executor,),
                name=f"robot-api-ros-runtime-{_clean_node_suffix(self.robot_id)}",
                daemon=True,
            )
            thread.start()
        except Exception as exc:
            self._error = f"ROS2 runtime failed: {exc}"
            return

        self._rclpy = rclpy
        self._context = context
        self._twist_type = Twist
        self._string_type = String
        self._node = node
        self._executor = executor
        self._thread = thread
        self._available = True

    def wait_for_status(self, timeout_sec: float = 0.8) -> bool:
        deadline = monotonic() + max(0.0, float(timeout_sec))
        while monotonic() < deadline:
            if self._latest_message() is not None:
                return True
            sleep(0.04)
        return self._latest_message() is not None

    def _spin_executor(self, executor: Any) -> None:
        try:
            executor.spin()
        except Exception as exc:
            if exc.__class__.__name__ in {"ExternalShutdownException", "ShutdownException"}:
                return
            self._error = f"ROS2 runtime spin failed: {exc}"
            self._available = False

    def _on_status(self, message: Any) -> None:
        with self._lock:
            previous = self._latest_status
            self._latest_status = message
            self._latest_status_at = monotonic()
        self._persist_event(previous, message)

    def _latest_message(self) -> Any | None:
        with self._lock:
            return self._latest_status

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
            return {
                "robotId": self.robot_name,
                "mapId": "",
                "connected": False,
                "localizationOk": False,
                "localizationAgeSec": 9999.0,
                "state": "DISCONNECTED",
                "message": self._error or f"Waiting for {self.status_topic}.",
                "targetLm": "",
                "nearestLm": "",
                "currentEdgeId": "",
                "routeId": "",
                "routeProgress": 0.0,
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
        return {
            "robotId": str(getattr(message, "robot_id", "") or self.robot_name),
            "mapId": str(getattr(message, "map_id", "") or ""),
            "connected": connected,
            "localizationOk": localization_ok,
            "localizationAgeSec": float(getattr(message, "localization_age_sec", 9999.0)),
            "state": str(getattr(message, "state", "") or "UNKNOWN"),
            "message": str(getattr(message, "message", "") or ""),
            "targetLm": str(getattr(message, "target_lm", "") or ""),
            "nearestLm": str(getattr(message, "nearest_lm", "") or ""),
            "currentEdgeId": str(getattr(message, "current_edge_id", "") or ""),
            "routeId": str(getattr(message, "route_id", "") or ""),
            "routeProgress": float(getattr(message, "route_progress", 0.0)),
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

    def _route_payload(self, robot: dict[str, Any]) -> dict[str, Any] | None:
        target = str(robot.get("targetLm") or "").strip()
        route_id = str(robot.get("routeId") or "").strip()
        if not target and not route_id:
            return None
        return {
            "routeId": route_id or f"ros2-{target}",
            "goalLm": target,
            "nodes": [target] if target else [],
            "trajectory": [],
        }

    def _persist_event(self, previous: Any | None, current: Any) -> None:
        state = str(getattr(current, "state", "") or "UNKNOWN")
        message = str(getattr(current, "message", "") or "")
        previous_state = str(getattr(previous, "state", "") or "") if previous is not None else ""
        previous_message = str(getattr(previous, "message", "") or "") if previous is not None else ""
        if state == previous_state and message == previous_message:
            return
        level = "error" if state == "ERROR" else ("warn" if state in {"DISCONNECTED", "LOCALIZING"} else "info")
        self._events.append({"stamp": monotonic(), "level": level, "message": message or state})
        self._events = self._events[-120:]
