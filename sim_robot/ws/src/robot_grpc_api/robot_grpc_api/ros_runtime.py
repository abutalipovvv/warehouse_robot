from __future__ import annotations

import json
import math
import os
import shutil
import signal
import subprocess
import tempfile
import re
import threading
import time
from pathlib import Path
from time import monotonic, sleep
from typing import Any

import yaml
from robot_planner.route_core import WarehouseMapLoader, WorldPoint

NAV2_RUNTIME_PARAMETERS: tuple[tuple[str, str, str], ...] = (
    ("nav2.amcl.update_min_d", "/amcl/set_parameters", "update_min_d"),
    ("nav2.amcl.update_min_a", "/amcl/set_parameters", "update_min_a"),
    ("nav2.amcl.transform_tolerance", "/amcl/set_parameters", "transform_tolerance"),
    ("nav2.amcl.min_particles", "/amcl/set_parameters", "min_particles"),
    ("nav2.amcl.max_particles", "/amcl/set_parameters", "max_particles"),
    ("nav2.controller_server.controller_frequency", "/controller_server/set_parameters", "controller_frequency"),
    ("nav2.controller_server.follow_path.vx_max", "/controller_server/set_parameters", "FollowPath.vx_max"),
    ("nav2.controller_server.follow_path.vx_min", "/controller_server/set_parameters", "FollowPath.vx_min"),
    ("nav2.controller_server.follow_path.wz_max", "/controller_server/set_parameters", "FollowPath.wz_max"),
    ("nav2.local_costmap.robot_radius", "/local_costmap/local_costmap/set_parameters", "robot_radius"),
    (
        "nav2.local_costmap.inflation_radius",
        "/local_costmap/local_costmap/set_parameters",
        "inflation_layer.inflation_radius",
    ),
    (
        "nav2.local_costmap.cost_scaling_factor",
        "/local_costmap/local_costmap/set_parameters",
        "inflation_layer.cost_scaling_factor",
    ),
    ("nav2.global_costmap.robot_radius", "/global_costmap/global_costmap/set_parameters", "robot_radius"),
    (
        "nav2.global_costmap.inflation_radius",
        "/global_costmap/global_costmap/set_parameters",
        "inflation_layer.inflation_radius",
    ),
)

STATUS_STALE_TIMEOUT_SEC = 3.0

NAV2_LIFECYCLE_MANAGER_SERVICES: tuple[tuple[str, str], ...] = (
    ("/lifecycle_manager_navigation/manage_nodes", "navigation"),
    ("/lifecycle_manager_localization/manage_nodes", "localization"),
    ("/lifecycle_manager_map_server/manage_nodes", "map_server"),
)

NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES: tuple[tuple[str, str], ...] = (
    ("/lifecycle_manager_map_server/manage_nodes", "map_server"),
    ("/lifecycle_manager_localization/manage_nodes", "localization"),
    ("/lifecycle_manager_navigation/manage_nodes", "navigation"),
)

NAV2_LIFECYCLE_NODES: tuple[str, ...] = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "bt_navigator",
)

NAV2_LIFECYCLE_RESUME_NODES: tuple[str, ...] = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "bt_navigator",
)


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
        odom_topic: str = "/odom",
        initial_pose_topic: str = "/initialpose",
        scan_topic: str = "/scan",
        map_frame: str = "map",
        base_frame: str = "base_link",
        go_to_lm_topic: str = "/go_to_lm",
        plan_service_name: str = "/route/plan",
        execute_service_name: str = "/route/execute",
        cancel_service_name: str = "/route/cancel",
        route_pause_service_name: str = "/route/pause",
        route_load_map_service_name: str = "/route/load_map",
        status_load_map_service_name: str = "/status/load_map",
        map_state_service_name: str = "/robot/maps/state",
        map_load_service_name: str = "/robot/maps/load",
        map_list_service_name: str = "/robot/maps/list",
        map_get_bundle_service_name: str = "/robot/maps/get_bundle",
        map_put_bundle_service_name: str = "/robot/maps/put_bundle",
        map_topic: str = "/map",
        slam_save_map_service_name: str = "/slam_toolbox/save_map",
        reset_odom_service_name: str = "/reset_odom",
        slam_params_file: str | None = None,
        slam_launch_file: str | None = None,
        params_path: str | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_name
        self.host = host
        self.domain_id = domain_id
        self.namespace = namespace.strip().strip("/")
        self.status_topic = self._topic(status_topic)
        self.cmd_vel_topic = self._topic(cmd_vel_topic)
        self.odom_topic = self._topic(odom_topic)
        self.initial_pose_topic = self._topic(initial_pose_topic)
        self.scan_topic = self._topic(scan_topic)
        self.map_frame = str(map_frame or "map").strip() or "map"
        self.base_frame = str(base_frame or "base_link").strip() or "base_link"
        self.go_to_lm_topic = self._topic(go_to_lm_topic)
        self.plan_service_name = self._topic(plan_service_name)
        self.execute_service_name = self._topic(execute_service_name)
        self.cancel_service_name = self._topic(cancel_service_name)
        self.route_pause_service_name = self._topic(route_pause_service_name)
        self.route_load_map_service_name = self._topic(route_load_map_service_name)
        self.status_load_map_service_name = self._topic(status_load_map_service_name)
        self.map_state_service_name = self._topic(map_state_service_name)
        self.map_load_service_name = self._topic(map_load_service_name)
        self.map_list_service_name = self._topic(map_list_service_name)
        self.map_get_bundle_service_name = self._topic(map_get_bundle_service_name)
        self.map_put_bundle_service_name = self._topic(map_put_bundle_service_name)
        self.map_topic = self._topic(map_topic)
        self.slam_save_map_service_name = self._topic(slam_save_map_service_name)
        self.reset_odom_service_name = self._topic(reset_odom_service_name) if str(reset_odom_service_name or "").strip() else ""
        self.slam_params_file = Path(slam_params_file).expanduser().resolve() if slam_params_file else self._default_slam_params_file()
        self.slam_launch_file = Path(slam_launch_file).expanduser().resolve() if slam_launch_file else self._default_slam_launch_file()
        self.params_path = Path(params_path).expanduser().resolve() if params_path else None
        self._lock = threading.Lock()
        self._latest_status: Any | None = None
        self._latest_status_at: float | None = None
        self._latest_scan: Any | None = None
        self._latest_scan_at: float | None = None
        self._latest_odom_pose: dict[str, float] | None = None
        self._latest_odom_at: float | None = None
        self._latest_slam_pose: dict[str, float] | None = None
        self._latest_slam_pose_at: float | None = None
        self._latest_map: Any | None = None
        self._latest_map_at: float | None = None
        self._control_owner_id = ""
        self._control_owner_name = ""
        self._control_acquired_at: float | None = None
        self._control_lease_ms = 0
        self._navigation_paused = False
        self._localization_confirmed = False
        self._relocation_requested_at: float | None = None
        self._events: list[dict[str, Any]] = []
        self._available = False
        self._error = ""
        self._node = None
        self._rclpy = None
        self._context = None
        self._time_type = None
        self._tf_buffer = None
        self._tf_listener = None
        self._twist_type = None
        self._odom_type = None
        self._pose_with_covariance_type = None
        self._laser_scan_type = None
        self._string_type = None
        self._set_parameters_type = None
        self._list_parameters_type = None
        self._manage_lifecycle_nodes_type = None
        self._change_state_type = None
        self._transition_type = None
        self._parameter_type = None
        self._parameter_value_type = None
        self._parameter_type_enum = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._cmd_vel_pub = None
        self._initial_pose_pub = None
        self._go_to_lm_pub = None
        self._plan_route_client = None
        self._execute_route_client = None
        self._cancel_route_client = None
        self._route_pause_client = None
        self._route_load_map_client = None
        self._status_load_map_client = None
        self._map_state_client = None
        self._map_load_client = None
        self._map_list_client = None
        self._map_get_bundle_client = None
        self._map_put_bundle_client = None
        self._slam_save_map_client = None
        self._reset_odom_client = None
        self._nav2_lifecycle_clients: dict[str, Any] = {}
        self._nav2_change_state_clients: dict[str, Any] = {}
        self._save_map_type = None
        self._std_empty_type = None
        self._occupancy_grid_type = None
        self._slam_process: subprocess.Popen | None = None
        self._slam_temp_dir: Path | None = None
        self._nav2_paused_for_slam = False
        self._slam_ignore_maps_until = 0.0
        self._slam_state: dict[str, Any] = {
            "active": False,
            "state": "idle",
            "message": "",
            "sessionId": "",
            "progress": 0,
        }
        self._slam_trail: list[dict[str, float]] = []
        self._nav2_param_clients: dict[str, Any] = {}
        self._nav2_list_param_clients: dict[str, Any] = {}
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

    def _default_slam_params_file(self) -> Path:
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "slam_toolbox" / "config" / "mapper_params_online_async.yaml"
            if candidate.is_file():
                return candidate.resolve()
            candidate = parent / "src" / "slam_toolbox" / "config" / "mapper_params_online_async.yaml"
            if candidate.is_file():
                return candidate.resolve()
        return Path("sim_robot/ws/src/slam_toolbox/config/mapper_params_online_async.yaml").resolve()

    def _default_slam_launch_file(self) -> Path:
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "slam_toolbox" / "launch" / "online_async_launch.py"
            if candidate.is_file():
                return candidate.resolve()
            candidate = parent / "src" / "slam_toolbox" / "launch" / "online_async_launch.py"
            if candidate.is_file():
                return candidate.resolve()
        return Path("sim_robot/ws/src/slam_toolbox/launch/online_async_launch.py").resolve()

    def close(self) -> None:
        self._stop_slam_process()
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
            "odomTopic": self.odom_topic,
            "initialPoseTopic": self.initial_pose_topic,
            "scanTopic": self.scan_topic,
            "mapFrame": self.map_frame,
            "baseFrame": self.base_frame,
            "goToLmTopic": self.go_to_lm_topic,
            "planService": self.plan_service_name,
            "executeService": self.execute_service_name,
            "cancelService": self.cancel_service_name,
            "routePauseService": self.route_pause_service_name,
            "routeLoadMapService": self.route_load_map_service_name,
            "statusLoadMapService": self.status_load_map_service_name,
            "mapStateService": self.map_state_service_name,
            "mapLoadService": self.map_load_service_name,
            "mapListService": self.map_list_service_name,
            "mapGetBundleService": self.map_get_bundle_service_name,
            "mapPutBundleService": self.map_put_bundle_service_name,
            "mapTopic": self.map_topic,
            "slamSaveMapService": self.slam_save_map_service_name,
            "resetOdomService": self.reset_odom_service_name,
            "slamParamsFile": str(self.slam_params_file),
            "slamLaunchFile": str(self.slam_launch_file),
            "paramsPath": str(self.params_path or ""),
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
        robot.update(self._control_payload(robot))
        return {
            "ok": True,
            "robot": robot,
            "events": list(self._events[-120:]),
            "route": self._route_payload(robot),
        }

    def status_robot_payload(self) -> dict[str, Any]:
        return self.status_payload()["robot"]

    def teleop(self, *, linear: float, angular: float, timeout_ms: int = 350, owner_id: str = "") -> dict[str, Any]:
        del timeout_ms
        self._ensure_manual_control_allowed("manual control")
        self._ensure_control_owner(owner_id, action="manual control")
        self._publish_twist(linear, angular)
        return {"ok": True, "linear": float(linear), "angular": float(angular)}

    def teleop_stop(self) -> dict[str, Any]:
        self._publish_twist(0.0, 0.0)
        return {"ok": True}

    def laser_scan_payload(self, *, topic: str = "/scan", include_intensities: bool = False) -> dict[str, Any]:
        requested_topic = self._topic(topic or self.scan_topic)
        if requested_topic != self.scan_topic:
            raise ValueError(f"scan topic {requested_topic} is not configured; use {self.scan_topic}")
        with self._lock:
            message = self._latest_scan
            received_at = self._latest_scan_at
        if message is None:
            raise ValueError(f"Waiting for LaserScan on {self.scan_topic}.")

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        stamp_sec = float(getattr(stamp, "sec", 0.0) or 0.0) + float(getattr(stamp, "nanosec", 0.0) or 0.0) / 1e9
        ranges = [float(item) for item in list(getattr(message, "ranges", []) or [])]
        intensities = []
        if include_intensities:
            intensities = [float(item) for item in list(getattr(message, "intensities", []) or [])]
        return {
            "ok": True,
            "robotId": self.robot_id,
            "topic": self.scan_topic,
            "frameId": str(getattr(header, "frame_id", "") or ""),
            "stampSec": stamp_sec,
            "receivedAgeSec": 9999.0 if received_at is None else max(0.0, monotonic() - received_at),
            "angleMin": float(getattr(message, "angle_min", 0.0)),
            "angleMax": float(getattr(message, "angle_max", 0.0)),
            "angleIncrement": float(getattr(message, "angle_increment", 0.0)),
            "timeIncrement": float(getattr(message, "time_increment", 0.0)),
            "scanTime": float(getattr(message, "scan_time", 0.0)),
            "rangeMin": float(getattr(message, "range_min", 0.0)),
            "rangeMax": float(getattr(message, "range_max", 0.0)),
            "ranges": ranges,
            "intensities": intensities,
        }

    def stop(self, *, owner_id: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="stop")
        self._publish_twist(0.0, 0.0)
        try:
            self.cancel_route(owner_id=owner_id)
        except Exception:
            self._publish_go_to_lm("cancel")
        return {"ok": True}

    def cancel_route(self, *, owner_id: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="cancel route")
        if self._cancel_route_client is not None and self._service_available(self._cancel_route_client, 0.05):
            request = self._cancel_route_client.srv_type.Request()
            request.message = "Route canceled."
            response = self._call_service(self._cancel_route_client, request, "route cancel")
            if not bool(response.ok):
                raise ValueError(str(response.error or "route cancel failed"))
        else:
            self._publish_go_to_lm("cancel")
        self._navigation_paused = False
        return {"ok": True}

    def execute_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._ensure_navigation_mode("execute route")
        self._ensure_control_owner(str(payload.get("ownerId") or payload.get("owner_id") or ""), action="execute route")
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
            self._navigation_paused = False
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
        self._navigation_paused = False
        route = {
            "routeId": f"ros2-{goal_lm}",
            "goalLm": goal_lm,
            "startLm": source_lm,
            "nodes": [item for item in [source_lm, goal_lm] if item],
            "trajectory": [],
        }
        return {"ok": True, "route": route}

    def acquire_control(
        self,
        *,
        owner_id: str,
        owner_name: str = "",
        force: bool = False,
        lease_ms: int = 0,
    ) -> dict[str, Any]:
        owner = self._clean_owner_id(owner_id)
        if not owner:
            raise ValueError("owner_id is required")
        with self._lock:
            self._expire_control_owner_locked()
            if self._control_owner_id and self._control_owner_id != owner and not force:
                raise ValueError(f"control is owned by {self._control_owner_name or self._control_owner_id}")
            self._control_owner_id = owner
            self._control_owner_name = str(owner_name or owner)
            self._control_acquired_at = monotonic()
            self._control_lease_ms = max(0, int(lease_ms or 0))
        return {"ok": True, "control": self._control_state_payload()}

    def release_control(self, *, owner_id: str, force: bool = False) -> dict[str, Any]:
        owner = self._clean_owner_id(owner_id)
        with self._lock:
            self._expire_control_owner_locked()
            if self._control_owner_id and self._control_owner_id != owner and not force:
                raise ValueError(f"control is owned by {self._control_owner_name or self._control_owner_id}")
            self._control_owner_id = ""
            self._control_owner_name = ""
            self._control_acquired_at = None
            self._control_lease_ms = 0
        return {"ok": True, "control": self._control_state_payload()}

    def relocate(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        owner_id: str = "",
        frame_id: str = "map",
        covariance_json: str = "",
        confirm: bool = False,
    ) -> dict[str, Any]:
        self._ensure_navigation_mode("relocate")
        self._ensure_control_owner(owner_id, action="relocate")
        if self._initial_pose_pub is None or self._pose_with_covariance_type is None:
            raise ValueError(self._error or "initial pose publisher is not available")
        ros_x, ros_y, ros_yaw = self._map_pose_to_ros_pose(float(x), float(y), float(yaw))
        message = self._pose_with_covariance_type()
        message.header.frame_id = str(frame_id or "map")
        if self._node is not None:
            message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = ros_x
        message.pose.pose.position.y = ros_y
        message.pose.pose.position.z = 0.0
        qz = math.sin(ros_yaw * 0.5)
        qw = math.cos(ros_yaw * 0.5)
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        covariance = self._covariance_from_json(covariance_json)
        if covariance:
            message.pose.covariance = covariance
        else:
            message.pose.covariance[0] = 0.25
            message.pose.covariance[7] = 0.25
            message.pose.covariance[35] = 0.06853891945200942
        self._initial_pose_pub.publish(message)
        self._publish_twist(0.0, 0.0)
        with self._lock:
            self._localization_confirmed = bool(confirm)
            self._relocation_requested_at = monotonic()
        return {"ok": True, "relocate": {"x": float(x), "y": float(y), "yaw": float(yaw), "frameId": message.header.frame_id}}

    def _map_pose_to_ros_pose(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        active = self.active_map_payload()
        map_dir = Path(str(active.get("mapDir") or "")).resolve()
        if not map_dir.is_dir():
            raise ValueError(f"active map directory is not available: {map_dir}")
        loaded_map = WarehouseMapLoader(map_dir).load()
        point = loaded_map.map_metadata.map_to_ros_point(WorldPoint(x=float(x), y=float(y)))
        return (
            float(point.x),
            float(point.y),
            float(loaded_map.map_metadata.map_yaw_to_ros(float(yaw))),
        )

    def confirm_localization(self, *, accepted: bool = True, message: str = "", owner_id: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="confirm localization")
        with self._lock:
            self._localization_confirmed = bool(accepted)
            if accepted:
                self._relocation_requested_at = None
        return {
            "ok": True,
            "localizationConfirmed": bool(accepted),
            "message": message or ("Localization confirmed." if accepted else "Localization rejected."),
        }

    def pause_route(self, *, owner_id: str = "", message: str = "") -> dict[str, Any]:
        self._ensure_control_owner(owner_id, action="pause route")
        self._set_route_paused(True, message or "Route paused by operator.")
        return {"ok": True, "navigationPaused": True}

    def resume_route(self, *, owner_id: str = "", message: str = "") -> dict[str, Any]:
        self._ensure_navigation_mode("resume route")
        self._ensure_control_owner(owner_id, action="resume route")
        self._set_route_paused(False, message or "Route resumed by operator.")
        return {"ok": True, "navigationPaused": False}

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
        if activate:
            self._ensure_navigation_mode("push active map")
            self._stop_navigation_before_map_change()
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

    def load_map(self, map_name: str, *, allow_during_transition: bool = False) -> dict[str, Any]:
        if not allow_during_transition:
            self._ensure_navigation_mode("load map")
        self._stop_navigation_before_map_change()
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

    def _ensure_navigation_mode(self, action: str) -> None:
        with self._lock:
            active = bool(self._slam_state.get("active"))
            state = str(self._slam_state.get("state") or "idle")
        if active or state in {"starting", "mapping", "saving", "canceling", "resuming"}:
            raise ValueError(f"Cannot {action} while robot is in {state} mode")

    def _ensure_manual_control_allowed(self, action: str) -> None:
        with self._lock:
            active = bool(self._slam_state.get("active"))
            state = str(self._slam_state.get("state") or "idle")
        if state in {"starting", "saving", "canceling", "resuming"}:
            raise ValueError(f"Cannot {action} while robot is switching modes ({state})")
        if active and state != "mapping":
            raise ValueError(f"Cannot {action} while robot is in {state} mode")

    def _stop_navigation_before_map_change(self) -> None:
        errors: list[str] = []
        try:
            self._publish_twist(0.0, 0.0)
        except Exception as exc:
            errors.append(f"stop failed: {exc}")
        try:
            if self._cancel_route_client is not None and self._service_available(self._cancel_route_client, 0.1):
                request = self._cancel_route_client.srv_type.Request()
                request.message = "Route canceled before map change."
                response = self._call_service(self._cancel_route_client, request, "route cancel", timeout_sec=1.5)
                if not bool(response.ok):
                    errors.append(str(response.error or "route cancel failed"))
            else:
                self._publish_go_to_lm("cancel")
        except Exception as exc:
            errors.append(f"route cancel failed: {exc}")
        self._navigation_paused = False
        if errors:
            self._append_runtime_event("warn", "map change pre-stop warning: " + "; ".join(errors))

    def slam_defaults_payload(self) -> dict[str, Any]:
        try:
            payload = yaml.safe_load(self.slam_params_file.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"SLAM params file does not exist: {self.slam_params_file}") from exc
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            raise ValueError(f"SLAM params file must contain a YAML object: {self.slam_params_file}")
        return {"ok": True, "params": payload, "paramsPath": str(self.slam_params_file)}

    def start_slam(
        self,
        params_payload: dict[str, Any] | None = None,
        *,
        use_sim_time: bool = True,
        command_id: str = "",
    ) -> dict[str, Any]:
        del command_id
        with self._lock:
            if bool(self._slam_state.get("active")):
                raise ValueError("SLAM is already running")
            state = str(self._slam_state.get("state") or "idle")
            if state in {"starting", "saving", "canceling", "resuming"}:
                raise ValueError(f"SLAM mode switch is already in progress: {state}")
            session_id = f"slam-{self.robot_id}-{int(time.time())}"
            self._latest_map = None
            self._latest_map_at = None
            self._slam_trail = []
            self._slam_ignore_maps_until = monotonic() + 1.0
            self._slam_state = {
                "active": True,
                "state": "starting",
                "message": "Switching from navigation to 2D SLAM.",
                "sessionId": session_id,
                "startedAtSec": time.time(),
                "progress": 0,
                "savedMapName": "",
                "mapDir": "",
                "nav2Paused": False,
            }

        if not self.slam_launch_file.is_file():
            with self._lock:
                self._slam_state.update({"active": False, "state": "error", "message": "SLAM launch file is missing."})
            raise ValueError(f"SLAM launch file does not exist: {self.slam_launch_file}")

        try:
            default_params = self.slam_defaults_payload()["params"]
            params = params_payload if isinstance(params_payload, dict) and params_payload else default_params
            params = self._coerce_slam_params(params, default_params)
            params = self._normalize_slam_params(params, default_params)
        except Exception as exc:
            with self._lock:
                self._slam_state.update({"active": False, "state": "error", "message": f"Invalid SLAM parameters: {exc}"})
            raise
        try:
            nav2_control = self._pause_nav2_for_slam()
        except Exception as exc:
            with self._lock:
                self._slam_state.update({"active": False, "state": "error", "message": f"Failed to pause Nav2: {exc}"})
            raise
        with self._lock:
            self._nav2_paused_for_slam = bool(nav2_control.get("changed"))
            self._slam_state["nav2Paused"] = bool(nav2_control.get("changed"))
        try:
            with self._lock:
                self._slam_state["message"] = "Resetting odometry before 2D SLAM."
            self._reset_odom_for_slam()
        except Exception as exc:
            if bool(nav2_control.get("changed")):
                self._resume_nav2_after_slam()
            with self._lock:
                self._slam_state.update({"active": False, "state": "error", "message": f"Failed to reset odom: {exc}"})
            raise
        temp_dir = Path(tempfile.mkdtemp(prefix=f"{_clean_node_suffix(self.robot_id)}-slam-"))
        params_file = temp_dir / "mapper_params_online_async.yaml"
        params_file.write_text(yaml.safe_dump(params, sort_keys=False, allow_unicode=True), encoding="utf-8")

        cmd = [
            "ros2",
            "launch",
            str(self.slam_launch_file),
            f"slam_params_file:={params_file}",
            f"use_sim_time:={'true' if use_sim_time else 'false'}",
            f"reset_odom_service:={self.reset_odom_service_name}",
        ]
        try:
            print(f"[robot_api_server] Starting 2D SLAM launch: {' '.join(cmd)}", flush=True)
            process = subprocess.Popen(cmd, start_new_session=True)
            with self._lock:
                self._slam_process = process
                self._slam_temp_dir = temp_dir
            sleep(0.5)
            if process.poll() is not None:
                raise ValueError(f"SLAM launch exited immediately with code {process.returncode}")
        except FileNotFoundError as exc:
            shutil.rmtree(temp_dir, ignore_errors=True)
            if bool(nav2_control.get("changed")):
                self._resume_nav2_after_slam()
            with self._lock:
                self._slam_state.update({"active": False, "state": "error", "message": "ros2 command is not available."})
            raise ValueError("ros2 command is not available; source the ROS environment before starting SLAM") from exc
        except Exception as exc:
            self._stop_slam_process()
            if bool(nav2_control.get("changed")):
                self._resume_nav2_after_slam()
            with self._lock:
                self._slam_state.update({"active": False, "state": "error", "message": f"Failed to start 2D SLAM: {exc}"})
            raise

        with self._lock:
            self._slam_process = process
            self._slam_temp_dir = temp_dir
            self._nav2_paused_for_slam = bool(nav2_control.get("changed"))
            self._slam_state = {
                "active": True,
                "state": "mapping",
                "message": "2D SLAM is running. Nav2 is paused and manual WASD teleop is available.",
                "sessionId": session_id,
                "startedAtSec": time.time(),
                "progress": 0,
                "savedMapName": "",
                "mapDir": "",
                "nav2Paused": bool(nav2_control.get("changed")),
            }
            state = dict(self._slam_state)
        return {"ok": True, "state": state}

    def slam_state_payload(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._slam_state)
            message = self._latest_map
            trail_points = len(self._slam_trail)
        if message is not None:
            info = getattr(message, "info", None)
            header = getattr(message, "header", None)
            state.update(
                {
                    "mapWidth": int(getattr(info, "width", 0) or 0),
                    "mapHeight": int(getattr(info, "height", 0) or 0),
                    "resolution": float(getattr(info, "resolution", 0.0) or 0.0),
                    "frameId": str(getattr(header, "frame_id", "") or ""),
                }
            )
        state["trailPoints"] = trail_points
        return {"ok": True, "state": state}

    def slam_map_frame_payload(self, *, include_cells: bool = True) -> dict[str, Any]:
        with self._lock:
            message = self._latest_map
            state = dict(self._slam_state)
            trail = list(self._slam_trail)
            active = bool(state.get("active"))
        if message is None:
            state.setdefault("message", f"Waiting for OccupancyGrid on {self.map_topic}.")
            return {
                "ok": True,
                "robotId": self.robot_id,
                "sessionId": str(state.get("sessionId") or ""),
                "state": state,
                "trail": trail,
                "pose": self._current_slam_pose_payload(),
            }

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        stamp_sec = float(getattr(stamp, "sec", 0.0) or 0.0) + float(getattr(stamp, "nanosec", 0.0) or 0.0) / 1e9
        info = getattr(message, "info", None)
        origin = getattr(info, "origin", None)
        origin_position = getattr(origin, "position", None)
        origin_orientation = getattr(origin, "orientation", None)
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        state.update(
            {
                "active": active,
                "mapWidth": width,
                "mapHeight": height,
                "resolution": float(getattr(info, "resolution", 0.0) or 0.0),
                "frameId": str(getattr(header, "frame_id", "") or ""),
                "trailPoints": len(trail),
            }
        )
        cells = b""
        if include_cells:
            cells = self._slam_cells_to_bytes(getattr(message, "data", []) or [])
        return {
            "ok": True,
            "robotId": self.robot_id,
            "sessionId": str(state.get("sessionId") or ""),
            "frameId": str(getattr(header, "frame_id", "") or ""),
            "stampSec": stamp_sec,
            "width": width,
            "height": height,
            "resolution": float(getattr(info, "resolution", 0.0) or 0.0),
            "originX": float(getattr(origin_position, "x", 0.0) or 0.0),
            "originY": float(getattr(origin_position, "y", 0.0) or 0.0),
            "originYaw": self._yaw_from_quaternion(origin_orientation),
            "cells": cells,
            "pose": self._current_slam_pose_payload(),
            "trail": trail,
            "state": state,
        }

    def finish_slam(self, *, map_name: str, activate: bool = True, command_id: str = "") -> dict[str, Any]:
        del command_id
        safe_name = self._safe_map_name(map_name)
        if not safe_name:
            raise ValueError("map_name is required")
        with self._lock:
            if not bool(self._slam_state.get("active")) and self._latest_map is None:
                raise ValueError("SLAM is not running and no live map is available")
            self._slam_state.update({"state": "saving", "message": "Preparing SLAM map save.", "progress": 5})

        maps_root = self._slam_maps_root()
        target = (maps_root / f"{safe_name}.smap").resolve()
        if maps_root not in target.parents:
            raise ValueError("map must stay inside maps_root")
        if target.exists():
            raise ValueError(f"map already exists: {target.name}")

        self._set_slam_progress(20, "Saving occupancy grid as PGM/YAML.")
        target.mkdir(parents=True, exist_ok=False)
        try:
            self._save_slam_map_files(target, safe_name)
            self._set_slam_progress(55, "Creating editable smap files.")
            self._write_empty_smap_sidecars(target, safe_name)
            self._stop_slam_process()
            with self._lock:
                self._slam_state.update({"state": "resuming", "message": "Restoring Nav2 after SLAM.", "progress": 64})
            self._resume_nav2_after_slam()
            loaded = {"ok": True, "mapName": safe_name, "mapDir": str(target), "mapId": safe_name}
            if activate:
                self._set_slam_progress(72, "Loading new map on robot.")
                loaded = self.load_map(safe_name, allow_during_transition=True)
            self._set_slam_progress(86, "Building map bundle for operator pull.")
            bundle = self.pull_map_bundle_payload(safe_name)
            with self._lock:
                self._slam_state.update(
                    {
                        "active": False,
                        "state": "done",
                        "message": f"SLAM map saved: {safe_name}.",
                        "progress": 100,
                        "savedMapName": safe_name,
                        "mapDir": str(target),
                    }
                )
                state = dict(self._slam_state)
            return {
                "ok": True,
                "state": state,
                "mapName": safe_name,
                "mapDir": str(target),
                "mapId": str(loaded.get("mapId") or safe_name),
                "signature": str(bundle.get("signature") or ""),
                "bundleJson": json.dumps(bundle, ensure_ascii=False),
            }
        except Exception:
            shutil.rmtree(target, ignore_errors=True)
            with self._lock:
                running = self._slam_process is not None and self._slam_process.poll() is None
                self._slam_state.update(
                    {
                        "active": running,
                        "state": "error",
                        "progress": max(1, int(self._slam_state.get("progress") or 1)),
                    }
                )
            raise

    def cancel_slam(self, *, reason: str = "", command_id: str = "") -> dict[str, Any]:
        del command_id
        with self._lock:
            self._slam_state.update({"state": "canceling", "message": reason or "Canceling SLAM.", "progress": 0})
        self._stop_slam_process()
        with self._lock:
            self._slam_state.update({"state": "resuming", "message": "Restoring Nav2 after SLAM cancel."})
        self._resume_nav2_after_slam()
        with self._lock:
            self._slam_state.update(
                {
                    "active": False,
                    "state": "canceled",
                    "message": reason or "SLAM canceled.",
                    "progress": 0,
                }
            )
            state = dict(self._slam_state)
        return {"ok": True, "state": state}

    def params_payload(self) -> dict[str, Any]:
        if self.params_path is None:
            raise ValueError("params path is not configured")
        try:
            payload = yaml.safe_load(self.params_path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"params file does not exist: {self.params_path}") from exc
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            raise ValueError(f"params file must contain a YAML object: {self.params_path}")
        return {"ok": True, "params": payload, "path": str(self.params_path)}

    def save_params_payload(self, params_payload: dict[str, Any], *, reload_runtime: bool = True) -> dict[str, Any]:
        if self.params_path is None:
            raise ValueError("params path is not configured")
        if not isinstance(params_payload, dict):
            raise ValueError("params payload must be an object")
        previous_payload: dict[str, Any] | None = None
        try:
            loaded = yaml.safe_load(self.params_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                previous_payload = loaded
        except Exception:
            previous_payload = None
        self.params_path.parent.mkdir(parents=True, exist_ok=True)
        self.params_path.write_text(
            yaml.safe_dump(params_payload, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        warnings = []
        reloaded = False
        if reload_runtime:
            try:
                reloaded = self._apply_nav2_runtime_params(params_payload, previous_payload=previous_payload) or reloaded
            except Exception as exc:
                warnings.append(f"Nav2 runtime apply failed: {exc}")
            try:
                reloaded = self._reload_route_status_params() or reloaded
            except Exception as exc:
                warnings.append(f"route/status reload failed: {exc}")
        return {
            "ok": True,
            "params": params_payload,
            "path": str(self.params_path),
            "reloaded": reloaded,
            "warning": "; ".join(warnings),
        }

    def _apply_saved_nav2_params_when_ready(self) -> None:
        if self.params_path is None:
            return
        for _attempt in range(20):
            sleep(1.0)
            try:
                payload = yaml.safe_load(self.params_path.read_text(encoding="utf-8"))
            except Exception:
                return
            if not isinstance(payload, dict) or "nav2" not in payload:
                return
            try:
                if self._apply_nav2_runtime_params(payload, require_available=False):
                    return
            except Exception:
                continue

    def _apply_nav2_runtime_params(
        self,
        params_payload: dict[str, Any],
        *,
        require_available: bool = True,
        previous_payload: dict[str, Any] | None = None,
    ) -> bool:
        if not isinstance(params_payload.get("nav2"), dict):
            return False
        if self._node is None or self._set_parameters_type is None:
            if require_available:
                raise ValueError("ROS2 parameter runtime is not available")
            return False

        grouped: dict[str, dict[str, Any]] = {}
        for source_path, service_name, param_name in NAV2_RUNTIME_PARAMETERS:
            value = self._deep_get(params_payload, source_path)
            if value is None:
                continue
            if previous_payload is not None and self._values_equal(value, self._deep_get(previous_payload, source_path)):
                continue
            grouped.setdefault(service_name, {})[param_name] = value

        velocity = params_payload.get("nav2", {}).get("velocity_smoother")
        if isinstance(velocity, dict):
            max_x = velocity.get("max_velocity_x")
            max_theta = velocity.get("max_velocity_theta")
            previous_velocity = previous_payload.get("nav2", {}).get("velocity_smoother") if isinstance(previous_payload, dict) else None
            velocity_changed = max_x is not None or max_theta is not None
            if isinstance(previous_velocity, dict):
                velocity_changed = (
                    not self._values_equal(max_x, previous_velocity.get("max_velocity_x"))
                    or not self._values_equal(max_theta, previous_velocity.get("max_velocity_theta"))
                )
            if velocity_changed:
                previous_max_x = previous_velocity.get("max_velocity_x") if isinstance(previous_velocity, dict) else None
                previous_max_theta = previous_velocity.get("max_velocity_theta") if isinstance(previous_velocity, dict) else None
                grouped.setdefault("/velocity_smoother/set_parameters", {})["max_velocity"] = [
                    float(max_x if max_x is not None else (previous_max_x if previous_max_x is not None else 0.5)),
                    0.0,
                    float(max_theta if max_theta is not None else (previous_max_theta if previous_max_theta is not None else 2.0)),
                ]

        applied = False
        missing: list[str] = []
        for service_name, values in grouped.items():
            client = self._nav2_param_clients.get(service_name)
            if client is None:
                client = self._node.create_client(self._set_parameters_type, service_name)
                self._nav2_param_clients[service_name] = client
            if not self._service_available(client, 0.25):
                missing.append(service_name)
                continue
            values = self._filter_declared_parameters(service_name, values, require_available=require_available)
            if not values:
                continue
            request = self._set_parameters_type.Request()
            request.parameters = [
                self._parameter_message(param_name, value)
                for param_name, value in values.items()
            ]
            response = self._call_service(client, request, f"{service_name} set_parameters", timeout_sec=3.0)
            failed = [
                result.reason or "parameter rejected"
                for result in getattr(response, "results", [])
                if not bool(getattr(result, "successful", False))
            ]
            if failed:
                raise ValueError(f"{service_name}: {'; '.join(failed)}")
            applied = True
        if missing and require_available:
            raise ValueError(f"Nav2 parameter services unavailable: {', '.join(missing)}")
        return applied

    def _filter_declared_parameters(
        self,
        set_service_name: str,
        values: dict[str, Any],
        *,
        require_available: bool,
    ) -> dict[str, Any]:
        if self._node is None or self._list_parameters_type is None:
            return values
        list_service_name = set_service_name.rsplit("/", 1)[0] + "/list_parameters"
        client = self._nav2_list_param_clients.get(list_service_name)
        if client is None:
            client = self._node.create_client(self._list_parameters_type, list_service_name)
            self._nav2_list_param_clients[list_service_name] = client
        if not self._service_available(client, 0.15):
            return values if require_available else {}
        request = self._list_parameters_type.Request()
        request.prefixes = []
        request.depth = 0
        response = self._call_service(client, request, f"{list_service_name} list_parameters", timeout_sec=2.0)
        declared = set(str(name) for name in getattr(getattr(response, "result", None), "names", []))
        if not declared:
            return values
        return {name: value for name, value in values.items() if name in declared}

    def _parameter_message(self, name: str, value: Any) -> Any:
        if self._parameter_type is None or self._parameter_value_type is None or self._parameter_type_enum is None:
            raise ValueError("ROS2 parameter message types are not available")
        parameter = self._parameter_type()
        parameter.name = str(name)
        parameter_value = self._parameter_value_type()
        if isinstance(value, bool):
            parameter_value.type = self._parameter_type_enum.PARAMETER_BOOL
            parameter_value.bool_value = bool(value)
        elif isinstance(value, int) and not isinstance(value, bool):
            parameter_value.type = self._parameter_type_enum.PARAMETER_INTEGER
            parameter_value.integer_value = int(value)
        elif isinstance(value, float):
            parameter_value.type = self._parameter_type_enum.PARAMETER_DOUBLE
            parameter_value.double_value = float(value)
        elif isinstance(value, list):
            parameter_value.type = self._parameter_type_enum.PARAMETER_DOUBLE_ARRAY
            parameter_value.double_array_value = [float(item) for item in value]
        else:
            parameter_value.type = self._parameter_type_enum.PARAMETER_STRING
            parameter_value.string_value = str(value)
        parameter.value = parameter_value
        return parameter

    def _deep_get(self, source: dict[str, Any], path: str) -> Any:
        current: Any = source
        for part in str(path).split("."):
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]
        return current

    def _values_equal(self, left: Any, right: Any) -> bool:
        if isinstance(left, (int, float)) or isinstance(right, (int, float)):
            try:
                return abs(float(left) - float(right)) < 0.000001
            except (TypeError, ValueError):
                return False
        return left == right

    def _reload_route_status_params(self) -> bool:
        active = self.active_map_payload()
        map_dir = str(active.get("mapDir") or "").strip()
        map_name = str(active.get("mapName") or active.get("mapId") or "").strip()
        if not map_dir:
            raise ValueError("active map directory is not available")

        reloaded = False
        for client, label in (
            (self._route_load_map_client, "route/load_map"),
            (self._status_load_map_client, "status/load_map"),
        ):
            if client is None:
                continue
            if not self._service_available(client, 0.2):
                continue
            request = client.srv_type.Request()
            request.map_name = map_name
            request.map_dir = map_dir
            response = self._call_service(client, request, label, timeout_sec=5.0)
            if not bool(response.ok):
                raise ValueError(str(response.error or f"{label} failed"))
            reloaded = True
        if not reloaded:
            raise ValueError("route/status load-map services are not available")
        return reloaded

    def _execute_pose_route(self, pose: dict[str, float], payload: dict[str, Any]) -> dict[str, Any]:
        if self._execute_route_client is not None and self._service_available(self._execute_route_client, 0.05):
            status = self.status_robot_payload()
            start_pose = payload.get("startPose") if isinstance(payload.get("startPose"), dict) else None
            if isinstance(start_pose, dict):
                current = {
                    "x": float(start_pose.get("x", 0.0) or 0.0),
                    "y": float(start_pose.get("y", 0.0) or 0.0),
                    "yaw": float(start_pose.get("yaw", 0.0) or 0.0),
                }
            else:
                current = status.get("pose") if isinstance(status.get("pose"), dict) else None
            if not isinstance(current, dict):
                raise ValueError("robot pose is not available yet")

            route_payload = self._straight_pose_route_payload(current, pose)
            request = self._execute_route_client.srv_type.Request()
            request.route_json = json.dumps(route_payload, ensure_ascii=False)
            response = self._call_service(self._execute_route_client, request, "route execute")
            if not bool(response.ok):
                raise ValueError(str(response.error or "route execute failed"))
            self._navigation_paused = False
            return {"ok": True, "route": route_payload}

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

    def _straight_pose_route_payload(self, start: dict[str, Any], goal: dict[str, float]) -> dict[str, Any]:
        start_x = float(start.get("x", 0.0) or 0.0)
        start_y = float(start.get("y", 0.0) or 0.0)
        start_yaw = float(start.get("yaw", 0.0) or 0.0)
        goal_x = float(goal.get("x", 0.0) or 0.0)
        goal_y = float(goal.get("y", 0.0) or 0.0)
        goal_yaw = float(goal.get("yaw", start_yaw) or 0.0)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        travel_yaw = math.atan2(goal_y - start_y, goal_x - start_x) if distance > 1e-6 else goal_yaw
        sample_distance = 0.05
        steps = max(1, math.ceil(distance / sample_distance))
        trajectory: list[dict[str, Any]] = []
        for step in range(steps + 1):
            ratio = step / steps
            yaw = travel_yaw if step < steps else goal_yaw
            trajectory.append(
                {
                    "x": start_x + ((goal_x - start_x) * ratio),
                    "y": start_y + ((goal_y - start_y) * ratio),
                    "yaw": yaw,
                    "edgeId": "CURRENT_POSE->GOAL_POSE",
                    "motionDirection": "forward",
                }
            )
        label = f"pose({goal_x:.3f},{goal_y:.3f})"
        return {
            "routeId": f"pose-{self.robot_id}-{int(time.time() * 1000)}",
            "protocol": "pose_route",
            "startLm": "CURRENT_POSE",
            "goalLm": label,
            "goalPose": {"x": goal_x, "y": goal_y, "yaw": goal_yaw},
            "nodes": ["CURRENT_POSE", label],
            "length": distance,
            "trajectory": trajectory,
        }

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

    def _current_pose_payload(self) -> dict[str, float]:
        status = self._message_to_robot_payload(self._latest_message())
        pose = status.get("pose") if isinstance(status.get("pose"), dict) else {}
        if not isinstance(pose, dict):
            pose = {}
        return {
            "x": float(pose.get("x", 0.0) or 0.0),
            "y": float(pose.get("y", 0.0) or 0.0),
            "yaw": float(pose.get("yaw", 0.0) or 0.0),
            "stampSec": time.time(),
        }

    def _current_slam_pose_payload(self) -> dict[str, float]:
        with self._lock:
            pose = dict(self._latest_slam_pose) if isinstance(self._latest_slam_pose, dict) else None
            received_at = self._latest_slam_pose_at
        if pose is not None and received_at is not None and monotonic() - received_at <= STATUS_STALE_TIMEOUT_SEC:
            pose["stampSec"] = time.time()
            return pose
        with self._lock:
            pose = dict(self._latest_odom_pose) if isinstance(self._latest_odom_pose, dict) else None
            received_at = self._latest_odom_at
        if pose is not None and received_at is not None and monotonic() - received_at <= STATUS_STALE_TIMEOUT_SEC:
            pose["stampSec"] = time.time()
            return pose
        return self._current_pose_payload()

    def _set_slam_progress(self, progress: int, message: str) -> None:
        with self._lock:
            self._slam_state.update(
                {
                    "progress": max(0, min(100, int(progress))),
                    "message": str(message or ""),
                }
            )

    def _slam_cells_to_bytes(self, values: Any) -> bytes:
        encoded = bytearray()
        for item in values:
            try:
                value = int(item)
            except (TypeError, ValueError):
                value = -1
            value = max(-1, min(100, value))
            encoded.append(value + 1)
        return bytes(encoded)

    def _safe_map_name(self, value: str) -> str:
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip()).strip("._")
        if safe.endswith(".smap"):
            safe = safe[:-5]
        return safe

    def _coerce_slam_params(self, value: Any, template: Any) -> Any:
        if isinstance(value, dict):
            template_dict = template if isinstance(template, dict) else {}
            return {key: self._coerce_slam_params(item, template_dict.get(key)) for key, item in value.items()}
        if isinstance(value, list):
            template_list = template if isinstance(template, list) else []
            if not template_list:
                return list(value)
            return [
                self._coerce_slam_params(item, template_list[min(index, len(template_list) - 1)])
                for index, item in enumerate(value)
            ]
        if isinstance(template, bool) or isinstance(value, bool):
            return value
        if isinstance(template, float) and isinstance(value, int):
            return float(value)
        if isinstance(template, int) and isinstance(value, float) and value.is_integer():
            return int(value)
        return value

    def _normalize_slam_params(self, value: Any, template: Any) -> Any:
        if not isinstance(value, dict):
            return value
        params = value.get("slam_toolbox")
        template_params = template.get("slam_toolbox") if isinstance(template, dict) else None
        if not isinstance(params, dict) or not isinstance(template_params, dict):
            return value
        ros_params = params.get("ros__parameters")
        template_ros_params = template_params.get("ros__parameters")
        if not isinstance(ros_params, dict) or not isinstance(template_ros_params, dict):
            return value

        default_base_frame = str(template_ros_params.get("base_frame") or "").strip()
        if default_base_frame and str(ros_params.get("base_frame") or "").strip() == "base_footprint":
            ros_params["base_frame"] = default_base_frame
        return value

    def _slam_maps_root(self) -> Path:
        try:
            active = self.active_map_payload()
            active_dir = Path(str(active.get("mapDir") or "")).resolve()
            if active_dir.is_dir():
                return active_dir.parent.resolve()
        except Exception:
            pass
        return self._default_slam_launch_file().parents[2] / "robot_map_manager" / "maps_out"

    def _save_slam_map_files(self, target: Path, safe_name: str) -> None:
        service_error = ""
        if self._slam_save_map_client is not None and self._save_map_type is not None:
            try:
                if self._service_available(self._slam_save_map_client, 0.5):
                    request = self._save_map_type.Request()
                    request.name.data = str(target / safe_name)
                    response = self._call_service(self._slam_save_map_client, request, "slam_toolbox/save_map", timeout_sec=20.0)
                    if int(getattr(response, "result", 255)) == 0 and (target / f"{safe_name}.yaml").is_file():
                        return
                    service_error = f"slam_toolbox/save_map result={int(getattr(response, 'result', 255))}"
            except Exception as exc:
                service_error = str(exc)
        self._write_current_map_files(target, safe_name, service_error=service_error)

    def _write_current_map_files(self, target: Path, safe_name: str, *, service_error: str = "") -> None:
        with self._lock:
            message = self._latest_map
        if message is None:
            detail = f" ({service_error})" if service_error else ""
            raise ValueError(f"No live SLAM map is available to save{detail}")
        info = getattr(message, "info", None)
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        resolution = float(getattr(info, "resolution", 0.05) or 0.05)
        if width <= 0 or height <= 0:
            raise ValueError("Live SLAM map has invalid dimensions")
        data = list(getattr(message, "data", []) or [])
        if len(data) < width * height:
            raise ValueError("Live SLAM map data is shorter than expected")

        pixels = bytearray()
        for image_y in range(height):
            grid_y = height - 1 - image_y
            row = data[grid_y * width : (grid_y + 1) * width]
            for item in row:
                try:
                    value = int(item)
                except (TypeError, ValueError):
                    value = -1
                if value < 0:
                    pixels.append(205)
                elif value >= 65:
                    pixels.append(0)
                elif value <= 25:
                    pixels.append(254)
                else:
                    pixels.append(205)

        (target / f"{safe_name}.pgm").write_bytes(
            f"P5\n# Created by warehouse_robot SLAM\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
        )
        origin = getattr(info, "origin", None)
        position = getattr(origin, "position", None)
        orientation = getattr(origin, "orientation", None)
        ros_map = {
            "image": f"{safe_name}.pgm",
            "mode": "trinary",
            "resolution": resolution,
            "origin": [
                float(getattr(position, "x", 0.0) or 0.0),
                float(getattr(position, "y", 0.0) or 0.0),
                self._yaw_from_quaternion(orientation),
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        (target / f"{safe_name}.yaml").write_text(yaml.safe_dump(ros_map, sort_keys=False), encoding="utf-8")

    def _write_empty_smap_sidecars(self, target: Path, safe_name: str) -> None:
        ros_yaml = yaml.safe_load((target / f"{safe_name}.yaml").read_text(encoding="utf-8"))
        resolution = float(ros_yaml.get("resolution", 0.05) if isinstance(ros_yaml, dict) else 0.05)
        width = height = 0
        try:
            width, height = self._pgm_size(target / f"{safe_name}.pgm")
        except Exception:
            pass
        min_x = 0.0
        min_y = 0.0
        max_x = min_x + (width * resolution)
        max_y = min_y + (height * resolution)
        (target / "LMs.yaml").write_text(
            yaml.safe_dump(
                {"mapName": safe_name, "coordinateFrame": "map_top_left", "LMs": []},
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )
        (target / "graphs.yaml").write_text(
            yaml.safe_dump(
                {"mapName": safe_name, "coordinateFrame": "map_top_left", "primitives": []},
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )
        (target / "graph_edges_lengths.yaml").write_text(
            yaml.safe_dump([], sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        (target / "primitives_lengths.csv").write_text(
            "idx,kind,type,start_x,start_y,end_x,end_y,length_m\n",
            encoding="utf-8",
        )
        created_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        (target / ".operator_meta.json").write_text(
            json.dumps(
                {
                    "source": "slam_toolbox",
                    "robotId": self.robot_id,
                    "createdAt": created_at,
                    "mapName": safe_name,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        summary = {
            "header": {
                "mapType": "2D-Map",
                "mapName": safe_name,
                "minPos": {"x": min_x, "y": min_y},
                "maxPos": {"x": max_x, "y": max_y},
                "resolution": resolution,
                "version": "slam_toolbox",
            },
            "counts": {
                "LMs_found": 0,
                "edges_total": 0,
                "grid": {"width": width, "height": height},
            },
            "outputs": {
                "pgm": f"{safe_name}.pgm",
                "ros_map_yaml": f"{safe_name}.yaml",
                "LMs_yaml": "LMs.yaml",
                "graphs_yaml": "graphs.yaml",
                "graph_edges_lengths_yaml": "graph_edges_lengths.yaml",
                "primitives_lengths_csv": "primitives_lengths.csv",
                "summary_json": "smap_summary.json",
            },
        }
        (target / "smap_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    def _pgm_size(self, path: Path) -> tuple[int, int]:
        data = path.read_bytes()
        tokens: list[bytes] = []
        index = 0
        while index < len(data) and len(tokens) < 3:
            while index < len(data) and chr(data[index]).isspace():
                index += 1
            if index < len(data) and data[index] == 35:
                while index < len(data) and data[index] not in (10, 13):
                    index += 1
                continue
            start = index
            while index < len(data) and not chr(data[index]).isspace():
                index += 1
            if start < index:
                tokens.append(data[start:index])
        if len(tokens) < 3 or tokens[0] not in {b"P5", b"P2"}:
            raise ValueError(f"invalid PGM: {path}")
        return int(tokens[1]), int(tokens[2])

    def _reset_odom_for_slam(self) -> None:
        if not self.reset_odom_service_name:
            print("[robot_api_server] Odom reset before 2D SLAM is disabled.", flush=True)
            return
        if self._reset_odom_client is None:
            raise ValueError(f"{self.reset_odom_service_name} client is not configured")

        reset_started_at = monotonic()
        with self._lock:
            self._latest_odom_pose = None
            self._latest_odom_at = None
            self._latest_slam_pose = None
            self._latest_slam_pose_at = None
            self._slam_trail = []

        request = self._reset_odom_client.srv_type.Request()
        self._call_service(self._reset_odom_client, request, "reset odom", timeout_sec=3.0)
        self._wait_for_zero_odom_after_reset(reset_started_at)
        print(f"[robot_api_server] Odom reset for 2D SLAM via {self.reset_odom_service_name}", flush=True)

    def _wait_for_zero_odom_after_reset(self, reset_started_at: float) -> None:
        deadline = monotonic() + 2.5
        last_pose: dict[str, float] | None = None
        while monotonic() < deadline:
            with self._lock:
                pose = dict(self._latest_odom_pose) if isinstance(self._latest_odom_pose, dict) else None
                received_at = self._latest_odom_at
            if pose is not None:
                last_pose = pose
            if pose is not None and received_at is not None and received_at >= reset_started_at:
                x = float(pose.get("x", 0.0) or 0.0)
                y = float(pose.get("y", 0.0) or 0.0)
                yaw = self._normalize_angle(float(pose.get("yaw", 0.0) or 0.0))
                if abs(x) <= 0.08 and abs(y) <= 0.08 and abs(yaw) <= 0.15:
                    return
            sleep(0.02)
        if last_pose is None:
            raise ValueError(f"no fresh odom received on {self.odom_topic} after reset")
        raise ValueError(
            "odom did not settle near zero after reset: "
            f"x={float(last_pose.get('x', 0.0) or 0.0):.3f}, "
            f"y={float(last_pose.get('y', 0.0) or 0.0):.3f}, "
            f"yaw={self._normalize_angle(float(last_pose.get('yaw', 0.0) or 0.0)):.3f}"
        )

    def _pause_nav2_for_slam(self) -> dict[str, Any]:
        details: dict[str, Any] = {"changed": False, "managers": [], "nodes": [], "errors": []}
        self._stop_robot_motion_for_slam(details)

        paused_labels: set[str] = set()
        manager_type = self._manage_lifecycle_nodes_type
        if manager_type is not None:
            pause_command = int(manager_type.Request.PAUSE)
            for service_name, label in NAV2_LIFECYCLE_MANAGER_SERVICES:
                if self._call_nav2_lifecycle_manager(service_name, label, pause_command, "pause", details):
                    paused_labels.add(label)
                    details["changed"] = True

        covered_nodes: set[str] = set()
        if "navigation" in paused_labels:
            covered_nodes.update({"controller_server", "planner_server", "behavior_server", "smoother_server", "bt_navigator"})
        if "localization" in paused_labels:
            covered_nodes.add("amcl")
        if "map_server" in paused_labels:
            covered_nodes.add("map_server")

        transition_type = self._transition_type
        if transition_type is not None:
            for node_name in NAV2_LIFECYCLE_NODES:
                if node_name in covered_nodes:
                    continue
                if self._call_nav2_node_transition(
                    node_name,
                    int(transition_type.TRANSITION_DEACTIVATE),
                    "deactivate",
                    details,
                ):
                    details["changed"] = True

        if bool(details.get("changed")):
            print(
                "[robot_api_server] Nav2 paused for 2D SLAM: "
                f"managers={details.get('managers') or []}, nodes={details.get('nodes') or []}",
                flush=True,
            )
        else:
            print("[robot_api_server] Nav2 lifecycle pause did not find active Nav2 services before 2D SLAM.", flush=True)
        return details

    def _resume_nav2_after_slam(self) -> dict[str, Any]:
        with self._lock:
            should_resume = bool(self._nav2_paused_for_slam)
            self._nav2_paused_for_slam = False
        details: dict[str, Any] = {"changed": False, "managers": [], "nodes": [], "errors": []}
        if not should_resume:
            return details

        resumed_labels: set[str] = set()
        manager_type = self._manage_lifecycle_nodes_type
        if manager_type is not None:
            resume_command = int(manager_type.Request.RESUME)
            for service_name, label in NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES:
                if self._call_nav2_lifecycle_manager(service_name, label, resume_command, "resume", details):
                    resumed_labels.add(label)
                    details["changed"] = True

        covered_nodes: set[str] = set()
        if "map_server" in resumed_labels:
            covered_nodes.add("map_server")
        if "localization" in resumed_labels:
            covered_nodes.add("amcl")
        if "navigation" in resumed_labels:
            covered_nodes.update({"controller_server", "planner_server", "behavior_server", "smoother_server", "bt_navigator"})

        transition_type = self._transition_type
        if transition_type is not None:
            for node_name in NAV2_LIFECYCLE_RESUME_NODES:
                if node_name in covered_nodes:
                    continue
                if self._call_nav2_node_transition(
                    node_name,
                    int(transition_type.TRANSITION_ACTIVATE),
                    "activate",
                    details,
                ):
                    details["changed"] = True

        print(
            "[robot_api_server] Nav2 resume after 2D SLAM: "
            f"managers={details.get('managers') or []}, nodes={details.get('nodes') or []}",
            flush=True,
        )
        return details

    def _stop_robot_motion_for_slam(self, details: dict[str, Any]) -> None:
        try:
            self._publish_twist(0.0, 0.0)
        except Exception as exc:
            details["errors"].append(f"cmd_vel stop failed: {exc}")

        try:
            if self._cancel_route_client is not None and self._service_available(self._cancel_route_client, 0.05):
                request = self._cancel_route_client.srv_type.Request()
                request.message = "Route canceled before 2D SLAM."
                response = self._call_service(self._cancel_route_client, request, "route cancel", timeout_sec=1.5)
                if not bool(response.ok):
                    details["errors"].append(str(response.error or "route cancel failed"))
            else:
                self._publish_go_to_lm("cancel")
        except Exception as exc:
            details["errors"].append(f"route cancel failed: {exc}")

    def _call_nav2_lifecycle_manager(
        self,
        service_name: str,
        label: str,
        command: int,
        action: str,
        details: dict[str, Any],
    ) -> bool:
        manager_type = self._manage_lifecycle_nodes_type
        if manager_type is None:
            return False
        client = self._nav2_lifecycle_clients.get(self._topic(service_name))
        if client is None or not self._service_available(client, 0.2):
            return False
        request = manager_type.Request()
        request.command = int(command)
        try:
            response = self._call_service(client, request, f"Nav2 {label} lifecycle {action}", timeout_sec=5.0)
        except Exception as exc:
            details["errors"].append(f"Nav2 {label} lifecycle {action} failed: {exc}")
            return False
        if not bool(getattr(response, "success", False)):
            details["errors"].append(f"Nav2 {label} lifecycle {action} returned success=false")
            return False
        details["managers"].append(label)
        return True

    def _call_nav2_node_transition(
        self,
        node_name: str,
        transition_id: int,
        transition_label: str,
        details: dict[str, Any],
    ) -> bool:
        if self._change_state_type is None:
            return False
        service_name = self._topic(f"/{node_name}/change_state")
        client = self._nav2_change_state_clients.get(service_name)
        if client is None or not self._service_available(client, 0.2):
            return False
        request = self._change_state_type.Request()
        request.transition.id = int(transition_id)
        request.transition.label = str(transition_label)
        try:
            response = self._call_service(client, request, f"Nav2 {node_name} {transition_label}", timeout_sec=3.0)
        except Exception as exc:
            details["errors"].append(f"Nav2 {node_name} {transition_label} failed: {exc}")
            return False
        if not bool(getattr(response, "success", False)):
            details["errors"].append(f"Nav2 {node_name} {transition_label} returned success=false")
            return False
        details["nodes"].append(node_name)
        return True

    def _stop_slam_process(self) -> None:
        process = self._slam_process
        self._slam_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except Exception:
                try:
                    process.terminate()
                except Exception:
                    pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except Exception:
                    process.kill()
        temp_dir = self._slam_temp_dir
        self._slam_temp_dir = None
        if temp_dir is not None:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def _yaw_from_quaternion(self, orientation: Any) -> float:
        if orientation is None:
            return 0.0
        x = float(getattr(orientation, "x", 0.0) or 0.0)
        y = float(getattr(orientation, "y", 0.0) or 0.0)
        z = float(getattr(orientation, "z", 0.0) or 0.0)
        w = float(getattr(orientation, "w", 1.0) or 1.0)
        return math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))

    def _stamp_sec(self, stamp: Any) -> float:
        if stamp is None:
            return 0.0
        try:
            return float(getattr(stamp, "sec", 0.0) or 0.0) + float(getattr(stamp, "nanosec", 0.0) or 0.0) / 1e9
        except (TypeError, ValueError):
            return 0.0

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(float(angle)), math.cos(float(angle)))

    def _start(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from lifecycle_msgs.msg import Transition
            from lifecycle_msgs.srv import ChangeState
            from nav_msgs.msg import OccupancyGrid, Odometry
            from nav2_msgs.srv import ManageLifecycleNodes
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.time import Time
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import ListParameters, SetParameters
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
                SetRoutePaused,
            )
            from sensor_msgs.msg import LaserScan
            from std_msgs.msg import String
            from std_srvs.srv import Empty
            from tf2_ros import Buffer, TransformListener
            try:
                from slam_toolbox.srv import SaveMap
            except Exception:
                SaveMap = None
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
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
            self._cmd_vel_pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)
            self._initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, self.initial_pose_topic, 10)
            self._go_to_lm_pub = node.create_publisher(String, self.go_to_lm_topic, 10)
            self._plan_route_client = node.create_client(PlanRoute, self.plan_service_name)
            self._execute_route_client = node.create_client(ExecuteRoute, self.execute_service_name)
            self._cancel_route_client = node.create_client(CancelRoute, self.cancel_service_name)
            self._route_pause_client = node.create_client(SetRoutePaused, self.route_pause_service_name)
            self._route_load_map_client = node.create_client(LoadRobotMap, self.route_load_map_service_name)
            self._status_load_map_client = node.create_client(LoadRobotMap, self.status_load_map_service_name)
            self._map_state_client = node.create_client(GetRobotMapState, self.map_state_service_name)
            self._map_load_client = node.create_client(LoadRobotMap, self.map_load_service_name)
            self._map_list_client = node.create_client(ListRobotMaps, self.map_list_service_name)
            self._map_get_bundle_client = node.create_client(GetRobotMapBundle, self.map_get_bundle_service_name)
            self._map_put_bundle_client = node.create_client(PutRobotMapBundle, self.map_put_bundle_service_name)
            if SaveMap is not None:
                self._slam_save_map_client = node.create_client(SaveMap, self.slam_save_map_service_name)
            if self.reset_odom_service_name:
                self._reset_odom_client = node.create_client(Empty, self.reset_odom_service_name)
            self._nav2_lifecycle_clients = {
                self._topic(service_name): node.create_client(ManageLifecycleNodes, self._topic(service_name))
                for service_name, _label in NAV2_LIFECYCLE_MANAGER_SERVICES
            }
            self._nav2_change_state_clients = {
                self._topic(f"/{node_name}/change_state"): node.create_client(ChangeState, self._topic(f"/{node_name}/change_state"))
                for node_name in NAV2_LIFECYCLE_NODES
            }
            node.create_subscription(RobotStatus, self.status_topic, self._on_status, 10)
            node.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
            node.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
            node.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 1)
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
        self._time_type = Time
        self._tf_buffer = tf_buffer
        self._tf_listener = tf_listener
        self._twist_type = Twist
        self._odom_type = Odometry
        self._pose_with_covariance_type = PoseWithCovarianceStamped
        self._laser_scan_type = LaserScan
        self._occupancy_grid_type = OccupancyGrid
        self._save_map_type = SaveMap
        self._std_empty_type = Empty
        self._string_type = String
        self._set_parameters_type = SetParameters
        self._list_parameters_type = ListParameters
        self._manage_lifecycle_nodes_type = ManageLifecycleNodes
        self._change_state_type = ChangeState
        self._transition_type = Transition
        self._parameter_type = Parameter
        self._parameter_value_type = ParameterValue
        self._parameter_type_enum = ParameterType
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

    def _on_odom(self, message: Any) -> None:
        odom_pose = self._odom_pose_payload(message)
        if odom_pose is None:
            return
        slam_pose = self._slam_tf_pose_payload() or odom_pose
        with self._lock:
            self._latest_odom_pose = odom_pose
            self._latest_odom_at = monotonic()
            self._latest_slam_pose = slam_pose
            self._latest_slam_pose_at = monotonic()
            self._append_slam_trail_pose_locked(slam_pose)

    def _on_scan(self, message: Any) -> None:
        with self._lock:
            self._latest_scan = message
            self._latest_scan_at = monotonic()

    def _on_map(self, message: Any) -> None:
        with self._lock:
            state = str(self._slam_state.get("state") or "idle")
            if not bool(self._slam_state.get("active")) or state not in {"starting", "mapping", "saving"}:
                return
            if monotonic() < self._slam_ignore_maps_until:
                return
            self._latest_map = message
            self._latest_map_at = monotonic()

    def _odom_pose_payload(self, message: Any) -> dict[str, float] | None:
        try:
            header = getattr(message, "header", None)
            pose_msg = getattr(getattr(message, "pose", None), "pose", None)
            position = getattr(pose_msg, "position", None)
            orientation = getattr(pose_msg, "orientation", None)
            return {
                "x": float(getattr(position, "x", 0.0) or 0.0),
                "y": float(getattr(position, "y", 0.0) or 0.0),
                "yaw": self._yaw_from_quaternion(orientation),
                "stampSec": self._stamp_sec(getattr(header, "stamp", None)) or time.time(),
            }
        except (TypeError, ValueError):
            return None

    def _slam_tf_pose_payload(self) -> dict[str, float] | None:
        tf_buffer = self._tf_buffer
        time_type = self._time_type
        if tf_buffer is None or time_type is None:
            return None
        try:
            transform = tf_buffer.lookup_transform(self.map_frame, self.base_frame, time_type())
            transform_stamp_sec = self._stamp_sec(getattr(getattr(transform, "header", None), "stamp", None))
            with self._lock:
                odom_pose = self._latest_odom_pose if isinstance(self._latest_odom_pose, dict) else None
            odom_stamp_sec = float(odom_pose.get("stampSec", 0.0) or 0.0) if odom_pose is not None else 0.0
            if transform_stamp_sec > 0.0 and odom_stamp_sec > 0.0 and transform_stamp_sec + 0.5 < odom_stamp_sec:
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return {
                "x": float(getattr(translation, "x", 0.0) or 0.0),
                "y": float(getattr(translation, "y", 0.0) or 0.0),
                "yaw": self._yaw_from_quaternion(rotation),
                "stampSec": transform_stamp_sec or time.time(),
            }
        except Exception:
            return None

    def _append_slam_trail_pose_locked(self, pose: dict[str, float]) -> None:
        if not bool(self._slam_state.get("active")) or str(self._slam_state.get("state") or "") != "mapping":
            return
        pose = {
            "x": float(pose.get("x", 0.0) or 0.0),
            "y": float(pose.get("y", 0.0) or 0.0),
            "yaw": float(pose.get("yaw", 0.0) or 0.0),
            "stampSec": float(pose.get("stampSec", time.time()) or time.time()),
        }
        if self._slam_trail:
            previous = self._slam_trail[-1]
            distance = math.hypot(pose["x"] - previous["x"], pose["y"] - previous["y"])
            if distance < 0.03:
                return
            if distance > 1.0:
                self._slam_trail = [pose]
                return
        self._slam_trail.append(pose)
        if len(self._slam_trail) > 5000:
            self._slam_trail = self._slam_trail[-5000:]

    def _latest_message(self) -> Any | None:
        with self._lock:
            message = self._latest_status
            received_at = self._latest_status_at
        if message is None or received_at is None:
            return None
        if monotonic() - received_at > STATUS_STALE_TIMEOUT_SEC:
            return None
        return message

    def _status_age_sec(self) -> float | None:
        with self._lock:
            received_at = self._latest_status_at
        if received_at is None:
            return None
        return max(0.0, monotonic() - received_at)

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
            status_age = self._status_age_sec()
            if status_age is None:
                detail = f"Waiting for {self.status_topic}."
            else:
                detail = f"Robot status is stale ({status_age:.1f}s). Waiting for {self.status_topic}."
            return {
                "robotId": self.robot_name,
                "mapId": "",
                "connected": False,
                "localizationOk": False,
                "localizationAgeSec": 9999.0,
                "statusAgeSec": 9999.0 if status_age is None else status_age,
                "state": "DISCONNECTED",
                "message": self._error or detail,
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
        payload = {
            "robotId": str(getattr(message, "robot_id", "") or self.robot_name),
            "mapId": str(getattr(message, "map_id", "") or ""),
            "connected": connected,
            "localizationOk": localization_ok,
            "localizationAgeSec": float(getattr(message, "localization_age_sec", 9999.0)),
            "statusAgeSec": float(self._status_age_sec() or 0.0),
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
        return payload

    def _set_route_paused(self, paused: bool, message: str) -> None:
        if self._route_pause_client is not None and self._service_available(self._route_pause_client, 0.05):
            request = self._route_pause_client.srv_type.Request()
            request.paused = bool(paused)
            request.message = str(message or "")
            response = self._call_service(self._route_pause_client, request, "route pause", timeout_sec=3.0)
            if not bool(response.ok):
                raise ValueError(str(response.error or "route pause failed"))
        else:
            if paused:
                self._publish_twist(0.0, 0.0)
            if self._route_pause_client is None:
                raise ValueError("route pause service is not configured")
            raise ValueError("route pause service is not available")
        self._navigation_paused = bool(paused)

    def _control_payload(self, robot: dict[str, Any]) -> dict[str, Any]:
        control = self._control_state_payload()
        state = str(robot.get("state") or "").upper()
        return {
            "control": control,
            "controlOwner": control["ownerId"],
            "controlOwnerName": control["ownerName"],
            "controlState": control["state"],
            "navigationPaused": bool(self._navigation_paused or state == "PAUSED"),
            "localizationConfirmed": bool(self._localization_confirmed),
            "relocationRequested": self._relocation_requested_at is not None,
        }

    def _control_state_payload(self) -> dict[str, Any]:
        with self._lock:
            self._expire_control_owner_locked()
            acquired_age = 0.0
            if self._control_acquired_at is not None:
                acquired_age = max(0.0, monotonic() - self._control_acquired_at)
            return {
                "state": "OWNED" if self._control_owner_id else "FREE",
                "ownerId": self._control_owner_id,
                "ownerName": self._control_owner_name,
                "leaseMs": self._control_lease_ms,
                "acquiredAgeSec": acquired_age,
            }

    def _ensure_control_owner(self, owner_id: str, *, action: str) -> None:
        owner = self._clean_owner_id(owner_id)
        with self._lock:
            self._expire_control_owner_locked()
            if not self._control_owner_id:
                if owner:
                    self._control_owner_id = owner
                    self._control_owner_name = owner
                    self._control_acquired_at = monotonic()
                    self._control_lease_ms = 0
                return
            if owner and owner == self._control_owner_id:
                self._control_acquired_at = self._control_acquired_at or monotonic()
                return
            if not owner:
                raise ValueError(f"cannot {action}: control is owned by {self._control_owner_name or self._control_owner_id}")
            raise ValueError(f"cannot {action}: control is owned by {self._control_owner_name or self._control_owner_id}")

    def _expire_control_owner_locked(self) -> None:
        if not self._control_owner_id or not self._control_lease_ms or self._control_acquired_at is None:
            return
        if (monotonic() - self._control_acquired_at) * 1000.0 <= float(self._control_lease_ms):
            return
        self._control_owner_id = ""
        self._control_owner_name = ""
        self._control_acquired_at = None
        self._control_lease_ms = 0

    @staticmethod
    def _clean_owner_id(owner_id: str) -> str:
        return re.sub(r"\s+", " ", str(owner_id or "").strip())[:120]

    @staticmethod
    def _covariance_from_json(value: str) -> list[float]:
        if not value:
            return []
        try:
            payload = json.loads(value)
        except json.JSONDecodeError:
            return []
        if not isinstance(payload, list) or len(payload) != 36:
            return []
        try:
            return [float(item) for item in payload]
        except (TypeError, ValueError):
            return []

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

    def _append_runtime_event(self, level: str, message: str) -> None:
        with self._lock:
            self._events.append({"stamp": monotonic(), "level": str(level or "info"), "message": str(message or "")})
            self._events = self._events[-120:]
