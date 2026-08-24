"""Own ROS node wiring, subscriptions, callbacks and runtime state."""

from __future__ import annotations

import math
import re
import subprocess
import threading
import time
from pathlib import Path
from time import monotonic, sleep
from typing import Any

def _clean_node_suffix(value: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_]", "_", value.strip())
    clean = re.sub(r"_+", "_", clean).strip("_")
    if not clean or clean[0].isdigit():
        clean = f"robot_{clean or 'api'}"
    return clean[:64]

class RosRuntimeLifecycleMixin:
    """Own ROS node wiring, subscriptions, callbacks and runtime state."""

    def __init__(
        self,
        *,
        robot_id: str,
        robot_name: str,
        host: str = "",
        domain_id: int | None = None,
        namespace: str = "",
        status_topic: str = "/robot_status",
        cmd_vel_topic: str = "motion/teleop_cmd_vel",
        driver_cmd_vel_topic: str = "cmd_vel",
        motion_mode_topic: str = "motion/mode",
        initial_pose_topic: str = "/initialpose",
        scan_topic: str = "/scan",
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
        self.driver_cmd_vel_topic = self._topic(driver_cmd_vel_topic)
        self.motion_mode_topic = self._topic(motion_mode_topic)
        self.initial_pose_topic = self._topic(initial_pose_topic)
        self.scan_topic = self._topic(scan_topic)
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
        self.slam_params_file = Path(slam_params_file).expanduser().resolve() if slam_params_file else self._default_slam_params_file()
        self.slam_launch_file = Path(slam_launch_file).expanduser().resolve() if slam_launch_file else self._default_slam_launch_file()
        self.params_path = Path(params_path).expanduser().resolve() if params_path else None
        self._lock = threading.Lock()
        self._latest_status: Any | None = None
        self._latest_status_at: float | None = None
        self._latest_scan: Any | None = None
        self._latest_scan_at: float | None = None
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
        self._twist_type = None
        self._pose_with_covariance_type = None
        self._laser_scan_type = None
        self._string_type = None
        self._set_parameters_type = None
        self._list_parameters_type = None
        self._parameter_type = None
        self._parameter_value_type = None
        self._parameter_type_enum = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._cmd_vel_pub = None
        self._motion_mode_pub = None
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
        self._save_map_type = None
        self._occupancy_grid_type = None
        self._slam_process: subprocess.Popen | None = None
        self._slam_temp_dir: Path | None = None
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
            candidate = (
                parent
                / "robot"
                / "ros2_libs"
                / "src"
                / "slam_toolbox"
                / "config"
                / "mapper_params_online_async.yaml"
            )
            if candidate.is_file():
                return candidate.resolve()
        return Path("robot/ros2_libs/src/slam_toolbox/config/mapper_params_online_async.yaml").resolve()

    def _default_slam_launch_file(self) -> Path:
        for parent in Path(__file__).resolve().parents:
            candidate = (
                parent
                / "robot"
                / "ros2_libs"
                / "src"
                / "slam_toolbox"
                / "launch"
                / "online_async_launch.py"
            )
            if candidate.is_file():
                return candidate.resolve()
        return Path("robot/ros2_libs/src/slam_toolbox/launch/online_async_launch.py").resolve()

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

    def _start(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from nav_msgs.msg import OccupancyGrid
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
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
            self._cmd_vel_pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)
            self._motion_mode_pub = node.create_publisher(
                String,
                self.motion_mode_topic,
                10,
            )
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
            node.create_subscription(RobotStatus, self.status_topic, self._on_status, 10)
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
        self._twist_type = Twist
        self._pose_with_covariance_type = PoseWithCovarianceStamped
        self._laser_scan_type = LaserScan
        self._occupancy_grid_type = OccupancyGrid
        self._save_map_type = SaveMap
        self._string_type = String
        self._set_parameters_type = SetParameters
        self._list_parameters_type = ListParameters
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
            self._append_slam_trail_locked(message)
        self._persist_event(previous, message)

    def _on_scan(self, message: Any) -> None:
        with self._lock:
            self._latest_scan = message
            self._latest_scan_at = monotonic()

    def _on_map(self, message: Any) -> None:
        with self._lock:
            self._latest_map = message
            self._latest_map_at = monotonic()

    def _append_slam_trail_locked(self, message: Any) -> None:
        if not bool(self._slam_state.get("active")):
            return
        try:
            pose = {
                "x": float(getattr(message, "pose_x", 0.0)),
                "y": float(getattr(message, "pose_y", 0.0)),
                "yaw": float(getattr(message, "pose_yaw", 0.0)),
                "stampSec": time.time(),
            }
        except (TypeError, ValueError):
            return
        if self._slam_trail:
            previous = self._slam_trail[-1]
            if math.hypot(pose["x"] - previous["x"], pose["y"] - previous["y"]) < 0.03:
                return
        self._slam_trail.append(pose)
        if len(self._slam_trail) > 5000:
            self._slam_trail = self._slam_trail[-5000:]

    def _latest_message(self) -> Any | None:
        with self._lock:
            return self._latest_status
