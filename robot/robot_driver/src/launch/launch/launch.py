#!/usr/bin/env python3
import json
import os
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _project_root() -> Path:
    share_dir = Path(get_package_share_directory("robot_launch")).resolve()
    for parent in share_dir.parents:
        source_root = parent / "src"
        if (
            (source_root / "robot_map_manager").is_dir()
            and (source_root / "params" / "params.yaml").is_file()
        ):
            return parent.resolve()
        driver_root = parent / "robot" / "robot_driver"
        if (
            (driver_root / "src" / "robot_map_manager").is_dir()
            and (driver_root / "src" / "params" / "params.yaml").is_file()
        ):
            return driver_root.resolve()
    # Standard isolated colcon layout:
    # <workspace>/install/<package>/share/<package>.  The previous fixed
    # parents[4] escaped one level above the workspace and produced paths such
    # as /home/user/params.yaml.
    for parent in share_dir.parents:
        if parent.name == "install":
            return parent.parent.resolve()
    return share_dir.parents[3].resolve()


def _maps_root(project_root: Path) -> Path:
    candidates = [
        project_root / "src" / "robot_map_manager" / "maps_out",
        project_root / "maps_out",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()
    return candidates[0].resolve()


def _params_path(project_root: Path) -> Path:
    candidates = [
        project_root / "src" / "params" / "params.yaml",
        project_root / "config" / "params.yaml",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve()


def _slam_params_path(project_root: Path) -> Path:
    installed = (
        Path(get_package_share_directory("slam_toolbox"))
        / "config"
        / "mapper_params_online_async.yaml"
    )
    candidates = [
        installed,
        project_root.parent
        / "ros2_libs"
        / "src"
        / "slam_toolbox"
        / "config"
        / "mapper_params_online_async.yaml",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve()


def _slam_launch_path(project_root: Path) -> Path:
    installed = (
        Path(get_package_share_directory("slam_toolbox"))
        / "launch"
        / "online_async_launch.py"
    )
    candidates = [
        installed,
        project_root.parent
        / "ros2_libs"
        / "src"
        / "slam_toolbox"
        / "launch"
        / "online_async_launch.py",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve()


def _map_state_file(maps_root: Path) -> Path:
    return (maps_root.parent / ".active_map.json").resolve()


def _default_active_map_dir(maps_root: Path) -> Path:
    fallback = maps_root / "22.05.26_smap.smap"
    state_file = _map_state_file(maps_root)
    if not state_file.exists():
        return fallback
    try:
        payload = json.loads(state_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return fallback
    raw_map_dir = Path(str(payload.get("mapDir") or "")).expanduser()
    map_dir = raw_map_dir if raw_map_dir.is_absolute() else maps_root / raw_map_dir
    if map_dir.is_dir():
        return map_dir.resolve()
    map_name = str(payload.get("mapName") or payload.get("mapId") or "").strip()
    if map_name:
        named_map = maps_root / f"{map_name.removesuffix('.smap')}.smap"
        if named_map.is_dir():
            return named_map.resolve()
    return fallback


def _validate_robot_identity(context):
    required = str(
        LaunchConfiguration("require_unique_identity").perform(context)
    ).strip().lower() in {"1", "true", "yes", "on"}
    if not required:
        return []
    robot_id = str(LaunchConfiguration("robot_id").perform(context)).strip()
    namespace = str(
        LaunchConfiguration("robot_namespace").perform(context)
    ).strip().strip("/")
    domain_raw = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]{1,63}", robot_id):
        raise RuntimeError(
            "ROBOT_ID must start with a letter and contain 2-64 letters, "
            "digits, underscores or hyphens"
        )
    if not namespace or not re.fullmatch(
        r"[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*",
        namespace,
    ):
        raise RuntimeError(
            "ROS_NAMESPACE is required for a real robot and must be a valid "
            "ROS namespace"
        )
    try:
        domain_id = int(domain_raw)
    except ValueError as exc:
        raise RuntimeError(
            "ROS_DOMAIN_ID is required for a real robot and must be an integer"
        ) from exc
    if not 0 <= domain_id <= 232:
        raise RuntimeError("ROS_DOMAIN_ID must be between 0 and 232")
    if os.environ.get("RMW_IMPLEMENTATION", "").strip() != "rmw_cyclonedds_cpp":
        raise RuntimeError(
            "RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp for robot deployment"
        )
    if not os.environ.get("CYCLONEDDS_URI", "").strip():
        raise RuntimeError("CYCLONEDDS_URI is required for robot deployment")
    return []


def generate_launch_description() -> LaunchDescription:
    project_root = _project_root()
    maps_root = _maps_root(project_root)
    default_map_dir = str(_default_active_map_dir(maps_root))
    default_params = str(_params_path(project_root))
    default_slam_params = str(_slam_params_path(project_root))
    default_slam_launch = str(_slam_launch_path(project_root))
    default_maps_root = str(maps_root)
    default_state_file = str(_map_state_file(maps_root))

    arguments = [
        DeclareLaunchArgument("map_dir", default_value=default_map_dir),
        DeclareLaunchArgument("maps_root", default_value=default_maps_root),
        DeclareLaunchArgument("state_file", default_value=default_state_file),
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="robot1"),
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=EnvironmentVariable("ROS_NAMESPACE", default_value=""),
        ),
        DeclareLaunchArgument(
            "require_unique_identity",
            default_value=EnvironmentVariable(
                "WAREHOUSE_REQUIRE_IDENTITY",
                default_value="false",
            ),
        ),
        DeclareLaunchArgument("robot_api_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("robot_api_port", default_value="50051"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
        DeclareLaunchArgument("route_cmd_vel_topic", default_value="motion/route_cmd_vel"),
        DeclareLaunchArgument("teleop_cmd_vel_topic", default_value="motion/teleop_cmd_vel"),
        DeclareLaunchArgument("nav2_cmd_vel_topic", default_value="motion/nav2_cmd_vel"),
        DeclareLaunchArgument("motion_mode_topic", default_value="motion/mode"),
        DeclareLaunchArgument("motion_state_topic", default_value="motion/state"),
        DeclareLaunchArgument("amcl_topic", default_value="amcl_pose"),
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        DeclareLaunchArgument("battery_topic", default_value="bms"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("status_topic", default_value="robot_status"),
        DeclareLaunchArgument("executor_status_topic", default_value="route/executor_state"),
        DeclareLaunchArgument("plan_service", default_value="route/plan"),
        DeclareLaunchArgument("execute_service", default_value="route/execute"),
        DeclareLaunchArgument("cancel_service", default_value="route/cancel"),
        DeclareLaunchArgument("route_pause_service", default_value="route/pause"),
        DeclareLaunchArgument("route_load_map_service", default_value="route/load_map"),
        DeclareLaunchArgument("status_load_map_service", default_value="status/load_map"),
        DeclareLaunchArgument("manager_load_map_service", default_value="robot/maps/load"),
        DeclareLaunchArgument("manager_state_service", default_value="robot/maps/state"),
        DeclareLaunchArgument("manager_list_service", default_value="robot/maps/list"),
        DeclareLaunchArgument("manager_get_bundle_service", default_value="robot/maps/get_bundle"),
        DeclareLaunchArgument("manager_put_bundle_service", default_value="robot/maps/put_bundle"),
        DeclareLaunchArgument("map_server_load_service", default_value="map_server/load_map"),
        DeclareLaunchArgument("map_topic", default_value="map"),
        DeclareLaunchArgument("initial_pose_topic", default_value="initialpose"),
        DeclareLaunchArgument("slam_save_map_service", default_value="slam_toolbox/save_map"),
        DeclareLaunchArgument("reset_odom_service", default_value="reset_odom"),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
        DeclareLaunchArgument("slam_launch_file", default_value=default_slam_launch),
    ]

    status_node = Node(
        package="robot_status",
        executable="status_node",
        name="robot_status",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        arguments=[
            "--map-dir",
            LaunchConfiguration("map_dir"),
            "--params",
            LaunchConfiguration("params"),
            "--robot-id",
            LaunchConfiguration("robot_id"),
            "--cmd-vel-topic",
            LaunchConfiguration("cmd_vel_topic"),
            "--amcl-topic",
            LaunchConfiguration("amcl_topic"),
            "--odom-topic",
            LaunchConfiguration("odom_topic"),
            "--battery-topic",
            LaunchConfiguration("battery_topic"),
            "--status-topic",
            LaunchConfiguration("status_topic"),
            "--executor-status-topic",
            LaunchConfiguration("executor_status_topic"),
            "--load-map-service",
            LaunchConfiguration("status_load_map_service"),
        ],
    )

    route_node = Node(
        package="robot_planner",
        executable="route_node",
        name="robot_route",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        arguments=[
            "--map-dir",
            LaunchConfiguration("map_dir"),
            "--params",
            LaunchConfiguration("params"),
            "--robot-id",
            LaunchConfiguration("robot_id"),
            "--cmd-vel-topic",
            LaunchConfiguration("route_cmd_vel_topic"),
            "--motion-mode-topic",
            LaunchConfiguration("motion_mode_topic"),
            "--odom-topic",
            LaunchConfiguration("odom_topic"),
            "--initial-pose-topic",
            LaunchConfiguration("initial_pose_topic"),
            "--status-topic",
            LaunchConfiguration("status_topic"),
            "--executor-status-topic",
            LaunchConfiguration("executor_status_topic"),
            "--plan-service",
            LaunchConfiguration("plan_service"),
            "--execute-service",
            LaunchConfiguration("execute_service"),
            "--cancel-service",
            LaunchConfiguration("cancel_service"),
            "--pause-service",
            LaunchConfiguration("route_pause_service"),
            "--load-map-service",
            LaunchConfiguration("route_load_map_service"),
        ],
    )

    map_manager_node = Node(
        package="robot_map_manager",
        executable="map_manager_node",
        name="robot_map_manager",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        arguments=[
            "--map-dir",
            LaunchConfiguration("map_dir"),
            "--maps-root",
            LaunchConfiguration("maps_root"),
            "--state-file",
            LaunchConfiguration("state_file"),
            "--map-server-load-service",
            LaunchConfiguration("map_server_load_service"),
            "--route-load-map-service",
            LaunchConfiguration("route_load_map_service"),
            "--status-load-map-service",
            LaunchConfiguration("status_load_map_service"),
            "--manager-load-service",
            LaunchConfiguration("manager_load_map_service"),
            "--manager-state-service",
            LaunchConfiguration("manager_state_service"),
            "--manager-list-service",
            LaunchConfiguration("manager_list_service"),
            "--manager-get-bundle-service",
            LaunchConfiguration("manager_get_bundle_service"),
            "--manager-put-bundle-service",
            LaunchConfiguration("manager_put_bundle_service"),
        ],
    )

    robot_api_node = Node(
        package="robot_grpc_api",
        executable="robot_api_server",
        name="robot_api",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        arguments=[
            "--host",
            LaunchConfiguration("robot_api_host"),
            "--port",
            LaunchConfiguration("robot_api_port"),
            "--robot-id",
            LaunchConfiguration("robot_id"),
            "--robot-name",
            LaunchConfiguration("robot_id"),
            "--namespace",
            LaunchConfiguration("robot_namespace"),
            "--status-topic",
            LaunchConfiguration("status_topic"),
            "--cmd-vel-topic",
            LaunchConfiguration("teleop_cmd_vel_topic"),
            "--driver-cmd-vel-topic",
            LaunchConfiguration("cmd_vel_topic"),
            "--motion-mode-topic",
            LaunchConfiguration("motion_mode_topic"),
            "--odom-topic",
            LaunchConfiguration("odom_topic"),
            "--initial-pose-topic",
            LaunchConfiguration("initial_pose_topic"),
            "--map-frame",
            LaunchConfiguration("map_frame"),
            "--base-frame",
            LaunchConfiguration("base_frame"),
            "--go-to-lm-topic",
            "go_to_lm",
            "--plan-service",
            LaunchConfiguration("plan_service"),
            "--execute-service",
            LaunchConfiguration("execute_service"),
            "--cancel-service",
            LaunchConfiguration("cancel_service"),
            "--route-pause-service",
            LaunchConfiguration("route_pause_service"),
            "--route-load-map-service",
            LaunchConfiguration("route_load_map_service"),
            "--status-load-map-service",
            LaunchConfiguration("status_load_map_service"),
            "--map-state-service",
            LaunchConfiguration("manager_state_service"),
            "--map-load-service",
            LaunchConfiguration("manager_load_map_service"),
            "--map-list-service",
            LaunchConfiguration("manager_list_service"),
            "--map-get-bundle-service",
            LaunchConfiguration("manager_get_bundle_service"),
            "--map-put-bundle-service",
            LaunchConfiguration("manager_put_bundle_service"),
            "--map-topic",
            LaunchConfiguration("map_topic"),
            "--slam-save-map-service",
            LaunchConfiguration("slam_save_map_service"),
            "--reset-odom-service",
            LaunchConfiguration("reset_odom_service"),
            "--slam-params-file",
            LaunchConfiguration("slam_params_file"),
            "--slam-launch-file",
            LaunchConfiguration("slam_launch_file"),
            "--params",
            LaunchConfiguration("params"),
        ],
    )

    motion_gateway_node = Node(
        package="robot_motion_gateway",
        executable="motion_gateway",
        name="robot_motion_gateway",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        arguments=[
            "--robot-id",
            LaunchConfiguration("robot_id"),
            "--params",
            LaunchConfiguration("params"),
            "--output-topic",
            LaunchConfiguration("cmd_vel_topic"),
            "--route-topic",
            LaunchConfiguration("route_cmd_vel_topic"),
            "--teleop-topic",
            LaunchConfiguration("teleop_cmd_vel_topic"),
            "--nav2-topic",
            LaunchConfiguration("nav2_cmd_vel_topic"),
            "--mode-topic",
            LaunchConfiguration("motion_mode_topic"),
            "--state-topic",
            LaunchConfiguration("motion_state_topic"),
        ],
    )

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=_validate_robot_identity),
            motion_gateway_node,
            status_node,
            route_node,
            map_manager_node,
            robot_api_node,
        ]
    )
