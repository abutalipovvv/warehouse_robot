#!/usr/bin/env python3
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _project_root() -> Path:
    share_dir = Path(get_package_share_directory("robot_launch"))
    for parent in share_dir.parents:
        if (parent / "commands.txt").exists() and (parent / "robot").exists():
            return parent
        if (parent / "fleet_manager").exists() and (parent / "robot").exists():
            return parent
    return share_dir.parents[4]


def _maps_root(project_root: Path) -> Path:
    candidates = [
        project_root / "map_data" / "maps_out",
        project_root / "fleet_manager" / "map_data" / "maps_out",
        project_root / "robot" / "ws" / "src" / "robot_map_manager" / "maps_out",
        project_root / "operator_app" / "map_out" / "robot1_127.0.0.1_50051",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()
    return candidates[0].resolve()


def _params_path(project_root: Path) -> Path:
    candidates = [
        project_root / "params.yaml",
        project_root / "fleet_manager" / "params.yaml",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve()


def _default_active_map_dir(project_root: Path) -> Path:
    fallback = _maps_root(project_root) / "22.05.26_smap.smap"
    state_file = project_root / "robot" / ".active_map.json"
    if not state_file.exists():
        return fallback
    try:
        payload = json.loads(state_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return fallback
    map_dir = Path(str(payload.get("mapDir") or "")).expanduser()
    if map_dir.is_dir():
        return map_dir
    return fallback


def generate_launch_description() -> LaunchDescription:
    project_root = _project_root()
    maps_root = _maps_root(project_root)
    default_map_dir = str(_default_active_map_dir(project_root))
    default_params = str(_params_path(project_root))
    default_maps_root = str(maps_root)
    default_state_file = str((project_root / "robot" / ".active_map.json").resolve())

    arguments = [
        DeclareLaunchArgument("map_dir", default_value=default_map_dir),
        DeclareLaunchArgument("maps_root", default_value=default_maps_root),
        DeclareLaunchArgument("state_file", default_value=default_state_file),
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("robot_id", default_value="robot1"),
        DeclareLaunchArgument("robot_api_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("robot_api_port", default_value="50051"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("amcl_topic", default_value="/amcl_pose"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("status_topic", default_value="/robot_status"),
        DeclareLaunchArgument("executor_status_topic", default_value="/route/executor_state"),
        DeclareLaunchArgument("plan_service", default_value="/route/plan"),
        DeclareLaunchArgument("execute_service", default_value="/route/execute"),
        DeclareLaunchArgument("cancel_service", default_value="/route/cancel"),
        DeclareLaunchArgument("route_load_map_service", default_value="/route/load_map"),
        DeclareLaunchArgument("status_load_map_service", default_value="/status/load_map"),
        DeclareLaunchArgument("manager_load_map_service", default_value="/robot/maps/load"),
        DeclareLaunchArgument("manager_state_service", default_value="/robot/maps/state"),
        DeclareLaunchArgument("manager_list_service", default_value="/robot/maps/list"),
        DeclareLaunchArgument("manager_get_bundle_service", default_value="/robot/maps/get_bundle"),
        DeclareLaunchArgument("manager_put_bundle_service", default_value="/robot/maps/put_bundle"),
        DeclareLaunchArgument("map_server_load_service", default_value="/map_server/load_map"),
    ]

    status_node = Node(
        package="robot_status",
        executable="status_node",
        name="robot_status",
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
            "--load-map-service",
            LaunchConfiguration("route_load_map_service"),
        ],
    )

    map_manager_node = Node(
        package="robot_map_manager",
        executable="map_manager_node",
        name="robot_map_manager",
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
        package="robot_api",
        executable="robot_api_server",
        name="robot_api",
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
            "--status-topic",
            LaunchConfiguration("status_topic"),
            "--cmd-vel-topic",
            LaunchConfiguration("cmd_vel_topic"),
            "--go-to-lm-topic",
            "/go_to_lm",
            "--plan-service",
            LaunchConfiguration("plan_service"),
            "--execute-service",
            LaunchConfiguration("execute_service"),
            "--cancel-service",
            LaunchConfiguration("cancel_service"),
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
        ],
    )

    return LaunchDescription(arguments + [status_node, route_node, map_manager_node, robot_api_node])
