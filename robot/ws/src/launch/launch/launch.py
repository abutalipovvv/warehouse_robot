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
        if (parent / "map_data").exists() and (parent / "route_core").exists():
            return parent
    return share_dir.parents[4]


def _default_active_map_dir(project_root: Path) -> Path:
    fallback = project_root / "map_data" / "maps_out" / "22.05.26_smap.smap"
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
    default_map_dir = str(_default_active_map_dir(project_root))
    default_params = str(project_root / "params.yaml")
    default_maps_root = str((project_root / "map_data" / "maps_out").resolve())
    default_state_file = str((project_root / "robot" / ".active_map.json").resolve())

    arguments = [
        DeclareLaunchArgument("map_dir", default_value=default_map_dir),
        DeclareLaunchArgument("maps_root", default_value=default_maps_root),
        DeclareLaunchArgument("state_file", default_value=default_state_file),
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("robot_id", default_value="robot1"),
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
        ],
    )

    return LaunchDescription(arguments + [status_node, route_node, map_manager_node])
