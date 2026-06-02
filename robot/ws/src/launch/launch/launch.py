#!/usr/bin/env python3
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


def generate_launch_description() -> LaunchDescription:
    project_root = _project_root()
    default_map_dir = str(project_root / "map_data" / "maps_out" / "22.05.26_smap.smap")
    default_params = str(project_root / "params.yaml")

    arguments = [
        DeclareLaunchArgument("map_dir", default_value=default_map_dir),
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
        ],
    )

    return LaunchDescription(arguments + [status_node, route_node])
