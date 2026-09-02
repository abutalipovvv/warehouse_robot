#!/usr/bin/env python3
"""Start one complete simulation robot stack inside its own container."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package: str, name: str) -> str:
    return os.path.join(get_package_share_directory(package), "launch", name)


def _environment_argument(name: str, default: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        name.lower(),
        default_value=EnvironmentVariable(name, default_value=default),
    )


def generate_launch_description() -> LaunchDescription:
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("ecom_mobile_robot_description", "launch.py")
        )
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("robot_launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "nav2_enable_rviz": "false",
            "nav2_profile": LaunchConfiguration("nav2_profile"),
            "nav2_start_delay": "1.0",
            "robot_start_delay": "5.0",
            "robot_id": LaunchConfiguration("robot_id"),
            "ros_domain_id": LaunchConfiguration("ros_domain_id"),
            "robot_namespace": "",
            "robot_api_host": LaunchConfiguration("robot_api_host"),
            "robot_api_port": LaunchConfiguration("robot_api_port"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            "map_dir": LaunchConfiguration("map_dir"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "maps_root": LaunchConfiguration("maps_root"),
            "state_file": LaunchConfiguration("state_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            _environment_argument("ROBOT_ID", "robot11"),
            _environment_argument("ROS_DOMAIN_ID", "11"),
            _environment_argument("ROBOT_API_HOST", "127.0.0.11"),
            _environment_argument("ROBOT_API_PORT", "50051"),
            _environment_argument("INITIAL_POSE_X", "-4.902"),
            _environment_argument("INITIAL_POSE_Y", "1.362"),
            _environment_argument("INITIAL_POSE_YAW", "3.141592653589793"),
            _environment_argument("MAP_DIR", "/maps/22.05.26_smap.smap"),
            _environment_argument(
                "MAP_YAML", "/maps/22.05.26_smap.smap/22.05.26_smap.yaml"
            ),
            _environment_argument("MAPS_ROOT", "/maps"),
            _environment_argument(
                "STATE_FILE", "/var/lib/warehouse_robot/state/active_map.json"
            ),
            _environment_argument("NAV2_PROFILE", "navigation"),
            _environment_argument("FAKE_BMS", "true"),
            description,
            Node(
                package="robot_status",
                executable="fake_bms_publisher",
                name="fake_bms_pub",
                output="log",
                condition=IfCondition(LaunchConfiguration("fake_bms")),
            ),
            bringup,
        ]
    )
