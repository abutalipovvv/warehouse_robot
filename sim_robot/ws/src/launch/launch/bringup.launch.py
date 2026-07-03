#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_file(package: str, *relative: str) -> str:
    return os.path.join(get_package_share_directory(package), "launch", *relative)


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    stage_world = LaunchConfiguration("stage_world")
    enforce_prefixes = LaunchConfiguration("enforce_prefixes")
    one_tf_tree = LaunchConfiguration("one_tf_tree")
    stage_enable_gui = LaunchConfiguration("stage_enable_gui")
    stage_fake_bms = LaunchConfiguration("stage_fake_bms")
    nav2_enable_rviz = LaunchConfiguration("nav2_enable_rviz")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    robot_start_delay = LaunchConfiguration("robot_start_delay")
    robot_id = LaunchConfiguration("robot_id")
    robot_api_host = LaunchConfiguration("robot_api_host")
    robot_api_port = LaunchConfiguration("robot_api_port")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("stage_world", default_value="22.05.26_smap"),
        DeclareLaunchArgument("enforce_prefixes", default_value="false"),
        DeclareLaunchArgument("one_tf_tree", default_value="false"),
        DeclareLaunchArgument("stage_enable_gui", default_value="true"),
        DeclareLaunchArgument("stage_fake_bms", default_value="true"),
        DeclareLaunchArgument("nav2_enable_rviz", default_value="true"),
        DeclareLaunchArgument("nav2_start_delay", default_value="2.0"),
        DeclareLaunchArgument("robot_start_delay", default_value="6.0"),
        DeclareLaunchArgument("robot_id", default_value="robot1"),
        DeclareLaunchArgument("robot_api_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("robot_api_port", default_value="50051"),
    ]

    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("stage_ros2", "stage.launch.py")),
        launch_arguments={
            "world": stage_world,
            "enforce_prefixes": enforce_prefixes,
            "one_tf_tree": one_tf_tree,
            "enable_gui": stage_enable_gui,
            "fake_bms": stage_fake_bms,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("nav2", "nav2_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_rviz": nav2_enable_rviz,
        }.items(),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("robot_launch", "launch.py")),
        launch_arguments={
            "robot_id": robot_id,
            "robot_api_host": robot_api_host,
            "robot_api_port": robot_api_port,
        }.items(),
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(msg="[bringup] Starting Stage."),
            stage,
            TimerAction(
                period=nav2_start_delay,
                actions=[
                    LogInfo(msg="[bringup] Starting Nav2."),
                    nav2,
                ],
            ),
            TimerAction(
                period=robot_start_delay,
                actions=[
                    LogInfo(msg="[bringup] Starting robot API/planner/status."),
                    robot,
                ],
            ),
        ]
    )
