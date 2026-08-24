#!/usr/bin/env python3

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _launch_file(package: str, name: str) -> str:
    return os.path.join(get_package_share_directory(package), 'launch', name)


def generate_launch_description() -> LaunchDescription:
    stage_world = LaunchConfiguration('stage_world')
    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    one_tf_tree = LaunchConfiguration('one_tf_tree')
    stage_enable_gui = LaunchConfiguration('stage_enable_gui')
    stage_fake_bms = LaunchConfiguration('stage_fake_bms')
    robot_bringup_delay = LaunchConfiguration('robot_bringup_delay')
    robot_id = LaunchConfiguration('robot_id')
    robot_namespace = LaunchConfiguration('robot_namespace')

    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('stage_ros2', 'stage.launch.py')
        ),
        launch_arguments={
            'world': stage_world,
            'enforce_prefixes': enforce_prefixes,
            'one_tf_tree': one_tf_tree,
            'enable_gui': stage_enable_gui,
            'fake_bms': stage_fake_bms,
        }.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('robot_launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'nav2_enable_rviz': LaunchConfiguration('nav2_enable_rviz'),
            'nav2_profile': LaunchConfiguration('nav2_profile'),
            'nav2_start_delay': LaunchConfiguration('nav2_start_delay'),
            'robot_start_delay': LaunchConfiguration('robot_start_delay'),
            'robot_id': robot_id,
            'robot_namespace': robot_namespace,
            'robot_api_host': LaunchConfiguration('robot_api_host'),
            'robot_api_port': LaunchConfiguration('robot_api_port'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('stage_world', default_value='22.05.26_smap'),
            DeclareLaunchArgument('enforce_prefixes', default_value='false'),
            DeclareLaunchArgument('one_tf_tree', default_value='false'),
            DeclareLaunchArgument('stage_enable_gui', default_value='true'),
            DeclareLaunchArgument('stage_fake_bms', default_value='true'),
            DeclareLaunchArgument('robot_bringup_delay', default_value='1.0'),
            DeclareLaunchArgument('nav2_enable_rviz', default_value='false'),
            DeclareLaunchArgument('nav2_profile', default_value='localization'),
            DeclareLaunchArgument('nav2_start_delay', default_value='1.0'),
            DeclareLaunchArgument('robot_start_delay', default_value='5.0'),
            DeclareLaunchArgument(
                'robot_id',
                default_value=EnvironmentVariable(
                    'ROBOT_ID', default_value='robot1'
                ),
            ),
            DeclareLaunchArgument(
                'robot_namespace',
                default_value=EnvironmentVariable(
                    'ROS_NAMESPACE', default_value=''
                ),
            ),
            DeclareLaunchArgument('robot_api_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('robot_api_port', default_value='50051'),
            LogInfo(msg='[simulation] Starting Stage hardware adapter.'),
            stage,
            TimerAction(
                period=robot_bringup_delay,
                actions=[
                    LogInfo(msg='[simulation] Starting common robot runtime.'),
                    robot,
                ],
            ),
        ]
    )
