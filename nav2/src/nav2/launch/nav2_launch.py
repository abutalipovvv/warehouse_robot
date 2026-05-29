#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_lm_route = LaunchConfiguration("enable_lm_route")
    default_goal_lm = LaunchConfiguration("default_goal_lm")
    web_port = LaunchConfiguration("web_port")
    pkg_path = get_package_share_directory("nav2")

    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="enable_rviz",
            default_value="true",
            description="Enable rviz launch",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="enable_lm_route",
            default_value="false",
            description="Enable strict LM route manager",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="default_goal_lm",
            default_value="",
            description="Optional LM goal to start after localization",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="web_port",
            default_value="8765",
            description="HTTP port for the operator panel served by lm_route_manager",
        )
    )


    nav2_launch_file = os.path.join(pkg_path, "launch", "nav2", "bringup_launch.py")
    map_yaml_file = os.path.join(pkg_path, "maps", "22.05.26_smap.yaml")
    params_file = os.path.join(pkg_path, "config", "nav2_params.yaml")

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "map": map_yaml_file,
            "use_namespace": "False",
            "params_file": params_file,
            "autostart": "true",
            "use_sim_time": use_sim_time,
            "log_level": "warn",
            "map_server": "True",
        }.items(),
    )

    message = (
        f"{{header: {{frame_id: map}}, pose: {{pose: {{position: "
        f"{{x: [-4.902], y: [1.362], z: 0.1}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, }} }}"
    )
    initial_pose_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
            '/initialpose',
            'geometry_msgs/PoseWithCovarianceStamped',
            message
        ],
        output='screen'
    )
    nav2_actions = GroupAction(
        [
            bringup_cmd,
            #initial_pose_cmd,
        ]
    )

    rviz_launch_file = os.path.join(pkg_path, "launch", "rviz_launch.py")
    rviz_config_file = os.path.join(pkg_path, "rviz", "nav2_default_view.rviz")
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        launch_arguments={
            "use_namespace": "false",
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
        condition=IfCondition(enable_rviz),
    )

    fake_bms = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/battery_state",
            "sensor_msgs/msg/BatteryState",
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 12.0, percentage: 0.8, capacity: 5.0}",
            "-r",
            "1",
        ],
        output="log",
    )
    goal_pose_bridge = Node(
        package="nav2",
        executable="goal_pose_bridge.py",
        name="goal_pose_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    lm_route_manager = Node(
        package="nav2",
        executable="lm_route_manager.py",
        name="lm_route_manager",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_dir": os.path.join(pkg_path, "maps")},
            {"default_goal_lm": default_goal_lm},
            {"path_topic": "/lm_route_path"},
            {"goal_topic": "/lm_goal"},
            {"sample_distance": 0.05},
            {"web_port": web_port},
            {"enable_web_server": True},
        ],
        condition=IfCondition(enable_lm_route),
    )


    ld.add_action(GroupAction([fake_bms, goal_pose_bridge, lm_route_manager, nav2_actions, rviz]))
    return ld
