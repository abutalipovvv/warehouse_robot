#!/usr/bin/env python3

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    nav2_cmd_vel_topic = LaunchConfiguration("nav2_cmd_vel_topic")
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key=namespace,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )
    common_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    def nav2_node(package: str, executable: str, name: str, *, velocity=False):
        remappings = list(common_remappings)
        if velocity:
            remappings.append(("cmd_vel", nav2_cmd_vel_topic))
        return Node(
            package=package,
            executable=executable,
            namespace=namespace,
            name=name,
            output="screen",
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        )

    managed_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument(
                "nav2_cmd_vel_topic",
                default_value="motion/nav2_cmd_vel",
            ),
            nav2_node(
                "nav2_controller",
                "controller_server",
                "controller_server",
                velocity=True,
            ),
            nav2_node("nav2_smoother", "smoother_server", "smoother_server"),
            nav2_node("nav2_planner", "planner_server", "planner_server"),
            nav2_node(
                "nav2_behaviors",
                "behavior_server",
                "behavior_server",
                velocity=True,
            ),
            nav2_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=namespace,
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": managed_nodes},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
