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
    map_yaml = LaunchConfiguration("map")
    log_level = LaunchConfiguration("log_level")
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key=namespace,
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        },
        convert_types=True,
    )
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
    ]
    common = {
        "namespace": namespace,
        "output": "screen",
        "parameters": [configured_params],
        "arguments": ["--ros-args", "--log-level", log_level],
        "remappings": remappings,
    }
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                **common,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                **common,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=namespace,
                name="lifecycle_manager_map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["map_server"]},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=namespace,
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["amcl"]},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
