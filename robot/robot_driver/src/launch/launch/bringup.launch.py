#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _launch_file(package: str, *relative: str) -> str:
    return os.path.join(get_package_share_directory(package), "launch", *relative)


def _configure_network_identity(context):
    from robot_grpc_api.network_identity import resolve_network_identity

    identity = resolve_network_identity(
        LaunchConfiguration("robot_id").perform(context),
        LaunchConfiguration("ros_domain_id").perform(context),
    )
    actions = [
        SetLaunchConfiguration("robot_id", identity.robot_id),
        SetLaunchConfiguration("ros_domain_id", str(identity.domain_id)),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(identity.domain_id)),
    ]
    if identity.interface != "explicit":
        actions.append(
            LogInfo(
                msg=(
                    f"[identity] {identity.interface}={identity.ipv4} "
                    f"ROBOT_ID={identity.robot_id} "
                    f"ROS_DOMAIN_ID={identity.domain_id}"
                )
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_enable_rviz = LaunchConfiguration("nav2_enable_rviz")
    nav2_profile = LaunchConfiguration("nav2_profile")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    robot_start_delay = LaunchConfiguration("robot_start_delay")
    robot_id = LaunchConfiguration("robot_id")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_api_host = LaunchConfiguration("robot_api_host")
    robot_api_port = LaunchConfiguration("robot_api_port")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_enable_rviz", default_value="false"),
        DeclareLaunchArgument("nav2_profile", default_value="localization"),
        DeclareLaunchArgument("nav2_start_delay", default_value="0.0"),
        DeclareLaunchArgument("robot_start_delay", default_value="2.0"),
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="auto"),
        ),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value=EnvironmentVariable(
                "ROS_DOMAIN_ID",
                default_value="auto",
            ),
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=EnvironmentVariable("ROS_NAMESPACE", default_value=""),
        ),
        DeclareLaunchArgument("robot_api_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("robot_api_port", default_value="50051"),
    ]

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("warehouse_nav2_bringup", "nav2_launch.py")
        ),
        launch_arguments={
            "profile": nav2_profile,
            "namespace": robot_namespace,
            "use_sim_time": use_sim_time,
            "enable_rviz": nav2_enable_rviz,
            "nav2_cmd_vel_topic": "motion/nav2_cmd_vel",
        }.items(),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("robot_launch", "launch.py")),
        launch_arguments={
            "robot_id": robot_id,
            "ros_domain_id": ros_domain_id,
            "robot_namespace": robot_namespace,
            "robot_api_host": robot_api_host,
            "robot_api_port": robot_api_port,
        }.items(),
    )

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=_configure_network_identity),
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
                    LogInfo(msg="[bringup] Starting robot runtime."),
                    robot,
                ],
            ),
        ]
    )
