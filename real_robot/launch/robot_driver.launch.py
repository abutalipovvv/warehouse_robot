from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.192.5"),
        DeclareLaunchArgument("robot_id", default_value="robot1"),
        DeclareLaunchArgument("map_id", default_value=""),
        DeclareLaunchArgument("status_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("command_duration_ms", default_value="350"),
        DeclareLaunchArgument("socket_timeout_sec", default_value="0.8"),
        DeclareLaunchArgument("localization_confidence_min", default_value="0.2"),
        DeclareLaunchArgument("acquire_control_on_start", default_value="false"),
        DeclareLaunchArgument("acquire_control_before_command", default_value="false"),
        DeclareLaunchArgument("release_control_on_shutdown", default_value="true"),
        DeclareLaunchArgument("control_nick_name", default_value="warehouse_robot_driver"),
        DeclareLaunchArgument("default_source_id", default_value=""),
        DeclareLaunchArgument("odom_frame_id", default_value="map"),
        DeclareLaunchArgument("base_frame_id", default_value="base_link"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("status_topic", default_value="/robot_status"),
        DeclareLaunchArgument("bms_topic", default_value="/bms"),
        DeclareLaunchArgument("go_to_lm_topic", default_value="/go_to_lm"),
        DeclareLaunchArgument("navigate_status_topic", default_value="/navigate_status"),
    ]

    driver_node = Node(
        package="real_robot",
        executable="robot_driver",
        name="robot_driver",
        output="screen",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "robot_id": LaunchConfiguration("robot_id"),
                "map_id": LaunchConfiguration("map_id"),
                "status_rate_hz": ParameterValue(LaunchConfiguration("status_rate_hz"), value_type=float),
                "command_duration_ms": ParameterValue(LaunchConfiguration("command_duration_ms"), value_type=int),
                "socket_timeout_sec": ParameterValue(LaunchConfiguration("socket_timeout_sec"), value_type=float),
                "localization_confidence_min": ParameterValue(
                    LaunchConfiguration("localization_confidence_min"),
                    value_type=float,
                ),
                "acquire_control_on_start": ParameterValue(
                    LaunchConfiguration("acquire_control_on_start"),
                    value_type=bool,
                ),
                "acquire_control_before_command": ParameterValue(
                    LaunchConfiguration("acquire_control_before_command"),
                    value_type=bool,
                ),
                "release_control_on_shutdown": ParameterValue(
                    LaunchConfiguration("release_control_on_shutdown"),
                    value_type=bool,
                ),
                "control_nick_name": LaunchConfiguration("control_nick_name"),
                "default_source_id": LaunchConfiguration("default_source_id"),
                "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "bms_topic": LaunchConfiguration("bms_topic"),
                "go_to_lm_topic": LaunchConfiguration("go_to_lm_topic"),
                "navigate_status_topic": LaunchConfiguration("navigate_status_topic"),
            }
        ],
    )

    return LaunchDescription(arguments + [driver_node])
