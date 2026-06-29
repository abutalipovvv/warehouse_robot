from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.192.5"),
        DeclareLaunchArgument("robot_id", default_value="robot1"),
        DeclareLaunchArgument("robot_api_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("robot_api_port", default_value="50051"),
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
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("status_topic", default_value="/robot_status"),
        DeclareLaunchArgument("go_to_lm_topic", default_value="/go_to_lm"),
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
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "go_to_lm_topic": LaunchConfiguration("go_to_lm_topic"),
            }
        ],
    )

    robot_api_node = Node(
        package="real_robot",
        executable="robot_api_server",
        name="robot_api",
        output="screen",
        arguments=[
            "--host",
            LaunchConfiguration("robot_api_host"),
            "--port",
            LaunchConfiguration("robot_api_port"),
            "--robot-id",
            LaunchConfiguration("robot_id"),
            "--robot-name",
            LaunchConfiguration("robot_id"),
            "--status-topic",
            LaunchConfiguration("status_topic"),
            "--cmd-vel-topic",
            LaunchConfiguration("cmd_vel_topic"),
            "--go-to-lm-topic",
            LaunchConfiguration("go_to_lm_topic"),
        ],
    )

    return LaunchDescription(arguments + [driver_node, robot_api_node])
