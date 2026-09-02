#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    this_directory = get_package_share_directory('stage_ros2')

    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='22.05.26_smap'),
        description='World file relative to the project world file, without .world'
    )

    explicit_world_file = LaunchConfiguration('world_file')
    explicit_world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description=(
            'Absolute Stage world path. When set, it takes precedence over world.'
        )
    )

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment'
    )

    use_static_transformations = LaunchConfiguration('use_static_transformations')
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames!'
    )

    enable_gui = LaunchConfiguration('enable_gui')
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='Run Stage with its GUI window enabled'
    )

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static'
    )

    fake_bms = LaunchConfiguration('fake_bms')
    fake_bms_arg = DeclareLaunchArgument(
        'fake_bms',
        default_value='true',
        description=(
            'Publish fake sensor_msgs/BatteryState on /bms for '
            'teleworker_status in simulation'
        )
    )

    publish_imu = LaunchConfiguration('publish_imu')
    publish_imu_arg = DeclareLaunchArgument(
        'publish_imu',
        default_value='true',
        description='Publish simulated IMU on /imu'
    )

    use_imu_for_odom_yaw = LaunchConfiguration('use_imu_for_odom_yaw')
    use_imu_for_odom_yaw_arg = DeclareLaunchArgument(
        'use_imu_for_odom_yaw',
        default_value='true',
        description='Use simulated IMU yaw and gyro to stabilize Stage odometry'
    )

    robot_domain_map = LaunchConfiguration('robot_domain_map')
    robot_domain_map_arg = DeclareLaunchArgument(
        'robot_domain_map',
        default_value='',
        description=(
            'Comma-separated robot_id=ROS_DOMAIN_ID mapping. When set, each '
            'Stage model publishes unprefixed topics in its own ROS domain.'
        )
    )

    publish_robot_state = LaunchConfiguration('publish_robot_state')
    publish_robot_state_arg = DeclareLaunchArgument(
        'publish_robot_state',
        default_value='true',
        description=(
            'Start one robot_state_publisher in the Stage process domain. '
            'Disable this for robot_domain_map mode; each robot runtime must '
            'start its own robot_state_publisher.'
        )
    )

    def stage_world_configuration(context):
        file = explicit_world_file.perform(context).strip()
        if file:
            file = os.path.abspath(os.path.expanduser(file))
        else:
            file = os.path.join(
                this_directory,
                'world',
                context.launch_configurations['world'] + '.world'
            )
        return [SetLaunchConfiguration('resolved_world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('ecom_mobile_robot_description'),
                'urdf',
                'ecom_stage.urdf.xacro'
            ])
        ]),
        value_type=str
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
        condition=IfCondition(publish_robot_state),
    )
    return LaunchDescription([
        stage_world_arg,
        explicit_world_file_arg,
        one_tf_tree_arg,
        enforce_prefixes_arg,
        enable_gui_arg,
        use_static_transformations_arg,
        fake_bms_arg,
        publish_imu_arg,
        use_imu_for_odom_yaw_arg,
        robot_domain_map_arg,
        publish_robot_state_arg,
        stage_world_configuration_arg,

        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{
                'one_tf_tree': one_tf_tree,
                'enforce_prefixes': enforce_prefixes,
                'enable_gui': enable_gui,
                'use_static_transformations': use_static_transformations,
                'world_file': [LaunchConfiguration('resolved_world_file')],
                'publish_imu': publish_imu,
                'use_imu_for_odom_yaw': use_imu_for_odom_yaw,
                'robot_domain_map': robot_domain_map,
            }],
        ),

        rsp_node,

        ExecuteProcess(
            cmd=[
                sys.executable,
                '-m',
                'stage_ros2.fake_bms_publisher',
            ],
            name='fake_bms_pub',
            output='log',
            condition=IfCondition(fake_bms),
        )
    ])
