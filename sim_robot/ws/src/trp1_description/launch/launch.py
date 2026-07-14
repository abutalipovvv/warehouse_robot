#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Compatibility entry point: existing launch commands keep working while
    # the actual description is now owned by the Ecom package.
    robot_description = ParameterValue(Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('ecom_mobile_robot_description'),
            'urdf',
            'ecom_stage.urdf.xacro'
        ])
    ]), value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        rsp_node,
    ])
