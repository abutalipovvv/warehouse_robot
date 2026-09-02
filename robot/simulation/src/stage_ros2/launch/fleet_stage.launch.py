#!/usr/bin/env python3

from dataclasses import replace
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from stage_ros2.fleet_world import (
    apply_domain_override,
    load_fleet_definition,
    robot_domain_map,
    write_generated_world,
)


_GENERATED_WORLDS: set[Path] = set()


def _configure_fleet(context):
    package_share = Path(get_package_share_directory('stage_ros2'))
    config_path = LaunchConfiguration('fleet_config').perform(context)
    fleet = load_fleet_definition(config_path)
    robots = list(fleet.robots)
    robots = apply_domain_override(
        robots, LaunchConfiguration('robot_domain_map').perform(context)
    )
    fleet = replace(fleet, robots=tuple(robots))
    generated_world = write_generated_world(
        package_share / 'world' / 'fleet_generated.world.in',
        fleet,
        trp1_include=package_share / 'world' / 'include' / 'trp1.inc',
    )
    _GENERATED_WORLDS.add(generated_world)
    domains = robot_domain_map(robots)

    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'stage.launch.py')
        ),
        launch_arguments={
            'world_file': str(generated_world),
            'robot_domain_map': domains,
            'enable_gui': LaunchConfiguration('enable_gui'),
            'publish_imu': LaunchConfiguration('publish_imu'),
            'use_imu_for_odom_yaw': LaunchConfiguration(
                'use_imu_for_odom_yaw'
            ),
            'enforce_prefixes': 'false',
            'one_tf_tree': 'false',
            'fake_bms': 'false',
            'publish_robot_state': 'false',
        }.items(),
    )
    identities = ', '.join(
        f'{robot.robot_id}@{robot.lm}={robot.ip}:domain{robot.ros_domain_id}'
        for robot in robots
    )
    return [
        LogInfo(
            msg=(
                f'[fleet_stage] world={fleet.world} SMAP={fleet.smap.smap_path}; '
                f'generated one Stage world with {len(robots)} robots: {identities}'
            )
        ),
        stage_launch,
    ]


def _cleanup_generated_world(context):
    del context
    for path in tuple(_GENERATED_WORLDS):
        try:
            path.unlink(missing_ok=True)
        finally:
            _GENERATED_WORLDS.discard(path)
    return []


def generate_launch_description() -> LaunchDescription:
    """Generate one Stage world with one namespace-free ROS domain per robot."""
    package_share = get_package_share_directory('stage_ros2')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'fleet_config',
                default_value=str(Path(package_share) / 'config' / 'fleet.yaml'),
                description=(
                    'Fleet YAML containing world, SMAP path and robot '
                    'robot_id/domain/IP/LM definitions.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_domain_map',
                default_value='',
                description=(
                    'Optional robot_id=domain_id override. It must contain '
                    'exactly the same robot IDs as fleet_config.'
                ),
            ),
            DeclareLaunchArgument('enable_gui', default_value='false'),
            DeclareLaunchArgument('publish_imu', default_value='true'),
            DeclareLaunchArgument(
                'use_imu_for_odom_yaw', default_value='true'
            ),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(function=_cleanup_generated_world)
                    ]
                )
            ),
            OpaqueFunction(function=_configure_fleet),
        ]
    )
