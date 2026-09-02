#!/usr/bin/env python3
import json
import os
from pathlib import Path

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


def _default_map_paths() -> tuple[str, str, str, str]:
    """Mirror the legacy active-map defaults while allowing fleet overrides."""
    share_dir = Path(get_package_share_directory("robot_launch")).resolve()
    candidates: list[Path] = []
    for parent in share_dir.parents:
        candidates.extend(
            (
                parent / "src" / "robot_map_manager" / "maps_out",
                parent
                / "robot"
                / "robot_driver"
                / "src"
                / "robot_map_manager"
                / "maps_out",
            )
        )
    maps_root = next((path.resolve() for path in candidates if path.is_dir()), None)
    if maps_root is None:
        maps_root = candidates[0].resolve()
    state_file = (maps_root.parent / ".active_map.json").resolve()
    map_dir = (maps_root / "22.05.26_smap.smap").resolve()
    if state_file.is_file():
        try:
            payload = json.loads(state_file.read_text(encoding="utf-8"))
            raw_path = Path(str(payload.get("mapDir") or "")).expanduser()
            candidate = raw_path if raw_path.is_absolute() else maps_root / raw_path
            if candidate.is_dir():
                map_dir = candidate.resolve()
        except (OSError, json.JSONDecodeError):
            pass
    excluded = {
        "LMs.yaml",
        "graphs.yaml",
        "graph_edges_lengths.yaml",
        "traffic_zones.yaml",
    }
    map_yamls = sorted(
        path for path in map_dir.glob("*.yaml") if path.name not in excluded
    )
    map_yaml = map_yamls[0] if map_yamls else map_dir / "map.yaml"
    return str(map_dir), str(map_yaml.resolve()), str(maps_root), str(state_file)


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
    default_map_dir, default_map_yaml, default_maps_root, default_state_file = (
        _default_map_paths()
    )
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
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    map_dir = LaunchConfiguration("map_dir")
    map_yaml = LaunchConfiguration("map_yaml")
    maps_root = LaunchConfiguration("maps_root")
    state_file = LaunchConfiguration("state_file")

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
        DeclareLaunchArgument("initial_pose_x", default_value="-4.902"),
        DeclareLaunchArgument("initial_pose_y", default_value="1.362"),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="3.141592653589793",
        ),
        DeclareLaunchArgument("map_dir", default_value=default_map_dir),
        DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
        DeclareLaunchArgument("maps_root", default_value=default_maps_root),
        DeclareLaunchArgument("state_file", default_value=default_state_file),
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
            "map": map_yaml,
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_yaw": initial_pose_yaw,
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
            "map_dir": map_dir,
            "maps_root": maps_root,
            "state_file": state_file,
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
