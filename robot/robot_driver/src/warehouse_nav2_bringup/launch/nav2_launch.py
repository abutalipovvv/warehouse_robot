#!/usr/bin/env python3

from __future__ import annotations

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _project_root(package_share: str) -> Path:
    share_dir = Path(package_share).resolve()
    for parent in share_dir.parents:
        if (parent / "src" / "robot_map_manager" / "maps_out").is_dir():
            return parent.resolve()
        driver_root = parent / "robot" / "robot_driver"
        if (driver_root / "src" / "robot_map_manager" / "maps_out").is_dir():
            return driver_root.resolve()
    return share_dir.parents[3].resolve()


def _default_map_yaml(package_share: str) -> str:
    project_root = _project_root(package_share)
    roots = [
        project_root / "src" / "robot_map_manager" / "maps_out",
        project_root / "maps_out",
    ]
    maps_root = next((item for item in roots if item.is_dir()), roots[0])
    map_dir = maps_root / "22.05.26_smap.smap"
    for state_file in (maps_root.parent / ".active_map.json",):
        if not state_file.is_file():
            continue
        try:
            payload = json.loads(state_file.read_text(encoding="utf-8"))
            raw_candidate = Path(str(payload.get("mapDir") or "")).expanduser()
            candidate = (
                raw_candidate
                if raw_candidate.is_absolute()
                else maps_root / raw_candidate
            )
            if candidate.is_dir():
                map_dir = candidate.resolve()
                break
            map_name = str(
                payload.get("mapName") or payload.get("mapId") or ""
            ).strip()
            named_map = maps_root / f"{map_name.removesuffix('.smap')}.smap"
            if map_name and named_map.is_dir():
                map_dir = named_map.resolve()
                break
        except (OSError, json.JSONDecodeError):
            continue
    excluded = {
        "LMs.yaml",
        "graphs.yaml",
        "graph_edges_lengths.yaml",
        "traffic_zones.yaml",
    }
    candidates = sorted(
        path for path in map_dir.glob("*.yaml") if path.name not in excluded
    )
    if not candidates:
        raise FileNotFoundError(f"No ROS map yaml found in {map_dir}")
    return str(candidates[0])


def _validate_profile(context):
    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    if profile not in {"localization", "navigation"}:
        raise RuntimeError("Nav2 profile must be localization or navigation")
    return []


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("warehouse_nav2_bringup")
    launch_dir = os.path.join(package_share, "launch")
    params_file = os.path.join(package_share, "config", "nav2_params.yaml")
    rviz_file = os.path.join(package_share, "rviz", "nav2_default_view.rviz")
    profile = LaunchConfiguration("profile")
    common_arguments = {
        "namespace": LaunchConfiguration("namespace"),
        "map": LaunchConfiguration("map"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "autostart": LaunchConfiguration("autostart"),
        "params_file": LaunchConfiguration("params_file"),
        "log_level": LaunchConfiguration("log_level"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="localization"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("map", default_value=_default_map_yaml(package_share)),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument("params_file", default_value=params_file),
            DeclareLaunchArgument(
                "nav2_cmd_vel_topic",
                default_value="motion/nav2_cmd_vel",
            ),
            OpaqueFunction(function=_validate_profile),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization.launch.py")
                ),
                launch_arguments=common_arguments.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation.launch.py")
                ),
                launch_arguments={
                    **common_arguments,
                    "nav2_cmd_vel_topic": LaunchConfiguration(
                        "nav2_cmd_vel_topic"
                    ),
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", profile, "' == 'navigation'"])
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=LaunchConfiguration("namespace"),
                name="nav2_rviz",
                output="screen",
                arguments=["-d", rviz_file],
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
