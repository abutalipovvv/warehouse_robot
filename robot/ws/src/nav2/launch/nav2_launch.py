#!/usr/bin/env python3

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _project_root(pkg_path: str) -> Path:
    share_dir = Path(pkg_path)
    for parent in share_dir.parents:
        if (parent / "map_data").exists():
            return parent
    return share_dir.parents[4]


def _default_map_yaml(pkg_path: str) -> str:
    project_root = _project_root(pkg_path)
    fallback_dir = project_root / "map_data" / "maps_out" / "22.05.26_smap.smap"
    state_file = project_root / "robot" / ".active_map.json"
    map_dir = fallback_dir
    if state_file.exists():
        try:
            payload = json.loads(state_file.read_text(encoding="utf-8"))
            candidate = Path(str(payload.get("mapDir") or "")).expanduser()
            if candidate.is_dir():
                map_dir = candidate
        except (OSError, json.JSONDecodeError):
            pass
    candidates = sorted(
        path
        for path in map_dir.glob("*.yaml")
        if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
    )
    if candidates:
        return str(candidates[0])
    return os.path.join(pkg_path, "maps", "22.05.26_smap.yaml")



def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_rviz = LaunchConfiguration("enable_rviz")
    pkg_path = get_package_share_directory("nav2")

    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="enable_rviz",
            default_value="true",
            description="Enable rviz launch",
        )
    )


    nav2_launch_file = os.path.join(pkg_path, "launch", "nav2", "bringup_launch.py")
    map_yaml_file = _default_map_yaml(pkg_path)
    params_file = os.path.join(pkg_path, "config", "nav2_params.yaml")

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "map": map_yaml_file,
            "use_namespace": "False",
            "params_file": params_file,
            "autostart": "true",
            "use_sim_time": use_sim_time,
            "log_level": "warn",
            "map_server": "True",
        }.items(),
    )

    rviz_launch_file = os.path.join(pkg_path, "launch", "rviz_launch.py")
    rviz_config_file = os.path.join(pkg_path, "rviz", "nav2_default_view.rviz")
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        launch_arguments={
            "use_namespace": "false",
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
        condition=IfCondition(enable_rviz),
    )

    ld.add_action(bringup_cmd)
    ld.add_action(rviz)
    return ld
