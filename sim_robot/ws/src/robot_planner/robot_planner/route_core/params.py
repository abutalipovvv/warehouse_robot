from __future__ import annotations

import ast
import math
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def discover_default_params_path() -> Path:
    module_path = Path(__file__).resolve()
    for parent in module_path.parents:
        ros_candidate = parent / "params.yaml"
        if (parent / "robot_planner").exists() and (parent / "robot_map_manager").exists():
            return ros_candidate
        if ros_candidate.exists() and (parent / "robot_map_manager").exists():
            return ros_candidate

    cwd_candidate = Path.cwd() / "params.yaml"
    if cwd_candidate.exists():
        return cwd_candidate.resolve()
    if len(module_path.parents) > 3:
        return module_path.parents[3] / "params.yaml"
    return module_path.parents[1] / "params.yaml"


DEFAULT_PARAMS_PATH = discover_default_params_path()
DEFAULT_NAV2_ROBOT_RADIUS = 0.22
DEFAULT_NAV2_FOOTPRINT_SEGMENTS = 16


def _circle_footprint(radius: float, segments: int = DEFAULT_NAV2_FOOTPRINT_SEGMENTS) -> list[dict[str, float]]:
    count = max(8, int(segments or DEFAULT_NAV2_FOOTPRINT_SEGMENTS))
    safe_radius = max(0.01, float(radius or DEFAULT_NAV2_ROBOT_RADIUS))
    return [
        {
            "x": round(math.cos((2.0 * math.pi * index) / count) * safe_radius, 6),
            "y": round(math.sin((2.0 * math.pi * index) / count) * safe_radius, 6),
        }
        for index in range(count)
    ]


def _nav2_costmap_params(nav2_params: dict[str, Any], costmap_name: str) -> dict[str, Any]:
    root = nav2_params.get(costmap_name)
    if not isinstance(root, dict):
        return {}
    nested = root.get(costmap_name)
    if isinstance(nested, dict) and isinstance(nested.get("ros__parameters"), dict):
        return nested["ros__parameters"]
    if isinstance(root.get("ros__parameters"), dict):
        return root["ros__parameters"]
    return {}


def _parse_nav2_footprint(raw_footprint: Any) -> list[dict[str, float]]:
    if isinstance(raw_footprint, str):
        try:
            raw_footprint = ast.literal_eval(raw_footprint.strip())
        except (SyntaxError, ValueError):
            return []
    if not isinstance(raw_footprint, list):
        return []
    footprint: list[dict[str, float]] = []
    for item in raw_footprint:
        if isinstance(item, dict):
            point = item
            if "x" not in point or "y" not in point:
                continue
            footprint.append({"x": float(point["x"]), "y": float(point["y"])})
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            footprint.append({"x": float(item[0]), "y": float(item[1])})
    return footprint if len(footprint) >= 3 else []


def _resolve_nav2_params_path(robot_model: dict[str, Any], params_path: Path) -> Path | None:
    raw_path = robot_model.get("nav2_params_path") or robot_model.get("nav2ParamsPath")
    candidates: list[Path] = []
    if raw_path:
        candidate = Path(str(raw_path)).expanduser()
        candidates.append(candidate if candidate.is_absolute() else params_path.parent / candidate)
    for parent in (params_path.parent, *params_path.parents):
        candidates.append(parent / "nav2" / "config" / "nav2_params.yaml")
        candidates.append(parent / "robot" / "ws" / "src" / "nav2" / "config" / "nav2_params.yaml")
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_file():
            return resolved
    return None


def _apply_nav2_robot_model(params: dict[str, Any], params_path: Path) -> dict[str, Any]:
    robot_model = params.setdefault("robot_model", {})
    if not isinstance(robot_model, dict):
        return params
    source = str(robot_model.get("source") or "").strip().lower()
    if source not in {"nav2", "nav2_params", "nav2_costmap"}:
        return params
    nav2_path = _resolve_nav2_params_path(robot_model, params_path)
    if nav2_path is None:
        return params
    loaded = yaml.safe_load(nav2_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        return params

    segments = int(robot_model.get("footprint_segments", DEFAULT_NAV2_FOOTPRINT_SEGMENTS) or DEFAULT_NAV2_FOOTPRINT_SEGMENTS)
    for costmap_name in ("local_costmap", "global_costmap"):
        costmap_params = _nav2_costmap_params(loaded, costmap_name)
        footprint = _parse_nav2_footprint(costmap_params.get("footprint"))
        if footprint:
            robot_model["footprint"] = footprint
            robot_model["nav2_costmap"] = costmap_name
            robot_model["nav2_params_path"] = str(nav2_path)
            return params

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap_params = _nav2_costmap_params(loaded, costmap_name)
        if "robot_radius" not in costmap_params:
            continue
        radius = float(costmap_params["robot_radius"])
        robot_model["radius"] = radius
        robot_model["footprint"] = _circle_footprint(radius, segments)
        robot_model["nav2_costmap"] = costmap_name
        robot_model["nav2_params_path"] = str(nav2_path)
        return params
    return params

DEFAULT_ROUTE_PARAMS: dict[str, Any] = {
    "robot_model": {
        "source": "nav2",
        "radius": DEFAULT_NAV2_ROBOT_RADIUS,
        "footprint_segments": DEFAULT_NAV2_FOOTPRINT_SEGMENTS,
        "footprint": _circle_footprint(DEFAULT_NAV2_ROBOT_RADIUS),
        "frames": {
            "lidar": {"x": 0.28, "y": 0.0, "label": "LiDAR", "color": "#1f6feb"},
            "imu": {"x": 0.0, "y": 0.0, "label": "IMU", "color": "#d95521"},
            "wheel_left": {"x": 0.0, "y": 0.225, "label": "WL", "color": "#2f3a4a"},
            "wheel_right": {"x": 0.0, "y": -0.225, "label": "WR", "color": "#2f3a4a"},
        },
    },
    "navigation": {
        "route_speed": 0.35,
        "footprint_lookahead": 0.80,
        "collision_margin": 0.04,
        "stop_distance": 0.40,
        "angular_gain": 2.20,
        "max_angular_speed": 0.90,
        "rotate_in_place_angle_deg": 32.0,
        "curve_speed_limit": 0.25,
        "rejoin_speed_limit": 0.16,
        "hard_rejoin_speed_limit": 0.06,
    },
    "planner": {
        "nearest_lm_tolerance": 0.05,
        "trajectory_sample_distance": 0.05,
        "on_route_tolerance": 0.12,
        "precision_start_distance": 0.10,
    },
    "localization": {
        "pose_source": "AMCL + IMU-aided odom",
        "localization_timeout": 0.50,
        "amcl_correction_timeout": 5.00,
        "allowed_lateral_error": 0.02,
        "allowed_yaw_error_deg": 1.0,
        "accept_stale_pose_when_stationary": True,
        "stationary_linear_velocity_epsilon": 0.02,
        "stationary_angular_velocity_epsilon": 0.05,
    },
    "manual": {
        "linear_speed": 0.25,
        "angular_speed": 0.90,
        "prediction_time": 1.00,
        "prediction_step": 0.10,
    },
    "fleet": {
        "reservation_time_step_sec": 1.00,
        "reservation_horizon_sec": 8.00,
        "reservation_safety_time_sec": 0.35,
        "robot_clearance_m": 0.35,
        "continuous_collision_step_sec": 0.10,
        "wait_time_sec": 1.00,
        "wait_cost": 6,
        "replan_interval_sec": 1.00,
        "remote_route_chunk_lms": 5,
        "cbs_low_level_max_time": 160,
        "cbs_max_high_level_nodes": 2000,
        "cbs_max_planning_time_sec": 5.00,
    },
}

DEFAULT_ROUTE_PARAMS.pop("fleet", None)


def load_route_params(
    path: Path | None = None,
    create: bool = False,
    defaults: dict[str, Any] | None = None,
) -> dict[str, Any]:
    params_path = path or DEFAULT_PARAMS_PATH
    default_params = DEFAULT_ROUTE_PARAMS if defaults is None else defaults
    if not params_path.exists():
        params = deepcopy(default_params)
        params = _apply_nav2_robot_model(params, params_path)
        if create:
            save_route_params(params, params_path, defaults=default_params)
        return params

    loaded = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        loaded = {}
    return _apply_nav2_robot_model(_deep_merge(deepcopy(default_params), loaded), params_path)


def save_route_params(
    params: dict[str, Any],
    path: Path | None = None,
    defaults: dict[str, Any] | None = None,
) -> Path:
    params_path = path or DEFAULT_PARAMS_PATH
    default_params = DEFAULT_ROUTE_PARAMS if defaults is None else defaults
    params_path.parent.mkdir(parents=True, exist_ok=True)
    merged = _deep_merge(deepcopy(default_params), params)
    params_path.write_text(
        yaml.safe_dump(merged, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return params_path


def _deep_merge(defaults: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(defaults.get(key), dict):
            defaults[key] = _deep_merge(defaults[key], value)
        else:
            defaults[key] = value
    return defaults
