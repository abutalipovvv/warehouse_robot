from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


DEFAULT_PARAMS_PATH = Path(__file__).resolve().parents[1] / "params.yaml"

DEFAULT_ROUTE_PARAMS: dict[str, Any] = {
    "robot_model": {
        "footprint": [
            {"x": 0.35, "y": 0.275},
            {"x": 0.35, "y": -0.275},
            {"x": -0.35, "y": -0.275},
            {"x": -0.35, "y": 0.275},
        ],
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
        "cbs_low_level_max_time": 160,
        "cbs_max_high_level_nodes": 2000,
        "cbs_max_planning_time_sec": 5.00,
    },
}


def load_route_params(path: Path | None = None, create: bool = False) -> dict[str, Any]:
    params_path = path or DEFAULT_PARAMS_PATH
    if not params_path.exists():
        params = deepcopy(DEFAULT_ROUTE_PARAMS)
        if create:
            save_route_params(params, params_path)
        return params

    loaded = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        loaded = {}
    return _deep_merge(deepcopy(DEFAULT_ROUTE_PARAMS), loaded)


def save_route_params(params: dict[str, Any], path: Path | None = None) -> Path:
    params_path = path or DEFAULT_PARAMS_PATH
    params_path.parent.mkdir(parents=True, exist_ok=True)
    merged = _deep_merge(deepcopy(DEFAULT_ROUTE_PARAMS), params)
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
