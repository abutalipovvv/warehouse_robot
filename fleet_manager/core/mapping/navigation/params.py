from __future__ import annotations

import ast
from enum import Enum
import logging
import math
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml

from fleet_manager.storage import atomic_write_text


DEFAULT_PARAMS_PATH = (
    Path(__file__).resolve().parents[3] / "config" / "params.yaml"
)
DEFAULT_NAV2_ROBOT_RADIUS = 0.22
DEFAULT_NAV2_FOOTPRINT_SEGMENTS = 16
LOGGER = logging.getLogger(__name__)


class ConfigurationMode(str, Enum):
    COMPATIBILITY = "compatibility"
    STRICT = "strict"


class ConfigurationError(ValueError):
    """A configuration value is invalid in strict mode."""


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
    "strict_configuration": False,
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
        "rolling_target_buffer_sec": 75.0,
        "rolling_refill_threshold_sec": 55.0,
        "rolling_urgent_threshold_sec": 30.0,
        "rolling_critical_threshold_sec": 15.0,
        "rolling_emergency_threshold_sec": 5.0,
        "rolling_max_prepared_buffer_sec": 150.0,
        "rolling_refill_stagger_window_sec": 8.0,
        "rolling_normal_batch_size": 12,
        "planning_queue_max_size": 2,
        "rolling_horizon_scale_with_simulation_time": True,
        "rolling_horizon_max_sec": 120.0,
        "reservation_safety_time_sec": 0.35,
        "robot_clearance_m": 0.35,
        "continuous_collision_step_sec": 0.10,
        "wait_time_sec": 1.00,
        "wait_cost": 6,
        "controlled_corridors_enabled": True,
        "controlled_corridor_auto_detect": False,
        "controlled_corridor_min_edges": 1,
        "controlled_corridor_schedule_horizon_sec": 120.0,
        "controlled_corridor_commit_horizon_sec": 2.0,
        "controlled_corridor_slot_headway_sec": 1.0,
        "controlled_corridor_direction_change_sec": 0.9,
        "controlled_corridor_direction_switch_cost_sec": 1.5,
        "controlled_corridor_schedule_hysteresis_sec": 2.0,
        "controlled_corridor_occupancy_recheck_sec": 0.1,
        "controlled_corridor_priority_cost_sec": 0.05,
        "controlled_corridor_wait_age_cost_sec": 0.03,
        "controlled_corridor_starvation_sec": 8.0,
        "controlled_corridor_starvation_age_quantum_sec": 2.0,
        "controlled_corridor_max_direction_batch": 3,
        "controlled_corridor_max_adaptive_direction_batch": 12,
        "controlled_corridor_phase_amortization_sec": 4.0,
        "controlled_corridor_max_phase_extension_sec": 30.0,
        "controlled_corridor_direct_transition_penalty_m": 4.0,
        "stationary_recovery_retry_sec": 4.0,
        # After repeated no-detour failures, an inactive route-less robot may
        # receive a hidden ordinary MAPF task to a nearby safe holding pocket.
        "parked_clearance_relocation_enabled": True,
        "parked_clearance_relocation_failures": 2,
        "parked_clearance_relocation_max_hops": 8,
        "parked_clearance_relocation_cooldown_sec": 12.0,
        "parked_clearance_relocation_timeout_sec": 120.0,
        "deadlock_recovery_cooldown_sec": 4.0,
        "replan_interval_sec": 1.00,
        # Zero sends the complete graph route in one robot-side contract.
        "remote_route_chunk_lms": 0,
        "cbs_low_level_max_time": 160,
        "cbs_max_high_level_nodes": 2000,
        "cbs_max_planning_time_sec": 5.00,
    },
}


def load_route_params(
    path: Path | None = None,
    create: bool = False,
    defaults: dict[str, Any] | None = None,
    *,
    strict: bool = False,
) -> dict[str, Any]:
    params_path = path or DEFAULT_PARAMS_PATH
    default_params = DEFAULT_ROUTE_PARAMS if defaults is None else defaults
    if not params_path.exists():
        params = deepcopy(default_params)
        params["strict_configuration"] = bool(strict)
        params = _apply_nav2_robot_model(params, params_path)
        if create:
            save_route_params(params, params_path, defaults=default_params)
        return params

    loaded = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        if strict:
            raise ConfigurationError(
                f"configuration root: expected mapping, received {loaded!r}"
            )
        loaded = {}
    schema = _configuration_schema(default_params, include_packaged=defaults is None)
    _validate_config_mapping(
        loaded,
        schema,
        path="configuration",
        strict=strict,
    )
    merged = _deep_merge(deepcopy(default_params), loaded)
    merged["strict_configuration"] = bool(strict)
    _validate_config_consistency(merged, strict=strict)
    return _apply_nav2_robot_model(merged, params_path)


def save_route_params(
    params: dict[str, Any],
    path: Path | None = None,
    defaults: dict[str, Any] | None = None,
) -> Path:
    params_path = path or DEFAULT_PARAMS_PATH
    default_params = DEFAULT_ROUTE_PARAMS if defaults is None else defaults
    merged = _deep_merge(deepcopy(default_params), params)
    atomic_write_text(
        params_path,
        yaml.safe_dump(merged, allow_unicode=True, sort_keys=False),
    )
    return params_path


def _deep_merge(defaults: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(defaults.get(key), dict):
            defaults[key] = _deep_merge(defaults[key], value)
        else:
            defaults[key] = value
    return defaults


def _configuration_schema(
    defaults: dict[str, Any],
    *,
    include_packaged: bool,
) -> dict[str, Any]:
    schema = deepcopy(defaults)
    if not include_packaged or not DEFAULT_PARAMS_PATH.is_file():
        return schema
    packaged = yaml.safe_load(DEFAULT_PARAMS_PATH.read_text(encoding="utf-8"))
    if isinstance(packaged, dict):
        _deep_merge(schema, packaged)
    return schema


def _validate_config_mapping(
    values: dict[str, Any],
    schema: dict[str, Any],
    *,
    path: str,
    strict: bool,
) -> None:
    for key, value in values.items():
        value_path = f"{path}.{key}"
        if key not in schema:
            message = f"{value_path}: unknown key, received {value!r}"
            if strict:
                raise ConfigurationError(message)
            LOGGER.warning("configuration_compatibility: %s", message)
            continue
        expected = schema[key]
        if isinstance(expected, dict):
            if not isinstance(value, dict):
                _config_invalid(
                    value_path,
                    value,
                    "mapping",
                    strict=strict,
                )
                continue
            _validate_config_mapping(
                value,
                expected,
                path=value_path,
                strict=strict,
            )
            continue
        _validate_config_scalar(
            value_path,
            value,
            expected,
            strict=strict,
        )


def _validate_config_scalar(
    path: str,
    value: Any,
    expected: Any,
    *,
    strict: bool,
) -> None:
    if expected is None:
        return
    if isinstance(expected, bool):
        valid = isinstance(value, bool)
        expected_name = "boolean"
    elif isinstance(expected, (int, float)):
        valid = (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value))
        )
        expected_name = "finite number"
    elif isinstance(expected, str):
        valid = isinstance(value, str)
        expected_name = "string"
    elif isinstance(expected, list):
        valid = isinstance(value, list)
        expected_name = "list"
    else:
        valid = isinstance(value, type(expected))
        expected_name = type(expected).__name__
    if not valid:
        _config_invalid(
            path,
            value,
            expected_name,
            strict=strict,
        )


def _config_invalid(
    path: str,
    value: Any,
    expected: str,
    *,
    strict: bool,
) -> None:
    message = f"{path}: expected {expected}, received {value!r}"
    if strict:
        raise ConfigurationError(message)
    LOGGER.warning("configuration_compatibility: %s", message)


def _validate_config_consistency(
    params: dict[str, Any],
    *,
    strict: bool,
) -> None:
    if not strict:
        return
    fleet = params.get("fleet", {})
    if not isinstance(fleet, dict):
        return
    backend = str(fleet.get("planner_backend", "cbs") or "").strip().lower()
    allowed_backends = {
        "cbs",
        "rolling-sipp",
        "rolling_sipp",
        "sipp",
        "hybrid",
        "rolling_sipp+cbs",
        "sipp+cbs",
    }
    if backend not in allowed_backends:
        raise ConfigurationError(
            "configuration.fleet.planner_backend: unknown backend, "
            f"received {fleet.get('planner_backend')!r}"
        )
    positive = (
        "reservation_time_step_sec",
        "reservation_horizon_sec",
        "cbs_low_level_max_time",
        "cbs_max_high_level_nodes",
        "cbs_max_planning_time_sec",
        "rolling_target_buffer_sec",
        "rolling_refill_threshold_sec",
        "rolling_urgent_threshold_sec",
        "rolling_critical_threshold_sec",
        "rolling_emergency_threshold_sec",
        "rolling_max_prepared_buffer_sec",
        "planning_queue_max_size",
    )
    for key in positive:
        if key in fleet and float(fleet[key]) <= 0.0:
            raise ConfigurationError(
                f"configuration.fleet.{key}: expected value > 0, "
                f"received {fleet[key]!r}"
            )
    queue_size = fleet.get("planning_queue_max_size")
    if queue_size is not None and (
        float(queue_size) != int(float(queue_size))
        or not 1 <= int(float(queue_size)) <= 8
    ):
        raise ConfigurationError(
            "configuration.fleet.planning_queue_max_size: expected integer "
            f"from 1 to 8, received {queue_size!r}"
        )
    rolling_keys = (
        "rolling_emergency_threshold_sec",
        "rolling_critical_threshold_sec",
        "rolling_urgent_threshold_sec",
        "rolling_refill_threshold_sec",
        "rolling_target_buffer_sec",
        "rolling_max_prepared_buffer_sec",
    )
    if all(key in fleet for key in rolling_keys):
        rolling_values = tuple(float(fleet[key]) for key in rolling_keys)
        ordered = all(
            current < following
            for current, following in zip(
                rolling_values[:-2],
                rolling_values[1:-1],
            )
        )
        ordered = ordered and rolling_values[-2] <= rolling_values[-1]
        if not ordered:
            raise ConfigurationError(
                "configuration.fleet rolling thresholds: expected "
                "emergency < critical < urgent < refill < target <= max, "
                f"received {rolling_values!r}"
            )
    batch = fleet.get("controlled_corridor_max_direction_batch")
    adaptive = fleet.get("controlled_corridor_max_adaptive_direction_batch")
    if batch is not None and adaptive is not None and int(adaptive) < int(batch):
        raise ConfigurationError(
            "configuration.fleet.controlled_corridor_max_adaptive_direction_batch: "
            "must be >= controlled_corridor_max_direction_batch, "
            f"received {adaptive!r}"
        )
