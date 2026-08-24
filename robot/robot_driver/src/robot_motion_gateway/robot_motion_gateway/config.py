"""Load motion gateway limits from the shared robot parameter file."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .model import MotionLimits, MotionTimeouts


@dataclass(frozen=True, slots=True)
class GatewaySettings:
    limits: MotionLimits
    timeouts: MotionTimeouts
    publish_rate_hz: float


def load_gateway_settings(path: Path | None) -> GatewaySettings:
    payload = _yaml_payload(path)
    gateway = _mapping(payload.get("motion_gateway"))
    manual = _mapping(payload.get("manual"))
    navigation = _mapping(payload.get("navigation"))
    trajectory = _mapping(navigation.get("trajectory_speed_profile"))

    manual_linear = _number(manual.get("linear_speed"), 1.3)
    return GatewaySettings(
        limits=MotionLimits(
            max_forward_speed=_number(
                gateway.get("max_forward_speed"),
                max(
                    manual_linear,
                    _number(trajectory.get("max_forward_speed"), 0.8),
                ),
            ),
            max_backward_speed=_number(
                gateway.get("max_backward_speed"),
                max(
                    manual_linear,
                    _number(trajectory.get("max_backward_speed"), 0.3),
                ),
            ),
            max_angular_speed=_number(
                gateway.get("max_angular_speed"),
                max(
                    _number(manual.get("angular_speed"), 1.8),
                    _number(navigation.get("max_angular_speed"), 0.9),
                ),
            ),
        ).normalized(),
        timeouts=MotionTimeouts(
            route=_number(gateway.get("route_timeout_sec"), 0.25),
            teleop=_number(gateway.get("teleop_timeout_sec"), 0.45),
            nav2=_number(gateway.get("nav2_timeout_sec"), 0.30),
        ),
        publish_rate_hz=min(
            100.0,
            max(10.0, _number(gateway.get("publish_rate_hz"), 50.0)),
        ),
    )


def _yaml_payload(path: Path | None) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _mapping(value: object) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _number(value: object, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback
