from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True, slots=True)
class SpeedProfileParameters:
    """Limits for deterministic robot-local linear velocity profiling."""

    control_period: float
    max_acceleration: float
    max_deceleration: float
    precision_start_distance: float
    precision_speed_limit: float
    precision_min_speed: float
    precision_linear_gain: float
    goal_position_tolerance: float

    @classmethod
    def from_payload(cls, params: dict[str, Any]) -> SpeedProfileParameters:
        navigation = params.get("navigation", {})
        planner = params.get("planner", {})
        localization = params.get("localization", {})
        if not isinstance(navigation, dict):
            navigation = {}
        if not isinstance(planner, dict):
            planner = {}
        if not isinstance(localization, dict):
            localization = {}
        return cls(
            control_period=max(
                0.005,
                float(navigation.get("control_period", 0.05) or 0.05),
            ),
            max_acceleration=max(
                0.05,
                float(
                    navigation.get("max_linear_acceleration", 0.50) or 0.50
                ),
            ),
            max_deceleration=max(
                0.05,
                float(
                    navigation.get("max_linear_deceleration", 0.80) or 0.80
                ),
            ),
            precision_start_distance=max(
                0.02,
                float(
                    planner.get("precision_start_distance", 0.10) or 0.10
                ),
            ),
            precision_speed_limit=max(
                0.01,
                float(
                    navigation.get("goal_precision_speed_limit", 0.08) or 0.08
                ),
            ),
            precision_min_speed=max(
                0.001,
                float(
                    navigation.get("goal_precision_min_speed", 0.01) or 0.01
                ),
            ),
            precision_linear_gain=max(
                0.1,
                float(
                    navigation.get("goal_precision_linear_gain", 1.2) or 1.2
                ),
            ),
            goal_position_tolerance=max(
                0.001,
                float(
                    localization.get("goal_position_tolerance", 0.005)
                    or 0.005
                ),
            ),
        )


class SpeedProfiler:
    """Apply acceleration, braking and final-approach limits to linear speed."""

    def __init__(self) -> None:
        self._speed = 0.0

    @property
    def speed(self) -> float:
        return self._speed

    def reset(self) -> None:
        self._speed = 0.0

    def step(
        self,
        requested_speed: float,
        goal_distance: float,
        params: SpeedProfileParameters,
    ) -> float:
        requested = float(requested_speed)
        distance = max(0.0, float(goal_distance))
        if (
            abs(requested) <= 1e-9
            or distance <= params.goal_position_tolerance
        ):
            self.reset()
            return 0.0

        direction = -1.0 if requested < 0.0 else 1.0
        if self._speed * direction < 0.0:
            self.reset()

        target = abs(requested)
        distance_to_stop = max(
            0.0,
            distance - params.goal_position_tolerance,
        )
        braking_limit = math.sqrt(
            2.0 * params.max_deceleration * distance_to_stop
        )
        target = min(target, braking_limit)

        if distance <= params.precision_start_distance:
            precision_limit = max(
                params.precision_min_speed,
                params.precision_linear_gain * distance_to_stop,
            )
            target = min(
                target,
                params.precision_speed_limit,
                precision_limit,
            )

        current = abs(self._speed)
        maximum_delta = params.max_acceleration * params.control_period
        if target > current:
            current = min(target, current + maximum_delta)
        else:
            # Geometry, curvature and braking caps are safety constraints and
            # must take effect immediately. The physical deceleration envelope
            # is already represented by ``braking_limit`` above.
            current = target
        self._speed = direction * current
        return self._speed


__all__ = ["SpeedProfileParameters", "SpeedProfiler"]
