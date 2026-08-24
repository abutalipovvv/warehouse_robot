from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from ..math import TrajectoryArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class TrajectorySpeedParameters:
    """Limits used to build a velocity envelope for a complete route."""

    enabled: bool
    control_period: float
    max_acceleration: float
    max_deceleration: float
    max_forward_speed: float
    max_backward_speed: float
    max_lateral_acceleration: float
    max_jerk: float
    jerk_smoothing_iterations: int
    curve_speed_limit: float
    curve_curvature_threshold: float
    max_angular_speed: float
    curve_angular_reserve: float

    @classmethod
    def from_payload(cls, params: dict[str, Any]) -> "TrajectorySpeedParameters":
        navigation = params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        raw = navigation.get("trajectory_speed_profile", {})
        if not isinstance(raw, dict):
            raw = {}

        route_speed = max(
            0.05,
            float(navigation.get("route_speed", 0.35) or 0.35),
        )
        strict_speed = max(
            0.05,
            float(navigation.get("strict_speed_limit", route_speed) or route_speed),
        )
        default_speed_limit = min(route_speed, strict_speed)
        return cls(
            enabled=bool(raw.get("enabled", True)),
            control_period=max(
                0.005,
                float(navigation.get("control_period", 0.05) or 0.05),
            ),
            max_acceleration=max(
                0.05,
                float(navigation.get("max_linear_acceleration", 0.50) or 0.50),
            ),
            max_deceleration=max(
                0.05,
                float(navigation.get("max_linear_deceleration", 0.80) or 0.80),
            ),
            max_forward_speed=max(
                0.05,
                float(
                    raw.get("max_forward_speed", default_speed_limit)
                    or default_speed_limit
                ),
            ),
            max_backward_speed=max(
                0.05,
                float(
                    raw.get("max_backward_speed", default_speed_limit)
                    or default_speed_limit
                ),
            ),
            max_lateral_acceleration=max(
                0.01,
                float(raw.get("max_lateral_acceleration", 0.20) or 0.20),
            ),
            max_jerk=max(
                0.0,
                float(raw.get("max_jerk", 1.50) or 0.0),
            ),
            jerk_smoothing_iterations=max(
                0,
                min(
                    8,
                    int(raw.get("jerk_smoothing_iterations", 3) or 0),
                ),
            ),
            curve_speed_limit=max(
                0.05,
                float(navigation.get("curve_speed_limit", 0.22) or 0.22),
            ),
            curve_curvature_threshold=max(
                0.001,
                float(
                    navigation.get("curve_curvature_threshold", 0.05)
                    or 0.05
                ),
            ),
            max_angular_speed=max(
                0.05,
                float(navigation.get("max_angular_speed", 0.90) or 0.90),
            ),
            curve_angular_reserve=min(
                0.75,
                max(
                    0.10,
                    float(navigation.get("curve_angular_reserve", 0.35) or 0.35),
                ),
            ),
        )


@dataclass(frozen=True, slots=True)
class TrajectorySpeedProfile:
    """Precomputed speed envelope indexed by trajectory arc length."""

    s: FloatArray
    speed_limits: FloatArray
    acceleration: FloatArray

    @classmethod
    def build(
        cls,
        trajectory: TrajectoryArray,
        params: TrajectorySpeedParameters,
    ) -> "TrajectorySpeedProfile":
        speed_limits = cls._geometry_limits(trajectory, params)
        for _ in range(2):
            cls._apply_acceleration_limits(
                trajectory.s,
                speed_limits,
                params.max_acceleration,
                params.max_deceleration,
            )
        if params.max_jerk > 0.0:
            for _ in range(params.jerk_smoothing_iterations):
                cls._smooth_acceleration_rise(
                    trajectory.s,
                    speed_limits,
                    params.max_acceleration,
                    params.max_jerk,
                    params.control_period,
                )
                cls._smooth_acceleration_rise(
                    trajectory.length - trajectory.s[::-1],
                    speed_limits[::-1],
                    params.max_deceleration,
                    params.max_jerk,
                    params.control_period,
                )
                cls._apply_acceleration_limits(
                    trajectory.s,
                    speed_limits,
                    params.max_acceleration,
                    params.max_deceleration,
                )

        acceleration = cls._segment_acceleration(trajectory.s, speed_limits)
        for array in (speed_limits, acceleration):
            array.setflags(write=False)
        return cls(
            s=trajectory.s,
            speed_limits=speed_limits,
            acceleration=acceleration,
        )

    def speed_at(self, path_distance: float) -> float:
        target = float(np.clip(path_distance, 0.0, float(self.s[-1])))
        return float(np.interp(target, self.s, self.speed_limits))

    @staticmethod
    def _geometry_limits(
        trajectory: TrajectoryArray,
        params: TrajectorySpeedParameters,
    ) -> FloatArray:
        backward = trajectory.motion_directions == "backward"
        limits = np.where(
            backward,
            params.max_backward_speed,
            params.max_forward_speed,
        ).astype(np.float64)

        curvature = np.abs(trajectory.curvature)
        curved = curvature > params.curve_curvature_threshold
        safe_curvature = np.maximum(curvature, 1e-9)
        lateral_limit = np.sqrt(
            params.max_lateral_acceleration / safe_curvature
        )
        angular_budget = params.max_angular_speed * (
            1.0 - params.curve_angular_reserve
        )
        angular_limit = angular_budget / safe_curvature
        limits = np.minimum(limits, lateral_limit)
        limits = np.where(
            curved,
            np.minimum(limits, params.curve_speed_limit),
            limits,
        )
        limits = np.where(curved, np.minimum(limits, angular_limit), limits)
        limits[-1] = 0.0
        return limits

    @staticmethod
    def _apply_acceleration_limits(
        s: FloatArray,
        speeds: FloatArray,
        max_acceleration: float,
        max_deceleration: float,
    ) -> None:
        for index in range(1, speeds.size):
            distance = float(s[index] - s[index - 1])
            if distance <= 1e-12:
                speeds[index] = min(speeds[index], speeds[index - 1])
                continue
            reachable = math.sqrt(
                max(
                    0.0,
                    speeds[index - 1] ** 2
                    + 2.0 * max_acceleration * distance,
                )
            )
            speeds[index] = min(speeds[index], reachable)

        for index in range(speeds.size - 2, -1, -1):
            distance = float(s[index + 1] - s[index])
            if distance <= 1e-12:
                speeds[index] = min(speeds[index], speeds[index + 1])
                continue
            reachable = math.sqrt(
                max(
                    0.0,
                    speeds[index + 1] ** 2
                    + 2.0 * max_deceleration * distance,
                )
            )
            speeds[index] = min(speeds[index], reachable)

    @staticmethod
    def _smooth_acceleration_rise(
        s: FloatArray,
        speeds: FloatArray,
        max_acceleration: float,
        max_jerk: float,
        control_period: float,
    ) -> None:
        """Lower peaks so positive acceleration builds up without a step."""

        previous_acceleration = 0.0
        for index in range(speeds.size - 1):
            distance = float(s[index + 1] - s[index])
            if distance <= 1e-12:
                previous_acceleration = 0.0
                continue
            requested = (
                speeds[index + 1] ** 2 - speeds[index] ** 2
            ) / (2.0 * distance)
            if requested <= 0.0:
                previous_acceleration = 0.0
                continue
            travel_time = 2.0 * distance / max(
                speeds[index] + speeds[index + 1],
                0.02,
            )
            travel_time = float(np.clip(travel_time, control_period, 0.50))
            allowed = min(
                max_acceleration,
                previous_acceleration + max_jerk * travel_time,
            )
            if requested > allowed:
                speeds[index + 1] = min(
                    speeds[index + 1],
                    math.sqrt(
                        max(
                            0.0,
                            speeds[index] ** 2 + 2.0 * allowed * distance,
                        )
                    ),
                )
                requested = allowed
            previous_acceleration = requested

    @staticmethod
    def _segment_acceleration(s: FloatArray, speeds: FloatArray) -> FloatArray:
        acceleration = np.zeros_like(speeds)
        if speeds.size < 2:
            return acceleration
        distance = np.diff(s)
        acceleration[:-1] = np.divide(
            np.diff(speeds**2),
            2.0 * distance,
            out=np.zeros_like(distance),
            where=distance > 1e-12,
        )
        acceleration[-1] = acceleration[-2]
        return acceleration


__all__ = ["TrajectorySpeedParameters", "TrajectorySpeedProfile"]
