from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True, slots=True)
class ArrivalParameters:
    """Physical conditions required before a route may become ARRIVED."""

    position_tolerance: float
    yaw_tolerance: float
    linear_velocity_tolerance: float
    angular_velocity_tolerance: float
    stable_cycles: int

    @classmethod
    def from_payload(cls, params: dict[str, Any]) -> ArrivalParameters:
        localization = params.get("localization", {})
        if not isinstance(localization, dict):
            localization = {}
        return cls(
            position_tolerance=max(
                0.001,
                float(
                    localization.get("goal_position_tolerance", 0.005)
                    or 0.005
                ),
            ),
            yaw_tolerance=math.radians(
                max(
                    0.1,
                    float(
                        localization.get("allowed_yaw_error_deg", 1.0)
                        or 1.0
                    ),
                )
            ),
            linear_velocity_tolerance=max(
                0.001,
                float(
                    localization.get(
                        "goal_linear_velocity_tolerance",
                        0.01,
                    )
                    or 0.01
                ),
            ),
            angular_velocity_tolerance=max(
                0.001,
                float(
                    localization.get(
                        "goal_angular_velocity_tolerance",
                        0.03,
                    )
                    or 0.03
                ),
            ),
            stable_cycles=max(
                1,
                int(localization.get("arrival_stable_cycles", 5) or 5),
            ),
        )


class ArrivalMonitor:
    """Debounce millimetre-level arrival against pose and velocity noise."""

    def __init__(self) -> None:
        self._stable_cycles = 0

    @property
    def stable_cycles(self) -> int:
        return self._stable_cycles

    def reset(self) -> None:
        self._stable_cycles = 0

    def update(
        self,
        *,
        remaining_distance: float,
        goal_position_error: float,
        goal_yaw_error: float,
        linear_velocity: float,
        angular_velocity: float,
        params: ArrivalParameters,
    ) -> bool:
        stable = (
            max(0.0, float(remaining_distance))
            <= params.position_tolerance
            and max(0.0, float(goal_position_error))
            <= params.position_tolerance
            and abs(float(goal_yaw_error)) <= params.yaw_tolerance
            and abs(float(linear_velocity))
            <= params.linear_velocity_tolerance
            and abs(float(angular_velocity))
            <= params.angular_velocity_tolerance
        )
        self._stable_cycles = self._stable_cycles + 1 if stable else 0
        return self._stable_cycles >= params.stable_cycles


__all__ = ["ArrivalMonitor", "ArrivalParameters"]
