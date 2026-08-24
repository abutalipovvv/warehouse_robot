from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class LqrParameters:
    """Configuration for the two-state differential-drive LQR."""

    q_lateral: float
    q_heading: float
    r_angular: float
    min_model_speed: float
    speed_resolution: float
    riccati_iterations: int
    riccati_tolerance: float
    curvature_feedforward_gain: float

    @classmethod
    def from_payload(cls, params: dict[str, Any]) -> "LqrParameters":
        navigation = params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        raw = navigation.get("tracking_lqr", {})
        if not isinstance(raw, dict):
            raw = {}
        return cls(
            q_lateral=max(1e-6, float(raw.get("q_lateral", 5.0) or 5.0)),
            q_heading=max(1e-6, float(raw.get("q_heading", 2.5) or 2.5)),
            r_angular=max(1e-6, float(raw.get("r_angular", 1.0) or 1.0)),
            min_model_speed=max(
                0.01,
                float(raw.get("min_model_speed", 0.08) or 0.08),
            ),
            speed_resolution=max(
                0.001,
                float(raw.get("speed_resolution", 0.02) or 0.02),
            ),
            riccati_iterations=max(
                8,
                int(raw.get("riccati_iterations", 80) or 80),
            ),
            riccati_tolerance=max(
                1e-12,
                float(raw.get("riccati_tolerance", 1e-9) or 1e-9),
            ),
            curvature_feedforward_gain=max(
                0.0,
                float(raw.get("curvature_feedforward_gain", 1.0) or 0.0),
            ),
        )


@dataclass(frozen=True, slots=True)
class LqrTerms:
    lateral_feedback: float = 0.0
    heading_feedback: float = 0.0
    feedforward: float = 0.0
    unclamped: float = 0.0
    output: float = 0.0


class LqrController:
    """LQR feedback for cross-track and heading error.

    The model uses the standard path-error state ``[robot lateral offset,
    robot heading - path heading]``. The executor exposes the opposite sign
    for heading, so the conversion is kept here instead of leaking controller
    conventions into route execution.
    """

    def __init__(self, params: LqrParameters) -> None:
        self.params = params
        self.last_terms = LqrTerms()
        self._gain_key: tuple[float, float] | None = None
        self._gain = np.zeros((1, 2), dtype=np.float64)

    def configure(self, params: LqrParameters) -> None:
        if params == self.params:
            return
        self.params = params
        self.reset()

    def reset(self) -> None:
        self.last_terms = LqrTerms()
        self._gain_key = None
        self._gain = np.zeros((1, 2), dtype=np.float64)

    def step(
        self,
        *,
        cross_track_error: float,
        heading_error: float,
        linear_speed: float,
        curvature: float,
        max_output: float,
        dt: float,
    ) -> float:
        values = np.asarray(
            (
                cross_track_error,
                heading_error,
                linear_speed,
                curvature,
                dt,
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("LQR inputs must contain finite values")

        control_dt = max(1e-4, float(dt))
        model_speed = self._quantized_speed(abs(float(linear_speed)))
        gain = self._gain_for(model_speed, control_dt)
        state = np.asarray(
            (float(cross_track_error), -float(heading_error)),
            dtype=np.float64,
        )
        feedback = -(gain @ state).item()
        lateral_feedback = -float(gain[0, 0] * state[0])
        heading_feedback = -float(gain[0, 1] * state[1])
        feedforward = (
            self.params.curvature_feedforward_gain
            * abs(float(linear_speed))
            * float(curvature)
        )
        raw_output = float(feedforward + feedback)
        limit = max(0.0, float(max_output))
        output = float(np.clip(raw_output, -limit, limit)) if limit else 0.0
        self.last_terms = LqrTerms(
            lateral_feedback=lateral_feedback,
            heading_feedback=heading_feedback,
            feedforward=feedforward,
            unclamped=raw_output,
            output=output,
        )
        return output

    def _quantized_speed(self, speed: float) -> float:
        resolution = self.params.speed_resolution
        bounded = max(self.params.min_model_speed, float(speed))
        return max(
            self.params.min_model_speed,
            round(bounded / resolution) * resolution,
        )

    def _gain_for(self, speed: float, dt: float) -> FloatArray:
        key = (round(float(speed), 6), round(float(dt), 6))
        if key == self._gain_key:
            return self._gain

        system = np.asarray(
            ((1.0, speed * dt), (0.0, 1.0)),
            dtype=np.float64,
        )
        input_matrix = np.asarray(((0.0,), (dt,)), dtype=np.float64)
        state_cost = np.diag(
            np.asarray(
                (self.params.q_lateral, self.params.q_heading),
                dtype=np.float64,
            )
        )
        input_cost = self.params.r_angular
        riccati = state_cost.copy()

        for _ in range(self.params.riccati_iterations):
            denominator = input_cost + float(
                (input_matrix.T @ riccati @ input_matrix).item()
            )
            gain = (input_matrix.T @ riccati @ system) / denominator
            updated = (
                system.T @ riccati @ system
                - system.T @ riccati @ input_matrix @ gain
                + state_cost
            )
            if float(np.max(np.abs(updated - riccati))) <= self.params.riccati_tolerance:
                riccati = updated
                break
            riccati = updated

        denominator = input_cost + float(
            (input_matrix.T @ riccati @ input_matrix).item()
        )
        self._gain = (input_matrix.T @ riccati @ system) / denominator
        self._gain_key = key
        return self._gain


__all__ = ["LqrController", "LqrParameters", "LqrTerms"]
