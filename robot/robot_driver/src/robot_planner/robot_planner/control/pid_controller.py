from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Any, Sequence

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


def _pair(value: Any, default: tuple[float, float]) -> tuple[float, float]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            pass
    return default


@dataclass(frozen=True, slots=True)
class PidParameters:
    enabled: bool
    kp: tuple[float, float]
    ki: tuple[float, float]
    kd: tuple[float, float]
    integral_limit: tuple[float, float]
    derivative_filter_alpha: float
    dt_min: float
    dt_max: float
    curvature_feedforward_gain: float

    @classmethod
    def from_payload(cls, params: dict[str, Any]) -> "PidParameters":
        navigation = params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        raw = navigation.get("tracking_pid", {})
        if not isinstance(raw, dict):
            raw = {}
        angular_gain = max(0.0, float(navigation.get("angular_gain", 2.2) or 2.2))
        return cls(
            enabled=bool(raw.get("enabled", True)),
            kp=_pair(raw.get("kp"), (angular_gain, angular_gain)),
            ki=_pair(raw.get("ki"), (0.04, 0.01)),
            kd=_pair(raw.get("kd"), (0.06, 0.03)),
            integral_limit=tuple(
                max(0.0, item)
                for item in _pair(raw.get("integral_limit"), (0.35, 0.35))
            ),
            derivative_filter_alpha=min(
                1.0,
                max(0.0, float(raw.get("derivative_filter_alpha", 0.25) or 0.25)),
            ),
            dt_min=max(1e-4, float(raw.get("dt_min", 0.005) or 0.005)),
            dt_max=max(0.01, float(raw.get("dt_max", 0.20) or 0.20)),
            curvature_feedforward_gain=max(
                0.0,
                float(raw.get("curvature_feedforward_gain", 1.0) or 0.0),
            ),
        )


@dataclass(frozen=True, slots=True)
class PidTerms:
    proportional: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0
    feedforward: float = 0.0
    unclamped: float = 0.0
    output: float = 0.0


class PidController:
    """PID for lateral/heading errors, implemented with NumPy vectors."""

    def __init__(self, params: PidParameters) -> None:
        self.params = params
        self._configure_matrices()
        self.last_terms = PidTerms()
        self.reset()

    def configure(self, params: PidParameters) -> None:
        if params == self.params:
            return
        self.params = params
        self._configure_matrices()
        self.reset()

    def reset(self) -> None:
        self._integral = np.zeros(2, dtype=np.float64)
        self._previous_error: FloatArray | None = None
        self._filtered_derivative = np.zeros(2, dtype=np.float64)
        self._last_update: float | None = None
        self.last_terms = PidTerms()

    def step(
        self,
        error_state: Sequence[float] | FloatArray,
        *,
        feedforward: float,
        max_output: float,
        dt: float | None = None,
        now: float | None = None,
    ) -> float:
        error = np.asarray(error_state, dtype=np.float64)
        if error.shape != (2,) or not np.all(np.isfinite(error)):
            raise ValueError("PID error_state must contain two finite values")

        control_dt = self._control_dt(dt=dt, now=now)
        if self._previous_error is None:
            derivative = np.zeros(2, dtype=np.float64)
        else:
            raw_derivative = (error - self._previous_error) / control_dt
            alpha = self.params.derivative_filter_alpha
            derivative = alpha * raw_derivative + (1.0 - alpha) * self._filtered_derivative

        candidate_integral = np.clip(
            self._integral + error * control_dt,
            -self._integral_limit,
            self._integral_limit,
        )
        proportional_term = float((self._kp @ error).item())
        derivative_term = float((self._kd @ derivative).item())
        integral_term = float((self._ki @ candidate_integral).item())
        ff_term = self.params.curvature_feedforward_gain * float(feedforward)
        raw_output = ff_term + proportional_term + integral_term + derivative_term
        limit = max(0.0, float(max_output))
        output = float(np.clip(raw_output, -limit, limit)) if limit > 0.0 else 0.0

        # Conditional integration: do not accumulate further into saturation.
        if abs(raw_output - output) > 1e-12 and raw_output * integral_term > 0.0:
            candidate_integral = self._integral
            integral_term = float((self._ki @ candidate_integral).item())
            raw_output = ff_term + proportional_term + integral_term + derivative_term
            output = float(np.clip(raw_output, -limit, limit)) if limit > 0.0 else 0.0

        self._integral = candidate_integral
        self._previous_error = error.copy()
        self._filtered_derivative = derivative
        self.last_terms = PidTerms(
            proportional=proportional_term,
            integral=integral_term,
            derivative=derivative_term,
            feedforward=ff_term,
            unclamped=raw_output,
            output=output,
        )
        return output

    def _configure_matrices(self) -> None:
        self._kp = np.asarray(self.params.kp, dtype=np.float64).reshape(1, 2)
        self._ki = np.asarray(self.params.ki, dtype=np.float64).reshape(1, 2)
        self._kd = np.asarray(self.params.kd, dtype=np.float64).reshape(1, 2)
        self._integral_limit = np.asarray(self.params.integral_limit, dtype=np.float64)

    def _control_dt(self, *, dt: float | None, now: float | None) -> float:
        if dt is not None:
            value = float(dt)
        else:
            current = monotonic() if now is None else float(now)
            value = (
                self.params.dt_min
                if self._last_update is None
                else current - self._last_update
            )
            self._last_update = current
        if not np.isfinite(value):
            value = self.params.dt_min
        return min(self.params.dt_max, max(self.params.dt_min, value))
