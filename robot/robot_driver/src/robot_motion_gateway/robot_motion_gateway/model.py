"""Pure motion arbitration model, independent from ROS wiring."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class MotionMode(str, Enum):
    IDLE = "IDLE"
    ROUTE = "ROUTE"
    TELEOP = "TELEOP"
    NAV2 = "NAV2"
    SLAM = "SLAM"
    ESTOP = "ESTOP"

    @classmethod
    def parse(cls, value: object) -> "MotionMode":
        normalized = str(value or "").strip().upper()
        try:
            return cls(normalized)
        except ValueError as exc:
            allowed = ", ".join(mode.value for mode in cls)
            raise ValueError(
                f"unknown motion mode {value!r}; expected one of: {allowed}"
            ) from exc


MODE_SOURCE: dict[MotionMode, str] = {
    MotionMode.ROUTE: "route",
    MotionMode.TELEOP: "teleop",
    MotionMode.NAV2: "nav2",
}


@dataclass(frozen=True, slots=True)
class MotionLimits:
    max_forward_speed: float = 1.5
    max_backward_speed: float = 1.3
    max_angular_speed: float = 1.8

    def normalized(self) -> "MotionLimits":
        return MotionLimits(
            max_forward_speed=_positive(self.max_forward_speed, 1.5),
            max_backward_speed=_positive(self.max_backward_speed, 1.3),
            max_angular_speed=_positive(self.max_angular_speed, 1.8),
        )


@dataclass(frozen=True, slots=True)
class MotionTimeouts:
    route: float = 0.25
    teleop: float = 0.45
    nav2: float = 0.30

    def for_source(self, source: str) -> float:
        if source == "route":
            return _positive(self.route, 0.25)
        if source == "teleop":
            return _positive(self.teleop, 0.45)
        if source == "nav2":
            return _positive(self.nav2, 0.30)
        raise ValueError(f"unknown motion source: {source}")


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    linear: float
    angular: float
    received_at: float


@dataclass(frozen=True, slots=True)
class CommandDecision:
    mode: MotionMode
    active_source: str
    linear: float
    angular: float
    watchdog_stop: bool
    reason: str


class MotionArbiter:
    """Select exactly one velocity source for the current robot mode."""

    VALID_SOURCES = frozenset(MODE_SOURCE.values())

    def __init__(
        self,
        *,
        limits: MotionLimits | None = None,
        timeouts: MotionTimeouts | None = None,
    ) -> None:
        self.limits = (limits or MotionLimits()).normalized()
        self.timeouts = timeouts or MotionTimeouts()
        self.mode = MotionMode.IDLE
        self.reason = "startup"
        self._commands: dict[str, VelocityCommand] = {}

    def configure(
        self,
        *,
        limits: MotionLimits,
        timeouts: MotionTimeouts,
    ) -> None:
        self.limits = limits.normalized()
        self.timeouts = timeouts

    def transition(
        self,
        mode: MotionMode | str,
        *,
        reason: str = "",
    ) -> bool:
        next_mode = mode if isinstance(mode, MotionMode) else MotionMode.parse(mode)
        changed = next_mode != self.mode
        if changed:
            # Never replay a command received under an earlier ownership mode.
            self._commands.clear()
            self.mode = next_mode
        self.reason = str(reason or next_mode.value.lower())
        return changed

    def accept(
        self,
        source: str,
        *,
        linear: float,
        angular: float,
        received_at: float,
    ) -> None:
        normalized = str(source or "").strip().lower()
        if normalized not in self.VALID_SOURCES:
            raise ValueError(f"unknown motion source: {source}")
        values = (float(linear), float(angular), float(received_at))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("velocity command must contain finite values")
        self._commands[normalized] = VelocityCommand(*values)

    def decide(self, *, now: float) -> CommandDecision:
        source = MODE_SOURCE.get(self.mode, "")
        if not source:
            return CommandDecision(
                mode=self.mode,
                active_source="",
                linear=0.0,
                angular=0.0,
                watchdog_stop=False,
                reason=self.reason,
            )

        command = self._commands.get(source)
        if command is None:
            return self._watchdog_decision(source, "waiting for command")
        age = max(0.0, float(now) - command.received_at)
        timeout = self.timeouts.for_source(source)
        if age > timeout:
            return self._watchdog_decision(
                source,
                f"{source} command stale for {age:.3f}s",
            )

        limits = self.limits
        linear = min(
            limits.max_forward_speed,
            max(-limits.max_backward_speed, command.linear),
        )
        angular = min(
            limits.max_angular_speed,
            max(-limits.max_angular_speed, command.angular),
        )
        return CommandDecision(
            mode=self.mode,
            active_source=source,
            linear=linear,
            angular=angular,
            watchdog_stop=False,
            reason=self.reason,
        )

    def _watchdog_decision(
        self,
        source: str,
        reason: str,
    ) -> CommandDecision:
        return CommandDecision(
            mode=self.mode,
            active_source=source,
            linear=0.0,
            angular=0.0,
            watchdog_stop=True,
            reason=reason,
        )


def _positive(value: object, fallback: float) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return fallback
    return numeric if math.isfinite(numeric) and numeric > 0.0 else fallback
