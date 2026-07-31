"""Fleet motion runtime composed from focused policies."""

from __future__ import annotations

from .motion_kinematics import FleetMotionKinematicsMixin
from .motion_lifecycle import FleetRuntimeLifecycleMixin
from .motion_safety import FleetMotionSafetyMixin
from .motion_step import FleetMotionStepMixin


class FleetMotionRuntimeMixin(
    FleetMotionStepMixin,
    FleetMotionKinematicsMixin,
    FleetMotionSafetyMixin,
    FleetRuntimeLifecycleMixin,
):
    """Advance fleet motion while preserving runtime safety."""


__all__ = ["FleetMotionRuntimeMixin"]
