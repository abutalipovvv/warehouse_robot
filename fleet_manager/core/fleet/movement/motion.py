"""Fleet motion runtime composed from focused policies."""

from __future__ import annotations

from .kinematics import FleetMotionKinematicsMixin
from .lifecycle import FleetRuntimeLifecycleMixin
from .safety import FleetMotionSafetyMixin
from .step import FleetMotionStepMixin


class FleetMotionRuntimeMixin(
    FleetMotionStepMixin,
    FleetMotionKinematicsMixin,
    FleetMotionSafetyMixin,
    FleetRuntimeLifecycleMixin,
):
    """Advance fleet motion while preserving runtime safety."""


__all__ = ["FleetMotionRuntimeMixin"]
