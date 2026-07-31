"""Composition of runtime replan and retreat lifecycles."""

from __future__ import annotations

from .motion_replanning import FleetRuntimeReplanMixin
from .motion_retreat import FleetDeadlockRetreatMixin


class FleetRuntimeLifecycleMixin(
    FleetDeadlockRetreatMixin,
    FleetRuntimeReplanMixin,
):
    """Provide recovery lifecycles used by the motion time step."""


__all__ = ["FleetRuntimeLifecycleMixin"]
