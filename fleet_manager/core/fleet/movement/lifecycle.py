"""Composition of runtime replan and retreat lifecycles."""

from __future__ import annotations

from .deadlock_retreat import FleetDeadlockRetreatMixin
from .replanning import FleetRuntimeReplanMixin


class FleetRuntimeLifecycleMixin(
    FleetDeadlockRetreatMixin,
    FleetRuntimeReplanMixin,
):
    """Provide recovery lifecycles used by the motion time step."""
