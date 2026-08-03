"""Composition of reservation and continuous-wait scheduling."""

from __future__ import annotations

from .planning_continuous import TrafficContinuousWaitSchedulingMixin
from .planning_reservations import TrafficReservationMixin


class TrafficReservationSchedulingMixin(
    TrafficContinuousWaitSchedulingMixin,
    TrafficReservationMixin,
):
    """Provide the complete traffic-reservation scheduling policy."""


__all__ = ["TrafficReservationSchedulingMixin"]
