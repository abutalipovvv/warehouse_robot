"""Traffic-planning facade shared by all fleet transports."""

from __future__ import annotations

from typing import Any

from .continuous import TrafficContinuousWaitSchedulingMixin
from .preparation import TrafficPlanPreparationMixin
from .reservations import TrafficReservationMixin
from .results import TrafficPlanResultMixin


class TrafficPlanningMixin(
    TrafficPlanPreparationMixin,
    TrafficContinuousWaitSchedulingMixin,
    TrafficReservationMixin,
    TrafficPlanResultMixin,
):
    """Serialize access to the reusable fleet planner."""

    def _plan_valid_requests(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        # FleetMapfPlanner keeps reusable graph/planner objects. Dynamic
        # dispatch runs in one background thread, while explicit operator
        # requests may still arrive from the HTTP server.
        with self._planner_lock:
            return self._plan_valid_requests_unlocked(
                valid_requests,
                payload,
            )
