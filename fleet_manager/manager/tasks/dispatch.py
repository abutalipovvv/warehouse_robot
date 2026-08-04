"""Composition of fleet task dispatch capabilities."""

from __future__ import annotations

from fleet_manager.manager.tasks.order_admission import OrderAdmissionMixin
from fleet_manager.manager.tasks.stationary_blockers import StationaryBlockerRecoveryMixin
from fleet_manager.manager.tasks.stationary_clearance import StationaryClearanceMixin
from fleet_manager.manager.tasks.dispatch_requests import DispatchRequestBatchMixin
from fleet_manager.manager.tasks.planning_jobs import AsyncPlanningJobMixin
from fleet_manager.manager.tasks.replanning import RuntimeReplanMixin
from fleet_manager.manager.tasks.rolling_continuation import RollingContinuationMixin
from fleet_manager.manager.tasks.dispatch_results import DispatchResultMixin
from fleet_manager.manager.tasks.order_lifecycle import OrderLifecycleMixin


class FleetTaskDispatchMixin(
    OrderAdmissionMixin,
    StationaryBlockerRecoveryMixin,
    StationaryClearanceMixin,
    DispatchRequestBatchMixin,
    AsyncPlanningJobMixin,
    RuntimeReplanMixin,
    RollingContinuationMixin,
    DispatchResultMixin,
    OrderLifecycleMixin,
):
    """Preserve dispatch hooks while composing focused capabilities."""
