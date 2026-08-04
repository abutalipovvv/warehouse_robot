"""Immutable models exchanged by planning components."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from threading import Event
from typing import Any

from fleet_manager.core.fleet.domain.models import FleetRobot


FrozenValue = Any


@dataclass(frozen=True, slots=True)
class FrozenMapping:
    """A deterministic immutable copy of a string-keyed mapping."""

    items: tuple[tuple[str, FrozenValue], ...] = ()

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any] | None) -> FrozenMapping:
        if values is None:
            return cls()
        frozen_items = tuple(
            (str(key), freeze_value(value))
            for key, value in sorted(values.items(), key=lambda item: str(item[0]))
        )
        return cls(frozen_items)

    def to_dict(self) -> dict[str, Any]:
        return {
            key: thaw_value(value)
            for key, value in self.items
        }

    def get(self, key: str, default: Any = None) -> Any:
        for item_key, value in self.items:
            if item_key == key:
                return thaw_value(value)
        return default


def freeze_value(value: Any) -> FrozenValue:
    """Copy supported planning data into immutable primitive structures."""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return FrozenMapping.from_mapping(value)
    if isinstance(value, (list, tuple)):
        return tuple(freeze_value(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return tuple(
            sorted(
                (freeze_value(item) for item in value),
                key=repr,
            )
        )
    raise TypeError(
        f"planning snapshot cannot freeze {type(value).__name__}"
    )


def thaw_value(value: FrozenValue) -> Any:
    """Return a fresh mutable copy for the existing solver API."""

    if isinstance(value, FrozenMapping):
        return value.to_dict()
    if isinstance(value, tuple):
        return [thaw_value(item) for item in value]
    return value


@dataclass(frozen=True, slots=True)
class RobotPlanningState:
    """Compact robot state relevant to route and reservation planning."""

    robot_id: str
    status: str
    current_lm: str
    target_lm: str
    active_order_id: str
    route_revision: int
    route_clock: float
    pose: FrozenMapping

    @classmethod
    def from_robot(cls, robot: FleetRobot) -> RobotPlanningState:
        return cls(
            robot_id=robot.name,
            status=robot.status,
            current_lm=robot.current_lm,
            target_lm=robot.target_lm,
            active_order_id=robot.active_order_id,
            route_revision=int(robot.route_revision),
            route_clock=float(robot.route_clock),
            pose=FrozenMapping.from_mapping(robot.pose),
        )


@dataclass(frozen=True, slots=True)
class RoutePlanningState:
    """Immutable committed route used to validate future occupancy."""

    robot_id: str
    route_revision: int
    route_clock: float
    trajectory: tuple[FrozenMapping, ...]

    @classmethod
    def from_robot(cls, robot: FleetRobot) -> RoutePlanningState:
        return cls(
            robot_id=robot.name,
            route_revision=int(robot.route_revision),
            route_clock=float(robot.route_clock),
            trajectory=tuple(
                FrozenMapping.from_mapping(point)
                for point in robot.trajectory
                if isinstance(point, dict)
            ),
        )


@dataclass(frozen=True, slots=True)
class ReservationSnapshot:
    """One temporal vertex or edge reservation supplied to MAPF."""

    kind: str
    owner: str
    start: float
    end: float
    resource: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class TrafficResourceSnapshot:
    """One live traffic-zone or controlled-corridor lease."""

    kind: str
    resource_id: str
    owner: str
    expires_at: float


@dataclass(frozen=True, slots=True)
class PlanningSnapshot:
    """Complete immutable input for one finite solver transaction."""

    revision: int
    created_at: float
    robots: tuple[RobotPlanningState, ...]
    active_routes: tuple[RoutePlanningState, ...]
    reservations: tuple[ReservationSnapshot, ...]
    traffic_resources: tuple[TrafficResourceSnapshot, ...]
    blockers: tuple[str, ...]
    graph_revision: int | str | None
    map_revision: int | str | None
    requests: tuple[FrozenMapping, ...]
    primary_payload: FrozenMapping
    fallback_payload: FrozenMapping | None = None
    soft_blocked_lms: tuple[str, ...] = ()
    strict_stationary_avoidance: bool = True
    reservation_offset: float = 0.0
    held_snapshot_owners: tuple[str, ...] = ()
    release_owners: tuple[str, ...] = ()

    def request_dicts(self) -> list[dict[str, Any]]:
        return [request.to_dict() for request in self.requests]

    def primary_payload_dict(self) -> dict[str, Any]:
        return self.primary_payload.to_dict()

    def fallback_payload_dict(self) -> dict[str, Any] | None:
        if self.fallback_payload is None:
            return None
        return self.fallback_payload.to_dict()


class PlanningReason(str, Enum):
    SAFETY_REPLAN = "safety_replan"
    DEADLOCK_RECOVERY = "deadlock_recovery"
    ROLLING_CONTINUATION = "rolling_continuation"
    ORDER_DISPATCH = "order_dispatch"
    BACKGROUND_OPTIMIZATION = "background_optimization"

    @classmethod
    def from_job_kind(cls, kind: str) -> PlanningReason:
        normalized = str(kind).strip().lower()
        if normalized == "runtime_replan":
            return cls.SAFETY_REPLAN
        if normalized == "coupled_replan":
            return cls.DEADLOCK_RECOVERY
        if normalized in {"prefetch", "prefetch_batch"}:
            return cls.ROLLING_CONTINUATION
        if normalized == "dispatch":
            return cls.ORDER_DISPATCH
        return cls.BACKGROUND_OPTIMIZATION


class PlanningPriority(IntEnum):
    SAFETY_REPLAN = 0
    DEADLOCK_RECOVERY = 10
    ROLLING_CONTINUATION = 20
    ORDER_DISPATCH = 30
    BACKGROUND_OPTIMIZATION = 40

    @classmethod
    def for_reason(cls, reason: PlanningReason) -> PlanningPriority:
        return cls[reason.name]


class PlanningJobStatus(str, Enum):
    QUEUED = "queued"
    RUNNING = "running"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    STALE = "stale"
    FAILED = "failed"
    COMMITTED = "committed"


_ALLOWED_JOB_TRANSITIONS = {
    PlanningJobStatus.QUEUED: {
        PlanningJobStatus.RUNNING,
        PlanningJobStatus.CANCELLED,
        PlanningJobStatus.STALE,
    },
    PlanningJobStatus.RUNNING: {
        PlanningJobStatus.COMPLETED,
        PlanningJobStatus.CANCELLED,
        PlanningJobStatus.STALE,
        PlanningJobStatus.FAILED,
    },
    PlanningJobStatus.COMPLETED: {
        PlanningJobStatus.COMMITTED,
        PlanningJobStatus.FAILED,
        PlanningJobStatus.STALE,
    },
    PlanningJobStatus.CANCELLED: set(),
    PlanningJobStatus.STALE: set(),
    PlanningJobStatus.FAILED: set(),
    PlanningJobStatus.COMMITTED: set(),
}


class CancellationToken:
    """Cooperative cancellation signal passed only to planning code."""

    __slots__ = ("_event",)

    def __init__(self) -> None:
        self._event = Event()

    @property
    def cancelled(self) -> bool:
        return self._event.is_set()

    def cancel(self) -> None:
        self._event.set()


@dataclass(slots=True)
class PlanningJob:
    job_id: str
    reason: PlanningReason
    priority: PlanningPriority
    snapshot: PlanningSnapshot
    submitted_at: float
    deadline: float | None = None
    cancellation_token: CancellationToken = field(
        default_factory=CancellationToken
    )
    coalescing_key: str = ""
    robot_ids: tuple[str, ...] = ()
    conflict_component_ids: tuple[str, ...] = ()
    status: PlanningJobStatus = PlanningJobStatus.QUEUED
    started_at: float | None = None
    finished_at: float | None = None

    def transition(self, next_status: PlanningJobStatus) -> None:
        if next_status is self.status:
            return
        allowed = _ALLOWED_JOB_TRANSITIONS[self.status]
        if next_status not in allowed:
            raise ValueError(
                f"planning job cannot transition from "
                f"{self.status.value} to {next_status.value}"
            )
        self.status = next_status


@dataclass(frozen=True, slots=True)
class PlanCandidate:
    expected_revision: int
    job_id: str
    reason: PlanningReason
    created_at: float
    finished_at: float
    result: FrozenMapping
    plans: tuple[FrozenMapping, ...] = ()
    reservations: tuple[FrozenMapping, ...] = ()
    metadata: FrozenMapping = field(default_factory=FrozenMapping)
    diagnostics: FrozenMapping = field(default_factory=FrozenMapping)

    @classmethod
    def from_result(
        cls,
        job: PlanningJob,
        result: dict[str, Any],
        *,
        finished_at: float,
        metadata: dict[str, Any] | None = None,
    ) -> PlanCandidate:
        plans = tuple(
            FrozenMapping.from_mapping(plan)
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        )
        reservations = tuple(
            FrozenMapping.from_mapping(reservation)
            for reservation in result.get("reservations", [])
            if isinstance(reservation, dict)
        )
        debug = result.get("debug")
        diagnostics = debug if isinstance(debug, dict) else {}
        return cls(
            expected_revision=job.snapshot.revision,
            job_id=job.job_id,
            reason=job.reason,
            created_at=job.snapshot.created_at,
            finished_at=float(finished_at),
            result=FrozenMapping.from_mapping(result),
            plans=plans,
            reservations=reservations,
            metadata=FrozenMapping.from_mapping(metadata),
            diagnostics=FrozenMapping.from_mapping(diagnostics),
        )
