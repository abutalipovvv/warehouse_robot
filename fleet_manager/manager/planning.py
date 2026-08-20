"""Immutable models exchanged by planning components."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from threading import Event, Lock
from time import monotonic
from typing import Any, Callable, Generic, TypeVar

from fleet_manager.robot.model import FleetRobot


FrozenValue = Any
PlannerCall = Callable[..., dict[str, Any]]
CommitValueT = TypeVar("CommitValueT")
CheckpointT = TypeVar("CheckpointT")


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
class PlanningDependencyStamp:
    """Planning inputs whose owners can invalidate one planner transaction.

    A queued order changes the fleet-wide revision, but it cannot invalidate
    a route already planned for another robot. Route revisions of requested
    robots and reservation owners, authored traffic ownership, and world
    blockers can invalidate it and are therefore captured explicitly.
    """

    map_revision: int | str | None = None
    graph_revision: int | str | None = None
    robot_route_revisions: tuple[tuple[str, int], ...] = ()
    traffic_resource_owners: tuple[tuple[str, str, str], ...] = ()
    world_blockers: tuple[str, ...] = ()


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
    dependency_stamp: PlanningDependencyStamp = field(
        default_factory=PlanningDependencyStamp
    )
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
    ROLLING_EMERGENCY = 5
    ROLLING_CRITICAL = 10
    DEADLOCK_RECOVERY = 15
    ROLLING_URGENT = 20
    ROLLING_CONTINUATION = 25
    ROLLING_NORMAL = 25
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
    DEADLINE_EXCEEDED = "deadline_exceeded"
    STALE = "stale"
    FAILED = "failed"
    COMMITTED = "committed"


_ALLOWED_JOB_TRANSITIONS = {
    PlanningJobStatus.QUEUED: {
        PlanningJobStatus.RUNNING,
        PlanningJobStatus.CANCELLED,
        PlanningJobStatus.DEADLINE_EXCEEDED,
        PlanningJobStatus.STALE,
    },
    PlanningJobStatus.RUNNING: {
        PlanningJobStatus.COMPLETED,
        PlanningJobStatus.CANCELLED,
        PlanningJobStatus.DEADLINE_EXCEEDED,
        PlanningJobStatus.STALE,
        PlanningJobStatus.FAILED,
    },
    PlanningJobStatus.COMPLETED: {
        PlanningJobStatus.COMMITTED,
        PlanningJobStatus.FAILED,
        PlanningJobStatus.STALE,
    },
    PlanningJobStatus.CANCELLED: set(),
    PlanningJobStatus.DEADLINE_EXCEEDED: set(),
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


class PlanningCancelledError(RuntimeError):
    """A planning job was cancelled by its owner."""


class PlanningDeadlineExceededError(RuntimeError):
    """A planning job exceeded its explicit deadline."""


@dataclass(frozen=True, slots=True)
class PlanningControl:
    """Small cooperative control passed into long-running algorithms."""

    cancellation_token: CancellationToken
    deadline: float | None = None
    clock: Callable[[], float] = monotonic

    def check(self) -> None:
        if self.deadline is not None and self.clock() > self.deadline:
            self.cancellation_token.cancel()
            raise PlanningDeadlineExceededError("planning deadline exceeded")
        if self.cancellation_token.cancelled:
            raise PlanningCancelledError("planning job cancelled")

    def should_cancel(self) -> bool:
        """Raise on cancellation so nested algorithms stop immediately."""

        self.check()
        return False


@dataclass(frozen=True, slots=True)
class PlanningJob:
    """Immutable request accepted by the planning scheduler."""

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


@dataclass(slots=True)
class PlanningJobRecord:
    """Mutable owner-thread state around one immutable planning request."""

    job: PlanningJob | None = None
    status: PlanningJobStatus = PlanningJobStatus.QUEUED
    started_at: float | None = None
    finished_at: float | None = None
    candidate: PlanCandidate | None = None
    error: str | None = None
    kind: str = "dispatch"
    entries: list[Any] = field(default_factory=list)
    requests: list[dict[str, Any]] = field(default_factory=list)
    payload: dict[str, Any] = field(default_factory=dict)
    result: dict[str, Any] | None = None
    done: bool = False
    discard: bool = False
    stale: bool = False
    expected_revision: int | None = None
    corridor_gates: dict[str, dict[str, Any]] = field(default_factory=dict)
    cycle: tuple[str, ...] = ()
    order_id: str = ""
    robot_name: str = ""
    generation: int = 0
    route_revision: int = 0
    route_clock: float = 0.0
    route_revisions: dict[str, int] = field(default_factory=dict)
    start_lm: str = ""
    final_goal: str = ""
    escape_goal: str = ""
    request: dict[str, Any] | None = None
    vacancy_recovery_signature: tuple[tuple[str, str, int], ...] = ()
    conflict_component_ids: tuple[str, ...] = ()

    def transition_to(self, next_status: PlanningJobStatus) -> None:
        if next_status is self.status:
            return
        allowed = _ALLOWED_JOB_TRANSITIONS[self.status]
        if next_status not in allowed:
            raise ValueError(
                f"planning job cannot transition from "
                f"{self.status.value} to {next_status.value}"
            )
        self.status = next_status

    @property
    def job_id(self) -> str:
        return self.job.job_id if self.job is not None else ""


@dataclass(frozen=True, slots=True)
class PlanCandidate:
    expected_revision: int
    job_id: str
    reason: PlanningReason
    created_at: float
    finished_at: float
    result: FrozenMapping
    dependency_stamp: PlanningDependencyStamp = field(
        default_factory=PlanningDependencyStamp
    )
    backend_used: str = ""
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
        backend_used: str = "",
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
            dependency_stamp=job.snapshot.dependency_stamp,
            backend_used=str(backend_used),
            plans=plans,
            reservations=reservations,
            metadata=FrozenMapping.from_mapping(metadata),
            diagnostics=FrozenMapping.from_mapping(diagnostics),
        )


class PlanningSolverService:
    """Run the planner using only immutable data from a planning job."""

    def __init__(
        self,
        planner_call: PlannerCall,
        planner_lock: Lock,
        *,
        accepts_control: bool = False,
    ) -> None:
        if not callable(planner_call):
            raise TypeError("planner_call must be callable")
        self._planner_call = planner_call
        self._planner_lock = planner_lock
        self._accepts_control = bool(accepts_control)

    def solve(self, job: PlanningJob) -> PlanCandidate:
        control = PlanningControl(
            cancellation_token=job.cancellation_token,
            deadline=job.deadline,
        )
        control.check()

        with self._planner_lock:
            control.check()
            primary_result = self._call_planner(
                job.snapshot.primary_payload_dict(),
                control,
            )
            fallback_result: dict[str, Any] | None = None
            fallback_payload = job.snapshot.fallback_payload_dict()
            if (
                not primary_result.get("ok")
                and fallback_payload is not None
            ):
                control.check()
                fallback_result = self._call_planner(
                    fallback_payload,
                    control,
                )

        control.check()
        selected_result = primary_result
        selected_source = "primary"
        if (
            fallback_result is not None
            and fallback_result.get("ok")
            and not job.snapshot.strict_stationary_avoidance
        ):
            selected_result = fallback_result
            selected_source = "fallback"
        backend_used = self._backend_name(selected_result)
        metadata = {
            "primaryResult": primary_result,
            "fallbackResult": fallback_result,
            "selectedSource": selected_source,
            "backend": backend_used,
        }
        return PlanCandidate.from_result(
            job,
            selected_result,
            finished_at=monotonic(),
            metadata=metadata,
            backend_used=backend_used,
        )

    def _call_planner(
        self,
        payload: dict[str, Any],
        control: PlanningControl,
    ) -> dict[str, Any]:
        if self._accepts_control:
            return self._planner_call(
                payload,
                should_cancel=control.should_cancel,
            )
        return self._planner_call(payload)

    @staticmethod
    def _backend_name(result: dict[str, Any]) -> str:
        debug = result.get("debug")
        if not isinstance(debug, dict):
            return ""
        return str(debug.get("plannerBackend") or debug.get("backend") or "")


class PlanCommitStatus(str, Enum):
    COMMITTED = "committed"
    STALE = "stale"


@dataclass(frozen=True, slots=True)
class PlanCommitResult(Generic[CommitValueT]):
    status: PlanCommitStatus
    value: CommitValueT | None = None


class PlanCommitService:
    """Validate revision and apply one all-or-nothing runtime mutation."""

    def __init__(self, current_revision: Callable[[], int]) -> None:
        if not callable(current_revision):
            raise TypeError("current_revision must be callable")
        self._current_revision = current_revision

    def commit(
        self,
        candidate: PlanCandidate,
        *,
        validate: Callable[[], None],
        capture: Callable[[], CheckpointT],
        apply: Callable[[], CommitValueT],
        restore: Callable[[CheckpointT], None],
        is_current: Callable[[], bool] | None = None,
    ) -> PlanCommitResult[CommitValueT]:
        current = is_current or (
            lambda: candidate.expected_revision == self._current_revision()
        )
        if not current():
            return PlanCommitResult(PlanCommitStatus.STALE)

        validate()
        # Validation can observe external robot input, so check again before
        # taking the rollback checkpoint.
        if not current():
            return PlanCommitResult(PlanCommitStatus.STALE)

        checkpoint = capture()
        try:
            value = apply()
        except BaseException:
            restore(checkpoint)
            raise
        return PlanCommitResult(PlanCommitStatus.COMMITTED, value)
