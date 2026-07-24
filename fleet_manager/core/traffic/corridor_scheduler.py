"""Rolling slot scheduler for map-authored controlled corridors.

The scheduler deliberately knows nothing about ordinary graph traffic.  A
caller supplies the ids of the explicitly authored controlled-corridor
resources and requests whose passages use those resources.  Free-space
traffic remains the responsibility of the normal route planner and MAPF
stack.

Opposing or perpendicular phases treat each authored region as a
capacity-one resource.  Compatible robots in the same canonical flow may
form a convoy: their passages can overlap while both entry and exit order
retain the configured headway.

The scheduling function is deterministic and side-effect free.  The small
``CentralCorridorScheduler`` wrapper only retains the last immutable schedule
so callers can publish a stable schedule epoch instead of issuing a new
command on every control tick.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from math import ceil, floor, isfinite, ulp
from typing import Collection, Hashable, Iterable, Mapping


RouteRevision = Hashable
_MIN_TIME_TOLERANCE_SEC = 1e-9


def _absolute_window_fits(
    origin: float,
    exit_offset_sec: float,
    absolute_exit: float,
) -> bool:
    """Compare an offset with an epoch timestamp without cancellation."""
    tolerance = max(
        _MIN_TIME_TOLERANCE_SEC,
        4.0 * ulp(origin),
        4.0 * ulp(absolute_exit),
    )
    return origin + exit_offset_sec <= absolute_exit + tolerance


class CorridorSlotState(str, Enum):
    """How strongly a scheduled passage is bound."""

    TENTATIVE = "tentative"
    COMMITTED = "committed"


class CorridorDecisionStatus(str, Enum):
    """Result of scheduling one request."""

    GRANTED = "granted"
    DEFERRED = "deferred"
    REJECTED = "rejected"


@dataclass(frozen=True, slots=True)
class CorridorSchedulerConfig:
    """Policy for a rolling controlled-corridor calendar.

    ``headway_sec`` is enforced at both entry and exit for a same-direction
    convoy.  ``direction_change_sec`` is used when the direction changes and
    should include the time needed to prove that the last footprint has left
    the corridor.  Starved requests are grouped by age quantum; a phase may
    serve at most ``max_direction_batch`` while another starved direction is
    waiting.  Long no-wait passages may use a larger adaptive batch: changing
    direction requires the complete conflicting resource window to clear,
    while another compatible convoy member adds only ``headway_sec``.  The
    adaptive batch amortizes that clearance without allowing the oldest
    opposing flow to accumulate more than ``max_phase_extension_sec`` of
    additional convoy delay.
    """

    horizon_sec: float = 30.0
    commit_horizon_sec: float = 2.0
    headway_sec: float = 0.35
    direction_change_sec: float = 0.9
    starvation_sec: float = 8.0
    direction_switch_cost_sec: float = 1.5
    priority_cost_sec: float = 0.05
    wait_age_cost_sec: float = 0.03
    tentative_change_penalty_sec: float = 2.0
    occupancy_recheck_sec: float = 0.5
    starvation_age_quantum_sec: float = 2.0
    max_direction_batch: int = 3
    max_adaptive_direction_batch: int = 12
    phase_amortization_sec: float = 4.0
    max_phase_extension_sec: float = 8.0

    def __post_init__(self) -> None:
        non_negative = {
            "horizon_sec": self.horizon_sec,
            "commit_horizon_sec": self.commit_horizon_sec,
            "headway_sec": self.headway_sec,
            "direction_change_sec": self.direction_change_sec,
            "starvation_sec": self.starvation_sec,
            "direction_switch_cost_sec": self.direction_switch_cost_sec,
            "priority_cost_sec": self.priority_cost_sec,
            "wait_age_cost_sec": self.wait_age_cost_sec,
            "tentative_change_penalty_sec": self.tentative_change_penalty_sec,
            "occupancy_recheck_sec": self.occupancy_recheck_sec,
            "starvation_age_quantum_sec": self.starvation_age_quantum_sec,
            "phase_amortization_sec": self.phase_amortization_sec,
            "max_phase_extension_sec": self.max_phase_extension_sec,
        }
        invalid = [
            name
            for name, value in non_negative.items()
            if not isfinite(value) or value < 0.0
        ]
        if invalid:
            names = ", ".join(sorted(invalid))
            raise ValueError(
                "corridor scheduler values must be finite and >= 0: "
                + names
            )
        if self.horizon_sec <= 0.0:
            raise ValueError("horizon_sec must be greater than zero")
        if self.headway_sec <= 0.0:
            raise ValueError("headway_sec must be greater than zero")
        if self.occupancy_recheck_sec <= 0.0:
            raise ValueError("occupancy_recheck_sec must be greater than zero")
        if self.starvation_age_quantum_sec <= 0.0:
            raise ValueError(
                "starvation_age_quantum_sec must be greater than zero"
            )
        if self.phase_amortization_sec <= 0.0:
            raise ValueError("phase_amortization_sec must be greater than zero")
        if (
            isinstance(self.max_direction_batch, bool)
            or not isinstance(self.max_direction_batch, int)
            or self.max_direction_batch <= 0
        ):
            raise ValueError("max_direction_batch must be a positive integer")
        if (
            isinstance(self.max_adaptive_direction_batch, bool)
            or not isinstance(self.max_adaptive_direction_batch, int)
            or self.max_adaptive_direction_batch < self.max_direction_batch
        ):
            raise ValueError(
                "max_adaptive_direction_batch must be an integer greater "
                "than or equal to max_direction_batch"
            )


@dataclass(frozen=True, slots=True)
class CorridorResourceWindow:
    """Relative occupancy window for one authored corridor resource.

    A passage may cross several narrow rectangles before reaching another
    legal holding LM.  Admission for that passage is atomic, but the robot
    does not physically occupy every rectangle for the complete traversal.
    Keeping a per-resource window lets the scheduler pipeline independent
    sections without weakening the all-or-nothing entry decision.
    """

    region_id: str
    entry_offset_sec: float
    exit_offset_sec: float
    direction: str

    def __post_init__(self) -> None:
        region_id = self.region_id.strip()
        direction = self.direction.strip()
        if not region_id or not direction:
            raise ValueError(
                "resource window region_id and direction must not be empty"
            )
        if (
            not isfinite(self.entry_offset_sec)
            or not isfinite(self.exit_offset_sec)
            or self.entry_offset_sec < 0.0
            or self.exit_offset_sec <= self.entry_offset_sec
        ):
            raise ValueError(
                "resource window offsets must be finite and "
                "0 <= entry < exit"
            )
        object.__setattr__(self, "region_id", region_id)
        object.__setattr__(self, "direction", direction)


@dataclass(frozen=True, slots=True)
class CorridorRequest:
    """A robot's desired atomic passage through controlled regions.

    Times are absolute scheduler-clock seconds.  ``duration_sec`` must cover
    the whole protected movement: entry turn, corridor traversal, exit turn
    and any footprint-clearance margin required by the caller.

    ``direction`` must be a canonical entry-portal to exit-portal phase id,
    not the robot yaw or the direction of one edge.  Requests with the same
    value may be batched; different values require a direction-change guard.
    ``downstream_ready_at`` implements "do not block the box": entry is
    delayed so the robot cannot reach its exit before downstream space is
    available.  ``past_commit_point`` means the robot crossed its last safe
    stop line.  Its matching committed slot is then rolled forward past a
    missed predicted exit instead of being reordered.  The caller clears the
    flag after physical exit.  A superseding route must change its passage
    identity (regions, canonical direction, staging or exit); a revision-only
    rolling handoff intentionally keeps the same grant.
    """

    robot_id: str
    regions: tuple[str, ...]
    direction: str
    earliest_entry: float
    duration_sec: float
    staging_lm: str
    exit_lm: str
    route_revision: RouteRevision
    priority: float = 0.0
    wait_age_sec: float = 0.0
    deadline: float | None = None
    downstream_ready_at: float | None = None
    downstream_available: bool = True
    predecessor_robot_id: str | None = None
    entered: bool = False
    past_commit_point: bool = False
    requires_explicit_commit: bool = False
    resource_windows: tuple[CorridorResourceWindow, ...] = ()

    def __post_init__(self) -> None:
        robot_id = self.robot_id.strip()
        direction = self.direction.strip()
        staging_lm = self.staging_lm.strip()
        exit_lm = self.exit_lm.strip()
        predecessor_robot_id = str(
            self.predecessor_robot_id or ""
        ).strip()
        regions = tuple(dict.fromkeys(region.strip() for region in self.regions))
        if not robot_id:
            raise ValueError("robot_id must not be empty")
        if not direction:
            raise ValueError("direction must not be empty")
        if not staging_lm or not exit_lm:
            raise ValueError("staging_lm and exit_lm must not be empty")
        if not regions or any(not region for region in regions):
            raise ValueError("regions must contain non-empty controlled-region ids")
        finite_values = {
            "earliest_entry": self.earliest_entry,
            "duration_sec": self.duration_sec,
            "priority": self.priority,
            "wait_age_sec": self.wait_age_sec,
        }
        if self.downstream_ready_at is not None:
            finite_values["downstream_ready_at"] = self.downstream_ready_at
        if self.deadline is not None:
            finite_values["deadline"] = self.deadline
        invalid = [
            name for name, value in finite_values.items() if not isfinite(value)
        ]
        if invalid:
            names = ", ".join(sorted(invalid))
            raise ValueError(f"corridor request values must be finite: {names}")
        if self.duration_sec <= 0.0:
            raise ValueError("duration_sec must be greater than zero")
        if self.wait_age_sec < 0.0:
            raise ValueError("wait_age_sec must be >= 0")
        if predecessor_robot_id == robot_id:
            raise ValueError("a corridor request cannot precede itself")
        resource_windows = self.resource_windows or tuple(
            CorridorResourceWindow(
                region_id=region_id,
                entry_offset_sec=0.0,
                exit_offset_sec=self.duration_sec,
                direction=direction,
            )
            for region_id in regions
        )
        window_regions = tuple(
            window.region_id for window in resource_windows
        )
        if (
            len(set(window_regions)) != len(window_regions)
            or set(window_regions) != set(regions)
        ):
            raise ValueError(
                "resource windows must contain every request region exactly once"
            )
        if any(
            window.exit_offset_sec > self.duration_sec + 1e-9
            for window in resource_windows
        ):
            raise ValueError(
                "resource window cannot extend beyond passage duration"
            )
        resource_windows = tuple(
            next(
                window
                for window in resource_windows
                if window.region_id == region_id
            )
            for region_id in regions
        )
        object.__setattr__(self, "robot_id", robot_id)
        object.__setattr__(self, "direction", direction)
        object.__setattr__(self, "staging_lm", staging_lm)
        object.__setattr__(self, "exit_lm", exit_lm)
        object.__setattr__(self, "regions", regions)
        object.__setattr__(
            self,
            "predecessor_robot_id",
            predecessor_robot_id or None,
        )
        object.__setattr__(self, "resource_windows", resource_windows)


@dataclass(frozen=True, slots=True)
class CorridorOccupancy:
    """Authoritative observation of a footprint inside a corridor.

    The adapter must submit this record on every scheduling snapshot until
    the complete footprint has crossed the exit boundary.  If the predicted
    exit is already in the past, the scheduler extends the reservation by a
    short recheck interval instead of assuming that the corridor is empty.
    Thus physical truth always outranks a tentative or pre-entry command.
    """

    robot_id: str
    regions: tuple[str, ...]
    direction: str
    entered_at: float
    expected_exit_time: float
    exit_lm: str
    route_revision: RouteRevision
    staging_lm: str = ""
    resource_windows: tuple[CorridorResourceWindow, ...] = ()

    def __post_init__(self) -> None:
        robot_id = self.robot_id.strip()
        direction = self.direction.strip()
        exit_lm = self.exit_lm.strip()
        staging_lm = self.staging_lm.strip()
        regions = tuple(dict.fromkeys(region.strip() for region in self.regions))
        if not robot_id or not direction or not exit_lm:
            raise ValueError(
                "occupancy robot_id, direction and exit_lm must not be empty"
            )
        if not regions or any(not region for region in regions):
            raise ValueError(
                "occupancy regions must contain controlled-region ids"
            )
        if not isfinite(self.entered_at) or not isfinite(
            self.expected_exit_time
        ):
            raise ValueError("occupancy times must be finite")
        object.__setattr__(self, "robot_id", robot_id)
        object.__setattr__(self, "direction", direction)
        object.__setattr__(self, "exit_lm", exit_lm)
        object.__setattr__(self, "staging_lm", staging_lm)
        object.__setattr__(self, "regions", regions)
        resource_windows = self.resource_windows
        if resource_windows:
            window_regions = tuple(
                window.region_id for window in resource_windows
            )
            if (
                len(set(window_regions)) != len(window_regions)
                or set(window_regions) != set(regions)
            ):
                raise ValueError(
                    "occupancy resource windows must contain every region "
                    "exactly once"
                )
            if any(
                not _absolute_window_fits(
                    self.entered_at,
                    window.exit_offset_sec,
                    self.expected_exit_time,
                )
                for window in resource_windows
            ):
                raise ValueError(
                    "occupancy resource window cannot extend beyond "
                    "expected exit"
                )
            resource_windows = tuple(
                next(
                    window
                    for window in resource_windows
                    if window.region_id == region_id
                )
                for region_id in regions
            )
        object.__setattr__(self, "resource_windows", resource_windows)


@dataclass(frozen=True, slots=True)
class CorridorSlot:
    """An immutable reservation in the controlled-corridor calendar."""

    robot_id: str
    regions: tuple[str, ...]
    direction: str
    entry_time: float
    exit_time: float
    staging_lm: str
    exit_lm: str
    route_revision: RouteRevision
    state: CorridorSlotState
    resource_windows: tuple[CorridorResourceWindow, ...] = ()
    past_commit_point: bool = False
    physically_observed: bool = False

    @property
    def duration_sec(self) -> float:
        return self.exit_time - self.entry_time

    def __post_init__(self) -> None:
        resource_windows = self.resource_windows or tuple(
            CorridorResourceWindow(
                region_id=region_id,
                entry_offset_sec=0.0,
                exit_offset_sec=self.duration_sec,
                direction=self.direction,
            )
            for region_id in self.regions
        )
        window_regions = tuple(
            window.region_id for window in resource_windows
        )
        if (
            len(set(window_regions)) != len(window_regions)
            or set(window_regions) != set(self.regions)
        ):
            raise ValueError(
                "slot resource windows must contain every region exactly once"
            )
        if any(
            not _absolute_window_fits(
                self.entry_time,
                window.exit_offset_sec,
                self.exit_time,
            )
            for window in resource_windows
        ):
            raise ValueError(
                "slot resource window cannot extend beyond slot duration"
            )
        object.__setattr__(
            self,
            "resource_windows",
            tuple(
                next(
                    window
                    for window in resource_windows
                    if window.region_id == region_id
                )
                for region_id in self.regions
            ),
        )


@dataclass(frozen=True, slots=True)
class CorridorDecision:
    """Scheduling result for one robot."""

    robot_id: str
    status: CorridorDecisionStatus
    reason: str
    slot: CorridorSlot | None = None


@dataclass(frozen=True, slots=True)
class CorridorSchedule:
    """An immutable rolling calendar and its stable publication epoch."""

    epoch: int
    generated_at: float
    horizon_end: float
    slots: tuple[CorridorSlot, ...] = ()
    decisions: Mapping[str, CorridorDecision] = field(default_factory=dict)
    changed: bool = False

    def slot_for(self, robot_id: str) -> CorridorSlot | None:
        decision = self.decisions.get(robot_id)
        if decision is not None:
            return decision.slot
        return next(
            (slot for slot in self.slots if slot.robot_id == robot_id),
            None,
        )


def build_corridor_schedule(
    requests: Iterable[CorridorRequest],
    *,
    controlled_regions: Collection[str],
    now: float,
    config: CorridorSchedulerConfig | None = None,
    previous: CorridorSchedule | None = None,
    occupancies: Iterable[CorridorOccupancy] | None = None,
    pinned_slots: Iterable[CorridorSlot] | None = None,
) -> CorridorSchedule:
    """Build a deterministic calendar for explicit controlled corridors.

    Committed slots are immutable until their exit time.  Tentative slots are
    rebuilt on every call, but deterministic ordering and an unchanged
    semantic fingerprint keep the public epoch stable.  A request whose route
    revision changes loses only its tentative reservation; an already
    committed passage survives the rolling-route handoff.

    ``occupancies=None`` means no authoritative physical snapshot was
    available, so point-of-no-return authority is retained conservatively.
    Any supplied iterable is a complete snapshot: after a physically observed
    slot's bounded exit grace, absence proves that the footprint has left and
    a stale ``past_commit_point`` request cannot recreate the old passage.
    """

    policy = config or CorridorSchedulerConfig()
    if not isfinite(now):
        raise ValueError("now must be finite")
    explicit_regions = frozenset(
        region.strip() for region in controlled_regions if region.strip()
    )
    request_by_robot = _requests_by_robot(requests)
    occupancy_snapshot_provided = occupancies is not None
    occupancy_by_robot = _occupancies_by_robot(occupancies or ())
    pinned_by_robot = {
        slot.robot_id: slot
        for slot in (pinned_slots or ())
        if slot.state is CorridorSlotState.TENTATIVE
    }
    for request in request_by_robot.values():
        if (
            request.entered
            and not occupancy_snapshot_provided
            and request.robot_id not in occupancy_by_robot
        ):
            entered_at = min(now, request.earliest_entry)
            occupancy_by_robot[request.robot_id] = CorridorOccupancy(
                robot_id=request.robot_id,
                regions=request.regions,
                direction=request.direction,
                entered_at=entered_at,
                expected_exit_time=entered_at + request.duration_sec,
                staging_lm=request.staging_lm,
                exit_lm=request.exit_lm,
                route_revision=request.route_revision,
                resource_windows=request.resource_windows,
            )
    horizon_end = now + policy.horizon_sec
    slots: list[CorridorSlot] = []
    decisions: dict[str, CorridorDecision] = {}

    for occupancy in occupancy_by_robot.values():
        unknown = tuple(
            region
            for region in occupancy.regions
            if region not in explicit_regions
        )
        if unknown:
            raise ValueError(
                "physical occupancy is outside explicit controlled corridors: "
                + ", ".join(unknown)
            )
        occupancy_entry = min(occupancy.entered_at, now)
        occupancy_exit = max(
            occupancy.expected_exit_time,
            now + policy.occupancy_recheck_sec,
        )
        if occupancy.resource_windows:
            origin_shift = occupancy.entered_at - occupancy_entry
            occupancy_windows = tuple(
                CorridorResourceWindow(
                    region_id=window.region_id,
                    entry_offset_sec=(
                        window.entry_offset_sec + origin_shift
                    ),
                    exit_offset_sec=(
                        window.exit_offset_sec + origin_shift
                    ),
                    direction=window.direction,
                )
                for window in occupancy.resource_windows
            )
        else:
            occupancy_windows = tuple(
                CorridorResourceWindow(
                    region_id=region_id,
                    entry_offset_sec=0.0,
                    exit_offset_sec=occupancy_exit - occupancy_entry,
                    direction=occupancy.direction,
                )
                for region_id in occupancy.regions
            )
        occupancy_exit = max(
            occupancy_exit,
            occupancy_entry
            + max(
                window.exit_offset_sec
                for window in occupancy_windows
            ),
        )
        slot = CorridorSlot(
            robot_id=occupancy.robot_id,
            regions=occupancy.regions,
            direction=occupancy.direction,
            entry_time=occupancy_entry,
            exit_time=occupancy_exit,
            staging_lm=occupancy.staging_lm,
            exit_lm=occupancy.exit_lm,
            route_revision=occupancy.route_revision,
            state=CorridorSlotState.COMMITTED,
            resource_windows=occupancy_windows,
            past_commit_point=True,
            physically_observed=True,
        )
        slots.append(slot)
        decisions[occupancy.robot_id] = CorridorDecision(
            robot_id=occupancy.robot_id,
            status=CorridorDecisionStatus.GRANTED,
            reason="authoritative physical corridor occupancy",
            slot=slot,
        )

    physically_occupied_robots = set(occupancy_by_robot)
    cleared_passage_robots: set[str] = set()

    # A committed command is a safety contract only for the same physical
    # passage.  A route-revision handoff may refresh its revision, while a
    # changed portal or direction must obtain a new slot.  An entered passage
    # survives one missing rolling-planner snapshot until its bounded exit.
    if previous is not None:
        for old_slot in previous.slots:
            if old_slot.robot_id in physically_occupied_robots:
                continue
            if not _slot_interval_fits(
                old_slot,
                slots,
                policy,
            ):
                # Physical occupancy invalidates a not-yet-entered command.
                # Compare exact local resource windows: an owner in region A
                # must not erase a valid future slot in region B merely because
                # both belong to the same atomic passage bundle.
                continue
            request = request_by_robot.get(old_slot.robot_id)
            same_passage = (
                request is not None
                and _same_passage(request, old_slot)
            )
            confirmed_physical_absence = (
                occupancy_snapshot_provided
                and old_slot.physically_observed
                and old_slot.robot_id not in physically_occupied_robots
            )
            if (
                confirmed_physical_absence
                and old_slot.exit_time <= now
            ):
                if same_passage:
                    cleared_passage_robots.add(old_slot.robot_id)
                continue
            same_revision = (
                same_passage
                and request is not None
                and old_slot.route_revision == request.route_revision
            )
            pinned_tentative = bool(
                old_slot.state is CorridorSlotState.TENTATIVE
                and pinned_by_robot.get(old_slot.robot_id) == old_slot
                and same_revision
                and request is not None
                and request.requires_explicit_commit
                and request.downstream_available
            )
            past_commit_authority = (
                same_passage
                and request is not None
                and request.past_commit_point
                and not confirmed_physical_absence
            )
            missed_pre_entry_commit = bool(
                occupancy_snapshot_provided
                and same_passage
                and request is not None
                and not request.entered
                and not request.past_commit_point
                and old_slot.robot_id not in physically_occupied_robots
                and request.earliest_entry
                > old_slot.entry_time + policy.occupancy_recheck_sec
            )
            if missed_pre_entry_commit and not pinned_tentative:
                # Calendar commitment becomes immutable only after the robot
                # leaves its final safe staging LM. The authoritative pose
                # snapshot says this robot is still outside, while its fresh
                # ETA says it missed the old start. Keeping the complete old
                # passage here creates a 40–60 second ghost reservation.
                continue
            queue_predecessor = (
                request.predecessor_robot_id
                if request is not None
                else None
            )
            if (
                same_passage
                and request is not None
                and not request.entered
                and queue_predecessor
                and (
                    queue_predecessor in request_by_robot
                    or queue_predecessor in occupancy_by_robot
                )
            ):
                # A newly observed body is physically ahead of this entrant.
                # A pre-entry command may not let the follower overtake it,
                # even when the follower crossed an upstream staging LM.
                continue
            if (
                old_slot.past_commit_point
                and request is not None
                and not past_commit_authority
                and not confirmed_physical_absence
            ):
                # A fresh request snapshot is authoritative: clearing the
                # flag means the footprint exited; changing the revision
                # means the old passage was superseded.
                continue
            promotable_tentative = (
                old_slot.state is CorridorSlotState.TENTATIVE
                and same_revision
                and request is not None
                and not request.requires_explicit_commit
                and request.downstream_available
                and _request_base_entry(request, now) <= old_slot.entry_time
                and (
                    request.past_commit_point
                    or old_slot.entry_time
                    <= now + policy.commit_horizon_sec
                )
            )
            retained_without_snapshot = (
                request is None
                and old_slot.past_commit_point
                and old_slot.entry_time <= now
                and (
                    not occupancy_snapshot_provided
                    or old_slot.physically_observed
                )
            )
            if not same_passage and not retained_without_snapshot:
                continue
            if (
                (
                    old_slot.state is CorridorSlotState.COMMITTED
                    or promotable_tentative
                    or pinned_tentative
                )
                and (
                    old_slot.exit_time > now
                    or past_commit_authority
                )
                and (
                    old_slot.robot_id in request_by_robot
                    or old_slot.entry_time <= now
                )
            ):
                # A robot which crossed the external stop line owns the next
                # passage, but an overdue prediction is not a growing history
                # interval. Rebase the immutable route-time template at
                # ``now``. Keeping the original entry and stretching every
                # resource window fed elapsed calendar time back into the next
                # occupancy projection, making the window grow exponentially
                # while route_clock was stopped.
                rebase_unobserved_authority = bool(
                    past_commit_authority
                    and request is not None
                    and not old_slot.physically_observed
                    and old_slot.exit_time <= now
                )
                retained_entry_time = (
                    max(now, request.earliest_entry)
                    if rebase_unobserved_authority
                    else old_slot.entry_time
                )
                authority_window_origin = (
                    min(
                        window.entry_offset_sec
                        for window in request.resource_windows
                    )
                    if rebase_unobserved_authority
                    else 0.0
                )
                rebased_authority_windows = (
                    tuple(
                        CorridorResourceWindow(
                            region_id=window.region_id,
                            entry_offset_sec=(
                                window.entry_offset_sec
                                - authority_window_origin
                            ),
                            exit_offset_sec=(
                                window.exit_offset_sec
                                - authority_window_origin
                            ),
                            direction=window.direction,
                        )
                        for window in request.resource_windows
                    )
                    if rebase_unobserved_authority
                    else ()
                )
                retained_exit_time = (
                    retained_entry_time
                    + max(
                        window.exit_offset_sec
                        for window in rebased_authority_windows
                    )
                    if rebase_unobserved_authority
                    else (
                        max(
                            old_slot.exit_time,
                            now + policy.occupancy_recheck_sec,
                        )
                        if past_commit_authority
                        else old_slot.exit_time
                    )
                )
                retained_windows = (
                    rebased_authority_windows
                    if rebase_unobserved_authority
                    else old_slot.resource_windows
                )
                retained_slot = CorridorSlot(
                    robot_id=old_slot.robot_id,
                    regions=old_slot.regions,
                    direction=old_slot.direction,
                    entry_time=retained_entry_time,
                    exit_time=retained_exit_time,
                    staging_lm=old_slot.staging_lm,
                    exit_lm=old_slot.exit_lm,
                    route_revision=(
                        request.route_revision
                        if same_passage and request is not None
                        else old_slot.route_revision
                    ),
                    state=(
                        CorridorSlotState.TENTATIVE
                        if pinned_tentative
                        else CorridorSlotState.COMMITTED
                    ),
                    resource_windows=retained_windows,
                    past_commit_point=(
                        old_slot.past_commit_point
                        or past_commit_authority
                    ),
                    physically_observed=old_slot.physically_observed,
                )
                if (
                    not past_commit_authority
                    and not _slot_interval_fits(
                        retained_slot,
                        slots,
                        policy,
                    )
                ):
                    continue
                slots.append(retained_slot)

    occupied_robots = {slot.robot_id for slot in slots}
    pending: list[CorridorRequest] = []
    preferred_entries: dict[str, float] = {}
    if previous is not None:
        for old_slot in previous.slots:
            request = request_by_robot.get(old_slot.robot_id)
            if (
                old_slot.state is CorridorSlotState.TENTATIVE
                and request is not None
                and old_slot.route_revision == request.route_revision
                and _same_passage(request, old_slot)
            ):
                preferred_entries[request.robot_id] = old_slot.entry_time

    for request in request_by_robot.values():
        if request.robot_id in cleared_passage_robots:
            decisions[request.robot_id] = CorridorDecision(
                robot_id=request.robot_id,
                status=CorridorDecisionStatus.DEFERRED,
                reason=(
                    "previous physical passage observed clear; "
                    "await route advance"
                ),
            )
            continue
        if request.robot_id in occupied_robots:
            if request.robot_id in physically_occupied_robots:
                continue
            retained = next(
                slot for slot in slots if slot.robot_id == request.robot_id
            )
            decisions[request.robot_id] = CorridorDecision(
                robot_id=request.robot_id,
                status=CorridorDecisionStatus.GRANTED,
                reason=(
                    "in-flight MAPF slot pinned"
                    if retained.state is CorridorSlotState.TENTATIVE
                    else "committed passage retained"
                ),
                slot=retained,
            )
            continue
        unknown = tuple(
            region for region in request.regions if region not in explicit_regions
        )
        if unknown:
            decisions[request.robot_id] = CorridorDecision(
                robot_id=request.robot_id,
                status=CorridorDecisionStatus.REJECTED,
                reason=(
                    "request is outside explicit controlled corridors: "
                    + ", ".join(unknown)
                ),
            )
            continue
        if not request.downstream_available:
            decisions[request.robot_id] = CorridorDecision(
                robot_id=request.robot_id,
                status=CorridorDecisionStatus.DEFERRED,
                reason=f"downstream exit {request.exit_lm} is unavailable",
            )
            continue
        pending.append(request)

    while pending:
        pending_robot_ids = {
            request.robot_id
            for request in pending
        }
        scheduled_robot_ids = {
            slot.robot_id
            for slot in slots
        }
        proposals: list[
            tuple[
                tuple[object, ...],
                CorridorRequest,
                tuple[float, float, int],
            ]
        ] = []
        for request in pending:
            predecessor = request.predecessor_robot_id
            if predecessor:
                if predecessor in pending_robot_ids:
                    continue
                if (
                    predecessor in request_by_robot
                    or predecessor in occupancy_by_robot
                ) and predecessor not in scheduled_robot_ids:
                    # The physical queue leader has no safe slot yet. Its
                    # follower must stay queued instead of bypassing it.
                    continue
            if _has_pending_flow_predecessor(request, pending):
                continue
            placement = _earliest_placement(
                request,
                slots=slots,
                now=now,
                horizon_end=horizon_end,
                config=policy,
            )
            if placement is None:
                continue
            entry_time, exit_time, phase_switches = placement
            starved_rank = (
                0 if request.wait_age_sec >= policy.starvation_sec else 1
            )
            entered_rank = (
                0
                if request.entered or request.past_commit_point
                else 1
            )
            command_rank = (
                1 if request.requires_explicit_commit else 0
            )
            effective_finish = (
                exit_time
                + (phase_switches * policy.direction_switch_cost_sec)
                - (request.priority * policy.priority_cost_sec)
                - (
                    min(request.wait_age_sec, policy.starvation_sec)
                    * policy.wait_age_cost_sec
                )
            )
            preferred_entry = preferred_entries.get(request.robot_id)
            if preferred_entry is not None:
                preferred_placement = _placement_at(
                    request,
                    entry_time=preferred_entry,
                    slots=slots,
                    now=now,
                    horizon_end=horizon_end,
                    config=policy,
                )
                if (
                    preferred_placement is not None
                    and preferred_placement[1]
                    <= exit_time + policy.tentative_change_penalty_sec
                ):
                    placement = preferred_placement
                    entry_time, exit_time, phase_switches = placement
                    effective_finish = (
                        exit_time
                        + (
                            phase_switches
                            * policy.direction_switch_cost_sec
                        )
                        - (request.priority * policy.priority_cost_sec)
                        - (
                            min(
                                request.wait_age_sec,
                                policy.starvation_sec,
                            )
                            * policy.wait_age_cost_sec
                        )
                    )
            stability_delta = (
                abs(entry_time - preferred_entry)
                if preferred_entry is not None
                else 0.0
            )
            (
                forced_phase_rank,
                age_quantum_rank,
                phase_preference_rank,
            ) = _phase_fairness_ranks(
                request,
                entry_time=entry_time,
                slots=slots,
                pending=pending,
                config=policy,
            )
            proposals.append(
                (
                    (
                        entered_rank,
                        command_rank,
                        starved_rank,
                        forced_phase_rank,
                        phase_preference_rank,
                        age_quantum_rank if starved_rank == 0 else 0,
                        (
                            request.deadline
                            if starved_rank == 0
                            and request.deadline is not None
                            else float("inf")
                            if starved_rank == 0
                            else 0.0
                        ),
                        -request.priority if starved_rank == 0 else 0.0,
                        (
                            -request.wait_age_sec
                            if starved_rank == 0
                            else 0.0
                        ),
                        round(effective_finish, 9),
                        phase_switches,
                        round(stability_delta, 9),
                        -request.priority,
                        -request.wait_age_sec,
                        request.earliest_entry,
                        request.robot_id,
                    ),
                    request,
                    placement,
                )
            )

        if not proposals:
            break
        _, request, placement = min(proposals, key=lambda item: item[0])
        entry_time, exit_time, _ = placement
        state = (
            CorridorSlotState.COMMITTED
            if request.entered
            or request.past_commit_point
            or (
                not request.requires_explicit_commit
                and entry_time <= now + policy.commit_horizon_sec
            )
            else CorridorSlotState.TENTATIVE
        )
        slot = CorridorSlot(
            robot_id=request.robot_id,
            regions=request.regions,
            direction=request.direction,
            entry_time=entry_time,
            exit_time=exit_time,
            staging_lm=request.staging_lm,
            exit_lm=request.exit_lm,
            route_revision=request.route_revision,
            state=state,
            resource_windows=request.resource_windows,
            past_commit_point=request.past_commit_point,
            physically_observed=False,
        )
        slots.append(slot)
        decisions[request.robot_id] = CorridorDecision(
            robot_id=request.robot_id,
            status=CorridorDecisionStatus.GRANTED,
            reason=(
                "committed inside execution horizon"
                if state is CorridorSlotState.COMMITTED
                else "tentative rolling slot"
            ),
            slot=slot,
        )
        pending.remove(request)

    for request in pending:
        reason = (
            "earliest entry is outside rolling horizon"
            if _request_base_entry(request, now) > horizon_end
            else "no safe corridor slot inside rolling horizon"
        )
        decisions[request.robot_id] = CorridorDecision(
            robot_id=request.robot_id,
            status=CorridorDecisionStatus.DEFERRED,
            reason=reason,
        )

    ordered_slots = tuple(
        sorted(slots, key=lambda slot: (slot.entry_time, slot.robot_id))
    )
    old_fingerprint = (
        _schedule_fingerprint(previous.slots)
        if previous is not None
        else ()
    )
    new_fingerprint = _schedule_fingerprint(ordered_slots)
    changed = new_fingerprint != old_fingerprint
    if previous is None:
        epoch = 1 if ordered_slots else 0
    else:
        epoch = previous.epoch + 1 if changed else previous.epoch
    return CorridorSchedule(
        epoch=epoch,
        generated_at=now,
        horizon_end=horizon_end,
        slots=ordered_slots,
        decisions=decisions,
        changed=changed,
    )


class CentralCorridorScheduler:
    """Stateful facade that retains only the last immutable calendar."""

    def __init__(
        self,
        controlled_regions: Collection[str],
        *,
        config: CorridorSchedulerConfig | None = None,
    ) -> None:
        self._controlled_regions = frozenset(
            region.strip() for region in controlled_regions if region.strip()
        )
        self._config = config or CorridorSchedulerConfig()
        self._schedule: CorridorSchedule | None = None
        self._pinned_slots: dict[str, CorridorSlot] = {}

    @property
    def controlled_regions(self) -> frozenset[str]:
        return self._controlled_regions

    @property
    def current_schedule(self) -> CorridorSchedule | None:
        return self._schedule

    def update(
        self,
        requests: Iterable[CorridorRequest],
        *,
        now: float,
        occupancies: Iterable[CorridorOccupancy] | None = None,
    ) -> CorridorSchedule:
        self._schedule = build_corridor_schedule(
            requests,
            controlled_regions=self._controlled_regions,
            now=now,
            config=self._config,
            previous=self._schedule,
            occupancies=occupancies,
            pinned_slots=self._pinned_slots.values(),
        )
        for robot_id, pinned in list(self._pinned_slots.items()):
            current = self._schedule.slot_for(robot_id)
            if (
                current != pinned
                or current.state is not CorridorSlotState.TENTATIVE
            ):
                # Physical occupancy and already committed commands outrank an
                # in-flight proposal.  Once either invalidates the exact slot,
                # the worker may finish but its captured commit must fail.
                self._pinned_slots.pop(robot_id, None)
        return self._schedule

    def pin_slot(
        self,
        robot_id: str,
        *,
        expected: CorridorSlot,
    ) -> bool:
        """Hold one exact tentative proposal while its MAPF worker runs."""
        schedule = self._schedule
        name = str(robot_id)
        current = schedule.slot_for(name) if schedule is not None else None
        if (
            current != expected
            or expected.robot_id != name
            or expected.state is not CorridorSlotState.TENTATIVE
        ):
            return False
        existing = self._pinned_slots.get(name)
        if existing is not None and existing != expected:
            return False
        self._pinned_slots[name] = expected
        return True

    def release_slot_pin(
        self,
        robot_id: str,
        *,
        expected: CorridorSlot | None = None,
    ) -> None:
        """Release only the worker lease that captured ``expected``."""
        name = str(robot_id)
        current = self._pinned_slots.get(name)
        if current is None:
            return
        if expected is not None and current != expected:
            return
        self._pinned_slots.pop(name, None)

    def commit_slot(
        self,
        robot_id: str,
        *,
        expected: CorridorSlot,
        actual: CorridorSlot | None = None,
    ) -> CorridorSchedule | None:
        """Promote an unchanged tentative slot to an issued command.

        Planning a continuation is a two-phase transaction: first the rolling
        calendar offers a tentative slot, then MAPF proves that its actual
        resource windows fit.  At that point the command must survive the
        route-revision handoff exactly like an imminent/entered passage.
        ``expected`` prevents a stale worker result from committing a slot
        which the runtime scheduler has already moved.
        """
        schedule = self._schedule
        if schedule is None:
            return None
        current = schedule.slot_for(str(robot_id))
        if current is None or current != expected:
            return None
        candidate = actual or current
        if (
            candidate.robot_id != current.robot_id
            or candidate.regions != current.regions
            or candidate.direction != current.direction
            or candidate.staging_lm != current.staging_lm
            or candidate.exit_lm != current.exit_lm
            or tuple(
                (window.region_id, window.direction)
                for window in candidate.resource_windows
            )
            != tuple(
                (window.region_id, window.direction)
                for window in current.resource_windows
            )
            or candidate.entry_time
            < schedule.generated_at - self._config.occupancy_recheck_sec
            or candidate.entry_time > schedule.horizon_end + 1e-9
        ):
            return None
        candidate = replace(
            candidate,
            state=CorridorSlotState.COMMITTED,
            physically_observed=False,
        )
        other_slots = tuple(
            slot
            for slot in schedule.slots
            if slot.robot_id != robot_id
        )
        committed_others = tuple(
            slot
            for slot in other_slots
            if slot.state is CorridorSlotState.COMMITTED
        )
        if not _slot_interval_fits(
            candidate,
            committed_others,
            self._config,
        ):
            return None
        if (
            current.state is CorridorSlotState.COMMITTED
            and candidate == current
        ):
            return schedule

        committed = candidate
        # A tentative slot is a calendar proposal, not an issued command. Exact
        # SIPP timing may legitimately arrive later than its nominal proposal
        # because of an ordinary edge/turn reservation. Keep physical and
        # already committed passages immutable, displace only overlapping
        # tentative proposals, and let the next update place them again around
        # this validated command.
        retained_others = tuple(
            slot
            for slot in other_slots
            if (
                slot.state is CorridorSlotState.COMMITTED
                or _slot_interval_fits(
                    committed,
                    (slot,),
                    self._config,
                )
            )
        )
        displaced_robot_ids = {
            slot.robot_id
            for slot in other_slots
            if slot not in retained_others
        }
        slots = tuple(
            sorted(
                (
                    committed,
                    *retained_others,
                ),
                key=lambda slot: (slot.entry_time, slot.robot_id),
            )
        )
        decisions = dict(schedule.decisions)
        for displaced_robot_id in displaced_robot_ids:
            decisions[displaced_robot_id] = CorridorDecision(
                robot_id=displaced_robot_id,
                status=CorridorDecisionStatus.DEFERRED,
                reason="tentative slot displaced by validated command",
            )
        decisions[str(robot_id)] = CorridorDecision(
            robot_id=str(robot_id),
            status=CorridorDecisionStatus.GRANTED,
            reason="committed after MAPF resource validation",
            slot=committed,
        )
        self._schedule = CorridorSchedule(
            epoch=schedule.epoch + 1,
            generated_at=schedule.generated_at,
            horizon_end=schedule.horizon_end,
            slots=slots,
            decisions=decisions,
            changed=True,
        )
        self._pinned_slots.pop(str(robot_id), None)
        for displaced_robot_id in displaced_robot_ids:
            self._pinned_slots.pop(displaced_robot_id, None)
        return self._schedule

    def reset(self) -> None:
        self._schedule = None
        self._pinned_slots.clear()


def _requests_by_robot(
    requests: Iterable[CorridorRequest],
) -> dict[str, CorridorRequest]:
    result: dict[str, CorridorRequest] = {}
    for request in requests:
        if request.robot_id in result:
            raise ValueError(
                f"duplicate corridor request for robot {request.robot_id!r}"
            )
        result[request.robot_id] = request
    return result


def _occupancies_by_robot(
    occupancies: Iterable[CorridorOccupancy],
) -> dict[str, CorridorOccupancy]:
    result: dict[str, CorridorOccupancy] = {}
    for occupancy in occupancies:
        if occupancy.robot_id in result:
            raise ValueError(
                f"duplicate corridor occupancy for robot {occupancy.robot_id!r}"
            )
        result[occupancy.robot_id] = occupancy
    return result


def _request_base_entry(request: CorridorRequest, now: float) -> float:
    entry = max(now, request.earliest_entry)
    if request.downstream_ready_at is not None:
        entry = max(entry, request.downstream_ready_at - request.duration_sec)
    return entry


def _same_passage(
    request: CorridorRequest,
    slot: CorridorSlot,
) -> bool:
    return (
        request.regions == slot.regions
        and tuple(
            (window.region_id, window.direction)
            for window in request.resource_windows
        )
        == tuple(
            (window.region_id, window.direction)
            for window in slot.resource_windows
        )
        and request.staging_lm == slot.staging_lm
        and request.exit_lm == slot.exit_lm
    )


def _requests_local_flow_match(
    first: CorridorRequest,
    second: CorridorRequest,
) -> bool:
    first_directions = {
        window.region_id: window.direction
        for window in first.resource_windows
    }
    second_directions = {
        window.region_id: window.direction
        for window in second.resource_windows
    }
    common = set(first_directions).intersection(second_directions)
    return bool(common) and all(
        first_directions[region_id] == second_directions[region_id]
        for region_id in common
    )


def _request_slot_local_flow_match(
    request: CorridorRequest,
    slot: CorridorSlot,
) -> bool:
    request_directions = {
        window.region_id: window.direction
        for window in request.resource_windows
    }
    slot_directions = {
        window.region_id: window.direction
        for window in slot.resource_windows
    }
    common = set(request_directions).intersection(slot_directions)
    return bool(common) and all(
        request_directions[region_id] == slot_directions[region_id]
        for region_id in common
    )


def _slots_local_flow_match(
    first: CorridorSlot,
    second: CorridorSlot,
) -> bool:
    first_directions = {
        window.region_id: window.direction
        for window in first.resource_windows
    }
    second_directions = {
        window.region_id: window.direction
        for window in second.resource_windows
    }
    common = set(first_directions).intersection(second_directions)
    return bool(common) and all(
        first_directions[region_id] == second_directions[region_id]
        for region_id in common
    )


def _has_pending_flow_predecessor(
    request: CorridorRequest,
    pending: Iterable[CorridorRequest],
) -> bool:
    request_regions = set(request.regions)
    request_key = (
        request.earliest_entry,
        -request.wait_age_sec,
        -request.priority,
        request.robot_id,
    )
    return any(
        other.robot_id != request.robot_id
        and other.predecessor_robot_id != request.robot_id
        and not request_regions.isdisjoint(other.regions)
        and _requests_local_flow_match(request, other)
        and (
            other.earliest_entry,
            -other.wait_age_sec,
            -other.priority,
            other.robot_id,
        )
        < request_key
        for other in pending
    )


def _phase_fairness_ranks(
    request: CorridorRequest,
    *,
    entry_time: float,
    slots: Iterable[CorridorSlot],
    pending: Iterable[CorridorRequest],
    config: CorridorSchedulerConfig,
) -> tuple[int, int, int]:
    pending_requests = tuple(pending)
    request_regions = set(request.regions)
    relevant_before = sorted(
        (
            slot
            for slot in slots
            if not request_regions.isdisjoint(slot.regions)
            and slot.entry_time <= entry_time + 1e-9
        ),
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    age_quantum_rank = -int(
        request.wait_age_sec // config.starvation_age_quantum_sec
    )
    if not relevant_before:
        return (0, age_quantum_rank, 0)

    active_slot = relevant_before[-1]
    active_run = 0
    for slot in reversed(relevant_before):
        if not _slots_local_flow_match(slot, active_slot):
            break
        active_run += 1
    opposing_starved_waits = [
        other
        for other in pending_requests
        if (
            other.robot_id != request.robot_id
            and other.wait_age_sec >= config.starvation_sec
            and not request_regions.isdisjoint(other.regions)
            and not _request_slot_local_flow_match(
                other,
                active_slot,
            )
        )
    ]
    direction_batch_limit = config.max_direction_batch
    if opposing_starved_waits:
        oldest_opposing = min(
            opposing_starved_waits,
            key=lambda other: (
                -other.wait_age_sec,
                (
                    other.deadline
                    if other.deadline is not None
                    else float("inf")
                ),
                -other.priority,
                other.robot_id,
            ),
        )
        direction_batch_limit = _adaptive_direction_batch_limit(
            active_slot,
            oldest_opposing,
            config,
        )
    force_other_phase = (
        active_run >= direction_batch_limit
        and bool(opposing_starved_waits)
    )
    same_phase = _request_slot_local_flow_match(
        request,
        active_slot,
    )
    forced_phase_rank = (
        1 if force_other_phase and same_phase else 0
    )
    phase_preference_rank = 0 if same_phase else 1
    if force_other_phase:
        phase_preference_rank = 0 if not same_phase else 1
    return (
        forced_phase_rank,
        age_quantum_rank,
        phase_preference_rank,
    )


def _adaptive_direction_batch_limit(
    active_slot: CorridorSlot,
    oldest_opposing: CorridorRequest,
    config: CorridorSchedulerConfig,
) -> int:
    """Return a throughput-aware, starvation-bounded phase length.

    A direction switch cannot occur until every shared local resource window
    used by ``active_slot`` is clear for ``oldest_opposing``.  That exact
    reversal cost can be tens of seconds for a passage which atomically crosses
    several authored rectangles.  In contrast, one more compatible convoy
    member moves the eventual switch by only ``headway_sec``.

    ``phase_amortization_sec`` limits the clearance overhead assigned to one
    robot.  The independent fairness cap spends at most
    ``max_phase_extension_sec`` beyond the base batch, and that allowance
    shrinks as the opposing request ages beyond ``starvation_sec``.
    """

    reversal_entry = _request_entry_after_slot(
        oldest_opposing,
        active_slot,
        config,
    )
    clearance_cost = max(
        config.headway_sec,
        reversal_entry - active_slot.entry_time,
    )
    amortization_window = max(
        config.headway_sec,
        config.phase_amortization_sec,
    )
    amortized_limit = max(
        config.max_direction_batch,
        int(ceil((clearance_cost / amortization_window) - 1e-12)),
    )

    age_over_starvation = max(
        0.0,
        oldest_opposing.wait_age_sec - config.starvation_sec,
    )
    extension_remaining = max(
        0.0,
        config.max_phase_extension_sec - age_over_starvation,
    )
    fairness_limit = (
        config.max_direction_batch
        + int(
            floor(
                (extension_remaining / config.headway_sec)
                + 1e-12
            )
        )
    )
    return max(
        config.max_direction_batch,
        min(
            config.max_adaptive_direction_batch,
            amortized_limit,
            fairness_limit,
        ),
    )


def _earliest_placement(
    request: CorridorRequest,
    *,
    slots: Iterable[CorridorSlot],
    now: float,
    horizon_end: float,
    config: CorridorSchedulerConfig,
) -> tuple[float, float, int] | None:
    relevant = sorted(
        (
            slot
            for slot in slots
            if not set(request.regions).isdisjoint(slot.regions)
        ),
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    candidate_entry = _request_base_entry(request, now)

    # Moving a conflicting passage forward can only resolve that conflict by
    # placing its shared resource window after the existing one.  Repeat
    # because the shift may reach another future window on a different
    # corridor resource.  The monotonic loop is deterministic and bounded by
    # the finite calendar; unlike a graph search it cannot oscillate.
    max_iterations = max(
        4,
        (len(relevant) + 1) * (len(request.regions) + 1),
    )
    for _ in range(max_iterations):
        if candidate_entry > horizon_end + 1e-9:
            return None
        required_entry = candidate_entry
        for slot in relevant:
            predecessor = (
                request.predecessor_robot_id == slot.robot_id
            )
            if (
                not predecessor
                and _request_slot_windows_fit(
                    request,
                    candidate_entry,
                    slot,
                    config,
                )
            ):
                continue
            required_entry = max(
                required_entry,
                _request_entry_after_slot(
                    request,
                    slot,
                    config,
                ),
            )
        if required_entry <= candidate_entry + 1e-9:
            previous, following = _calendar_neighbours(
                request,
                candidate_entry,
                relevant,
            )
            return (
                candidate_entry,
                candidate_entry + request.duration_sec,
                _phase_switches(
                    previous,
                    following,
                    request.direction,
                ),
            )
        candidate_entry = required_entry
    return None


def _placement_at(
    request: CorridorRequest,
    *,
    entry_time: float,
    slots: Iterable[CorridorSlot],
    now: float,
    horizon_end: float,
    config: CorridorSchedulerConfig,
) -> tuple[float, float, int] | None:
    if (
        entry_time + 1e-9 < _request_base_entry(request, now)
        or entry_time > horizon_end + 1e-9
    ):
        return None
    relevant = sorted(
        (
            slot
            for slot in slots
            if not set(request.regions).isdisjoint(slot.regions)
        ),
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    if any(
        not _request_slot_windows_fit(
            request,
            entry_time,
            slot,
            config,
        )
        for slot in relevant
    ):
        return None
    if request.predecessor_robot_id:
        predecessor = next(
            (
                slot
                for slot in relevant
                if slot.robot_id == request.predecessor_robot_id
            ),
            None,
        )
        if (
            predecessor is None
            or entry_time + 1e-9
            < _request_entry_after_slot(
                request,
                predecessor,
                config,
            )
        ):
            return None
    previous, following = _calendar_neighbours(
        request,
        entry_time,
        relevant,
    )
    return (
        entry_time,
        entry_time + request.duration_sec,
        _phase_switches(previous, following, request.direction),
    )


def _resource_window_map(
    windows: Iterable[CorridorResourceWindow],
    *,
    origin: float,
) -> dict[str, tuple[float, float, str]]:
    return {
        window.region_id: (
            origin + window.entry_offset_sec,
            origin + window.exit_offset_sec,
            window.direction,
        )
        for window in windows
    }


def _request_slot_windows_fit(
    request: CorridorRequest,
    entry_time: float,
    slot: CorridorSlot,
    config: CorridorSchedulerConfig,
) -> bool:
    return _resource_window_maps_fit(
        _resource_window_map(
            request.resource_windows,
            origin=entry_time,
        ),
        _resource_window_map(
            slot.resource_windows,
            origin=slot.entry_time,
        ),
        config,
    )


def _resource_window_maps_fit(
    first: Mapping[str, tuple[float, float, str]],
    second: Mapping[str, tuple[float, float, str]],
    config: CorridorSchedulerConfig,
) -> bool:
    guard = _direction_change_guard(config)
    for region_id in set(first).intersection(second):
        first_entry, first_exit, first_direction = first[region_id]
        second_entry, second_exit, second_direction = second[region_id]
        if first_direction == second_direction:
            first_is_after = (
                first_entry
                >= second_entry + config.headway_sec - 1e-9
                and first_exit
                >= second_exit + config.headway_sec - 1e-9
            )
            first_is_before = (
                first_entry + config.headway_sec
                <= second_entry + 1e-9
                and first_exit + config.headway_sec
                <= second_exit + 1e-9
            )
            if not first_is_after and not first_is_before:
                return False
            continue
        if not (
            first_exit + guard <= second_entry + 1e-9
            or first_entry >= second_exit + guard - 1e-9
        ):
            return False
    return True


def _request_entry_after_slot(
    request: CorridorRequest,
    slot: CorridorSlot,
    config: CorridorSchedulerConfig,
) -> float:
    request_windows = {
        window.region_id: window
        for window in request.resource_windows
    }
    slot_windows = _resource_window_map(
        slot.resource_windows,
        origin=slot.entry_time,
    )
    required = float("-inf")
    guard = _direction_change_guard(config)
    for region_id in set(request_windows).intersection(slot_windows):
        request_window = request_windows[region_id]
        slot_entry, slot_exit, slot_direction = slot_windows[region_id]
        if request_window.direction == slot_direction:
            required = max(
                required,
                slot_entry
                + config.headway_sec
                - request_window.entry_offset_sec,
                slot_exit
                + config.headway_sec
                - request_window.exit_offset_sec,
            )
        else:
            required = max(
                required,
                slot_exit
                + guard
                - request_window.entry_offset_sec,
            )
    return required


def _calendar_neighbours(
    request: CorridorRequest,
    entry_time: float,
    slots: Iterable[CorridorSlot],
) -> tuple[CorridorSlot | None, CorridorSlot | None]:
    relevant = sorted(
        (
            slot
            for slot in slots
            if not set(request.regions).isdisjoint(slot.regions)
        ),
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    previous: CorridorSlot | None = None
    following: CorridorSlot | None = None
    for slot in relevant:
        if slot.entry_time <= entry_time + 1e-9:
            previous = slot
        elif following is None:
            following = slot
    return previous, following


def _slot_interval_fits(
    candidate: CorridorSlot,
    slots: Iterable[CorridorSlot],
    config: CorridorSchedulerConfig,
) -> bool:
    for other in slots:
        if set(candidate.regions).isdisjoint(other.regions):
            continue
        if not _resource_window_maps_fit(
            _resource_window_map(
                candidate.resource_windows,
                origin=candidate.entry_time,
            ),
            _resource_window_map(
                other.resource_windows,
                origin=other.entry_time,
            ),
            config,
        ):
            return False
    return True


def _direction_change_guard(config: CorridorSchedulerConfig) -> float:
    return max(config.headway_sec, config.direction_change_sec)


def _phase_switches(
    previous: CorridorSlot | None,
    following: CorridorSlot | None,
    direction: str,
) -> int:
    switches = 0
    if previous is not None and previous.direction != direction:
        switches += 1
    if following is not None and following.direction != direction:
        switches += 1
    return switches


def _schedule_fingerprint(
    slots: Iterable[CorridorSlot],
) -> tuple[tuple[object, ...], ...]:
    return tuple(
        (
            slot.robot_id,
            slot.regions,
            slot.direction,
            round(slot.entry_time, 9),
            round(slot.exit_time, 9),
            slot.staging_lm,
            slot.exit_lm,
            slot.route_revision,
            slot.state.value,
            tuple(
                (
                    window.region_id,
                    round(window.entry_offset_sec, 9),
                    round(window.exit_offset_sec, 9),
                    window.direction,
                )
                for window in slot.resource_windows
            ),
            slot.past_commit_point,
            slot.physically_observed,
        )
        for slot in slots
    )
