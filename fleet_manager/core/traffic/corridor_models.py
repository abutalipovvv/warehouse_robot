"""Immutable contracts for controlled-corridor scheduling."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite, ulp
from typing import Hashable, Mapping


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


__all__ = [
    "CorridorDecision",
    "CorridorDecisionStatus",
    "CorridorOccupancy",
    "CorridorRequest",
    "CorridorResourceWindow",
    "CorridorSchedule",
    "CorridorSchedulerConfig",
    "CorridorSlot",
    "CorridorSlotState",
    "RouteRevision",
]
