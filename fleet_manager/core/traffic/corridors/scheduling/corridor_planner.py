"""Deterministic policy for building controlled-corridor schedules."""

from __future__ import annotations

from math import ceil, floor, isfinite
from typing import Collection, Iterable

from fleet_manager.core.traffic.corridors.scheduling.corridor_calendar import (
    CorridorCalendar,
    request_entry_after_slot as _request_entry_after_slot,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorDecision,
    CorridorDecisionStatus,
    CorridorOccupancy,
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSchedule,
    CorridorSchedulerConfig,
    CorridorSlot,
    CorridorSlotState,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_planning_models import (
    _PlanningContext,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_retention import (
    retain_previous_slots as _retain_previous_slots,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_scheduling import (
    schedule_pending as _schedule_pending_requests,
)


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
    """Rank a proposal inside the active, starvation-bounded direction phase."""
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


def _schedule_fingerprint(
    slots: Iterable[CorridorSlot],
) -> tuple[tuple[object, ...], ...]:
    """Return the stable semantic identity used to advance the public epoch."""
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




class CorridorScheduleBuilder:
    """Compose validation, fairness policy and calendar placement."""

    def __init__(
        self,
        config: CorridorSchedulerConfig | None = None,
    ) -> None:
        self._config = config or CorridorSchedulerConfig()

    def build(
        self,
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

        context = self._prepare_context(
            requests,
            controlled_regions=controlled_regions,
            now=now,
            config=config,
            occupancies=occupancies,
            pinned_slots=pinned_slots,
        )
        self._add_physical_occupancies(context)
        self._retain_previous_slots(context, previous)
        pending, preferred_entries = self._collect_pending(
            context,
            previous,
        )
        self._schedule_pending(context, pending, preferred_entries)
        return self._publish(context, pending, previous)

    def _prepare_context(
        self,
        requests: Iterable[CorridorRequest],
        *,
        controlled_regions: Collection[str],
        now: float,
        config: CorridorSchedulerConfig | None,
        occupancies: Iterable[CorridorOccupancy] | None,
        pinned_slots: Iterable[CorridorSlot] | None,
    ) -> _PlanningContext:
        if not isfinite(now):
            raise ValueError("now must be finite")
        policy = config or self._config
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
        return _PlanningContext(
            policy=policy,
            calendar=CorridorCalendar(policy),
            now=now,
            horizon_end=now + policy.horizon_sec,
            explicit_regions=explicit_regions,
            request_by_robot=request_by_robot,
            occupancy_snapshot_provided=occupancy_snapshot_provided,
            occupancy_by_robot=occupancy_by_robot,
            pinned_by_robot=pinned_by_robot,
            physically_occupied_robots=frozenset(occupancy_by_robot),
        )

    @staticmethod
    def _add_physical_occupancies(context: _PlanningContext) -> None:
        """Install authoritative footprint observations before all proposals."""
        policy = context.policy
        now = context.now
        explicit_regions = context.explicit_regions
        occupancy_by_robot = context.occupancy_by_robot
        slots = context.slots
        decisions = context.decisions

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


    @staticmethod
    def _retain_previous_slots(
        context: _PlanningContext,
        previous: CorridorSchedule | None,
    ) -> None:
        """Keep commitments and valid worker pins from the previous calendar."""
        _retain_previous_slots(
            context,
            previous,
            same_passage=_same_passage,
        )

    @staticmethod
    def _collect_pending(
        context: _PlanningContext,
        previous: CorridorSchedule | None,
    ) -> tuple[list[CorridorRequest], dict[str, float]]:
        """Classify requests and retain tentative starts as soft preferences."""
        explicit_regions = context.explicit_regions
        request_by_robot = context.request_by_robot
        physically_occupied_robots = context.physically_occupied_robots
        cleared_passage_robots = context.cleared_passage_robots
        slots = context.slots
        decisions = context.decisions

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

        return pending, preferred_entries

    @staticmethod
    def _schedule_pending(
        context: _PlanningContext,
        pending: list[CorridorRequest],
        preferred_entries: dict[str, float],
    ) -> None:
        """Choose and place one best safe proposal until no request can fit."""
        _schedule_pending_requests(
            context,
            pending,
            preferred_entries,
            has_flow_predecessor=_has_pending_flow_predecessor,
            phase_fairness_ranks=_phase_fairness_ranks,
        )

    @staticmethod
    def _publish(
        context: _PlanningContext,
        pending: list[CorridorRequest],
        previous: CorridorSchedule | None,
    ) -> CorridorSchedule:
        """Finalize deferred decisions and derive the stable schedule epoch."""
        calendar = context.calendar
        now = context.now
        horizon_end = context.horizon_end
        slots = context.slots
        decisions = context.decisions

        for request in pending:
            reason = (
                "earliest entry is outside rolling horizon"
                if calendar.request_base_entry(request, now) > horizon_end
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
