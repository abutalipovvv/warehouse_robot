"""Retention stages for committed and worker-pinned corridor slots."""

from __future__ import annotations

from collections.abc import Callable

from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSchedule,
    CorridorSlot,
    CorridorSlotState,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_planning_models import (
    _PlanningContext,
)


_SamePassage = Callable[[CorridorRequest, CorridorSlot], bool]


def retain_previous_slots(
    context: _PlanningContext,
    previous: CorridorSchedule | None,
    *,
    same_passage: _SamePassage,
) -> None:
    """Retain every still-authoritative slot in original calendar order."""
    if previous is None:
        return

    for old_slot in previous.slots:
        retained_slot = _retained_previous_slot(
            context,
            old_slot,
            same_passage=same_passage,
        )
        if retained_slot is not None:
            context.slots.append(retained_slot)


def _retained_previous_slot(
    context: _PlanningContext,
    old_slot: CorridorSlot,
    *,
    same_passage: _SamePassage,
) -> CorridorSlot | None:
    """Classify and materialize one still-authoritative old slot."""
    if old_slot.robot_id in context.physically_occupied_robots:
        return None
    if not context.calendar.slot_interval_fits(
        old_slot,
        context.slots,
    ):
        # Physical occupancy invalidates a not-yet-entered command.
        # Exact local resource windows keep independent bundle sections.
        return None

    now = context.now
    request = context.request_by_robot.get(old_slot.robot_id)
    passage_matches = (
        request is not None
        and same_passage(request, old_slot)
    )
    confirmed_physical_absence = (
        context.occupancy_snapshot_provided
        and old_slot.physically_observed
        and old_slot.robot_id
        not in context.physically_occupied_robots
    )
    if confirmed_physical_absence and old_slot.exit_time <= now:
        if passage_matches:
            context.cleared_passage_robots.add(old_slot.robot_id)
        return None

    same_revision = (
        passage_matches
        and request is not None
        and old_slot.route_revision == request.route_revision
    )
    pinned_tentative = bool(
        old_slot.state is CorridorSlotState.TENTATIVE
        and context.pinned_by_robot.get(old_slot.robot_id) == old_slot
        and same_revision
        and request is not None
        and request.requires_explicit_commit
        and request.downstream_available
    )
    past_commit_authority = (
        passage_matches
        and request is not None
        and request.past_commit_point
        and not confirmed_physical_absence
    )
    if _retention_blocked_before_entry(
        context,
        old_slot,
        request=request,
        passage_matches=passage_matches,
        pinned_tentative=pinned_tentative,
        past_commit_authority=past_commit_authority,
        confirmed_physical_absence=confirmed_physical_absence,
    ):
        return None
    if not _retention_state_is_live(
        context,
        old_slot,
        request=request,
        passage_matches=passage_matches,
        same_revision=same_revision,
        pinned_tentative=pinned_tentative,
        past_commit_authority=past_commit_authority,
    ):
        return None

    rebase_unobserved_authority = bool(
        past_commit_authority
        and request is not None
        and not old_slot.physically_observed
        and old_slot.exit_time <= now
    )
    retained_slot = _materialize_retained_slot(
        context,
        old_slot,
        request=request,
        same_passage=passage_matches,
        pinned_tentative=pinned_tentative,
        past_commit_authority=past_commit_authority,
        rebase_unobserved_authority=(
            rebase_unobserved_authority
        ),
    )
    if (
        not past_commit_authority
        and not context.calendar.slot_interval_fits(
            retained_slot,
            context.slots,
        )
    ):
        return None
    return retained_slot


def _retention_blocked_before_entry(
    context: _PlanningContext,
    old_slot: CorridorSlot,
    *,
    request: CorridorRequest | None,
    passage_matches: bool,
    pinned_tentative: bool,
    past_commit_authority: bool,
    confirmed_physical_absence: bool,
) -> bool:
    """Reject a missed command, queue overtake or revoked authority."""
    missed_pre_entry_commit = bool(
        context.occupancy_snapshot_provided
        and passage_matches
        and request is not None
        and not request.entered
        and not request.past_commit_point
        and old_slot.robot_id
        not in context.physically_occupied_robots
        and request.earliest_entry
        > (
            old_slot.entry_time
            + context.policy.occupancy_recheck_sec
        )
    )
    if missed_pre_entry_commit and not pinned_tentative:
        # A fresh pose and ETA prove that this pre-entry command was missed.
        return True

    queue_predecessor = (
        request.predecessor_robot_id
        if request is not None
        else None
    )
    if (
        passage_matches
        and request is not None
        and not request.entered
        and queue_predecessor
        and (
            queue_predecessor in context.request_by_robot
            or queue_predecessor in context.occupancy_by_robot
        )
    ):
        # A follower cannot retain a slot ahead of its observed leader.
        return True

    return bool(
        old_slot.past_commit_point
        and request is not None
        and not past_commit_authority
        and not confirmed_physical_absence
    )


def _retention_state_is_live(
    context: _PlanningContext,
    old_slot: CorridorSlot,
    *,
    request: CorridorRequest | None,
    passage_matches: bool,
    same_revision: bool,
    pinned_tentative: bool,
    past_commit_authority: bool,
) -> bool:
    """Accept only live committed, promotable or explicitly pinned state."""
    promotable_tentative = (
        old_slot.state is CorridorSlotState.TENTATIVE
        and same_revision
        and request is not None
        and not request.requires_explicit_commit
        and request.downstream_available
        and context.calendar.request_base_entry(
            request,
            context.now,
        )
        <= old_slot.entry_time
        and (
            request.past_commit_point
            or old_slot.entry_time
            <= (
                context.now
                + context.policy.commit_horizon_sec
            )
        )
    )
    retained_without_snapshot = (
        request is None
        and old_slot.past_commit_point
        and old_slot.entry_time <= context.now
        and (
            not context.occupancy_snapshot_provided
            or old_slot.physically_observed
        )
    )
    if not passage_matches and not retained_without_snapshot:
        return False

    retained_state = (
        old_slot.state is CorridorSlotState.COMMITTED
        or promotable_tentative
        or pinned_tentative
    )
    still_authoritative = (
        old_slot.exit_time > context.now
        or past_commit_authority
    )
    request_is_live = (
        old_slot.robot_id in context.request_by_robot
        or old_slot.entry_time <= context.now
    )
    return bool(
        retained_state
        and still_authoritative
        and request_is_live
    )


def _materialize_retained_slot(
    context: _PlanningContext,
    old_slot: CorridorSlot,
    *,
    request: CorridorRequest | None,
    same_passage: bool,
    pinned_tentative: bool,
    past_commit_authority: bool,
    rebase_unobserved_authority: bool,
) -> CorridorSlot:
    """Rebuild one retained slot, rebasing stale unobserved authority."""
    rebase = rebase_unobserved_authority
    retained_entry_time = (
        max(context.now, request.earliest_entry)
        if rebase and request is not None
        else old_slot.entry_time
    )
    authority_window_origin = (
        min(
            window.entry_offset_sec
            for window in request.resource_windows
        )
        if rebase and request is not None
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
        if rebase and request is not None
        else ()
    )
    retained_exit_time = (
        retained_entry_time
        + max(
            window.exit_offset_sec
            for window in rebased_authority_windows
        )
        if rebase
        else (
            max(
                old_slot.exit_time,
                context.now
                + context.policy.occupancy_recheck_sec,
            )
            if past_commit_authority
            else old_slot.exit_time
        )
    )
    retained_windows = (
        rebased_authority_windows
        if rebase
        else old_slot.resource_windows
    )
    return CorridorSlot(
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
