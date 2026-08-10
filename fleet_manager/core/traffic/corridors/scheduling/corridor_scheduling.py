"""Proposal, ranking and placement stages for pending corridor requests."""

from __future__ import annotations

from collections.abc import Callable, Iterable

from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorDecision,
    CorridorDecisionStatus,
    CorridorRequest,
    CorridorSchedulerConfig,
    CorridorSlot,
    CorridorSlotState,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_planning_models import (
    _PlacementProposal,
    _PlanningContext,
)


_HasFlowPredecessor = Callable[
    [
        CorridorRequest,
        Iterable[CorridorRequest],
        dict[str, dict[str, str]],
    ],
    bool,
]
_PhaseFairnessRanks = Callable[..., tuple[int, int, int]]


def schedule_pending(
    context: _PlanningContext,
    pending: list[CorridorRequest],
    preferred_entries: dict[str, float],
    *,
    has_flow_predecessor: _HasFlowPredecessor,
    phase_fairness_ranks: _PhaseFairnessRanks,
) -> None:
    """Choose and place safe proposals in independent resource components."""
    original_order = {
        request.robot_id: index
        for index, request in enumerate(pending)
    }
    flow_directions = {
        request.robot_id: {
            window.region_id: window.direction
            for window in request.resource_windows
        }
        for request in pending
    }
    remaining: list[CorridorRequest] = []
    for component in _pending_resource_components(pending):
        _schedule_pending_component(
            context,
            component,
            preferred_entries,
            flow_directions,
            has_flow_predecessor=has_flow_predecessor,
            phase_fairness_ranks=phase_fairness_ranks,
        )
        remaining.extend(component)
    pending[:] = sorted(
        remaining,
        key=lambda request: original_order[request.robot_id],
    )


def _schedule_pending_component(
    context: _PlanningContext,
    pending: list[CorridorRequest],
    preferred_entries: dict[str, float],
    flow_directions: dict[str, dict[str, str]],
    *,
    has_flow_predecessor: _HasFlowPredecessor,
    phase_fairness_ranks: _PhaseFairnessRanks,
) -> None:
    """Run the existing deterministic placement loop for one component."""
    while pending:
        pending_robot_ids = {
            request.robot_id
            for request in pending
        }
        scheduled_robot_ids = {
            slot.robot_id
            for slot in context.slots
        }
        proposals: list[_PlacementProposal] = []
        for request in pending:
            if not _direct_predecessor_is_ready(
                context,
                request,
                pending_robot_ids,
                scheduled_robot_ids,
            ):
                continue
            if has_flow_predecessor(
                request,
                pending,
                flow_directions,
            ):
                continue
            proposal = _build_proposal(
                context,
                request,
                pending,
                preferred_entries,
                phase_fairness_ranks=phase_fairness_ranks,
            )
            if proposal is not None:
                proposals.append(proposal)

        if not proposals:
            break
        selected = min(proposals, key=lambda proposal: proposal.rank)
        _grant_proposal(context, selected)
        pending.remove(selected.request)


def _pending_resource_components(
    pending: list[CorridorRequest],
) -> list[list[CorridorRequest]]:
    """Group requests which can influence each other's calendar placement.

    Requests using disjoint authored resources cannot change one another's
    slot, phase fairness or predecessor order. Scheduling a 100-robot fleet as
    one global proposal loop repeated the same work for every independent
    corridor and made the runtime tick cubic in fleet size.
    """
    if len(pending) < 2:
        return [list(pending)] if pending else []

    by_name = {request.robot_id: request for request in pending}
    neighbours = {request.robot_id: set() for request in pending}
    users_by_region: dict[str, list[str]] = {}
    for request in pending:
        for region_id in request.regions:
            users_by_region.setdefault(region_id, []).append(
                request.robot_id
            )
        predecessor = request.predecessor_robot_id
        if predecessor and predecessor in by_name:
            neighbours[request.robot_id].add(predecessor)
            neighbours[predecessor].add(request.robot_id)
    for users in users_by_region.values():
        if len(users) < 2:
            continue
        first = users[0]
        for other in users[1:]:
            neighbours[first].add(other)
            neighbours[other].add(first)

    components: list[list[CorridorRequest]] = []
    visited: set[str] = set()
    order_index = {
        request.robot_id: index
        for index, request in enumerate(pending)
    }
    for request in pending:
        if request.robot_id in visited:
            continue
        names: list[str] = []
        queue = [request.robot_id]
        visited.add(request.robot_id)
        while queue:
            name = queue.pop(0)
            names.append(name)
            additions = sorted(
                neighbours[name] - visited,
                key=order_index.get,
            )
            visited.update(additions)
            queue.extend(additions)
        components.append([by_name[name] for name in names])
    return components


def _direct_predecessor_is_ready(
    context: _PlanningContext,
    request: CorridorRequest,
    pending_robot_ids: set[str],
    scheduled_robot_ids: set[str],
) -> bool:
    """Prevent a physical queue follower from bypassing its leader."""
    predecessor = request.predecessor_robot_id
    if not predecessor:
        return True
    if predecessor in pending_robot_ids:
        return False
    predecessor_is_known = (
        predecessor in context.request_by_robot
        or predecessor in context.occupancy_by_robot
    )
    return not (
        predecessor_is_known
        and predecessor not in scheduled_robot_ids
    )


def _build_proposal(
    context: _PlanningContext,
    request: CorridorRequest,
    pending: list[CorridorRequest],
    preferred_entries: dict[str, float],
    *,
    phase_fairness_ranks: _PhaseFairnessRanks,
) -> _PlacementProposal | None:
    """Find one request placement and derive its complete stable rank."""
    placement = context.calendar.earliest_placement(
        request,
        slots=context.slots,
        now=context.now,
        horizon_end=context.horizon_end,
    )
    if placement is None:
        return None

    entry_time, exit_time, phase_switches = placement
    policy = context.policy
    preferred_entry = preferred_entries.get(request.robot_id)
    if preferred_entry is not None:
        preferred_placement = context.calendar.placement_at(
            request,
            entry_time=preferred_entry,
            slots=context.slots,
            now=context.now,
            horizon_end=context.horizon_end,
        )
        if (
            preferred_placement is not None
            and preferred_placement[1]
            <= exit_time + policy.tentative_change_penalty_sec
        ):
            placement = preferred_placement
            entry_time, exit_time, phase_switches = placement

    effective_finish = _effective_finish(
        request,
        exit_time,
        phase_switches,
        policy,
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
    ) = phase_fairness_ranks(
        request,
        entry_time=entry_time,
        slots=context.slots,
        pending=pending,
        config=policy,
    )
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
    return _PlacementProposal(
        rank=(
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
        request=request,
        placement=placement,
    )


def _effective_finish(
    request: CorridorRequest,
    exit_time: float,
    phase_switches: int,
    policy: CorridorSchedulerConfig,
) -> float:
    """Apply switch, priority and bounded wait-age costs."""
    return (
        exit_time
        + (phase_switches * policy.direction_switch_cost_sec)
        - (request.priority * policy.priority_cost_sec)
        - (
            min(request.wait_age_sec, policy.starvation_sec)
            * policy.wait_age_cost_sec
        )
    )


def _grant_proposal(
    context: _PlanningContext,
    proposal: _PlacementProposal,
) -> None:
    """Materialize one selected proposal and publish its decision."""
    request = proposal.request
    entry_time, exit_time, _ = proposal.placement
    state = (
        CorridorSlotState.COMMITTED
        if request.entered
        or request.past_commit_point
        or (
            not request.requires_explicit_commit
            and entry_time
            <= context.now + context.policy.commit_horizon_sec
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
    context.slots.append(slot)
    context.decisions[request.robot_id] = CorridorDecision(
        robot_id=request.robot_id,
        status=CorridorDecisionStatus.GRANTED,
        reason=(
            "committed inside execution horizon"
            if state is CorridorSlotState.COMMITTED
            else "tentative rolling slot"
        ),
        slot=slot,
    )
