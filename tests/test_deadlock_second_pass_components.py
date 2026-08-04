from __future__ import annotations

import inspect
from types import SimpleNamespace

from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.traffic.deadlocks.recovery.cycles.deadlock_cycle_recovery import (
    WaitCycleRecoveryMixin,
)
from fleet_manager.manager.traffic.deadlocks.recovery.cycles.deadlock_cycle_recovery_models import (
    _WaitCycleDecisionContext,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_candidate_models import (
    _EvacuationSearchContext,
)
from fleet_manager.manager.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_candidates import (
    EvacuationCandidateMixin,
)


def test_candidate_search_context_preserves_expanded_component_order() -> None:
    winner = FleetRobot(name="winner", current_lm="A")
    loser = FleetRobot(name="loser", current_lm="B")
    tail = FleetRobot(name="tail", current_lm="C")
    owner = EvacuationCandidateMixin.__new__(
        EvacuationCandidateMixin
    )
    calls: list[tuple[tuple[str, ...], str]] = []

    def expand(robots, selected):
        calls.append((tuple(robot.name for robot in robots), selected.name))
        return [*robots, tail], {
            selected.name: -1,
            loser.name: 0,
            tail.name: 1,
        }

    owner._controlled_corridor_portal_queue_component = expand

    context = owner._evacuation_search_context(
        [winner, loser],
        winner,
        {"region-a"},
    )

    assert isinstance(context, _EvacuationSearchContext)
    assert [robot.name for robot in context.robots] == [
        "winner",
        "loser",
        "tail",
    ]
    assert context.portal_queue_depths == {
        "winner": -1,
        "loser": 0,
        "tail": 1,
    }
    assert calls == [(("winner", "loser"), "winner")]


def test_cycle_priority_key_keeps_geometry_first_tie_break_order() -> None:
    first = FleetRobot(
        name="first",
        current_lm="A",
        blocked_since=2.0,
        traffic_priority_until=4.0,
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        blocked_since=1.0,
        traffic_priority_until=3.0,
    )
    owner = WaitCycleRecoveryMixin.__new__(WaitCycleRecoveryMixin)
    owner._active_order_for_robot = lambda robot: SimpleNamespace(
        priority={"first": 10, "second": 1}[robot.name],
    )
    owner._cycle_forward_clearance = lambda robot, _robots: {
        "first": 1.0,
        "second": 3.0,
    }[robot.name]

    selected = min(
        [first, second],
        key=lambda robot: owner._wait_cycle_priority_key(
            robot,
            [first, second],
            8.0,
        ),
    )

    assert selected is second


def test_second_pass_stages_are_bounded_and_context_is_slotted() -> None:
    candidate_stages = (
        EvacuationCandidateMixin._build_deadlock_evacuation_candidates,
        EvacuationCandidateMixin._evacuation_search_context,
        EvacuationCandidateMixin._evacuation_candidate_probe,
        EvacuationCandidateMixin._candidate_has_reciprocal_blocker,
        EvacuationCandidateMixin._audit_evacuation_candidate,
        EvacuationCandidateMixin._build_candidate_graph_escape,
    )
    cycle_stages = (
        WaitCycleRecoveryMixin._break_runtime_wait_cycle,
        WaitCycleRecoveryMixin._wait_cycle_decision_context,
        WaitCycleRecoveryMixin._wait_cycle_priority_key,
        WaitCycleRecoveryMixin._wait_cycle_arbitration_due,
        WaitCycleRecoveryMixin._escalate_wait_cycle_recovery,
        WaitCycleRecoveryMixin._grant_wait_cycle_priority,
    )

    assert max(
        len(inspect.getsourcelines(stage)[0])
        for stage in (*candidate_stages, *cycle_stages)
    ) <= 130
    assert _WaitCycleDecisionContext.__slots__
