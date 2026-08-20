from __future__ import annotations

import ast
import inspect
from pathlib import Path

from fleet_manager.manager.tasks.planning_jobs import (
    AsyncPlanningJobMixin,
    _CoupledReplanContext,
    _CoupledReplanMember,
)
from fleet_manager.robot.model import FleetRobot


def test_coupled_replan_hook_keeps_stable_signature() -> None:
    signature = inspect.signature(
        AsyncPlanningJobMixin._start_async_coupled_replan
    )
    assert tuple(signature.parameters) == (
        "self",
        "robots",
        "winner",
        "now",
    )
    assert signature.return_annotation == "bool"


def test_coupled_replan_stage_models_express_mutability() -> None:
    assert _CoupledReplanMember.__dataclass_params__.frozen
    assert "__slots__" in _CoupledReplanMember.__dict__
    assert "__dict__" not in _CoupledReplanMember.__dict__

    assert not _CoupledReplanContext.__dataclass_params__.frozen
    assert "__slots__" in _CoupledReplanContext.__dict__
    assert "__dict__" not in _CoupledReplanContext.__dict__


def test_coupled_replan_orchestration_stays_in_focused_stages() -> None:
    source = (
        Path(__file__).parents[1]
        / "fleet_manager/manager/tasks/planning_jobs.py"
    )
    tree = ast.parse(source.read_text())
    methods = {
        node.name: node
        for parent in tree.body
        if isinstance(parent, ast.ClassDef)
        and parent.name == "AsyncPlanningJobMixin"
        for node in parent.body
        if isinstance(node, ast.FunctionDef)
        and "coupled_replan" in node.name
    }
    expected = {
        "_start_async_coupled_replan",
        "_coupled_replan_context",
        "_coupled_replan_is_eligible",
        "_coupled_replan_is_overdue",
        "_coupled_replan_yields_planner_turn",
        "_record_coupled_replan_start_failure",
        "_build_coupled_replan_requests",
        "_validated_coupled_replan_member",
        "_append_coupled_replan_request",
        "_build_coupled_replan_job",
        "_submit_coupled_replan_job",
    }
    assert set(methods) == expected
    assert methods["_start_async_coupled_replan"].end_lineno - (
        methods["_start_async_coupled_replan"].lineno
    ) + 1 <= 20
    assert max(
        method.end_lineno - method.lineno + 1
        for method in methods.values()
    ) <= 80


def test_coupled_replan_fairness_guards_preserve_short_circuit_order() -> None:
    class Harness(AsyncPlanningJobMixin):
        def __init__(
            self,
            last_kind: str,
            *,
            urgent: bool,
            queued: bool,
        ) -> None:
            self._last_async_job_kind = last_kind
            self.urgent = urgent
            self.queued = queued
            self.calls: list[str] = []

        def _ready_rolling_prefetch_entries(self) -> list[tuple[str, float]]:
            self.calls.append("prefetch")
            return [("robot", 1.0)] if self.urgent else []

        def _rolling_prefetch_urgent_lead(self) -> float:
            self.calls.append("lead")
            return 2.0

        def _queued_simulated_dispatch_waiting(self, now: float) -> bool:
            assert now == 10.0
            self.calls.append("dispatch")
            return self.queued

    urgent = Harness("coupled_replan", urgent=True, queued=True)
    assert urgent._coupled_replan_yields_planner_turn(10.0)
    assert urgent.calls == ["prefetch", "lead"]

    queued = Harness("prefetch", urgent=True, queued=True)
    assert queued._coupled_replan_yields_planner_turn(10.0)
    assert queued.calls == ["dispatch"]

    dispatch = Harness("dispatch", urgent=False, queued=True)
    assert not dispatch._coupled_replan_yields_planner_turn(10.0)
    assert dispatch.calls == ["prefetch"]


def test_overdue_coupled_replan_cannot_be_starved_by_other_job_kinds() -> None:
    instance = AsyncPlanningJobMixin.__new__(AsyncPlanningJobMixin)
    instance._deadlock_retreat_after = lambda: 4.5
    now = 100.0
    robots = [
        FleetRobot(
            name=name,
            current_lm=name,
            blocked_since=now - age,
            traffic_stall_since=now - age,
        )
        for name, age in (("first", 6.0), ("second", 5.0))
    ]
    context = _CoupledReplanContext(
        robots=robots,
        winner=robots[0],
        now=now,
        cycle_key=("first", "second"),
    )

    assert instance._coupled_replan_is_overdue(context)

    robots[0].traffic_stall_since = now - 1.0
    robots[0].blocked_since = now - 1.0
    robots[1].traffic_stall_since = now - 2.0
    robots[1].blocked_since = now - 2.0

    assert not instance._coupled_replan_is_overdue(context)


def test_coupled_replan_start_failure_is_idempotent() -> None:
    instance = AsyncPlanningJobMixin.__new__(AsyncPlanningJobMixin)
    instance._coupled_replan_failures = {}
    instance.traffic_metrics = {"coupledReplansFailed": 0}
    events: list[tuple[str, str]] = []
    instance._event = lambda level, message: events.append((level, message))
    cycle = ("a", "b")

    instance._record_coupled_replan_start_failure(
        cycle,
        event_message="first",
    )
    instance._record_coupled_replan_start_failure(
        cycle,
        event_message="second",
    )

    assert instance._coupled_replan_failures == {cycle: 1}
    assert instance.traffic_metrics["coupledReplansFailed"] == 1
    assert events == [("warn", "first")]
