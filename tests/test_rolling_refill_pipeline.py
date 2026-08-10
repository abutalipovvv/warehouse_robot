from __future__ import annotations

from threading import Event
from types import SimpleNamespace
from typing import Any

import pytest

from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.manager.planning import (
    PlanCandidate,
    PlanningJobRecord,
    PlanningJobStatus,
    PlanningPriority,
)
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.manager.tasks.rolling_continuation import (
    RollingAppendStatus,
    RollingBufferPolicy,
    RollingRefillUrgency,
)
from fleet_manager.robot.model import FleetRobot
from fleet_manager.runtime.simulation.manager import FleetManagerSim
from operator_app.core.fleet_benchmark_runner import (
    FleetBenchmarkRunner,
    FleetBenchmarkScenario,
    SCENARIOS,
)


def test_route_buffer_seconds_has_one_authoritative_calculation() -> None:
    robot = FleetRobot(
        name="robot",
        current_lm="N0",
        route_clock=3.25,
        trajectory=[{"t": 0.0}, {"t": 9.5}],
    )

    assert robot.route_buffer_seconds == pytest.approx(6.25)
    robot.route_clock = 12.0
    assert robot.route_buffer_seconds == 0.0
    robot.trajectory = []
    assert robot.route_buffer_seconds == 0.0


def test_buffer_urgency_and_staggering_are_deterministic() -> None:
    policy = _buffer_policy()

    assert policy.urgency(100.0) is RollingRefillUrgency.HEALTHY
    assert policy.urgency(55.0) is RollingRefillUrgency.NORMAL
    assert policy.urgency(25.0) is RollingRefillUrgency.URGENT
    assert policy.urgency(10.0) is RollingRefillUrgency.CRITICAL
    assert policy.urgency(3.0) is RollingRefillUrgency.EMPTY

    first = [policy.normal_threshold(f"robot-{index:03d}") for index in range(100)]
    second = [policy.normal_threshold(f"robot-{index:03d}") for index in range(100)]
    assert first == second
    assert len({round(value, 3) for value in first}) > 90
    assert min(first) >= policy.refill_sec
    assert max(first) <= policy.refill_sec + policy.stagger_window_sec
    assert all(
        policy.urgency_for(f"robot-{index:03d}", 9.0)
        is RollingRefillUrgency.CRITICAL
        for index in range(100)
    )


def test_live_refill_job_does_not_block_selection_of_next_robot(
    monkeypatch,
) -> None:
    manager = _line_manager(3)
    first = _refill_entry(manager, "first", "N0", "N1", ["N0", "N1", "N2"])
    second = _refill_entry(manager, "second", "N2", "N3", ["N2", "N3"])
    record = PlanningJobRecord(kind="prefetch")
    record.job = SimpleNamespace(robot_ids=(first[1].name,))
    manager.planning_state.jobs["prefetch-first"] = record
    monkeypatch.setattr(
        manager._rolling_continuation_service,
        "select_candidates",
        lambda: [first, second],
    )

    try:
        selected = manager._rolling_prefetch_candidates()
        assert [entry[1].name for entry in selected] == [second[1].name]
    finally:
        manager.planning_state.jobs.clear()
        manager.close()


def test_moving_emergency_refill_precedes_a_safe_hold(
    monkeypatch,
) -> None:
    manager = _line_manager(3)
    safe_hold = _refill_entry(
        manager,
        "safe-hold",
        "N0",
        "N1",
        ["N0", "N1", "N3"],
    )
    emergency = _refill_entry(
        manager,
        "emergency",
        "N1",
        "N2",
        ["N1", "N2", "N3"],
    )
    safe_hold[1].status = "WAITING"
    safe_hold[1].route_clock = 10.0
    emergency[1].route_clock = 8.0
    monkeypatch.setattr(
        manager._rolling_continuation_service,
        "select_candidates",
        lambda: [safe_hold, emergency],
    )
    monkeypatch.setattr(
        manager,
        "_robot_waits_for_controlled_corridor",
        lambda robot: robot.name == "safe-hold",
    )

    try:
        selected = manager._rolling_prefetch_candidates()
        assert [entry[1].name for entry in selected] == [
            "emergency",
            "safe-hold",
        ]
    finally:
        manager.close()


def test_absolute_reservation_start_is_converted_once_at_planner_boundary() -> None:
    manager = _line_manager(2)
    prepared = manager.planner._request_preparer.prepare({
        "planningTimeSec": 100.0,
        "robots": [
            {
                "name": "first",
                "startLm": "N0",
                "goalLm": "N1",
                "reservationStartTimeSec": 103.0,
            },
            {
                "name": "second",
                "startLm": "N1",
                "goalLm": "N2",
                "reservationStartTimeSec": 108.0,
            },
        ],
    })

    assert [request.start_not_before_tick for request in prepared.requests] == [
        3,
        8,
    ]


def test_joint_sipp_batch_keeps_individual_handoff_times_on_append() -> None:
    manager = _line_manager(4)
    payload = {
        "planningTimeSec": 100.0,
        "plannerBackend": "rolling-sipp",
        "robots": [
            {
                "name": "first",
                "startLm": "N0",
                "goalLm": "N3",
                "reservationStartTimeSec": 103.0,
            },
            {
                "name": "second",
                "startLm": "N1",
                "goalLm": "N4",
                "reservationStartTimeSec": 105.0,
            },
        ],
    }

    result = manager.planner.plan(payload)

    assert result["ok"]
    plans = {plan["robot"]: plan for plan in result["plans"]}
    for name, start_lm, end_time, offset in (
        ("first", "N0", 3.0, 3.0),
        ("second", "N1", 5.0, 5.0),
    ):
        order = FleetOrder(
            order_id=f"order-{name}",
            target_lm=plans[name]["goalLm"],
            status="EXECUTING",
        )
        start_index = int(start_lm.removeprefix("N"))
        robot = FleetRobot(
            name=name,
            current_lm=start_lm,
            status="MOVING",
            active_order_id=order.order_id,
            route_chunk_goal_lm=start_lm,
            route_final_lm=plans[name]["goalLm"],
            route_clock=1.0,
            trajectory=[
                {"t": 0.0, "x": float(start_index), "y": 0.0, "yaw": 0.0,
                 "lm": start_lm},
                {"t": end_time, "x": float(start_index), "y": 0.0, "yaw": 0.0,
                 "lm": start_lm},
            ],
            plan_nodes=[start_lm],
        )
        append_plan = dict(plans[name])
        append_plan["rollingStartOffsetSec"] = offset
        original_end = float(append_plan["trajectory"][-1]["t"])

        append_result = manager._append_rolling_prefetch(
            robot,
            order,
            append_plan,
            plans[name]["goalLm"],
        )

        assert append_result.status is RollingAppendStatus.APPENDED
        assert robot.route_clock == 1.0
        assert robot.trajectory[-1]["t"] == pytest.approx(
            end_time + original_end
        )
        assert robot.trajectory[-1]["lm"] == plans[name]["goalLm"]


def test_conflict_components_separate_independent_spatial_suffixes() -> None:
    manager = _line_manager(6)
    entries = [
        _refill_entry(manager, "first", "N0", "N2", ["N0", "N1", "N2"]),
        _refill_entry(manager, "peer", "N2", "N4", ["N2", "N3", "N4"]),
        _refill_entry(manager, "independent", "N5", "N6", ["N5", "N6"]),
    ]

    batches = manager._rolling_refill_batches(entries)

    assert [
        tuple(candidate.robot_id for candidate in batch.candidates)
        for batch in batches
    ] == [("first", "peer"), ("independent",)]


def test_critical_refill_isolated_from_unrelated_normal_batch() -> None:
    manager = _line_manager(6)
    critical = _refill_entry(
        manager,
        "critical",
        "N0",
        "N2",
        ["N0", "N1", "N2"],
    )
    normal = _refill_entry(
        manager,
        "normal",
        "N4",
        "N5",
        ["N4", "N5"],
    )
    critical = (*critical[:-1], 8.0)
    normal = (*normal[:-1], 45.0)

    selected = manager._rolling_ahead_prefetch_batch(
        [critical, normal],
        critical,
    )

    assert [entry[1].name for entry in selected] == ["critical"]


@pytest.mark.parametrize(
    ("mutate", "expected"),
    [
        (lambda robot, plan: setattr(robot, "trajectory", []),
         RollingAppendStatus.INVALID_CURRENT_TRAJECTORY),
        (lambda robot, plan: plan.update(trajectory=[plan["trajectory"][0]]),
         RollingAppendStatus.INVALID_CONTINUATION),
        (lambda robot, plan: plan.update(startLm="N0"),
         RollingAppendStatus.START_LM_MISMATCH),
        (lambda robot, plan: plan["trajectory"][0].update(x=5.0),
         RollingAppendStatus.POSITION_GAP),
        (lambda robot, plan: plan["trajectory"][1].update(t=0.0),
         RollingAppendStatus.NO_TIME_PROGRESS),
        (lambda robot, plan: plan.update(nodes=["N0", "N2"]),
         RollingAppendStatus.NODE_MISMATCH),
        (lambda robot, plan: plan.update(nodes=[], goalLm=""),
         RollingAppendStatus.MISSING_GOAL),
    ],
)
def test_append_returns_a_specific_rejection_reason(
    mutate: Any,
    expected: RollingAppendStatus,
) -> None:
    manager, order, robot, plan = _append_fixture()
    mutate(robot, plan)

    result = manager._append_rolling_prefetch(robot, order, plan, "N2")

    assert result.status is expected


def test_rolling_commit_summary_counts_applied_and_rejected_entries() -> None:
    manager = _line_manager(4)
    first = _refill_entry(manager, "first", "N0", "N1", ["N0", "N1"])
    second = _refill_entry(manager, "second", "N2", "N3", ["N2", "N3"])
    first[1].route_revision = 1
    second[1].route_revision = 2
    plans = [
        _continuation_plan("first", "N1", "N2"),
        _continuation_plan("second", "N3", "N4"),
    ]
    # Keep the second robot healthy and force an explicit node mismatch. It
    # must be rejected, not silently counted as a committed pending handoff.
    plans[1]["nodes"] = ["N2", "N4"]
    job = PlanningJobRecord(
        kind="prefetch_batch",
        entries=[first, second],
        route_revisions={"first": 1, "second": 2},
        result={"ok": True, "plans": plans},
    )

    summary = manager._finish_async_rolling_prefetch(job)

    assert summary.appended == 1
    assert summary.pending_handoff == 0
    assert summary.rejected == 1
    assert summary.committed_count == 1
    assert first[1].pending_route is None
    assert second[1].pending_route is None
    assert manager.planning_state.rolling_metrics.rolling_append_failures == 1


def test_successful_rolling_result_marks_planning_job_committed() -> None:
    manager = _line_manager(2)
    entry = _refill_entry(manager, "robot", "N0", "N1", ["N0", "N1"])
    entry[1].route_revision = 7
    live_job = PlanningJobRecord(
        kind="prefetch",
        entries=[entry],
        route_revisions={"robot": 7},
    )
    manager._build_planning_job(live_job, [entry[2]], {"robots": [entry[2]]})
    live_job.transition_to(PlanningJobStatus.RUNNING)
    live_job.transition_to(PlanningJobStatus.COMPLETED)
    live_job.result = {
        "ok": True,
        "plans": [_continuation_plan("robot", "N1", "N2")],
    }

    committed = manager._finish_planning_result(live_job, None)

    assert committed == 1
    assert live_job.status is PlanningJobStatus.COMMITTED


def test_two_successful_rolling_entries_are_both_counted() -> None:
    manager = _line_manager(4)
    first = _refill_entry(manager, "first", "N0", "N1", ["N0", "N1"])
    second = _refill_entry(manager, "second", "N2", "N3", ["N2", "N3"])
    first[1].route_revision = 1
    second[1].route_revision = 2
    summary = manager._finish_async_rolling_prefetch(PlanningJobRecord(
        kind="prefetch_batch",
        entries=[first, second],
        route_revisions={"first": 1, "second": 2},
        result={
            "ok": True,
            "plans": [
                _continuation_plan("first", "N1", "N2"),
                _continuation_plan("second", "N3", "N4"),
            ],
        },
    ))

    assert summary.appended == 2
    assert summary.committed_count == 2


def test_rolling_result_without_plans_has_zero_commits() -> None:
    manager = _line_manager(2)
    entry = _refill_entry(manager, "robot", "N0", "N1", ["N0", "N1"])
    summary = manager._finish_async_rolling_prefetch(PlanningJobRecord(
        kind="prefetch",
        entries=[entry],
        route_revisions={"robot": entry[1].route_revision},
        result={"ok": False, "plans": []},
    ))

    assert summary.committed_count == 0
    assert summary.deferred == 1


def test_repeated_empty_refill_queues_a_changed_spatial_route(
    monkeypatch,
) -> None:
    manager = _line_manager(3)
    entry = _refill_entry(manager, "robot", "N0", "N1", ["N0", "N1", "N3"])
    manager._rolling_prefetch_failures[entry[1].name] = 1
    queued: list[tuple[str, str]] = []
    monkeypatch.setattr(
        manager,
        "_queue_alternate_corridor_detour",
        lambda order, start, goal, **kwargs: queued.append((start, goal)) or True,
    )
    job = PlanningJobRecord(
        kind="prefetch",
        entries=[entry],
        result={
            "ok": False,
            "plans": [],
            "debug": {"reason": "no_low_level_path:robot:no_sipp_path"},
        },
    )

    manager._finish_failed_rolling_prefetch(job, [entry], job.result)

    assert manager._rolling_prefetch_failures[entry[1].name] == 2
    assert queued == [("N1", "N3")]


@pytest.mark.parametrize(
    ("previous_failures", "route_clock"),
    [(1, 0.0), (0, 8.0)],
)
def test_critical_refill_releases_a_proven_waiting_blocker(
    monkeypatch,
    previous_failures: int,
    route_clock: float,
) -> None:
    manager = _line_manager(3)
    entry = _refill_entry(manager, "robot", "N0", "N1", ["N0", "N1", "N3"])
    blocker_entry = _refill_entry(
        manager,
        "blocker",
        "N2",
        "N3",
        ["N2", "N3"],
    )
    blocker = blocker_entry[1]
    blocker.status = "WAITING"
    entry[1].route_clock = route_clock
    if previous_failures:
        manager._rolling_prefetch_failures[entry[1].name] = previous_failures
    evacuations: list[tuple[list[str], str]] = []
    monkeypatch.setattr(
        manager,
        "_valid_rolling_prefetch_blockers",
        lambda robot_name: {blocker.name} if robot_name == "robot" else set(),
    )
    monkeypatch.setattr(
        manager,
        "_start_deadlock_corridor_evacuation",
        lambda robots, winner, now: (
            evacuations.append(([robot.name for robot in robots], winner.name))
            or blocker.name
        ),
    )
    job = PlanningJobRecord(
        kind="prefetch",
        entries=[entry],
        result={
            "ok": False,
            "plans": [],
            "debug": {"reason": "no_low_level_path:robot:no_sipp_path"},
        },
    )

    manager._finish_failed_rolling_prefetch(job, [entry], job.result)

    assert evacuations == [(["robot", "blocker"], "robot")]


def test_first_emergency_refill_failure_queues_an_alternate_corridor(
    monkeypatch,
) -> None:
    manager = _line_manager(3)
    order, robot, request, final_goal, _ = _refill_entry(
        manager,
        "robot",
        "N0",
        "N1",
        ["N0", "N1", "N3"],
    )
    robot.route_clock = 8.0
    queued: list[tuple[str, str, str]] = []
    monkeypatch.setattr(
        manager,
        "_queue_alternate_corridor_detour",
        lambda queued_order, start_lm, goal_lm, **_kwargs: (
            queued.append((queued_order.order_id, start_lm, goal_lm))
            or True
        ),
    )

    assert manager._queue_critical_rolling_detour(
        order,
        robot,
        request,
        final_goal,
        1,
    )
    assert queued == [(order.order_id, "N1", "N3")]


def test_critical_refill_job_priority_precedes_dispatch() -> None:
    manager = _line_manager(2)
    entry = _refill_entry(manager, "critical", "N0", "N1", ["N0", "N1"])
    entry[1].route_clock = 0.0
    entry[1].trajectory[-1]["t"] = 9.0
    live_job = PlanningJobRecord(kind="prefetch", entries=[entry])

    planning_job = manager._build_planning_job(
        live_job,
        [entry[2]],
        {"robots": [entry[2]]},
    )

    assert planning_job.priority is PlanningPriority.ROLLING_CRITICAL
    assert planning_job.priority < PlanningPriority.ORDER_DISPATCH


def test_critical_refill_is_admitted_before_deadlock_recovery(
    monkeypatch,
) -> None:
    manager = _line_manager(2)
    recovery_started: list[str] = []
    cycle = SimpleNamespace(
        async_simulated=True,
        clearance_departure_ready=False,
        recovery_yields_dispatch_turn=False,
        critical_prefetch_waiting=True,
        prefetch_turn_after_dispatch=False,
        dispatched=0,
        now=10.0,
    )
    monkeypatch.setattr(
        manager,
        "_queue_commanded_sink_vacancy_replan",
        lambda now: recovery_started.append("queued"),
    )

    try:
        assert manager._start_dispatch_runtime_replan(cycle) is None
        assert recovery_started == []
    finally:
        manager.close()


def test_critical_refill_does_not_yield_to_ordinary_dispatch(
    monkeypatch,
) -> None:
    manager = _line_manager(2)
    entry = _refill_entry(manager, "critical", "N0", "N1", ["N0", "N1"])
    entry[1].trajectory[-1]["t"] = 9.0
    started: list[str] = []
    cycle = SimpleNamespace(
        async_simulated=True,
        dispatched=0,
        early_prefetch_entries=[entry],
        ready=[("ordinary-dispatch",)],
    )
    manager._last_async_job_kind = "prefetch_batch"
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda entries: started.append(entries[0][1].name) or True,
    )

    try:
        assert manager._start_dispatch_prefetch(cycle) == 0
        assert started == [entry[1].name]
    finally:
        manager.close()


def test_failed_moving_critical_admission_still_blocks_new_dispatch(
    monkeypatch,
) -> None:
    manager = _line_manager(2)
    entry = _refill_entry(manager, "critical", "N0", "N1", ["N0", "N1"])
    entry[1].trajectory[-1]["t"] = 9.0
    cycle = SimpleNamespace(
        async_simulated=True,
        dispatched=0,
        early_prefetch_entries=[entry],
        ready=[("ordinary-dispatch",)],
    )
    monkeypatch.setattr(
        manager,
        "_start_async_rolling_prefetch",
        lambda _entries: False,
    )

    try:
        assert manager._start_dispatch_prefetch(cycle) == 0
    finally:
        manager.close()


def test_critical_refill_can_queue_behind_an_active_solver_job() -> None:
    manager = _line_manager(2)
    started = Event()
    release = Event()
    blocking_record = PlanningJobRecord(kind="dispatch")
    blocking_job = manager._build_planning_job(
        blocking_record,
        [{"name": "busy", "startLm": "N0", "goalLm": "N1"}],
        {"robots": []},
    )

    def blocking_solver(job: Any) -> PlanCandidate:
        started.set()
        release.wait(2.0)
        return PlanCandidate.from_result(
            job,
            {"ok": True, "plans": []},
            finished_at=job.submitted_at + 0.1,
        )

    assert manager._planning_worker.submit_job(blocking_job, blocking_solver)
    assert started.wait(1.0)

    order = FleetOrder(
        order_id="rolling-order",
        target_lm="N2",
        assigned_robot="rolling",
        status="EXECUTING",
        spatial_route_nodes=["N0", "N1", "N2"],
    )
    robot = FleetRobot(
        name="rolling",
        current_lm="N0",
        target_lm="N1",
        active_order_id=order.order_id,
        status="MOVING",
        route_revision=1,
        route_chunk_goal_lm="N1",
        route_final_lm="N2",
        trajectory=_segment("N0", "N1"),
        plan_nodes=["N0", "N1"],
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    entry = (
        order,
        robot,
        {
            "name": robot.name,
            "startLm": "N1",
            "goalLm": "N2",
            "routeNodes": ["N1", "N2"],
        },
        "N2",
        9.0,
    )
    try:
        assert manager._async_simulated_dispatch_active()
        assert not manager._planning_scheduler_saturated()
        assert manager._start_async_rolling_prefetch(entry)
        assert manager._planning_worker.stats().queued_jobs == 1
    finally:
        release.set()
        manager.close()


def test_revision_cancels_queued_dispatch_and_requeues_order() -> None:
    manager = _line_manager(2)
    started = Event()
    release = Event()
    blocking_record = PlanningJobRecord(kind="prefetch")
    blocking_job = manager._build_planning_job(
        blocking_record,
        [{"name": "rolling-busy", "startLm": "N0", "goalLm": "N1"}],
        {"robots": []},
    )

    def blocking_solver(job: Any) -> PlanCandidate:
        started.set()
        release.wait(2.0)
        return PlanCandidate.from_result(
            job,
            {"ok": True, "plans": []},
            finished_at=job.submitted_at + 0.1,
        )

    assert manager._planning_worker.submit_job(blocking_job, blocking_solver)
    assert started.wait(1.0)
    order = FleetOrder(
        order_id="queued-order",
        target_lm="N2",
        status="PLANNING",
    )
    manager.orders[order.order_id] = order
    dispatch_record = PlanningJobRecord(
        kind="dispatch",
        entries=[(order,)],
    )
    dispatch_job = manager._build_planning_job(
        dispatch_record,
        [{"name": "dispatch-robot", "startLm": "N0", "goalLm": "N2"}],
        {"robots": []},
    )
    assert manager._planning_worker.submit_job(
        dispatch_job,
        manager._planning_solver_service.solve,
    )
    try:
        manager._advance_planning_revision("unrelated order changed")
        manager._collect_completed_planning_candidates()

        assert order.status == "QUEUED"
        assert dispatch_job.job_id not in manager.planning_state.jobs
        assert manager.planning_state.diagnostic_counts[
            "planning_job_cancelled"
        ] >= 1
    finally:
        release.set()
        manager.close()


def test_spatial_route_cursor_reuses_cached_suffix() -> None:
    manager = _line_manager(4)
    order = FleetOrder(
        order_id="order",
        target_lm="N4",
        spatial_route_nodes=["N0", "N1", "N2", "N3", "N4"],
        spatial_route_cursor=2,
    )

    suffix = manager._ensure_order_spatial_route(
        order,
        "N2",
        "N4",
        release_robot_names=set(),
    )

    assert suffix == ["N2", "N3", "N4"]
    assert order.spatial_route_cursor == 2
    assert manager.planning_state.diagnostic_counts["spatial_route_reused"] == 1


def test_live_route_transaction_protects_critical_buffer_metric() -> None:
    manager = _line_manager(2)
    order = FleetOrder(
        order_id="order",
        target_lm="N2",
        assigned_robot="robot",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="robot",
        current_lm="N0",
        target_lm="N1",
        active_order_id=order.order_id,
        status="MOVING",
        route_revision=1,
        route_chunk_goal_lm="N1",
        route_final_lm="N2",
        trajectory=_segment("N0", "N1"),
        route_clock=2.0,
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    record = PlanningJobRecord(kind="runtime_replan")
    manager._build_planning_job(
        record,
        [{"name": robot.name, "startLm": "N0", "goalLm": "N2"}],
        {"robots": []},
    )

    try:
        manager._observe_rolling_runtime(manager._now())

        assert record.status is PlanningJobStatus.QUEUED
        assert (
            manager.planning_state.rolling_metrics.rolling_buffer_unprotected
            == 0
        )
    finally:
        manager.close()


def test_headless_benchmark_returns_rolling_health_summary() -> None:
    scenario = FleetBenchmarkScenario(
        name="smoke",
        robot_count=2,
        duration_sec=0.5,
        map_name="benchmark_open_kiva.smap",
        traffic_zones=False,
        controlled_corridors=False,
    )

    result = FleetBenchmarkRunner(
        scenario,
        deterministic=True,
        tick_sec=0.1,
    ).run()

    assert result["scenario"] == "smoke"
    assert result["robots"] == 2
    assert result["durationSimSec"] >= 0.5
    assert result["effectiveHorizonSec"] == 75.0
    assert result["rollingUnderrunEventTail"] == []
    assert result["physicalCollisions"] == 0
    assert "routeBufferUnderruns" in result
    assert "p99SolverDurationSec" in result


def test_100_robot_traffic_scenarios_use_large_authored_corridor_map() -> None:
    assert SCENARIOS["zones-100"].map_name == "smart_kiva_large_w_mode.smap"
    assert SCENARIOS["corridors-100"].map_name == (
        "smart_kiva_large_w_mode.smap"
    )


def _buffer_policy() -> RollingBufferPolicy:
    return RollingBufferPolicy(
        target_sec=75.0,
        refill_sec=55.0,
        urgent_sec=25.0,
        critical_sec=10.0,
        emergency_sec=3.0,
        maximum_sec=150.0,
        stagger_window_sec=8.0,
    )


def _append_fixture() -> tuple[
    FleetManagerSim,
    FleetOrder,
    FleetRobot,
    dict[str, Any],
]:
    manager = _line_manager(2)
    order = FleetOrder(
        order_id="order",
        target_lm="N2",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="robot",
        current_lm="N0",
        target_lm="N1",
        status="MOVING",
        active_order_id=order.order_id,
        route_chunk_goal_lm="N1",
        route_final_lm="N2",
        trajectory=_segment("N0", "N1"),
        plan_nodes=["N0", "N1"],
    )
    plan = _continuation_plan(robot.name, "N1", "N2")
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    return manager, order, robot, plan


def _refill_entry(
    manager: FleetManagerSim,
    name: str,
    start_lm: str,
    goal_lm: str,
    route_nodes: list[str],
) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]:
    start_index = int(start_lm.removeprefix("N"))
    goal_index = int(goal_lm.removeprefix("N"))
    order = FleetOrder(
        order_id=f"order-{name}",
        target_lm=goal_lm,
        status="EXECUTING",
        vehicle=name,
        assigned_robot=name,
        spatial_route_nodes=list(route_nodes),
    )
    robot = FleetRobot(
        name=name,
        current_lm=start_lm,
        status="MOVING",
        active_order_id=order.order_id,
        route_chunk_goal_lm=goal_lm,
        route_final_lm=route_nodes[-1],
        route_clock=0.0,
        trajectory=_segment(start_lm, goal_lm),
        plan_nodes=[start_lm, goal_lm],
    )
    request = {
        "name": name,
        "startLm": goal_lm,
        "goalLm": route_nodes[-1],
        "routeNodes": list(route_nodes),
        "reservationStartTimeSec": 10.0,
    }
    manager.orders[order.order_id] = order
    manager.robots[name] = robot
    del start_index, goal_index
    # Hand-written continuation plans and planner-produced trajectories both
    # use time relative to the robot's handoff. Absolute prediction offsets
    # belong only to temporal reservations.
    return order, robot, request, route_nodes[-1], 0.0


def _continuation_plan(
    robot_name: str,
    start_lm: str,
    goal_lm: str,
) -> dict[str, Any]:
    return {
        "robot": robot_name,
        "startLm": start_lm,
        "goalLm": goal_lm,
        "finalGoalLm": goal_lm,
        "nodes": [start_lm, goal_lm],
        "trajectory": _segment(start_lm, goal_lm),
    }


def _line_manager(edge_count: int) -> FleetManagerSim:
    landmarks = {
        f"N{index}": Landmark(name=f"N{index}", x=float(index), y=0.0)
        for index in range(edge_count + 1)
    }
    edges = [
        GraphEdge(
            from_name=f"N{index}",
            to_name=f"N{index + 1}",
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(float(index), 0.0),
                WorldPoint(float(index + 1), 0.0),
            ),
            properties={"direction": 1},
        )
        for index in range(edge_count)
    ]
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "reservation_time_step_sec": 1.0,
                "runtime_replan_lm_tolerance_m": 0.1,
                "rolling_target_buffer_sec": 75.0,
                "rolling_refill_threshold_sec": 55.0,
                "rolling_urgent_threshold_sec": 25.0,
                "rolling_critical_threshold_sec": 10.0,
                "rolling_emergency_threshold_sec": 3.0,
                "rolling_max_prepared_buffer_sec": 150.0,
                "rolling_refill_stagger_window_sec": 8.0,
            },
        },
    )


def _segment(start_lm: str, goal_lm: str) -> list[dict[str, Any]]:
    start_index = int(start_lm.removeprefix("N"))
    goal_index = int(goal_lm.removeprefix("N"))
    return [
        {
            "t": 0.0,
            "x": float(start_index),
            "y": 0.0,
            "yaw": 0.0,
            "lm": start_lm,
        },
        {
            "t": 10.0,
            "x": float(goal_index),
            "y": 0.0,
            "yaw": 0.0,
            "lm": goal_lm,
        },
    ]
