from __future__ import annotations

from threading import Event, Thread, current_thread
from time import monotonic, sleep

import pytest

from fleet_manager.manager.manager import FleetManagerCore
from fleet_manager.manager.scheduler import (
    PlanningWorker,
    PlanningWorkerState,
)
from fleet_manager.manager.planning import (
    FrozenMapping,
    PlanCandidate,
    PlanningJob,
    PlanningJobRecord,
    PlanningPriority,
    PlanningReason,
    PlanningSnapshot,
    PlanningSolverService,
)
from fleet_manager.core.mapping.maps.models import (
    GraphEdge,
    Landmark,
    WorldPoint,
)


def _wait_until(predicate, *, timeout: float = 1.0) -> bool:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return True
        sleep(0.002)
    return predicate()


def _manager() -> FleetManagerCore:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(
            WorldPoint(0.0, 0.0),
            WorldPoint(1.0, 0.0),
        ),
        properties={"direction": 1},
    )
    return FleetManagerCore(
        landmarks,
        [edge],
        params={"fleet": {}},
    )


def _planning_job(job_id: str) -> PlanningJob:
    snapshot = PlanningSnapshot(
        revision=1,
        created_at=monotonic(),
        robots=(),
        active_routes=(),
        reservations=(),
        traffic_resources=(),
        blockers=(),
        graph_revision=None,
        map_revision=None,
        requests=(),
        primary_payload=FrozenMapping(),
    )
    return PlanningJob(
        job_id=job_id,
        reason=PlanningReason.ORDER_DISPATCH,
        priority=PlanningPriority.ORDER_DISPATCH,
        snapshot=snapshot,
        submitted_at=monotonic(),
    )


def _candidate(job: PlanningJob) -> PlanCandidate:
    return PlanCandidate.from_result(
        job,
        {"ok": True, "plans": []},
        finished_at=monotonic(),
    )


def _install_planner(manager: FleetManagerCore, planner) -> None:
    def planner_call(payload):
        requests = payload.get("robots", [])
        return planner(requests, payload)

    manager._planning_solver_service = PlanningSolverService(
        planner_call,
        manager._planner_lock,
    )


def test_worker_owns_one_finite_non_daemon_thread() -> None:
    worker = PlanningWorker(name="test-planner", max_queue_size=1)
    started = Event()
    release = Event()
    observations: list[tuple[str, bool]] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        observations.append(
            (current_thread().name, current_thread().daemon)
        )
        started.set()
        release.wait(1.0)
        return _candidate(job)

    assert worker.state is PlanningWorkerState.IDLE
    assert worker.submit_job(_planning_job("active"), solver)
    assert started.wait(1.0)
    assert worker.state is PlanningWorkerState.RUNNING
    assert worker.active_submission == 1
    assert worker.submit_job(_planning_job("queued"), _candidate)
    assert not worker.submit_job(_planning_job("queue-full"), _candidate)

    release.set()
    assert worker.join(timeout=1.0)
    assert worker.state is PlanningWorkerState.IDLE
    assert worker.active_submission == 0
    assert observations == [("test-planner", False)]
    assert worker.close()


def test_worker_can_run_a_new_job_after_previous_thread_exits() -> None:
    worker = PlanningWorker()
    completed: list[int] = []

    def first(job: PlanningJob) -> PlanCandidate:
        completed.append(1)
        return _candidate(job)

    def second(job: PlanningJob) -> PlanCandidate:
        completed.append(2)
        return _candidate(job)

    assert worker.submit_job(_planning_job("first"), first)
    assert worker.join(timeout=1.0)
    assert worker.submit_job(_planning_job("second"), second)
    assert worker.join(timeout=1.0)

    assert completed == [1, 2]
    assert worker.submission_count == 2
    assert worker.close()


def test_unexpected_task_exception_is_recorded_and_worker_recovers() -> None:
    worker = PlanningWorker()

    def fail(_job: PlanningJob) -> PlanCandidate:
        raise ValueError("planner exploded")

    assert worker.submit_job(_planning_job("failed"), fail)
    assert worker.join(timeout=1.0)
    assert _wait_until(lambda: worker.state is PlanningWorkerState.IDLE)

    failure = worker.last_failure
    assert failure is not None
    assert failure.submission == 1
    assert failure.exception_type == "ValueError"
    assert failure.message == "planner exploded"

    recovered = Event()

    def recover(job: PlanningJob) -> PlanCandidate:
        recovered.set()
        return _candidate(job)

    assert worker.submit_job(_planning_job("recovered"), recover)
    assert recovered.wait(1.0)
    assert worker.join(timeout=1.0)
    assert worker.close()


def test_timed_out_close_rejects_work_until_job_finishes() -> None:
    worker = PlanningWorker()
    started = Event()
    release = Event()

    def solver(job: PlanningJob) -> PlanCandidate:
        started.set()
        release.wait(1.0)
        return _candidate(job)

    assert worker.submit_job(_planning_job("active"), solver)
    assert started.wait(1.0)
    assert not worker.close(timeout=0.01)
    assert worker.state is PlanningWorkerState.CLOSING
    assert not worker.submit_job(_planning_job("rejected"), _candidate)

    release.set()
    assert worker.close(timeout=1.0)
    assert worker.state is PlanningWorkerState.CLOSED
    assert worker.close()
    assert not worker.submit_job(_planning_job("closed"), _candidate)


@pytest.mark.parametrize(
    "timeout",
    [-1.0, float("inf"), float("nan")],
)
def test_worker_rejects_invalid_join_timeout(timeout: float) -> None:
    worker = PlanningWorker()
    with pytest.raises(ValueError, match="timeout"):
        worker.join(timeout)
    assert worker.close()


def test_dispatch_completion_does_not_publish_into_replaced_job(
    monkeypatch,
) -> None:
    manager = _manager()
    planning_started = Event()
    release_planning = Event()
    stale_job = PlanningJobRecord(kind="dispatch")
    replacement_job = PlanningJobRecord(kind="prefetch")

    def plan(_requests, _payload):
        planning_started.set()
        release_planning.wait(1.0)
        return {"ok": True, "plans": []}

    _install_planner(manager, plan)
    with manager._dispatch_job_lock:
        manager._dispatch_job = stale_job
    assert manager._submit_async_planning_job(
        stale_job,
        [],
        {},
    )
    assert planning_started.wait(1.0)

    with manager._dispatch_job_lock:
        manager._dispatch_job = replacement_job
    release_planning.set()
    assert manager._planning_worker.join(timeout=1.0)

    assert stale_job.kind == "dispatch"
    assert stale_job.done is False
    assert stale_job.result is None
    assert stale_job.candidate is None
    assert replacement_job.kind == "prefetch"
    assert replacement_job.done is False
    assert replacement_job.result is None
    with manager._dispatch_job_lock:
        manager._dispatch_job = None
    manager.close()


def test_dispatch_helper_preserves_background_failure_reason(
    monkeypatch,
) -> None:
    manager = _manager()
    job = PlanningJobRecord(kind="dispatch")

    def fail(_requests, _payload):
        raise RuntimeError("no route")

    _install_planner(manager, fail)
    with manager._dispatch_job_lock:
        manager._dispatch_job = job
    assert manager._submit_async_planning_job(
        job,
        [],
        {},
    )
    assert manager._planning_worker.join(timeout=1.0)

    assert job.done is True
    candidate = job.candidate
    assert isinstance(candidate, PlanCandidate)
    assert candidate.result.get("ok") is False
    assert candidate.diagnostics.get("reason") == "planning solver failed: no route"
    manager.close()


def test_manager_close_discards_current_job_and_joins_worker(
    monkeypatch,
) -> None:
    manager = _manager()
    planning_started = Event()
    release_planning = Event()
    close_finished = Event()
    corridor_gates = {"r1": {"slot": "test"}}
    released_gates: list[object] = []
    job = PlanningJobRecord(
        kind="dispatch",
        corridor_gates=corridor_gates,
    )

    def plan(_requests, _payload):
        planning_started.set()
        release_planning.wait(1.0)
        return {"ok": True, "plans": []}

    _install_planner(manager, plan)
    monkeypatch.setattr(
        manager,
        "_release_controlled_corridor_gate_pins",
        released_gates.append,
    )
    with manager._dispatch_job_lock:
        manager._dispatch_job = job
    assert manager._submit_async_planning_job(
        job,
        [],
        {},
    )
    assert planning_started.wait(1.0)

    closer = Thread(
        target=lambda: (manager.close(), close_finished.set()),
        name="test-manager-close",
        daemon=False,
    )
    closer.start()
    assert _wait_until(lambda: job.discard)
    assert not close_finished.is_set()

    release_planning.set()
    closer.join(1.0)
    assert not closer.is_alive()
    assert close_finished.is_set()
    assert job.discard is True
    assert manager._dispatch_job is None
    assert manager._planning_worker.state is PlanningWorkerState.CLOSED
    assert released_gates == [corridor_gates]
