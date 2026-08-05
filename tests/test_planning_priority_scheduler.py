from __future__ import annotations

from threading import Event, Thread
from time import monotonic, sleep

from fleet_manager.manager.scheduler import (
    PlanningWorker,
    PlanningWorkerJobEvent,
    PlanningWorkerState,
)
from fleet_manager.manager.planning import (
    FrozenMapping,
    PlanCandidate,
    PlanningJob,
    PlanningJobStatus,
    PlanningPriority,
    PlanningReason,
    PlanningSnapshot,
)


def _snapshot(revision: int = 1) -> PlanningSnapshot:
    return PlanningSnapshot(
        revision=revision,
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


def _job(
    job_id: str,
    priority: PlanningPriority,
    *,
    key: str = "",
    deadline: float | None = None,
) -> PlanningJob:
    return PlanningJob(
        job_id=job_id,
        reason=PlanningReason[priority.name],
        priority=priority,
        snapshot=_snapshot(),
        submitted_at=monotonic(),
        deadline=deadline,
        coalescing_key=key,
    )


def _candidate(job: PlanningJob) -> PlanCandidate:
    return PlanCandidate.from_result(
        job,
        {"ok": True, "plans": []},
        finished_at=monotonic(),
    )


def _terminal_statuses(
    events: tuple[PlanningWorkerJobEvent, ...],
) -> dict[str, PlanningJobStatus]:
    terminal = {
        PlanningJobStatus.COMPLETED,
        PlanningJobStatus.CANCELLED,
        PlanningJobStatus.DEADLINE_EXCEEDED,
        PlanningJobStatus.FAILED,
    }
    return {
        event.job_id: event.status
        for event in events
        if event.status in terminal
    }


def test_priority_order_and_fifo_inside_same_priority() -> None:
    worker = PlanningWorker(max_queue_size=8)
    first_started = Event()
    release_first = Event()
    order: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        order.append(job.job_id)
        if job.job_id == "active":
            first_started.set()
            release_first.wait(1.0)
        return _candidate(job)

    assert worker.submit_job(
        _job("active", PlanningPriority.BACKGROUND_OPTIMIZATION),
        solver,
    )
    assert first_started.wait(1.0)
    assert worker.submit_job(_job("dispatch-1", PlanningPriority.ORDER_DISPATCH), solver)
    assert worker.submit_job(_job("dispatch-2", PlanningPriority.ORDER_DISPATCH), solver)
    assert worker.submit_job(_job("rolling", PlanningPriority.ROLLING_CONTINUATION), solver)
    assert worker.submit_job(_job("safety", PlanningPriority.SAFETY_REPLAN), solver)
    release_first.set()

    assert worker.join(1.0)
    assert order == [
        "active",
        "safety",
        "rolling",
        "dispatch-1",
        "dispatch-2",
    ]
    assert worker.close()


def test_coalescing_replaces_only_queued_duplicate() -> None:
    worker = PlanningWorker(max_queue_size=4)
    first_started = Event()
    release_first = Event()
    executed: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        executed.append(job.job_id)
        if job.job_id == "active":
            first_started.set()
            release_first.wait(1.0)
        return _candidate(job)

    assert worker.submit_job(_job("active", PlanningPriority.ORDER_DISPATCH), solver)
    assert first_started.wait(1.0)
    old = _job("rolling-old", PlanningPriority.ROLLING_CONTINUATION, key="rolling:r1")
    new = _job("rolling-new", PlanningPriority.ROLLING_CONTINUATION, key="rolling:r1")
    assert worker.submit_job(old, solver)
    assert worker.submit_job(new, solver)
    release_first.set()
    assert worker.join(1.0)

    statuses = _terminal_statuses(worker.take_job_events())
    assert old.cancellation_token.cancelled
    assert statuses["rolling-old"] is PlanningJobStatus.CANCELLED
    assert executed == ["active", "rolling-new"]
    assert worker.close()


def test_bounded_queue_rejects_equal_work_but_admits_safety() -> None:
    worker = PlanningWorker(max_queue_size=2)
    first_started = Event()
    release_first = Event()
    executed: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        executed.append(job.job_id)
        if job.job_id == "active":
            first_started.set()
            release_first.wait(1.0)
        return _candidate(job)

    assert worker.submit_job(_job("active", PlanningPriority.ORDER_DISPATCH), solver)
    assert first_started.wait(1.0)
    background_1 = _job("background-1", PlanningPriority.BACKGROUND_OPTIMIZATION)
    background_2 = _job("background-2", PlanningPriority.BACKGROUND_OPTIMIZATION)
    assert worker.submit_job(background_1, solver)
    assert worker.submit_job(background_2, solver)
    assert not worker.submit_job(
        _job("background-3", PlanningPriority.BACKGROUND_OPTIMIZATION),
        solver,
    )
    safety = _job("safety", PlanningPriority.SAFETY_REPLAN)
    assert worker.submit_job(safety, solver)
    release_first.set()
    assert worker.join(1.0)

    statuses = _terminal_statuses(worker.take_job_events())
    assert statuses["background-2"] is PlanningJobStatus.CANCELLED
    assert executed == ["active", "safety", "background-1"]
    assert worker.close()


def test_cancel_queued_job_and_deadline_handling() -> None:
    worker = PlanningWorker(max_queue_size=3)
    first_started = Event()
    release_first = Event()
    executed: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        executed.append(job.job_id)
        if job.job_id == "active":
            first_started.set()
            release_first.wait(1.0)
        return _candidate(job)

    assert worker.submit_job(_job("active", PlanningPriority.ORDER_DISPATCH), solver)
    assert first_started.wait(1.0)
    cancelled = _job("cancelled", PlanningPriority.ORDER_DISPATCH)
    expired = _job(
        "expired",
        PlanningPriority.SAFETY_REPLAN,
        deadline=monotonic() - 1.0,
    )
    assert worker.submit_job(cancelled, solver)
    assert worker.cancel_job(cancelled.job_id)
    assert worker.submit_job(expired, solver)
    release_first.set()
    assert worker.join(1.0)

    statuses = _terminal_statuses(worker.take_job_events())
    assert statuses["cancelled"] is PlanningJobStatus.CANCELLED
    assert statuses["expired"] is PlanningJobStatus.DEADLINE_EXCEEDED
    assert executed == ["active"]
    assert worker.close()


def test_coalescing_stress_keeps_physical_heap_bounded() -> None:
    worker = PlanningWorker(max_queue_size=2)
    active_started = Event()
    release_active = Event()
    executed: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        executed.append(job.job_id)
        if job.job_id == "active":
            active_started.set()
            release_active.wait(2.0)
        return _candidate(job)

    assert worker.submit_job(_job("active", PlanningPriority.ORDER_DISPATCH), solver)
    assert active_started.wait(1.0)
    latest = ""
    for index in range(1000):
        latest = f"rolling-{index}"
        assert worker.submit_job(
            _job(
                latest,
                PlanningPriority.ROLLING_CONTINUATION,
                key="rolling:r1",
            ),
            solver,
        )
        stats = worker.stats()
        assert stats.queued_jobs <= 1
        assert stats.heap_entries <= 1

    release_active.set()
    assert worker.join(2.0)
    assert executed == ["active", latest]
    assert worker.stats().heap_entries == 0
    assert worker.close()


def test_cancelled_queued_jobs_do_not_leave_heap_entries() -> None:
    worker = PlanningWorker(max_queue_size=2)
    active_started = Event()
    release_active = Event()

    def solver(job: PlanningJob) -> PlanCandidate:
        if job.job_id == "active":
            active_started.set()
            release_active.wait(1.0)
        return _candidate(job)

    assert worker.submit_job(_job("active", PlanningPriority.ORDER_DISPATCH), solver)
    assert active_started.wait(1.0)
    for index in range(100):
        queued = _job(f"cancel-{index}", PlanningPriority.ORDER_DISPATCH)
        assert worker.submit_job(queued, solver)
        assert worker.cancel_job(queued.job_id)
        stats = worker.stats()
        assert stats.queued_jobs == 0
        assert stats.heap_entries == 0

    release_active.set()
    assert worker.join(1.0)
    assert worker.close()


def test_exception_is_reported_and_next_job_still_runs() -> None:
    worker = PlanningWorker()
    executed: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        executed.append(job.job_id)
        if job.job_id == "broken":
            raise ValueError("solver exploded")
        return _candidate(job)

    assert worker.submit_job(_job("broken", PlanningPriority.SAFETY_REPLAN), solver)
    assert worker.submit_job(_job("next", PlanningPriority.ORDER_DISPATCH), solver)
    assert worker.join(1.0)

    statuses = _terminal_statuses(worker.take_job_events())
    assert statuses["broken"] is PlanningJobStatus.FAILED
    assert statuses["next"] is PlanningJobStatus.COMPLETED
    assert worker.last_failure is not None
    assert worker.last_failure.exception_type == "ValueError"
    assert len(worker.take_completed_results()) == 2
    assert worker.close()


def test_submit_during_atomic_worker_retirement_starts_replacement() -> None:
    class PausingRetirementWorker(PlanningWorker):
        def __init__(self) -> None:
            super().__init__()
            self.retirement_started = Event()
            self.allow_retirement = Event()
            self._pause_once = True

        def _retire_current_thread_locked(self) -> None:
            if self._pause_once and not self._close_requested:
                self._pause_once = False
                self.retirement_started.set()
                self.allow_retirement.wait(1.0)
            super()._retire_current_thread_locked()

    worker = PausingRetirementWorker()
    executed: list[str] = []

    def solver(job: PlanningJob) -> PlanCandidate:
        executed.append(job.job_id)
        return _candidate(job)

    assert worker.submit_job(
        _job("first", PlanningPriority.ORDER_DISPATCH),
        solver,
    )
    assert worker.retirement_started.wait(1.0)

    submitted = Event()

    def submit_second() -> None:
        assert worker.submit_job(
            _job("second", PlanningPriority.SAFETY_REPLAN),
            solver,
        )
        submitted.set()

    submitter = Thread(target=submit_second)
    submitter.start()
    # Retirement still owns the condition, so submission cannot observe the
    # old live thread between the empty decision and ownership release.
    assert not submitted.wait(0.02)
    worker.allow_retirement.set()
    submitter.join(1.0)

    assert submitted.is_set()
    assert worker.join(1.0)
    assert executed == ["first", "second"]
    assert worker.stats().queued_jobs == 0
    assert worker.close()


def test_shutdown_cancels_running_and_queued_jobs() -> None:
    worker = PlanningWorker(max_queue_size=2)
    started = Event()

    def solver(job: PlanningJob) -> PlanCandidate:
        started.set()
        deadline = monotonic() + 1.0
        while not job.cancellation_token.cancelled and monotonic() < deadline:
            sleep(0.001)
        return _candidate(job)

    active = _job("active", PlanningPriority.ORDER_DISPATCH)
    queued = _job("queued", PlanningPriority.ORDER_DISPATCH)
    assert worker.submit_job(active, solver)
    assert started.wait(1.0)
    assert worker.submit_job(queued, solver)

    assert worker.close(timeout=1.0)
    statuses = _terminal_statuses(worker.take_job_events())
    assert worker.state is PlanningWorkerState.CLOSED
    assert active.cancellation_token.cancelled
    assert queued.cancellation_token.cancelled
    assert statuses["active"] is PlanningJobStatus.CANCELLED
    assert statuses["queued"] is PlanningJobStatus.CANCELLED


def test_worker_returns_candidate_without_applying_live_state() -> None:
    worker = PlanningWorker()
    live_state = {"route": "old", "reservations": ["old"]}

    def solver(job: PlanningJob) -> PlanCandidate:
        return PlanCandidate.from_result(
            job,
            {"ok": True, "plans": [{"robot": "r1", "nodes": ["A", "B"]}]},
            finished_at=monotonic(),
        )

    assert worker.submit_job(_job("candidate", PlanningPriority.ORDER_DISPATCH), solver)
    assert worker.join(1.0)

    assert live_state == {"route": "old", "reservations": ["old"]}
    candidates = worker.take_completed_results()
    assert len(candidates) == 1
    assert candidates[0].expected_revision == 1
    assert worker.close()
