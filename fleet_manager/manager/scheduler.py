"""Bounded planning scheduler and its explicit solver boundaries."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from heapq import heapify, heappop, heappush
from math import isfinite
from threading import Condition, Thread, current_thread
from time import monotonic
from typing import Callable

from fleet_manager.manager.planning import (
    PlanCandidate,
    PlanningCancelledError,
    PlanningDeadlineExceededError,
    PlanningJob,
    PlanningJobStatus,
)


class PlanningWorkerState(str, Enum):
    """Lifecycle of a :class:`PlanningWorker`."""

    IDLE = "idle"
    RUNNING = "running"
    CLOSING = "closing"
    CLOSED = "closed"


@dataclass(frozen=True, slots=True)
class PlanningWorkerFailure:
    """Compact diagnostics for an unexpected task exception."""

    occurred_at: float
    submission: int
    exception_type: str
    message: str


@dataclass(frozen=True, slots=True)
class PlanningWorkerJobEvent:
    """Immutable lifecycle update consumed by the runtime owner."""

    job_id: str
    status: PlanningJobStatus
    occurred_at: float
    message: str = ""


@dataclass(frozen=True, slots=True)
class PlanningWorkerStats:
    """Read-only queue sizes for diagnostics and boundedness tests."""

    queued_jobs: int
    heap_entries: int
    running_jobs: int
    completed_results: int


PlanningSolver = Callable[[PlanningJob], PlanCandidate]
PlanningCompletionConsumer = Callable[[], None]


class PlanningWorker:
    """Own one finite solver thread and a bounded priority queue.

    The thread drains currently accepted jobs and exits when the queue is
    empty. There is no permanently waiting non-daemon thread, which is useful
    for tests and small tools without an application lifecycle.
    """

    def __init__(
        self,
        *,
        name: str = "fleet-planning-worker",
        max_queue_size: int = 8,
    ) -> None:
        worker_name = str(name).strip()
        if not worker_name:
            raise ValueError("name must not be empty")
        if isinstance(max_queue_size, bool) or not isinstance(max_queue_size, int):
            raise TypeError("max_queue_size must be an int")
        if max_queue_size < 1:
            raise ValueError("max_queue_size must be at least 1")

        self._name = worker_name
        self._condition = Condition()
        self._state = PlanningWorkerState.IDLE
        self._thread: Thread | None = None
        self._close_requested = False
        self._submission_count = 0
        self._active_submission = 0
        self._last_failure: PlanningWorkerFailure | None = None
        self._completion_consumer: PlanningCompletionConsumer | None = None
        self._completed_results: deque[PlanCandidate] = deque()
        self._job_events: deque[PlanningWorkerJobEvent] = deque()
        self._max_queue_size = max_queue_size
        self._job_sequence = 0
        self._job_queue: list[
            tuple[int, int, str, PlanningJob, PlanningSolver]
        ] = []
        self._queued_jobs: dict[str, PlanningJob] = {}
        self._active_job: PlanningJob | None = None

    @property
    def name(self) -> str:
        return self._name

    @property
    def state(self) -> PlanningWorkerState:
        with self._condition:
            return self._state

    @property
    def is_running(self) -> bool:
        with self._condition:
            return bool(
                self._thread is not None
                and self._thread.is_alive()
            )

    @property
    def submission_count(self) -> int:
        with self._condition:
            return self._submission_count

    @property
    def active_submission(self) -> int:
        with self._condition:
            return self._active_submission

    @property
    def last_failure(self) -> PlanningWorkerFailure | None:
        with self._condition:
            return self._last_failure

    @property
    def max_queue_size(self) -> int:
        return self._max_queue_size

    def stats(self) -> PlanningWorkerStats:
        with self._condition:
            return PlanningWorkerStats(
                queued_jobs=len(self._queued_jobs),
                heap_entries=len(self._job_queue),
                running_jobs=int(self._active_job is not None),
                completed_results=len(self._completed_results),
            )

    def set_completion_consumer(
        self,
        consumer: PlanningCompletionConsumer | None,
    ) -> None:
        """Set a callback run by the thread which successfully joins work."""

        if consumer is not None and not callable(consumer):
            raise TypeError("completion consumer must be callable or None")
        with self._condition:
            self._completion_consumer = consumer

    def submit_job(
        self,
        job: PlanningJob,
        solver: PlanningSolver,
    ) -> bool:
        """Queue one bounded priority job for the single solver thread."""

        if not isinstance(job, PlanningJob):
            raise TypeError("job must be a PlanningJob")
        if not callable(solver):
            raise TypeError("solver must be callable")
        with self._condition:
            if self._close_requested:
                return False
            if job.job_id in self._queued_jobs:
                return False

            self._coalesce_locked(job)
            if not self._make_queue_space_locked(job):
                return False

            self._job_sequence += 1
            sequence = self._job_sequence
            self._submission_count += 1
            self._queued_jobs[job.job_id] = job
            heappush(
                self._job_queue,
                (
                    int(job.priority),
                    sequence,
                    job.job_id,
                    job,
                    solver,
                ),
            )
            if self._thread is None or not self._thread.is_alive():
                previous = self._thread
                if previous is not None:
                    previous.join(0.0)
                worker = Thread(
                    target=self._run_scheduler,
                    name=self._name,
                    daemon=False,
                )
                self._thread = worker
                self._state = PlanningWorkerState.RUNNING
                try:
                    worker.start()
                except BaseException:
                    self._thread = None
                    self._queued_jobs.pop(job.job_id, None)
                    self._rebuild_heap_locked()
                    job.cancellation_token.cancel()
                    self._state = PlanningWorkerState.IDLE
                    self._condition.notify_all()
                    raise
            else:
                self._state = PlanningWorkerState.RUNNING
                self._condition.notify_all()
            return True

    def cancel_job(self, job_id: str) -> bool:
        """Cancel queued work or signal the currently running solver."""

        clean_id = str(job_id).strip()
        if not clean_id:
            return False
        with self._condition:
            queued = self._queued_jobs.pop(clean_id, None)
            if queued is not None:
                queued.cancellation_token.cancel()
                self._rebuild_heap_locked()
                self._publish_job_event_locked(
                    queued,
                    PlanningJobStatus.CANCELLED,
                    "planning job cancelled while queued",
                )
                self._condition.notify_all()
                return True
            active = self._active_job
            if active is not None and active.job_id == clean_id:
                active.cancellation_token.cancel()
                self._condition.notify_all()
                return True
            return False

    def take_completed_results(self) -> tuple[PlanCandidate, ...]:
        """Remove all completed outputs in deterministic publication order."""

        with self._condition:
            results = tuple(self._completed_results)
            self._completed_results.clear()
            return results

    def take_job_events(self) -> tuple[PlanningWorkerJobEvent, ...]:
        """Return lifecycle updates for application by the runtime owner."""

        with self._condition:
            events = tuple(self._job_events)
            self._job_events.clear()
            return events

    def _coalesce_locked(self, new_job: PlanningJob) -> None:
        key = new_job.coalescing_key
        if not key:
            return
        for job_id, queued in list(self._queued_jobs.items()):
            if queued.coalescing_key != key:
                continue
            self._queued_jobs.pop(job_id, None)
            queued.cancellation_token.cancel()
            self._publish_job_event_locked(
                queued,
                PlanningJobStatus.CANCELLED,
                "planning job replaced by newer coalesced work",
            )
        self._rebuild_heap_locked()
        active = self._active_job
        if active is not None and active.coalescing_key == key:
            active.cancellation_token.cancel()

    def _make_queue_space_locked(self, new_job: PlanningJob) -> bool:
        if len(self._queued_jobs) < self._max_queue_size:
            return True
        worst = max(
            self._queued_jobs.values(),
            key=lambda item: (int(item.priority), item.submitted_at, item.job_id),
        )
        if int(new_job.priority) >= int(worst.priority):
            return False
        self._queued_jobs.pop(worst.job_id, None)
        worst.cancellation_token.cancel()
        self._rebuild_heap_locked()
        self._publish_job_event_locked(
            worst,
            PlanningJobStatus.CANCELLED,
            "planning job evicted by higher-priority work",
        )
        return True

    def _rebuild_heap_locked(self) -> None:
        """Drop stale entries left by coalescing or queued cancellation."""

        self._job_queue = [
            entry
            for entry in self._job_queue
            if self._queued_jobs.get(entry[2]) is entry[3]
        ]
        heapify(self._job_queue)

    def _next_job_locked(
        self,
    ) -> tuple[PlanningJob, PlanningSolver] | None:
        while self._job_queue:
            _, _, job_id, job, solver = heappop(self._job_queue)
            if self._queued_jobs.get(job_id) is not job:
                continue
            self._queued_jobs.pop(job_id, None)
            return job, solver
        return None

    def _run_scheduler(self) -> None:
        try:
            while True:
                with self._condition:
                    if self._close_requested:
                        self._cancel_queued_jobs_locked()
                    next_job = self._next_job_locked()
                    if next_job is None:
                        # Relinquish scheduler ownership while the empty-queue
                        # decision is still protected by the same condition.
                        # A submitter can now observe either this worker or
                        # ``None``, never a live thread that has already
                        # decided to exit.
                        self._retire_current_thread_locked()
                        return
                    job, solver = next_job
                    self._active_job = job
                    self._active_submission = self._submission_count
                    self._publish_job_event_locked(
                        job,
                        PlanningJobStatus.RUNNING,
                    )

                candidate, status, message = self._run_scheduled_job(
                    job,
                    solver,
                )

                with self._condition:
                    self._completed_results.append(candidate)
                    self._publish_job_event_locked(job, status, message)
                    if self._active_job is job:
                        self._active_job = None
                    self._active_submission = 0
                    self._condition.notify_all()
        finally:
            with self._condition:
                # Unexpected failures still release ownership. After the
                # normal empty-queue path this is intentionally a no-op, and
                # it cannot overwrite a replacement worker started meanwhile.
                self._retire_current_thread_locked()

    def _retire_current_thread_locked(self) -> None:
        """Atomically release ownership held by the calling worker thread."""

        if self._thread is not current_thread():
            return
        self._thread = None
        self._active_job = None
        self._active_submission = 0
        self._state = (
            PlanningWorkerState.CLOSED
            if self._close_requested
            else PlanningWorkerState.IDLE
        )
        self._condition.notify_all()

    def _run_scheduled_job(
        self,
        job: PlanningJob,
        solver: PlanningSolver,
    ) -> tuple[PlanCandidate, PlanningJobStatus, str]:
        if job.deadline is not None and monotonic() > job.deadline:
            job.cancellation_token.cancel()
            message = "planning deadline exceeded"
            return (
                self._terminal_candidate(job, message),
                PlanningJobStatus.DEADLINE_EXCEEDED,
                message,
            )
        try:
            candidate = solver(job)
        except PlanningDeadlineExceededError as exc:
            job.cancellation_token.cancel()
            message = str(exc) or "planning deadline exceeded"
            return (
                self._terminal_candidate(job, message),
                PlanningJobStatus.DEADLINE_EXCEEDED,
                message,
            )
        except PlanningCancelledError as exc:
            message = str(exc) or "planning job cancelled"
            return (
                self._terminal_candidate(job, message),
                PlanningJobStatus.CANCELLED,
                message,
            )
        except Exception as exc:
            with self._condition:
                self._last_failure = PlanningWorkerFailure(
                    occurred_at=monotonic(),
                    submission=self._submission_count,
                    exception_type=type(exc).__name__,
                    message=str(exc),
                )
            message = f"planning solver failed: {exc}"
            return (
                self._terminal_candidate(job, message),
                PlanningJobStatus.FAILED,
                message,
            )
        if job.cancellation_token.cancelled:
            return candidate, PlanningJobStatus.CANCELLED, "planning job cancelled"
        elif job.deadline is not None and candidate.finished_at > job.deadline:
            job.cancellation_token.cancel()
            return (
                candidate,
                PlanningJobStatus.DEADLINE_EXCEEDED,
                "planning deadline exceeded",
            )
        return candidate, PlanningJobStatus.COMPLETED, ""

    @staticmethod
    def _terminal_candidate(job: PlanningJob, reason: str) -> PlanCandidate:
        return PlanCandidate.from_result(
            job,
            {
                "ok": False,
                "plans": [],
                "debug": {"reason": reason},
            },
            finished_at=monotonic(),
        )

    def _cancel_queued_jobs_locked(self) -> None:
        for job in self._queued_jobs.values():
            job.cancellation_token.cancel()
            self._publish_job_event_locked(
                job,
                PlanningJobStatus.CANCELLED,
                "planning worker shutdown",
            )
        self._queued_jobs.clear()
        self._job_queue.clear()
        active = self._active_job
        if active is not None:
            active.cancellation_token.cancel()

    def _publish_job_event_locked(
        self,
        job: PlanningJob,
        status: PlanningJobStatus,
        message: str = "",
    ) -> None:
        self._job_events.append(
            PlanningWorkerJobEvent(
                job_id=job.job_id,
                status=status,
                occurred_at=monotonic(),
                message=message,
            )
        )

    def join(self, timeout: float | None = None) -> bool:
        """Wait for the current finite task without closing the worker."""

        wait_seconds = self._validated_timeout(timeout)
        with self._condition:
            worker = self._thread
        if worker is None:
            consumer = self._completion_consumer
            if consumer is not None:
                consumer()
            return True
        if worker is current_thread():
            return False
        worker.join(wait_seconds)
        stopped = not worker.is_alive()
        if stopped:
            consumer = self._completion_consumer
            if consumer is not None:
                consumer()
        return stopped

    def close(self, timeout: float | None = None) -> bool:
        """Reject future submissions and join the active task.

        Closing is idempotent. A timed-out close leaves the worker in
        ``CLOSING``; the task's finalizer performs the transition to
        ``CLOSED`` when it returns.
        """

        wait_seconds = self._validated_timeout(timeout)
        with self._condition:
            if self._state is PlanningWorkerState.CLOSED:
                worker = self._thread
                already_closed = True
                if worker is None:
                    return True
            else:
                already_closed = False

                self._close_requested = True
                self._cancel_queued_jobs_locked()
                worker = self._thread
                if worker is None:
                    self._active_submission = 0
                    self._state = PlanningWorkerState.CLOSED
                    self._condition.notify_all()
                    return True
                if not worker.is_alive():
                    worker.join(0.0)
                    self._active_submission = 0
                    self._state = PlanningWorkerState.CLOSED
                    self._condition.notify_all()
                    return True

                self._state = PlanningWorkerState.CLOSING
                self._condition.notify_all()

        if worker is current_thread():
            return False
        worker.join(wait_seconds)
        stopped = not worker.is_alive()
        if stopped and not already_closed:
            with self._condition:
                self._active_submission = 0
                self._state = PlanningWorkerState.CLOSED
                self._condition.notify_all()
        return stopped

    @staticmethod
    def _validated_timeout(timeout: float | None) -> float | None:
        if timeout is None:
            return None
        value = float(timeout)
        if not isfinite(value) or value < 0.0:
            raise ValueError("timeout must be a finite non-negative number")
        return value
