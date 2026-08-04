"""Bounded planning scheduler and its explicit solver boundaries."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from heapq import heappop, heappush
from math import isfinite
from threading import Condition, Lock, Thread, current_thread
from time import monotonic
from typing import Any, Callable, Generic, TypeVar

from fleet_manager.core.planning_models import (
    PlanCandidate,
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


PlanningTask = Callable[[], None]
PlanningSolver = Callable[[PlanningJob], PlanCandidate]
PlanningCompletionConsumer = Callable[[], None]
PlannerCall = Callable[[dict[str, Any]], dict[str, Any]]
CommitValueT = TypeVar("CommitValueT")
CheckpointT = TypeVar("CheckpointT")


class PlanningSolverService:
    """Run a solver from immutable input without live manager access."""

    def __init__(self, planner_call: PlannerCall, planner_lock: Lock) -> None:
        if not callable(planner_call):
            raise TypeError("planner_call must be callable")
        self._planner_call = planner_call
        self._planner_lock = planner_lock

    def solve(self, job: PlanningJob) -> PlanCandidate:
        token = job.cancellation_token
        if token.cancelled:
            return self._cancelled_candidate(job)

        with self._planner_lock:
            primary_result = self._planner_call(
                job.snapshot.primary_payload_dict()
            )
            fallback_result: dict[str, Any] | None = None
            fallback_payload = job.snapshot.fallback_payload_dict()
            if (
                not primary_result.get("ok")
                and fallback_payload is not None
                and not token.cancelled
            ):
                fallback_result = self._planner_call(fallback_payload)

        if token.cancelled:
            return self._cancelled_candidate(job)
        metadata = {
            "fallbackResult": fallback_result,
            "backend": self._backend_name(primary_result),
        }
        return PlanCandidate.from_result(
            job,
            primary_result,
            finished_at=monotonic(),
            metadata=metadata,
        )

    @staticmethod
    def _cancelled_candidate(job: PlanningJob) -> PlanCandidate:
        return PlanCandidate.from_result(
            job,
            {
                "ok": False,
                "plans": [],
                "debug": {"reason": "planning job cancelled"},
            },
            finished_at=monotonic(),
            metadata={"cancelled": True},
        )

    @staticmethod
    def _backend_name(result: dict[str, Any]) -> str:
        debug = result.get("debug")
        if not isinstance(debug, dict):
            return ""
        return str(debug.get("plannerBackend") or debug.get("backend") or "")


class PlanCommitStatus(str, Enum):
    COMMITTED = "committed"
    STALE = "stale"


@dataclass(frozen=True, slots=True)
class PlanCommitResult(Generic[CommitValueT]):
    status: PlanCommitStatus
    value: CommitValueT | None = None


class PlanCommitService:
    """Validate revision and apply one all-or-nothing runtime mutation."""

    def __init__(self, current_revision: Callable[[], int]) -> None:
        if not callable(current_revision):
            raise TypeError("current_revision must be callable")
        self._current_revision = current_revision

    def commit(
        self,
        candidate: PlanCandidate,
        *,
        validate: Callable[[], None],
        capture: Callable[[], CheckpointT],
        apply: Callable[[], CommitValueT],
        restore: Callable[[CheckpointT], None],
    ) -> PlanCommitResult[CommitValueT]:
        if candidate.expected_revision != self._current_revision():
            return PlanCommitResult(PlanCommitStatus.STALE)

        validate()
        # Validation can observe external robot input, so check again before
        # taking the rollback checkpoint.
        if candidate.expected_revision != self._current_revision():
            return PlanCommitResult(PlanCommitStatus.STALE)

        checkpoint = capture()
        try:
            value = apply()
        except BaseException:
            restore(checkpoint)
            raise
        return PlanCommitResult(PlanCommitStatus.COMMITTED, value)


class PlanningWorker:
    """Own one finite solver thread and a bounded priority queue.

    The thread drains currently accepted jobs and exits when the queue is
    empty. There is no permanently waiting non-daemon thread, which is useful
    for tests and small tools without an application lifecycle. ``submit``
    keeps the historical single-task API; ``submit_job`` is the scheduler API.
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
        self._thread_mode = ""
        self._close_requested = False
        self._submission_count = 0
        self._active_submission = 0
        self._last_failure: PlanningWorkerFailure | None = None
        self._completion_consumer: PlanningCompletionConsumer | None = None
        self._completed_results: deque[Any] = deque()
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

    def submit(
        self,
        task: PlanningTask,
        *,
        thread_name: str | None = None,
    ) -> bool:
        """Start ``task`` when the worker is idle.

        Returns ``False`` when another task is still alive or shutdown has
        started. Rejection never queues hidden work.
        """

        if not callable(task):
            raise TypeError("task must be callable")

        with self._condition:
            if (
                self._close_requested
                or self._state is not PlanningWorkerState.IDLE
            ):
                return False

            previous = self._thread
            if previous is not None:
                if previous.is_alive():
                    return False
                previous.join(0.0)

            self._submission_count += 1
            submission = self._submission_count
            self._active_submission = submission
            self._state = PlanningWorkerState.RUNNING
            name = str(thread_name or "").strip()
            if not name:
                name = f"{self._name}-{submission}"
            worker = Thread(
                target=self._run,
                args=(submission, task),
                name=name,
                daemon=False,
            )
            self._thread = worker
            self._thread_mode = "legacy"

            try:
                worker.start()
            except BaseException:
                if self._thread is worker:
                    self._thread = None
                    self._thread_mode = ""
                    self._active_submission = 0
                    self._state = PlanningWorkerState.IDLE
                    self._condition.notify_all()
                raise
        return True

    def publish_result(self, result: Any) -> None:
        """Publish worker output for the runtime thread to consume."""

        with self._condition:
            self._completed_results.append(result)
            self._condition.notify_all()

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
        if job.status is not PlanningJobStatus.QUEUED:
            raise ValueError("a submitted planning job must be queued")

        with self._condition:
            if self._close_requested:
                return False
            if (
                self._thread is not None
                and self._thread.is_alive()
                and self._thread_mode == "legacy"
            ):
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
                self._thread_mode = "scheduler"
                self._state = PlanningWorkerState.RUNNING
                try:
                    worker.start()
                except BaseException:
                    self._thread = None
                    self._thread_mode = ""
                    self._queued_jobs.pop(job.job_id, None)
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

    def take_completed_results(self) -> tuple[Any, ...]:
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
        self._publish_job_event_locked(
            worst,
            PlanningJobStatus.CANCELLED,
            "planning job evicted by higher-priority work",
        )
        return True

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
                if self._thread is current_thread():
                    self._thread = None
                    self._thread_mode = ""
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
                PlanningJobStatus.CANCELLED,
                message,
            )
        try:
            candidate = solver(job)
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
                PlanningJobStatus.CANCELLED,
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

    def _run(self, submission: int, task: PlanningTask) -> None:
        try:
            task()
        except Exception as exc:
            failure = PlanningWorkerFailure(
                occurred_at=monotonic(),
                submission=submission,
                exception_type=type(exc).__name__,
                message=str(exc),
            )
            with self._condition:
                self._last_failure = failure
        finally:
            with self._condition:
                if self._active_submission == submission:
                    self._active_submission = 0
                    self._thread_mode = ""
                    self._state = (
                        PlanningWorkerState.CLOSED
                        if self._close_requested
                        else PlanningWorkerState.IDLE
                    )
                    self._condition.notify_all()

    @staticmethod
    def _validated_timeout(timeout: float | None) -> float | None:
        if timeout is None:
            return None
        value = float(timeout)
        if not isfinite(value) or value < 0.0:
            raise ValueError("timeout must be a finite non-negative number")
        return value

