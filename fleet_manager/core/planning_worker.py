"""Owned, single-job worker for finite Fleet Manager planning tasks."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from threading import Condition, Thread, current_thread
from time import monotonic
from typing import Callable


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


PlanningTask = Callable[[], None]


class PlanningWorker:
    """Own at most one finite planning thread.

    A thread is created only for an accepted task and exits as soon as that
    task returns. There is no permanently waiting non-daemon thread, which is
    important for unit tests and small command-line tools that do not own an
    explicit application lifecycle.
    """

    def __init__(self, *, name: str = "fleet-planning-worker") -> None:
        worker_name = str(name).strip()
        if not worker_name:
            raise ValueError("name must not be empty")

        self._name = worker_name
        self._condition = Condition()
        self._state = PlanningWorkerState.IDLE
        self._thread: Thread | None = None
        self._close_requested = False
        self._submission_count = 0
        self._active_submission = 0
        self._last_failure: PlanningWorkerFailure | None = None

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

            try:
                worker.start()
            except BaseException:
                if self._thread is worker:
                    self._thread = None
                    self._active_submission = 0
                    self._state = PlanningWorkerState.IDLE
                    self._condition.notify_all()
                raise
        return True

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
        return not worker.is_alive()

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


__all__ = [
    "PlanningTask",
    "PlanningWorker",
    "PlanningWorkerFailure",
    "PlanningWorkerState",
]
