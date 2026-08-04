"""A small, owned execution loop for Fleet Manager runtime work.

The loop deliberately knows nothing about robots, maps, or planning.  Its only
job is to call one ``step`` function and explicit commands from one dedicated
thread. This keeps mutable runtime state under a single owner and makes the
component reusable by both real and simulated fleet runtimes.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import isfinite
from threading import Condition, Event, Thread, current_thread
from time import monotonic
from typing import Any, Callable, Generic, TypeVar


class RuntimeLoopState(str, Enum):
    """Lifecycle states of :class:`RuntimeLoop`."""

    STOPPED = "stopped"
    RUNNING = "running"
    STOPPING = "stopping"
    CLOSED = "closed"


@dataclass(frozen=True, slots=True)
class RuntimeLoopFailure:
    """Serializable description of a failed runtime step."""

    occurred_at: float
    attempt: int
    exception_type: str
    message: str
    operation: str = "step"


RuntimeStep = Callable[[], None]
RuntimeErrorHandler = Callable[[RuntimeLoopFailure], None]
RuntimeIntervalProvider = Callable[[], float]
CommandResultT = TypeVar("CommandResultT")


@dataclass(slots=True)
class _RuntimeCommand(Generic[CommandResultT]):
    callback: Callable[[], CommandResultT]
    completed: Event
    started: bool = False
    result: CommandResultT | None = None
    error: BaseException | None = None


class RuntimeLoop:
    """Run a state-owning callback periodically in one managed thread.

    ``start`` and ``stop`` are idempotent, and a stopped loop may be started
    again.  ``close`` is final: it stops the worker and prevents future starts.
    The worker is intentionally non-daemon, so an orderly application shutdown
    must close it instead of silently abandoning mutable runtime state.

    A step exception is recorded and passed to ``on_error``.  It does not kill
    the worker or delay the next scheduled step.  An exception raised by the
    error reporter is isolated as well.
    """

    def __init__(
        self,
        step: RuntimeStep,
        *,
        interval_seconds: float = 0.1,
        name: str = "fleet-runtime",
        on_error: RuntimeErrorHandler | None = None,
        interval_provider: RuntimeIntervalProvider | None = None,
    ) -> None:
        if not callable(step):
            raise TypeError("step must be callable")
        if on_error is not None and not callable(on_error):
            raise TypeError("on_error must be callable")
        if interval_provider is not None and not callable(interval_provider):
            raise TypeError("interval_provider must be callable")

        interval = self._positive_interval(
            interval_seconds,
            field_name="interval_seconds",
        )

        thread_name = str(name).strip()
        if not thread_name:
            raise ValueError("name must not be empty")

        self._step = step
        self._interval_seconds = interval
        self._name = thread_name
        self._on_error = on_error
        self._interval_provider = interval_provider

        self._condition = Condition()
        self._state = RuntimeLoopState.STOPPED
        self._thread: Thread | None = None
        self._stop_requested = False
        self._step_requested = False
        self._close_requested = False

        self._attempt_count = 0
        self._successful_step_count = 0
        self._failure_count = 0
        self._last_failure: RuntimeLoopFailure | None = None
        self._commands: deque[_RuntimeCommand[Any]] = deque()
        self._owner_thread: Thread | None = None

    @property
    def name(self) -> str:
        return self._name

    @property
    def interval_seconds(self) -> float:
        with self._condition:
            return self._interval_seconds

    @property
    def state(self) -> RuntimeLoopState:
        with self._condition:
            return self._state

    @property
    def is_running(self) -> bool:
        return self.state is RuntimeLoopState.RUNNING

    @property
    def attempt_count(self) -> int:
        with self._condition:
            return self._attempt_count

    @property
    def successful_step_count(self) -> int:
        with self._condition:
            return self._successful_step_count

    @property
    def failure_count(self) -> int:
        with self._condition:
            return self._failure_count

    @property
    def last_failure(self) -> RuntimeLoopFailure | None:
        with self._condition:
            return self._last_failure

    def start(self) -> bool:
        """Start the worker.

        Returns ``True`` when a new worker was started and ``False`` when the
        loop was already running or stopping.  Starting a closed loop is a
        programming error and raises ``RuntimeError``.
        """

        with self._condition:
            if self._close_requested or self._state is RuntimeLoopState.CLOSED:
                raise RuntimeError("a closed runtime loop cannot be started")
            if self._state is not RuntimeLoopState.STOPPED:
                return False

            self._stop_requested = False
            self._step_requested = False
            self._state = RuntimeLoopState.RUNNING
            worker = Thread(
                target=self._run,
                name=self._name,
                daemon=False,
            )
            self._thread = worker

            try:
                worker.start()
            except BaseException:
                if self._thread is worker:
                    self._thread = None
                    self._state = RuntimeLoopState.STOPPED
                    self._condition.notify_all()
                raise
        return True

    def request_step(self) -> bool:
        """Ask the owner thread to run one step as soon as possible.

        Multiple requests made before the owner can react are coalesced.  The
        callback is never executed in the requesting thread.
        """

        with self._condition:
            if self._state is not RuntimeLoopState.RUNNING:
                return False
            self._step_requested = True
            self._condition.notify()
            return True

    def wake(self) -> bool:
        """Readable alias for :meth:`request_step`."""

        return self.request_step()

    def execute(
        self,
        command: Callable[[], CommandResultT],
        *,
        timeout: float | None = None,
    ) -> CommandResultT:
        """Run a state mutation on the owner thread and return its result."""

        if not callable(command):
            raise TypeError("command must be callable")
        wait_seconds = self._validated_timeout(timeout)
        with self._condition:
            if self._owner_thread is current_thread():
                run_here = True
            elif self._state is RuntimeLoopState.STOPPED:
                run_here = True
            elif self._state is RuntimeLoopState.RUNNING:
                run_here = False
            else:
                raise RuntimeError("runtime loop is not accepting commands")
            if not run_here:
                pending = _RuntimeCommand(
                    callback=command,
                    completed=Event(),
                )
                self._commands.append(pending)
                self._condition.notify_all()

        if run_here:
            return command()
        if not pending.completed.wait(wait_seconds):
            with self._condition:
                if not pending.started:
                    try:
                        self._commands.remove(pending)
                    except ValueError:
                        pass
                    else:
                        raise TimeoutError(
                            "runtime command did not start in time"
                        )
            # Once execution has started, returning early would leave the
            # caller unsure whether its mutation was applied.  Wait for the
            # owner to finish and return the real outcome instead.
            pending.completed.wait()
        if pending.error is not None:
            raise pending.error
        return pending.result  # type: ignore[return-value]

    def stop(self, timeout: float | None = None) -> bool:
        """Stop the worker and wait for its current step to finish.

        Returns ``True`` after a clean stop.  A ``False`` result means the
        timeout expired (or the method was called by the worker itself); the
        stop request remains active and the thread will exit after its step.
        """

        wait_seconds = self._validated_timeout(timeout)
        with self._condition:
            if self._state in {
                RuntimeLoopState.STOPPED,
                RuntimeLoopState.CLOSED,
            }:
                return True

            self._stop_requested = True
            self._state = RuntimeLoopState.STOPPING
            worker = self._thread
            self._reject_pending_commands_locked()
            self._condition.notify_all()

        if worker is None:
            return True
        if worker is current_thread():
            return False

        worker.join(wait_seconds)
        return not worker.is_alive()

    def close(self, timeout: float | None = None) -> bool:
        """Permanently stop the loop.

        Closing is idempotent.  If a timeout expires while a step is blocked,
        the lifecycle remains ``STOPPING`` until that step returns; the worker
        then performs the final transition to ``CLOSED``.
        """

        wait_seconds = self._validated_timeout(timeout)
        with self._condition:
            if self._state is RuntimeLoopState.CLOSED:
                return True

            self._close_requested = True
            if self._state is RuntimeLoopState.STOPPED:
                self._state = RuntimeLoopState.CLOSED
                self._condition.notify_all()
                return True

            self._stop_requested = True
            self._state = RuntimeLoopState.STOPPING
            worker = self._thread
            self._reject_pending_commands_locked()
            self._condition.notify_all()

        if worker is None:
            return True
        if worker is current_thread():
            return False

        worker.join(wait_seconds)
        return not worker.is_alive()

    def _run(self) -> None:
        next_step_at = monotonic()
        self._owner_thread = current_thread()
        try:
            while True:
                with self._condition:
                    while True:
                        if self._stop_requested:
                            return

                        now = monotonic()
                        scheduled_step = now >= next_step_at
                        requested_step = self._step_requested
                        commands_waiting = bool(self._commands)
                        if scheduled_step or requested_step or commands_waiting:
                            self._step_requested = False
                            break

                        self._condition.wait(next_step_at - now)

                self._execute_pending_commands()
                should_step = scheduled_step or requested_step
                if not should_step:
                    continue

                self._execute_step()

                with self._condition:
                    if self._stop_requested:
                        return
                    attempt = self._attempt_count
                interval = self._next_interval(attempt)
                finished_at = monotonic()
                if scheduled_step:
                    next_step_at += interval
                if next_step_at <= finished_at:
                    skipped_intervals = (
                        int((finished_at - next_step_at) // interval)
                        + 1
                    )
                    next_step_at += skipped_intervals * interval
        finally:
            with self._condition:
                if self._thread is current_thread():
                    self._thread = None
                    self._owner_thread = None
                    self._state = (
                        RuntimeLoopState.CLOSED
                        if self._close_requested
                        else RuntimeLoopState.STOPPED
                    )
                    self._reject_pending_commands_locked()
                    self._condition.notify_all()

    def _execute_pending_commands(self) -> None:
        while True:
            with self._condition:
                if not self._commands:
                    return
                pending = self._commands.popleft()
                pending.started = True
            try:
                pending.result = pending.callback()
            except BaseException as exc:
                pending.error = exc
            finally:
                pending.completed.set()

    def _reject_pending_commands_locked(self) -> None:
        while self._commands:
            pending = self._commands.popleft()
            pending.error = RuntimeError("runtime loop stopped before command")
            pending.completed.set()

    def _execute_step(self) -> None:
        with self._condition:
            self._attempt_count += 1
            attempt = self._attempt_count

        try:
            self._step()
        except Exception as exc:
            self._record_failure(exc, attempt=attempt, operation="step")
        else:
            with self._condition:
                self._successful_step_count += 1

    def _next_interval(self, attempt: int) -> float:
        provider = self._interval_provider
        if provider is None:
            return self.interval_seconds
        try:
            interval = self._positive_interval(
                provider(),
                field_name="interval_provider result",
            )
        except Exception as exc:
            self._record_failure(
                exc,
                attempt=attempt,
                operation="interval_provider",
            )
            return self.interval_seconds

        with self._condition:
            self._interval_seconds = interval
        return interval

    def _record_failure(
        self,
        exc: Exception,
        *,
        attempt: int,
        operation: str,
    ) -> None:
        failure = RuntimeLoopFailure(
            occurred_at=monotonic(),
            attempt=attempt,
            exception_type=type(exc).__name__,
            message=str(exc),
            operation=operation,
        )
        with self._condition:
            self._failure_count += 1
            self._last_failure = failure

        if self._on_error is not None:
            try:
                self._on_error(failure)
            except Exception:
                # Error reporting must never terminate the state owner.
                pass

    @staticmethod
    def _validated_timeout(timeout: float | None) -> float | None:
        if timeout is None:
            return None
        value = float(timeout)
        if not isfinite(value) or value < 0.0:
            raise ValueError("timeout must be a finite non-negative number")
        return value

    @staticmethod
    def _positive_interval(value: float, *, field_name: str) -> float:
        interval = float(value)
        if not isfinite(interval) or interval <= 0.0:
            raise ValueError(
                f"{field_name} must be a finite positive number"
            )
        return interval
