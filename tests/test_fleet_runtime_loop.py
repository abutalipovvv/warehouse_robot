from __future__ import annotations

from threading import Event, Lock, current_thread, get_ident
from time import monotonic, sleep

import pytest

from fleet_manager.runtime.loop import (
    RuntimeLoop,
    RuntimeLoopFailure,
    RuntimeLoopState,
)


def _wait_until(
    predicate,
    *,
    timeout: float = 1.0,
    interval: float = 0.002,
) -> bool:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return True
        sleep(interval)
    return predicate()


def test_loop_runs_all_steps_in_one_non_daemon_owner_thread() -> None:
    calls: list[tuple[int, bool]] = []
    calls_lock = Lock()
    enough_calls = Event()

    def step() -> None:
        with calls_lock:
            calls.append((get_ident(), current_thread().daemon))
            if len(calls) >= 3:
                enough_calls.set()

    loop = RuntimeLoop(
        step,
        interval_seconds=0.01,
        name="test-fleet-owner",
    )

    assert loop.state is RuntimeLoopState.STOPPED
    assert loop.start() is True
    assert loop.start() is False
    assert enough_calls.wait(1.0)
    assert loop.stop(timeout=1.0) is True
    assert loop.state is RuntimeLoopState.STOPPED

    with calls_lock:
        owner_ids = {owner_id for owner_id, _daemon in calls}
        daemon_values = {daemon for _owner_id, daemon in calls}
        count_after_stop = len(calls)

    sleep(0.03)
    assert len(calls) == count_after_stop
    assert owner_ids == {calls[0][0]}
    assert calls[0][0] != get_ident()
    assert daemon_values == {False}
    assert loop.attempt_count == count_after_stop
    assert loop.successful_step_count == count_after_stop
    assert loop.failure_count == 0

    assert loop.stop() is True
    assert loop.close() is True


def test_request_step_wakes_owner_without_using_calling_thread() -> None:
    owner_ids: list[int] = []
    first_step = Event()
    requested_step = Event()

    def step() -> None:
        owner_ids.append(get_ident())
        if len(owner_ids) == 1:
            first_step.set()
        if len(owner_ids) == 2:
            requested_step.set()

    loop = RuntimeLoop(step, interval_seconds=30.0)
    loop.start()
    assert first_step.wait(1.0)

    caller_id = get_ident()
    started_at = monotonic()
    assert loop.request_step() is True
    assert requested_step.wait(0.5)
    assert monotonic() - started_at < 0.5
    assert owner_ids == [owner_ids[0], owner_ids[0]]
    assert owner_ids[0] != caller_id

    assert loop.wake() is True
    assert _wait_until(lambda: loop.successful_step_count == 3)
    assert loop.close(timeout=1.0) is True
    assert loop.request_step() is False
    assert loop.wake() is False


def test_step_and_error_reporter_failures_do_not_stop_loop() -> None:
    reports: list[RuntimeLoopFailure] = []
    recovered = Event()

    def step() -> None:
        if len(reports) < 2:
            raise ValueError(f"failure {len(reports) + 1}")
        recovered.set()

    def report(failure: RuntimeLoopFailure) -> None:
        reports.append(failure)
        if len(reports) == 1:
            raise RuntimeError("reporter failure must be isolated")

    loop = RuntimeLoop(
        step,
        interval_seconds=0.01,
        on_error=report,
    )
    loop.start()

    assert recovered.wait(1.0)
    assert loop.is_running
    assert loop.failure_count == 2
    assert loop.successful_step_count >= 1
    assert [failure.attempt for failure in reports] == [1, 2]
    assert [failure.exception_type for failure in reports] == [
        "ValueError",
        "ValueError",
    ]
    assert loop.last_failure == reports[-1]
    assert loop.close(timeout=1.0)


def test_interval_provider_updates_the_period_from_owner_thread() -> None:
    configured_interval = [0.08]
    call_times: list[float] = []
    provider_threads: list[int] = []
    enough_calls = Event()

    def step() -> None:
        call_times.append(monotonic())
        if len(call_times) >= 3:
            enough_calls.set()

    def interval_provider() -> float:
        provider_threads.append(get_ident())
        configured_interval[0] = 0.01
        return configured_interval[0]

    loop = RuntimeLoop(
        step,
        interval_seconds=0.08,
        interval_provider=interval_provider,
    )
    loop.start()

    assert enough_calls.wait(1.0)
    assert loop.close(timeout=1.0)
    assert loop.interval_seconds == pytest.approx(0.01)
    assert len(set(provider_threads)) == 1
    assert provider_threads[0] != get_ident()
    assert call_times[2] - call_times[1] < 0.05


def test_invalid_interval_provider_is_reported_and_uses_last_interval() -> None:
    reports: list[RuntimeLoopFailure] = []
    second_step = Event()
    calls = 0

    def step() -> None:
        nonlocal calls
        calls += 1
        if calls >= 2:
            second_step.set()

    loop = RuntimeLoop(
        step,
        interval_seconds=0.01,
        interval_provider=lambda: 0.0,
        on_error=reports.append,
    )
    loop.start()

    assert second_step.wait(1.0)
    assert loop.close(timeout=1.0)
    assert reports
    assert reports[0].operation == "interval_provider"
    assert reports[0].exception_type == "ValueError"
    assert loop.interval_seconds == pytest.approx(0.01)


def test_blocked_step_exposes_stopping_state_until_clean_join() -> None:
    step_started = Event()
    release_step = Event()

    def step() -> None:
        step_started.set()
        release_step.wait(1.0)

    loop = RuntimeLoop(step, interval_seconds=1.0)
    loop.start()
    assert step_started.wait(1.0)

    assert loop.stop(timeout=0.01) is False
    assert loop.state is RuntimeLoopState.STOPPING
    assert loop.start() is False

    release_step.set()
    assert _wait_until(lambda: loop.state is RuntimeLoopState.STOPPED)
    assert loop.stop() is True
    assert loop.close() is True


def test_stopped_loop_can_restart_but_closed_loop_cannot() -> None:
    calls = 0
    called = Event()

    def step() -> None:
        nonlocal calls
        calls += 1
        called.set()

    loop = RuntimeLoop(step, interval_seconds=1.0)
    assert loop.start()
    assert called.wait(1.0)
    assert loop.stop(timeout=1.0)

    called.clear()
    assert loop.start()
    assert called.wait(1.0)
    assert calls == 2
    assert loop.close(timeout=1.0)
    assert loop.state is RuntimeLoopState.CLOSED
    assert loop.close() is True

    with pytest.raises(RuntimeError, match="closed"):
        loop.start()


@pytest.mark.parametrize(
    "interval",
    [0.0, -1.0, float("inf"), float("nan")],
)
def test_interval_must_be_finite_and_positive(interval: float) -> None:
    with pytest.raises(ValueError, match="interval_seconds"):
        RuntimeLoop(lambda: None, interval_seconds=interval)


@pytest.mark.parametrize(
    "timeout",
    [-1.0, float("inf"), float("nan")],
)
def test_stop_timeout_must_be_finite_and_non_negative(
    timeout: float,
) -> None:
    loop = RuntimeLoop(lambda: None)
    with pytest.raises(ValueError, match="timeout"):
        loop.stop(timeout)
    assert loop.close()
