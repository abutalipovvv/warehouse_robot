from __future__ import annotations

from threading import Event, Thread, current_thread
from time import monotonic, sleep

import pytest

from fleet_manager.core.fleet.management.manager import FleetManagerCore
from fleet_manager.core.planning_scheduler import (
    PlanningWorker,
    PlanningWorkerState,
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


def test_worker_owns_one_finite_non_daemon_thread() -> None:
    worker = PlanningWorker(name="test-planner")
    started = Event()
    release = Event()
    observations: list[tuple[str, bool]] = []

    def task() -> None:
        observations.append(
            (current_thread().name, current_thread().daemon)
        )
        started.set()
        release.wait(1.0)

    assert worker.state is PlanningWorkerState.IDLE
    assert worker.submit(task, thread_name="test-mapf-job")
    assert started.wait(1.0)
    assert worker.state is PlanningWorkerState.RUNNING
    assert worker.active_submission == 1
    assert not worker.submit(lambda: None)

    release.set()
    assert worker.join(timeout=1.0)
    assert worker.state is PlanningWorkerState.IDLE
    assert worker.active_submission == 0
    assert observations == [("test-mapf-job", False)]
    assert worker.close()


def test_worker_can_run_a_new_job_after_previous_thread_exits() -> None:
    worker = PlanningWorker()
    completed: list[int] = []

    assert worker.submit(lambda: completed.append(1))
    assert worker.join(timeout=1.0)
    assert worker.submit(lambda: completed.append(2))
    assert worker.join(timeout=1.0)

    assert completed == [1, 2]
    assert worker.submission_count == 2
    assert worker.close()


def test_unexpected_task_exception_is_recorded_and_worker_recovers() -> None:
    worker = PlanningWorker()

    def fail() -> None:
        raise ValueError("planner exploded")

    assert worker.submit(fail)
    assert worker.join(timeout=1.0)
    assert _wait_until(lambda: worker.state is PlanningWorkerState.IDLE)

    failure = worker.last_failure
    assert failure is not None
    assert failure.submission == 1
    assert failure.exception_type == "ValueError"
    assert failure.message == "planner exploded"

    recovered = Event()
    assert worker.submit(recovered.set)
    assert recovered.wait(1.0)
    assert worker.join(timeout=1.0)
    assert worker.close()


def test_timed_out_close_rejects_work_until_job_finishes() -> None:
    worker = PlanningWorker()
    started = Event()
    release = Event()

    def task() -> None:
        started.set()
        release.wait(1.0)

    assert worker.submit(task)
    assert started.wait(1.0)
    assert not worker.close(timeout=0.01)
    assert worker.state is PlanningWorkerState.CLOSING
    assert not worker.submit(lambda: None)

    release.set()
    assert worker.close(timeout=1.0)
    assert worker.state is PlanningWorkerState.CLOSED
    assert worker.close()
    assert not worker.submit(lambda: None)


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
    stale_job = {"kind": "dispatch", "done": False, "result": None}
    replacement_job = {
        "kind": "prefetch",
        "done": False,
        "result": None,
    }

    def plan(_requests, _payload):
        planning_started.set()
        release_planning.wait(1.0)
        return {"ok": True, "plans": []}

    monkeypatch.setattr(manager, "_plan_valid_requests", plan)
    with manager._dispatch_job_lock:
        manager._dispatch_job = stale_job
    assert manager._submit_async_planning_job(
        stale_job,
        [],
        {},
        failure_reason="background planner failed",
        thread_name="test-stale-mapf",
    )
    assert planning_started.wait(1.0)

    with manager._dispatch_job_lock:
        manager._dispatch_job = replacement_job
    release_planning.set()
    assert manager._planning_worker.join(timeout=1.0)

    assert stale_job == {
        "kind": "dispatch",
        "done": False,
        "result": None,
    }
    assert replacement_job == {
        "kind": "prefetch",
        "done": False,
        "result": None,
    }
    with manager._dispatch_job_lock:
        manager._dispatch_job = None
    manager.close()


def test_dispatch_helper_preserves_background_failure_reason(
    monkeypatch,
) -> None:
    manager = _manager()
    job = {"kind": "dispatch", "done": False, "result": None}

    def fail(_requests, _payload):
        raise RuntimeError("no route")

    monkeypatch.setattr(manager, "_plan_valid_requests", fail)
    with manager._dispatch_job_lock:
        manager._dispatch_job = job
    assert manager._submit_async_planning_job(
        job,
        [],
        {},
        failure_reason="background planner failed",
        thread_name="test-failed-mapf",
    )
    assert manager._planning_worker.join(timeout=1.0)

    assert job["done"] is True
    assert job["result"] == {
        "ok": False,
        "plans": [],
        "debug": {
            "reason": "background planner failed: no route",
        },
    }
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
    job = {
        "kind": "dispatch",
        "done": False,
        "result": None,
        "corridor_gates": corridor_gates,
    }

    def plan(_requests, _payload):
        planning_started.set()
        release_planning.wait(1.0)
        return {"ok": True, "plans": []}

    monkeypatch.setattr(manager, "_plan_valid_requests", plan)
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
        failure_reason="background planner failed",
        thread_name="test-close-mapf",
    )
    assert planning_started.wait(1.0)

    closer = Thread(
        target=lambda: (manager.close(), close_finished.set()),
        name="test-manager-close",
        daemon=False,
    )
    closer.start()
    assert _wait_until(lambda: bool(job.get("discard")))
    assert not close_finished.is_set()

    release_planning.set()
    closer.join(1.0)
    assert not closer.is_alive()
    assert close_finished.is_set()
    assert job["discard"] is True
    assert manager._dispatch_job is None
    assert manager._planning_worker.state is PlanningWorkerState.CLOSED
    assert released_gates == [corridor_gates]
