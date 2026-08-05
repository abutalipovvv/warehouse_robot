from __future__ import annotations

from threading import Event, Lock
from time import monotonic, sleep

import pytest

from fleet_manager.manager.planning import (
    FrozenMapping,
    PlanCommitService,
    PlanCommitStatus,
    PlanningJob,
    PlanningJobStatus,
    PlanningPriority,
    PlanningReason,
    PlanningSnapshot,
    PlanningSolverService,
)
from fleet_manager.manager.scheduler import PlanningWorker


def _snapshot(
    primary: dict[str, object],
    fallback: dict[str, object] | None = None,
    *,
    revision: int = 1,
    strict_stationary_avoidance: bool = False,
) -> PlanningSnapshot:
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
        primary_payload=FrozenMapping.from_mapping(primary),
        fallback_payload=(
            FrozenMapping.from_mapping(fallback)
            if fallback is not None
            else None
        ),
        strict_stationary_avoidance=strict_stationary_avoidance,
    )


def _job(
    job_id: str,
    primary: dict[str, object],
    fallback: dict[str, object] | None = None,
    *,
    revision: int = 1,
    deadline: float | None = None,
    priority: PlanningPriority = PlanningPriority.ORDER_DISPATCH,
) -> PlanningJob:
    return PlanningJob(
        job_id=job_id,
        reason=PlanningReason[priority.name],
        priority=priority,
        snapshot=_snapshot(primary, fallback, revision=revision),
        submitted_at=monotonic(),
        deadline=deadline,
    )


def _result(name: str, *, ok: bool) -> dict[str, object]:
    return {
        "ok": ok,
        "plans": (
            [{"robot": "r1", "nodes": ["A", name]}]
            if ok
            else []
        ),
        "debug": {"backend": name, "reason": name},
    }


def test_primary_success_does_not_run_fallback() -> None:
    calls: list[str] = []

    def planner(payload: dict[str, object]) -> dict[str, object]:
        name = str(payload["name"])
        calls.append(name)
        return _result(name, ok=True)

    service = PlanningSolverService(planner, Lock())
    candidate = service.solve(
        _job("primary", {"name": "primary"}, {"name": "fallback"})
    )

    assert calls == ["primary"]
    assert candidate.result.get("ok") is True
    assert candidate.backend_used == "primary"
    assert candidate.metadata.get("selectedSource") == "primary"


def test_successful_fallback_is_the_candidate_result() -> None:
    calls: list[str] = []

    def planner(payload: dict[str, object]) -> dict[str, object]:
        name = str(payload["name"])
        calls.append(name)
        return _result(name, ok=name == "fallback")

    service = PlanningSolverService(planner, Lock())
    candidate = service.solve(
        _job("fallback", {"name": "primary"}, {"name": "fallback"})
    )

    assert calls == ["primary", "fallback"]
    assert candidate.result.get("ok") is True
    assert candidate.result.get("plans")[0]["nodes"] == ["A", "fallback"]
    assert candidate.backend_used == "fallback"
    assert candidate.metadata.get("selectedSource") == "fallback"
    assert candidate.metadata.get("primaryResult")["ok"] is False
    assert candidate.metadata.get("fallbackResult")["ok"] is True


def test_successful_fallback_is_applied_by_commit_service() -> None:
    def planner(payload: dict[str, object]) -> dict[str, object]:
        name = str(payload["name"])
        return _result(name, ok=name == "fallback")

    candidate = PlanningSolverService(planner, Lock()).solve(
        _job("fallback-commit", {"name": "primary"}, {"name": "fallback"})
    )
    live_route = ["old"]
    outcome = PlanCommitService(lambda: 1).commit(
        candidate,
        validate=lambda: None,
        capture=lambda: list(live_route),
        apply=lambda: live_route.__setitem__(
            slice(None),
            candidate.result.get("plans")[0]["nodes"],
        ),
        restore=lambda checkpoint: live_route.__setitem__(
            slice(None),
            checkpoint,
        ),
    )

    assert outcome.status is PlanCommitStatus.COMMITTED
    assert live_route == ["A", "fallback"]
    assert candidate.backend_used == "fallback"


def test_failed_primary_and_fallback_produce_failure_candidate() -> None:
    def planner(payload: dict[str, object]) -> dict[str, object]:
        return _result(str(payload["name"]), ok=False)

    service = PlanningSolverService(planner, Lock())
    candidate = service.solve(
        _job("failed", {"name": "primary"}, {"name": "fallback"})
    )

    assert candidate.result.get("ok") is False
    assert candidate.plans == ()
    assert candidate.metadata.get("primaryResult")["ok"] is False
    assert candidate.metadata.get("fallbackResult")["ok"] is False


def test_primary_exception_keeps_existing_no_fallback_policy() -> None:
    calls: list[str] = []

    def planner(payload: dict[str, object]) -> dict[str, object]:
        calls.append(str(payload["name"]))
        raise RuntimeError("primary exploded")

    service = PlanningSolverService(planner, Lock())
    with pytest.raises(RuntimeError, match="primary exploded"):
        service.solve(
            _job("exception", {"name": "primary"}, {"name": "fallback"})
        )
    assert calls == ["primary"]


def test_stale_fallback_candidate_is_not_committed() -> None:
    def planner(payload: dict[str, object]) -> dict[str, object]:
        return _result(
            str(payload["name"]),
            ok=payload["name"] == "fallback",
        )

    candidate = PlanningSolverService(planner, Lock()).solve(
        _job(
            "stale-fallback",
            {"name": "primary"},
            {"name": "fallback"},
            revision=10,
        )
    )
    live_routes = ["old"]
    outcome = PlanCommitService(lambda: 11).commit(
        candidate,
        validate=lambda: None,
        capture=lambda: list(live_routes),
        apply=lambda: live_routes.append("fallback"),
        restore=lambda checkpoint: live_routes.__setitem__(slice(None), checkpoint),
    )

    assert outcome.status is PlanCommitStatus.STALE
    assert live_routes == ["old"]


def test_cancellation_stops_solver_and_releases_worker_for_safety_job() -> None:
    started = Event()

    def planner(
        payload: dict[str, object],
        *,
        should_cancel,
    ) -> dict[str, object]:
        if payload["name"] == "long":
            started.set()
            while True:
                should_cancel()
                sleep(0.001)
        return _result(str(payload["name"]), ok=True)

    service = PlanningSolverService(planner, Lock(), accepts_control=True)
    worker = PlanningWorker(max_queue_size=2)
    long_job = _job("long", {"name": "long"})
    safety_job = _job(
        "safety",
        {"name": "safety"},
        priority=PlanningPriority.SAFETY_REPLAN,
    )

    assert worker.submit_job(long_job, service.solve)
    assert started.wait(1.0)
    assert worker.cancel_job(long_job.job_id)
    assert worker.submit_job(safety_job, service.solve)
    assert worker.join(1.0)

    statuses = {
        event.job_id: event.status
        for event in worker.take_job_events()
        if event.status is not PlanningJobStatus.RUNNING
    }
    results = {item.job_id: item for item in worker.take_completed_results()}
    assert statuses["long"] is PlanningJobStatus.CANCELLED
    assert statuses["safety"] is PlanningJobStatus.COMPLETED
    assert results["long"].result.get("ok") is False
    assert results["safety"].result.get("ok") is True
    assert worker.close()


def test_deadline_stops_cooperative_solver() -> None:
    def planner(
        _payload: dict[str, object],
        *,
        should_cancel,
    ) -> dict[str, object]:
        while True:
            should_cancel()
            sleep(0.001)

    service = PlanningSolverService(planner, Lock(), accepts_control=True)
    worker = PlanningWorker()
    job = _job(
        "deadline",
        {"name": "long"},
        deadline=monotonic() + 0.02,
    )

    assert worker.submit_job(job, service.solve)
    assert worker.join(1.0)
    terminal = [
        event
        for event in worker.take_job_events()
        if event.status is not PlanningJobStatus.RUNNING
    ]
    assert terminal[-1].status is PlanningJobStatus.DEADLINE_EXCEEDED
    assert worker.take_completed_results()[0].result.get("ok") is False
    assert worker.close()


def test_shutdown_cooperatively_stops_running_solver() -> None:
    started = Event()

    def planner(
        _payload: dict[str, object],
        *,
        should_cancel,
    ) -> dict[str, object]:
        started.set()
        while True:
            should_cancel()
            sleep(0.001)

    service = PlanningSolverService(planner, Lock(), accepts_control=True)
    worker = PlanningWorker()
    assert worker.submit_job(_job("shutdown", {"name": "long"}), service.solve)
    assert started.wait(1.0)
    assert worker.close(1.0)
