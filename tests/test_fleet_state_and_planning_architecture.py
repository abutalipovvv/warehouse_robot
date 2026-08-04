from __future__ import annotations

from dataclasses import FrozenInstanceError
from threading import Event, Thread, current_thread
from time import monotonic, sleep

import pytest

from fleet_manager.core.fleet.management.manager import FleetManagerCore
from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.core.planning_scheduler import (
    PlanCommitService,
    PlanCommitStatus,
)
from fleet_manager.core.tasks.order_admission import OrderAdmissionService
from fleet_manager.core.tasks.rolling_continuation import (
    RollingContinuationService,
)
from fleet_manager.core.manager_state import (
    FleetState,
    PlanningSnapshotFactory,
    PlanningState,
    TrafficState,
)
from fleet_manager.core.planning_models import (
    FrozenMapping,
    PlanCandidate,
    PlanningJob,
    PlanningJobStatus,
    PlanningPriority,
    PlanningReason,
    PlanningSnapshot,
)
from fleet_manager.runtime.loop import RuntimeLoop


def _graph() -> tuple[dict[str, Landmark], list[GraphEdge]]:
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
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
        properties={"direction": 1},
    )
    return landmarks, [edge]


def _empty_snapshot(revision: int) -> PlanningSnapshot:
    return PlanningSnapshot(
        revision=revision,
        created_at=1.0,
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


def _candidate(revision: int) -> PlanCandidate:
    return PlanCandidate(
        expected_revision=revision,
        job_id="job-1",
        reason=PlanningReason.ORDER_DISPATCH,
        created_at=1.0,
        finished_at=2.0,
        result=FrozenMapping.from_mapping({"ok": True, "plans": []}),
    )


def test_compatibility_properties_are_the_same_state_objects() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    try:
        assert manager.robots is manager.fleet_state.robots
        assert manager.task_manager is manager.fleet_state.task_manager
        assert manager._runtime_replans is manager.planning_state.runtime_replans
        assert (
            manager._controlled_corridor_leases
            is manager.traffic_state.controlled_corridor_leases
        )
        assert (
            manager._active_wait_cycles
            is manager.recovery_state.active_wait_cycles
        )
    finally:
        manager.close()


def test_reset_clears_the_owned_containers_without_replacing_collections() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    replans = manager.planning_state.runtime_replans
    leases = manager.traffic_state.controlled_corridor_leases
    cycles = manager.recovery_state.active_wait_cycles
    replans["r1"] = {"stage": "queued"}
    leases["c1"] = ("r1", 10.0)
    cycles[("r1", "r2")] = {"winner": "r1"}
    try:
        manager.reset_planning_runtime_state()
        assert manager.planning_state.runtime_replans is replans
        assert manager.traffic_state.controlled_corridor_leases is leases
        assert manager.recovery_state.active_wait_cycles is cycles
        assert replans == {}
        assert leases == {}
        assert cycles == {}
    finally:
        manager.close()


def test_planning_snapshot_has_no_mutable_link_to_live_state() -> None:
    fleet_state = FleetState()
    traffic_state = TrafficState()
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    fleet_state.robots[robot.name] = robot
    traffic_state.controlled_corridor_leases["c1"] = ("r1", 8.0)
    snapshot = PlanningSnapshotFactory(fleet_state, traffic_state).create(
        created_at=2.0,
        requests=[{"name": "r1", "startLm": "A", "goalLm": "B"}],
        primary_payload={
            "robots": [{"name": "r1"}],
            "reserved_vertex_intervals": [
                {"robot": "r2", "node": "A", "start": 1.0, "end": 2.0}
            ],
        },
        blockers=["B"],
    )

    robot.pose["x"] = 99.0
    request_copy = snapshot.request_dicts()
    request_copy[0]["name"] = "changed"

    assert snapshot.robots[0].pose.get("x") == 0.0
    assert snapshot.request_dicts()[0]["name"] == "r1"
    assert snapshot.reservations[0].resource == ("A",)
    assert snapshot.traffic_resources[0].resource_id == "c1"
    assert not hasattr(snapshot, "__dict__")
    assert not hasattr(snapshot, "manager")
    assert not hasattr(snapshot, "runtime_loop")
    with pytest.raises(FrozenInstanceError):
        snapshot.revision = 20  # type: ignore[misc]


def test_revision_changes_only_for_planning_relevant_robot_update() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    try:
        manager.update_robot({"name": "r1", "currentLm": "A"})
        first_revision = manager.planning_revision
        manager.update_robot({"name": "r1"})
        assert manager.planning_revision == first_revision
        manager.update_robot({"name": "r1", "status": "STOPPED"})
        assert manager.planning_revision == first_revision + 1

        manager.robots["r1"].updated_at += 10.0
        manager._synchronize_planning_revision()
        assert manager.planning_revision == first_revision + 1
        manager.traffic_state.stationary_blockers["A"] = "r1"
        manager._synchronize_planning_revision()
        assert manager.planning_revision == first_revision + 2
    finally:
        manager.close()


def test_planning_job_captures_preparation_changes_in_its_revision() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    try:
        initial_revision = manager.planning_revision
        manager.traffic_state.stationary_blockers["A"] = "r1"

        planning_job = manager._build_planning_job(
            {"kind": "dispatch"},
            [],
            {},
        )

        assert manager.planning_revision == initial_revision + 1
        assert planning_job.snapshot.revision == manager.planning_revision
        assert manager._synchronize_planning_revision() == initial_revision + 1
    finally:
        manager.close()


def test_plan_commit_rejects_stale_candidate_without_mutation() -> None:
    revision = 11
    routes = ["old-route"]
    reservations = ["old-reservation"]
    service = PlanCommitService(lambda: revision)

    outcome = service.commit(
        _candidate(10),
        validate=lambda: None,
        capture=lambda: (list(routes), list(reservations)),
        apply=lambda: routes.append("new-route"),
        restore=lambda checkpoint: None,
    )

    assert outcome.status is PlanCommitStatus.STALE
    assert routes == ["old-route"]
    assert reservations == ["old-reservation"]


def test_manager_marks_old_candidate_stale_and_requeues_replan() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        trajectory=[{"t": 0.0, "x": 0.0}, {"t": 1.0, "x": 1.0}],
    )
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        status="PLANNING",
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    manager.traffic_state.temporal_reservations.append(
        {"robot": "other", "node": "A"}
    )
    reservations_before = list(manager.traffic_state.temporal_reservations)
    route_before = list(robot.trajectory)
    manager.fleet_state.revision.value = 10
    planning_job = PlanningJob(
        job_id="runtime_replan-1",
        reason=PlanningReason.SAFETY_REPLAN,
        priority=PlanningPriority.SAFETY_REPLAN,
        snapshot=_empty_snapshot(10),
        submitted_at=1.0,
        status=PlanningJobStatus.COMPLETED,
    )
    candidate = PlanCandidate.from_result(
        planning_job,
        {
            "ok": True,
            "plans": [{"robot": "r1", "nodes": ["A", "B"]}],
        },
        finished_at=2.0,
    )
    manager.planning_state.jobs[planning_job.job_id] = planning_job
    manager.planning_state.runtime_replans["r1"] = {"stage": "planning"}
    live_job = {
        "job_id": planning_job.job_id,
        "kind": "runtime_replan",
        "robot_name": "r1",
        "planning_job": planning_job,
        "entries": [(order, robot, {}, "B")],
    }

    try:
        manager.fleet_state.revision.value = 11
        manager._reject_stale_planning_candidate(live_job, candidate)

        assert robot.trajectory == route_before
        assert manager.traffic_state.temporal_reservations == reservations_before
        assert order.status == "QUEUED"
        assert manager.planning_state.runtime_replans["r1"]["stage"] == "queued"
        assert planning_job.status is PlanningJobStatus.STALE
        assert manager.planning_state.stale_candidates == 1
    finally:
        manager.close()


def test_plan_commit_rolls_back_route_and_reservations_on_failure() -> None:
    routes = ["old-route"]
    reservations = ["old-reservation"]
    service = PlanCommitService(lambda: 10)

    def capture() -> tuple[list[str], list[str]]:
        return list(routes), list(reservations)

    def restore(checkpoint: tuple[list[str], list[str]]) -> None:
        routes[:] = checkpoint[0]
        reservations[:] = checkpoint[1]

    def apply() -> None:
        routes.append("half-route")
        reservations.append("half-reservation")
        raise RuntimeError("validation failed during apply")

    with pytest.raises(RuntimeError, match="validation failed"):
        service.commit(
            _candidate(10),
            validate=lambda: None,
            capture=capture,
            apply=apply,
            restore=restore,
        )

    assert routes == ["old-route"]
    assert reservations == ["old-reservation"]


def test_planning_job_state_machine_rejects_invalid_transition() -> None:
    job = PlanningJob(
        job_id="job-1",
        reason=PlanningReason.SAFETY_REPLAN,
        priority=PlanningPriority.SAFETY_REPLAN,
        snapshot=_empty_snapshot(1),
        submitted_at=1.0,
    )
    job.transition(PlanningJobStatus.RUNNING)
    job.transition(PlanningJobStatus.COMPLETED)
    job.transition(PlanningJobStatus.COMMITTED)

    with pytest.raises(ValueError, match="cannot transition"):
        job.transition(PlanningJobStatus.RUNNING)


def test_order_admission_service_works_without_full_manager() -> None:
    fleet_state = FleetState()
    fleet_state.robots["r1"] = FleetRobot(name="r1", current_lm="A")
    service = OrderAdmissionService(
        fleet_state,
        {"A": object(), "B": object()},
        lambda: 12.5,
    )

    order = service.build(
        {
            "id": "o1",
            "vehicle": "r1",
            "targets": ["A", {"targetLm": "B"}],
            "speed": "0.8",
        }
    )

    assert order.order_id == "o1"
    assert order.targets == ["A", "B"]
    assert order.speed == 0.8
    assert order.created_at == 12.5
    with pytest.raises(ValueError, match="unknown robot"):
        service.build({"targetLm": "B", "vehicle": "missing"})


def test_rolling_continuation_service_works_without_manager() -> None:
    state = PlanningState()
    state.rolling_prefetch_eligible_since["r1"] = 4.0
    state.rolling_prefetch_failures["r1"] = 2
    service = RollingContinuationService(state, lambda order: 0.5)
    order = FleetOrder(order_id="o1", target_lm="B", updated_at=5.0)
    robot = FleetRobot(name="r1", current_lm="A", updated_at=6.0)

    assert service.boundary_wait_since(order, robot, 10.0) == 4.0
    assert service.boundary_priority(order, robot, 10.0) == (5.0, 4.0, "r1")
    assert service.candidate_priority(order, robot, 2.0, 10.0) == (
        12.0,
        0.0,
        4.0,
        "r1",
    )


def test_runtime_commands_execute_on_owner_thread_without_extra_step() -> None:
    steps: list[str] = []
    first_step = Event()

    def step() -> None:
        steps.append(current_thread().name)
        first_step.set()

    loop = RuntimeLoop(
        step,
        interval_seconds=10.0,
        name="state-owner",
    )
    assert loop.start()
    try:
        assert first_step.wait(1.0)
        initial_attempts = loop.attempt_count
        result: list[str] = []

        def call() -> None:
            result.append(loop.execute(lambda: current_thread().name))

        caller = Thread(target=call, name="api-caller")
        caller.start()
        caller.join(1.0)
        assert result == ["state-owner"]
        assert loop.attempt_count == initial_attempts
    finally:
        assert loop.close()


def test_runtime_command_propagates_error_and_stop_rejects_pending() -> None:
    step_started = Event()
    release_step = Event()

    def step() -> None:
        step_started.set()
        release_step.wait(1.0)

    loop = RuntimeLoop(step, interval_seconds=10.0, name="blocked-owner")
    assert loop.start()
    assert step_started.wait(1.0)
    errors: list[BaseException] = []

    def call() -> None:
        try:
            loop.execute(lambda: "never applied")
        except BaseException as exc:
            errors.append(exc)

    caller = Thread(target=call)
    caller.start()
    deadline = monotonic() + 1.0
    while monotonic() < deadline:
        with loop._condition:
            if loop._commands:
                break
        sleep(0.001)
    with loop._condition:
        assert len(loop._commands) == 1
    assert not loop.stop(timeout=0.0)
    caller.join(1.0)
    release_step.set()
    assert loop.close(timeout=1.0)

    assert len(errors) == 1
    assert isinstance(errors[0], RuntimeError)
    assert "stopped before command" in str(errors[0])

    error_loop = RuntimeLoop(lambda: None, interval_seconds=10.0)
    assert error_loop.start()
    try:
        with pytest.raises(ValueError, match="owner failure"):
            error_loop.execute(
                lambda: (_ for _ in ()).throw(ValueError("owner failure"))
            )
    finally:
        assert error_loop.close()
