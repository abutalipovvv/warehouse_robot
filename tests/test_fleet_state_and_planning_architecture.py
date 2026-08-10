from __future__ import annotations

from dataclasses import FrozenInstanceError
from threading import Event, Thread, current_thread
from time import monotonic, sleep

import pytest

from fleet_manager.manager.manager import FleetManagerCore
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.manager.planning import (
    PlanCommitService,
    PlanCommitStatus,
)
from fleet_manager.manager.tasks.order_admission import (
    AdmissionStatus,
    OrderAdmissionService,
)
from fleet_manager.manager.tasks.replanning import ReplanningService
from fleet_manager.manager.tasks.rolling_continuation import (
    RollingContinuationService,
)
from fleet_manager.manager.state import (
    FleetState,
    PlanningSnapshotFactory,
    PlanningState,
    RecoveryState,
    TrafficState,
)
from fleet_manager.manager.planning import (
    FrozenMapping,
    PlanCandidate,
    PlanningJob,
    PlanningJobRecord,
    PlanningJobStatus,
    PlanningPriority,
    PlanningReason,
    PlanningSnapshot,
)
from fleet_manager.manager.scheduler import PlanningWorker
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
    holds = manager.traffic_state.controlled_corridor_active_holds
    cycles = manager.recovery_state.active_wait_cycles
    replans["r1"] = {"stage": "queued"}
    leases["c1"] = ("r1", 10.0)
    holds["r1"] = {"lm": "A", "route_revision": 1}
    cycles[("r1", "r2")] = {"winner": "r1"}
    try:
        manager.reset_planning_runtime_state()
        assert manager.planning_state.runtime_replans is replans
        assert manager.traffic_state.controlled_corridor_leases is leases
        assert manager.traffic_state.controlled_corridor_active_holds is holds
        assert manager.recovery_state.active_wait_cycles is cycles
        assert replans == {}
        assert leases == {}
        assert holds == {}
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
        trajectory=[{"t": 0.0}, {"t": 5.0}],
    )
    unrelated = FleetRobot(
        name="r2",
        current_lm="B",
        pose={"x": 10.0, "y": 0.0, "yaw": 0.0},
        trajectory=[{"t": 0.0}, {"t": 50.0}],
    )
    fleet_state.robots[robot.name] = robot
    fleet_state.robots[unrelated.name] = unrelated
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
    assert [route.robot_id for route in snapshot.active_routes] == ["r1"]
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


def test_internal_order_planning_status_does_not_advance_revision() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    order = FleetOrder(
        order_id="pending-order",
        target_lm="B",
        status="QUEUED",
    )
    manager.orders[order.order_id] = order
    manager._synchronize_planning_revision()
    queued_revision = manager.planning_revision
    try:
        order.status = "PLANNING"
        manager._synchronize_planning_revision()
        assert manager.planning_revision == queued_revision

        order.target_lm = "A"
        manager._synchronize_planning_revision()
        assert manager.planning_revision == queued_revision + 1
    finally:
        manager.close()


def test_lease_heartbeat_does_not_stale_planning_revision() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    try:
        manager.traffic_state.controlled_corridor_leases["corridor-1"] = (
            "r1",
            10.0,
        )
        manager.traffic_state.traffic_zone_leases[("zone-1", "r1")] = 10.0
        manager._synchronize_planning_revision()
        owned_revision = manager.planning_revision
        live_job = PlanningJobRecord(kind="dispatch")
        planning_job = manager._build_planning_job(live_job, [], {})
        candidate = PlanCandidate.from_result(
            planning_job,
            {"ok": True, "plans": []},
            finished_at=monotonic(),
        )

        manager.traffic_state.controlled_corridor_leases["corridor-1"] = (
            "r1",
            11.0,
        )
        manager.traffic_state.traffic_zone_leases[("zone-1", "r1")] = 11.0
        manager._synchronize_planning_revision()
        assert manager.planning_revision == owned_revision
        assert manager._planning_candidate_is_current(live_job, candidate)

        manager.traffic_state.controlled_corridor_leases["corridor-1"] = (
            "r2",
            12.0,
        )
        manager.traffic_state.traffic_zone_leases.pop(("zone-1", "r1"))
        manager.traffic_state.traffic_zone_leases[("zone-1", "r2")] = 12.0
        manager._synchronize_planning_revision()
        assert manager.planning_revision == owned_revision + 1
        assert not manager._planning_candidate_is_current(live_job, candidate)
    finally:
        manager.close()


def test_rolling_candidate_ignores_unrelated_order_revision() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    robot = FleetRobot(
        name="rolling-robot",
        current_lm="A",
        target_lm="B",
        active_order_id="active-order",
        route_revision=7,
        status="MOVING",
        trajectory=[{"t": 0.0}, {"t": 20.0}],
    )
    manager.robots[robot.name] = robot
    manager.orders["active-order"] = FleetOrder(
        order_id="active-order",
        target_lm="B",
        assigned_robot=robot.name,
        status="EXECUTING",
    )
    manager._synchronize_planning_revision()
    record = PlanningJobRecord(kind="prefetch", route_revisions={robot.name: 7})
    planning_job = manager._build_planning_job(
        record,
        [{"name": robot.name, "startLm": "B", "goalLm": "A"}],
        {},
    )
    candidate = PlanCandidate.from_result(
        planning_job,
        {"ok": True, "plans": []},
        finished_at=monotonic(),
    )
    try:
        manager.orders["unrelated-order"] = FleetOrder(
            order_id="unrelated-order",
            target_lm="A",
        )
        manager._synchronize_planning_revision()

        assert candidate.expected_revision != manager.planning_revision
        assert manager._planning_candidate_is_current(record, candidate)
        assert manager.planning_state.diagnostic_counts[
            "planning_candidate_dependency_validated"
        ] >= 1
    finally:
        manager.close()


def test_rolling_dependency_stamp_rejects_participant_route_change() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    robot = FleetRobot(
        name="rolling-robot",
        current_lm="A",
        active_order_id="active-order",
        route_revision=4,
        status="MOVING",
        trajectory=[{"t": 0.0}, {"t": 20.0}],
    )
    manager.robots[robot.name] = robot
    manager._synchronize_planning_revision()
    record = PlanningJobRecord(kind="prefetch", route_revisions={robot.name: 4})
    planning_job = manager._build_planning_job(
        record,
        [{"name": robot.name, "startLm": "B", "goalLm": "A"}],
        {},
    )
    candidate = PlanCandidate.from_result(
        planning_job,
        {"ok": True, "plans": []},
        finished_at=monotonic(),
    )
    try:
        robot.route_revision = 5
        manager._synchronize_planning_revision()

        assert not manager._planning_candidate_is_current(record, candidate)
    finally:
        manager.close()


def test_dependency_aware_commit_still_checks_current_state_twice() -> None:
    revision = 3
    current_checks = 0
    applied: list[str] = []

    def is_current() -> bool:
        nonlocal current_checks
        current_checks += 1
        return current_checks == 1

    outcome = PlanCommitService(lambda: revision).commit(
        _candidate(2),
        validate=lambda: None,
        capture=lambda: (),
        apply=lambda: applied.append("route"),
        restore=lambda checkpoint: None,
        is_current=is_current,
    )

    assert outcome.status is PlanCommitStatus.STALE
    assert current_checks == 2
    assert applied == []


def test_planning_job_captures_preparation_changes_in_its_revision() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    try:
        initial_revision = manager.planning_revision
        manager.traffic_state.stationary_blockers["A"] = "r1"

        planning_job = manager._build_planning_job(
            PlanningJobRecord(kind="dispatch"),
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
    )
    planning_record = PlanningJobRecord(planning_job)
    planning_record.transition_to(PlanningJobStatus.RUNNING)
    planning_record.transition_to(PlanningJobStatus.COMPLETED)
    candidate = PlanCandidate.from_result(
        planning_job,
        {
            "ok": True,
            "plans": [{"robot": "r1", "nodes": ["A", "B"]}],
        },
        finished_at=2.0,
    )
    manager.planning_state.jobs[planning_job.job_id] = planning_record
    manager.planning_state.runtime_replans["r1"] = {"stage": "planning"}
    planning_record.kind = "runtime_replan"
    planning_record.robot_name = "r1"
    planning_record.entries = [(order, robot, {}, "B")]

    try:
        manager.fleet_state.revision.value = 11
        manager._reject_stale_planning_candidate(planning_record, candidate)

        assert robot.trajectory == route_before
        assert manager.traffic_state.temporal_reservations == reservations_before
        assert order.status == "QUEUED"
        assert manager.planning_state.runtime_replans["r1"]["stage"] == "queued"
        assert planning_record.status is PlanningJobStatus.STALE
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


def test_manager_commit_checkpoint_copies_only_job_participants() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    participant = FleetRobot(
        name="participant",
        current_lm="A",
        active_order_id="participant-order",
        trajectory=[{"t": 0.0}, {"t": 10.0}],
    )
    unrelated = FleetRobot(
        name="unrelated",
        current_lm="B",
        active_order_id="unrelated-order",
        trajectory=[{"t": 0.0}, {"t": 20.0}],
    )
    participant_order = FleetOrder(
        order_id="participant-order",
        target_lm="B",
        assigned_robot=participant.name,
    )
    unrelated_order = FleetOrder(
        order_id="unrelated-order",
        target_lm="A",
        assigned_robot=unrelated.name,
    )
    manager.robots.update({
        participant.name: participant,
        unrelated.name: unrelated,
    })
    manager.orders.update({
        participant_order.order_id: participant_order,
        unrelated_order.order_id: unrelated_order,
    })
    record = PlanningJobRecord(kind="dispatch")
    record.entries = [
        (participant_order, participant, {"startLm": "A"}, "B")
    ]
    intents = manager.traffic_state.controlled_corridor_prefetch_intents
    intents[participant.name] = {"slot": {"epoch": 1}}
    intents[unrelated.name] = {"slot": {"epoch": 2}}

    try:
        checkpoint = manager._capture_plan_commit_state(record)

        assert [robot.name for robot, _ in checkpoint.robot_state] == [
            participant.name
        ]
        assert [order.order_id for order, _ in checkpoint.order_state] == [
            participant_order.order_id
        ]
        assert [name for name, _, _ in checkpoint.corridor_intent_state] == [
            participant.name
        ]

        participant.route_clock = 8.0
        participant_order.status = "EXECUTING"
        unrelated.route_clock = 7.0
        unrelated_order.status = "COMPLETED"
        intents[participant.name]["slot"]["epoch"] = 10
        intents[unrelated.name]["slot"]["epoch"] = 20
        manager._restore_plan_commit_state(checkpoint)

        assert participant.route_clock == 0.0
        assert participant_order.status == "QUEUED"
        assert unrelated.route_clock == 7.0
        assert unrelated_order.status == "COMPLETED"
        assert intents[participant.name]["slot"]["epoch"] == 1
        assert intents[unrelated.name]["slot"]["epoch"] == 20
    finally:
        manager.close()


def test_worker_returns_candidate_and_runtime_owner_commits_it() -> None:
    revision = [10]
    routes: list[str] = []
    reservations: list[str] = []
    solver_threads: list[str] = []
    commit_threads: list[str] = []
    worker = PlanningWorker(name="planning-owner-test", max_queue_size=1)
    job = PlanningJob(
        job_id="owner-job",
        reason=PlanningReason.ORDER_DISPATCH,
        priority=PlanningPriority.ORDER_DISPATCH,
        snapshot=_empty_snapshot(10),
        submitted_at=monotonic(),
    )

    def solve(planning_job: PlanningJob) -> PlanCandidate:
        solver_threads.append(current_thread().name)
        return PlanCandidate.from_result(
            planning_job,
            {
                "ok": True,
                "plans": [{"robot": "r1", "route": "A-B"}],
                "reservations": [{"resource": "A-B"}],
            },
            finished_at=monotonic(),
        )

    assert worker.submit_job(job, solve)
    assert worker.join(timeout=1.0)
    candidates = worker.take_completed_results()
    assert len(candidates) == 1
    candidate = candidates[0]
    assert solver_threads == ["planning-owner-test"]
    assert routes == []
    assert reservations == []

    commit_service = PlanCommitService(lambda: revision[0])

    def apply() -> None:
        commit_threads.append(current_thread().name)
        routes.append("A-B")
        reservations.append("A-B")

    def restore(checkpoint: tuple[list[str], list[str]]) -> None:
        routes[:] = checkpoint[0]
        reservations[:] = checkpoint[1]

    runtime = RuntimeLoop(
        lambda: None,
        interval_seconds=10.0,
        name="planning-commit-owner",
    )
    assert runtime.start()
    try:
        outcome = runtime.execute(
            lambda: commit_service.commit(
                candidate,
                validate=lambda: None,
                capture=lambda: (list(routes), list(reservations)),
                apply=apply,
                restore=restore,
            )
        )
        assert outcome.status is PlanCommitStatus.COMMITTED
        assert commit_threads == ["planning-commit-owner"]
        assert routes == ["A-B"]
        assert reservations == ["A-B"]

        revision[0] = 11
        stale_outcome = runtime.execute(
            lambda: commit_service.commit(
                candidate,
                validate=lambda: None,
                capture=lambda: (list(routes), list(reservations)),
                apply=apply,
                restore=lambda checkpoint: None,
            )
        )
        assert stale_outcome.status is PlanCommitStatus.STALE
        assert commit_threads == ["planning-commit-owner"]
        assert routes == ["A-B"]
        assert reservations == ["A-B"]
    finally:
        assert runtime.close()
        assert worker.close()


def test_planning_job_state_machine_rejects_invalid_transition() -> None:
    job = PlanningJob(
        job_id="job-1",
        reason=PlanningReason.SAFETY_REPLAN,
        priority=PlanningPriority.SAFETY_REPLAN,
        snapshot=_empty_snapshot(1),
        submitted_at=1.0,
    )
    record = PlanningJobRecord(job)
    record.transition_to(PlanningJobStatus.RUNNING)
    record.transition_to(PlanningJobStatus.COMPLETED)
    record.transition_to(PlanningJobStatus.COMMITTED)

    with pytest.raises(ValueError, match="cannot transition"):
        record.transition_to(PlanningJobStatus.RUNNING)


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


def test_order_admission_service_selects_robot_without_manager() -> None:
    landmarks, _ = _graph()
    fleet_state = FleetState()
    near = FleetRobot(name="near", current_lm="A")
    busy = FleetRobot(
        name="busy",
        current_lm="B",
        active_order_id="active",
    )
    fleet_state.robots.update({near.name: near, busy.name: busy})
    service = OrderAdmissionService(
        fleet_state,
        landmarks,
        lambda: 1.0,
        robot_landmark=lambda robot: robot.current_lm,
    )
    order = FleetOrder(order_id="o1", target_lm="B")

    decision = service.admission_result(order)

    assert decision.status is AdmissionStatus.ACCEPTED
    assert decision.robot_id == near.name
    assert service.candidate_robots(order) == [near]
    near.status = "MOVING"
    deferred = service.admission_result(order)
    assert deferred.status is AdmissionStatus.DEFERRED
    assert deferred.reason == "no available robot"


def test_rolling_continuation_service_works_without_manager() -> None:
    state = PlanningState()
    state.rolling_prefetch_eligible_since["r1"] = 4.0
    state.rolling_prefetch_failures["r1"] = 2
    service = RollingContinuationService(
        FleetState(),
        state,
        lambda order: 0.5,
        lambda: 10.0,
    )
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

    service.mark_eligible(order, robot, 0.0)
    service.defer_prefetch(
        order,
        robot,
        boundary_waiting=True,
        boundary_retry_interval=0.5,
        time_scale=1.0,
    )
    assert robot.rolling_boundary_since == 4.0
    assert state.rolling_prefetch_retry_at[robot.name] == 10.5

    service.defer_prefetch(
        order,
        robot,
        boundary_waiting=False,
        boundary_retry_interval=0.5,
        time_scale=1.0,
        maximum_retry_delay=0.75,
    )
    assert state.rolling_prefetch_retry_at[robot.name] == 10.75


def test_rolling_service_builds_candidates_without_manager() -> None:
    fleet_state = FleetState()
    planning_state = PlanningState()
    order = FleetOrder(
        order_id="o1",
        target_lm="C",
        targets=["C"],
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        status="MOVING",
        active_order_id=order.order_id,
        route_chunk_goal_lm="B",
        route_clock=4.0,
        trajectory=[
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"t": 5.0, "x": 1.0, "y": 0.0, "yaw": 0.0},
        ],
    )
    fleet_state.robots[robot.name] = robot
    fleet_state.task_manager.orders[order.order_id] = order

    def attach_route(request, _order, start, goal, final) -> None:
        request["spatialRouteNodes"] = [start, goal, final]

    service = RollingContinuationService(
        fleet_state,
        planning_state,
        lambda _order: 0.5,
        lambda: 10.0,
        active_order_target=lambda current: current.target_lm,
        planning_goal=lambda _start, final, _order: final,
        pose_at_trajectory=lambda trajectory, _elapsed: {
            "x": trajectory[-1]["x"],
            "y": trajectory[-1]["y"],
            "yaw": trajectory[-1]["yaw"],
        },
        pose_at_landmark=lambda _landmark: None,
        attach_spatial_route=attach_route,
        valid_blockers=lambda _name: [],
        waits_at_boundary=lambda _robot: False,
        prefetch_lead=lambda: 2.0,
    )

    candidates = service.select_candidates()

    assert len(candidates) == 1
    _, selected_robot, request, final_goal, remaining = candidates[0]
    assert selected_robot is robot
    assert request == {
        "name": "r1",
        "startLm": "B",
        "goalLm": "C",
            "startPose": {"x": 1.0, "y": 0.0, "yaw": 0.0},
            "spatialRouteNodes": ["B", "C", "C"],
            "reservationStartTimeSec": 11.0,
        }
    assert final_goal == "C"
    assert remaining == 1.0
    assert planning_state.rolling_prefetch_eligible_since == {"r1": 10.0}


def test_replanning_service_works_without_full_manager() -> None:
    fleet_state = FleetState()
    planning_state = PlanningState()
    recovery_state = RecoveryState()
    order = FleetOrder(
        order_id="o1",
        target_lm="B",
        vehicle="r1",
        assigned_robot="r1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="r1",
        current_lm="A",
        active_order_id=order.order_id,
        route_revision=3,
        route_clock=1.5,
    )
    fleet_state.task_manager.orders[order.order_id] = order
    fleet_state.robots[robot.name] = robot
    service = ReplanningService(
        fleet_state,
        planning_state,
        recovery_state,
        lambda _robot: "A",
        lambda: 20.0,
    )

    state = service.install_transaction(
        robot,
        order,
        start_lm="A",
        now=20.0,
        reason="traffic changed",
        generation=1,
        blocker_names=("r2",),
        causal_blocker_signatures=(("r2", "B", 2),),
        wait_dependency_signature=("r2", "A->B"),
        retained_route_superseded=False,
        requires_spatial_replan=False,
    )

    assert planning_state.runtime_replans[robot.name] is state
    assert service.state_is_current(robot, state)
    assert order.status == "PLANNING"
    assert robot.status == "WAITING"

    recovery_state.coupled_replan_failures[("r1", "r2")] = 2
    recovery_state.coupled_replan_last_attempt[("r1", "r2")] = 19.0
    assert service.coupled_failure_count({"r1"}) == 2
    assert service.coupled_latest_attempt({"r1"}) == 19.0
    service.clear_coupled_attempts({"r1", "r2"})
    assert recovery_state.coupled_replan_failures == {}
    assert recovery_state.coupled_replan_last_attempt == {}

    robot.route_revision += 1
    assert not service.state_is_current(robot, state)


def test_replanning_service_records_retry_without_committing() -> None:
    fleet_state = FleetState()
    planning_state = PlanningState()
    recovery_state = RecoveryState()
    order = FleetOrder(order_id="o1", target_lm="B", status="PLANNING")
    robot = FleetRobot(name="r1", current_lm="A", status="WAITING")
    service = ReplanningService(
        fleet_state,
        planning_state,
        recovery_state,
        lambda _robot: "A",
        lambda: 30.0,
        lambda _order: 2.0,
    )
    state = {"reason": "traffic changed", "failures": 0}

    failure = service.record_failure(
        order,
        robot,
        state,
        "reserved edge",
        debug={"backend": "rolling_sipp"},
        dynamic_conflict_signature=("r2", "A"),
        reservation_conflict_signature=("r3", "A->B"),
    )

    assert failure.failures == 1
    assert failure.now == 30.0
    assert state["stage"] == "retry"
    assert state["retry_at"] == 32.0
    assert state["dynamic_conflict_count"] == 1
    assert state["reservation_conflict_count"] == 1
    assert order.dispatch_failures == 1
    assert robot.trajectory == []


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
