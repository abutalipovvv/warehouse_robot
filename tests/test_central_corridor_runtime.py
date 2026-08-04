from __future__ import annotations

from dataclasses import replace
from threading import Event

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.manager.planning import PlanningSolverService
from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorDecisionStatus,
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSlot,
    CorridorSlotState,
)
from fleet_manager.runtime.simulation.manager import FleetManagerSim


REGION = "corridor:runtime"


def _install_planning_solver(manager: FleetManagerSim, planner) -> None:
    def planner_call(payload):
        return planner(payload.get("robots", []), payload)

    manager._planning_solver_service = PlanningSolverService(
        planner_call,
        manager._planner_lock,
    )


def _manager(*, controlled: bool = True) -> FleetManagerSim:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    properties: dict[str, object] = {"direction": 2}
    if controlled:
        properties["controlled_region"] = REGION
    edges = [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[source].to_point(),
                landmarks[target].to_point(),
            ),
            properties=dict(properties),
        )
        for source, target in (("A", "B"), ("B", "A"))
    ]
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "controlled_corridor_schedule_horizon_sec": 30.0,
                "controlled_corridor_commit_horizon_sec": 2.0,
                "traffic_zone_control_enabled": False,
            }
        },
    )


def _robot(name: str) -> FleetRobot:
    return FleetRobot(
        name=name,
        current_lm="A",
        target_lm="B",
        status="MOVING",
        active_order_id=f"order-{name}",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 20.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
    )


def _entry(entry_clock: float, duration: float) -> dict[str, object]:
    return {
        "region": REGION,
        "regions": (REGION,),
        "passage": REGION,
        "src": "A",
        "dst": "B",
        "holding_lm": "A",
        "staging_clock": entry_clock,
        "entry_clock": entry_clock,
        "exit_clock": entry_clock + duration,
        "exit_lm": "B",
        "direction": "A->B",
        "eta": entry_clock,
        "at_staging": entry_clock == 0.0,
        "passed_staging": False,
    }


def test_runtime_calendar_backfills_near_robots_before_far_slot(
    monkeypatch,
) -> None:
    manager = _manager()
    manager.robots = {
        name: _robot(name)
        for name in ("far", "near-1", "near-2")
    }
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
        for robot in manager.robots.values()
    }
    entries = {
        "far": _entry(12.0, 3.0),
        "near-1": _entry(0.0, 3.0),
        "near-2": _entry(0.0, 3.0),
    }
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda robot: dict(entries[robot.name]),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )
    now = 1_000.0
    manager._controlled_corridor_wait_since[
        (REGION, "flow:east", 0, "far")
    ] = now - 20.0

    manager._prepare_controlled_corridor_admissions(now)

    schedule = manager._controlled_corridor_schedule
    assert schedule is not None
    slots = {slot.robot_id: slot for slot in schedule.slots}
    assert slots["near-1"].entry_time == now
    assert slots["near-2"].entry_time == now + 1.0
    assert slots["near-2"].exit_time < slots["far"].entry_time
    assert slots["far"].entry_time == now + 12.0
    assert manager._controlled_corridor_winners == {
        "near-1": REGION,
    }
    assert not manager._controlled_corridor_has_grant(
        "far",
        (REGION,),
    )
    assert manager._controlled_corridor_queues[REGION] == [
        "near-2",
        "far",
    ]


def test_open_space_does_not_create_central_corridor_scheduler() -> None:
    manager = _manager(controlled=False)

    manager._prepare_controlled_corridor_admissions(1_000.0)

    assert manager._controlled_corridor_scheduler is None
    assert manager._controlled_corridor_schedule is None
    assert manager._controlled_corridor_passages == {}
    assert manager._controlled_corridor_leases == {}


def test_runtime_parses_adaptive_corridor_phase_policy() -> None:
    manager = _manager()
    manager.params["fleet"].update(
        {
            "controlled_corridor_max_direction_batch": 5,
            # The adaptive ceiling can never invalidate the guaranteed base
            # phase, even when an operator enters a smaller value.
            "controlled_corridor_max_adaptive_direction_batch": 2,
            "controlled_corridor_phase_amortization_sec": 3.5,
            "controlled_corridor_max_phase_extension_sec": 7.0,
        }
    )

    config = manager._controlled_corridor_scheduler_config()

    assert config.max_direction_batch == 5
    assert config.max_adaptive_direction_batch == 5
    assert config.phase_amortization_sec == 3.5
    assert config.max_phase_extension_sec == 7.0


def _rolling_boundary_robot(
    name: str,
    *,
    start_lm: str,
    goal_lm: str,
) -> tuple[FleetOrder, FleetRobot, dict[str, object]]:
    order = FleetOrder(
        order_id=f"order-{name}",
        target_lm=goal_lm,
        vehicle=name,
        assigned_robot=name,
        status="PLANNING",
    )
    pose = {
        "x": 0.0 if start_lm == "A" else 1.0,
        "y": 0.0,
        "yaw": 0.0 if start_lm == "A" else 3.141592653589793,
    }
    robot = FleetRobot(
        name=name,
        current_lm=start_lm,
        target_lm=start_lm,
        status="WAITING",
        active_order_id=order.order_id,
        pose=dict(pose),
        trajectory=[
            {
                "t": 0.0,
                **pose,
                "edgeId": f"{start_lm}->{start_lm}",
                "lm": start_lm,
            },
            {
                "t": 5.0,
                **pose,
                "edgeId": f"{start_lm}->{start_lm}",
                "lm": start_lm,
            },
        ],
        route_clock=0.0,
        route_revision=7,
        route_chunk_goal_lm=start_lm,
        route_final_lm=goal_lm,
        has_executed_route=True,
        last_reason="rolling continuation pending",
    )
    request: dict[str, object] = {
        "name": name,
        "startLm": start_lm,
        "goalLm": goal_lm,
        "startPose": dict(pose),
        "routeNodes": [start_lm, goal_lm],
    }
    return order, robot, request


def test_open_space_prefetch_gate_is_completely_inert() -> None:
    manager = _manager(controlled=False)
    order, robot, request = _rolling_boundary_robot(
        "open",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot

    gate = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=1_000.0,
    )

    assert gate is None
    assert manager._controlled_corridor_prefetch_intents == {}


def test_prefetch_intent_receives_a_global_slot_on_next_runtime_tick() -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "rolling",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    now = 1_000.0

    first = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    assert first is not None
    assert first["ready"] is False

    manager._prepare_controlled_corridor_admissions(now)
    second = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )

    assert second is not None
    assert second["ready"] is True
    assert second["departureNotBefore"]["node"] == "A"
    assert second["departureNotBefore"]["timeSec"] >= 0.0


def test_identical_corridor_intent_tracks_parent_spatial_route_revision() -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "same-prefix",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    now = 1_000.0
    intent = manager._controlled_corridor_prefetch_intent(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    assert intent is not None
    order.spatial_route_revision += 1

    refreshed = manager._controlled_corridor_prefetch_intent(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now + 0.1,
    )

    assert refreshed is intent
    assert refreshed["spatial_route_revision"] == (
        order.spatial_route_revision
    )
    assert manager._controlled_corridor_intent_is_current(
        robot,
        order,
        refreshed,
    )


def test_late_corridor_slot_commits_only_the_approach_to_stop_line() -> None:
    manager = _manager()
    manager.landmarks["X"] = Landmark(name="X", x=-2.0, y=0.0)
    request: dict[str, object] = {
        "name": "approach",
        "startLm": "X",
        "goalLm": "B",
        "routeNodes": ["X", "A", "B"],
    }
    corridor_request = CorridorRequest(
        robot_id="approach",
        regions=(REGION,),
        direction="A->B",
        earliest_entry=1_000.0,
        duration_sec=5.0,
        staging_lm="A",
        exit_lm="B",
        route_revision=1,
    )
    intent = {
        "request": corridor_request,
        "entry": {
            "src": "A",
            "holding_lm": "A",
            "exit_lm": "B",
        },
    }

    assert manager._prepare_corridor_approach_request(request, intent)
    assert request["goalLm"] == "A"
    assert request["routeNodes"] == ["X", "A"]


def test_missed_corridor_slot_still_releases_safe_approach_prefix(
    monkeypatch,
) -> None:
    landmarks = {
        "X": Landmark(name="X", x=-3.0, y=0.0),
        "S": Landmark(name="S", x=-2.0, y=0.0),
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=abs(landmarks[target].x - landmarks[source].x),
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[source].to_point(),
                landmarks[target].to_point(),
            ),
            properties={
                "direction": 2,
                **(
                    {"controlled_region": REGION}
                    if {source, target} == {"A", "B"}
                    else {}
                ),
            },
        )
        for source, target in (
            ("X", "S"),
            ("S", "X"),
            ("S", "A"),
            ("A", "S"),
            ("A", "B"),
            ("B", "A"),
        )
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "controlled_corridor_schedule_horizon_sec": 30.0,
                "controlled_corridor_commit_horizon_sec": 2.0,
                "traffic_zone_control_enabled": False,
            }
        },
    )
    order = FleetOrder(
        order_id="order-approach",
        target_lm="B",
        vehicle="approach",
        assigned_robot="approach",
        status="PLANNING",
        spatial_route_nodes=["X", "S", "A", "B"],
    )
    robot = FleetRobot(
        name="approach",
        current_lm="X",
        target_lm="X",
        status="WAITING",
        active_order_id=order.order_id,
        pose={"x": -3.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": -3.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "X->X",
                "lm": "X",
            },
            {
                "t": 1.0,
                "x": -3.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "X->X",
                "lm": "X",
            },
        ],
        route_clock=1.0,
        route_revision=1,
        route_chunk_goal_lm="X",
        route_final_lm="B",
        has_executed_route=True,
        last_reason="rolling continuation pending",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    monkeypatch.setattr(manager, "_rolling_horizon", lambda: 0.1)
    request: dict[str, object] = {
        "name": robot.name,
        "startLm": "X",
        "goalLm": "B",
        "routeNodes": ["X", "S", "A", "B"],
        "startPose": dict(robot.pose),
    }
    now = 1_000.0

    first = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=0.0,
        now=now,
    )
    assert first is not None
    assert first["ready"] is False
    manager._prepare_controlled_corridor_admissions(now)
    scheduled_intent = manager._controlled_corridor_prefetch_intents[
        robot.name
    ]
    assert manager._controlled_corridor_schedule is not None
    assert scheduled_intent["last_schedule_epoch"] == (
        manager._controlled_corridor_schedule.epoch
    )

    missed = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=0.0,
        now=now + 20.0,
    )

    assert missed is not None
    assert missed["ready"] is True
    assert missed["approachOnly"] is True
    assert request["goalLm"] == "S"
    assert request["routeNodes"] == ["X", "S"]


def test_corridor_approach_chunks_queue_on_distinct_safe_lms() -> None:
    names = ("X3", "X2", "X1", "A", "B")
    landmarks = {
        name: Landmark(name=name, x=float(index), y=0.0)
        for index, name in enumerate(names)
    }
    edges: list[GraphEdge] = []
    for source, target in zip(names, names[1:]):
        properties: dict[str, object] = {"direction": 2}
        if (source, target) == ("A", "B"):
            properties["controlled_region"] = REGION
        for src, dst in ((source, target), (target, source)):
            edges.append(
                GraphEdge(
                    from_name=src,
                    to_name=dst,
                    length=1.0,
                    kind="line",
                    edge_type="FeatureLine",
                    world_points=(
                        landmarks[src].to_point(),
                        landmarks[dst].to_point(),
                    ),
                    properties=dict(properties),
                )
            )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
            }
        },
    )
    starts = {
        "leader": "X1",
        "follower": "X2",
        "tail": "X3",
    }
    expected_holds = {
        "leader": "A",
        "follower": "X1",
        "tail": "X2",
    }
    for robot_name, start_lm in starts.items():
        robot = FleetRobot(
            name=robot_name,
            current_lm=start_lm,
            status="WAITING",
            active_order_id=f"order-{robot_name}",
            pose={
                "x": landmarks[start_lm].x,
                "y": 0.0,
                "yaw": 0.0,
            },
            route_revision=1,
        )
        manager.robots[robot_name] = robot
        route_nodes = list(names[names.index(start_lm):])
        request: dict[str, object] = {
            "name": robot_name,
            "startLm": start_lm,
            "goalLm": "B",
            "routeNodes": route_nodes,
        }
        corridor_request = CorridorRequest(
            robot_id=robot_name,
            regions=(REGION,),
            direction="A->B",
            earliest_entry=1_000.0,
            duration_sec=5.0,
            staging_lm="A",
            exit_lm="B",
            route_revision=1,
        )
        intent = {
            "signature": (robot_name,),
            "order_id": robot.active_order_id,
            "request": corridor_request,
            "entry": {
                "src": "A",
                "holding_lm": "A",
                "exit_lm": "B",
            },
        }
        manager._controlled_corridor_prefetch_intents[robot_name] = intent

        assert manager._prepare_corridor_approach_request(
            request,
            intent,
            robot=robot,
        )
        assert request["goalLm"] == expected_holds[robot_name]
        assert request["routeNodes"][-1] == expected_holds[robot_name]

    assert {
        assignment["lm"]
        for assignment in manager._controlled_corridor_approach_holds.values()
    } == {"A", "X1", "X2"}


def test_late_corridor_slot_at_stop_line_is_an_atomic_command(
    monkeypatch,
) -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "stop-line",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    now = 1_000.0
    monkeypatch.setattr(manager, "_rolling_horizon", lambda: 0.1)

    first = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=0.0,
        now=now,
    )
    assert first is not None
    assert first["ready"] is False
    manager._prepare_controlled_corridor_admissions(now)

    second = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=0.0,
        now=now,
    )
    assert second is not None
    assert second["ready"] is True
    assert not second.get("approachOnly")
    assert request["goalLm"] == "B"


def test_rolling_result_keeps_gated_corridor_through_safe_exit(
    monkeypatch,
) -> None:
    manager = _manager()
    monkeypatch.setattr(manager, "_rolling_horizon", lambda: 0.1)
    corridor_request = CorridorRequest(
        robot_id="atomic",
        regions=(REGION,),
        direction="A->C",
        earliest_entry=1_000.0,
        duration_sec=8.0,
        staging_lm="A",
        exit_lm="C",
        route_revision=1,
    )
    result = {
        "ok": True,
        "timeStepSec": 0.2,
        "plans": [
            {
                "robot": "atomic",
                "startLm": "A",
                "goalLm": "C",
                "nodes": ["A", "B", "C"],
                "times": [0, 20, 40],
                "trajectory": [],
            }
        ],
    }

    trimmed = manager._rolling_result(
        result,
        {"atomic": "C"},
        corridor_gates={
            "atomic": {
                "intent": {
                    "request": corridor_request,
                    "entry": {"src": "A", "dst": "B"},
                }
            }
        },
    )

    plan = trimmed["plans"][0]
    assert plan["nodes"] == ["A", "B", "C"]
    assert plan["goalLm"] == "C"
    assert not plan["rollingChunk"]


def test_commit_scan_sees_corridor_after_long_scheduled_stop_line_wait() -> None:
    manager = _manager()
    robot = FleetRobot(
        name="delayed",
        current_lm="A",
        status="WAITING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
            {
                "t": 40.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->A",
                "lm": "A",
            },
            {
                "t": 43.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
    )

    assert manager._next_controlled_corridor_entry(robot) is None
    complete = manager._next_controlled_corridor_entry(
        robot,
        lookahead_sec=float("inf"),
    )
    assert complete is not None
    assert complete["holding_lm"] == "A"
    assert complete["exit_lm"] == "B"


def test_opposing_prefetch_intents_are_serialized_before_mapf(
    monkeypatch,
) -> None:
    manager = _manager()
    # This test exercises calendar phase ordering. Physical downstream
    # occupancy is covered separately; otherwise the two synthetic robots
    # placed directly on each other's exits correctly make both requests
    # unavailable.
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )
    first_order, first_robot, first_request = _rolling_boundary_robot(
        "east",
        start_lm="A",
        goal_lm="B",
    )
    second_order, second_robot, second_request = _rolling_boundary_robot(
        "west",
        start_lm="B",
        goal_lm="A",
    )
    manager.orders = {
        first_order.order_id: first_order,
        second_order.order_id: second_order,
    }
    manager.robots = {
        first_robot.name: first_robot,
        second_robot.name: second_robot,
    }
    now = 1_000.0
    for order, robot, request in (
        (first_order, first_robot, first_request),
        (second_order, second_robot, second_request),
    ):
        gate = manager._controlled_corridor_prefetch_gate(
            order,
            robot,
            request,
            prediction_offset=5.0,
            now=now,
        )
        assert gate is not None
        assert gate["ready"] is False

    manager._prepare_controlled_corridor_admissions(now)
    schedule = manager._controlled_corridor_schedule
    assert schedule is not None
    first_slot = schedule.slot_for(first_robot.name)
    second_slot = schedule.slot_for(second_robot.name)
    assert first_slot is not None
    assert second_slot is not None
    assert (
        first_slot.exit_time <= second_slot.entry_time
        or second_slot.exit_time <= first_slot.entry_time
    )


def test_live_prefetch_passes_central_slot_to_sipp(monkeypatch) -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "rolling",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    entry = (order, robot, request, "B", 5.0)

    manager._start_async_rolling_prefetch(entry)
    assert manager._dispatch_job is None
    manager._prepare_controlled_corridor_admissions(manager._now())

    captured: list[dict[str, object]] = []
    planned = Event()

    def capture(
        requests: list[dict[str, object]],
        _payload: dict[str, object],
    ) -> dict[str, object]:
        captured.extend(dict(item) for item in requests)
        planned.set()
        return {
            "ok": False,
            "plans": [],
            "debug": {"reason": "captured central gate"},
        }

    _install_planning_solver(manager, capture)
    manager._start_async_rolling_prefetch(entry)

    assert planned.wait(1.0)
    assert len(captured) == 1
    assert captured[0]["departureNotBefore"][0]["node"] == "A"
    assert captured[0]["authorizedControlledRegions"] == [REGION]
    job = manager._dispatch_job
    assert isinstance(job, dict)
    assert robot.name in job["corridor_gates"]


def test_initial_dispatch_uses_the_same_central_corridor_gate(
    monkeypatch,
) -> None:
    manager = _manager()
    order = FleetOrder(
        order_id="order-initial",
        target_lm="B",
        vehicle="initial",
        assigned_robot="initial",
        status="QUEUED",
    )
    robot = FleetRobot(
        name="initial",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    entry = (
        order,
        robot,
        {
            "name": robot.name,
            "startLm": "A",
            "goalLm": "B",
            "startPose": dict(robot.pose),
        },
        "B",
    )

    assert not manager._start_async_simulated_dispatch([entry])
    assert manager._dispatch_job is None
    assert order.status == "QUEUED"
    intent = manager._controlled_corridor_prefetch_intents[robot.name]
    assert intent["kind"] == "dispatch"

    manager._prepare_controlled_corridor_admissions(manager._now())
    captured: list[dict[str, object]] = []
    planned = Event()

    def capture(
        requests: list[dict[str, object]],
        _payload: dict[str, object],
    ) -> dict[str, object]:
        captured.extend(dict(item) for item in requests)
        planned.set()
        return {
            "ok": False,
            "plans": [],
            "debug": {"reason": "captured initial central gate"},
        }

    _install_planning_solver(manager, capture)
    assert manager._start_async_simulated_dispatch([entry])
    assert planned.wait(1.0)
    assert captured[0]["departureNotBefore"][0]["node"] == "A"
    assert captured[0]["authorizedControlledRegions"] == [REGION]
    assert robot.name in manager._dispatch_job["corridor_gates"]


def test_dispatch_continuous_failure_is_attributed_only_to_requester() -> None:
    manager = _manager(controlled=False)
    first = FleetRobot(
        name="first",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    second = FleetRobot(
        name="second",
        current_lm="B",
        status="IDLE",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="moving",
        current_lm="B",
        target_lm="A",
        status="MOVING",
        active_order_id="moving-order",
        pose={"x": 0.8, "y": 0.0, "yaw": 3.141592653589793},
        trajectory=[
            {"t": 0.0, "x": 1.0, "y": 0.0, "lm": "B"},
            {"t": 2.0, "x": 0.0, "y": 0.0, "lm": "A"},
        ],
    )
    first_order = FleetOrder(
        order_id="first-order",
        target_lm="B",
        vehicle=first.name,
        assigned_robot=first.name,
        status="PLANNING",
    )
    second_order = FleetOrder(
        order_id="second-order",
        target_lm="A",
        vehicle=second.name,
        assigned_robot=second.name,
        status="PLANNING",
    )
    manager.robots = {
        first.name: first,
        second.name: second,
        blocker.name: blocker,
    }
    manager.orders = {
        first_order.order_id: first_order,
        second_order.order_id: second_order,
    }
    entries = [
        (
            first_order,
            first,
            {"name": first.name, "startLm": "A", "goalLm": "B"},
            "B",
        ),
        (
            second_order,
            second,
            {"name": second.name, "startLm": "B", "goalLm": "A"},
            "A",
        ),
    ]

    manager._finish_simulated_order_batch(
        entries,
        {
            "ok": False,
            "plans": [],
            "debug": {
                "reason": "continuous_conflict_unresolved",
                "continuousUnresolved": 1,
                "continuousUnresolvedConflicts": [
                    {
                        "source": "committed",
                        "robot": first.name,
                        "other": blocker.name,
                        "edge": "A->B",
                        "time": 1.0,
                    }
                ],
            },
        },
    )

    assert first_order.dispatch_failures == 1
    assert first_order.error
    assert second_order.status == "QUEUED"
    assert second_order.dispatch_failures == 0
    assert second_order.error == ""
    assert not manager._dispatch_conflict_dependency_ready(first_order)
    blocker.route_revision += 1
    assert manager._dispatch_conflict_dependency_ready(first_order)


def test_validated_prefetch_slot_is_committed_before_revision_handoff() -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "rolling",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    now = 1_000.0
    first = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    assert first is not None
    manager._prepare_controlled_corridor_admissions(now)
    gate = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    assert gate is not None
    assert gate["ready"] is True
    intent = gate["intent"]
    snapshot = {
        "intent": intent,
        "signature": intent["signature"],
        "slot": gate["slot"],
    }

    assert manager._commit_controlled_corridor_prefetch_slot(
        robot,
        snapshot,
    )
    slot = manager._controlled_corridor_schedule.slot_for(robot.name)
    assert slot is not None
    assert slot.state is CorridorSlotState.COMMITTED


def test_prefetch_commit_rejects_a_slot_moved_after_worker_snapshot() -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "stale-worker",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    now = 1_000.0
    assert manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    manager._prepare_controlled_corridor_admissions(now)
    gate = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    assert gate is not None
    assert gate["ready"] is True
    captured_slot = gate["slot"]
    intent = gate["intent"]
    corridor_request = intent["request"]

    scheduler = manager._controlled_corridor_scheduler
    assert scheduler is not None
    scheduler.reset()
    moved = scheduler.update(
        [
            replace(
                corridor_request,
                earliest_entry=captured_slot.entry_time + 3.0,
            )
        ],
        now=now,
        occupancies=[],
    )
    manager._controlled_corridor_schedule = moved
    assert moved.slot_for(robot.name) != captured_slot

    snapshot = {
        "intent": intent,
        "signature": intent["signature"],
        "slot": captured_slot,
    }
    assert not manager._commit_controlled_corridor_prefetch_slot(
        robot,
        snapshot,
    )
    assert moved.slot_for(robot.name).state is CorridorSlotState.TENTATIVE


def test_runtime_gate_pin_holds_calendar_until_worker_release() -> None:
    manager = _manager()
    order, robot, request = _rolling_boundary_robot(
        "pinned-worker",
        start_lm="A",
        goal_lm="B",
    )
    manager.orders[order.order_id] = order
    manager.robots[robot.name] = robot
    now = 1_000.0
    assert manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    manager._prepare_controlled_corridor_admissions(now)
    gate = manager._controlled_corridor_prefetch_gate(
        order,
        robot,
        request,
        prediction_offset=5.0,
        now=now,
    )
    assert gate is not None
    assert gate["ready"] is True
    captured = gate["slot"]
    gates = {
        robot.name: {
            "intent": gate["intent"],
            "signature": gate["intent"]["signature"],
            "slot": captured,
        }
    }
    assert manager._pin_controlled_corridor_gates(gates)

    intent = gate["intent"]
    intent["request"] = replace(
        intent["request"],
        earliest_entry=captured.entry_time + 4.0,
    )
    manager._prepare_controlled_corridor_admissions(now + 0.1)
    assert (
        manager._controlled_corridor_schedule.slot_for(robot.name)
        == captured
    )

    manager._release_controlled_corridor_gate_pins(gates)
    manager._prepare_controlled_corridor_admissions(now + 0.1)
    assert (
        manager._controlled_corridor_schedule.slot_for(robot.name)
        != captured
    )


def test_temporary_slot_displacement_preserves_intent_queue_age() -> None:
    manager = _manager()
    request = CorridorRequest(
        robot_id="queued",
        regions=(REGION,),
        direction="flow:east",
        earliest_entry=910.0,
        duration_sec=3.0,
        staging_lm="A",
        exit_lm="B",
        route_revision=1,
        requires_explicit_commit=True,
    )
    intent = {
        "registered_at": 900.0,
        "last_schedule_epoch": 7,
        "request": request,
    }
    manager._controlled_corridor_prefetch_intents["queued"] = intent
    actual_slot = CorridorSlot(
        robot_id="queued",
        regions=(REGION,),
        direction="flow:east",
        entry_time=916.0,
        exit_time=920.0,
        staging_lm="A",
        exit_lm="B",
        route_revision=1,
        state=CorridorSlotState.COMMITTED,
    )
    gate = {
        "intent": intent,
        "actual_slot": actual_slot,
    }

    manager._handle_controlled_corridor_gate_rejection(
        "queued",
        gate,
        "corridor slot changed before command commit",
    )

    assert manager._controlled_corridor_prefetch_intents["queued"] is intent
    assert intent["registered_at"] == 900.0
    assert intent["last_schedule_epoch"] is None
    assert intent["request"].earliest_entry == 916.0
    assert intent["request"].duration_sec == 4.0
    assert (
        intent["request"].resource_windows
        == actual_slot.resource_windows
    )

    manager._handle_controlled_corridor_gate_rejection(
        "queued",
        gate,
        "MAPF result changed the scheduled corridor passage",
    )
    assert "queued" not in manager._controlled_corridor_prefetch_intents


def test_corridor_flow_direction_deduplicates_interpolated_samples() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
        "C": Landmark(name="C", x=1.0, y=1.0),
    }
    edges = [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[source].to_point(),
                landmarks[target].to_point(),
            ),
            properties={
                "direction": 2,
                "controlled_region": REGION,
            },
        )
        for source, target in (
            ("A", "B"),
            ("B", "A"),
            ("B", "C"),
            ("C", "B"),
        )
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "traffic_zone_control_enabled": False,
            }
        },
    )
    robot = FleetRobot(
        name="turning",
        current_lm="A",
        target_lm="C",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 0.25,
                "x": 0.25,
                "y": 0.0,
                "edgeId": "A->B",
                "lm": "",
            },
            {
                "t": 0.75,
                "x": 0.75,
                "y": 0.0,
                "edgeId": "A->B",
                "lm": "",
            },
            {
                "t": 1.0,
                "x": 1.0,
                "y": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
            {
                "t": 1.25,
                "x": 1.0,
                "y": 0.25,
                "edgeId": "B->C",
                "lm": "",
            },
            {
                "t": 1.75,
                "x": 1.0,
                "y": 0.75,
                "edgeId": "B->C",
                "lm": "",
            },
            {
                "t": 2.0,
                "x": 1.0,
                "y": 1.0,
                "edgeId": "B->C",
                "lm": "C",
            },
        ],
    )

    entry = manager._next_controlled_corridor_entry(robot)

    assert entry is not None
    windows = {
        window.region_id: window
        for window in entry["resource_windows"]
    }
    assert windows[REGION].direction == "flow:path:east>south"
    assert len(windows[REGION].direction) < 32


def test_staging_release_slot_does_not_slide_forward_while_robot_waits(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("waiting")
    manager.robots = {robot.name: robot}
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    }
    entry = _entry(3.0, 3.0)
    entry["staging_clock"] = 0.0
    entry["entry_clock"] = 3.0
    entry["exit_clock"] = 6.0
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: dict(entry),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)
    first = manager._controlled_corridor_schedule.slot_for(robot.name)
    manager._prepare_controlled_corridor_admissions(1_000.1)
    second = manager._controlled_corridor_schedule.slot_for(robot.name)

    assert first is not None
    assert second is not None
    assert first.entry_time == 1_000.0
    assert first.exit_time == 1_006.0
    assert second.entry_time == first.entry_time
    assert second.exit_time == first.exit_time
    assert manager._controlled_corridor_has_grant(
        robot.name,
        (REGION,),
    )


def test_red_light_allows_in_place_turn_before_staging_clock(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("turn-before-entry")
    robot.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    robot.route_clock = 0.0
    manager.robots = {robot.name: robot}
    entry = _entry(5.0, 3.0)
    entry["holding_lm"] = "A"
    entry["staging_clock"] = 5.0
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: dict(entry),
    )

    # The pose remains on A throughout an in-place turn. Geometry alone must
    # not paint that preparation red before its route-clock stop line.
    assert manager._controlled_corridor_admission_reason(robot, 0.1) == ""

    robot.route_clock = 4.95
    reason = manager._controlled_corridor_admission_reason(robot, 5.0)
    assert reason.startswith("corridor admission wait at A")


def test_passing_backed_off_staging_does_not_roll_atomic_bundle_forever(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("approach")
    robot.route_clock = 1.0
    manager.robots = {robot.name: robot}
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    }
    entry = _entry(3.0, 3.0)
    entry["staging_clock"] = 0.0
    entry["entry_clock"] = 3.0
    entry["exit_clock"] = 6.0
    entry["passed_staging"] = True
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: dict(entry),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)
    first = manager._controlled_corridor_schedule.slot_for(robot.name)
    manager._prepare_controlled_corridor_admissions(1_007.0)
    replacement = manager._controlled_corridor_schedule.slot_for(robot.name)

    assert first is not None
    assert first.past_commit_point
    assert replacement is not None
    assert replacement.entry_time >= 1_007.0
    assert replacement.duration_sec <= 6.000001
    assert max(
        window.exit_offset_sec
        for window in replacement.resource_windows
    ) <= 6.000001


def test_wait_after_backed_off_staging_is_not_irrevocably_committed(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("delayed-approach")
    robot.route_clock = 1.0
    manager.robots = {robot.name: robot}
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    }
    entry = _entry(3.0, 3.0)
    entry["staging_clock"] = 0.0
    entry["passed_staging"] = True
    entry["has_wait_after_staging"] = True
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: dict(entry),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)

    slot = manager._controlled_corridor_schedule.slot_for(robot.name)
    assert slot is not None
    assert not slot.past_commit_point


def test_physical_corridor_windows_remain_bounded_across_waiting_ticks(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("physical-owner")
    manager.robots = {robot.name: robot}
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    }
    entry = _entry(0.0, 20.0)
    entry["resource_windows"] = (
        CorridorResourceWindow(
            region_id=REGION,
            entry_offset_sec=0.0,
            exit_offset_sec=20.0,
            direction="flow:east",
        ),
    )
    physical = False
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: None if physical else dict(entry),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda _robot: {REGION} if physical else set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: {REGION} if physical else set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)
    physical = True
    robot.route_clock = 2.0

    for tick in range(1, 101):
        manager._prepare_controlled_corridor_admissions(
            1_000.0 + tick * 0.1
        )
        slot = manager._controlled_corridor_schedule.slot_for(robot.name)
        assert slot is not None
        assert slot.physically_observed
        assert slot.duration_sec <= 18.100001
        assert max(
            window.exit_offset_sec
            for window in slot.resource_windows
        ) <= 18.100001


def test_active_exit_body_blocks_unless_trajectory_proves_it_will_clear() -> None:
    manager = _manager()
    entrant = _robot("entrant")
    blocker = _robot("blocker")
    blocker.current_lm = "B"
    blocker.pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    blocker.status = "WAITING"
    blocker.last_reason = "occupied by another"
    manager.robots = {
        entrant.name: entrant,
        blocker.name: blocker,
    }

    assert (
        manager._controlled_corridor_downstream_blocker(
            entrant,
            "B",
            20.0,
        )
        == blocker.name
    )

    blocker.status = "MOVING"
    blocker.last_reason = "moving"
    blocker.route_clock = 0.0
    blocker.trajectory = [
        {
            "t": 0.0,
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "B->A",
            "lm": "B",
        },
        {
            "t": 2.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "B->A",
            "lm": "A",
        },
    ]

    assert (
        manager._controlled_corridor_downstream_blocker(
            entrant,
            "B",
            20.0,
        )
        == ""
    )


def test_future_terminal_body_blocks_corridor_box_before_it_arrives() -> None:
    manager = _manager()
    entrant = _robot("entrant")
    blocker = _robot("blocker")
    blocker.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    blocker.route_clock = 0.0
    blocker.trajectory = [
        {
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "A->B",
            "lm": "A",
        },
        {
            "t": 5.0,
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "A->B",
            "lm": "B",
        },
    ]
    manager.robots = {
        entrant.name: entrant,
        blocker.name: blocker,
    }

    assert not manager.collision.robot_footprints_conflict(
        {"x": 1.0, "y": 0.0, "yaw": 0.0},
        blocker.pose,
    )
    assert (
        manager._controlled_corridor_downstream_blocker(
            entrant,
            "B",
            20.0,
        )
        == blocker.name
    )

    # A committed through-route which has left B by our arrival remains
    # ordinary SIPP traffic rather than a permanent downstream obstacle.
    blocker.trajectory.append({
        "t": 10.0,
        "x": 0.0,
        "y": 0.0,
        "yaw": 3.141592653589793,
        "edgeId": "B->A",
        "lm": "A",
    })
    assert (
        manager._controlled_corridor_downstream_blocker(
            entrant,
            "B",
            20.0,
        )
        == ""
    )


def test_physical_owner_keeps_downstream_blocker_after_entry_is_consumed(
    monkeypatch,
) -> None:
    manager = _manager()
    owner = _robot("owner")
    owner.route_clock = 10.0
    manager.robots = {owner.name: owner}
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda robot: _entry(0.0, 20.0)
        if robot.name == owner.name
        else None,
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda robot: {REGION} if robot.name == owner.name else set(),
    )
    manager._prepare_controlled_corridor_admissions(1_000.0)

    blocker = _robot("blocker")
    blocker.current_lm = "B"
    blocker.pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    blocker.route_clock = 20.0
    blocker.status = "WAITING"
    blocker.last_reason = "occupied by owner"
    manager.robots[blocker.name] = blocker
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: None,
    )

    manager._prepare_controlled_corridor_admissions(1_000.1)

    assert manager._controlled_corridor_blockers == {
        owner.name: blocker.name,
    }


def test_physical_exit_prediction_uses_remaining_live_trajectory() -> None:
    manager = _manager()
    robot = _robot("owner")
    robot.route_clock = 5.0

    assert manager._controlled_corridor_physical_exit_time(
        robot,
        "B",
        1_000.0,
    ) == 1_015.0


def test_central_calendar_hides_external_admission_and_ordered_queue_wait(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("waiting")
    leader = _robot("leader")
    manager.robots = {
        robot.name: robot,
        leader.name: leader,
    }
    manager.orders = {
        current.active_order_id: FleetOrder(
            order_id=current.active_order_id,
            target_lm="B",
            vehicle=current.name,
            assigned_robot=current.name,
            status="EXECUTING",
        )
        for current in manager.robots.values()
    }
    entries = {
        robot.name: _entry(1.0, 3.0),
        leader.name: _entry(0.0, 3.0),
    }
    for current in entries.values():
        current["staging_clock"] = 0.0
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda current: dict(entries[current.name]),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)
    robot.last_reason = (
        f"corridor admission wait at A for {REGION}"
    )
    assert manager._central_corridor_manages_wait(robot)

    robot.last_reason = f"occupied by {leader.name}"
    robot.wait_for_robot = leader.name
    assert manager._central_corridor_manages_wait(robot)

    robot.last_reason = "occupied by unscheduled"
    robot.wait_for_robot = ""
    assert not manager._central_corridor_manages_wait(robot)

    robot.last_reason = (
        f"corridor admission wait at A for {REGION}"
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda _robot: {REGION},
    )
    assert not manager._central_corridor_manages_wait(robot)


def test_deferred_red_light_wait_is_not_promoted_to_global_replan(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("waiting")
    owner = _robot("owner")
    robot.pose = {"x": -1.0, "y": 0.0, "yaw": 0.0}
    owner.pose = {"x": 0.1, "y": 0.0, "yaw": 0.0}
    robot.last_reason = f"occupied by {owner.name}"
    robot.wait_for_robot = owner.name
    manager.robots = {
        robot.name: robot,
        owner.name: owner,
    }
    manager.orders = {
        current.active_order_id: FleetOrder(
            order_id=current.active_order_id,
            target_lm="B",
            vehicle=current.name,
            assigned_robot=current.name,
            status="EXECUTING",
        )
        for current in manager.robots.values()
    }
    entry = _entry(0.0, 3.0)
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: dict(entry),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda current, _exit_lm, _exit_clock: (
            owner.name if current.name == robot.name else ""
        ),
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)
    decision = manager._controlled_corridor_schedule.decisions[robot.name]
    assert decision.status is CorridorDecisionStatus.DEFERRED
    assert decision.slot is None

    robot.status = "WAITING"
    robot.last_reason = (
        f"corridor admission wait at A for {REGION}; owner {owner.name}"
    )
    robot.wait_for_robot = owner.name
    assert manager._central_corridor_manages_wait(robot)

    owner.status = "WAITING"
    owner.last_reason = f"occupied by {robot.name}"
    owner.wait_for_robot = robot.name
    assert not manager._central_corridor_manages_wait(robot)

    manager._controlled_corridor_blockers = {
        robot.name: owner.name,
        owner.name: robot.name,
    }
    assert not manager._central_corridor_manages_wait(robot)

    manager._controlled_corridor_blockers.clear()
    manager._controlled_corridor_passages[owner.name]["entered"] = True
    assert not manager._central_corridor_manages_wait(robot)


def test_reciprocal_red_light_cycle_keeps_declared_owner_as_winner(
    monkeypatch,
) -> None:
    manager = _manager()
    waiter = _robot("waiter")
    owner = _robot("owner")
    waiter.status = "WAITING"
    waiter.last_reason = (
        f"corridor admission wait at A for {REGION}; owner {owner.name}"
    )
    waiter.wait_for_robot = owner.name
    owner.status = "WAITING"
    owner.last_reason = f"occupied by {waiter.name}"
    owner.wait_for_robot = waiter.name
    manager.robots = {
        waiter.name: waiter,
        owner.name: owner,
    }
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda _robot: set(),
    )
    # Model the instant at which the old slot rolled out of the current
    # calendar while the physical reciprocal dependency still exists.
    manager._controlled_corridor_schedule = None

    assert manager._controlled_corridor_cycle_owner(
        [waiter, owner],
    ) is owner


def test_rolling_chunk_cuts_on_exact_lm_not_later_mid_edge_sample() -> None:
    manager = _manager()
    trajectory = [
        {
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "lm": "A",
        },
        {
            # Continuous interpolation reaches B a fraction before the
            # discrete SIPP tick.
            "t": 0.95,
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "lm": "B",
        },
        {
            # The old fallback selected this later sample and published B as
            # the endpoint even though the body was already leaving it.
            "t": 1.0,
            "x": 1.05,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "B->next",
        },
    ]

    assert manager._trajectory_chunk_end_index(
        trajectory,
        "B",
        1.0,
    ) == 1


def test_pending_corridor_gate_queues_exact_parked_exit_clearance(
    monkeypatch,
) -> None:
    manager = _manager()
    waiter = _robot("waiter")
    blocker = FleetRobot(
        name="parked",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 1.0, "y": 0.0, "yaw": 0.0},
    )
    manager.robots = {
        waiter.name: waiter,
        blocker.name: blocker,
    }
    manager.orders[waiter.active_order_id] = FleetOrder(
        order_id=waiter.active_order_id,
        target_lm="B",
        vehicle=waiter.name,
        assigned_robot=waiter.name,
        status="EXECUTING",
    )
    manager._controlled_corridor_blockers = {
        waiter.name: blocker.name,
    }
    calls: list[tuple[str, str, str]] = []

    def queue_clearance(
        current_waiter: FleetRobot,
        current_blocker: FleetRobot,
        *,
        cause: str,
    ) -> bool:
        calls.append(
            (current_waiter.name, current_blocker.name, cause)
        )
        return True

    monkeypatch.setattr(
        manager,
        "_queue_stationary_clearance_relocation",
        queue_clearance,
    )

    assert manager._queue_controlled_corridor_exit_clearance(waiter)
    assert calls == [
        (
            waiter.name,
            blocker.name,
            "controlled corridor exit occupied",
        )
    ]


def test_physical_corridor_recovery_is_latched_until_owner_clears(
    monkeypatch,
) -> None:
    manager = _manager()
    owner = _robot("owner")
    victim = _robot("victim")
    victim.status = "RETREATING"
    manager.robots = {
        owner.name: owner,
        victim.name: victim,
    }
    physical = {owner.name: {REGION}}
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda current: set(physical.get(current.name, set())),
    )
    key = manager._controlled_corridor_recovery_latch_key(
        owner,
        {REGION},
    )
    manager._controlled_corridor_recovery_latches[key] = victim.name
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_portal_queue_component",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("latched recovery must not discover another tail")
        ),
    )

    assert (
        manager._start_deadlock_corridor_evacuation(
            [owner, victim],
            owner,
            1_000.0,
        )
        == victim.name
    )

    victim.status = "WAITING"
    assert (
        manager._start_deadlock_corridor_evacuation(
            [owner, victim],
            owner,
            1_010.0,
        )
        == ""
    )
    assert manager._controlled_corridor_recovery_latches == {
        key: victim.name
    }

    physical.clear()
    manager._prune_controlled_corridor_recovery_latches()
    assert manager._controlled_corridor_recovery_latches == {}


def test_external_stop_footprint_does_not_become_self_occupancy(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("outside")
    manager.robots = {robot.name: robot}
    manager.orders = {
        robot.active_order_id: FleetOrder(
            order_id=robot.active_order_id,
            target_lm="B",
            vehicle=robot.name,
            assigned_robot=robot.name,
            status="EXECUTING",
        )
    }
    entry = _entry(3.0, 3.0)
    entry["staging_clock"] = 0.0
    monkeypatch.setattr(
        manager,
        "_next_controlled_corridor_entry",
        lambda _robot: dict(entry),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda _robot: set(),
    )
    monkeypatch.setattr(
        manager,
        "_controlled_regions_intersecting_footprint",
        lambda _robot: {REGION},
    )
    monkeypatch.setattr(
        manager,
        "_controlled_corridor_downstream_blocker",
        lambda _robot, _exit_lm, _exit_clock: "",
    )

    manager._prepare_controlled_corridor_admissions(1_000.0)

    slot = manager._controlled_corridor_schedule.slot_for(robot.name)
    assert slot is not None
    assert not slot.physically_observed
    assert not slot.past_commit_point
    assert manager._controlled_corridor_occupancy == {}


def test_entered_owner_ignores_only_distant_preflight_until_exit(
    monkeypatch,
) -> None:
    manager = _manager()
    robot = _robot("owner")
    manager.robots = {robot.name: robot}
    manager._controlled_corridor_passages[robot.name] = {
        "entered": True,
        "past_commit_point": True,
        "route_revision": robot.route_revision,
    }
    checked: list[float] = []

    def blocked_at_clock(
        _robot: FleetRobot,
        check_clock: float,
        **_kwargs: object,
    ) -> str:
        checked.append(check_clock)
        return "occupied by distant" if check_clock > 0.2 else ""

    monkeypatch.setattr(
        manager,
        "_lookahead_robot_candidates",
        lambda _robot, _lookahead: [],
    )
    monkeypatch.setattr(manager, "_blocked_at_clock", blocked_at_clock)

    assert manager._blocked_ahead(robot, 0.05) == ""
    assert checked == [0.05]

    manager._controlled_corridor_passages.clear()
    checked.clear()
    assert manager._blocked_ahead(robot, 0.05) == "occupied by distant"
    assert checked[0] == 0.05
    assert checked[-1] > 0.2


def test_runtime_extracts_sequential_resource_windows_from_one_atomic_passage() -> None:
    region_a = "corridor:a"
    region_b = "corridor:b"
    landmarks = {
        "H": Landmark(name="H", x=0.0, y=0.0),
        "A": Landmark(
            name="A",
            x=1.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": region_a,
            },
        ),
        "M": Landmark(
            name="M",
            x=2.0,
            y=0.0,
            properties={"can_wait": False},
        ),
        "B": Landmark(
            name="B",
            x=3.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": region_b,
            },
        ),
        "X": Landmark(name="X", x=4.0, y=0.0),
    }
    edge_specs = (
        ("H", "A", region_a),
        ("A", "M", region_a),
        ("M", "B", region_b),
        ("B", "X", region_b),
    )
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[src].to_point(),
                landmarks[dst].to_point(),
            ),
            properties={
                "direction": 2,
                "controlled_region": region,
            },
        )
        for src, dst, region in edge_specs
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
                "controlled_corridor_schedule_horizon_sec": 30.0,
                "traffic_zone_control_enabled": False,
            }
        },
    )
    clocks = (0.0, 2.0, 6.0, 8.0, 10.0)
    nodes = ("H", "A", "M", "B", "X")
    robot = FleetRobot(
        name="pipeline",
        current_lm="H",
        target_lm="X",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": clock,
                "x": landmarks[node].x,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": (
                    "H->H"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, (node, clock) in enumerate(zip(nodes, clocks))
        ],
    )

    entry = manager._next_controlled_corridor_entry(robot)

    assert entry is not None
    assert entry["regions"] == (region_a, region_b)
    windows = {
        window.region_id: window
        for window in entry["resource_windows"]
        if isinstance(window, CorridorResourceWindow)
    }
    assert windows[region_a].entry_offset_sec == 0.0
    assert windows[region_a].exit_offset_sec == 6.0
    assert windows[region_b].entry_offset_sec == 6.0
    assert windows[region_b].exit_offset_sec == 10.0
    assert {
        window.direction
        for window in windows.values()
    } == {"flow:east"}


def test_safe_hold_never_trims_a_delayed_plan_back_to_its_start() -> None:
    region = "corridor:hold"
    landmarks = {
        "S": Landmark(name="S", x=0.0, y=0.0),
        "I": Landmark(
            name="I",
            x=1.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": region,
            },
        ),
        "X": Landmark(name="X", x=2.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[src].to_point(),
                landmarks[dst].to_point(),
            ),
            properties={
                "direction": 2,
                "controlled_region": region,
            },
        )
        for src, dst in (("S", "I"), ("I", "X"))
    ]
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
            }
        },
    )
    graph = manager.planner._traffic_graph(
        manager.planner._route_speed({}),
    )

    selected = manager._rolling_safe_hold_index(
        ["S", "S", "I", "X"],
        2,
        "unreached-final-goal",
        traffic_graph=graph,
    )

    assert selected == 3


def test_rolling_endpoint_stays_clear_of_next_corridor_portal() -> None:
    region = "corridor:portal"
    landmarks = {
        "S": Landmark(name="S", x=0.0, y=0.0),
        "Q": Landmark(name="Q", x=1.0, y=0.0),
        "P": Landmark(name="P", x=2.0, y=0.0),
        "I": Landmark(
            name="I",
            x=3.0,
            y=0.0,
            properties={
                "can_wait": False,
                "controlled_region": region,
            },
        ),
        "X": Landmark(name="X", x=4.0, y=0.0),
    }
    edges = []
    for src, dst in (("S", "Q"), ("Q", "P"), ("P", "I"), ("I", "X")):
        properties = {"direction": 2}
        if src in {"P", "I"}:
            properties["controlled_region"] = region
        edges.append(
            GraphEdge(
                from_name=src,
                to_name=dst,
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    landmarks[src].to_point(),
                    landmarks[dst].to_point(),
                ),
                properties=properties,
            )
        )
    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "controlled_corridors_enabled": True,
                "controlled_corridor_auto_detect": False,
            }
        },
    )
    graph = manager.planner._traffic_graph(
        manager.planner._route_speed({}),
    )

    selected = manager._rolling_safe_hold_index(
        ["S", "Q", "P", "I", "X"],
        2,
        "unreached-final-goal",
        traffic_graph=graph,
    )

    assert selected == 1


def test_downstream_exit_clearer_moves_before_physical_corridor_owner(
    monkeypatch,
) -> None:
    manager = _manager()
    owner = _robot("owner")
    clearer = _robot("clearer")
    owner.status = "WAITING"
    owner.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    owner.last_reason = "yield to clearer"
    owner.wait_for_robot = clearer.name
    clearer.status = "WAITING"
    clearer.pose = {"x": 0.7, "y": 0.0, "yaw": 0.0}
    clearer.trajectory = [
        {
            "t": 0.0,
            "x": 0.7,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "A->B",
            "lm": "A",
        },
        {
            "t": 1.0,
            "x": 1.7,
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": "A->B",
            "lm": "B",
        },
    ]
    clearer.last_reason = "occupied by owner"
    clearer.wait_for_robot = owner.name
    manager.robots = {
        owner.name: owner,
        clearer.name: clearer,
    }
    manager._controlled_corridor_blockers = {
        owner.name: clearer.name,
    }
    manager._controlled_corridor_occupancy = {
        REGION: [owner.name],
    }
    monkeypatch.setattr(
        manager,
        "_controlled_regions_for_robot",
        lambda robot: {REGION} if robot.name == owner.name else set(),
    )

    manager._break_runtime_wait_cycle(
        [owner.name, clearer.name],
        dict(manager.robots),
        1_000.0,
    )

    assert clearer.status == "MOVING"
    assert clearer.last_reason == "deadlock priority granted"
    assert clearer.traffic_priority_until > 1_000.0
    assert owner.status == "WAITING"
    assert owner.last_reason == "yield to clearer"
