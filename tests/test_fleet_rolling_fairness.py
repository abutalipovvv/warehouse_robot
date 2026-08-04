from __future__ import annotations

import pytest

from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.simulation.manager import FleetManagerSim


def test_old_boundary_waiter_outranks_fresh_zero_failure_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = _manager()
    now = 1_000.0
    monkeypatch.setattr(manager, "_now", lambda: now)

    # Insert and name the fresh robot so incidental dict/name ordering favours
    # it. Fairness must instead select the robot that has already spent a long
    # time stopped at its rolling boundary, even after repeated failed plans.
    _add_boundary_waiter(
        manager,
        name="a-fresh",
        start_lm="N0",
        goal_lm="N1",
        waiting_since=now - 1.0,
        failures=0,
    )
    _add_boundary_waiter(
        manager,
        name="z-old",
        start_lm="O0",
        goal_lm="O1",
        waiting_since=now - 120.0,
        failures=7,
    )

    entries = manager._ready_rolling_prefetch_entries()

    assert entries
    assert entries[0][1].name == "z-old"


def test_boundary_retry_delay_is_bounded_independent_of_failures_and_time_scale(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = _manager()
    now = 2_000.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    order, robot = _add_boundary_waiter(
        manager,
        name="boundary",
        start_lm="O0",
        goal_lm="O1",
        waiting_since=now - 60.0,
        failures=1,
    )

    delays: list[float] = []
    for failures, time_scale in ((1, 1.0), (8, 1.0), (1, 4.0), (80, 4.0)):
        manager._rolling_prefetch_failures[robot.name] = failures
        monkeypatch.setattr(
            manager,
            "simulation_time_scale",
            lambda scale=time_scale: scale,
        )
        manager._defer_rolling_prefetch(robot, order)
        delays.append(manager._rolling_prefetch_retry_at[robot.name] - now)

    assert delays == pytest.approx([delays[0]] * len(delays))
    assert delays[0] <= manager.params["fleet"]["order_dispatch_retry_max_sec"]


def test_full_collapse_without_vacancy_falls_back_to_an_endpoint_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = _closed_corridor_manager()
    now = 3_000.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    _add_boundary_waiter(
        manager,
        name="from-a",
        start_lm="A",
        goal_lm="B",
        waiting_since=now - 30.0,
        failures=1,
    )
    _add_boundary_waiter(
        manager,
        name="from-b",
        start_lm="B",
        goal_lm="A",
        waiting_since=now - 20.0,
        failures=1,
    )

    # Every LM is occupied and each route needs the other robot's endpoint.
    # There is no third wait pocket, but that must not turn full-collapse
    # recovery into a permanent empty scheduler result.
    entries = manager._ready_rolling_prefetch_entries()

    assert entries
    assert entries[0][1].name in {"from-a", "from-b"}
    assert entries[0][-1] == pytest.approx(0.0)
    assert not entries[0][2].get("vacancyRecovery", False)


def test_overdue_boundary_is_not_starved_by_an_urgent_moving_continuation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = _manager()
    now = 4_000.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    _add_boundary_waiter(
        manager,
        name="overdue-boundary",
        start_lm="O0",
        goal_lm="O1",
        waiting_since=now - 30.0,
        failures=1,
    )
    remaining = min(0.25, manager._rolling_prefetch_urgent_lead() / 2.0)
    _add_moving_continuation(
        manager,
        name="urgent-moving",
        start_lm="N0",
        chunk_goal_lm="N1",
        final_goal_lm="N2",
        remaining=remaining,
        now=now,
    )

    assert 0.0 < remaining <= manager._rolling_prefetch_urgent_lead()
    entries = manager._ready_rolling_prefetch_entries()

    assert entries
    assert entries[0][1].name == "overdue-boundary"


def test_just_attempted_boundary_yields_to_the_earlier_moving_deadline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = _manager()
    now = 5_000.0
    monkeypatch.setattr(manager, "_now", lambda: now)
    order, boundary = _add_boundary_waiter(
        manager,
        name="just-attempted-boundary",
        start_lm="O0",
        goal_lm="O1",
        waiting_since=now - 30.0,
        failures=4,
    )
    manager._rolling_prefetch_last_attempt_at[boundary.name] = now
    retry_quantum = manager._rolling_boundary_retry_interval(order)
    remaining = retry_quantum / 2.0
    _add_moving_continuation(
        manager,
        name="earlier-moving-deadline",
        start_lm="N0",
        chunk_goal_lm="N1",
        final_goal_lm="N2",
        remaining=remaining,
        now=now,
    )

    entries = manager._ready_rolling_prefetch_entries()

    assert entries
    assert entries[0][1].name == "earlier-moving-deadline"


def _manager() -> FleetManagerSim:
    landmarks = {
        "N0": Landmark(name="N0", x=0.0, y=0.0),
        "N1": Landmark(name="N1", x=2.0, y=0.0),
        "N2": Landmark(name="N2", x=4.0, y=0.0),
        "O0": Landmark(name="O0", x=0.0, y=4.0),
        "O1": Landmark(name="O1", x=2.0, y=4.0),
    }
    edges = [
        GraphEdge(
            from_name=start_lm,
            to_name=goal_lm,
            length=2.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[start_lm].x, landmarks[start_lm].y),
                WorldPoint(landmarks[goal_lm].x, landmarks[goal_lm].y),
            ),
            properties={"direction": 1},
        )
        for start_lm, goal_lm in (
            ("N0", "N1"),
            ("N1", "N2"),
            ("O0", "O1"),
        )
    ]
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "order_dispatch_retry_sec": 0.25,
                "order_dispatch_retry_max_sec": 0.75,
                "rolling_prefetch_recovery_batch_size": 2,
            },
        },
    )


def _closed_corridor_manager() -> FleetManagerSim:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name=start_lm,
            to_name=goal_lm,
            length=2.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[start_lm].x, landmarks[start_lm].y),
                WorldPoint(landmarks[goal_lm].x, landmarks[goal_lm].y),
            ),
            properties={"direction": 1},
        )
        for start_lm, goal_lm in (("A", "B"), ("B", "A"))
    ]
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "order_dispatch_retry_sec": 0.25,
                "order_dispatch_retry_max_sec": 0.75,
                "rolling_prefetch_recovery_batch_size": 2,
            },
        },
    )


def _add_boundary_waiter(
    manager: FleetManagerSim,
    *,
    name: str,
    start_lm: str,
    goal_lm: str,
    waiting_since: float,
    failures: int,
) -> tuple[FleetOrder, FleetRobot]:
    start = manager.landmarks[start_lm]
    order = FleetOrder(
        order_id=f"order-{name}",
        target_lm=goal_lm,
        vehicle=name,
        assigned_robot=name,
        status="PLANNING",
        created_at=waiting_since,
        updated_at=waiting_since,
        error="rolling continuation pending",
        spatial_route_nodes=[start_lm, goal_lm],
    )
    robot = FleetRobot(
        name=name,
        current_lm=start_lm,
        target_lm=start_lm,
        status="WAITING",
        pose={"x": start.x, "y": start.y, "yaw": 0.0},
        trajectory=[
            {"x": start.x, "y": start.y, "yaw": 0.0, "t": 0.0, "lm": start_lm},
            {"x": start.x, "y": start.y, "yaw": 0.0, "t": 1.0, "lm": start_lm},
        ],
        route_clock=1.0,
        active_order_id=order.order_id,
        route_chunk_goal_lm=start_lm,
        route_final_lm=goal_lm,
        route_revision=1,
        has_executed_route=True,
        last_reason="rolling continuation pending",
        updated_at=waiting_since,
    )
    manager.orders[order.order_id] = order
    manager.robots[name] = robot
    manager._rolling_prefetch_failures[name] = failures
    return order, robot


def _add_moving_continuation(
    manager: FleetManagerSim,
    *,
    name: str,
    start_lm: str,
    chunk_goal_lm: str,
    final_goal_lm: str,
    remaining: float,
    now: float,
) -> tuple[FleetOrder, FleetRobot]:
    start = manager.landmarks[start_lm]
    chunk_goal = manager.landmarks[chunk_goal_lm]
    final_time = 10.0
    order = FleetOrder(
        order_id=f"order-{name}",
        target_lm=final_goal_lm,
        vehicle=name,
        assigned_robot=name,
        status="EXECUTING",
        created_at=now - 10.0,
        updated_at=now,
        spatial_route_nodes=[start_lm, chunk_goal_lm, final_goal_lm],
    )
    robot = FleetRobot(
        name=name,
        current_lm=start_lm,
        target_lm=chunk_goal_lm,
        status="MOVING",
        pose={"x": start.x, "y": start.y, "yaw": 0.0},
        trajectory=[
            {"x": start.x, "y": start.y, "yaw": 0.0, "t": 0.0, "lm": start_lm},
            {
                "x": chunk_goal.x,
                "y": chunk_goal.y,
                "yaw": 0.0,
                "t": final_time,
                "lm": chunk_goal_lm,
            },
        ],
        route_clock=final_time - remaining,
        active_order_id=order.order_id,
        route_chunk_goal_lm=chunk_goal_lm,
        route_final_lm=final_goal_lm,
        route_revision=2,
        has_executed_route=True,
        last_reason="moving",
        updated_at=now,
    )
    manager.orders[order.order_id] = order
    manager.robots[name] = robot
    return order, robot
