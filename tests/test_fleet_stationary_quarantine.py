from __future__ import annotations

from typing import Any

from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.simulation.manager import FleetManagerSim


def _manager() -> FleetManagerSim:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=0.0),
        "C": Landmark(name="C", x=4.0, y=0.0),
        "D": Landmark(name="D", x=2.0, y=2.0),
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=2.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[src].x, landmarks[src].y),
                WorldPoint(landmarks[dst].x, landmarks[dst].y),
            ),
            properties={"direction": 1},
        )
        for src, dst in (("A", "B"), ("B", "C"), ("B", "D"))
    ]
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "order_dispatch_retry_sec": 0.5,
                "order_dispatch_retry_max_sec": 4.0,
            },
        },
    )


def _stationary_failure(robot_name: str) -> dict[str, Any]:
    return {
        "ok": False,
        "plans": [],
        "debug": {
            "reason": (
                f"no_low_level_path:{robot_name}:A->C:"
                "stationary_robot_blocks_route"
            ),
            "stationaryRobotWait": True,
            "softBlockedLms": ["B"],
        },
    }


def _install_blocked_departure(
    manager: FleetManagerSim,
    monkeypatch: Any,
) -> tuple[FleetOrder, FleetRobot, FleetRobot, list[float], list[int]]:
    departing = FleetRobot(
        name="departing",
        current_lm="A",
        status="IDLE",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    blocker = FleetRobot(
        name="obstacle",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        order_id="blocked-order",
        target_lm="C",
        vehicle=departing.name,
        assigned_robot=departing.name,
        status="QUEUED",
        created_at=0.0,
        updated_at=0.0,
    )
    manager.robots = {
        departing.name: departing,
        blocker.name: blocker,
    }
    manager.orders = {order.order_id: order}

    clock = [100.0]
    planner_calls = [0]
    monkeypatch.setattr(manager, "_now", lambda: clock[0])

    def fail_stationary_route(
        requests: list[dict[str, Any]],
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        del requests, payload
        planner_calls[0] += 1
        return _stationary_failure(departing.name)

    monkeypatch.setattr(manager, "_plan_valid_requests", fail_stationary_route)
    return order, departing, blocker, clock, planner_calls


def _fail_same_stationary_departure_twice(
    manager: FleetManagerSim,
    order: FleetOrder,
    clock: list[float],
) -> None:
    manager._dispatch_orders()
    assert order.dispatch_failures == 1

    clock[0] += 10.0
    manager._dispatch_orders()
    assert order.dispatch_failures == 2


def test_stationary_failure_quarantine_waits_for_occupancy_change(
    monkeypatch: Any,
) -> None:
    manager = _manager()
    order, _, blocker, clock, planner_calls = _install_blocked_departure(
        manager,
        monkeypatch,
    )
    _fail_same_stationary_departure_twice(manager, order, clock)
    assert planner_calls[0] == 2

    # Advancing far beyond the ordinary four-second retry cap must not spend
    # the only planner slot on the exact same impossible departure again.
    clock[0] += 30.0
    manager._dispatch_orders()
    assert planner_calls[0] == 2
    assert order.dispatch_failures == 2

    # A real occupancy transition invalidates the quarantine signature and
    # permits one fresh attempt immediately.
    blocker.current_lm = "D"
    blocker.pose = {"x": 2.0, "y": 2.0, "yaw": 0.0}
    blocker.route_revision += 1
    clock[0] += 0.1
    manager._dispatch_orders()
    assert planner_calls[0] == 3


def test_quarantined_departure_remains_a_stationary_route_obstacle(
    monkeypatch: Any,
) -> None:
    manager = _manager()
    order, departing, blocker, clock, _ = _install_blocked_departure(
        manager,
        monkeypatch,
    )
    _fail_same_stationary_departure_twice(manager, order, clock)

    # Exclude the original obstacle to isolate the route-less departure. Its
    # queued command must not make the physical robot disappear from MAPF.
    assert manager._stationary_robot_blocked_lms(
        exclude_robot_names={blocker.name},
    ) == {departing.current_lm}
