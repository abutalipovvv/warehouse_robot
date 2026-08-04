from __future__ import annotations

from typing import Any

from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
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
                # These tests exercise the quarantine fallback explicitly.
                # The normal runtime first tries an internal traffic-clearance
                # move, covered by test_fleet_web_runtime.
                "parked_clearance_relocation_enabled": False,
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


def test_unrelated_parked_blocker_stays_quarantined_until_occupancy_changes(
    monkeypatch: Any,
) -> None:
    manager = _manager()
    order, _, blocker, clock, planner_calls = _install_blocked_departure(
        manager,
        monkeypatch,
    )
    _fail_same_stationary_departure_twice(manager, order, clock)
    assert planner_calls[0] == 2

    # Replanning cannot move an unrelated parked physical body. Keep the
    # failed order asleep even after the ordinary recovery cooldown.
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


def test_mutually_commanded_stationary_departures_retry_at_bounded_cadence(
    monkeypatch: Any,
) -> None:
    manager = _manager()
    order, _, blocker, clock, planner_calls = _install_blocked_departure(
        manager,
        monkeypatch,
    )
    manager.orders["blocker-departure"] = FleetOrder(
        order_id="blocker-departure",
        target_lm="D",
        vehicle=blocker.name,
        assigned_robot=blocker.name,
        status="QUEUED",
        created_at=0.0,
        updated_at=0.0,
    )
    _fail_same_stationary_departure_twice(manager, order, clock)

    clock[0] += 30.0
    manager._dispatch_orders()

    assert planner_calls[0] >= 3
    assert order.dispatch_failures >= 3


def test_continuous_turn_envelope_block_exports_stationary_identity(
    monkeypatch: Any,
) -> None:
    manager = _manager()
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="B",
        status="ARRIVED",
        pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
    )
    conflict = {"time": 0.5, "other": "parked", "edge": "A->B"}
    monkeypatch.setattr(
        manager,
        "_first_continuous_corridor_conflict",
        lambda *_args, **_kwargs: conflict,
    )
    monkeypatch.setattr(manager, "_reservation_horizon", lambda: 5.0)
    monkeypatch.setattr(
        manager,
        "_wait_duration_for_conflict",
        lambda *_args, **_kwargs: 5.0,
    )
    result = manager._apply_continuous_reservation_waits({
        "ok": True,
        "debug": {"reason": "success"},
        "plans": [{
            "robot": "moving",
            "trajectory": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
                {"t": 1.0, "x": 2.0, "y": 0.0, "yaw": 0.0, "edgeId": "A->B"},
            ],
        }],
    })

    assert not result["ok"]
    assert result["plans"] == []
    debug = result["debug"]
    assert debug["continuousConflictRobot"] == "parked"
    assert debug["continuousConflictEdge"] == "A->B"
    assert debug["stationaryTurnEnvelopeBlock"]
    assert debug["stationaryBlockerRobots"] == ["parked"]
    assert debug["softBlockedLms"] == ["B"]
    assert "stationary_robot_blocks_route" in debug["deadlockReason"]


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
