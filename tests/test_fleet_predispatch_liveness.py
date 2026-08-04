from __future__ import annotations

from pathlib import Path
from time import monotonic, sleep

import yaml

from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.maps.models import (
    GraphEdge,
    Landmark,
    WorldPoint,
)
from fleet_manager.runtime.simulation.manager import FleetManagerSim


ROOT = Path(__file__).resolve().parents[1]
SMART_KIVA = (
    ROOT
    / "fleet_manager"
    / "map_data"
    / "maps_out"
    / "smart_kiva_large_w_mode.smap"
)


def test_stale_traffic_detour_is_released_when_new_occupancy_closes_it() -> None:
    """A transient edge ban must not freeze a later package-wave barrier."""
    landmarks = {
        "S": Landmark(name="S", x=0.0, y=0.0),
        "D": Landmark(name="D", x=1.0, y=0.0),
        "G": Landmark(name="G", x=2.0, y=0.0),
        "A": Landmark(name="A", x=0.0, y=1.0),
        "B": Landmark(name="B", x=1.0, y=1.0),
    }
    pairs = (
        ("S", "D"),
        ("D", "G"),
        ("S", "A"),
        ("A", "B"),
        ("B", "G"),
    )
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=(
                (landmarks[src].x - landmarks[dst].x) ** 2
                + (landmarks[src].y - landmarks[dst].y) ** 2
            ) ** 0.5,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(landmarks[src].x, landmarks[src].y),
                WorldPoint(landmarks[dst].x, landmarks[dst].y),
            ),
            properties={"direction": 1},
        )
        for first, second in pairs
        for src, dst in ((first, second), (second, first))
    ]
    manager = FleetManagerSim(landmarks, edges)
    owner = FleetRobot(name="owner", current_lm="S", status="ARRIVED")
    blocker = FleetRobot(name="parked", current_lm="A", status="ARRIVED")
    manager.robots = {owner.name: owner, blocker.name: blocker}
    order = FleetOrder(
        order_id="package-owner",
        target_lm="G",
        vehicle=owner.name,
        assigned_robot=owner.name,
        start_lm="S",
        status="QUEUED",
        traffic_detour_edges=[("S", "D"), ("D", "S")],
    )
    manager.orders[order.order_id] = order

    route = manager._ensure_order_spatial_route(
        order,
        "S",
        "G",
        release_robot_names={owner.name},
    )

    assert route == ["S", "D", "G"]
    assert order.traffic_detour_edges == []
    assert any(
        "released stale traffic detour" in event.message
        for event in manager.events
    )


def test_live_wave_three_mutual_stationary_departures_get_atomic_detour() -> None:
    """Regression for the 58/60 package-wave stall captured in the browser."""
    loaded = WarehouseMapLoader(SMART_KIVA).load()
    params = yaml.safe_load(
        (ROOT / "fleet_manager" / "config" / "params.yaml").read_text(
            encoding="utf-8"
        )
    )
    manager = FleetManagerSim(
        loaded.landmarks,
        loaded.edges,
        params=params,
        map_dir=SMART_KIVA,
        map_metadata=loaded.map_metadata,
    )

    # Exact graph occupancy from the failed third package wave. All robots had
    # arrived; only bench_001 and bench_008 still owned normal queued orders.
    occupied = {
        "bench_001": "S002033",
        "bench_002": "S016032",
        "bench_003": "S006034",
        "bench_004": "S002012",
        "bench_005": "S002005",
        "bench_006": "S004030",
        "bench_007": "S006032",
        "bench_008": "S002031",
        "bench_009": "S032011",
        "bench_010": "S026034",
        "bench_011": "S010034",
        "bench_012": "S032034",
        "bench_013": "S032018",
        "bench_014": "S030011",
        "bench_015": "S030021",
        "bench_016": "S016003",
        "bench_017": "S004008",
        "bench_018": "S020003",
        "bench_019": "S024003",
        "bench_020": "S022003",
    }
    for name, lm_name in occupied.items():
        pose = manager._pose_at_landmark(lm_name)
        assert pose is not None
        pose["yaw"] = 3.141592653589793
        manager.robots[name] = FleetRobot(
            name=name,
            current_lm=lm_name,
            status="ARRIVED",
            pose=pose,
        )

    orders = (
        FleetOrder(
            order_id="wave-3-bench-001",
            target_lm="S002003",
            vehicle="bench_001",
            assigned_robot="bench_001",
            start_lm="S002033",
            status="QUEUED",
            priority=0,
            speed=1.37,
            acceleration=0.6,
            rotate=True,
            turn_speed=0.9,
            dispatch_failures=32,
        ),
        FleetOrder(
            order_id="wave-3-bench-008",
            target_lm="S032032",
            vehicle="bench_008",
            assigned_robot="bench_008",
            start_lm="S002031",
            status="QUEUED",
            priority=1,
            speed=1.37,
            acceleration=0.6,
            rotate=True,
            turn_speed=0.9,
            dispatch_failures=35,
        ),
    )
    manager.orders = {order.order_id: order for order in orders}

    dispatched = manager._dispatch_orders(async_simulated=True)
    deadline = monotonic() + 6.0
    while monotonic() < deadline and not all(
        order.status == "EXECUTING" for order in orders
    ):
        # Production refreshes the central light calendar on every runtime
        # tick before another dispatch turn. This direct regression loop must
        # do the same after the first turn registers the next robot's intent.
        manager._prepare_controlled_corridor_admissions(manager._now())
        dispatched += manager._dispatch_orders(async_simulated=True)
        sleep(0.01)

    assert dispatched == 2
    assert all(order.status == "EXECUTING" for order in orders)
    assert all(manager.robots[order.vehicle].trajectory for order in orders)

    yielding = manager.orders["wave-3-bench-001"]
    assert yielding.traffic_detour_attempts == 1
    assert yielding.spatial_route_nodes[:2] == ["S002033", "S002034"]
    assert "S002031" not in yielding.spatial_route_nodes
    assert any(
        "pre-dispatch traffic release" in event.message
        for event in manager.events
    )


def _adjacent_departure_pairs(
    *,
    pair_count: int = 4,
    local_limit: int = 2,
) -> tuple[
    FleetManagerSim,
    list[tuple[FleetOrder, FleetRobot, dict[str, str], str]],
]:
    landmarks: dict[str, Landmark] = {}
    edges: list[GraphEdge] = []

    def landmark(name: str, x: float, y: float) -> None:
        landmarks[name] = Landmark(name=name, x=x, y=y)

    def edge(src: str, dst: str) -> None:
        edges.append(
            GraphEdge(
                from_name=src,
                to_name=dst,
                length=(
                    (landmarks[src].x - landmarks[dst].x) ** 2
                    + (landmarks[src].y - landmarks[dst].y) ** 2
                ) ** 0.5,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(landmarks[src].x, landmarks[src].y),
                    WorldPoint(landmarks[dst].x, landmarks[dst].y),
                ),
                properties={"direction": 1},
            )
        )

    pair_nodes: list[tuple[str, str, str, str]] = []
    for pair_index in range(pair_count):
        base_x = float(pair_index * 10)
        left = f"P{pair_index}A"
        right = f"P{pair_index}B"
        left_goal = f"P{pair_index}GA"
        right_goal = f"P{pair_index}GB"
        left_bypass = f"P{pair_index}XA"
        right_bypass = f"P{pair_index}XB"
        landmark(left, base_x, 0.0)
        landmark(right, base_x + 1.0, 0.0)
        landmark(left_goal, base_x + 2.0, 0.0)
        landmark(right_goal, base_x - 1.0, 0.0)
        landmark(left_bypass, base_x, 1.0)
        landmark(right_bypass, base_x + 1.0, 1.0)
        edge(left, right)
        edge(right, left_goal)
        edge(right, left)
        edge(left, right_goal)
        # Pair zero deliberately has no bypass. It remains an unchanged
        # component and proves it cannot monopolise later scheduler turns.
        if pair_index:
            edge(left, left_bypass)
            edge(left_bypass, left_goal)
            edge(right, right_bypass)
            edge(right_bypass, right_goal)
        pair_nodes.append((left, right, left_goal, right_goal))

    manager = FleetManagerSim(
        landmarks,
        edges,
        params={
            "fleet": {
                "local_cbs_max_robots": local_limit,
                "congestion_routing_enabled": False,
            }
        },
    )
    entries: list[tuple[FleetOrder, FleetRobot, dict[str, str], str]] = []
    for pair_index, (left, right, left_goal, right_goal) in enumerate(
        pair_nodes
    ):
        for suffix, start, peer, goal in (
            ("a", left, right, left_goal),
            ("b", right, left, right_goal),
        ):
            name = f"r{pair_index}{suffix}"
            order = FleetOrder(
                order_id=f"o-{name}",
                target_lm=goal,
                vehicle=name,
                assigned_robot=name,
                start_lm=start,
                status="QUEUED",
                dispatch_failures=3,
                spatial_route_nodes=[start, peer, goal],
                spatial_route_revision=100 + len(entries),
            )
            robot = FleetRobot(
                name=name,
                current_lm=start,
                status="ARRIVED",
            )
            manager.orders[order.order_id] = order
            manager.robots[name] = robot
            entries.append(
                (
                    order,
                    robot,
                    {"name": name, "startLm": start, "goalLm": goal},
                    goal,
                )
            )
    return manager, entries


def test_predispatch_rotates_disjoint_components_without_revision_churn(
    monkeypatch,
) -> None:
    manager, entries = _adjacent_departure_pairs()
    route_calls: list[tuple[str, str]] = []
    original_find_route = manager.planner.route_planner.find_route

    def counted_find_route(start: str, goal: str, **kwargs):
        route_calls.append((start, goal))
        return original_find_route(start, goal, **kwargs)

    monkeypatch.setattr(
        manager.planner.route_planner,
        "find_route",
        counted_find_route,
    )

    selected_components: list[frozenset[str]] = []
    calls_per_turn: list[int] = []
    for _ in range(4):
        before = len(route_calls)
        selected, _ = manager._coordinate_mutual_stationary_departures(
            entries
        )
        selected_components.append(frozenset(selected))
        calls_per_turn.append(len(route_calls) - before)

    assert selected_components == [
        frozenset({"r0a", "r0b"}),
        frozenset({"r1a", "r1b"}),
        frozenset({"r2a", "r2b"}),
        frozenset({"r3a", "r3b"}),
    ]
    assert all(count <= manager.planner.local_cbs_max_robots for count in calls_per_turn)

    # Every pair with a real bypass is now free of immediate peer starts. The
    # first pair is intentionally unavoidable and remains stable rather than
    # starving the other components or allocating revisions forever.
    for pair_index in range(1, 4):
        for suffix, peer_suffix in (("a", "b"), ("b", "a")):
            order = manager.orders[f"o-r{pair_index}{suffix}"]
            peer = manager.robots[f"r{pair_index}{peer_suffix}"].current_lm
            assert peer not in order.spatial_route_nodes[1:5]
            assert order.traffic_detour_attempts == 1

    stable = {
        order.order_id: (
            order.spatial_route_revision,
            order.traffic_detour_attempts,
            tuple(order.spatial_route_nodes),
        )
        for order in manager.orders.values()
    }
    call_count = len(route_calls)
    for order in manager.orders.values():
        order.dispatch_failures += 20
        order.updated_at += 100.0
    for _ in range(6):
        selected, _ = manager._coordinate_mutual_stationary_departures(
            entries
        )
        assert selected == {"r0a", "r0b"}

    assert len(route_calls) == call_count
    assert stable == {
        order.order_id: (
            order.spatial_route_revision,
            order.traffic_detour_attempts,
            tuple(order.spatial_route_nodes),
        )
        for order in manager.orders.values()
    }


def test_predispatch_bypass_survives_stationary_release_overlap(
    monkeypatch,
) -> None:
    manager, entries = _adjacent_departure_pairs(pair_count=2)
    # Drop the intentionally impossible first pair so the selected component
    # has a valid stable bypass.
    for order, robot, _, _ in entries[:2]:
        manager.orders.pop(order.order_id)
        manager.robots.pop(robot.name)
    monkeypatch.setattr(
        manager,
        "_stationary_release_robot_names",
        lambda: {"r1a"},
    )
    captured: list[dict[str, tuple[str, ...]]] = []

    def capture_batch(group):
        captured.append(
            {
                robot.name: tuple(order.spatial_route_nodes)
                for order, robot, _, _ in group
            }
        )
        return 0, {order.order_id for order, _, _, _ in group}

    monkeypatch.setattr(manager, "_dispatch_simulated_order_batch", capture_batch)

    manager._dispatch_orders()
    # The detour removes the predeparture conflict, so the next call reaches
    # the ordinary stationary-release path. Its protected fingerprint must
    # still prevent that path from erasing the stable bypass.
    manager._dispatch_orders()

    assert captured
    assert set(captured[0]) == {"r1a", "r1b"}
    assert captured[0]["r1a"] == ("P1A", "P1XA", "P1GA")
    assert captured[0]["r1b"] == ("P1B", "P1XB", "P1GB")
    repeated_left_routes = [
        routes["r1a"]
        for routes in captured
        if "r1a" in routes
    ]
    assert len(repeated_left_routes) >= 2
    assert set(repeated_left_routes) == {("P1A", "P1XA", "P1GA")}


def test_predispatch_retry_unlocks_when_external_stationary_blocker_moves(
    monkeypatch,
) -> None:
    manager, entries = _adjacent_departure_pairs(pair_count=2)
    entries = entries[2:]
    for order_id in ("o-r0a", "o-r0b"):
        manager.orders.pop(order_id)
    for robot_name in ("r0a", "r0b"):
        manager.robots.pop(robot_name)
    manager.robots["parked"] = FleetRobot(
        name="parked",
        current_lm="P1XA",
        status="STOPPED",
    )
    route_calls: list[tuple[str, str]] = []
    original_find_route = manager.planner.route_planner.find_route

    def counted_find_route(start: str, goal: str, **kwargs):
        route_calls.append((start, goal))
        return original_find_route(start, goal, **kwargs)

    monkeypatch.setattr(
        manager.planner.route_planner,
        "find_route",
        counted_find_route,
    )

    manager._coordinate_mutual_stationary_departures(entries)
    first_call_count = len(route_calls)
    assert manager.orders["o-r1a"].traffic_detour_attempts == 0
    assert manager.orders["o-r1b"].traffic_detour_attempts == 1

    manager._coordinate_mutual_stationary_departures(entries)
    assert len(route_calls) == first_call_count

    manager.robots["parked"].current_lm = "P0XA"
    manager._coordinate_mutual_stationary_departures(entries)

    assert len(route_calls) == first_call_count + 1
    assert manager.orders["o-r1a"].traffic_detour_attempts == 1
    assert manager.orders["o-r1a"].spatial_route_nodes == [
        "P1A",
        "P1XA",
        "P1GA",
    ]


def test_predispatch_does_not_claim_atomicity_for_mixed_motion_settings() -> None:
    manager, entries = _adjacent_departure_pairs(pair_count=1)
    entries[1][0].speed = 0.5

    selected, rerouted = manager._coordinate_mutual_stationary_departures(
        entries
    )

    assert not selected
    assert not rerouted
    assert all(order.traffic_detour_attempts == 0 for order in manager.orders.values())
