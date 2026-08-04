from __future__ import annotations

from typing import Any

import pytest

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.manager.tasks.manager import FleetTaskManager
from fleet_manager.runtime.simulation.manager import FleetManagerSim


def test_many_rolling_appends_keep_trajectory_and_plan_nodes_bounded() -> None:
    manager = _line_manager(80)
    final_goal = "N80"
    order = FleetOrder(
        order_id="order-1",
        target_lm=final_goal,
        vehicle="robot-1",
        assigned_robot="robot-1",
        status="EXECUTING",
    )
    robot = FleetRobot(
        name="robot-1",
        current_lm="N0",
        target_lm="N1",
        status="MOVING",
        pose=_pose(0),
        trajectory=_segment("N0", "N1"),
        plan_nodes=["N0", "N1"],
        active_order_id=order.order_id,
        route_chunk_goal_lm="N1",
        route_final_lm=final_goal,
    )
    manager.robots = {robot.name: robot}
    manager.orders = {order.order_id: order}

    maximum_trajectory_size = len(robot.trajectory)
    maximum_plan_size = len(robot.plan_nodes)
    route_clocks: list[float] = []
    retained_first_times: list[float] = []
    for goal_index in range(2, 81):
        start_index = goal_index - 1
        start_lm = f"N{start_index}"
        goal_lm = f"N{goal_index}"
        robot.current_lm = start_lm
        robot.pose = _pose(start_index)
        robot.route_clock = float(robot.trajectory[-1]["t"])
        route_clock_before_append = robot.route_clock
        pose_before_append = manager._pose_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        )

        appended = manager._append_rolling_prefetch(
            robot,
            order,
            {
                "startLm": start_lm,
                "goalLm": goal_lm,
                "finalGoalLm": final_goal,
                "nodes": [start_lm, goal_lm],
                "trajectory": _segment(start_lm, goal_lm),
            },
            final_goal,
        )

        assert appended
        assert robot.route_clock == pytest.approx(route_clock_before_append)
        assert manager._pose_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        ) == pose_before_append
        times = [float(sample["t"]) for sample in robot.trajectory]
        assert times == sorted(times)
        route_clocks.append(robot.route_clock)
        retained_first_times.append(times[0])
        maximum_trajectory_size = max(
            maximum_trajectory_size,
            len(robot.trajectory),
        )
        maximum_plan_size = max(maximum_plan_size, len(robot.plan_nodes))

    # One previous LM is retained for a graph-safe retreat, while already
    # executed order history does not grow with the number of rolling chunks.
    assert maximum_trajectory_size <= 4
    assert maximum_plan_size <= 4
    assert robot.plan_nodes == ["N77", "N78", "N79", "N80"]
    assert order.route_nodes == robot.plan_nodes
    assert route_clocks == sorted(route_clocks)
    assert retained_first_times == sorted(retained_first_times)
    assert retained_first_times[-1] > retained_first_times[0]


def test_reset_discards_inflight_planner_result_and_clears_prefetch_state() -> None:
    manager = _line_manager(2)
    robot = FleetRobot(name="robot-1", current_lm="N0")
    manager.robots[robot.name] = robot
    manager._rolling_prefetch_retry_at[robot.name] = 123.0
    manager._rolling_prefetch_failures[robot.name] = 7
    manager._rolling_prefetch_eligible_since[robot.name] = 100.0
    manager._rolling_prefetch_last_attempt_at[robot.name] = 110.0
    robot.rolling_boundary_since = 90.0
    manager._last_async_job_kind = "prefetch"
    stale_job: dict[str, Any] = {
        "kind": "prefetch",
        "done": False,
        "result": {
            "ok": True,
            "plans": [
                {
                    "robot": robot.name,
                    "startLm": "N0",
                    "goalLm": "N1",
                    "nodes": ["N0", "N1"],
                    "trajectory": _segment("N0", "N1"),
                }
            ],
        },
    }
    with manager._dispatch_job_lock:
        manager._dispatch_job = stale_job

    manager.reset_planning_runtime_state()

    assert stale_job["discard"] is True
    assert manager._dispatch_job is stale_job
    assert manager._rolling_prefetch_retry_at == {}
    assert manager._rolling_prefetch_failures == {}
    assert manager._rolling_prefetch_eligible_since == {}
    assert manager._rolling_prefetch_last_attempt_at == {}
    assert robot.rolling_boundary_since is None
    assert manager._last_async_job_kind == ""

    # Model the worker finishing after a benchmark reset. Its valid-looking
    # result must only release the worker slot, never mutate the fresh state.
    stale_job["done"] = True
    assert manager._finish_async_simulated_dispatch() == 0
    assert manager._dispatch_job is None
    assert manager._last_async_job_kind == ""
    assert robot.trajectory == []


def test_fresh_route_clears_prefetch_backoff_from_previous_route() -> None:
    manager = _line_manager(2)
    robot = FleetRobot(
        name="robot-1",
        current_lm="N0",
        pose=_pose(0),
    )
    order = FleetOrder(
        order_id="order-1",
        target_lm="N2",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="PLANNING",
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    manager._rolling_prefetch_retry_at[robot.name] = 123.0
    manager._rolling_prefetch_failures[robot.name] = 7
    manager._rolling_prefetch_eligible_since[robot.name] = 100.0
    manager._rolling_prefetch_last_attempt_at[robot.name] = 110.0
    robot.rolling_boundary_since = 90.0

    manager._apply_simulated_route_metadata(
        robot,
        order,
        {
            "startLm": "N0",
            "goalLm": "N1",
            "finalGoalLm": "N2",
            "nodes": ["N0", "N1"],
            "trajectory": _segment("N0", "N1"),
        },
        now=50.0,
    )

    assert robot.name not in manager._rolling_prefetch_retry_at
    assert robot.name not in manager._rolling_prefetch_failures
    assert robot.name not in manager._rolling_prefetch_eligible_since
    assert robot.name not in manager._rolling_prefetch_last_attempt_at
    assert robot.rolling_boundary_since is None
    assert robot.route_chunk_goal_lm == "N1"
    assert robot.route_final_lm == "N2"


def test_clear_robot_ephemeral_state_removes_all_name_keyed_traffic_state() -> None:
    manager = _line_manager(2)
    name = "robot-1"
    peer = "robot-2"
    manager._rolling_prefetch_retry_at.update({name: 10.0, peer: 11.0})
    manager._rolling_prefetch_failures.update({name: 2, peer: 3})
    manager._rolling_prefetch_eligible_since.update({name: 8.0, peer: 9.0})
    manager._rolling_prefetch_last_attempt_at.update({name: 9.0, peer: 10.0})
    manager._runtime_tick_route_clocks.update({name: 1.0, peer: 2.0})
    manager._active_wait_cycles[(name, peer)] = 1.0
    manager._wait_cycle_last_arbitration[(name, peer)] = 2.0
    manager._coupled_replan_last_attempt[(name, peer)] = 3.0
    manager._coupled_replan_failures[(name, peer)] = 4
    manager._controlled_corridor_wait_since[("corridor", name)] = 1.0
    manager._traffic_zone_wait_since[("zone", name)] = 1.0
    manager._controlled_corridor_winners.update(
        {name: "corridor", peer: "other-corridor"}
    )
    manager._traffic_zone_winners.update({name: "zone", peer: "other-zone"})
    manager._controlled_corridor_leases.update(
        {
            "corridor": (name, 100.0),
            "other-corridor": (peer, 100.0),
        }
    )
    manager._traffic_zone_leases.update(
        {
            ("zone", name): 100.0,
            ("other-zone", peer): 100.0,
        }
    )
    manager._controlled_corridor_occupancy["corridor"] = [name, peer]
    manager._controlled_corridor_queues["corridor"] = [name, peer]
    manager._traffic_zone_queues["zone"] = [name, peer]

    manager.clear_robot_ephemeral_state(name)

    assert name not in manager._rolling_prefetch_retry_at
    assert name not in manager._rolling_prefetch_failures
    assert name not in manager._rolling_prefetch_eligible_since
    assert name not in manager._rolling_prefetch_last_attempt_at
    assert name not in manager._runtime_tick_route_clocks
    assert all(name not in cycle for cycle in manager._active_wait_cycles)
    assert all(
        name not in cycle for cycle in manager._wait_cycle_last_arbitration
    )
    assert all(name not in cycle for cycle in manager._coupled_replan_last_attempt)
    assert all(name not in cycle for cycle in manager._coupled_replan_failures)
    assert all(key[1] != name for key in manager._controlled_corridor_wait_since)
    assert all(key[1] != name for key in manager._traffic_zone_wait_since)
    assert name not in manager._controlled_corridor_winners
    assert name not in manager._traffic_zone_winners
    assert all(owner != name for owner, _ in manager._controlled_corridor_leases.values())
    assert all(key[1] != name for key in manager._traffic_zone_leases)
    assert all(
        name not in robot_names
        for robot_names in manager._controlled_corridor_occupancy.values()
    )
    assert all(
        name not in robot_names
        for robot_names in manager._controlled_corridor_queues.values()
    )
    assert all(
        name not in robot_names
        for robot_names in manager._traffic_zone_queues.values()
    )

    # Unrelated robots retain their scheduling state.
    assert manager._rolling_prefetch_retry_at[peer] == 11.0
    assert manager._rolling_prefetch_eligible_since[peer] == 9.0
    assert manager._rolling_prefetch_last_attempt_at[peer] == 10.0
    assert manager._controlled_corridor_winners[peer] == "other-corridor"
    assert manager._traffic_zone_winners[peer] == "other-zone"
    assert manager._controlled_corridor_leases["other-corridor"][0] == peer
    assert ("other-zone", peer) in manager._traffic_zone_leases


def test_pending_by_robot_is_equivalent_to_individual_pending_queries() -> None:
    orders = {
        "late": FleetOrder(
            order_id="late",
            target_lm="N1",
            vehicle="robot-1",
            created_at=10.0,
        ),
        "early": FleetOrder(
            order_id="early",
            target_lm="N1",
            assigned_robot="robot-1",
            created_at=1.0,
        ),
        "same-owner": FleetOrder(
            order_id="same-owner",
            target_lm="N1",
            vehicle="robot-1",
            assigned_robot="robot-1",
            created_at=5.0,
        ),
        "two-owners": FleetOrder(
            order_id="two-owners",
            target_lm="N1",
            vehicle="robot-1",
            assigned_robot="robot-2",
            created_at=4.0,
        ),
        "terminal": FleetOrder(
            order_id="terminal",
            target_lm="N1",
            vehicle="robot-2",
            status="COMPLETED",
            created_at=0.0,
        ),
        "unassigned": FleetOrder(
            order_id="unassigned",
            target_lm="N1",
            created_at=2.0,
        ),
    }
    task_manager = FleetTaskManager(orders)

    pending_by_robot = task_manager.pending_by_robot()

    for robot_name in ("robot-1", "robot-2", "robot-3"):
        indexed_ids = [
            order.order_id
            for order in pending_by_robot.get(robot_name, [])
        ]
        queried_ids = [
            order.order_id
            for order in task_manager.pending_for_robot(robot_name)
        ]
        assert indexed_ids == queried_ids
    assert [
        order.order_id for order in pending_by_robot["robot-1"]
    ] == ["early", "two-owners", "same-owner", "late"]
    assert [
        order.order_id for order in pending_by_robot["robot-2"]
    ] == ["two-owners"]


def test_reservation_scans_skip_executed_trajectory_history(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = _line_manager(
        160,
        fleet={
            "reservation_horizon_sec": 10.0,
            "reservation_safety_time_sec": 0.35,
        },
    )
    trajectory = [
        {
            "t": float(index),
            "x": float(index),
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": f"N{max(0, index - 1)}->N{index}",
            "lm": f"N{index}",
        }
        for index in range(161)
    ]
    manager.robots["robot-1"] = FleetRobot(
        name="robot-1",
        current_lm="N140",
        target_lm="N160",
        status="MOVING",
        pose=_pose(140),
        trajectory=trajectory,
        route_clock=140.0,
    )
    parse_calls = 0
    original_parse = manager._parse_edge_id

    def counted_parse(edge_id: str) -> tuple[str, str] | None:
        nonlocal parse_calls
        parse_calls += 1
        return original_parse(edge_id)

    monkeypatch.setattr(manager, "_parse_edge_id", counted_parse)

    edge_intervals = manager._reserved_edge_intervals([])
    edge_parse_calls = parse_calls
    parse_calls = 0
    vertex_intervals = manager._reserved_vertex_intervals([])
    vertex_parse_calls = parse_calls

    assert edge_intervals
    assert vertex_intervals
    assert edge_parse_calls <= 20
    assert vertex_parse_calls <= 20
    assert all(src != "N0" for src, _, _, _, _ in edge_intervals)


def test_planner_lifelong_caches_evict_oldest_entries() -> None:
    manager = _line_manager(
        2,
        fleet={"traffic_graph_cache_max_entries": 3},
    )
    planner = manager.planner

    for speed_index in range(10):
        planner._traffic_graph(0.5 + (speed_index * 0.1))

    assert len(planner._traffic_graph_cache) == 3

    generic_cache: dict[int, str] = {}
    for index in range(10):
        planner._bounded_cache_store(generic_cache, index, str(index), 3)
    assert generic_cache == {7: "7", 8: "8", 9: "9"}
    planner._bounded_cache_store(generic_cache, 8, "updated", 3)
    assert generic_cache == {7: "7", 8: "updated", 9: "9"}


def test_task_store_prunes_old_terminal_history_without_touching_active_orders() -> None:
    orders = {
        f"terminal-{index}": FleetOrder(
            order_id=f"terminal-{index}",
            target_lm="N1",
            status="COMPLETED",
            updated_at=float(index),
        )
        for index in range(6)
    }
    active = FleetOrder(
        order_id="active",
        target_lm="N1",
        status="EXECUTING",
        updated_at=-1.0,
    )
    orders[active.order_id] = active
    task_manager = FleetTaskManager(orders)

    removed = task_manager.prune_terminal_history(2)

    assert set(removed) == {
        "terminal-0",
        "terminal-1",
        "terminal-2",
        "terminal-3",
    }
    assert set(task_manager.orders) == {
        "active",
        "terminal-4",
        "terminal-5",
    }


def test_new_order_prunes_history_and_cannot_inherit_reused_id_quarantine() -> None:
    manager = _line_manager(
        2,
        fleet={"terminal_order_history_limit": 2},
    )
    for index in range(4):
        order_id = f"old-{index}"
        manager.orders[order_id] = FleetOrder(
            order_id=order_id,
            target_lm="N1",
            status="COMPLETED",
            updated_at=float(index),
        )
        manager._stationary_order_retry_state[order_id] = {
            "failure_count": 99,
            "blocked_lms": ("N1",),
            "signature": (),
        }

    manager.set_order(
        {"id": "old-3", "targetLm": "N2"},
        dispatch=False,
    )

    assert set(manager.orders) == {"old-2", "old-3"}
    assert manager.orders["old-3"].status == "QUEUED"
    assert "old-3" not in manager._stationary_order_retry_state
    assert all(
        order_id in manager.orders
        for order_id in manager._stationary_order_retry_state
    )


def test_external_prefetch_blocker_escalates_every_affected_member() -> None:
    manager = _line_manager(3)
    entries = []
    revisions: dict[str, int] = {}
    for index, name in enumerate(("robot-1", "robot-2"), start=1):
        order = FleetOrder(
            order_id=f"order-{index}",
            target_lm="N3",
            vehicle=name,
            assigned_robot=name,
            status="PLANNING",
        )
        robot = FleetRobot(
            name=name,
            current_lm="N0",
            target_lm="N1",
            status="MOVING",
            pose=_pose(0),
            trajectory=_segment("N0", "N1"),
            active_order_id=order.order_id,
            route_revision=index,
            route_chunk_goal_lm="N1",
            route_final_lm="N3",
        )
        manager.orders[order.order_id] = order
        manager.robots[name] = robot
        entries.append(
            (
                order,
                robot,
                {"name": name, "startLm": "N1", "goalLm": "N2"},
                "N3",
                0.0,
            )
        )
        revisions[name] = index
    manager.robots["external-blocker"] = FleetRobot(
        name="external-blocker",
        current_lm="N2",
        pose=_pose(2),
    )

    manager._finish_async_rolling_prefetch({
        "kind": "prefetch_batch",
        "entries": entries,
        "route_revisions": revisions,
        "result": {
            "ok": False,
            "plans": [],
            "debug": {
                "reason": "no_low_level_path:external-blocker",
            },
        },
    })

    assert manager._rolling_prefetch_failures == {
        "robot-1": 1,
        "robot-2": 1,
    }


def _line_manager(
    edge_count: int,
    *,
    fleet: dict[str, Any] | None = None,
) -> FleetManagerSim:
    landmarks = {
        f"N{index}": Landmark(name=f"N{index}", x=float(index), y=0.0)
        for index in range(edge_count + 1)
    }
    edges = [
        GraphEdge(
            from_name=f"N{index}",
            to_name=f"N{index + 1}",
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(float(index), 0.0),
                WorldPoint(float(index + 1), 0.0),
            ),
            properties={"direction": 1},
        )
        for index in range(edge_count)
    ]
    return FleetManagerSim(
        landmarks,
        edges,
        params={
            "planner": {"on_route_tolerance": 0.1},
            "fleet": {
                "runtime_replan_lm_tolerance_m": 0.1,
                **(fleet or {}),
            },
        },
    )


def _segment(start_lm: str, goal_lm: str) -> list[dict[str, Any]]:
    start_index = int(start_lm.removeprefix("N"))
    goal_index = int(goal_lm.removeprefix("N"))
    return [
        {
            "t": 0.0,
            "x": float(start_index),
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": f"{start_lm}->{start_lm}",
            "lm": start_lm,
        },
        {
            "t": 1.0,
            "x": float(goal_index),
            "y": 0.0,
            "yaw": 0.0,
            "edgeId": f"{start_lm}->{goal_lm}",
            "lm": goal_lm,
        },
    ]


def _pose(index: int) -> dict[str, float]:
    return {"x": float(index), "y": 0.0, "yaw": 0.0}
