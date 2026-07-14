import math

from fleet_manager.mapf import LmCBSPlanner, LmRobotRequest


def test_fleet_mapf_is_owned_by_fleet_manager() -> None:
    from fleet_manager.mapf.lm_cbs import LmCBSPlanner as LocalPlanner

    assert LocalPlanner is LmCBSPlanner
    assert LmCBSPlanner.__module__.startswith("fleet_manager.mapf")


def test_two_robots_use_passing_bay_to_resolve_head_on_conflict() -> None:
    graph = {
        "A": ["B"],
        "B": ["A", "C", "D"],
        "C": ["B"],
        "D": ["B"],
    }
    planner = LmCBSPlanner(graph, low_level_max_time=12, max_high_level_nodes=1000)

    result = planner.plan_for_robots(
        [
            LmRobotRequest("r1", "A", "C"),
            LmRobotRequest("r2", "C", "A"),
        ]
    )

    assert result.debug.reason == "success"
    assert set(result.plans) == {"r1", "r2"}
    assert result.plans["r1"].nodes[0] == "A"
    assert result.plans["r1"].nodes[-1] == "C"
    assert result.plans["r2"].nodes[0] == "C"
    assert result.plans["r2"].nodes[-1] == "A"
    assert "D" in result.plans["r1"].nodes or "D" in result.plans["r2"].nodes
    assert_no_space_time_conflicts(result.plans)


def test_vertex_reservation_interval_inserts_waits() -> None:
    planner = LmCBSPlanner(
        {"A": ["B"], "B": ["C"], "C": []},
        low_level_max_time=10,
        max_high_level_nodes=100,
    )

    result = planner.plan_for_robots(
        [LmRobotRequest("r1", "A", "C")],
        reserved_vertex_intervals=[(1, 2, "B", "other")],
    )

    assert result.debug.reason == "success"
    assert result.plans["r1"].nodes == ["A", "A", "A", "B", "C"]
    assert result.plans["r1"].times == [0, 1, 2, 3, 4]


def test_edge_reservation_interval_inserts_waits() -> None:
    planner = LmCBSPlanner(
        {"A": ["B"], "B": ["C"], "C": []},
        low_level_max_time=10,
        max_high_level_nodes=100,
    )

    result = planner.plan_for_robots(
        [LmRobotRequest("r1", "A", "C")],
        reserved_edge_intervals=[(0, 2, "A", "B", "other")],
    )

    assert result.debug.reason == "success"
    assert result.plans["r1"].nodes == ["A", "A", "A", "A", "B", "C"]
    assert result.plans["r1"].times == [0, 1, 2, 3, 4, 5]


def test_shared_goal_is_explicitly_rejected() -> None:
    planner = LmCBSPlanner(
        {"A": ["C"], "B": ["C"], "C": []},
        low_level_max_time=8,
        max_high_level_nodes=100,
    )

    result = planner.plan_for_robots(
        [
            LmRobotRequest("r1", "A", "C"),
            LmRobotRequest("r2", "B", "C"),
        ]
    )

    assert result.plans == {}
    assert result.debug.reason == "shared_goal_not_supported:r1,r2@C"


def test_low_level_horizon_rejects_late_goal() -> None:
    planner = LmCBSPlanner(
        {"A": ["B"], "B": []},
        move_cost_fn=lambda _start, _goal: 20,
        low_level_max_time=5,
        max_high_level_nodes=100,
    )

    result = planner.plan_for_robots([LmRobotRequest("r1", "A", "B")])

    assert result.plans == {}
    assert result.debug.reason.startswith("no_low_level_path")


def test_cbs_reserves_rotation_before_entering_an_edge() -> None:
    planner = LmCBSPlanner(
        {"A": ["B"], "B": []},
        move_cost_fn=lambda _start, _goal: 1,
        heading_fn=lambda _start, _goal: 0.0,
        turn_cost_fn=lambda start, goal: 4 if abs(start - goal) > 1.0 else 0,
        low_level_max_time=10,
        max_high_level_nodes=100,
    )

    result = planner.plan_for_robots(
        [LmRobotRequest("r1", "A", "B", start_yaw=math.pi)]
    )

    plan = result.plans["r1"]
    assert plan.nodes == ["A", "A", "B"]
    assert plan.times == [0, 4, 5]
    assert plan.actions == ["start", "rotate", "move"]


def test_cbs_constrains_shared_topometric_resource_directly() -> None:
    planner = LmCBSPlanner(
        {"A": ["B"], "B": [], "C": ["D"], "D": []},
        vertex_resources_fn=lambda node: (f"vertex:{node}",),
        lane_resources_fn=lambda _start, _goal: ("mutex:crossing",),
        low_level_max_time=10,
        max_high_level_nodes=100,
    )

    result = planner.plan_for_robots(
        [
            LmRobotRequest("r1", "A", "B"),
            LmRobotRequest("r2", "C", "D"),
        ]
    )

    assert result.debug.reason == "success"
    assert result.debug.conflicts_resolved == 1
    assert sorted(plan.times[-1] for plan in result.plans.values()) == [1, 2]


def assert_no_space_time_conflicts(plans) -> None:
    paths = {
        name: list(zip(plan.times, plan.nodes))
        for name, plan in plans.items()
    }
    horizon = max(times[-1] for times in (plan.times for plan in plans.values()))

    for time_tick in range(horizon + 1):
        occupied: dict[str, str] = {}
        for robot_name, path in paths.items():
            node = state_at(path, time_tick)
            assert node not in occupied, (
                f"{robot_name} and {occupied[node]} occupy {node} at {time_tick}"
            )
            occupied[node] = robot_name

    names = sorted(paths)
    for first_index, first_name in enumerate(names):
        for second_name in names[first_index + 1:]:
            first_path = paths[first_name]
            second_path = paths[second_name]
            for time_tick in range(horizon):
                first_edge = (state_at(first_path, time_tick), state_at(first_path, time_tick + 1))
                second_edge = (state_at(second_path, time_tick), state_at(second_path, time_tick + 1))
                assert first_edge != second_edge[::-1], (
                    f"{first_name} and {second_name} swap {first_edge} at {time_tick}"
                )


def state_at(path: list[tuple[int, str]], time_tick: int) -> str:
    current = path[0][1]
    for state_time, node in path:
        if state_time > time_tick:
            return current
        current = node
    return current
