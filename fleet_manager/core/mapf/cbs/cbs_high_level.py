"""High-level Conflict-Based Search tree and public planner."""

from __future__ import annotations

import time as py_time
from typing import Callable

from .cbs_low_level import LmCBSEnvironment
from .cbs_models import (
    Constraints,
    HighLevelNode,
    LmRobotPlan,
    LmRobotRequest,
    NodeName,
    PlannerDebug,
    PlannerResult,
    State,
    VertexConstraint,
    VertexIntervalConstraint,
)
from .cbs_setup import CbsGlobalReservations, CbsPlanningLimits
from .cbs_tree import CbsHighLevelSearch


class LmCBSPlanner:
    """Plan a conflict-free set of landmark routes with CBS."""

    def __init__(
        self,
        graph: dict[NodeName, list[NodeName]],
        heuristic_fn: (
            Callable[[NodeName, NodeName], float] | None
        ) = None,
        move_cost_fn: (
            Callable[[NodeName, NodeName], int] | None
        ) = None,
        heading_fn: (
            Callable[[NodeName, NodeName], float] | None
        ) = None,
        heading_options_fn: (
            Callable[
                [NodeName, NodeName],
                tuple[float, ...],
            ]
            | None
        ) = None,
        turn_cost_fn: (
            Callable[[float, float], int] | None
        ) = None,
        vertex_resources_fn: (
            Callable[[NodeName], tuple[object, ...]] | None
        ) = None,
        rotation_resources_fn: (
            Callable[[NodeName], tuple[object, ...]] | None
        ) = None,
        lane_resources_fn: (
            Callable[
                [NodeName, NodeName],
                tuple[object, ...],
            ]
            | None
        ) = None,
        can_wait_fn: (
            Callable[[NodeName], bool] | None
        ) = None,
        low_level_max_time: int = 128,
        max_high_level_nodes: int = 2000,
        max_planning_time_sec: float = 5.0,
        wait_cost: int = 6,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.heading_fn = heading_fn
        self.heading_options_fn = heading_options_fn
        self.turn_cost_fn = turn_cost_fn
        self.vertex_resources_fn = vertex_resources_fn
        self.rotation_resources_fn = (
            rotation_resources_fn or vertex_resources_fn
        )
        self.lane_resources_fn = lane_resources_fn
        self.can_wait_fn = can_wait_fn
        self.low_level_max_time = max(
            1,
            int(low_level_max_time),
        )
        self.max_high_level_nodes = max(
            1,
            int(max_high_level_nodes),
        )
        self.max_planning_time_sec = max(
            0.0,
            float(max_planning_time_sec),
        )
        self.wait_cost = max(1, int(wait_cost))

    def plan_for_robots(
        self,
        robot_requests: list[LmRobotRequest],
        blocked_nodes: list[NodeName] | None = None,
        reserved_vertex_constraints: (
            list[tuple[int, NodeName]] | None
        ) = None,
        reserved_edge_constraints: (
            list[tuple[int, NodeName, NodeName]] | None
        ) = None,
        reserved_vertex_intervals: (
            list[tuple[int, int, NodeName, str]] | None
        ) = None,
        reserved_edge_intervals: (
            list[
                tuple[
                    int,
                    int,
                    NodeName,
                    NodeName,
                    str,
                ]
            ]
            | None
        ) = None,
        reserved_resource_intervals: (
            list[tuple[int, int, object]] | None
        ) = None,
        move_cost_fn: (
            Callable[[NodeName, NodeName], int] | None
        ) = None,
        low_level_max_time: int | None = None,
        max_high_level_nodes: int | None = None,
        max_planning_time_sec: float | None = None,
        should_cancel: Callable[[], bool] | None = None,
    ) -> PlannerResult:
        debug = PlannerDebug(reason="init")
        limits = self._planning_limits(
            low_level_max_time=low_level_max_time,
            max_high_level_nodes=max_high_level_nodes,
            max_planning_time_sec=max_planning_time_sec,
            should_cancel=should_cancel,
        )
        reservations = CbsGlobalReservations.from_raw(
            vertices=reserved_vertex_constraints or (),
            edges=reserved_edge_constraints or (),
            vertex_intervals=reserved_vertex_intervals or (),
            edge_intervals=reserved_edge_intervals or (),
            resource_intervals=reserved_resource_intervals or (),
        )

        if not robot_requests:
            debug.reason = "empty_requests"
            return PlannerResult(plans={}, debug=debug)

        blocked_set = set(blocked_nodes or ())
        request_error = self._request_error(
            robot_requests,
            blocked_set,
            set(reservations.vertices),
            list(reservations.vertex_intervals),
        )
        if request_error:
            debug.reason = request_error
            return PlannerResult(plans={}, debug=debug)

        environment = self._environment(
            robot_requests,
            blocked_nodes=blocked_set,
            reservations=reservations,
            move_cost_fn=move_cost_fn,
            limits=limits,
        )
        start = self._initial_node(robot_requests, environment)
        if not start.solution:
            debug.reason = self._initial_failure_reason(
                environment,
                limits,
            )
            return PlannerResult(plans={}, debug=debug)

        tree_result = CbsHighLevelSearch(
            environment,
            limits,
        ).solve(start)
        debug.reason = tree_result.reason
        debug.conflicts_resolved = tree_result.conflicts_resolved
        debug.high_level_nodes = tree_result.expanded_nodes
        if tree_result.solution_node is None:
            return PlannerResult(plans={}, debug=debug)
        return self._success_result(
            robot_requests,
            tree_result.solution_node,
            environment,
            debug,
            tree_result.conflicts_resolved,
            tree_result.expanded_nodes,
        )

    def _planning_limits(
        self,
        *,
        low_level_max_time: int | None,
        max_high_level_nodes: int | None,
        max_planning_time_sec: float | None,
        should_cancel: Callable[[], bool] | None,
    ) -> CbsPlanningLimits:
        low_level_limit = (
            self.low_level_max_time
            if low_level_max_time is None
            else max(1, int(low_level_max_time))
        )
        high_level_limit = (
            self.max_high_level_nodes
            if max_high_level_nodes is None
            else max(1, int(max_high_level_nodes))
        )
        planning_budget = (
            self.max_planning_time_sec
            if max_planning_time_sec is None
            else max(0.0, float(max_planning_time_sec))
        )
        started_at = py_time.monotonic()
        return CbsPlanningLimits(
            low_level_max_time=low_level_limit,
            high_level_max_nodes=high_level_limit,
            time_budget_seconds=planning_budget,
            started_at=started_at,
            clock=py_time.monotonic,
            should_cancel=should_cancel,
        )

    def _environment(
        self,
        robot_requests: list[LmRobotRequest],
        *,
        blocked_nodes: set[NodeName],
        reservations: CbsGlobalReservations,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None,
        limits: CbsPlanningLimits,
    ) -> LmCBSEnvironment:
        return LmCBSEnvironment(
            self.graph,
            robot_requests,
            blocked_nodes=blocked_nodes,
            global_vertex_constraints=set(reservations.vertices),
            global_edge_constraints=set(reservations.edges),
            global_vertex_intervals=list(reservations.vertex_intervals),
            global_edge_intervals=list(reservations.edge_intervals),
            global_resource_intervals=list(
                reservations.resource_intervals
            ),
            heuristic_fn=self.heuristic_fn,
            move_cost_fn=move_cost_fn or self.move_cost_fn,
            heading_fn=self.heading_fn,
            heading_options_fn=self.heading_options_fn,
            turn_cost_fn=self.turn_cost_fn,
            vertex_resources_fn=self.vertex_resources_fn,
            rotation_resources_fn=self.rotation_resources_fn,
            lane_resources_fn=self.lane_resources_fn,
            can_wait_fn=self.can_wait_fn,
            low_level_max_time=limits.low_level_max_time,
            wait_cost=self.wait_cost,
            planning_deadline=limits.deadline,
            should_cancel=limits.should_cancel,
        )

    @staticmethod
    def _initial_node(
        robot_requests: list[LmRobotRequest],
        environment: LmCBSEnvironment,
    ) -> HighLevelNode:
        start = HighLevelNode(
            constraint_dict={
                request.robot_name: Constraints()
                for request in robot_requests
            }
        )
        environment.constraint_dict = start.constraint_dict
        start.solution = environment.compute_solution() or {}
        if start.solution:
            start.cost = environment.compute_solution_cost(start.solution)
        return start

    @staticmethod
    def _initial_failure_reason(
        environment: LmCBSEnvironment,
        limits: CbsPlanningLimits,
    ) -> str:
        if environment.last_failure == "planning_timeout":
            return limits.timeout_reason
        return environment.last_failure or "initial_solution_failed"

    def _request_error(
        self,
        requests: list[LmRobotRequest],
        blocked_nodes: set[NodeName],
        global_vertex_constraints: set[VertexConstraint],
        global_vertex_intervals: list[
            VertexIntervalConstraint
        ],
    ) -> str:
        seen_goals: dict[NodeName, str] = {}
        for request in requests:
            if request.start_lm not in self.graph:
                return (
                    f"unknown_start:{request.robot_name}:"
                    f"{request.start_lm}"
                )
            if request.goal_lm not in self.graph:
                return (
                    f"unknown_goal:{request.robot_name}:"
                    f"{request.goal_lm}"
                )
            if request.start_lm in blocked_nodes:
                return f"start_blocked:{request.robot_name}"
            if request.goal_lm in blocked_nodes:
                return f"goal_blocked:{request.robot_name}"
            if (
                VertexConstraint(0, request.start_lm)
                in global_vertex_constraints
            ):
                return f"start_reserved:{request.robot_name}"
            for interval in global_vertex_intervals:
                if (
                    interval.node == request.start_lm
                    and interval.start_time
                    <= 0
                    <= interval.end_time
                ):
                    owner = (
                        f":{interval.owner}"
                        if interval.owner
                        else ""
                    )
                    return (
                        f"start_reserved_interval:"
                        f"{request.robot_name}{owner}"
                    )
            if request.goal_lm in seen_goals:
                return (
                    f"shared_goal_not_supported:"
                    f"{seen_goals[request.goal_lm]},"
                    f"{request.robot_name}"
                    f"@{request.goal_lm}"
                )
            seen_goals[request.goal_lm] = (
                request.robot_name
            )
        return ""

    def _success_result(
        self,
        requests: list[LmRobotRequest],
        node: HighLevelNode,
        environment: LmCBSEnvironment,
        debug: PlannerDebug,
        conflicts_resolved: int,
        high_level_nodes: int,
    ) -> PlannerResult:
        plans: dict[str, LmRobotPlan] = {}
        total_nodes = 0
        for request in requests:
            states = node.solution[request.robot_name]
            expanded = self._expand_kinematic_states(
                environment,
                states,
            )
            nodes = [
                state.node
                for state, _action in expanded
            ]
            total_nodes += len(nodes)
            plans[request.robot_name] = LmRobotPlan(
                robot_name=request.robot_name,
                start_lm=request.start_lm,
                goal_lm=request.goal_lm,
                nodes=nodes,
                times=[
                    state.time
                    for state, _action in expanded
                ],
                yaws=[
                    state.yaw
                    for state, _action in expanded
                ],
                actions=[
                    action
                    for _state, action in expanded
                ],
            )
        debug.reason = "success"
        debug.conflicts_resolved = conflicts_resolved
        debug.high_level_nodes = high_level_nodes
        debug.expanded_nodes = total_nodes
        return PlannerResult(plans=plans, debug=debug)

    def _expand_kinematic_states(
        self,
        env: LmCBSEnvironment,
        states: list[State],
    ) -> list[tuple[State, str]]:
        if not states:
            return []

        expanded: list[tuple[State, str]] = [
            (states[0], "start")
        ]
        for start, end in zip(states, states[1:]):
            if start.node == end.node:
                expanded.append((end, "wait"))
                continue
            turn_ticks, _ = env.transition_parts(
                start,
                end,
            )
            if turn_ticks:
                expanded.append(
                    (
                        State(
                            start.time + turn_ticks,
                            start.node,
                            end.yaw,
                        ),
                        "rotate",
                    )
                )
            expanded.append((end, "move"))
        return expanded
