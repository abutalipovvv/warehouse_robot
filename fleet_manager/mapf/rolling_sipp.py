from __future__ import annotations

import re
from typing import Callable

from .lm_cbs import LmRobotPlan, LmRobotRequest, PlannerDebug, PlannerResult
from .reservations import ReservationInterval, ReservationTable, ResourceId
from .sipp import SippPlanner, SippRobotRequest, TimedPath, TimedState
from .traffic_graph import TrafficGraph


NodeName = str


class RollingSippPlanner:
    def __init__(
        self,
        graph: TrafficGraph,
        *,
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        heading_fn: Callable[[NodeName, NodeName], float] | None = None,
        turn_cost_fn: Callable[[float, float], int] | None = None,
        low_level_max_time: int = 160,
        wait_cost: int = 6,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.heading_fn = heading_fn
        self.turn_cost_fn = turn_cost_fn
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.wait_cost = max(1, int(wait_cost))

    def validate_plans(
        self,
        robot_requests: list[LmRobotRequest],
        plans: dict[str, LmRobotPlan],
        *,
        reserved_vertex_constraints: list[tuple[int, NodeName]] | None = None,
        reserved_edge_constraints: list[tuple[int, NodeName, NodeName]] | None = None,
        reserved_vertex_intervals: list[tuple[int, int, NodeName, str]] | None = None,
        reserved_edge_intervals: list[tuple[int, int, NodeName, NodeName, str]] | None = None,
    ) -> str:
        reservations = ReservationTable(self.graph.reservation_capacities())
        self._apply_static_reservations(
            reservations,
            reserved_vertex_constraints or [],
            reserved_edge_constraints or [],
            reserved_vertex_intervals or [],
            reserved_edge_intervals or [],
        )
        self._reserve_initial_starts(reservations, robot_requests)
        for request in robot_requests:
            plan = plans.get(request.robot_name)
            if plan is None:
                return f"missing_plan:{request.robot_name}"
            path = self._timed_path(plan)
            if not self._path_is_free(reservations, path):
                return f"resource_conflict:{request.robot_name}"
            self._reserve_path(reservations, path)
        return ""

    def plan_for_robots(
        self,
        robot_requests: list[LmRobotRequest],
        *,
        blocked_nodes: list[NodeName] | None = None,
        blocked_edges: set[tuple[NodeName, NodeName]] | None = None,
        reserved_vertex_constraints: list[tuple[int, NodeName]] | None = None,
        reserved_edge_constraints: list[tuple[int, NodeName, NodeName]] | None = None,
        reserved_vertex_intervals: list[tuple[int, int, NodeName, str]] | None = None,
        reserved_edge_intervals: list[tuple[int, int, NodeName, NodeName, str]] | None = None,
    ) -> PlannerResult:
        debug = PlannerDebug(reason="rolling_sipp:init")
        blocked_set = set(blocked_nodes or [])
        blocked_edge_set = set(blocked_edges or set())
        if not robot_requests:
            debug.reason = "rolling_sipp:empty_requests"
            return PlannerResult(plans={}, debug=debug)

        seen_goals: dict[NodeName, str] = {}
        for request in robot_requests:
            if request.start_lm not in self.graph.vertices:
                debug.reason = f"rolling_sipp:unknown_start:{request.robot_name}:{request.start_lm}"
                return PlannerResult(plans={}, debug=debug)
            if request.goal_lm not in self.graph.vertices:
                debug.reason = f"rolling_sipp:unknown_goal:{request.robot_name}:{request.goal_lm}"
                return PlannerResult(plans={}, debug=debug)
            if request.start_lm in blocked_set:
                debug.reason = f"rolling_sipp:start_blocked:{request.robot_name}"
                return PlannerResult(plans={}, debug=debug)
            if request.goal_lm in blocked_set:
                debug.reason = f"rolling_sipp:goal_blocked:{request.robot_name}"
                return PlannerResult(plans={}, debug=debug)
            if request.goal_lm in seen_goals:
                debug.reason = (
                    f"rolling_sipp:shared_goal_not_supported:"
                    f"{seen_goals[request.goal_lm]},{request.robot_name}@{request.goal_lm}"
                )
                return PlannerResult(plans={}, debug=debug)
            seen_goals[request.goal_lm] = request.robot_name

        sipp = SippPlanner(
            self.graph,
            heuristic_fn=self.heuristic_fn,
            move_cost_fn=self.move_cost_fn,
            heading_fn=self.heading_fn,
            turn_cost_fn=self.turn_cost_fn,
            low_level_max_time=self.low_level_max_time,
            wait_cost=self.wait_cost,
        )
        expanded_nodes = 0
        priority_repairs = 0
        ordered_requests = list(robot_requests)
        seen_orders = {tuple(request.robot_name for request in ordered_requests)}
        max_priority_repairs = max(2, min(32, len(robot_requests) * 2))

        while True:
            reservations = ReservationTable(self.graph.reservation_capacities())
            self._apply_static_reservations(
                reservations,
                reserved_vertex_constraints or [],
                reserved_edge_constraints or [],
                reserved_vertex_intervals or [],
                reserved_edge_intervals or [],
            )
            self._reserve_initial_starts(reservations, robot_requests)
            plans: dict[str, LmRobotPlan] = {}
            retry_with_new_order = False
            for request in ordered_requests:
                path = sipp.plan(
                    SippRobotRequest(
                        robot_name=request.robot_name,
                        start_lm=request.start_lm,
                        goal_lm=request.goal_lm,
                        start_yaw=request.start_yaw,
                        route_nodes=request.route_nodes,
                    ),
                    reservations,
                    blocked_nodes=blocked_set,
                    blocked_edges=blocked_edge_set,
                )
                expanded_nodes += sipp.expanded_nodes
                if path is None:
                    failure = sipp.last_failure or f"rolling_sipp:no_path:{request.robot_name}"
                    blockers = self._blocking_plan_owners(
                        failure,
                        reservations,
                        planned_names=set(plans),
                    )
                    if not blockers:
                        blockers = sipp.blocking_robot_names & set(plans)
                    next_order = self._promote_before_blockers(
                        ordered_requests,
                        request.robot_name,
                        blockers,
                    )
                    order_key = tuple(item.robot_name for item in next_order)
                    if (
                        blockers
                        and priority_repairs < max_priority_repairs
                        and order_key not in seen_orders
                    ):
                        seen_orders.add(order_key)
                        ordered_requests = next_order
                        priority_repairs += 1
                        retry_with_new_order = True
                        break
                    debug.reason = failure
                    if blockers and order_key in seen_orders:
                        debug.reason = f"{failure}:priority_cycle"
                    elif blockers and priority_repairs >= max_priority_repairs:
                        debug.reason = f"{failure}:priority_repair_limit"
                    debug.conflicts_resolved = priority_repairs
                    debug.expanded_nodes = expanded_nodes
                    debug.high_level_nodes = len(seen_orders)
                    return PlannerResult(plans={}, debug=debug)
                self._reserve_path(reservations, path)
                plans[request.robot_name] = LmRobotPlan(
                    robot_name=request.robot_name,
                    start_lm=request.start_lm,
                    goal_lm=request.goal_lm,
                    nodes=path.nodes,
                    times=path.times,
                    yaws=path.yaws,
                    actions=path.actions,
                )

            if retry_with_new_order:
                continue
            break

        debug.reason = "rolling_sipp:success"
        if priority_repairs:
            debug.reason = f"{debug.reason}:priority_repairs={priority_repairs}"
        debug.conflicts_resolved = priority_repairs
        debug.expanded_nodes = expanded_nodes
        debug.high_level_nodes = len(seen_orders)
        return PlannerResult(plans=plans, debug=debug)

    def _reserve_initial_starts(
        self,
        reservations: ReservationTable,
        requests: list[LmRobotRequest],
    ) -> None:
        for request in requests:
            self._reserve_vertex(
                reservations,
                request.start_lm,
                0,
                1,
                request.robot_name,
                "initial_position",
            )

    def _timed_path(self, plan: LmRobotPlan) -> TimedPath:
        return TimedPath(
            robot_name=plan.robot_name,
            start_lm=plan.start_lm,
            goal_lm=plan.goal_lm,
            states=tuple(
                TimedState(
                    int(time_tick),
                    node,
                    float(plan.yaws[index]) if index < len(plan.yaws) else 0.0,
                    str(plan.actions[index]) if index < len(plan.actions) else "wait",
                )
                for index, (time_tick, node) in enumerate(zip(plan.times, plan.nodes))
            ),
        )

    def _path_is_free(self, reservations: ReservationTable, path: TimedPath) -> bool:
        states = list(path.states)
        if not states:
            return False
        for state in states:
            if not reservations.resources_are_free(
                self.graph.vertex_resources(state.node),
                state.time,
                state.time + 1,
                ignore_robot_name=path.robot_name,
            ):
                return False
        for start, end in zip(states, states[1:]):
            if start.node == end.node:
                resources = (
                    self.graph.rotation_resources(start.node)
                    if end.action == "rotate"
                    else self.graph.vertex_resources(start.node)
                )
                interval_end = end.time + 1
            else:
                lane = self.graph.lane_for(start.node, end.node)
                resources = (
                    self.graph.lane_resources(lane)
                    if lane is not None
                    else (ResourceId("lane", f"{start.node}->{end.node}"),)
                )
                interval_end = end.time
            if not reservations.resources_are_free(
                resources,
                start.time,
                interval_end,
                ignore_robot_name=path.robot_name,
            ):
                return False
        final = states[-1]
        return reservations.resources_are_free(
            self.graph.vertex_resources(final.node),
            final.time,
            self.low_level_max_time + 1,
            ignore_robot_name=path.robot_name,
        )

    def _blocking_plan_owners(
        self,
        failure: str,
        reservations: ReservationTable,
        *,
        planned_names: set[str],
    ) -> set[str]:
        resources: tuple[ResourceId, ...] = ()
        start = 0
        end = self.low_level_max_time + 1

        edge_match = re.search(r"reserved_edge:([^@]+)->([^@]+)@(\d+)-(\d+)", failure)
        if edge_match:
            src, dst = edge_match.group(1), edge_match.group(2)
            lane = self.graph.lane_for(src, dst)
            resources = (
                self.graph.lane_resources(lane)
                if lane is not None
                else (ResourceId("lane", f"{src}->{dst}"),)
            )
            start = int(edge_match.group(3))
            end = max(start + 1, int(edge_match.group(4)) + 1)
        else:
            vertex_match = re.search(r"reserved_lm:([^@]+)@(\d+)", failure)
            if vertex_match:
                resources = self.graph.vertex_resources(vertex_match.group(1))
                start = int(vertex_match.group(2))
                end = start + 1

        owners: set[str] = set()
        for resource in resources:
            for interval in reservations.conflicts(resource, start, end):
                if interval.robot_name in planned_names:
                    owners.add(interval.robot_name)
        return owners

    def _promote_before_blockers(
        self,
        requests: list[LmRobotRequest],
        robot_name: str,
        blockers: set[str],
    ) -> list[LmRobotRequest]:
        if not blockers:
            return list(requests)
        indices = {
            request.robot_name: index
            for index, request in enumerate(requests)
        }
        robot_index = indices.get(robot_name)
        blocker_indices = [indices[name] for name in blockers if name in indices]
        if robot_index is None or not blocker_indices:
            return list(requests)
        insert_at = min(blocker_indices)
        if insert_at >= robot_index:
            return list(requests)
        promoted = list(requests)
        request = promoted.pop(robot_index)
        promoted.insert(insert_at, request)
        return promoted

    def _apply_static_reservations(
        self,
        reservations: ReservationTable,
        vertex_constraints: list[tuple[int, NodeName]],
        edge_constraints: list[tuple[int, NodeName, NodeName]],
        vertex_intervals: list[tuple[int, int, NodeName, str]],
        edge_intervals: list[tuple[int, int, NodeName, NodeName, str]],
    ) -> None:
        for time_tick, node in vertex_constraints:
            self._reserve_vertex(reservations, node, time_tick, time_tick + 1, "reserved", "constraint")
        for time_tick, src, dst in edge_constraints:
            self._reserve_lane(reservations, src, dst, time_tick, time_tick + 1, "reserved", "constraint")
        for start, end, node, owner in vertex_intervals:
            self._reserve_vertex(reservations, node, start, end + 1, owner or "reserved", "reserved")
        for start, end, src, dst, owner in edge_intervals:
            self._reserve_lane(reservations, src, dst, start, end + 1, owner or "reserved", "reserved")

    def _reserve_path(self, reservations: ReservationTable, path: TimedPath) -> None:
        states = list(path.states)
        for state in states:
            self._reserve_vertex(
                reservations,
                state.node,
                state.time,
                state.time + 1,
                path.robot_name,
                "visit",
                committed=False,
            )
        for index in range(len(states) - 1):
            start = states[index]
            end = states[index + 1]
            if start.node == end.node:
                if end.action == "rotate":
                    self._reserve_rotation(
                        reservations,
                        start.node,
                        start.time,
                        end.time + 1,
                        path.robot_name,
                        committed=False,
                    )
                else:
                    self._reserve_vertex(
                        reservations,
                        start.node,
                        start.time,
                        end.time + 1,
                        path.robot_name,
                        "wait",
                        committed=False,
                    )
                continue
            self._reserve_lane(
                reservations,
                start.node,
                end.node,
                start.time,
                end.time,
                path.robot_name,
                "move",
                committed=False,
            )
        final = states[-1]
        self._reserve_vertex(
            reservations,
            final.node,
            final.time,
            self.low_level_max_time + 1,
            path.robot_name,
            "goal",
            committed=False,
        )

    def _reserve_vertex(
        self,
        reservations: ReservationTable,
        node: NodeName,
        start: int,
        end: int,
        robot_name: str,
        reason: str,
        *,
        committed: bool = True,
    ) -> None:
        for resource in self.graph.vertex_resources(node):
            reservations.reserve(
                ReservationInterval(
                    resource=resource,
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason=reason,
                    committed=committed,
                )
            )

    def _reserve_rotation(
        self,
        reservations: ReservationTable,
        node: NodeName,
        start: int,
        end: int,
        robot_name: str,
        *,
        committed: bool = True,
    ) -> None:
        for resource in self.graph.rotation_resources(node):
            reservations.reserve(
                ReservationInterval(
                    resource=resource,
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason="rotate",
                    committed=committed,
                )
            )

    def _reserve_lane(
        self,
        reservations: ReservationTable,
        src: NodeName,
        dst: NodeName,
        start: int,
        end: int,
        robot_name: str,
        reason: str,
        *,
        committed: bool = True,
    ) -> None:
        lane = self.graph.lane_for(src, dst)
        if lane is None:
            reservations.reserve(
                ReservationInterval(
                    resource=ResourceId("lane", f"{src}->{dst}"),
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason=reason,
                    committed=committed,
                )
            )
            return
        for resource in self.graph.lane_resources(lane):
            reservations.reserve(
                ReservationInterval(
                    resource=resource,
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason=reason,
                    committed=committed,
                )
            )
