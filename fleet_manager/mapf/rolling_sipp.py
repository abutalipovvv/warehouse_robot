from __future__ import annotations

from typing import Callable

from .lm_cbs import LmRobotPlan, LmRobotRequest, PlannerDebug, PlannerResult
from .reservations import ReservationInterval, ReservationTable, ResourceId
from .sipp import SippPlanner, SippRobotRequest, TimedPath
from .traffic_graph import TrafficGraph


NodeName = str


class RollingSippPlanner:
    def __init__(
        self,
        graph: TrafficGraph,
        *,
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        low_level_max_time: int = 160,
        wait_cost: int = 6,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.wait_cost = max(1, int(wait_cost))

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

        reservations = ReservationTable(self.graph.reservation_capacities())
        self._apply_static_reservations(
            reservations,
            reserved_vertex_constraints or [],
            reserved_edge_constraints or [],
            reserved_vertex_intervals or [],
            reserved_edge_intervals or [],
        )

        sipp = SippPlanner(
            self.graph,
            heuristic_fn=self.heuristic_fn,
            move_cost_fn=self.move_cost_fn,
            low_level_max_time=self.low_level_max_time,
            wait_cost=self.wait_cost,
        )
        plans: dict[str, LmRobotPlan] = {}
        expanded_nodes = 0
        for request in robot_requests:
            reservations.release_robot_uncommitted(request.robot_name)
            path = sipp.plan(
                SippRobotRequest(
                    robot_name=request.robot_name,
                    start_lm=request.start_lm,
                    goal_lm=request.goal_lm,
                ),
                reservations,
                blocked_nodes=blocked_set,
                blocked_edges=blocked_edge_set,
            )
            expanded_nodes += sipp.expanded_nodes
            if path is None:
                debug.reason = sipp.last_failure or f"rolling_sipp:no_path:{request.robot_name}"
                debug.expanded_nodes = expanded_nodes
                return PlannerResult(plans={}, debug=debug)
            self._reserve_path(reservations, path)
            plans[request.robot_name] = LmRobotPlan(
                robot_name=request.robot_name,
                start_lm=request.start_lm,
                goal_lm=request.goal_lm,
                nodes=path.nodes,
                times=path.times,
            )

        debug.reason = "rolling_sipp:success"
        debug.expanded_nodes = expanded_nodes
        debug.high_level_nodes = len(robot_requests)
        return PlannerResult(plans=plans, debug=debug)

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
