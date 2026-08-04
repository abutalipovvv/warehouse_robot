"""SIPP planner built on the shared deterministic A* search loop."""

from __future__ import annotations

from typing import Callable

from fleet_manager.core.search.astar import AStarSolver

from ..common.reservations import ReservationTable, ResourceId
from .sipp_models import (
    NodeName,
    SippRobotRequest,
    SippState,
    TimedPath,
)
from .sipp_problem import SippSearchProblem
from ..graph.traffic_graph_models import TrafficGraph

class SippPlanner:
    """Plan one robot through safe intervals."""

    def __init__(
        self,
        graph: TrafficGraph,
        *,
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        heading_fn: Callable[[NodeName, NodeName], float] | None = None,
        heading_options_fn: (
            Callable[[NodeName, NodeName], tuple[float, ...]] | None
        ) = None,
        turn_cost_fn: Callable[[float, float], int] | None = None,
        low_level_max_time: int = 160,
        wait_cost: int = 6,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = (
            heuristic_fn
            or (lambda _node, _goal: 0.0)
        )
        self.move_cost_fn = (
            move_cost_fn
            or (lambda _source, _target: 1)
        )
        self.heading_fn = (
            heading_fn
            or (lambda _source, _target: 0.0)
        )
        self.heading_options_fn = heading_options_fn
        self.turn_cost_fn = (
            turn_cost_fn
            or (lambda _from_yaw, _to_yaw: 0)
        )
        self.low_level_max_time = max(
            1,
            int(low_level_max_time),
        )
        self.wait_cost = max(1, int(wait_cost))

        self.expanded_nodes = 0
        self.last_failure = ""
        self.blocking_robot_names: set[str] = set()
        self.blocking_resources_by_robot: dict[
            str,
            set[ResourceId],
        ] = {}

    def plan(
        self,
        request: SippRobotRequest,
        reservations: ReservationTable,
        *,
        blocked_nodes: set[NodeName] | None = None,
        blocked_edges: set[tuple[NodeName, NodeName]] | None = None,
        planning_deadline: float | None = None,
    ) -> TimedPath | None:
        problem = SippSearchProblem(
            self.graph,
            request,
            reservations,
            blocked_nodes=blocked_nodes or set(),
            blocked_edges=blocked_edges or set(),
            heuristic_fn=self.heuristic_fn,
            move_cost_fn=self.move_cost_fn,
            heading_fn=self.heading_fn,
            heading_options_fn=self.heading_options_fn,
            turn_cost_fn=self.turn_cost_fn,
            low_level_max_time=self.low_level_max_time,
            wait_cost=self.wait_cost,
            planning_deadline=planning_deadline,
        )
        self._reset_diagnostics()

        if not problem.has_start_state:
            self._copy_diagnostics(problem)
            return None

        should_cancel = (
            problem.deadline_reached
            if planning_deadline is not None
            else None
        )
        result = AStarSolver[SippState]().solve(
            problem,
            should_cancel=should_cancel,
            cancellation_reason=problem.cancellation_reason,
        )
        self.expanded_nodes = result.expanded_count
        self._copy_diagnostics(problem)

        if result.found:
            return TimedPath(
                robot_name=request.robot_name,
                start_lm=request.start_lm,
                goal_lm=request.goal_lm,
                states=problem.expand_timed_path(result.path),
            )

        if result.failure_reason == problem.cancellation_reason:
            self.last_failure = problem.cancellation_reason
        elif not self.last_failure:
            self.last_failure = (
                f"no_sipp_path:{request.robot_name}:"
                f"{request.start_lm}->{request.goal_lm}"
            )
        return None

    def _reset_diagnostics(self) -> None:
        self.expanded_nodes = 0
        self.last_failure = ""
        self.blocking_robot_names = set()
        self.blocking_resources_by_robot = {}

    def _copy_diagnostics(
        self,
        problem: SippSearchProblem,
    ) -> None:
        self.last_failure = problem.last_failure
        self.blocking_robot_names = set(
            problem.blocking_robot_names
        )
        self.blocking_resources_by_robot = {
            robot_name: set(resources)
            for robot_name, resources
            in problem.blocking_resources_by_robot.items()
        }
