"""Single-agent state space and A* search used by CBS."""

from __future__ import annotations

from heapq import heappop, heappush
import time as py_time
from typing import Callable

from fleet_manager.core.math.geometry import normalize_angle_rounded
from fleet_manager.core.math.intervals import closed_intervals_overlap

from .cbs_conflicts import CbsConflictAnalyzer
from .cbs_models import (
    Conflict,
    Constraints,
    EdgeConstraint,
    EdgeIntervalConstraint,
    LmRobotRequest,
    NodeName,
    ResourceIntervalConstraint,
    State,
    VertexConstraint,
    VertexIntervalConstraint,
)


class LmCBSEnvironment:
    """Low-level temporal graph and per-agent constraints."""

    def __init__(
        self,
        graph: dict[NodeName, list[NodeName]],
        agent_requests: list[LmRobotRequest],
        blocked_nodes: set[NodeName] | None = None,
        global_vertex_constraints: (
            set[VertexConstraint] | None
        ) = None,
        global_edge_constraints: (
            set[EdgeConstraint] | None
        ) = None,
        global_vertex_intervals: (
            list[VertexIntervalConstraint] | None
        ) = None,
        global_edge_intervals: (
            list[EdgeIntervalConstraint] | None
        ) = None,
        global_resource_intervals: (
            list[ResourceIntervalConstraint] | None
        ) = None,
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
        rotation_allowed_fn: (
            Callable[[NodeName, float, float], bool] | None
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
        wait_cost: int = 6,
        planning_deadline: float | None = None,
        should_cancel: Callable[[], bool] | None = None,
    ) -> None:
        self.graph = graph
        self.agent_requests = agent_requests
        self.blocked_nodes = blocked_nodes or set()
        self.global_vertex_constraints = (
            global_vertex_constraints or set()
        )
        self.global_edge_constraints = (
            global_edge_constraints or set()
        )
        self.global_vertex_intervals = (
            global_vertex_intervals or []
        )
        self.global_edge_intervals = (
            global_edge_intervals or []
        )
        self.global_resource_intervals = set(
            global_resource_intervals or []
        )
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
        self.rotation_allowed_fn = (
            rotation_allowed_fn
            or (lambda _node, _from_yaw, _to_yaw: True)
        )
        self.vertex_resources_fn = vertex_resources_fn
        self.rotation_resources_fn = (
            rotation_resources_fn or vertex_resources_fn
        )
        self.lane_resources_fn = lane_resources_fn
        self.can_wait_fn = (
            can_wait_fn or (lambda _node: True)
        )
        self.low_level_max_time = max(
            1,
            int(low_level_max_time),
        )
        self.wait_cost = max(1, int(wait_cost))
        self.planning_deadline = planning_deadline
        self.should_cancel = should_cancel

        self.agent_dict: dict[
            str,
            dict[str, State],
        ] = {}
        self.constraint_dict: dict[str, Constraints] = {}
        self.constraints = Constraints()
        self.last_failure = ""
        self.route_next_by_agent: dict[
            str,
            dict[NodeName, NodeName],
        ] = {}
        self._make_agent_dict()
        self._conflicts = CbsConflictAnalyzer(
            transition_parts=self.transition_parts,
            vertex_resources_fn=self.vertex_resources_fn,
            rotation_resources_fn=(
                self.rotation_resources_fn
            ),
            lane_resources_fn=self.lane_resources_fn,
        )

    def _make_agent_dict(self) -> None:
        for agent in self.agent_requests:
            self.agent_dict[agent.robot_name] = {
                "start": State(
                    0,
                    agent.start_lm,
                    self._normalize_yaw(agent.start_yaw),
                ),
                "goal": State(0, agent.goal_lm),
            }
            if agent.route_nodes:
                self.route_next_by_agent[agent.robot_name] = {
                    start: goal
                    for start, goal in zip(
                        agent.route_nodes,
                        agent.route_nodes[1:],
                    )
                    if start != goal
                }

    def admissible_heuristic(
        self,
        state: State,
        agent_name: str,
    ) -> float:
        goal = self.agent_dict[agent_name]["goal"].node
        return self.heuristic_fn(state.node, goal)

    def is_at_goal(
        self,
        state: State,
        agent_name: str,
    ) -> bool:
        return (
            state.node
            == self.agent_dict[agent_name]["goal"].node
        )

    def is_goal_state_final(
        self,
        state: State,
        agent_name: str,
    ) -> bool:
        if not self.is_at_goal(state, agent_name):
            return False

        constraints = self.constraint_dict.get(
            agent_name,
            Constraints(),
        )
        for constraint in (
            constraints.vertex_constraints
            | self.global_vertex_constraints
        ):
            if (
                constraint.node == state.node
                and constraint.time >= state.time
            ):
                return False
        for interval in self.global_vertex_intervals:
            if (
                interval.node == state.node
                and interval.end_time >= state.time
            ):
                return False
        for interval in (
            constraints.vertex_interval_constraints
        ):
            if (
                interval.node == state.node
                and interval.end_time >= state.time
            ):
                return False
        for constraint in (
            constraints.edge_constraints
            | self.global_edge_constraints
        ):
            if (
                constraint.from_node == state.node
                and constraint.to_node == state.node
                and constraint.time >= state.time
            ):
                return False

        if self.vertex_resources_fn is not None:
            goal_resources = set(
                self.vertex_resources_fn(state.node)
            )
            for interval in (
                constraints.resource_interval_constraints
                | self.global_resource_intervals
            ):
                if (
                    interval.resource in goal_resources
                    and interval.end_time >= state.time
                ):
                    return False
        return True

    def state_valid(self, state: State) -> bool:
        if state.node not in self.graph:
            self.last_failure = f"unknown_node:{state.node}"
            return False
        if state.node in self.blocked_nodes:
            self.last_failure = f"blocked_lm:{state.node}"
            return False
        if (
            VertexConstraint(state.time, state.node)
            in self.constraints.vertex_constraints
        ):
            self.last_failure = (
                f"vertex_constrained:{state.node}@{state.time}"
            )
            return False
        if (
            VertexConstraint(state.time, state.node)
            in self.global_vertex_constraints
        ):
            self.last_failure = (
                f"reserved_lm:{state.node}@{state.time}"
            )
            return False

        for interval in self.global_vertex_intervals:
            if (
                interval.node == state.node
                and self._time_in_interval(
                    state.time,
                    interval.start_time,
                    interval.end_time,
                )
            ):
                owner = (
                    f":{interval.owner}"
                    if interval.owner
                    else ""
                )
                self.last_failure = (
                    f"reserved_lm_interval:{state.node}"
                    f"@{state.time}{owner}"
                )
                return False

        for interval in (
            self.constraints.vertex_interval_constraints
        ):
            if (
                interval.node == state.node
                and self._time_in_interval(
                    state.time,
                    interval.start_time,
                    interval.end_time,
                )
            ):
                self.last_failure = (
                    f"vertex_interval_constrained:"
                    f"{state.node}@{state.time}"
                )
                return False

        if (
            self.vertex_resources_fn is not None
            and not self._resource_constraints_allow(
                self.vertex_resources_fn(state.node),
                state.time,
                state.time,
            )
        ):
            self.last_failure = (
                f"resource_constrained:"
                f"{state.node}@{state.time}"
            )
            return False
        return True

    def transition_valid(
        self,
        state_1: State,
        state_2: State,
    ) -> bool:
        turn_ticks, _ = self.transition_parts(
            state_1,
            state_2,
        )
        move_start = state_1.time + turn_ticks
        if (
            turn_ticks > 0
            and not self.rotation_allowed_fn(
                state_1.node,
                state_1.yaw,
                state_2.yaw,
            )
        ):
            self.last_failure = f"rotation_blocked:{state_1.node}"
            return False
        if (
            state_1.node == state_2.node
            and self.vertex_resources_fn is not None
            and not self._resource_constraints_allow(
                self.vertex_resources_fn(state_1.node),
                state_1.time,
                state_2.time,
            )
        ):
            self.last_failure = (
                f"wait_resource_constrained:{state_1.node}"
                f"@{state_1.time}-{state_2.time}"
            )
            return False

        if (
            turn_ticks
            and not self._rotation_vertex_valid(
                state_1.node,
                state_1.time,
                move_start,
            )
        ):
            return False

        edge_constraint = EdgeConstraint(
            move_start,
            state_1.node,
            state_2.node,
        )
        if edge_constraint in self.constraints.edge_constraints:
            self.last_failure = (
                f"edge_constrained:{state_1.node}"
                f"->{state_2.node}@{state_1.time}"
            )
            return False
        if edge_constraint in self.global_edge_constraints:
            self.last_failure = (
                f"reserved_edge:{state_1.node}"
                f"->{state_2.node}@{state_1.time}"
            )
            return False

        move_state = State(
            move_start,
            state_1.node,
            state_2.yaw,
        )
        for interval in self.global_edge_intervals:
            if self._edge_interval_conflicts(
                move_state,
                state_2,
                interval,
            ):
                owner = (
                    f":{interval.owner}"
                    if interval.owner
                    else ""
                )
                self.last_failure = (
                    f"reserved_edge_interval:"
                    f"{state_1.node}->{state_2.node}"
                    f"@{state_1.time}-{state_2.time}"
                    f"{owner}"
                )
                return False

        for interval in (
            self.constraints.edge_interval_constraints
        ):
            if self._edge_interval_conflicts(
                move_state,
                state_2,
                interval,
            ):
                self.last_failure = (
                    f"edge_interval_constrained:"
                    f"{state_1.node}->{state_2.node}"
                    f"@{state_1.time}-{state_2.time}"
                )
                return False

        if (
            state_1.node != state_2.node
            and self.lane_resources_fn is not None
            and not self._resource_constraints_allow(
                self.lane_resources_fn(
                    state_1.node,
                    state_2.node,
                ),
                move_start,
                max(move_start, state_2.time - 1),
            )
        ):
            self.last_failure = (
                f"edge_resource_constrained:"
                f"{state_1.node}->{state_2.node}"
                f"@{move_start}-{state_2.time}"
            )
            return False
        return True

    def _rotation_vertex_valid(
        self,
        node: NodeName,
        start: int,
        end: int,
    ) -> bool:
        for constraint in (
            self.constraints.vertex_constraints
            | self.global_vertex_constraints
        ):
            if (
                constraint.node == node
                and start <= constraint.time <= end
            ):
                self.last_failure = (
                    f"rotation_vertex_constrained:"
                    f"{node}@{constraint.time}"
                )
                return False

        for interval in self.global_vertex_intervals:
            if (
                interval.node == node
                and self._intervals_overlap(
                    start,
                    end,
                    interval.start_time,
                    interval.end_time,
                )
            ):
                owner = (
                    f":{interval.owner}"
                    if interval.owner
                    else ""
                )
                self.last_failure = (
                    f"rotation_vertex_reserved:"
                    f"{node}@{start}-{end}{owner}"
                )
                return False

        for interval in (
            self.constraints.vertex_interval_constraints
        ):
            if (
                interval.node == node
                and self._intervals_overlap(
                    start,
                    end,
                    interval.start_time,
                    interval.end_time,
                )
            ):
                self.last_failure = (
                    f"rotation_vertex_interval_constrained:"
                    f"{node}@{start}-{end}"
                )
                return False

        if (
            self.rotation_resources_fn is not None
            and not self._resource_constraints_allow(
                self.rotation_resources_fn(node),
                start,
                end,
            )
        ):
            self.last_failure = (
                f"rotation_resource_constrained:"
                f"{node}@{start}-{end}"
            )
            return False
        return True

    def _resource_constraints_allow(
        self,
        resources: tuple[object, ...],
        start_time: int,
        end_time: int,
    ) -> bool:
        resource_set = set(resources)
        return not any(
            (
                interval.resource in resource_set
                and self._intervals_overlap(
                    start_time,
                    end_time,
                    interval.start_time,
                    interval.end_time,
                )
            )
            for interval in (
                self.constraints.resource_interval_constraints
                | self.global_resource_intervals
            )
        )

    @staticmethod
    def _time_in_interval(
        time_value: int,
        start_time: int,
        end_time: int,
    ) -> bool:
        return start_time <= time_value <= end_time

    @staticmethod
    def _intervals_overlap(
        start_a: int,
        end_a: int,
        start_b: int,
        end_b: int,
    ) -> bool:
        return closed_intervals_overlap(start_a, end_a, start_b, end_b)

    def _edge_interval_conflicts(
        self,
        state_1: State,
        state_2: State,
        interval: EdgeIntervalConstraint,
    ) -> bool:
        if state_1.node == state_2.node:
            return False
        same_edge = (
            state_1.node == interval.from_node
            and state_2.node == interval.to_node
        )
        reverse_edge = (
            state_1.node == interval.to_node
            and state_2.node == interval.from_node
        )
        if not same_edge and not reverse_edge:
            return False
        return self._intervals_overlap(
            state_1.time,
            state_2.time,
            interval.start_time,
            interval.end_time,
        )

    def get_neighbors(
        self,
        state: State,
        agent_name: str = "",
    ) -> list[State]:
        candidates = list(
            self.graph.get(state.node, [])
        ) + [state.node]
        route_next = self.route_next_by_agent.get(agent_name)
        if route_next is not None:
            expected = route_next.get(state.node)
            candidates = [
                node
                for node in candidates
                if node == state.node or node == expected
            ]

        neighbors: list[State] = []
        for node in candidates:
            if node == state.node:
                if not self.can_wait_fn(state.node):
                    continue
                next_state = State(
                    state.time + 1,
                    node,
                    state.yaw,
                )
            else:
                heading_options = (
                    self.heading_options_fn(
                        state.node,
                        node,
                    )
                    if self.heading_options_fn is not None
                    else (
                        self.heading_fn(state.node, node),
                    )
                )
                lane_yaws = tuple(
                    dict.fromkeys(
                        self._normalize_yaw(item)
                        for item in heading_options
                    )
                )
                if (
                    not self.can_wait_fn(state.node)
                    and len(lane_yaws) > 1
                ):
                    lane_yaws = (
                        min(
                            lane_yaws,
                            key=lambda value: (
                                max(
                                    0,
                                    int(
                                        self.turn_cost_fn(
                                            state.yaw,
                                            value,
                                        )
                                    ),
                                ),
                                value,
                            ),
                        ),
                    )
                for value in lane_yaws:
                    next_state = State(
                        (
                            state.time
                            + self.transition_duration(
                                state.node,
                                node,
                                from_yaw=state.yaw,
                                to_yaw=value,
                            )
                        ),
                        node,
                        value,
                    )
                    if (
                        self.state_valid(next_state)
                        and self.transition_valid(
                            state,
                            next_state,
                        )
                    ):
                        neighbors.append(next_state)
                continue

            if (
                self.state_valid(next_state)
                and self.transition_valid(state, next_state)
            ):
                neighbors.append(next_state)
        return neighbors

    def transition_duration(
        self,
        from_node: NodeName,
        to_node: NodeName,
        *,
        from_yaw: float = 0.0,
        to_yaw: float | None = None,
    ) -> int:
        if from_node == to_node:
            return 1
        lane_yaw = self._normalize_yaw(
            (
                self.heading_fn(from_node, to_node)
                if to_yaw is None
                else to_yaw
            )
        )
        move_ticks = max(
            1,
            int(self.move_cost_fn(from_node, to_node)),
        )
        turn_ticks = max(
            0,
            int(self.turn_cost_fn(from_yaw, lane_yaw)),
        )
        return move_ticks + turn_ticks

    def transition_cost(
        self,
        state_1: State,
        state_2: State,
    ) -> int:
        if state_1.node == state_2.node:
            return self.wait_cost
        return max(1, state_2.time - state_1.time)

    def transition_parts(
        self,
        state_1: State,
        state_2: State,
    ) -> tuple[int, int]:
        if state_1.node == state_2.node:
            return (
                0,
                max(1, state_2.time - state_1.time),
            )
        move_ticks = max(
            1,
            int(
                self.move_cost_fn(
                    state_1.node,
                    state_2.node,
                )
            ),
        )
        turn_ticks = max(
            0,
            (
                state_2.time
                - state_1.time
                - move_ticks
            ),
        )
        return turn_ticks, move_ticks

    @staticmethod
    def _normalize_yaw(value: float) -> float:
        return normalize_angle_rounded(value)

    def get_state(
        self,
        agent_name: str,
        solution: dict[str, list[State]],
        t: int,
    ) -> State:
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        return solution[agent_name][-1]

    def get_first_conflict(
        self,
        solution: dict[str, list[State]],
    ) -> Conflict | None:
        return self._conflicts.first_conflict(solution)

    def create_constraints_from_conflict(
        self,
        conflict: Conflict,
    ) -> dict[str, Constraints]:
        return self._conflicts.constraints_from_conflict(
            conflict
        )

    def compute_solution(
        self,
    ) -> dict[str, list[State]] | None:
        solution: dict[str, list[State]] = {}
        for agent_name in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(
                agent_name,
                Constraints(),
            )
            self.last_failure = ""
            local_solution = self.low_level_search(
                agent_name,
                self.low_level_max_time,
            )
            if not local_solution:
                if self.last_failure != "planning_timeout":
                    request = self.agent_dict[agent_name]
                    detail = (
                        self.last_failure
                        or (
                            f"{request['start'].node}"
                            f"->{request['goal'].node}"
                        )
                    )
                    self.last_failure = (
                        f"no_low_level_path:{agent_name}:"
                        f"{detail}"
                    )
                return None
            solution[agent_name] = local_solution
        return solution

    def compute_solution_cost(
        self,
        solution: dict[str, list[State]],
    ) -> int:
        cost = 0
        for path in solution.values():
            for start, end in zip(path, path[1:]):
                cost += self.transition_cost(start, end)
        return cost

    def low_level_search(
        self,
        agent_name: str,
        max_time: int,
    ) -> list[State] | None:
        initial_state = self.agent_dict[agent_name]["start"]
        self.constraints = self.constraint_dict.setdefault(
            agent_name,
            Constraints(),
        )
        if not self.state_valid(initial_state):
            return None

        open_heap: list[tuple[float, int, State]] = []
        open_set: set[State] = set()
        closed_set: set[State] = set()
        came_from: dict[State, State] = {}
        g_score: dict[State, int] = {
            initial_state: 0,
        }

        counter = 0
        heappush(
            open_heap,
            (
                self.admissible_heuristic(
                    initial_state,
                    agent_name,
                ),
                counter,
                initial_state,
            ),
        )
        open_set.add(initial_state)

        expansions = 0
        while open_heap:
            expansions += 1
            check_limits = expansions == 1 or expansions % 128 == 0
            cancelled = bool(
                check_limits
                and self.should_cancel is not None
                and self.should_cancel()
            )
            timed_out = bool(
                check_limits
                and self.planning_deadline is not None
                and py_time.monotonic() >= self.planning_deadline
            )
            if cancelled or timed_out:
                self.last_failure = (
                    "planning_cancelled"
                    if cancelled
                    else "planning_timeout"
                )
                return None

            _, _, current = heappop(open_heap)
            if current not in open_set:
                continue
            open_set.remove(current)

            if current.time > max_time:
                continue
            if self.is_goal_state_final(
                current,
                agent_name,
            ):
                return self.reconstruct_path(
                    came_from,
                    current,
                )

            closed_set.add(current)
            for neighbor in self.get_neighbors(
                current,
                agent_name,
            ):
                if neighbor in closed_set:
                    continue
                tentative_g = (
                    g_score[current]
                    + self.transition_cost(
                        current,
                        neighbor,
                    )
                )
                if tentative_g >= g_score.get(
                    neighbor,
                    10**18,
                ):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                counter += 1
                f_score = (
                    tentative_g
                    + self.admissible_heuristic(
                        neighbor,
                        agent_name,
                    )
                )
                heappush(
                    open_heap,
                    (f_score, counter, neighbor),
                )
                open_set.add(neighbor)
        return None

    def reconstruct_path(
        self,
        came_from: dict[State, State],
        current: State,
    ) -> list[State]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
