from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from heapq import heappop, heappush
from itertools import combinations
import math
import time as py_time
from typing import Callable


NodeName = str


@dataclass(frozen=True)
class State:
    time: int
    node: NodeName
    yaw: float = 0.0

    def is_equal_except_time(self, other: "State") -> bool:
        return self.node == other.node


@dataclass
class Conflict:
    VERTEX = 1
    EDGE = 2
    RESOURCE = 3

    time: int = -1
    end_time: int = -1
    type: int = -1
    agent_1: str = ""
    agent_2: str = ""
    node_1: NodeName | None = None
    node_2: NodeName | None = None
    agent_1_from: NodeName | None = None
    agent_1_to: NodeName | None = None
    agent_1_time: int = -1
    agent_2_from: NodeName | None = None
    agent_2_to: NodeName | None = None
    agent_2_time: int = -1
    agent_1_resource_kind: str = ""
    agent_2_resource_kind: str = ""
    resource: object | None = None


@dataclass(frozen=True)
class VertexConstraint:
    time: int
    node: NodeName


@dataclass(frozen=True)
class EdgeConstraint:
    time: int
    from_node: NodeName
    to_node: NodeName


@dataclass(frozen=True)
class VertexIntervalConstraint:
    start_time: int
    end_time: int
    node: NodeName
    owner: str = ""


@dataclass(frozen=True)
class EdgeIntervalConstraint:
    start_time: int
    end_time: int
    from_node: NodeName
    to_node: NodeName
    owner: str = ""


@dataclass(frozen=True)
class ResourceIntervalConstraint:
    start_time: int
    end_time: int
    resource: object


@dataclass(frozen=True)
class PathVertexInterval:
    agent: str
    start_time: int
    end_time: int
    node: NodeName


@dataclass(frozen=True)
class PathEdgeInterval:
    agent: str
    start_time: int
    end_time: int
    from_node: NodeName
    to_node: NodeName


@dataclass(frozen=True)
class PathResourceInterval:
    agent: str
    start_time: int
    end_time: int
    resource: object
    kind: str
    node: NodeName | None = None
    from_node: NodeName | None = None
    to_node: NodeName | None = None


@dataclass
class Constraints:
    vertex_constraints: set[VertexConstraint] = field(default_factory=set)
    edge_constraints: set[EdgeConstraint] = field(default_factory=set)
    vertex_interval_constraints: set[VertexIntervalConstraint] = field(default_factory=set)
    edge_interval_constraints: set[EdgeIntervalConstraint] = field(default_factory=set)
    resource_interval_constraints: set[ResourceIntervalConstraint] = field(default_factory=set)

    def add_constraint(self, other: "Constraints") -> None:
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints
        self.vertex_interval_constraints |= other.vertex_interval_constraints
        self.edge_interval_constraints |= other.edge_interval_constraints
        self.resource_interval_constraints |= other.resource_interval_constraints


@dataclass(frozen=True)
class LmRobotRequest:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    start_yaw: float = 0.0
    route_nodes: tuple[NodeName, ...] = ()


@dataclass
class LmRobotPlan:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    nodes: list[NodeName]
    times: list[int] = field(default_factory=list)
    yaws: list[float] = field(default_factory=list)
    actions: list[str] = field(default_factory=list)


@dataclass
class PlannerDebug:
    reason: str = "unknown"
    conflicts_resolved: int = 0
    high_level_nodes: int = 0
    expanded_nodes: int = 0


@dataclass
class PlannerResult:
    plans: dict[str, LmRobotPlan]
    debug: PlannerDebug


@dataclass
class HighLevelNode:
    solution: dict[str, list[State]] = field(default_factory=dict)
    constraint_dict: dict[str, Constraints] = field(default_factory=dict)
    cost: int = 0

    def __hash__(self) -> int:
        frozen_solution = tuple(
            (name, tuple((state.time, state.node, state.yaw) for state in path))
            for name, path in sorted(self.solution.items())
        )
        frozen_constraints = tuple(
            (
                name,
                tuple(sorted((item.time, item.node) for item in constraints.vertex_constraints)),
                tuple(sorted((item.time, item.from_node, item.to_node) for item in constraints.edge_constraints)),
                tuple(
                    sorted(
                        (item.start_time, item.end_time, item.node)
                        for item in constraints.vertex_interval_constraints
                    )
                ),
                tuple(
                    sorted(
                        (item.start_time, item.end_time, item.from_node, item.to_node)
                        for item in constraints.edge_interval_constraints
                    )
                ),
                tuple(
                    sorted(
                        (item.start_time, item.end_time, str(item.resource))
                        for item in constraints.resource_interval_constraints
                    )
                ),
            )
            for name, constraints in sorted(self.constraint_dict.items())
        )
        return hash((frozen_solution, frozen_constraints))

    def __lt__(self, other: "HighLevelNode") -> bool:
        return self.cost < other.cost


class LmCBSEnvironment:
    def __init__(
        self,
        graph: dict[NodeName, list[NodeName]],
        agent_requests: list[LmRobotRequest],
        blocked_nodes: set[NodeName] | None = None,
        global_vertex_constraints: set[VertexConstraint] | None = None,
        global_edge_constraints: set[EdgeConstraint] | None = None,
        global_vertex_intervals: list[VertexIntervalConstraint] | None = None,
        global_edge_intervals: list[EdgeIntervalConstraint] | None = None,
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        heading_fn: Callable[[NodeName, NodeName], float] | None = None,
        turn_cost_fn: Callable[[float, float], int] | None = None,
        vertex_resources_fn: Callable[[NodeName], tuple[object, ...]] | None = None,
        lane_resources_fn: Callable[[NodeName, NodeName], tuple[object, ...]] | None = None,
        low_level_max_time: int = 128,
        wait_cost: int = 6,
    ) -> None:
        self.graph = graph
        self.agent_requests = agent_requests
        self.blocked_nodes = blocked_nodes or set()
        self.global_vertex_constraints = global_vertex_constraints or set()
        self.global_edge_constraints = global_edge_constraints or set()
        self.global_vertex_intervals = global_vertex_intervals or []
        self.global_edge_intervals = global_edge_intervals or []
        self.heuristic_fn = heuristic_fn or (lambda _node, _goal: 0.0)
        self.move_cost_fn = move_cost_fn or (lambda _src, _dst: 1)
        self.heading_fn = heading_fn or (lambda _src, _dst: 0.0)
        self.turn_cost_fn = turn_cost_fn or (lambda _from_yaw, _to_yaw: 0)
        self.vertex_resources_fn = vertex_resources_fn
        self.lane_resources_fn = lane_resources_fn
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.wait_cost = max(1, int(wait_cost))
        self.agent_dict: dict[str, dict[str, State]] = {}
        self.constraint_dict: dict[str, Constraints] = {}
        self.constraints = Constraints()
        self.last_failure = ""
        self.route_next_by_agent: dict[str, dict[NodeName, NodeName]] = {}
        self._make_agent_dict()

    def _make_agent_dict(self) -> None:
        for agent in self.agent_requests:
            self.agent_dict[agent.robot_name] = {
                "start": State(0, agent.start_lm, self._normalize_yaw(agent.start_yaw)),
                "goal": State(0, agent.goal_lm),
            }
            if agent.route_nodes:
                self.route_next_by_agent[agent.robot_name] = {
                    start: goal
                    for start, goal in zip(agent.route_nodes, agent.route_nodes[1:])
                    if start != goal
                }

    def admissible_heuristic(self, state: State, agent_name: str) -> float:
        goal = self.agent_dict[agent_name]["goal"].node
        return self.heuristic_fn(state.node, goal)

    def is_at_goal(self, state: State, agent_name: str) -> bool:
        return state.node == self.agent_dict[agent_name]["goal"].node

    def is_goal_state_final(self, state: State, agent_name: str) -> bool:
        if not self.is_at_goal(state, agent_name):
            return False

        constraints = self.constraint_dict.get(agent_name, Constraints())
        for constraint in constraints.vertex_constraints | self.global_vertex_constraints:
            if constraint.node == state.node and constraint.time >= state.time:
                return False
        for interval in self.global_vertex_intervals:
            if interval.node == state.node and interval.end_time >= state.time:
                return False
        for interval in constraints.vertex_interval_constraints:
            if interval.node == state.node and interval.end_time >= state.time:
                return False
        for constraint in constraints.edge_constraints | self.global_edge_constraints:
            if (
                constraint.from_node == state.node
                and constraint.to_node == state.node
                and constraint.time >= state.time
            ):
                return False
        if self.vertex_resources_fn is not None:
            goal_resources = set(self.vertex_resources_fn(state.node))
            for interval in constraints.resource_interval_constraints:
                if interval.resource in goal_resources and interval.end_time >= state.time:
                    return False
        return True

    def state_valid(self, state: State) -> bool:
        if state.node not in self.graph:
            self.last_failure = f"unknown_node:{state.node}"
            return False
        if state.node in self.blocked_nodes:
            self.last_failure = f"blocked_lm:{state.node}"
            return False
        if VertexConstraint(state.time, state.node) in self.constraints.vertex_constraints:
            self.last_failure = f"vertex_constrained:{state.node}@{state.time}"
            return False
        if VertexConstraint(state.time, state.node) in self.global_vertex_constraints:
            self.last_failure = f"reserved_lm:{state.node}@{state.time}"
            return False
        for interval in self.global_vertex_intervals:
            if interval.node == state.node and self._time_in_interval(state.time, interval.start_time, interval.end_time):
                owner = f":{interval.owner}" if interval.owner else ""
                self.last_failure = f"reserved_lm_interval:{state.node}@{state.time}{owner}"
                return False
        for interval in self.constraints.vertex_interval_constraints:
            if interval.node == state.node and self._time_in_interval(
                state.time,
                interval.start_time,
                interval.end_time,
            ):
                self.last_failure = f"vertex_interval_constrained:{state.node}@{state.time}"
                return False
        if self.vertex_resources_fn is not None and not self._resource_constraints_allow(
            self.vertex_resources_fn(state.node),
            state.time,
            state.time,
        ):
            self.last_failure = f"resource_constrained:{state.node}@{state.time}"
            return False
        return True

    def transition_valid(self, state_1: State, state_2: State) -> bool:
        turn_ticks, _ = self.transition_parts(state_1, state_2)
        move_start = state_1.time + turn_ticks
        if state_1.node == state_2.node and self.vertex_resources_fn is not None:
            if not self._resource_constraints_allow(
                self.vertex_resources_fn(state_1.node),
                state_1.time,
                state_2.time,
            ):
                self.last_failure = (
                    f"wait_resource_constrained:{state_1.node}"
                    f"@{state_1.time}-{state_2.time}"
                )
                return False
        if turn_ticks and not self._rotation_vertex_valid(
            state_1.node,
            state_1.time,
            move_start,
        ):
            return False
        edge_constraint = EdgeConstraint(move_start, state_1.node, state_2.node)
        if edge_constraint in self.constraints.edge_constraints:
            self.last_failure = f"edge_constrained:{state_1.node}->{state_2.node}@{state_1.time}"
            return False
        if edge_constraint in self.global_edge_constraints:
            self.last_failure = f"reserved_edge:{state_1.node}->{state_2.node}@{state_1.time}"
            return False
        move_state = State(move_start, state_1.node, state_2.yaw)
        for interval in self.global_edge_intervals:
            if self._edge_interval_conflicts(move_state, state_2, interval):
                owner = f":{interval.owner}" if interval.owner else ""
                self.last_failure = (
                    f"reserved_edge_interval:{state_1.node}->{state_2.node}"
                    f"@{state_1.time}-{state_2.time}{owner}"
                )
                return False
        for interval in self.constraints.edge_interval_constraints:
            if self._edge_interval_conflicts(move_state, state_2, interval):
                self.last_failure = (
                    f"edge_interval_constrained:{state_1.node}->{state_2.node}"
                    f"@{state_1.time}-{state_2.time}"
                )
                return False
        if state_1.node != state_2.node and self.lane_resources_fn is not None:
            if not self._resource_constraints_allow(
                self.lane_resources_fn(state_1.node, state_2.node),
                move_start,
                max(move_start, state_2.time - 1),
            ):
                self.last_failure = (
                    f"edge_resource_constrained:{state_1.node}->{state_2.node}"
                    f"@{move_start}-{state_2.time}"
                )
                return False
        return True

    def _rotation_vertex_valid(self, node: NodeName, start: int, end: int) -> bool:
        for constraint in self.constraints.vertex_constraints | self.global_vertex_constraints:
            if constraint.node == node and start <= constraint.time <= end:
                self.last_failure = f"rotation_vertex_constrained:{node}@{constraint.time}"
                return False
        for interval in self.global_vertex_intervals:
            if interval.node == node and self._intervals_overlap(
                start,
                end,
                interval.start_time,
                interval.end_time,
            ):
                owner = f":{interval.owner}" if interval.owner else ""
                self.last_failure = f"rotation_vertex_reserved:{node}@{start}-{end}{owner}"
                return False
        for interval in self.constraints.vertex_interval_constraints:
            if interval.node == node and self._intervals_overlap(
                start,
                end,
                interval.start_time,
                interval.end_time,
            ):
                self.last_failure = f"rotation_vertex_interval_constrained:{node}@{start}-{end}"
                return False
        if self.vertex_resources_fn is not None and not self._resource_constraints_allow(
            self.vertex_resources_fn(node),
            start,
            end,
        ):
            self.last_failure = f"rotation_resource_constrained:{node}@{start}-{end}"
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
            interval.resource in resource_set
            and self._intervals_overlap(
                start_time,
                end_time,
                interval.start_time,
                interval.end_time,
            )
            for interval in self.constraints.resource_interval_constraints
        )

    def _time_in_interval(self, time_value: int, start_time: int, end_time: int) -> bool:
        return start_time <= time_value <= end_time

    def _intervals_overlap(self, start_a: int, end_a: int, start_b: int, end_b: int) -> bool:
        return start_a <= end_b and start_b <= end_a

    def _interval_overlap_start(self, start_a: int, end_a: int, start_b: int, end_b: int) -> int:
        return max(start_a, start_b)

    def _edge_interval_conflicts(
        self,
        state_1: State,
        state_2: State,
        interval: EdgeIntervalConstraint,
    ) -> bool:
        if state_1.node == state_2.node:
            return False
        same_edge = state_1.node == interval.from_node and state_2.node == interval.to_node
        reverse_edge = state_1.node == interval.to_node and state_2.node == interval.from_node
        if not same_edge and not reverse_edge:
            return False
        return self._intervals_overlap(
            state_1.time,
            state_2.time,
            interval.start_time,
            interval.end_time,
        )

    def get_neighbors(self, state: State, agent_name: str = "") -> list[State]:
        candidates = list(self.graph.get(state.node, [])) + [state.node]
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
                next_state = State(state.time + 1, node, state.yaw)
            else:
                target_yaw = self._normalize_yaw(self.heading_fn(state.node, node))
                next_state = State(
                    state.time + self.transition_duration(
                        state.node,
                        node,
                        from_yaw=state.yaw,
                        to_yaw=target_yaw,
                    ),
                    node,
                    target_yaw,
                )
            if self.state_valid(next_state) and self.transition_valid(state, next_state):
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
            self.heading_fn(from_node, to_node) if to_yaw is None else to_yaw
        )
        move_ticks = max(1, int(self.move_cost_fn(from_node, to_node)))
        turn_ticks = max(0, int(self.turn_cost_fn(from_yaw, lane_yaw)))
        return move_ticks + turn_ticks

    def transition_cost(self, state_1: State, state_2: State) -> int:
        if state_1.node == state_2.node:
            return self.wait_cost
        return max(1, state_2.time - state_1.time)

    def transition_parts(self, state_1: State, state_2: State) -> tuple[int, int]:
        if state_1.node == state_2.node:
            return 0, max(1, state_2.time - state_1.time)
        move_ticks = max(1, int(self.move_cost_fn(state_1.node, state_2.node)))
        turn_ticks = max(0, state_2.time - state_1.time - move_ticks)
        return turn_ticks, move_ticks

    @staticmethod
    def _normalize_yaw(value: float) -> float:
        return round(math.atan2(math.sin(float(value)), math.cos(float(value))), 9)

    def get_state(self, agent_name: str, solution: dict[str, list[State]], t: int) -> State:
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        return solution[agent_name][-1]

    def get_first_conflict(self, solution: dict[str, list[State]]) -> Conflict | None:
        if not solution:
            return None

        horizon = max(path[-1].time for path in solution.values() if path)
        vertex_intervals: dict[str, list[PathVertexInterval]] = {}
        edge_intervals: dict[str, list[PathEdgeInterval]] = {}
        resource_intervals: dict[str, list[PathResourceInterval]] = {}
        for agent_name, path in solution.items():
            vertices, edges = self._path_intervals(agent_name, path, horizon)
            vertex_intervals[agent_name] = vertices
            edge_intervals[agent_name] = edges
            if self.vertex_resources_fn is not None and self.lane_resources_fn is not None:
                resource_intervals[agent_name] = self._path_resource_intervals(
                    agent_name,
                    path,
                    horizon,
                )

        first_conflict: Conflict | None = None
        for agent_1, agent_2 in combinations(solution.keys(), 2):
            conflict = self._first_vertex_interval_conflict(
                vertex_intervals[agent_1],
                vertex_intervals[agent_2],
            )
            first_conflict = self._earlier_conflict(first_conflict, conflict)

            conflict = self._first_edge_interval_conflict(
                edge_intervals[agent_1],
                edge_intervals[agent_2],
            )
            first_conflict = self._earlier_conflict(first_conflict, conflict)
            if resource_intervals:
                conflict = self._first_resource_interval_conflict(
                    resource_intervals[agent_1],
                    resource_intervals[agent_2],
                )
                first_conflict = self._earlier_conflict(first_conflict, conflict)
        return first_conflict

    def _path_resource_intervals(
        self,
        agent_name: str,
        path: list[State],
        horizon: int,
    ) -> list[PathResourceInterval]:
        if not path or self.vertex_resources_fn is None or self.lane_resources_fn is None:
            return []
        intervals: set[PathResourceInterval] = set()

        def add_vertex(node: NodeName, start: int, end: int) -> None:
            for resource in self.vertex_resources_fn(node):
                intervals.add(
                    PathResourceInterval(
                        agent_name,
                        start,
                        max(start + 1, end),
                        resource,
                        "vertex",
                        node=node,
                    )
                )

        for state in path:
            add_vertex(state.node, state.time, state.time + 1)
        for start, end in zip(path, path[1:]):
            if start.node == end.node:
                add_vertex(start.node, start.time, end.time + 1)
                continue
            turn_ticks, _ = self.transition_parts(start, end)
            move_start = start.time + turn_ticks
            if turn_ticks:
                add_vertex(start.node, start.time, move_start + 1)
            for resource in self.lane_resources_fn(start.node, end.node):
                intervals.add(
                    PathResourceInterval(
                        agent_name,
                        move_start,
                        max(move_start + 1, end.time),
                        resource,
                        "edge",
                        from_node=start.node,
                        to_node=end.node,
                    )
                )
        final = path[-1]
        add_vertex(final.node, final.time, horizon + 1)
        return sorted(
            intervals,
            key=lambda item: (
                item.start_time,
                item.end_time,
                str(item.resource),
                item.kind,
            ),
        )

    def _first_resource_interval_conflict(
        self,
        first: list[PathResourceInterval],
        second: list[PathResourceInterval],
    ) -> Conflict | None:
        best: Conflict | None = None
        second_by_resource: dict[object, list[PathResourceInterval]] = {}
        for interval in second:
            second_by_resource.setdefault(interval.resource, []).append(interval)
        for interval_1 in first:
            for interval_2 in second_by_resource.get(interval_1.resource, []):
                start = max(interval_1.start_time, interval_2.start_time)
                end = min(interval_1.end_time, interval_2.end_time)
                if start >= end:
                    continue
                conflict = Conflict(
                    time=start,
                    end_time=end - 1,
                    type=Conflict.RESOURCE,
                    agent_1=interval_1.agent,
                    agent_2=interval_2.agent,
                    node_1=interval_1.node,
                    node_2=interval_2.node,
                    agent_1_from=interval_1.from_node,
                    agent_1_to=interval_1.to_node,
                    agent_2_from=interval_2.from_node,
                    agent_2_to=interval_2.to_node,
                    agent_1_resource_kind=interval_1.kind,
                    agent_2_resource_kind=interval_2.kind,
                    resource=interval_1.resource,
                )
                best = self._earlier_conflict(best, conflict)
        return best

    def _path_intervals(
        self,
        agent_name: str,
        path: list[State],
        horizon: int,
    ) -> tuple[list[PathVertexInterval], list[PathEdgeInterval]]:
        if not path:
            return [], []

        vertex_intervals: list[PathVertexInterval] = []
        edge_intervals: list[PathEdgeInterval] = []
        for state in path:
            vertex_intervals.append(
                PathVertexInterval(agent_name, state.time, state.time, state.node)
            )

        for index in range(len(path) - 1):
            start = path[index]
            end = path[index + 1]
            if start.node == end.node:
                vertex_intervals.append(
                    PathVertexInterval(agent_name, start.time, end.time, start.node)
                )
                continue
            turn_ticks, _ = self.transition_parts(start, end)
            if turn_ticks:
                vertex_intervals.append(
                    PathVertexInterval(
                        agent_name,
                        start.time,
                        start.time + turn_ticks,
                        start.node,
                    )
                )
            edge_intervals.append(
                PathEdgeInterval(
                    agent=agent_name,
                    start_time=start.time + turn_ticks,
                    end_time=end.time,
                    from_node=start.node,
                    to_node=end.node,
                )
            )

        final = path[-1]
        if final.time <= horizon:
            vertex_intervals.append(
                PathVertexInterval(agent_name, final.time, horizon, final.node)
            )
        return vertex_intervals, edge_intervals

    def _first_vertex_interval_conflict(
        self,
        first: list[PathVertexInterval],
        second: list[PathVertexInterval],
    ) -> Conflict | None:
        best: Conflict | None = None
        for interval_1 in first:
            for interval_2 in second:
                if interval_1.node != interval_2.node:
                    continue
                if not self._intervals_overlap(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                ):
                    continue
                start_time = self._interval_overlap_start(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                )
                end_time = min(interval_1.end_time, interval_2.end_time)
                conflict = Conflict(
                    time=start_time,
                    end_time=end_time,
                    type=Conflict.VERTEX,
                    agent_1=interval_1.agent,
                    agent_2=interval_2.agent,
                    node_1=interval_1.node,
                )
                best = self._earlier_conflict(best, conflict)
        return best

    def _first_edge_interval_conflict(
        self,
        first: list[PathEdgeInterval],
        second: list[PathEdgeInterval],
    ) -> Conflict | None:
        best: Conflict | None = None
        for interval_1 in first:
            for interval_2 in second:
                same_edge = (
                    interval_1.from_node == interval_2.from_node
                    and interval_1.to_node == interval_2.to_node
                )
                reverse_edge = (
                    interval_1.from_node == interval_2.to_node
                    and interval_1.to_node == interval_2.from_node
                )
                if not same_edge and not reverse_edge:
                    continue
                if not self._intervals_overlap(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                ):
                    continue
                start_time = self._interval_overlap_start(
                    interval_1.start_time,
                    interval_1.end_time,
                    interval_2.start_time,
                    interval_2.end_time,
                )
                end_time = min(interval_1.end_time, interval_2.end_time)
                conflict = Conflict(
                    time=start_time,
                    end_time=end_time,
                    type=Conflict.EDGE,
                    agent_1=interval_1.agent,
                    agent_2=interval_2.agent,
                    node_1=interval_1.from_node,
                    node_2=interval_1.to_node,
                    agent_1_from=interval_1.from_node,
                    agent_1_to=interval_1.to_node,
                    agent_1_time=interval_1.start_time,
                    agent_2_from=interval_2.from_node,
                    agent_2_to=interval_2.to_node,
                    agent_2_time=interval_2.start_time,
                )
                best = self._earlier_conflict(best, conflict)
        return best

    def _earlier_conflict(
        self,
        current: Conflict | None,
        candidate: Conflict | None,
    ) -> Conflict | None:
        if candidate is None:
            return current
        if current is None:
            return candidate
        if candidate.time != current.time:
            return candidate if candidate.time < current.time else current
        return candidate if candidate.type < current.type else current

    def create_constraints_from_conflict(self, conflict: Conflict) -> dict[str, Constraints]:
        if conflict.type == Conflict.VERTEX:
            assert conflict.node_1 is not None
            constraint = Constraints()
            constraint.vertex_interval_constraints.add(
                VertexIntervalConstraint(
                    start_time=conflict.time,
                    end_time=max(conflict.time, conflict.end_time),
                    node=conflict.node_1,
                )
            )
            return {
                conflict.agent_1: constraint,
                conflict.agent_2: constraint,
            }

        if conflict.type == Conflict.RESOURCE:
            assert conflict.resource is not None
            constraint_1 = Constraints()
            constraint_2 = Constraints()
            resource_constraint = ResourceIntervalConstraint(
                start_time=conflict.time,
                end_time=max(conflict.time, conflict.end_time),
                resource=conflict.resource,
            )
            constraint_1.resource_interval_constraints.add(resource_constraint)
            constraint_2.resource_interval_constraints.add(resource_constraint)
            return {
                conflict.agent_1: constraint_1,
                conflict.agent_2: constraint_2,
            }

        assert conflict.node_1 is not None
        assert conflict.node_2 is not None
        constraint_1 = Constraints()
        constraint_2 = Constraints()
        agent_1_from = conflict.agent_1_from or conflict.node_1
        agent_1_to = conflict.agent_1_to or conflict.node_2
        agent_2_from = conflict.agent_2_from or conflict.node_2
        agent_2_to = conflict.agent_2_to or conflict.node_1
        constraint_1.edge_interval_constraints.add(
            EdgeIntervalConstraint(
                start_time=conflict.time,
                end_time=max(conflict.time, conflict.end_time),
                from_node=agent_1_from,
                to_node=agent_1_to,
            )
        )
        constraint_2.edge_interval_constraints.add(
            EdgeIntervalConstraint(
                start_time=conflict.time,
                end_time=max(conflict.time, conflict.end_time),
                from_node=agent_2_from,
                to_node=agent_2_to,
            )
        )
        return {
            conflict.agent_1: constraint_1,
            conflict.agent_2: constraint_2,
        }

    def compute_solution(self) -> dict[str, list[State]] | None:
        solution: dict[str, list[State]] = {}
        for agent_name in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent_name, Constraints())
            local_solution = self.low_level_search(agent_name, self.low_level_max_time)
            if not local_solution:
                if not self.last_failure:
                    request = self.agent_dict[agent_name]
                    self.last_failure = (
                        f"no_low_level_path:{agent_name}:"
                        f"{request['start'].node}->{request['goal'].node}"
                    )
                return None
            solution[agent_name] = local_solution
        return solution

    def compute_solution_cost(self, solution: dict[str, list[State]]) -> int:
        cost = 0
        for path in solution.values():
            for index in range(1, len(path)):
                cost += self.transition_cost(path[index - 1], path[index])
        return cost

    def low_level_search(self, agent_name: str, max_time: int) -> list[State] | None:
        initial_state = self.agent_dict[agent_name]["start"]
        self.constraints = self.constraint_dict.setdefault(agent_name, Constraints())
        if not self.state_valid(initial_state):
            return None

        open_heap: list[tuple[float, int, State]] = []
        open_set: set[State] = set()
        closed_set: set[State] = set()
        came_from: dict[State, State] = {}
        g_score: dict[State, int] = {initial_state: 0}

        counter = 0
        heappush(open_heap, (self.admissible_heuristic(initial_state, agent_name), counter, initial_state))
        open_set.add(initial_state)

        while open_heap:
            _, _, current = heappop(open_heap)
            if current not in open_set:
                continue
            open_set.remove(current)

            if current.time > max_time:
                continue
            if self.is_goal_state_final(current, agent_name):
                return self.reconstruct_path(came_from, current)

            closed_set.add(current)
            for neighbor in self.get_neighbors(current, agent_name):
                if neighbor in closed_set:
                    continue
                tentative_g = g_score[current] + self.transition_cost(current, neighbor)
                if tentative_g >= g_score.get(neighbor, 10**18):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                counter += 1
                f_score = tentative_g + self.admissible_heuristic(neighbor, agent_name)
                heappush(open_heap, (f_score, counter, neighbor))
                open_set.add(neighbor)
        return None

    def reconstruct_path(self, came_from: dict[State, State], current: State) -> list[State]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


class LmCBSPlanner:
    def __init__(
        self,
        graph: dict[NodeName, list[NodeName]],
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        heading_fn: Callable[[NodeName, NodeName], float] | None = None,
        turn_cost_fn: Callable[[float, float], int] | None = None,
        vertex_resources_fn: Callable[[NodeName], tuple[object, ...]] | None = None,
        lane_resources_fn: Callable[[NodeName, NodeName], tuple[object, ...]] | None = None,
        low_level_max_time: int = 128,
        max_high_level_nodes: int = 2000,
        max_planning_time_sec: float = 5.0,
        wait_cost: int = 6,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.heading_fn = heading_fn
        self.turn_cost_fn = turn_cost_fn
        self.vertex_resources_fn = vertex_resources_fn
        self.lane_resources_fn = lane_resources_fn
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.max_high_level_nodes = max(1, int(max_high_level_nodes))
        self.max_planning_time_sec = max(0.0, float(max_planning_time_sec))
        self.wait_cost = max(1, int(wait_cost))

    def plan_for_robots(
        self,
        robot_requests: list[LmRobotRequest],
        blocked_nodes: list[NodeName] | None = None,
        reserved_vertex_constraints: list[tuple[int, NodeName]] | None = None,
        reserved_edge_constraints: list[tuple[int, NodeName, NodeName]] | None = None,
        reserved_vertex_intervals: list[tuple[int, int, NodeName, str]] | None = None,
        reserved_edge_intervals: list[tuple[int, int, NodeName, NodeName, str]] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        low_level_max_time: int | None = None,
        max_high_level_nodes: int | None = None,
        max_planning_time_sec: float | None = None,
    ) -> PlannerResult:
        debug = PlannerDebug(reason="init")
        blocked_nodes = blocked_nodes or []
        reserved_vertex_constraints = reserved_vertex_constraints or []
        reserved_edge_constraints = reserved_edge_constraints or []
        reserved_vertex_intervals = reserved_vertex_intervals or []
        reserved_edge_intervals = reserved_edge_intervals or []
        ll_max_time = self.low_level_max_time if low_level_max_time is None else max(1, int(low_level_max_time))
        hl_max_nodes = self.max_high_level_nodes if max_high_level_nodes is None else max(1, int(max_high_level_nodes))
        planning_budget = (
            self.max_planning_time_sec
            if max_planning_time_sec is None
            else max(0.0, float(max_planning_time_sec))
        )
        planning_start = py_time.monotonic()

        global_vertex_constraints = {
            VertexConstraint(time=t, node=node)
            for t, node in reserved_vertex_constraints
        }
        global_edge_constraints = {
            EdgeConstraint(time=t, from_node=src, to_node=dst)
            for t, src, dst in reserved_edge_constraints
        }
        global_vertex_intervals = [
            VertexIntervalConstraint(
                start_time=max(0, int(start)),
                end_time=max(0, int(end)),
                node=node,
                owner=owner,
            )
            for start, end, node, owner in reserved_vertex_intervals
            if node
        ]
        global_edge_intervals = [
            EdgeIntervalConstraint(
                start_time=max(0, int(start)),
                end_time=max(0, int(end)),
                from_node=src,
                to_node=dst,
                owner=owner,
            )
            for start, end, src, dst, owner in reserved_edge_intervals
            if src and dst
        ]

        if not robot_requests:
            debug.reason = "empty_requests"
            return PlannerResult(plans={}, debug=debug)

        blocked_set = set(blocked_nodes)
        seen_goals: dict[NodeName, str] = {}
        for req in robot_requests:
            if req.start_lm not in self.graph:
                debug.reason = f"unknown_start:{req.robot_name}:{req.start_lm}"
                return PlannerResult(plans={}, debug=debug)
            if req.goal_lm not in self.graph:
                debug.reason = f"unknown_goal:{req.robot_name}:{req.goal_lm}"
                return PlannerResult(plans={}, debug=debug)
            if req.start_lm in blocked_set:
                debug.reason = f"start_blocked:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)
            if req.goal_lm in blocked_set:
                debug.reason = f"goal_blocked:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)
            if VertexConstraint(0, req.start_lm) in global_vertex_constraints:
                debug.reason = f"start_reserved:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)
            for interval in global_vertex_intervals:
                if interval.node == req.start_lm and interval.start_time <= 0 <= interval.end_time:
                    owner = f":{interval.owner}" if interval.owner else ""
                    debug.reason = f"start_reserved_interval:{req.robot_name}{owner}"
                    return PlannerResult(plans={}, debug=debug)
            if req.goal_lm in seen_goals:
                debug.reason = f"shared_goal_not_supported:{seen_goals[req.goal_lm]},{req.robot_name}@{req.goal_lm}"
                return PlannerResult(plans={}, debug=debug)
            seen_goals[req.goal_lm] = req.robot_name

        env = LmCBSEnvironment(
            self.graph,
            robot_requests,
            blocked_nodes=blocked_set,
            global_vertex_constraints=global_vertex_constraints,
            global_edge_constraints=global_edge_constraints,
            global_vertex_intervals=global_vertex_intervals,
            global_edge_intervals=global_edge_intervals,
            heuristic_fn=self.heuristic_fn,
            move_cost_fn=move_cost_fn or self.move_cost_fn,
            heading_fn=self.heading_fn,
            turn_cost_fn=self.turn_cost_fn,
            vertex_resources_fn=self.vertex_resources_fn,
            lane_resources_fn=self.lane_resources_fn,
            low_level_max_time=ll_max_time,
            wait_cost=self.wait_cost,
        )

        open_set: set[HighLevelNode] = set()
        closed_set: set[HighLevelNode] = set()
        start = HighLevelNode()
        start.constraint_dict = {
            agent.robot_name: Constraints()
            for agent in robot_requests
        }
        env.constraint_dict = start.constraint_dict
        start.solution = env.compute_solution() or {}
        if not start.solution:
            debug.reason = env.last_failure or "initial_solution_failed"
            return PlannerResult(plans={}, debug=debug)
        start.cost = env.compute_solution_cost(start.solution)
        open_set.add(start)

        conflicts_resolved = 0
        high_level_nodes = 0
        while open_set:
            if py_time.monotonic() - planning_start >= planning_budget:
                debug.reason = f"planning_timeout:{planning_budget:.3f}s"
                debug.conflicts_resolved = conflicts_resolved
                debug.high_level_nodes = high_level_nodes
                return PlannerResult(plans={}, debug=debug)
            if high_level_nodes >= hl_max_nodes:
                debug.reason = f"high_level_node_limit:{hl_max_nodes}"
                debug.conflicts_resolved = conflicts_resolved
                debug.high_level_nodes = high_level_nodes
                return PlannerResult(plans={}, debug=debug)

            current = min(open_set)
            open_set.remove(current)
            closed_set.add(current)
            high_level_nodes += 1
            env.constraint_dict = current.constraint_dict
            conflict = env.get_first_conflict(current.solution)
            if conflict is None:
                plans: dict[str, LmRobotPlan] = {}
                total_nodes = 0
                for req in robot_requests:
                    states = current.solution[req.robot_name]
                    expanded = self._expand_kinematic_states(env, states)
                    nodes = [state.node for state, _ in expanded]
                    total_nodes += len(nodes)
                    plans[req.robot_name] = LmRobotPlan(
                        robot_name=req.robot_name,
                        start_lm=req.start_lm,
                        goal_lm=req.goal_lm,
                        nodes=nodes,
                        times=[state.time for state, _ in expanded],
                        yaws=[state.yaw for state, _ in expanded],
                        actions=[action for _, action in expanded],
                    )
                debug.reason = "success"
                debug.conflicts_resolved = conflicts_resolved
                debug.high_level_nodes = high_level_nodes
                debug.expanded_nodes = total_nodes
                return PlannerResult(plans=plans, debug=debug)

            conflicts_resolved += 1
            constraint_dict = env.create_constraints_from_conflict(conflict)
            for agent_name in constraint_dict.keys():
                new_node = deepcopy(current)
                new_node.constraint_dict[agent_name].add_constraint(constraint_dict[agent_name])
                env.constraint_dict = new_node.constraint_dict
                # A CBS child changes constraints for exactly one agent.  The
                # previous implementation replanned the entire fleet for every
                # child, which makes 20+ robots look quadratic/exponential much
                # earlier than necessary.  Keep unaffected paths and run one
                # low-level search, as standard CBS does.
                env.constraints = new_node.constraint_dict.setdefault(agent_name, Constraints())
                local_solution = env.low_level_search(agent_name, ll_max_time)
                if not local_solution:
                    continue
                new_node.solution[agent_name] = local_solution
                new_node.cost = env.compute_solution_cost(new_node.solution)
                if new_node not in closed_set:
                    open_set.add(new_node)

        debug.reason = "no_solution"
        debug.conflicts_resolved = conflicts_resolved
        debug.high_level_nodes = high_level_nodes
        return PlannerResult(plans={}, debug=debug)

    def _expand_kinematic_states(
        self,
        env: LmCBSEnvironment,
        states: list[State],
    ) -> list[tuple[State, str]]:
        if not states:
            return []
        expanded: list[tuple[State, str]] = [(states[0], "start")]
        for start, end in zip(states, states[1:]):
            if start.node == end.node:
                expanded.append((end, "wait"))
                continue
            turn_ticks, _ = env.transition_parts(start, end)
            if turn_ticks:
                expanded.append(
                    (
                        State(start.time + turn_ticks, start.node, end.yaw),
                        "rotate",
                    )
                )
            expanded.append((end, "move"))
        return expanded
