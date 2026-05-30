from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from heapq import heappop, heappush
from itertools import combinations
import time as py_time
from typing import Callable


NodeName = str


@dataclass(frozen=True)
class State:
    time: int
    node: NodeName

    def is_equal_except_time(self, other: "State") -> bool:
        return self.node == other.node


@dataclass
class Conflict:
    VERTEX = 1
    EDGE = 2

    time: int = -1
    type: int = -1
    agent_1: str = ""
    agent_2: str = ""
    node_1: NodeName | None = None
    node_2: NodeName | None = None


@dataclass(frozen=True)
class VertexConstraint:
    time: int
    node: NodeName


@dataclass(frozen=True)
class EdgeConstraint:
    time: int
    from_node: NodeName
    to_node: NodeName


@dataclass
class Constraints:
    vertex_constraints: set[VertexConstraint] = field(default_factory=set)
    edge_constraints: set[EdgeConstraint] = field(default_factory=set)

    def add_constraint(self, other: "Constraints") -> None:
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints


@dataclass(frozen=True)
class LmRobotRequest:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName


@dataclass
class LmRobotPlan:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    nodes: list[NodeName]


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
            (name, tuple((state.time, state.node) for state in path))
            for name, path in sorted(self.solution.items())
        )
        frozen_constraints = tuple(
            (
                name,
                tuple(sorted((item.time, item.node) for item in constraints.vertex_constraints)),
                tuple(sorted((item.time, item.from_node, item.to_node) for item in constraints.edge_constraints)),
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
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        low_level_max_time: int = 128,
    ) -> None:
        self.graph = graph
        self.agent_requests = agent_requests
        self.blocked_nodes = blocked_nodes or set()
        self.global_vertex_constraints = global_vertex_constraints or set()
        self.global_edge_constraints = global_edge_constraints or set()
        self.heuristic_fn = heuristic_fn or (lambda _node, _goal: 0.0)
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.agent_dict: dict[str, dict[str, State]] = {}
        self.constraint_dict: dict[str, Constraints] = {}
        self.constraints = Constraints()
        self._make_agent_dict()

    def _make_agent_dict(self) -> None:
        for agent in self.agent_requests:
            self.agent_dict[agent.robot_name] = {
                "start": State(0, agent.start_lm),
                "goal": State(0, agent.goal_lm),
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
        for constraint in constraints.edge_constraints | self.global_edge_constraints:
            if (
                constraint.from_node == state.node
                and constraint.to_node == state.node
                and constraint.time >= state.time
            ):
                return False
        return True

    def state_valid(self, state: State) -> bool:
        if state.node not in self.graph:
            return False
        if state.node in self.blocked_nodes:
            return False
        if VertexConstraint(state.time, state.node) in self.constraints.vertex_constraints:
            return False
        if VertexConstraint(state.time, state.node) in self.global_vertex_constraints:
            return False
        return True

    def transition_valid(self, state_1: State, state_2: State) -> bool:
        edge_constraint = EdgeConstraint(state_1.time, state_1.node, state_2.node)
        if edge_constraint in self.constraints.edge_constraints:
            return False
        if edge_constraint in self.global_edge_constraints:
            return False
        return True

    def get_neighbors(self, state: State) -> list[State]:
        candidates = [state.node] + self.graph.get(state.node, [])
        neighbors: list[State] = []
        for node in candidates:
            next_state = State(state.time + 1, node)
            if self.state_valid(next_state) and self.transition_valid(state, next_state):
                neighbors.append(next_state)
        return neighbors

    def get_state(self, agent_name: str, solution: dict[str, list[State]], t: int) -> State:
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        return solution[agent_name][-1]

    def get_first_conflict(self, solution: dict[str, list[State]]) -> Conflict | None:
        max_t = max(len(plan) for plan in solution.values())
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    return Conflict(
                        time=t,
                        type=Conflict.VERTEX,
                        agent_1=agent_1,
                        agent_2=agent_2,
                        node_1=state_1.node,
                    )

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)
                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)
                if state_1a.node == state_2b.node and state_1b.node == state_2a.node:
                    return Conflict(
                        time=t,
                        type=Conflict.EDGE,
                        agent_1=agent_1,
                        agent_2=agent_2,
                        node_1=state_1a.node,
                        node_2=state_1b.node,
                    )
        return None

    def create_constraints_from_conflict(self, conflict: Conflict) -> dict[str, Constraints]:
        if conflict.type == Conflict.VERTEX:
            assert conflict.node_1 is not None
            constraint = Constraints()
            constraint.vertex_constraints.add(VertexConstraint(conflict.time, conflict.node_1))
            return {
                conflict.agent_1: constraint,
                conflict.agent_2: constraint,
            }

        assert conflict.node_1 is not None
        assert conflict.node_2 is not None
        constraint_1 = Constraints()
        constraint_2 = Constraints()
        constraint_1.edge_constraints.add(
            EdgeConstraint(conflict.time, conflict.node_1, conflict.node_2)
        )
        constraint_2.edge_constraints.add(
            EdgeConstraint(conflict.time, conflict.node_2, conflict.node_1)
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
                return None
            solution[agent_name] = local_solution
        return solution

    def compute_solution_cost(self, solution: dict[str, list[State]]) -> int:
        return sum(len(path) for path in solution.values())

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

            if self.is_goal_state_final(current, agent_name):
                return self.reconstruct_path(came_from, current)
            if current.time > max_time:
                continue

            closed_set.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                tentative_g = g_score[current] + 1
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
        low_level_max_time: int = 128,
        max_high_level_nodes: int = 2000,
        max_planning_time_sec: float = 5.0,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = heuristic_fn
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.max_high_level_nodes = max(1, int(max_high_level_nodes))
        self.max_planning_time_sec = max(0.0, float(max_planning_time_sec))

    def plan_for_robots(
        self,
        robot_requests: list[LmRobotRequest],
        blocked_nodes: list[NodeName] | None = None,
        reserved_vertex_constraints: list[tuple[int, NodeName]] | None = None,
        reserved_edge_constraints: list[tuple[int, NodeName, NodeName]] | None = None,
        low_level_max_time: int | None = None,
        max_high_level_nodes: int | None = None,
        max_planning_time_sec: float | None = None,
    ) -> PlannerResult:
        debug = PlannerDebug(reason="init")
        blocked_nodes = blocked_nodes or []
        reserved_vertex_constraints = reserved_vertex_constraints or []
        reserved_edge_constraints = reserved_edge_constraints or []
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
            heuristic_fn=self.heuristic_fn,
            low_level_max_time=ll_max_time,
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
            debug.reason = "initial_solution_failed"
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
                    nodes = [state.node for state in states]
                    total_nodes += len(nodes)
                    plans[req.robot_name] = LmRobotPlan(
                        robot_name=req.robot_name,
                        start_lm=req.start_lm,
                        goal_lm=req.goal_lm,
                        nodes=nodes,
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
                new_solution = env.compute_solution()
                if not new_solution:
                    continue
                new_node.solution = new_solution
                new_node.cost = env.compute_solution_cost(new_node.solution)
                if new_node not in closed_set:
                    open_set.add(new_node)

        debug.reason = "no_solution"
        debug.conflicts_resolved = conflicts_resolved
        debug.high_level_nodes = high_level_nodes
        return PlannerResult(plans={}, debug=debug)
