from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from typing import Callable

from .reservations import ReservationTable, SafeInterval
from .traffic_graph import TrafficGraph, TrafficLane


NodeName = str


@dataclass(frozen=True, slots=True)
class SippRobotRequest:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName


@dataclass(frozen=True, slots=True)
class TimedState:
    time: int
    node: NodeName


@dataclass(frozen=True, slots=True)
class SippState:
    time: int
    node: NodeName
    interval_start: int
    interval_end: int

    @property
    def key(self) -> tuple[NodeName, int, int]:
        return (self.node, self.interval_start, self.interval_end)


@dataclass(frozen=True, slots=True)
class TimedPath:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    states: tuple[TimedState, ...]

    @property
    def nodes(self) -> list[NodeName]:
        return [state.node for state in self.states]

    @property
    def times(self) -> list[int]:
        return [state.time for state in self.states]


class SippPlanner:
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
        self.heuristic_fn = heuristic_fn or (lambda _node, _goal: 0.0)
        self.move_cost_fn = move_cost_fn or (lambda _src, _dst: 1)
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.wait_cost = max(1, int(wait_cost))
        self.expanded_nodes = 0
        self.last_failure = ""
        self._safe_interval_cache: dict[tuple[NodeName, str], tuple[SafeInterval, ...]] = {}

    def plan(
        self,
        request: SippRobotRequest,
        reservations: ReservationTable,
        *,
        blocked_nodes: set[NodeName] | None = None,
        blocked_edges: set[tuple[NodeName, NodeName]] | None = None,
    ) -> TimedPath | None:
        blocked_nodes = blocked_nodes or set()
        blocked_edges = blocked_edges or set()
        self.expanded_nodes = 0
        self.last_failure = ""
        self._safe_interval_cache = {}
        start = self._start_state(request, reservations, blocked_nodes)
        if start is None:
            return None

        open_heap: list[tuple[float, int, int, SippState]] = []
        tie_breaker = count()
        came_from: dict[tuple[NodeName, int, int], tuple[NodeName, int, int]] = {}
        g_score: dict[tuple[NodeName, int, int], int] = {start.key: 0}
        state_by_key: dict[tuple[NodeName, int, int], SippState] = {start.key: start}
        closed: dict[tuple[NodeName, int, int], int] = {}
        heappush(
            open_heap,
            (
                self._heuristic(start.node, request.goal_lm),
                next(tie_breaker),
                0,
                start,
            ),
        )

        while open_heap:
            _, _, _, current = heappop(open_heap)
            if current.time > g_score.get(current.key, 10**18):
                continue
            if closed.get(current.key, 10**18) <= current.time:
                continue
            closed[current.key] = current.time
            self.expanded_nodes += 1

            if current.node == request.goal_lm and current.interval_end >= self.low_level_max_time + 1:
                return TimedPath(
                    robot_name=request.robot_name,
                    start_lm=request.start_lm,
                    goal_lm=request.goal_lm,
                    states=tuple(self._reconstruct(came_from, state_by_key, current)),
                )
            if current.time >= self.low_level_max_time:
                continue

            for neighbor, transition_cost in self._neighbors(
                current,
                request.robot_name,
                reservations,
                blocked_nodes,
                blocked_edges,
            ):
                if closed.get(neighbor.key, 10**18) <= neighbor.time:
                    continue
                tentative_g = g_score[current.key] + transition_cost
                if tentative_g >= g_score.get(neighbor.key, 10**18):
                    continue
                came_from[neighbor.key] = current.key
                g_score[neighbor.key] = tentative_g
                state_by_key[neighbor.key] = neighbor
                f_score = tentative_g + self._heuristic(neighbor.node, request.goal_lm)
                heappush(
                    open_heap,
                    (
                        f_score,
                        next(tie_breaker),
                        neighbor.time,
                        neighbor,
                    ),
                )

        if not self.last_failure:
            self.last_failure = f"no_sipp_path:{request.robot_name}:{request.start_lm}->{request.goal_lm}"
        return None

    def _neighbors(
        self,
        state: SippState,
        robot_name: str,
        reservations: ReservationTable,
        blocked_nodes: set[NodeName],
        blocked_edges: set[tuple[NodeName, NodeName]],
    ) -> list[tuple[SippState, int]]:
        neighbors: list[tuple[SippState, int]] = []
        for lane in self.graph.neighbors(state.node):
            if (lane.from_lm, lane.to_lm) in blocked_edges:
                continue
            duration = self._transition_duration(lane)
            if lane.to_lm in blocked_nodes:
                self.last_failure = f"blocked_lm:{lane.to_lm}"
                continue
            target_intervals = self._safe_intervals_for_node(
                lane.to_lm,
                robot_name,
                reservations,
            )
            for interval in target_intervals:
                next_state = self._successor_for_interval(
                    state,
                    lane,
                    duration,
                    interval,
                    robot_name,
                    reservations,
                )
                if next_state is None:
                    continue
                wait_ticks = max(0, next_state.time - duration - state.time)
                transition_cost = duration + (wait_ticks * self.wait_cost)
                neighbors.append((next_state, transition_cost))
        return neighbors

    def _start_state(
        self,
        request: SippRobotRequest,
        reservations: ReservationTable,
        blocked_nodes: set[NodeName],
    ) -> SippState | None:
        if request.start_lm not in self.graph.vertices:
            self.last_failure = f"unknown_node:{request.start_lm}"
            return None
        if request.start_lm in blocked_nodes:
            self.last_failure = f"blocked_lm:{request.start_lm}"
            return None
        for interval in self._safe_intervals_for_node(
            request.start_lm,
            request.robot_name,
            reservations,
        ):
            if interval.contains(0):
                return SippState(0, request.start_lm, interval.start, interval.end)
        self.last_failure = f"reserved_lm:{request.start_lm}@0"
        return None

    def _successor_for_interval(
        self,
        state: SippState,
        lane: TrafficLane,
        duration: int,
        interval: SafeInterval,
        robot_name: str,
        reservations: ReservationTable,
    ) -> SippState | None:
        earliest_depart = max(state.time, interval.start - duration)
        latest_depart = min(state.interval_end, interval.end - duration)
        if earliest_depart > latest_depart:
            return None

        depart = self._earliest_free_departure(
            lane,
            earliest_depart,
            latest_depart,
            duration,
            robot_name,
            reservations,
        )
        if depart is None:
            return None
        vertex = self.graph.vertices.get(state.node)
        if vertex is not None and not vertex.can_wait and depart > state.time:
            self.last_failure = f"cannot_wait:{state.node}@{state.time}-{depart}"
            return None
        arrival = depart + duration
        if arrival > self.low_level_max_time or not interval.contains(arrival):
            return None
        return SippState(arrival, lane.to_lm, interval.start, interval.end)

    def _safe_intervals_for_node(
        self,
        node: NodeName,
        robot_name: str,
        reservations: ReservationTable,
    ) -> tuple[SafeInterval, ...]:
        cache_key = (node, robot_name)
        cached = self._safe_interval_cache.get(cache_key)
        if cached is not None:
            return cached
        intervals = reservations.safe_intervals_for_resources(
            self.graph.vertex_resources(node),
            0,
            self.low_level_max_time + 1,
            ignore_robot_name=robot_name,
        )
        self._safe_interval_cache[cache_key] = intervals
        return intervals

    def _earliest_free_departure(
        self,
        lane: TrafficLane,
        earliest_depart: int,
        latest_depart: int,
        duration: int,
        robot_name: str,
        reservations: ReservationTable,
    ) -> int | None:
        for depart in range(max(0, earliest_depart), max(0, latest_depart) + 1):
            end_time = depart + duration
            if reservations.resources_are_free(
                self.graph.lane_resources(lane),
                depart,
                end_time,
                ignore_robot_name=robot_name,
            ):
                return depart
        self.last_failure = (
            f"reserved_edge:{lane.from_lm}->{lane.to_lm}"
            f"@{earliest_depart}-{latest_depart + duration}"
        )
        return None

    def _transition_duration(self, lane: TrafficLane) -> int:
        return max(1, int(self.move_cost_fn(lane.from_lm, lane.to_lm)))

    def _heuristic(self, node: NodeName, goal: NodeName) -> float:
        return max(0.0, float(self.heuristic_fn(node, goal)))

    def _reconstruct(
        self,
        came_from: dict[tuple[NodeName, int, int], tuple[NodeName, int, int]],
        state_by_key: dict[tuple[NodeName, int, int], SippState],
        current: SippState,
    ) -> list[TimedState]:
        keys = [current.key]
        while keys[-1] in came_from:
            keys.append(came_from[keys[-1]])
        states = [state_by_key[key] for key in reversed(keys)]
        if not states:
            return []

        expanded: list[TimedState] = [TimedState(states[0].time, states[0].node)]
        for index in range(1, len(states)):
            previous = states[index - 1]
            current_state = states[index]
            duration = self._transition_duration_between(previous.node, current_state.node)
            depart_time = max(previous.time, current_state.time - duration)
            for time_tick in range(previous.time + 1, depart_time + 1):
                expanded.append(TimedState(time_tick, previous.node))
            expanded.append(TimedState(current_state.time, current_state.node))
        return expanded

    def _transition_duration_between(self, from_node: NodeName, to_node: NodeName) -> int:
        if from_node == to_node:
            return 1
        lane = self.graph.lane_for(from_node, to_node)
        if lane is None:
            return max(1, int(self.move_cost_fn(from_node, to_node)))
        return self._transition_duration(lane)
