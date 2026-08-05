"""Safe Interval Path Planning state space.

This module owns SIPP-specific temporal and reservation mathematics.  The
generic A* loop lives in :mod:`fleet_manager.core.search`; keeping the two concerns
separate makes both pieces independently testable.
"""

from __future__ import annotations

from time import monotonic
from typing import Callable, Sequence

from fleet_manager.core.math.geometry import normalize_angle_rounded

from ..common.reservations import ReservationTable, ResourceId, SafeInterval
from .sipp_models import (
    NodeName,
    SippRobotRequest,
    SippState,
    TimedState,
)
from ..graph.traffic_graph_models import TrafficGraph, TrafficLane


class SippSearchProblem:
    """SIPP state space and reservation diagnostics for one robot."""

    def __init__(
        self,
        graph: TrafficGraph,
        request: SippRobotRequest,
        reservations: ReservationTable,
        *,
        blocked_nodes: set[NodeName],
        blocked_edges: set[tuple[NodeName, NodeName]],
        heuristic_fn: Callable[[NodeName, NodeName], float],
        move_cost_fn: Callable[[NodeName, NodeName], int],
        heading_fn: Callable[[NodeName, NodeName], float],
        heading_options_fn: (
            Callable[[NodeName, NodeName], tuple[float, ...]] | None
        ),
        turn_cost_fn: Callable[[float, float], int],
        rotation_allowed_fn: Callable[[NodeName, float, float], bool],
        low_level_max_time: int,
        wait_cost: int,
        planning_deadline: float | None,
    ) -> None:
        self.graph = graph
        self.request = request
        self.reservations = reservations
        self.blocked_nodes = blocked_nodes
        self.blocked_edges = blocked_edges
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.heading_fn = heading_fn
        self.heading_options_fn = heading_options_fn
        self.turn_cost_fn = turn_cost_fn
        self.rotation_allowed_fn = rotation_allowed_fn
        self.low_level_max_time = low_level_max_time
        self.wait_cost = wait_cost
        self.planning_deadline = planning_deadline

        self.last_failure = ""
        self.blocking_robot_names: set[str] = set()
        self.blocking_resources_by_robot: dict[
            str,
            set[ResourceId],
        ] = {}
        self._safe_interval_cache: dict[
            tuple[NodeName, str],
            tuple[SafeInterval, ...],
        ] = {}
        self._initial_departure_not_before = max(
            0,
            int(request.initial_departure_not_before),
        )
        self._node_departure_not_before = {
            str(node): max(0, int(time_tick))
            for node, time_tick in request.node_departure_not_before
            if str(node)
        }
        self._authorized_controlled_regions = frozenset(
            str(region_id)
            for region_id in request.authorized_controlled_regions
            if str(region_id)
        )
        self._no_wait_nodes = frozenset(
            str(node)
            for node in request.no_wait_nodes
            if str(node)
        )
        self._route_next = self._build_route_next(request.route_nodes)
        self._start_state = self._find_start_state()

    @property
    def has_start_state(self) -> bool:
        return self._start_state is not None

    @property
    def start_state(self) -> SippState:
        if self._start_state is None:
            raise RuntimeError(
                "SIPP search cannot start after initial-state validation failed"
            )
        return self._start_state

    @property
    def cancellation_reason(self) -> str:
        return f"planning_timeout:{self.request.robot_name}"

    def deadline_reached(self) -> bool:
        return (
            self.planning_deadline is not None
            and monotonic() >= self.planning_deadline
        )

    @staticmethod
    def key(
        state: SippState,
    ) -> tuple[NodeName, int, int, int]:
        return state.key

    @staticmethod
    def dominance(
        state: SippState,
        _path_cost: float,
    ) -> float:
        """Prefer earlier arrival within the same safe interval and heading."""

        return float(state.time)

    def is_goal(self, state: SippState) -> bool:
        return (
            state.node == self.request.goal_lm
            and state.interval_end >= self.low_level_max_time + 1
        )

    def heuristic(self, state: SippState) -> float:
        return max(
            0.0,
            float(
                self.heuristic_fn(
                    state.node,
                    self.request.goal_lm,
                )
            ),
        )

    def neighbors(
        self,
        state: SippState,
    ) -> list[tuple[SippState, float]]:
        if state.time >= self.low_level_max_time:
            return []

        neighbors: list[tuple[SippState, float]] = []
        for lane in self.graph.neighbors(state.node):
            if (
                self._route_next is not None
                and lane.to_lm != self._route_next.get(state.node)
            ):
                continue
            if (lane.from_lm, lane.to_lm) in self.blocked_edges:
                continue

            move_duration = self._transition_duration(lane)
            if lane.to_lm in self.blocked_nodes:
                self.last_failure = f"blocked_lm:{lane.to_lm}"
                continue

            for lane_yaw in self._lane_yaws(state, lane):
                rotate_duration = max(
                    0,
                    int(self.turn_cost_fn(state.yaw, lane_yaw)),
                )
                if (
                    rotate_duration > 0
                    and not self.rotation_allowed_fn(
                        state.node,
                        state.yaw,
                        lane_yaw,
                    )
                ):
                    self.last_failure = f"rotation_blocked:{state.node}"
                    continue
                for interval in self._safe_intervals_for_node(
                    lane.to_lm,
                ):
                    next_state = self._successor_for_interval(
                        state,
                        lane,
                        move_duration,
                        rotate_duration,
                        lane_yaw,
                        interval,
                    )
                    if next_state is None:
                        continue
                    wait_ticks = max(
                        0,
                        next_state.time
                        - move_duration
                        - rotate_duration
                        - state.time,
                    )
                    transition_cost = (
                        move_duration
                        + rotate_duration
                        + (wait_ticks * self.wait_cost)
                    )
                    neighbors.append((next_state, transition_cost))
        return neighbors

    def expand_timed_path(
        self,
        states: Sequence[SippState],
    ) -> tuple[TimedState, ...]:
        """Expand sparse SIPP states into the legacy timed action format."""

        if not states:
            return ()

        expanded: list[TimedState] = [
            TimedState(
                states[0].time,
                states[0].node,
                states[0].yaw,
                "start",
            )
        ]
        for previous, current in zip(states, states[1:]):
            move_duration = self._transition_duration_between(
                previous.node,
                current.node,
            )
            rotate_duration = max(
                0,
                int(self.turn_cost_fn(previous.yaw, current.yaw)),
            )
            move_depart_time = max(
                previous.time,
                current.time - move_duration,
            )
            rotate_start_time = max(
                previous.time,
                move_depart_time - rotate_duration,
            )
            for time_tick in range(
                previous.time + 1,
                rotate_start_time + 1,
            ):
                expanded.append(
                    TimedState(
                        time_tick,
                        previous.node,
                        previous.yaw,
                        "wait",
                    )
                )
            if rotate_duration > 0:
                expanded.append(
                    TimedState(
                        move_depart_time,
                        previous.node,
                        current.yaw,
                        "rotate",
                    )
                )
            expanded.append(
                TimedState(
                    current.time,
                    current.node,
                    current.yaw,
                    "move",
                )
            )
        return tuple(expanded)

    @staticmethod
    def _build_route_next(
        route_nodes: tuple[NodeName, ...],
    ) -> dict[NodeName, NodeName] | None:
        if not route_nodes:
            return None
        return {
            source: destination
            for source, destination in zip(
                route_nodes,
                route_nodes[1:],
            )
            if source != destination
        }

    def _lane_yaws(
        self,
        state: SippState,
        lane: TrafficLane,
    ) -> tuple[float, ...]:
        heading_options = (
            self.heading_options_fn(lane.from_lm, lane.to_lm)
            if self.heading_options_fn is not None
            else (self.heading_fn(lane.from_lm, lane.to_lm),)
        )
        lane_yaws = tuple(
            dict.fromkeys(
                self._normalize_yaw(value)
                for value in heading_options
            )
        )
        vertex = self.graph.vertices.get(state.node)
        cannot_wait = (
            vertex is not None
            and (
                not vertex.can_wait
                or state.node in self._no_wait_nodes
            )
        )
        if not cannot_wait or len(lane_yaws) <= 1:
            return lane_yaws

        # An unspecified-motion edge offers both the geometric heading and its
        # reverse.  At a no-wait LM, keeping the unnecessary 180-degree option
        # could hide waiting inside a ROTATE action.
        return (
            min(
                lane_yaws,
                key=lambda value: (
                    max(
                        0,
                        int(self.turn_cost_fn(state.yaw, value)),
                    ),
                    value,
                ),
            ),
        )

    def _find_start_state(self) -> SippState | None:
        start_lm = self.request.start_lm
        if start_lm not in self.graph.vertices:
            self.last_failure = f"unknown_node:{start_lm}"
            return None
        if start_lm in self.blocked_nodes:
            self.last_failure = f"blocked_lm:{start_lm}"
            return None

        for interval in self._safe_intervals_for_node(start_lm):
            if interval.contains(0):
                return SippState(
                    0,
                    start_lm,
                    interval.start,
                    interval.end,
                    self._normalize_yaw(self.request.start_yaw),
                )
        self.last_failure = f"reserved_lm:{start_lm}@0"
        return None

    def _successor_for_interval(
        self,
        state: SippState,
        lane: TrafficLane,
        move_duration: int,
        rotate_duration: int,
        lane_yaw: float,
        interval: SafeInterval,
    ) -> SippState | None:
        earliest_depart = max(
            state.time + rotate_duration,
            interval.start - move_duration,
        )
        if (
            state.node == self.request.start_lm
            and state.time == 0
        ):
            earliest_depart = max(
                earliest_depart,
                self._initial_departure_not_before,
            )
        earliest_depart = max(
            earliest_depart,
            self._node_departure_not_before.get(state.node, 0),
        )

        # Safe intervals are half-open.  The source is reserved for
        # [depart, depart + 1), so departure at interval_end is invalid.
        latest_depart = min(
            state.interval_end - 1,
            interval.end - move_duration,
        )
        if earliest_depart > latest_depart:
            return None

        depart = self._earliest_free_departure(
            lane,
            earliest_depart,
            latest_depart,
            move_duration,
            rotate_duration,
        )
        if depart is None:
            return None

        vertex = self.graph.vertices.get(state.node)
        cannot_wait = (
            (vertex is not None and not vertex.can_wait)
            or state.node in self._no_wait_nodes
        )
        if cannot_wait and depart > state.time + rotate_duration:
            self.last_failure = (
                f"cannot_wait:{state.node}"
                f"@{state.time + rotate_duration}-{depart}"
            )
            return None

        arrival = depart + move_duration
        if (
            arrival > self.low_level_max_time
            or not interval.contains(arrival)
        ):
            return None
        return SippState(
            arrival,
            lane.to_lm,
            interval.start,
            interval.end,
            lane_yaw,
        )

    def _safe_intervals_for_node(
        self,
        node: NodeName,
    ) -> tuple[SafeInterval, ...]:
        robot_name = self.request.robot_name
        cache_key = (node, robot_name)
        cached = self._safe_interval_cache.get(cache_key)
        if cached is not None:
            return cached

        resources = self._request_resources(
            self.graph.vertex_resources(node)
        )
        intervals = self.reservations.safe_intervals_for_resources(
            resources,
            0,
            self.low_level_max_time + 1,
            ignore_robot_name=robot_name,
        )
        fully_safe = (
            SafeInterval(0, self.low_level_max_time + 1),
        )
        if intervals != fully_safe:
            self._record_blocking_owners(
                resources,
                0,
                self.low_level_max_time + 1,
            )
        self._safe_interval_cache[cache_key] = intervals
        return intervals

    def _earliest_free_departure(
        self,
        lane: TrafficLane,
        earliest_depart: int,
        latest_depart: int,
        duration: int,
        rotate_duration: int,
    ) -> int | None:
        robot_name = self.request.robot_name
        for depart in range(
            max(0, earliest_depart),
            max(0, latest_depart) + 1,
        ):
            if (
                self.planning_deadline is not None
                and monotonic() >= self.planning_deadline
            ):
                self.last_failure = self.cancellation_reason
                return None

            end_time = depart + duration
            rotate_start = max(0, depart - rotate_duration)
            if rotate_duration:
                rotation_resources = self._request_resources(
                    self.graph.rotation_resources(lane.from_lm)
                )
                rotation_is_free = (
                    self.reservations.resources_are_free(
                        rotation_resources,
                        rotate_start,
                        depart,
                        ignore_robot_name=robot_name,
                    )
                )
                if not rotation_is_free:
                    self._record_blocking_owners(
                        rotation_resources,
                        rotate_start,
                        depart,
                    )
                    continue

            lane_resources = self._request_resources(
                self.graph.lane_resources(lane)
            )
            if self.reservations.resources_are_free(
                lane_resources,
                depart,
                end_time,
                ignore_robot_name=robot_name,
            ):
                return depart
            self._record_blocking_owners(
                lane_resources,
                depart,
                end_time,
            )

        self.last_failure = (
            f"reserved_edge:{lane.from_lm}->{lane.to_lm}"
            f"@{earliest_depart}-{latest_depart + duration}"
        )
        return None

    def _request_resources(
        self,
        resources: tuple[ResourceId, ...],
    ) -> tuple[ResourceId, ...]:
        if not self._authorized_controlled_regions:
            return resources
        return tuple(
            resource
            for resource in resources
            if not (
                resource.kind == "controlled_region"
                and resource.name in self._authorized_controlled_regions
            )
        )

    def _record_blocking_owners(
        self,
        resources: tuple[ResourceId, ...],
        start: int,
        end: int,
    ) -> None:
        robot_name = self.request.robot_name
        for resource in resources:
            if self.reservations.is_free(
                resource,
                start,
                end,
                ignore_robot_name=robot_name,
            ):
                continue
            for interval in self.reservations.conflicts(
                resource,
                start,
                end,
                ignore_robot_name=robot_name,
            ):
                owner = interval.robot_name
                if owner and owner != robot_name:
                    self.blocking_robot_names.add(owner)
                    self.blocking_resources_by_robot.setdefault(
                        owner,
                        set(),
                    ).add(resource)

    def _transition_duration(self, lane: TrafficLane) -> int:
        return max(
            1,
            int(self.move_cost_fn(lane.from_lm, lane.to_lm)),
        )

    def _transition_duration_between(
        self,
        from_node: NodeName,
        to_node: NodeName,
    ) -> int:
        if from_node == to_node:
            return 1
        lane = self.graph.lane_for(from_node, to_node)
        if lane is None:
            return max(
                1,
                int(self.move_cost_fn(from_node, to_node)),
            )
        return self._transition_duration(lane)

    @staticmethod
    def _normalize_yaw(value: float) -> float:
        return normalize_angle_rounded(value)
