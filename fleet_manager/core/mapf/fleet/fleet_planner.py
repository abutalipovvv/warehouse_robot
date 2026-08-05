from __future__ import annotations

import heapq
import math
from typing import Any, Callable

from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, PlannedRoute
from fleet_manager.core.mapping.navigation.planner import LmRoutePlanner

from .fleet_planner_backends import BackendRunner, BackendSelector
from .fleet_planner_requests import PlanningRequestPreparer
from .fleet_planner_results import PlanningResultFormatter
from .fleet_planner_trajectory import FleetMotionModel, TrajectoryBuilder
from ..cbs.cbs_models import LmRobotRequest
from ..graph.traffic_graph_models import TrafficGraph


class FleetMapfPlanner:
    def __init__(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        params: dict[str, Any] | None = None,
    ) -> None:
        self.landmarks = landmarks
        self.edges = edges
        self.params = params or {}
        self._backend_selector = BackendSelector(
            strict=bool(self.params.get("strict_configuration", False)),
        )
        self.route_planner = LmRoutePlanner(landmarks, edges, params=params)
        self.graph = self._build_graph()
        self._traffic_graph_cache: dict[
            tuple[float, float, float, bool, bool, int],
            TrafficGraph,
        ] = {}
        self._controlled_corridor_ticks_cache: dict[tuple[float, float], int] = {}
        self._heuristic_cache: dict[tuple[str, str], float] = {}
        self.edge_by_key = {
            (edge.from_name, edge.to_name): edge
            for edge in edges
        }
        fleet_params = self.params.get("fleet", {})
        if not isinstance(fleet_params, dict):
            fleet_params = {}
        self.low_level_max_time = int(fleet_params.get("cbs_low_level_max_time", 160))
        self.max_high_level_nodes = int(fleet_params.get("cbs_max_high_level_nodes", 2000))
        self.max_planning_time_sec = float(fleet_params.get("cbs_max_planning_time_sec", 5.0))
        self.local_cbs_max_robots = max(
            2,
            int(fleet_params.get("local_cbs_max_robots", 8) or 8),
        )
        self.traffic_graph_cache_max_entries = max(
            1,
            int(fleet_params.get("traffic_graph_cache_max_entries", 16) or 16),
        )
        self.controlled_ticks_cache_max_entries = max(
            1,
            int(fleet_params.get("controlled_ticks_cache_max_entries", 32) or 32),
        )
        self.heuristic_cache_max_entries = max(
            1000,
            int(fleet_params.get("heuristic_cache_max_entries", 100000) or 100000),
        )
        self.time_step_sec = float(fleet_params.get("reservation_time_step_sec", 1.0))
        self.wait_time_sec = float(fleet_params.get("wait_time_sec", self.time_step_sec))
        self.wait_cost = int(fleet_params.get("wait_cost", 6))
        self.stretch_motion_to_reservation_ticks = bool(
            fleet_params.get("stretch_motion_to_reservation_ticks", False)
        )
        self.reserved_edge_detour_enabled = bool(
            fleet_params.get("reserved_edge_detour_enabled", False)
        )
        self.reserved_edge_hard_constraints_enabled = bool(
            fleet_params.get("reserved_edge_hard_constraints_enabled", True)
        )
        self.reservation_detour_horizon_sec = float(
            fleet_params.get(
                "reservation_detour_horizon_sec",
                fleet_params.get("reservation_horizon_sec", 8.0),
            )
        )
        self.min_robot_center_distance_m = self._min_robot_center_distance(fleet_params)
        self.rotation_min_robot_center_distance_m = self._rotation_min_robot_center_distance(
            fleet_params
        )
        controlled_corridors_value = fleet_params.get(
            "controlled_corridors_enabled",
            True,
        )
        self.controlled_corridors_mode = (
            str(controlled_corridors_value).strip().lower()
            if isinstance(controlled_corridors_value, str)
            else ("enabled" if bool(controlled_corridors_value) else "disabled")
        )
        self.controlled_corridors_enabled = (
            self._controlled_corridor_feature_is_enabled(
                controlled_corridors_value,
            )
        )
        if "controlled_corridor_auto_detect" in fleet_params:
            auto_detect_value = fleet_params.get(
                "controlled_corridor_auto_detect",
                False,
            )
        elif isinstance(controlled_corridors_value, str) and (
            controlled_corridors_value.strip().lower() in {"auto", "smart"}
        ):
            # Backward compatibility for the old combined ``auto`` setting.
            auto_detect_value = controlled_corridors_value
        else:
            # ``controlled_corridors_enabled: true`` now means explicit Traffic
            # Editor regions only. Topology inference must be requested
            # separately and can never be enabled accidentally by partial
            # embedded parameters.
            auto_detect_value = False
        self.controlled_corridor_auto_detect = (
            self.controlled_corridors_enabled
            and self._controlled_corridors_are_enabled(auto_detect_value)
        )
        self.controlled_corridor_min_edges = max(
            1,
            int(fleet_params.get("controlled_corridor_min_edges", 2) or 2),
        )
        self.planner_backend = self._planner_backend(fleet_params)
        planner_params = self.params.get("planner", {})
        if not isinstance(planner_params, dict):
            planner_params = {}
        try:
            self.start_pose_lm_tolerance_m = max(
                0.01,
                float(
                    fleet_params.get(
                        "start_pose_lm_tolerance_m",
                        planner_params.get("on_route_tolerance", 0.10),
                    )
                    or 0.10
                ),
            )
        except (TypeError, ValueError):
            self.start_pose_lm_tolerance_m = 0.10
        self._request_preparer = PlanningRequestPreparer(self)
        self._backend_runner = BackendRunner(self)
        self._result_formatter = PlanningResultFormatter(self)
        self._motion_model = FleetMotionModel(self)
        self._trajectory_builder = TrajectoryBuilder(self)

    def plan(
        self,
        payload: dict[str, Any],
        *,
        should_cancel: Callable[[], bool] | None = None,
    ) -> dict[str, Any]:
        if should_cancel is not None and should_cancel():
            raise InterruptedError("planning cancelled")
        prepared = self._request_preparer.prepare(payload)
        (
            result,
            used_blocked_edges,
            used_reserved_detour,
            fallback_reason,
        ) = self._run_selected_backend(
            prepared.requests,
            blocked_lms=prepared.blocked_lms,
            blocked_edges=prepared.blocked_edges,
            detour_blocked_edges=prepared.detour_blocked_edges,
            speed=prepared.speed,
            acceleration=prepared.acceleration,
            reserved_vertex_constraints=(
                prepared.reserved_vertex_constraints
            ),
            reserved_edge_constraints=prepared.reserved_edge_constraints,
            reserved_vertex_intervals=prepared.reserved_vertex_intervals,
            reserved_edge_intervals=prepared.reserved_edge_intervals,
            reserved_interval_edges=prepared.reserved_interval_edges,
            low_level_max_time=prepared.low_level_max_time,
            allow_cbs_fallback=prepared.allow_cbs_fallback,
            rotate_enabled=prepared.rotate_enabled,
            turn_speed=prepared.turn_speed,
            selected_backend=prepared.selected_backend,
            should_cancel=should_cancel,
        )
        if should_cancel is not None and should_cancel():
            raise InterruptedError("planning cancelled")
        return self._result_formatter.format(
            prepared,
            result,
            used_blocked_edges=used_blocked_edges,
            used_reserved_detour=used_reserved_detour,
            fallback_reason=fallback_reason,
        )

    def _timed_segments_for_nodes(
        self,
        nodes: list[str],
        times: list[int],
        actions: list[str] | None = None,
    ) -> list[dict[str, Any]]:
        return self._result_formatter.timed_segments(
            nodes,
            times,
            actions,
        )

    def _run_selected_backend(
        self,
        requests: list[LmRobotRequest],
        blocked_lms: set[str],
        blocked_edges: set[tuple[str, str]],
        detour_blocked_edges: set[tuple[str, str]],
        speed: float,
        acceleration: float,
        reserved_vertex_constraints: list[tuple[int, str]],
        reserved_edge_constraints: list[tuple[int, str, str]],
        reserved_vertex_intervals: list[tuple[int, int, str, str]],
        reserved_edge_intervals: list[tuple[int, int, str, str, str]],
        reserved_interval_edges: set[tuple[str, str]],
        low_level_max_time: int,
        allow_cbs_fallback: bool,
        rotate_enabled: bool,
        turn_speed: float,
        selected_backend: str,
        should_cancel: Callable[[], bool] | None = None,
    ):
        return self._backend_runner.run_selected(
            requests,
            blocked_lms=blocked_lms,
            blocked_edges=blocked_edges,
            detour_blocked_edges=detour_blocked_edges,
            speed=speed,
            acceleration=acceleration,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
            reserved_interval_edges=reserved_interval_edges,
            low_level_max_time=low_level_max_time,
            allow_cbs_fallback=allow_cbs_fallback,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            selected_backend=selected_backend,
            should_cancel=should_cancel,
        )

    def _run_cbs_with_reserved_detour(
        self,
        requests: list[LmRobotRequest],
        blocked_lms: set[str],
        blocked_edges: set[tuple[str, str]],
        detour_blocked_edges: set[tuple[str, str]],
        speed: float,
        acceleration: float,
        reserved_vertex_constraints: list[tuple[int, str]],
        reserved_edge_constraints: list[tuple[int, str, str]],
        reserved_vertex_intervals: list[tuple[int, int, str, str]],
        reserved_edge_intervals: list[tuple[int, int, str, str, str]],
        reserved_interval_edges: set[tuple[str, str]],
        low_level_max_time: int,
        rotate_enabled: bool,
        turn_speed: float,
        should_cancel: Callable[[], bool] | None = None,
    ):
        return self._backend_runner.run_cbs_with_reserved_detour(
            requests,
            blocked_lms=blocked_lms,
            blocked_edges=blocked_edges,
            detour_blocked_edges=detour_blocked_edges,
            speed=speed,
            acceleration=acceleration,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
            reserved_interval_edges=reserved_interval_edges,
            low_level_max_time=low_level_max_time,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            should_cancel=should_cancel,
        )

    def _run_cbs(
        self,
        requests: list[LmRobotRequest],
        blocked_lms: set[str],
        blocked_edges: set[tuple[str, str]],
        speed: float,
        acceleration: float,
        reserved_vertex_constraints: list[tuple[int, str]],
        reserved_edge_constraints: list[tuple[int, str, str]],
        reserved_vertex_intervals: list[tuple[int, int, str, str]],
        reserved_edge_intervals: list[tuple[int, int, str, str, str]],
        low_level_max_time: int,
        rotate_enabled: bool,
        turn_speed: float,
        should_cancel: Callable[[], bool] | None = None,
    ):
        return self._backend_runner.run_cbs(
            requests,
            blocked_lms=blocked_lms,
            blocked_edges=blocked_edges,
            speed=speed,
            acceleration=acceleration,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
            low_level_max_time=low_level_max_time,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            should_cancel=should_cancel,
        )

    def _run_rolling_sipp(
        self,
        requests: list[LmRobotRequest],
        blocked_lms: set[str],
        blocked_edges: set[tuple[str, str]],
        speed: float,
        acceleration: float,
        reserved_vertex_constraints: list[tuple[int, str]],
        reserved_edge_constraints: list[tuple[int, str, str]],
        reserved_vertex_intervals: list[tuple[int, int, str, str]],
        reserved_edge_intervals: list[tuple[int, int, str, str, str]],
        low_level_max_time: int,
        rotate_enabled: bool,
        turn_speed: float,
        should_cancel: Callable[[], bool] | None = None,
    ):
        return self._backend_runner.run_rolling_sipp(
            requests,
            blocked_lms=blocked_lms,
            blocked_edges=blocked_edges,
            speed=speed,
            acceleration=acceleration,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
            low_level_max_time=low_level_max_time,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            should_cancel=should_cancel,
        )

    def _payload_low_level_max_time(self, payload: dict[str, Any]) -> int:
        return self._request_preparer.payload_low_level_max_time(payload)

    def _request_route_nodes(
        self,
        item: dict[str, Any],
        *,
        robot_name: str,
        start_lm: str,
        goal_lm: str,
    ) -> tuple[str, ...]:
        return self._request_preparer.request_route_nodes(
            item,
            robot_name=robot_name,
            start_lm=start_lm,
            goal_lm=goal_lm,
        )

    def _traffic_graph(self, speed: float) -> TrafficGraph:
        key = (
            round(max(0.02, float(speed)), 6),
            round(self.min_robot_center_distance_m, 6),
            round(self.rotation_min_robot_center_distance_m, 6),
            self.controlled_corridors_enabled,
            self.controlled_corridor_auto_detect,
            self.controlled_corridor_min_edges,
        )
        cached = self._traffic_graph_cache.get(key)
        if cached is not None:
            return cached
        graph = TrafficGraph.from_route_core(
            self.landmarks,
            self.edges,
            default_speed_mps=speed,
            min_robot_center_distance_m=self.min_robot_center_distance_m,
            rotation_min_robot_center_distance_m=(
                self.rotation_min_robot_center_distance_m
            ),
            explicit_controlled_regions_enabled=self.controlled_corridors_enabled,
            controlled_corridors_enabled=self.controlled_corridor_auto_detect,
            controlled_corridor_min_edges=self.controlled_corridor_min_edges,
        )
        self._bounded_cache_store(
            self._traffic_graph_cache,
            key,
            graph,
            self.traffic_graph_cache_max_entries,
        )
        return graph

    def controlled_corridor_max_ticks(
        self,
        *,
        speed: float | None = None,
        acceleration: float | None = None,
    ) -> int:
        route_speed = (
            self._route_speed({})
            if speed is None
            else max(0.02, float(speed))
        )
        route_acceleration = (
            self._route_acceleration({})
            if acceleration is None
            else max(0.02, float(acceleration))
        )
        cache_key = (
            round(route_speed, 6),
            round(route_acceleration, 6),
        )
        cached = self._controlled_corridor_ticks_cache.get(cache_key)
        if cached is not None:
            return cached
        graph = self._traffic_graph(route_speed)
        region_groups: dict[str, dict[str, int]] = {}
        for lane in graph.lanes.values():
            if not lane.controlled_region_ids:
                continue
            ticks = self._edge_tick_cost(
                lane.from_lm,
                lane.to_lm,
                route_speed,
                route_acceleration,
            )
            for region_id in lane.controlled_region_ids:
                by_group = region_groups.setdefault(region_id, {})
                by_group[lane.lane_group_id] = max(
                    by_group.get(lane.lane_group_id, 0),
                    ticks,
                )
        maximum = max(
            (sum(groups.values()) for groups in region_groups.values()),
            default=0,
        )
        # Authored rectangles can form one continuous no-wait passage through
        # direct zone-to-zone edges.  The runtime admission controller grants
        # that whole safe-LM-to-safe-LM bundle atomically, so the low-level
        # horizon must be long enough to reach an external holding point, not
        # merely the end of one rectangle.
        controlled_endpoints = {
            endpoint
            for lane in graph.lanes.values()
            if lane.controlled_region_ids
            for endpoint in (lane.from_lm, lane.to_lm)
        }
        internal = {
            vertex.id
            for vertex in graph.vertices.values()
            if vertex.id in controlled_endpoints and not vertex.can_wait
        }
        boundary = {
            endpoint
            for lane in graph.lanes.values()
            if lane.controlled_region_ids
            for endpoint, other in (
                (lane.from_lm, lane.to_lm),
                (lane.to_lm, lane.from_lm),
            )
            if endpoint not in internal and other in internal
        }
        passage_maximum = 0
        for start in boundary:
            distances: dict[str, int] = {start: 0}
            pending: list[tuple[int, str]] = [(0, start)]
            while pending:
                distance, node = heapq.heappop(pending)
                if distance != distances.get(node):
                    continue
                if node != start and node not in internal:
                    passage_maximum = max(passage_maximum, distance)
                    continue
                for lane in graph.neighbors(node):
                    if not lane.controlled_region_ids:
                        continue
                    target = lane.to_lm
                    if target not in internal and target not in boundary:
                        continue
                    next_distance = distance + self._edge_tick_cost(
                        lane.from_lm,
                        lane.to_lm,
                        route_speed,
                        route_acceleration,
                    )
                    if next_distance >= distances.get(target, 1 << 60):
                        continue
                    distances[target] = next_distance
                    heapq.heappush(pending, (next_distance, target))
        maximum = max(maximum, passage_maximum)
        self._bounded_cache_store(
            self._controlled_corridor_ticks_cache,
            cache_key,
            maximum,
            self.controlled_ticks_cache_max_entries,
        )
        return maximum

    @staticmethod
    def _bounded_cache_store(
        cache: dict[Any, Any],
        key: Any,
        value: Any,
        maximum: int,
    ) -> None:
        """Keep lifelong runtime caches useful without retaining all inputs."""
        if key not in cache and len(cache) >= max(1, int(maximum)):
            cache.pop(next(iter(cache)), None)
        cache[key] = value

    def _controlled_corridors_are_enabled(self, value: Any) -> bool:
        """Resolve explicit booleans and map-aware SMART auto mode.

        ``auto`` deliberately requires every graph edge to carry the SMART
        marker. This enables topology-derived single-lane corridors on the
        generated SMART warehouse maps without silently changing arbitrary
        imported customer maps.
        """
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"auto", "smart"}:
                self.controlled_corridors_mode = "auto"
                return bool(self.edges) and all(
                    self._edge_has_smart_marker(edge)
                    for edge in self.edges
                )
            return normalized not in {"", "0", "false", "no", "off", "disabled"}
        return bool(value)

    @staticmethod
    def _controlled_corridor_feature_is_enabled(value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() not in {
                "",
                "0",
                "false",
                "no",
                "off",
                "disabled",
            }
        return bool(value)

    @staticmethod
    def _edge_has_smart_marker(edge: GraphEdge) -> bool:
        properties = edge.properties if isinstance(edge.properties, dict) else {}
        value = properties.get("smart", False)
        if isinstance(value, str):
            return value.strip().lower() not in {"", "0", "false", "no", "off"}
        return bool(value)

    def _min_robot_center_distance(self, fleet_params: dict[str, Any]) -> float:
        configured = fleet_params.get("mapf_min_robot_center_distance_m")
        if configured is not None:
            try:
                return max(0.0, float(configured))
            except (TypeError, ValueError):
                pass

        robot_model = self.params.get("robot_model", {})
        if not isinstance(robot_model, dict):
            robot_model = {}
        try:
            radius = max(0.0, float(robot_model.get("radius", 0.22) or 0.22))
        except (TypeError, ValueError):
            radius = 0.22
        footprint = robot_model.get("footprint")
        if isinstance(footprint, list):
            for point in footprint:
                if not isinstance(point, dict):
                    continue
                try:
                    radius = max(
                        radius,
                        math.hypot(
                            float(point.get("x", 0.0) or 0.0),
                            float(point.get("y", 0.0) or 0.0),
                        ),
                    )
                except (TypeError, ValueError):
                    continue
        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        try:
            collision_margin = max(
                0.0,
                float(navigation.get("collision_margin", 0.04) or 0.04),
            )
        except (TypeError, ValueError):
            collision_margin = 0.04
        try:
            configured_clearance = fleet_params.get("robot_clearance_m", 0.35)
            clearance = max(
                0.0,
                0.35
                if configured_clearance is None
                else float(configured_clearance),
            )
        except (TypeError, ValueError):
            clearance = 0.35
        return (radius * 2.0) + collision_margin + clearance

    def _rotation_min_robot_center_distance(self, fleet_params: dict[str, Any]) -> float:
        configured = fleet_params.get("mapf_rotation_center_distance_m")
        if configured is not None:
            try:
                return max(0.0, float(configured))
            except (TypeError, ValueError):
                pass

        robot_model = self.params.get("robot_model", {})
        if not isinstance(robot_model, dict):
            robot_model = {}
        try:
            radius = max(0.0, float(robot_model.get("radius", 0.22) or 0.22))
        except (TypeError, ValueError):
            radius = 0.22
        footprint = robot_model.get("footprint")
        if isinstance(footprint, list):
            for point in footprint:
                if not isinstance(point, dict):
                    continue
                try:
                    radius = max(
                        radius,
                        math.hypot(
                            float(point.get("x", 0.0) or 0.0),
                            float(point.get("y", 0.0) or 0.0),
                        ),
                    )
                except (TypeError, ValueError):
                    continue
        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        try:
            collision_margin = max(
                0.0,
                float(navigation.get("collision_margin", 0.04) or 0.04),
            )
        except (TypeError, ValueError):
            collision_margin = 0.04
        return (radius * 2.0) + collision_margin

    def _build_graph(self) -> dict[str, list[str]]:
        graph: dict[str, set[str]] = {name: set() for name in self.landmarks}
        for edge in self.edges:
            if edge.from_name in graph and edge.to_name in graph:
                graph[edge.from_name].add(edge.to_name)
        return {
            name: sorted(neighbors)
            for name, neighbors in graph.items()
        }

    def _graph_without_edges(self, blocked_edges: set[tuple[str, str]]) -> dict[str, list[str]]:
        if not blocked_edges:
            return self.graph
        return {
            name: [
                neighbor for neighbor in neighbors
                if (name, neighbor) not in blocked_edges
            ]
            for name, neighbors in self.graph.items()
        }

    def _blocked_edges(self, payload: dict[str, Any]) -> set[tuple[str, str]]:
        return self._request_preparer.blocked_edges(payload)

    def _planner_backend(self, fleet_params: dict[str, Any]) -> str:
        return self._backend_selector.from_fleet_params(fleet_params)

    def _planner_backend_for_payload(self, payload: dict[str, Any]) -> str:
        return self._backend_selector.from_payload(
            payload,
            default=self.planner_backend,
        )

    def _reserved_vertex_constraints(self, payload: dict[str, Any]) -> list[tuple[int, str]]:
        return self._request_preparer.reserved_vertex_constraints(
            payload
        )

    def _reserved_edge_constraints(self, payload: dict[str, Any]) -> list[tuple[int, str, str]]:
        return self._request_preparer.reserved_edge_constraints(payload)

    def _reserved_edge_intervals(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        return self._request_preparer.reserved_edge_intervals(payload)

    def _reserved_vertex_intervals(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        return self._request_preparer.reserved_vertex_intervals(payload)

    def _reserved_edge_interval_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, int, str, str, str]]:
        return (
            self._request_preparer.reserved_edge_interval_constraints(
                payload
            )
        )

    def _reserved_vertex_interval_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, int, str, str]]:
        return (
            self._request_preparer.reserved_vertex_interval_constraints(
                payload
            )
        )

    def _reserved_interval_blocked_edges(self, payload: dict[str, Any]) -> set[tuple[str, str]]:
        return self._request_preparer.reserved_interval_blocked_edges(
            payload
        )

    def _interval_seconds_to_ticks(self, start: float, end: float) -> tuple[int, int]:
        return self._request_preparer.interval_seconds_to_ticks(
            start,
            end,
        )

    def _int_value(self, value: Any) -> int | None:
        return self._request_preparer.int_value(value)

    def _float_value(self, value: Any) -> float | None:
        return self._request_preparer.float_value(value)

    def _edge_tick_cost(
        self,
        src: str,
        dst: str,
        speed: float,
        acceleration: float | None = None,
    ) -> int:
        return self._motion_model.edge_tick_cost(
            src,
            dst,
            speed,
            acceleration,
        )

    def _edge_speed(self, edge: GraphEdge, default_speed: float) -> float:
        return self._motion_model.edge_speed(edge, default_speed)

    def _route_speed(self, payload: dict[str, Any]) -> float:
        return self._request_preparer.route_speed(payload)

    def _route_acceleration(self, payload: dict[str, Any]) -> float:
        return self._request_preparer.route_acceleration(payload)

    def _rotate_enabled(self, payload: dict[str, Any]) -> bool:
        return self._request_preparer.rotate_enabled(payload)

    def _turn_speed(self, payload: dict[str, Any]) -> float:
        return self._request_preparer.turn_speed(payload)

    def _stretch_motion_to_reservation_ticks(self, payload: dict[str, Any]) -> bool:
        return self._request_preparer.stretch_motion(payload)

    def _travel_time(
        self,
        distance: float,
        speed: float,
        acceleration: float | None = None,
    ) -> float:
        return self._motion_model.travel_time(
            distance,
            speed,
            acceleration,
        )

    def _heuristic_ticks(self, start_lm: str, goal_lm: str) -> float:
        return self._motion_model.heuristic_ticks(start_lm, goal_lm)

    def _trajectory_for_nodes(
        self,
        nodes: list[str],
        speed: float,
        times: list[int] | None = None,
        *,
        acceleration: float = 0.0,
        rotate_enabled: bool = False,
        turn_speed: float = 0.9,
        stretch_motion_to_reservation_ticks: bool | None = None,
        start_yaw: float = 0.0,
        yaws: list[float] | None = None,
        actions: list[str] | None = None,
    ) -> list[dict[str, float | str]]:
        return self._trajectory_builder.build(
            nodes,
            speed,
            times,
            acceleration=acceleration,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            stretch_motion_to_reservation_ticks=(
                stretch_motion_to_reservation_ticks
            ),
            start_yaw=start_yaw,
            yaws=yaws,
            actions=actions,
        )

    def _planned_segment_duration(self, times: list[int] | None, index: int) -> float | None:
        return self._trajectory_builder.planned_segment_duration(
            times,
            index,
        )

    def _validate_start_pose_at_lm(
        self,
        robot_name: str,
        start_lm: str,
        start_pose: dict[str, float],
    ) -> None:
        return self._request_preparer.validate_start_pose_at_lm(
            robot_name,
            start_lm,
            start_pose,
        )

    def _rotation_duration(self, from_yaw: float, to_yaw: float, turn_speed: float) -> float:
        return self._motion_model.rotation_duration(
            from_yaw,
            to_yaw,
            turn_speed,
        )

    def _rotation_tick_cost(self, from_yaw: float, to_yaw: float, turn_speed: float) -> int:
        return self._motion_model.rotation_tick_cost(
            from_yaw,
            to_yaw,
            turn_speed,
        )

    def _edge_heading(self, from_lm: str, to_lm: str) -> float:
        return self._motion_model.edge_heading(from_lm, to_lm)

    def _edge_heading_options(
        self,
        from_lm: str,
        to_lm: str,
    ) -> tuple[float, ...]:
        return self._motion_model.edge_heading_options(
            from_lm,
            to_lm,
        )

    def _normalize_angle(self, value: float) -> float:
        return self._motion_model.normalize_angle(value)

    def _direct_route(self, from_lm: str, to_lm: str) -> PlannedRoute:
        return self._trajectory_builder.direct_route(from_lm, to_lm)

    def _annotate_sample_distances(
        self,
        samples: list[dict[str, float | str]],
    ) -> list[dict[str, float | str]]:
        return self._trajectory_builder.annotate_sample_distances(
            samples
        )
