"""Backend selection and execution for :class:`FleetMapfPlanner`."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Any

from ..cbs.lm_cbs import LmCBSPlanner, LmRobotRequest
from ..rolling.rolling_sipp import RollingSippPlanner


LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class _BackendRequest:
    robots: list[LmRobotRequest]
    blocked_lms: set[str]
    blocked_edges: set[tuple[str, str]]
    detour_blocked_edges: set[tuple[str, str]]
    speed: float
    acceleration: float
    reserved_vertex_constraints: list[tuple[int, str]]
    reserved_edge_constraints: list[tuple[int, str, str]]
    reserved_vertex_intervals: list[tuple[int, int, str, str]]
    reserved_edge_intervals: list[tuple[int, int, str, str, str]]
    reserved_interval_edges: set[tuple[str, str]]
    low_level_max_time: int
    rotate_enabled: bool
    turn_speed: float


class BackendSelector:
    """Normalize configuration and request-level backend aliases."""

    def __init__(self, *, strict: bool = False) -> None:
        self.strict = bool(strict)

    def normalize(
        self,
        value: Any,
        *,
        path: str = "fleet.planner_backend",
    ) -> str:
        backend = str(value or "cbs").strip().lower()
        if backend in {"rolling-sipp", "rolling_sipp", "sipp"}:
            return "rolling_sipp"
        if backend in {"hybrid", "rolling_sipp+cbs", "sipp+cbs"}:
            return "hybrid"
        if backend != "cbs":
            message = f"{path}: unknown backend, received {value!r}"
            if self.strict:
                raise ValueError(message)
            LOGGER.warning(
                "configuration_compatibility: %s; using cbs",
                message,
            )
        return "cbs"

    def from_fleet_params(self, fleet_params: dict[str, Any]) -> str:
        return self.normalize(
            fleet_params.get("planner_backend")
            or fleet_params.get("plannerBackend")
            or fleet_params.get("mapf_backend")
            or "cbs",
            path="fleet.planner_backend",
        )

    def from_payload(
        self,
        payload: dict[str, Any],
        *,
        default: str,
    ) -> str:
        override = (
            payload.get("plannerBackend")
            or payload.get("planner_backend")
        )
        return (
            default
            if override is None
            else self.normalize(override, path="payload.plannerBackend")
        )


class BackendRunner:
    """Run Rolling-SIPP/CBS while keeping fallback policy in one component."""

    def __init__(self, planner: Any) -> None:
        self.planner = planner

    def run_selected(
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
    ) -> tuple[Any, set[tuple[str, str]], bool, str]:
        request = _BackendRequest(
            robots=requests,
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
        )
        if selected_backend in {"rolling_sipp", "hybrid"}:
            return self._run_rolling_selection(
                request,
                selected_backend=selected_backend,
                allow_cbs_fallback=allow_cbs_fallback,
            )
        return self._run_cbs_selection(request)

    def _run_rolling_selection(
        self,
        request: _BackendRequest,
        *,
        selected_backend: str,
        allow_cbs_fallback: bool,
    ) -> tuple[Any, set[tuple[str, str]], bool, str]:
        result = self.planner._run_rolling_sipp(
            request.robots,
            blocked_lms=request.blocked_lms,
            blocked_edges=request.detour_blocked_edges,
            speed=request.speed,
            acceleration=request.acceleration,
            reserved_vertex_constraints=(
                request.reserved_vertex_constraints
            ),
            reserved_edge_constraints=request.reserved_edge_constraints,
            reserved_vertex_intervals=request.reserved_vertex_intervals,
            reserved_edge_intervals=request.reserved_edge_intervals,
            low_level_max_time=request.low_level_max_time,
            rotate_enabled=request.rotate_enabled,
            turn_speed=request.turn_speed,
        )
        if result.plans or selected_backend == "rolling_sipp":
            if result.plans and request.reserved_interval_edges:
                result.debug.reason = (
                    f"{result.debug.reason}:reserved_edge_detour"
                )
            return (
                result,
                request.detour_blocked_edges,
                bool(result.plans)
                and bool(request.reserved_interval_edges),
                "",
            )
        if not allow_cbs_fallback:
            return result, request.detour_blocked_edges, False, ""
        if self._local_cbs_is_too_large(result, request):
            return result, request.detour_blocked_edges, False, ""
        return self._hybrid_fallback(request, result)

    def _local_cbs_is_too_large(
        self,
        rolling_result: Any,
        request: _BackendRequest,
    ) -> bool:
        reason = str(rolling_result.debug.reason or "")
        is_priority_deadlock = any(
            marker in reason
            for marker in (
                "priority_cycle",
                "priority_repair_limit",
            )
        )
        return (
            is_priority_deadlock
            and len(request.robots) > self.planner.local_cbs_max_robots
        )

    def _hybrid_fallback(
        self,
        request: _BackendRequest,
        rolling_result: Any,
    ) -> tuple[Any, set[tuple[str, str]], bool, str]:
        rolling_reason = rolling_result.debug.reason
        rolling_blockers = tuple(
            rolling_result.debug.blocking_robots
        )
        rolling_reservations = tuple(
            rolling_result.debug.blocking_reservations
        )
        (
            cbs_result,
            used_edges,
            used_detour,
            cbs_fallback,
        ) = self._run_cbs_selection(request)
        cbs_reason = str(cbs_result.debug.reason or "")
        if cbs_result.plans:
            cbs_result.debug.reason = (
                f"{cbs_result.debug.reason}:hybrid_cbs_fallback"
            )
        elif rolling_blockers and not cbs_result.debug.blocking_robots:
            cbs_result.debug.blocking_robots = rolling_blockers
            cbs_result.debug.reason = rolling_reason
        if (
            rolling_reservations
            and not cbs_result.debug.blocking_reservations
        ):
            cbs_result.debug.blocking_reservations = (
                rolling_reservations
            )
        fallback_reason = (
            f"rolling_sipp:{rolling_reason};cbs:{cbs_reason}"
        )
        if cbs_fallback:
            fallback_reason = (
                f"{fallback_reason};"
                f"cbs_reserved_detour:{cbs_fallback}"
            )
        return (
            cbs_result,
            used_edges,
            used_detour,
            fallback_reason,
        )

    def _run_cbs_selection(
        self,
        request: _BackendRequest,
    ) -> tuple[Any, set[tuple[str, str]], bool, str]:
        return self.planner._run_cbs_with_reserved_detour(
            request.robots,
            blocked_lms=request.blocked_lms,
            blocked_edges=request.blocked_edges,
            detour_blocked_edges=request.detour_blocked_edges,
            speed=request.speed,
            acceleration=request.acceleration,
            reserved_vertex_constraints=(
                request.reserved_vertex_constraints
            ),
            reserved_edge_constraints=request.reserved_edge_constraints,
            reserved_vertex_intervals=request.reserved_vertex_intervals,
            reserved_edge_intervals=request.reserved_edge_intervals,
            reserved_interval_edges=request.reserved_interval_edges,
            low_level_max_time=request.low_level_max_time,
            rotate_enabled=request.rotate_enabled,
            turn_speed=request.turn_speed,
        )

    def run_cbs_with_reserved_detour(
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
    ) -> tuple[Any, set[tuple[str, str]], bool, str]:
        planner = self.planner
        result = planner._run_cbs(
            requests,
            blocked_lms=blocked_lms,
            blocked_edges=detour_blocked_edges,
            speed=speed,
            acceleration=acceleration,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
            low_level_max_time=low_level_max_time,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
        )
        used_blocked_edges = detour_blocked_edges
        used_reserved_detour = (
            bool(result.plans) and bool(reserved_interval_edges)
        )
        fallback_reason = ""
        if (
            not result.plans
            and reserved_interval_edges
            and detour_blocked_edges != blocked_edges
        ):
            fallback_reason = result.debug.reason
            used_blocked_edges = blocked_edges
            result = planner._run_cbs(
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
            )
            if result.plans:
                result.debug.reason = (
                    f"{result.debug.reason}:"
                    "reserved_interval_fallback_wait"
                )
                used_reserved_detour = False

        if used_reserved_detour:
            result.debug.reason = (
                f"{result.debug.reason}:reserved_edge_detour"
            )
        return (
            result,
            used_blocked_edges,
            used_reserved_detour,
            fallback_reason,
        )

    def run_cbs(
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
    ) -> Any:
        owner = self.planner
        motion = owner._motion_model
        graph = owner._graph_without_edges(blocked_edges)
        traffic_graph = owner._traffic_graph(speed)

        def lane_resources(src: str, dst: str) -> tuple[object, ...]:
            lane = traffic_graph.lane_for(src, dst)
            if lane is None:
                return ()
            return traffic_graph.lane_resources(lane)

        planner = LmCBSPlanner(
            graph,
            heuristic_fn=motion.heuristic_ticks,
            move_cost_fn=lambda src, dst: motion.edge_tick_cost(
                src,
                dst,
                speed,
                acceleration,
            ),
            heading_fn=motion.edge_heading,
            heading_options_fn=motion.edge_heading_options,
            turn_cost_fn=(
                lambda from_yaw, to_yaw: motion.rotation_tick_cost(
                    from_yaw,
                    to_yaw,
                    turn_speed,
                )
                if rotate_enabled
                else 0
            ),
            vertex_resources_fn=traffic_graph.vertex_resources,
            rotation_resources_fn=traffic_graph.rotation_resources,
            lane_resources_fn=lane_resources,
            can_wait_fn=lambda node: bool(
                traffic_graph.vertices.get(node) is None
                or traffic_graph.vertices[node].can_wait
            ),
            low_level_max_time=low_level_max_time,
            max_high_level_nodes=owner.max_high_level_nodes,
            max_planning_time_sec=owner.max_planning_time_sec,
            wait_cost=owner.wait_cost,
        )
        reserved_resources: set[tuple[int, int, object]] = set()
        for tick, node in reserved_vertex_constraints:
            for resource in traffic_graph.vertex_resources(node):
                reserved_resources.add((tick, tick, resource))
        for start, end, node, _owner in reserved_vertex_intervals:
            for resource in traffic_graph.vertex_resources(node):
                reserved_resources.add((start, end, resource))
        for tick, src, dst in reserved_edge_constraints:
            for resource in lane_resources(src, dst):
                reserved_resources.add((tick, tick, resource))
        if owner.reserved_edge_hard_constraints_enabled:
            for start, end, src, dst, _owner in reserved_edge_intervals:
                for resource in lane_resources(src, dst):
                    reserved_resources.add((start, end, resource))

        result = planner.plan_for_robots(
            requests,
            blocked_nodes=sorted(blocked_lms),
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=(
                reserved_edge_intervals
                if owner.reserved_edge_hard_constraints_enabled
                else []
            ),
            reserved_resource_intervals=sorted(
                reserved_resources,
                key=lambda item: (item[0], item[1], str(item[2])),
            ),
        )
        if result.plans:
            validator = RollingSippPlanner(
                traffic_graph,
                heuristic_fn=motion.heuristic_ticks,
                move_cost_fn=lambda src, dst: motion.edge_tick_cost(
                    src,
                    dst,
                    speed,
                    acceleration,
                ),
                heading_fn=motion.edge_heading,
                heading_options_fn=motion.edge_heading_options,
                turn_cost_fn=(
                    lambda from_yaw, to_yaw: motion.rotation_tick_cost(
                        from_yaw,
                        to_yaw,
                        turn_speed,
                    )
                    if rotate_enabled
                    else 0
                ),
                low_level_max_time=low_level_max_time,
                wait_cost=owner.wait_cost,
            )
            invalid_reason = validator.validate_plans(
                requests,
                result.plans,
                reserved_vertex_constraints=reserved_vertex_constraints,
                reserved_edge_constraints=reserved_edge_constraints,
                reserved_vertex_intervals=reserved_vertex_intervals,
                reserved_edge_intervals=(
                    reserved_edge_intervals
                    if owner.reserved_edge_hard_constraints_enabled
                    else []
                ),
            )
            if invalid_reason:
                result.plans = {}
                result.debug.reason = f"cbs_{invalid_reason}"
        return result

    def run_rolling_sipp(
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
    ) -> Any:
        owner = self.planner
        motion = owner._motion_model
        traffic_graph = owner._traffic_graph(speed)
        planner = RollingSippPlanner(
            traffic_graph,
            heuristic_fn=motion.heuristic_ticks,
            move_cost_fn=lambda src, dst: motion.edge_tick_cost(
                src,
                dst,
                speed,
                acceleration,
            ),
            heading_fn=motion.edge_heading,
            heading_options_fn=motion.edge_heading_options,
            turn_cost_fn=(
                lambda from_yaw, to_yaw: motion.rotation_tick_cost(
                    from_yaw,
                    to_yaw,
                    turn_speed,
                )
                if rotate_enabled
                else 0
            ),
            low_level_max_time=low_level_max_time,
            wait_cost=owner.wait_cost,
            max_planning_time_sec=owner.max_planning_time_sec,
        )
        return planner.plan_for_robots(
            requests,
            blocked_nodes=sorted(blocked_lms),
            blocked_edges=blocked_edges,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=(
                reserved_edge_intervals
                if owner.reserved_edge_hard_constraints_enabled
                else []
            ),
        )
