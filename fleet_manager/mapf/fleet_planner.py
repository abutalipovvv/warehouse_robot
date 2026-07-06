from __future__ import annotations

import math
from typing import Any

from fleet_manager.route_core import GraphEdge, Landmark, LmRoutePlanner, PlannedRoute

from .lm_cbs import LmCBSPlanner, LmRobotRequest
from .rolling_sipp import RollingSippPlanner
from .traffic_graph import TrafficGraph


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
        self.route_planner = LmRoutePlanner(landmarks, edges, params=params)
        self.graph = self._build_graph()
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
        self.time_step_sec = float(fleet_params.get("reservation_time_step_sec", 1.0))
        self.wait_time_sec = float(fleet_params.get("wait_time_sec", self.time_step_sec))
        self.wait_cost = int(fleet_params.get("wait_cost", 6))
        self.stretch_motion_to_reservation_ticks = bool(
            fleet_params.get("stretch_motion_to_reservation_ticks", False)
        )
        self.reserved_edge_detour_enabled = bool(
            fleet_params.get("reserved_edge_detour_enabled", True)
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
        self.planner_backend = self._planner_backend(fleet_params)

    def plan(self, payload: dict[str, Any]) -> dict[str, Any]:
        robots = payload.get("robots", [])
        if not isinstance(robots, list):
            raise ValueError("robots must be a list")

        speed = self._route_speed(payload)
        acceleration = self._route_acceleration(payload)
        rotate_enabled = self._rotate_enabled(payload)
        turn_speed = self._turn_speed(payload)
        stretch_motion = self._stretch_motion_to_reservation_ticks(payload)
        blocked_edges = self._blocked_edges(payload)
        blocked_lms = {
            str(name)
            for name in payload.get("blocked_lms", [])
            if isinstance(name, str)
        }
        start_poses: dict[str, dict[str, float]] = {}
        requests: list[LmRobotRequest] = []
        for item in robots:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", "")).strip()
            start_lm = str(item.get("startLm") or item.get("currentLm") or "").strip()
            goal_lm = str(item.get("goalLm") or item.get("targetLm") or "").strip()
            if not name or not start_lm or not goal_lm:
                continue
            pose = item.get("startPose")
            if isinstance(pose, dict):
                start_poses[name] = {
                    "x": float(pose.get("x", 0.0) or 0.0),
                    "y": float(pose.get("y", 0.0) or 0.0),
                    "yaw": float(pose.get("yaw", 0.0) or 0.0),
                }
            requests.append(LmRobotRequest(name, start_lm, goal_lm))

        reserved_vertex_constraints = self._reserved_vertex_constraints(payload)
        reserved_edge_constraints = self._reserved_edge_constraints(payload)
        reserved_vertex_intervals = self._reserved_vertex_interval_constraints(payload)
        reserved_edge_intervals = self._reserved_edge_interval_constraints(payload)
        reserved_interval_edges = self._reserved_interval_blocked_edges(payload)
        detour_blocked_edges = (
            blocked_edges | reserved_interval_edges
            if self.reserved_edge_detour_enabled
            else blocked_edges
        )

        result, used_blocked_edges, used_reserved_detour, fallback_reason = self._run_selected_backend(
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
        )

        plans = []
        if result.plans:
            for request in requests:
                plan = result.plans.get(request.robot_name)
                if plan is None:
                    continue
                trajectory = self._trajectory_for_nodes(
                    plan.nodes,
                    speed,
                    plan.times,
                    acceleration=acceleration,
                    rotate_enabled=rotate_enabled,
                    turn_speed=turn_speed,
                    stretch_motion_to_reservation_ticks=stretch_motion,
                )
                trajectory = self._prepend_start_pose_approach(
                    trajectory,
                    start_poses.get(request.robot_name),
                    speed,
                    request.start_lm,
                    acceleration=acceleration,
                    rotate_enabled=rotate_enabled,
                    turn_speed=turn_speed,
                )
                plans.append(
                    {
                        "robot": request.robot_name,
                        "startLm": request.start_lm,
                        "goalLm": request.goal_lm,
                        "nodes": plan.nodes,
                        "times": plan.times,
                        "trajectory": trajectory,
                        "arrivalTime": trajectory[-1]["t"] if trajectory else 0.0,
                    }
                )

        return {
            "ok": bool(result.plans) or not requests,
            "debug": {
                "reason": result.debug.reason,
                "conflictsResolved": result.debug.conflicts_resolved,
                "highLevelNodes": result.debug.high_level_nodes,
                "expandedNodes": result.debug.expanded_nodes,
                "blockedLms": sorted(blocked_lms),
                "blockedEdges": [f"{src}->{dst}" for src, dst in sorted(used_blocked_edges)],
                "hardBlockedEdges": [f"{src}->{dst}" for src, dst in sorted(blocked_edges)],
                "reservedDetourEdges": [f"{src}->{dst}" for src, dst in sorted(reserved_interval_edges)],
                "reservedVertices": len(reserved_vertex_constraints),
                "reservedEdges": len(reserved_edge_constraints),
                "reservedVertexIntervals": len(reserved_vertex_intervals),
                "reservedEdgeIntervals": len(reserved_edge_intervals),
                "reservedDetourEnabled": self.reserved_edge_detour_enabled,
                "reservedFallbackReason": fallback_reason,
                "waitCost": self.wait_cost,
                "plannerBackend": self.planner_backend,
                "routeSpeed": speed,
                "routeAcceleration": acceleration,
                "rotateEnabled": rotate_enabled,
                "turnSpeed": turn_speed,
            },
            "timeStepSec": self.time_step_sec,
            "plans": plans,
        }

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
    ):
        if self.planner_backend in {"rolling_sipp", "hybrid"}:
            result = self._run_rolling_sipp(
                requests,
                blocked_lms=blocked_lms,
                blocked_edges=detour_blocked_edges,
                speed=speed,
                acceleration=acceleration,
                reserved_vertex_constraints=reserved_vertex_constraints,
                reserved_edge_constraints=reserved_edge_constraints,
                reserved_vertex_intervals=reserved_vertex_intervals,
                reserved_edge_intervals=reserved_edge_intervals,
            )
            if result.plans or self.planner_backend == "rolling_sipp":
                if result.plans and reserved_interval_edges:
                    result.debug.reason = f"{result.debug.reason}:reserved_edge_detour"
                return (
                    result,
                    detour_blocked_edges,
                    bool(result.plans) and bool(reserved_interval_edges),
                    "",
                )
            rolling_reason = result.debug.reason
            cbs_result, used_edges, used_detour, cbs_fallback = self._run_cbs_with_reserved_detour(
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
            )
            if cbs_result.plans:
                cbs_result.debug.reason = f"{cbs_result.debug.reason}:hybrid_cbs_fallback"
            fallback_reason = f"rolling_sipp:{rolling_reason}"
            if cbs_fallback:
                fallback_reason = f"{fallback_reason};cbs:{cbs_fallback}"
            return cbs_result, used_edges, used_detour, fallback_reason

        return self._run_cbs_with_reserved_detour(
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
    ):
        result = self._run_cbs(
            requests,
            blocked_lms=blocked_lms,
            blocked_edges=detour_blocked_edges,
            speed=speed,
            acceleration=acceleration,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
        )
        used_blocked_edges = detour_blocked_edges
        used_reserved_detour = bool(result.plans) and bool(reserved_interval_edges)
        fallback_reason = ""
        if (
            not result.plans
            and reserved_interval_edges
            and detour_blocked_edges != blocked_edges
        ):
            fallback_reason = result.debug.reason
            used_blocked_edges = blocked_edges
            result = self._run_cbs(
                requests,
                blocked_lms=blocked_lms,
                blocked_edges=blocked_edges,
                speed=speed,
                acceleration=acceleration,
                reserved_vertex_constraints=reserved_vertex_constraints,
                reserved_edge_constraints=reserved_edge_constraints,
                reserved_vertex_intervals=reserved_vertex_intervals,
                reserved_edge_intervals=reserved_edge_intervals,
            )
            if result.plans:
                result.debug.reason = f"{result.debug.reason}:reserved_interval_fallback_wait"
                used_reserved_detour = False

        if used_reserved_detour:
            result.debug.reason = f"{result.debug.reason}:reserved_edge_detour"

        return result, used_blocked_edges, used_reserved_detour, fallback_reason

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
    ):
        graph = self._graph_without_edges(blocked_edges)
        planner = LmCBSPlanner(
            graph,
            heuristic_fn=self._heuristic_ticks,
            move_cost_fn=lambda src, dst: self._edge_tick_cost(src, dst, speed, acceleration),
            low_level_max_time=self.low_level_max_time,
            max_high_level_nodes=self.max_high_level_nodes,
            max_planning_time_sec=self.max_planning_time_sec,
            wait_cost=self.wait_cost,
        )
        return planner.plan_for_robots(
            requests,
            blocked_nodes=sorted(blocked_lms),
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=(
                reserved_edge_intervals
                if self.reserved_edge_hard_constraints_enabled
                else []
            ),
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
    ):
        traffic_graph = TrafficGraph.from_route_core(
            self.landmarks,
            self.edges,
            default_speed_mps=speed,
        )
        planner = RollingSippPlanner(
            traffic_graph,
            heuristic_fn=self._heuristic_ticks,
            move_cost_fn=lambda src, dst: self._edge_tick_cost(src, dst, speed, acceleration),
            low_level_max_time=self.low_level_max_time,
            wait_cost=self.wait_cost,
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
                if self.reserved_edge_hard_constraints_enabled
                else []
            ),
        )

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
        raw_edges = payload.get("blocked_edges") or payload.get("blockedEdges") or []
        if not isinstance(raw_edges, list):
            return set()
        blocked: set[tuple[str, str]] = set()
        for item in raw_edges:
            if isinstance(item, str) and "->" in item:
                src, dst = item.split("->", 1)
                blocked.add((src.strip(), dst.strip()))
            elif isinstance(item, dict):
                src = str(item.get("from") or item.get("fromLm") or "").strip()
                dst = str(item.get("to") or item.get("toLm") or "").strip()
                if src and dst:
                    blocked.add((src, dst))
        return blocked

    def _planner_backend(self, fleet_params: dict[str, Any]) -> str:
        backend = str(
            fleet_params.get("planner_backend")
            or fleet_params.get("plannerBackend")
            or fleet_params.get("mapf_backend")
            or "cbs"
        ).strip().lower()
        if backend in {"rolling-sipp", "rolling_sipp", "sipp"}:
            return "rolling_sipp"
        if backend in {"hybrid", "rolling_sipp+cbs", "sipp+cbs"}:
            return "hybrid"
        return "cbs"

    def _reserved_vertex_constraints(self, payload: dict[str, Any]) -> list[tuple[int, str]]:
        raw_constraints = (
            payload.get("reserved_vertex_constraints")
            or payload.get("reservedVertexConstraints")
            or []
        )
        if not isinstance(raw_constraints, list):
            return []
        constraints: list[tuple[int, str]] = []
        for item in raw_constraints:
            if isinstance(item, dict):
                time_tick = self._int_value(item.get("time") or item.get("t"))
                node = str(item.get("node") or item.get("lm") or "").strip()
                if time_tick is not None and node:
                    constraints.append((time_tick, node))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                time_tick = self._int_value(item[0])
                node = str(item[1]).strip()
                if time_tick is not None and node:
                    constraints.append((time_tick, node))
        return constraints

    def _reserved_edge_constraints(self, payload: dict[str, Any]) -> list[tuple[int, str, str]]:
        raw_constraints = (
            payload.get("reserved_edge_constraints")
            or payload.get("reservedEdgeConstraints")
            or []
        )
        if not isinstance(raw_constraints, list):
            return []
        constraints: list[tuple[int, str, str]] = []
        for item in raw_constraints:
            if isinstance(item, dict):
                time_tick = self._int_value(item.get("time") or item.get("t"))
                src = str(item.get("from") or item.get("src") or "").strip()
                dst = str(item.get("to") or item.get("dst") or "").strip()
                if time_tick is not None and src and dst:
                    constraints.append((time_tick, src, dst))
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                time_tick = self._int_value(item[0])
                src = str(item[1]).strip()
                dst = str(item[2]).strip()
                if time_tick is not None and src and dst:
                    constraints.append((time_tick, src, dst))
        return constraints

    def _reserved_edge_intervals(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        raw_intervals = (
            payload.get("reserved_edge_intervals")
            or payload.get("reservedEdgeIntervals")
            or []
        )
        if not isinstance(raw_intervals, list):
            return []
        return [
            item for item in raw_intervals
            if isinstance(item, dict)
        ]

    def _reserved_vertex_intervals(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        raw_intervals = (
            payload.get("reserved_vertex_intervals")
            or payload.get("reservedVertexIntervals")
            or []
        )
        if not isinstance(raw_intervals, list):
            return []
        return [
            item for item in raw_intervals
            if isinstance(item, dict)
        ]

    def _reserved_edge_interval_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, int, str, str, str]]:
        constraints: list[tuple[int, int, str, str, str]] = []
        for item in self._reserved_edge_intervals(payload):
            src = str(item.get("from") or item.get("src") or "").strip()
            dst = str(item.get("to") or item.get("dst") or "").strip()
            start = self._float_value(item.get("start") or item.get("startTime") or 0.0)
            end = self._float_value(item.get("end") or item.get("endTime") or 0.0)
            owner = str(item.get("robot") or item.get("owner") or "").strip()
            if not src or not dst or start is None or end is None:
                continue
            start_tick, end_tick = self._interval_seconds_to_ticks(start, end)
            constraints.append((start_tick, end_tick, src, dst, owner))
        return constraints

    def _reserved_vertex_interval_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, int, str, str]]:
        constraints: list[tuple[int, int, str, str]] = []
        for item in self._reserved_vertex_intervals(payload):
            node = str(item.get("node") or item.get("lm") or "").strip()
            start = self._float_value(item.get("start") or item.get("startTime") or 0.0)
            end = self._float_value(item.get("end") or item.get("endTime") or 0.0)
            owner = str(item.get("robot") or item.get("owner") or "").strip()
            if not node or start is None or end is None:
                continue
            start_tick, end_tick = self._interval_seconds_to_ticks(start, end)
            constraints.append((start_tick, end_tick, node, owner))
        return constraints

    def _reserved_interval_blocked_edges(self, payload: dict[str, Any]) -> set[tuple[str, str]]:
        blocked: set[tuple[str, str]] = set()
        if not self.reserved_edge_detour_enabled:
            return blocked
        horizon = max(0.0, self.reservation_detour_horizon_sec)
        for item in self._reserved_edge_intervals(payload):
            src = str(item.get("from") or item.get("src") or "").strip()
            dst = str(item.get("to") or item.get("dst") or "").strip()
            start = self._float_value(item.get("start") or item.get("startTime") or 0.0)
            end = self._float_value(item.get("end") or item.get("endTime") or 0.0)
            if not src or not dst or start is None or end is None:
                continue
            start_s = min(start, end)
            end_s = max(start, end)
            if end_s < 0.0 or start_s > horizon:
                continue
            blocked.add((src, dst))
            blocked.add((dst, src))
        return blocked

    def _interval_seconds_to_ticks(self, start: float, end: float) -> tuple[int, int]:
        start_s = max(0.0, min(float(start), float(end)))
        end_s = max(0.0, max(float(start), float(end)))
        step = max(0.001, self.time_step_sec)
        return math.floor(start_s / step), math.ceil(end_s / step)

    def _int_value(self, value: Any) -> int | None:
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return None

    def _float_value(self, value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _edge_tick_cost(self, src: str, dst: str, speed: float, acceleration: float | None = None) -> int:
        if src == dst:
            return 1
        edge = self.edge_by_key.get((src, dst))
        if edge is None:
            start = self.landmarks.get(src)
            goal = self.landmarks.get(dst)
            if start is None or goal is None:
                return 1
            length = math.hypot(goal.x - start.x, goal.y - start.y)
            edge_speed = speed
        else:
            length = max(float(edge.length), 0.0)
            edge_speed = self._edge_speed(edge, speed)
        travel_time = self._travel_time(length, edge_speed, acceleration)
        return max(1, math.ceil(travel_time / max(self.time_step_sec, 0.001)))

    def _edge_speed(self, edge: GraphEdge, default_speed: float) -> float:
        properties = edge.properties if isinstance(edge.properties, dict) else {}
        raw_speed = None
        for key in ("max_speed", "maxSpeed", "maxspeed", "speed", "speedLimit"):
            if key in properties:
                raw_speed = properties.get(key)
                break
        if raw_speed is None:
            return max(0.02, default_speed)
        try:
            return max(0.02, min(default_speed, float(raw_speed)))
        except (TypeError, ValueError):
            return max(0.02, default_speed)


    def _route_speed(self, payload: dict[str, Any]) -> float:
        raw_speed = payload.get("speed")
        if raw_speed is None:
            navigation = self.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_speed = navigation.get("route_speed")
        try:
            return max(0.02, float(raw_speed))
        except (TypeError, ValueError):
            return 0.35

    def _route_acceleration(self, payload: dict[str, Any]) -> float:
        raw_value = (
            payload.get("acceleration")
            or payload.get("routeAcceleration")
            or payload.get("route_acceleration")
        )
        if raw_value is None:
            navigation = self.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_value = navigation.get("route_acceleration")
        try:
            return max(0.0, float(raw_value))
        except (TypeError, ValueError):
            return 0.0

    def _rotate_enabled(self, payload: dict[str, Any]) -> bool:
        raw_value = (
            payload.get("rotate")
            if "rotate" in payload
            else payload.get("simulateRotation")
            if "simulateRotation" in payload
            else payload.get("simulate_rotation")
        )
        if raw_value is None:
            navigation = self.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_value = navigation.get("simulate_rotation", False)
        if isinstance(raw_value, str):
            return raw_value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(raw_value)

    def _turn_speed(self, payload: dict[str, Any]) -> float:
        raw_value = (
            payload.get("turnSpeed")
            or payload.get("turn_speed")
            or payload.get("rotationSpeed")
            or payload.get("rotation_speed")
        )
        if raw_value is None:
            navigation = self.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_value = navigation.get("turn_speed") or navigation.get("max_angular_speed")
        try:
            return max(0.05, float(raw_value))
        except (TypeError, ValueError):
            return 0.9

    def _stretch_motion_to_reservation_ticks(self, payload: dict[str, Any]) -> bool:
        raw_value = (
            payload.get("stretchMotionToReservationTicks")
            if "stretchMotionToReservationTicks" in payload
            else payload.get("stretch_motion_to_reservation_ticks")
        )
        if raw_value is None:
            return bool(self.stretch_motion_to_reservation_ticks)
        if isinstance(raw_value, str):
            return raw_value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(raw_value)

    def _travel_time(self, distance: float, speed: float, acceleration: float | None = None) -> float:
        distance = max(0.0, float(distance or 0.0))
        speed = max(0.02, float(speed or 0.02))
        acceleration = max(0.0, float(acceleration or 0.0))
        if distance <= 0.0:
            return 0.0
        if acceleration <= 0.0:
            return distance / speed
        ramp_distance = (speed * speed) / acceleration
        if distance <= ramp_distance:
            return 2.0 * math.sqrt(distance / acceleration)
        return (2.0 * speed / acceleration) + ((distance - ramp_distance) / speed)

    def _heuristic_ticks(self, start_lm: str, goal_lm: str) -> float:
        key = (start_lm, goal_lm)
        if key in self._heuristic_cache:
            return self._heuristic_cache[key]
        try:
            route = self.route_planner.find_route(start_lm, goal_lm)
        except ValueError:
            return 0.0
        if route.length <= 0:
            return 0.0
        speed = self._route_speed({})
        acceleration = self._route_acceleration({})
        value = max(1.0, self._travel_time(route.length, speed, acceleration) / max(self.time_step_sec, 1e-6))
        self._heuristic_cache[key] = value
        return value

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
    ) -> list[dict[str, float | str]]:
        if not nodes:
            return []

        first = self.landmarks[nodes[0]]
        trajectory: list[dict[str, float | str]] = [
            {
                "t": 0.0,
                "x": first.x,
                "y": first.y,
                "yaw": 0.0,
                "edgeId": f"{first.name}->{first.name}",
                "lm": first.name,
                "motionDirection": "not_specified",
            }
        ]
        current_time = 0.0
        last_yaw = 0.0

        for index in range(1, len(nodes)):
            from_lm = nodes[index - 1]
            to_lm = nodes[index]
            planned_duration = self._planned_segment_duration(times, index)
            if from_lm == to_lm:
                current_time += max(0.05, planned_duration or self.wait_time_sec)
                landmark = self.landmarks[from_lm]
                trajectory.append(
                    {
                        "t": current_time,
                        "x": landmark.x,
                        "y": landmark.y,
                        "yaw": last_yaw,
                        "edgeId": f"{from_lm}->{from_lm}",
                        "lm": from_lm,
                        "motionDirection": "not_specified",
                    }
                )
                continue

            route = self._direct_route(from_lm, to_lm)
            samples = self.route_planner.sample_route(route)
            segment = self._annotate_sample_distances(samples)
            segment_length = max(segment[-1]["s"] if segment else 0.0, 1e-6)
            segment_yaw = float(segment[1]["yaw"] if len(segment) > 1 else segment[0]["yaw"])
            if rotate_enabled:
                rotate_duration = self._rotation_duration(last_yaw, segment_yaw, turn_speed)
                if rotate_duration > 0.001:
                    current_time += rotate_duration
                    anchor = trajectory[-1]
                    trajectory.append(
                        {
                            "t": current_time,
                            "x": float(anchor.get("x", 0.0) or 0.0),
                            "y": float(anchor.get("y", 0.0) or 0.0),
                            "yaw": segment_yaw,
                            "edgeId": f"WAIT@ROTATE:{from_lm}->{to_lm}",
                            "lm": from_lm,
                            "motionDirection": "rotate",
                        }
                    )
                    last_yaw = segment_yaw
            continuous_duration = self._travel_time(segment_length, speed, acceleration)
            stretch_motion = self.stretch_motion_to_reservation_ticks if stretch_motion_to_reservation_ticks is None else stretch_motion_to_reservation_ticks
            if stretch_motion:
                duration = max(continuous_duration, planned_duration or 0.0, 0.05)
            else:
                duration = max(continuous_duration, 0.05)

            for sample in segment[1:]:
                t = current_time + (float(sample["s"]) / segment_length) * duration
                last_yaw = float(sample["yaw"])
                trajectory.append(
                    {
                        "t": t,
                        "x": float(sample["x"]),
                        "y": float(sample["y"]),
                        "yaw": last_yaw,
                        "edgeId": str(sample["edgeId"]),
                        "motionDirection": str(sample.get("motionDirection", "not_specified")),
                    }
                )
            current_time += duration

        if trajectory:
            trajectory[-1]["lm"] = nodes[-1]
        return trajectory

    def _planned_segment_duration(self, times: list[int] | None, index: int) -> float | None:
        if not times or index <= 0 or index >= len(times):
            return None
        return max(0.0, (int(times[index]) - int(times[index - 1])) * self.time_step_sec)

    def _prepend_start_pose_approach(
        self,
        trajectory: list[dict[str, float | str]],
        start_pose: dict[str, float] | None,
        speed: float,
        start_lm: str,
        *,
        acceleration: float = 0.0,
        rotate_enabled: bool = False,
        turn_speed: float = 0.9,
    ) -> list[dict[str, float | str]]:
        if not trajectory or start_pose is None:
            return trajectory

        first = trajectory[0]
        start_x = float(start_pose.get("x", 0.0) or 0.0)
        start_y = float(start_pose.get("y", 0.0) or 0.0)
        start_yaw = float(start_pose.get("yaw", 0.0) or 0.0)
        goal_x = float(first.get("x", 0.0) or 0.0)
        goal_y = float(first.get("y", 0.0) or 0.0)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance <= 0.03:
            trajectory[0] = {**first, "yaw": start_yaw}
            return trajectory

        yaw = math.atan2(goal_y - start_y, goal_x - start_x)
        sample_distance = max(0.04, self.route_planner.default_sample_distance)
        steps = max(1, math.ceil(distance / sample_distance))
        move_duration = max(self._travel_time(distance, speed, acceleration), 0.05)
        rotate_duration = self._rotation_duration(start_yaw, yaw, turn_speed) if rotate_enabled else 0.0
        approach: list[dict[str, float | str]] = []
        if rotate_duration > 0.001:
            approach.append(
                {
                    "t": 0.0,
                    "x": start_x,
                    "y": start_y,
                    "yaw": start_yaw,
                    "edgeId": f"WAIT@ROTATE:CURRENT->{start_lm}",
                    "motionDirection": "rotate",
                }
            )
            approach.append(
                {
                    "t": rotate_duration,
                    "x": start_x,
                    "y": start_y,
                    "yaw": yaw,
                    "edgeId": f"WAIT@ROTATE:CURRENT->{start_lm}",
                    "motionDirection": "rotate",
                }
            )
        for step in range(steps + 1):
            ratio = step / steps
            approach.append(
                {
                    "t": rotate_duration + (move_duration * ratio),
                    "x": start_x + ((goal_x - start_x) * ratio),
                    "y": start_y + ((goal_y - start_y) * ratio),
                    "yaw": yaw if (step > 0 or rotate_duration > 0.001) else start_yaw,
                    "edgeId": f"CURRENT->{start_lm}",
                    "motionDirection": "not_specified",
                }
            )
        shifted = [
            {
                **sample,
                "t": float(sample.get("t", 0.0) or 0.0) + rotate_duration + move_duration,
            }
            for sample in trajectory[1:]
        ]
        return approach + shifted

    def _rotation_duration(self, from_yaw: float, to_yaw: float, turn_speed: float) -> float:
        delta = abs(self._normalize_angle(float(to_yaw or 0.0) - float(from_yaw or 0.0)))
        if delta < math.radians(2.0):
            return 0.0
        return delta / max(0.05, float(turn_speed or 0.05))

    def _normalize_angle(self, value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def _direct_route(self, from_lm: str, to_lm: str) -> PlannedRoute:
        edge = self.edge_by_key.get((from_lm, to_lm))
        if edge is not None:
            return PlannedRoute(nodes=[from_lm, to_lm], edges=[edge], length=edge.length)
        return self.route_planner.find_route(from_lm, to_lm)

    def _annotate_sample_distances(
        self,
        samples: list[dict[str, float | str]],
    ) -> list[dict[str, float | str]]:
        distance = 0.0
        annotated: list[dict[str, float | str]] = []
        for index, sample in enumerate(samples):
            if index > 0:
                prev = samples[index - 1]
                distance += math.hypot(
                    float(sample["x"]) - float(prev["x"]),
                    float(sample["y"]) - float(prev["y"]),
                )
            annotated.append({**sample, "s": distance})
        return annotated
