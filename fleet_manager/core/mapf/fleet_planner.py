from __future__ import annotations

import heapq
import math
from typing import Any

from fleet_manager.core.route_core.models import GraphEdge, Landmark, PlannedRoute
from fleet_manager.core.route_core.planner import LmRoutePlanner

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

    def plan(self, payload: dict[str, Any]) -> dict[str, Any]:
        robots = payload.get("robots", [])
        if not isinstance(robots, list):
            raise ValueError("robots must be a list")

        speed = self._route_speed(payload)
        acceleration = self._route_acceleration(payload)
        rotate_enabled = self._rotate_enabled(payload)
        turn_speed = self._turn_speed(payload)
        stretch_motion = self._stretch_motion_to_reservation_ticks(payload)
        low_level_max_time = self._payload_low_level_max_time(payload)
        allow_cbs_fallback = bool(payload.get("allowCbsFallback", True))
        reserved_detour_enabled = bool(
            payload.get("reservedEdgeDetourEnabled", self.reserved_edge_detour_enabled)
        )
        blocked_edges = self._blocked_edges(payload)
        blocked_lms = {
            str(name)
            for name in payload.get("blocked_lms", [])
            if isinstance(name, str)
        }
        requests: list[LmRobotRequest] = []
        start_yaws: dict[str, float] = {}
        seen_robot_names: set[str] = set()
        for index, item in enumerate(robots):
            if not isinstance(item, dict):
                raise ValueError(f"robots[{index}] must be an object")
            name = str(item.get("name", "")).strip()
            start_lm = str(item.get("startLm") or item.get("currentLm") or "").strip()
            goal_lm = str(item.get("goalLm") or item.get("targetLm") or "").strip()
            if not name or not start_lm or not goal_lm:
                raise ValueError(
                    f"robots[{index}] requires non-empty name, startLm/currentLm, and goalLm/targetLm"
                )
            if name in seen_robot_names:
                raise ValueError(f"duplicate robot name: {name}")
            seen_robot_names.add(name)
            pose = item.get("startPose")
            start_yaw = 0.0
            if isinstance(pose, dict):
                clean_pose = {
                    "x": float(pose.get("x", 0.0) or 0.0),
                    "y": float(pose.get("y", 0.0) or 0.0),
                    "yaw": float(pose.get("yaw", 0.0) or 0.0),
                }
                self._validate_start_pose_at_lm(name, start_lm, clean_pose)
                start_yaw = clean_pose["yaw"]
            start_yaws[name] = start_yaw
            route_nodes = self._request_route_nodes(
                item,
                robot_name=name,
                start_lm=start_lm,
                goal_lm=goal_lm,
            )
            try:
                start_not_before_sec = max(
                    0.0,
                    float(item.get("startNotBeforeSec", 0.0) or 0.0),
                )
            except (TypeError, ValueError):
                start_not_before_sec = 0.0
            start_not_before_tick = int(
                math.ceil(
                    start_not_before_sec
                    / max(0.001, self.time_step_sec)
                    - 1e-9
                )
            )
            node_departure_not_before: list[tuple[str, int]] = []
            raw_node_gates = item.get("departureNotBefore", ())
            if isinstance(raw_node_gates, dict):
                raw_node_gates = [
                    {"node": node, "timeSec": value}
                    for node, value in raw_node_gates.items()
                ]
            if isinstance(raw_node_gates, (list, tuple)):
                for raw_gate in raw_node_gates:
                    if not isinstance(raw_gate, dict):
                        continue
                    node = str(
                        raw_gate.get("node")
                        or raw_gate.get("lm")
                        or ""
                    ).strip()
                    if not node:
                        continue
                    if route_nodes and node not in route_nodes:
                        raise ValueError(
                            f"robots[{index}] departure gate node {node!r} "
                            "is outside routeNodes"
                        )
                    try:
                        time_sec = max(
                            0.0,
                            float(
                                raw_gate.get("timeSec")
                                or raw_gate.get("notBeforeSec")
                                or 0.0
                            ),
                        )
                    except (TypeError, ValueError):
                        time_sec = 0.0
                    node_departure_not_before.append(
                        (
                            node,
                            int(
                                math.ceil(
                                    time_sec
                                    / max(0.001, self.time_step_sec)
                                    - 1e-9
                                )
                            ),
                        )
                    )
            raw_authorized_regions = item.get(
                "authorizedControlledRegions",
                item.get("authorized_controlled_regions", ()),
            )
            if raw_authorized_regions is None:
                raw_authorized_regions = ()
            if not isinstance(raw_authorized_regions, (list, tuple, set)):
                raise ValueError(
                    f"robots[{index}] authorizedControlledRegions must be a list"
                )
            authorized_controlled_regions = tuple(dict.fromkeys(
                str(region_id).strip()
                for region_id in raw_authorized_regions
                if str(region_id).strip()
            ))
            raw_no_wait_nodes = item.get(
                "noWaitNodes",
                item.get("no_wait_nodes", ()),
            )
            if raw_no_wait_nodes is None:
                raw_no_wait_nodes = ()
            if not isinstance(raw_no_wait_nodes, (list, tuple, set)):
                raise ValueError(
                    f"robots[{index}] noWaitNodes must be a list"
                )
            no_wait_nodes = tuple(dict.fromkeys(
                str(node).strip()
                for node in raw_no_wait_nodes
                if str(node).strip()
            ))
            if route_nodes:
                outside_route = set(no_wait_nodes) - set(route_nodes)
                if outside_route:
                    raise ValueError(
                        f"robots[{index}] noWaitNodes contains node(s) "
                        "outside routeNodes: "
                        + ", ".join(sorted(outside_route))
                    )
            if authorized_controlled_regions:
                if not node_departure_not_before:
                    raise ValueError(
                        f"robots[{index}] corridor authority requires "
                        "departureNotBefore"
                    )
                if not route_nodes:
                    raise ValueError(
                        f"robots[{index}] corridor authority requires routeNodes"
                    )
                traffic_graph = self._traffic_graph(speed)
                route_regions = {
                    region_id
                    for src, dst in zip(route_nodes, route_nodes[1:])
                    for lane in (traffic_graph.lane_for(src, dst),)
                    if lane is not None
                    for region_id in lane.controlled_region_ids
                }
                unauthorized = (
                    set(authorized_controlled_regions) - route_regions
                )
                if unauthorized:
                    raise ValueError(
                        f"robots[{index}] corridor authority contains region(s) "
                        "outside routeNodes: "
                        + ", ".join(sorted(unauthorized))
                    )
            requests.append(
                LmRobotRequest(
                    name,
                    start_lm,
                    goal_lm,
                    start_yaw,
                    route_nodes,
                    start_not_before_tick,
                    tuple(node_departure_not_before),
                    authorized_controlled_regions,
                    no_wait_nodes,
                )
            )

        reserved_vertex_constraints = self._reserved_vertex_constraints(payload)
        reserved_edge_constraints = self._reserved_edge_constraints(payload)
        reserved_vertex_intervals = self._reserved_vertex_interval_constraints(payload)
        reserved_edge_intervals = self._reserved_edge_interval_constraints(payload)
        reserved_interval_edges = self._reserved_interval_blocked_edges(payload)
        detour_blocked_edges = (
            blocked_edges | reserved_interval_edges
            if reserved_detour_enabled
            else blocked_edges
        )
        if any(
            request.start_not_before_tick > 0
            or bool(request.node_departure_not_before)
            for request in requests
        ):
            # CBS currently models every agent at t=0. A corridor slot's
            # delayed start is therefore a Rolling-SIPP-only temporal
            # contract; silently falling back to CBS would erase it.
            allow_cbs_fallback = False

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
            low_level_max_time=low_level_max_time,
            allow_cbs_fallback=allow_cbs_fallback,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            selected_backend=self._planner_backend_for_payload(payload),
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
                    start_yaw=start_yaws.get(request.robot_name, request.start_yaw),
                    yaws=plan.yaws,
                    actions=plan.actions,
                )
                plans.append(
                    {
                        "robot": request.robot_name,
                        "startLm": request.start_lm,
                        "goalLm": request.goal_lm,
                        "nodes": plan.nodes,
                        "times": plan.times,
                        "yaws": plan.yaws,
                        "actions": plan.actions,
                        "timedSegments": self._timed_segments_for_nodes(
                            plan.nodes,
                            plan.times,
                            plan.actions,
                        ),
                        "trajectory": trajectory,
                        "arrivalTime": trajectory[-1]["t"] if trajectory else 0.0,
                    }
                )

        deadlock = any(
            marker in str(result.debug.reason or "")
            for marker in ("priority_cycle", "priority_repair_limit")
        )
        traffic_graph = self._traffic_graph(speed)
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
                "reservedDetourEnabled": reserved_detour_enabled,
                "reservedFallbackReason": fallback_reason,
                "waitCost": self.wait_cost,
                "plannerBackend": self._planner_backend_for_payload(payload),
                "lowLevelMaxTime": low_level_max_time,
                "cbsFallbackAllowed": allow_cbs_fallback,
                "routeSpeed": speed,
                "routeAcceleration": acceleration,
                "rotateEnabled": rotate_enabled,
                "turnSpeed": turn_speed,
                "deadlock": deadlock,
                "reservationBlockerRobots": list(
                    result.debug.blocking_robots
                ),
                "reservationBlockers": [
                    {
                        "robot": robot_name,
                        "resource": resource,
                    }
                    for robot_name, resource
                    in result.debug.blocking_reservations
                ],
                "controlledCorridors": len(traffic_graph.controlled_region_ids()),
                "controlledCorridorsMode": self.controlled_corridors_mode,
                "controlledCorridorAutoDetect": (
                    self.controlled_corridor_auto_detect
                ),
                "deadlockReason": (
                    "cyclic traffic dependencies could not be safely ordered"
                    if deadlock
                    else ""
                ),
            },
            "timeStepSec": self.time_step_sec,
            "plans": plans,
        }

    def _timed_segments_for_nodes(
        self,
        nodes: list[str],
        times: list[int],
        actions: list[str] | None = None,
    ) -> list[dict[str, Any]]:
        segments: list[dict[str, Any]] = []
        for index in range(1, min(len(nodes), len(times))):
            start_tick = int(times[index - 1])
            end_tick = int(times[index])
            start_node = nodes[index - 1]
            end_node = nodes[index]
            action = (
                str(actions[index]).strip().lower()
                if actions is not None and index < len(actions)
                else ""
            )
            base = {
                "startTick": start_tick,
                "endTick": end_tick,
                "notBeforeSec": start_tick * self.time_step_sec,
                "plannedArrivalSec": end_tick * self.time_step_sec,
            }
            if start_node == end_node:
                segments.append(
                    {
                        **base,
                        "kind": "rotate" if action == "rotate" else "wait",
                        "node": start_node,
                    }
                )
            else:
                edge = self.edge_by_key.get((start_node, end_node))
                segments.append(
                    {
                        **base,
                        "kind": "move",
                        "from": start_node,
                        "to": end_node,
                        "motionDirection": (
                            edge.motion_direction_label(edge.motion_direction_code())
                            if edge is not None
                            else "not_specified"
                        ),
                    }
                )
        return segments

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
    ):
        if selected_backend in {"rolling_sipp", "hybrid"}:
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
                low_level_max_time=low_level_max_time,
                rotate_enabled=rotate_enabled,
                turn_speed=turn_speed,
            )
            if result.plans or selected_backend == "rolling_sipp":
                if result.plans and reserved_interval_edges:
                    result.debug.reason = f"{result.debug.reason}:reserved_edge_detour"
                return (
                    result,
                    detour_blocked_edges,
                    bool(result.plans) and bool(reserved_interval_edges),
                    "",
                )
            if not allow_cbs_fallback:
                return result, detour_blocked_edges, False, ""
            rolling_reason = result.debug.reason
            rolling_blockers = tuple(result.debug.blocking_robots)
            rolling_reservations = tuple(
                result.debug.blocking_reservations
            )
            if any(
                marker in str(rolling_reason or "")
                for marker in ("priority_cycle", "priority_repair_limit")
            ) and len(requests) > self.local_cbs_max_robots:
                return result, detour_blocked_edges, False, ""
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
                low_level_max_time=low_level_max_time,
                rotate_enabled=rotate_enabled,
                turn_speed=turn_speed,
            )
            cbs_reason = str(cbs_result.debug.reason or "")
            if cbs_result.plans:
                cbs_result.debug.reason = f"{cbs_result.debug.reason}:hybrid_cbs_fallback"
            elif rolling_blockers and not cbs_result.debug.blocking_robots:
                # CBS sees the same external reservation resources, but its
                # low-level constraints do not retain owner strings. Preserve
                # the complete owner/resource pair discovered by the preceding
                # SIPP pass. Keeping CBS's unrelated final resource together
                # with SIPP's owner would manufacture a false wait dependency.
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
                    f"{fallback_reason};cbs_reserved_detour:{cbs_fallback}"
                )
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
            low_level_max_time=low_level_max_time,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
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
            low_level_max_time=low_level_max_time,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
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
                low_level_max_time=low_level_max_time,
                rotate_enabled=rotate_enabled,
                turn_speed=turn_speed,
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
        low_level_max_time: int,
        rotate_enabled: bool,
        turn_speed: float,
    ):
        graph = self._graph_without_edges(blocked_edges)
        traffic_graph = self._traffic_graph(speed)

        def lane_resources(src: str, dst: str) -> tuple[object, ...]:
            lane = traffic_graph.lane_for(src, dst)
            if lane is None:
                return ()
            return traffic_graph.lane_resources(lane)

        planner = LmCBSPlanner(
            graph,
            heuristic_fn=self._heuristic_ticks,
            move_cost_fn=lambda src, dst: self._edge_tick_cost(src, dst, speed, acceleration),
            heading_fn=self._edge_heading,
            heading_options_fn=self._edge_heading_options,
            turn_cost_fn=(
                lambda from_yaw, to_yaw: self._rotation_tick_cost(
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
            max_high_level_nodes=self.max_high_level_nodes,
            max_planning_time_sec=self.max_planning_time_sec,
            wait_cost=self.wait_cost,
        )
        # CBS must see the same compound resources as the SIPP validator.
        # A reserved internal LM occupies its whole controlled corridor, and a
        # reserved edge also consumes lane-group/endpoint clearance resources.
        # Previously CBS constrained only the literal node/edge, produced a
        # nominal solution through that region, and the validator rejected it
        # as ``cbs_resource_conflict`` on every retry.
        reserved_resource_intervals: set[tuple[int, int, object]] = set()
        for tick, node in reserved_vertex_constraints:
            for resource in traffic_graph.vertex_resources(node):
                reserved_resource_intervals.add((tick, tick, resource))
        for start, end, node, _owner in reserved_vertex_intervals:
            for resource in traffic_graph.vertex_resources(node):
                reserved_resource_intervals.add((start, end, resource))
        for tick, src, dst in reserved_edge_constraints:
            for resource in lane_resources(src, dst):
                reserved_resource_intervals.add((tick, tick, resource))
        if self.reserved_edge_hard_constraints_enabled:
            for start, end, src, dst, _owner in reserved_edge_intervals:
                for resource in lane_resources(src, dst):
                    reserved_resource_intervals.add((start, end, resource))
        result = planner.plan_for_robots(
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
            reserved_resource_intervals=sorted(
                reserved_resource_intervals,
                key=lambda item: (item[0], item[1], str(item[2])),
            ),
        )
        if result.plans:
            validator = RollingSippPlanner(
                traffic_graph,
                heuristic_fn=self._heuristic_ticks,
                move_cost_fn=lambda src, dst: self._edge_tick_cost(src, dst, speed, acceleration),
                heading_fn=self._edge_heading,
                heading_options_fn=self._edge_heading_options,
                turn_cost_fn=(
                    lambda from_yaw, to_yaw: self._rotation_tick_cost(
                        from_yaw,
                        to_yaw,
                        turn_speed,
                    )
                    if rotate_enabled
                    else 0
                ),
                low_level_max_time=low_level_max_time,
                wait_cost=self.wait_cost,
            )
            invalid_reason = validator.validate_plans(
                requests,
                result.plans,
                reserved_vertex_constraints=reserved_vertex_constraints,
                reserved_edge_constraints=reserved_edge_constraints,
                reserved_vertex_intervals=reserved_vertex_intervals,
                reserved_edge_intervals=(
                    reserved_edge_intervals
                    if self.reserved_edge_hard_constraints_enabled
                    else []
                ),
            )
            if invalid_reason:
                result.plans = {}
                result.debug.reason = f"cbs_{invalid_reason}"
        return result

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
    ):
        traffic_graph = self._traffic_graph(speed)
        planner = RollingSippPlanner(
            traffic_graph,
            heuristic_fn=self._heuristic_ticks,
            move_cost_fn=lambda src, dst: self._edge_tick_cost(src, dst, speed, acceleration),
            heading_fn=self._edge_heading,
            heading_options_fn=self._edge_heading_options,
            turn_cost_fn=(
                lambda from_yaw, to_yaw: self._rotation_tick_cost(
                    from_yaw,
                    to_yaw,
                    turn_speed,
                )
                if rotate_enabled
                else 0
            ),
            low_level_max_time=low_level_max_time,
            wait_cost=self.wait_cost,
            max_planning_time_sec=self.max_planning_time_sec,
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

    def _payload_low_level_max_time(self, payload: dict[str, Any]) -> int:
        raw = payload.get("lowLevelMaxTime", self.low_level_max_time)
        try:
            value = int(raw)
        except (TypeError, ValueError):
            value = self.low_level_max_time
        return max(1, min(self.low_level_max_time, value))

    def _request_route_nodes(
        self,
        item: dict[str, Any],
        *,
        robot_name: str,
        start_lm: str,
        goal_lm: str,
    ) -> tuple[str, ...]:
        raw = item.get("routeNodes") or item.get("route_nodes")
        if not isinstance(raw, list):
            return ()
        nodes = tuple(
            str(node).strip()
            for node in raw
            if str(node).strip()
        )
        if not nodes:
            return ()
        if nodes[0] != start_lm or nodes[-1] != goal_lm:
            raise ValueError(
                f"routeNodes for {robot_name} must span {start_lm}->{goal_lm}"
            )
        for src, dst in zip(nodes, nodes[1:]):
            if dst not in self.graph.get(src, []):
                raise ValueError(
                    f"routeNodes for {robot_name} contains non-edge {src}->{dst}"
                )
        return nodes

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

    def _planner_backend_for_payload(self, payload: dict[str, Any]) -> str:
        override = payload.get("plannerBackend") or payload.get("planner_backend")
        if override is None:
            return self.planner_backend
        return self._planner_backend({"planner_backend": override})

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
                raw_time = item["time"] if "time" in item else item.get("t")
                time_tick = self._int_value(raw_time)
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
                raw_time = item["time"] if "time" in item else item.get("t")
                time_tick = self._int_value(raw_time)
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
        # Graph landmarks are scheduling vertices, not physical stop points.
        # Applying a complete accelerate/decelerate profile independently to
        # every 0.5 m edge made a robot brake to zero at every map cell and
        # turned a 1.37 m/s route into roughly 0.25 m/s stop-and-go motion.
        # Reserve consecutive MOVE edges at their speed limit; explicit WAIT
        # and ROTATE actions still own separate ticks. Route execution and the
        # renderer interpolate continuously across the resulting timestamps.
        del acceleration
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
        travel_time = length / max(0.02, edge_speed)
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
        self._bounded_cache_store(
            self._heuristic_cache,
            key,
            value,
            self.heuristic_cache_max_entries,
        )
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
        start_yaw: float = 0.0,
        yaws: list[float] | None = None,
        actions: list[str] | None = None,
    ) -> list[dict[str, float | str]]:
        if not nodes:
            return []

        first = self.landmarks[nodes[0]]
        trajectory: list[dict[str, float | str]] = [
            {
                "t": 0.0,
                "x": first.x,
                "y": first.y,
                "yaw": float(start_yaw),
                "edgeId": f"{first.name}->{first.name}",
                "lm": first.name,
                "motionDirection": "not_specified",
            }
        ]
        current_time = 0.0
        last_yaw = float(start_yaw)
        has_kinematic_timing = bool(
            actions
            and len(actions) == len(nodes)
            and yaws
            and len(yaws) == len(nodes)
        )

        for index in range(1, len(nodes)):
            from_lm = nodes[index - 1]
            to_lm = nodes[index]
            planned_duration = self._planned_segment_duration(times, index)
            planned_action = (
                str(actions[index]).strip().lower()
                if actions is not None and index < len(actions)
                else ""
            )
            planned_yaw = (
                float(yaws[index])
                if yaws is not None and index < len(yaws)
                else last_yaw
            )
            if from_lm == to_lm:
                current_time += max(0.05, planned_duration or self.wait_time_sec)
                landmark = self.landmarks[from_lm]
                is_rotation = planned_action == "rotate"
                trajectory.append(
                    {
                        "t": current_time,
                        "x": landmark.x,
                        "y": landmark.y,
                        "yaw": planned_yaw if is_rotation else last_yaw,
                        "edgeId": (
                            f"WAIT@ROTATE:{from_lm}"
                            if is_rotation
                            else f"{from_lm}->{from_lm}"
                        ),
                        "lm": from_lm,
                        "motionDirection": "rotate" if is_rotation else "not_specified",
                    }
                )
                if is_rotation:
                    last_yaw = planned_yaw
                continue

            route = self._direct_route(from_lm, to_lm)
            samples = self.route_planner.sample_route(route)
            segment = self._annotate_sample_distances(samples)
            segment_length = max(segment[-1]["s"] if segment else 0.0, 1e-6)
            segment_yaw = float(
                segment[1]["yaw"] if len(segment) > 1 else segment[0]["yaw"]
            )
            edge = self.edge_by_key.get((from_lm, to_lm))
            reverse_unspecified = bool(
                has_kinematic_timing
                and edge is not None
                and edge.motion_direction_code() == -1
                and abs(
                    abs(self._normalize_angle(planned_yaw - segment_yaw))
                    - math.pi
                )
                <= 0.000001
            )
            continuous_duration = (
                segment_length / max(0.02, speed)
                if has_kinematic_timing
                else self._travel_time(segment_length, speed, acceleration)
            )
            stretch_motion = self.stretch_motion_to_reservation_ticks if stretch_motion_to_reservation_ticks is None else stretch_motion_to_reservation_ticks
            rotate_duration = 0.0
            if rotate_enabled and not has_kinematic_timing:
                rotate_duration = self._rotation_duration(last_yaw, segment_yaw, turn_speed)
                if stretch_motion and planned_duration is not None:
                    reservation_slack = max(0.0, planned_duration - continuous_duration)
                    # Compatibility for old plans without explicit ROTATE
                    # actions. New SIPP/CBS plans reserve the turn separately.
                    if rotate_duration > reservation_slack + 0.000001:
                        rotate_duration = 0.0
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
            if has_kinematic_timing:
                # The reservation is authoritative. ROTATE was emitted as a
                # separate same-node action, so this interval is MOVE only.
                duration = max(continuous_duration, planned_duration or 0.0, 0.05)
            elif stretch_motion:
                duration = max(
                    continuous_duration,
                    (planned_duration or 0.0) - rotate_duration,
                    0.05,
                )
            else:
                duration = max(continuous_duration, 0.05)

            for sample in segment[1:]:
                t = current_time + (float(sample["s"]) / segment_length) * duration
                last_yaw = self._normalize_angle(
                    float(sample["yaw"])
                    + (math.pi if reverse_unspecified else 0.0)
                )
                trajectory.append(
                    {
                        "t": t,
                        "x": float(sample["x"]),
                        "y": float(sample["y"]),
                        "yaw": last_yaw,
                        "edgeId": str(sample["edgeId"]),
                        "motionDirection": (
                            "backward"
                            if reverse_unspecified
                            else str(
                                sample.get(
                                    "motionDirection",
                                    "not_specified",
                                )
                            )
                        ),
                    }
                )
            current_time += duration
            if trajectory:
                trajectory[-1]["lm"] = to_lm

        if trajectory:
            trajectory[-1]["lm"] = nodes[-1]
        return trajectory

    def _planned_segment_duration(self, times: list[int] | None, index: int) -> float | None:
        if not times or index <= 0 or index >= len(times):
            return None
        return max(0.0, (int(times[index]) - int(times[index - 1])) * self.time_step_sec)

    def _validate_start_pose_at_lm(
        self,
        robot_name: str,
        start_lm: str,
        start_pose: dict[str, float],
    ) -> None:
        landmark = self.landmarks.get(start_lm)
        if landmark is None:
            raise ValueError(f"{robot_name}: unknown start LM: {start_lm}")
        distance = math.hypot(
            landmark.x - float(start_pose.get("x", 0.0) or 0.0),
            landmark.y - float(start_pose.get("y", 0.0) or 0.0),
        )
        if distance <= self.start_pose_lm_tolerance_m:
            return
        raise ValueError(
            f"{robot_name}: start pose is {distance:.3f} m from {start_lm}; "
            "off-graph approach is forbidden, replan at a landmark"
        )

    def _rotation_duration(self, from_yaw: float, to_yaw: float, turn_speed: float) -> float:
        delta = abs(self._normalize_angle(float(to_yaw or 0.0) - float(from_yaw or 0.0)))
        if delta < math.radians(2.0):
            return 0.0
        return delta / max(0.05, float(turn_speed or 0.05))

    def _rotation_tick_cost(self, from_yaw: float, to_yaw: float, turn_speed: float) -> int:
        duration = self._rotation_duration(from_yaw, to_yaw, turn_speed)
        if duration <= 0.0:
            return 0
        return max(1, int(math.ceil(duration / max(self.time_step_sec, 1e-6))))

    def _edge_heading(self, from_lm: str, to_lm: str) -> float:
        start = self.landmarks.get(from_lm)
        goal = self.landmarks.get(to_lm)
        if start is None or goal is None:
            return 0.0
        dx = goal.x - start.x
        dy = goal.y - start.y
        edge = self.edge_by_key.get((from_lm, to_lm))
        if (
            edge is not None
            and edge.geometry is not None
            and str(edge.geometry.geometry).lower() == "bezier"
            and len(edge.geometry.control_points) >= 2
        ):
            first = edge.geometry.control_points[0]
            second = edge.geometry.control_points[1]
            tangent_x = second.x - first.x
            tangent_y = second.y - first.y
            if math.hypot(tangent_x, tangent_y) > 1e-9:
                dx = tangent_x
                dy = tangent_y
        heading = math.atan2(dy, dx)
        if edge is not None and edge.motion_direction_code() == 1:
            heading += math.pi
        return self._normalize_angle(heading)

    def _edge_heading_options(
        self,
        from_lm: str,
        to_lm: str,
    ) -> tuple[float, ...]:
        """Return every body orientation allowed by an authored edge.

        ``forward`` and ``backward`` are strict motion rules.  An unspecified
        edge deliberately permits either orientation, so SIPP/CBS may keep the
        current yaw and translate backwards instead of reserving an unsafe or
        unnecessary in-place turn.
        """
        heading = self._edge_heading(from_lm, to_lm)
        edge = self.edge_by_key.get((from_lm, to_lm))
        if edge is None or edge.motion_direction_code() != -1:
            return (heading,)
        return (
            heading,
            self._normalize_angle(heading + math.pi),
        )

    def _normalize_angle(self, value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def _direct_route(self, from_lm: str, to_lm: str) -> PlannedRoute:
        edge = self.edge_by_key.get((from_lm, to_lm))
        if edge is None:
            raise ValueError(
                f"MAPF returned non-adjacent landmarks: {from_lm}->{to_lm}"
            )
        return PlannedRoute(nodes=[from_lm, to_lm], edges=[edge], length=edge.length)

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
