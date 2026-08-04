"""Validation and normalization of Fleet MAPF request payloads."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from ..cbs.lm_cbs import LmRobotRequest


@dataclass(slots=True)
class PreparedFleetRequest:
    payload: dict[str, Any]
    requests: list[LmRobotRequest]
    start_yaws: dict[str, float]
    speed: float
    acceleration: float
    rotate_enabled: bool
    turn_speed: float
    stretch_motion: bool
    low_level_max_time: int
    allow_cbs_fallback: bool
    reserved_detour_enabled: bool
    blocked_lms: set[str]
    blocked_edges: set[tuple[str, str]]
    detour_blocked_edges: set[tuple[str, str]]
    reserved_vertex_constraints: list[tuple[int, str]]
    reserved_edge_constraints: list[tuple[int, str, str]]
    reserved_vertex_intervals: list[tuple[int, int, str, str]]
    reserved_edge_intervals: list[tuple[int, int, str, str, str]]
    reserved_interval_edges: set[tuple[str, str]]
    selected_backend: str


class PlanningRequestPreparer:
    """Turn the permissive web payload into one typed planning request."""

    def __init__(self, planner: Any) -> None:
        self.planner = planner

    def prepare(self, payload: dict[str, Any]) -> PreparedFleetRequest:
        robots = payload.get("robots", [])
        if not isinstance(robots, list):
            raise ValueError("robots must be a list")

        speed = self.route_speed(payload)
        acceleration = self.route_acceleration(payload)
        rotate_enabled = self.rotate_enabled(payload)
        turn_speed = self.turn_speed(payload)
        requests, start_yaws = self._robot_requests(
            robots,
            speed=speed,
        )
        blocked_edges = self.blocked_edges(payload)
        reserved_vertex_constraints = self.reserved_vertex_constraints(
            payload
        )
        reserved_edge_constraints = self.reserved_edge_constraints(payload)
        reserved_vertex_intervals = (
            self.reserved_vertex_interval_constraints(payload)
        )
        reserved_edge_intervals = (
            self.reserved_edge_interval_constraints(payload)
        )
        reserved_interval_edges = self.reserved_interval_blocked_edges(
            payload
        )
        reserved_detour_enabled = bool(
            payload.get(
                "reservedEdgeDetourEnabled",
                self.planner.reserved_edge_detour_enabled,
            )
        )
        detour_blocked_edges = (
            blocked_edges | reserved_interval_edges
            if reserved_detour_enabled
            else blocked_edges
        )
        allow_cbs_fallback = bool(
            payload.get("allowCbsFallback", True)
        )
        if any(
            request.start_not_before_tick > 0
            or bool(request.node_departure_not_before)
            for request in requests
        ):
            allow_cbs_fallback = False

        return PreparedFleetRequest(
            payload=payload,
            requests=requests,
            start_yaws=start_yaws,
            speed=speed,
            acceleration=acceleration,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            stretch_motion=self.stretch_motion(payload),
            low_level_max_time=self.payload_low_level_max_time(payload),
            allow_cbs_fallback=allow_cbs_fallback,
            reserved_detour_enabled=reserved_detour_enabled,
            blocked_lms={
                str(name)
                for name in payload.get("blocked_lms", [])
                if isinstance(name, str)
            },
            blocked_edges=blocked_edges,
            detour_blocked_edges=detour_blocked_edges,
            reserved_vertex_constraints=reserved_vertex_constraints,
            reserved_edge_constraints=reserved_edge_constraints,
            reserved_vertex_intervals=reserved_vertex_intervals,
            reserved_edge_intervals=reserved_edge_intervals,
            reserved_interval_edges=reserved_interval_edges,
            selected_backend=self.planner._planner_backend_for_payload(
                payload
            ),
        )

    def _robot_requests(
        self,
        robots: list[object],
        *,
        speed: float,
    ) -> tuple[list[LmRobotRequest], dict[str, float]]:
        requests: list[LmRobotRequest] = []
        start_yaws: dict[str, float] = {}
        seen_names: set[str] = set()
        for index, item in enumerate(robots):
            if not isinstance(item, dict):
                raise ValueError(f"robots[{index}] must be an object")
            request, start_yaw = self._robot_request(
                item,
                index=index,
                speed=speed,
                seen_names=seen_names,
            )
            requests.append(request)
            start_yaws[request.robot_name] = start_yaw
        return requests, start_yaws

    def _robot_request(
        self,
        item: dict[str, Any],
        *,
        index: int,
        speed: float,
        seen_names: set[str],
    ) -> tuple[LmRobotRequest, float]:
        name = str(item.get("name", "")).strip()
        start_lm = str(
            item.get("startLm") or item.get("currentLm") or ""
        ).strip()
        goal_lm = str(
            item.get("goalLm") or item.get("targetLm") or ""
        ).strip()
        if not name or not start_lm or not goal_lm:
            raise ValueError(
                f"robots[{index}] requires non-empty name, "
                "startLm/currentLm, and goalLm/targetLm"
            )
        if name in seen_names:
            raise ValueError(f"duplicate robot name: {name}")
        seen_names.add(name)

        start_yaw = self._start_yaw(
            item,
            robot_name=name,
            start_lm=start_lm,
        )
        route_nodes = self.planner._request_route_nodes(
            item,
            robot_name=name,
            start_lm=start_lm,
            goal_lm=goal_lm,
        )
        departure_gates = self._departure_gates(
            item,
            index=index,
            route_nodes=route_nodes,
        )
        authorized_regions = self._string_tuple(
            item.get(
                "authorizedControlledRegions",
                item.get("authorized_controlled_regions", ()),
            ),
            error=(
                f"robots[{index}] "
                "authorizedControlledRegions must be a list"
            ),
        )
        no_wait_nodes = self._string_tuple(
            item.get("noWaitNodes", item.get("no_wait_nodes", ())),
            error=f"robots[{index}] noWaitNodes must be a list",
        )
        if route_nodes:
            outside_route = set(no_wait_nodes) - set(route_nodes)
            if outside_route:
                raise ValueError(
                    f"robots[{index}] noWaitNodes contains node(s) "
                    "outside routeNodes: "
                    + ", ".join(sorted(outside_route))
                )
        self._validate_corridor_authority(
            index=index,
            route_nodes=route_nodes,
            departure_gates=departure_gates,
            authorized_regions=authorized_regions,
            speed=speed,
        )
        return (
            LmRobotRequest(
                name,
                start_lm,
                goal_lm,
                start_yaw,
                route_nodes,
                self._start_not_before_tick(item),
                departure_gates,
                authorized_regions,
                no_wait_nodes,
            ),
            start_yaw,
        )

    def _start_yaw(
        self,
        item: dict[str, Any],
        *,
        robot_name: str,
        start_lm: str,
    ) -> float:
        pose = item.get("startPose")
        if not isinstance(pose, dict):
            return 0.0
        clean_pose = {
            "x": float(pose.get("x", 0.0) or 0.0),
            "y": float(pose.get("y", 0.0) or 0.0),
            "yaw": float(pose.get("yaw", 0.0) or 0.0),
        }
        self.planner._validate_start_pose_at_lm(
            robot_name,
            start_lm,
            clean_pose,
        )
        return clean_pose["yaw"]

    def _start_not_before_tick(self, item: dict[str, Any]) -> int:
        try:
            start_not_before_sec = max(
                0.0,
                float(item.get("startNotBeforeSec", 0.0) or 0.0),
            )
        except (TypeError, ValueError):
            start_not_before_sec = 0.0
        return self._not_before_tick(start_not_before_sec)

    def _departure_gates(
        self,
        item: dict[str, Any],
        *,
        index: int,
        route_nodes: tuple[str, ...],
    ) -> tuple[tuple[str, int], ...]:
        raw_gates = item.get("departureNotBefore", ())
        if isinstance(raw_gates, dict):
            raw_gates = [
                {"node": node, "timeSec": value}
                for node, value in raw_gates.items()
            ]
        gates: list[tuple[str, int]] = []
        if not isinstance(raw_gates, (list, tuple)):
            return ()
        for raw_gate in raw_gates:
            if not isinstance(raw_gate, dict):
                continue
            node = str(
                raw_gate.get("node") or raw_gate.get("lm") or ""
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
            gates.append((node, self._not_before_tick(time_sec)))
        return tuple(gates)

    def _not_before_tick(self, seconds: float) -> int:
        return int(
            math.ceil(
                seconds
                / max(0.001, self.planner.time_step_sec)
                - 1e-9
            )
        )

    @staticmethod
    def _string_tuple(value: object, *, error: str) -> tuple[str, ...]:
        if value is None:
            value = ()
        if not isinstance(value, (list, tuple, set)):
            raise ValueError(error)
        return tuple(
            dict.fromkeys(
                str(item).strip()
                for item in value
                if str(item).strip()
            )
        )

    def _validate_corridor_authority(
        self,
        *,
        index: int,
        route_nodes: tuple[str, ...],
        departure_gates: tuple[tuple[str, int], ...],
        authorized_regions: tuple[str, ...],
        speed: float,
    ) -> None:
        if not authorized_regions:
            return
        if not departure_gates:
            raise ValueError(
                f"robots[{index}] corridor authority requires "
                "departureNotBefore"
            )
        if not route_nodes:
            raise ValueError(
                f"robots[{index}] corridor authority requires routeNodes"
            )
        traffic_graph = self.planner._traffic_graph(speed)
        route_regions = {
            region_id
            for src, dst in zip(route_nodes, route_nodes[1:])
            for lane in (traffic_graph.lane_for(src, dst),)
            if lane is not None
            for region_id in lane.controlled_region_ids
        }
        unauthorized = set(authorized_regions) - route_regions
        if unauthorized:
            raise ValueError(
                f"robots[{index}] corridor authority contains region(s) "
                "outside routeNodes: "
                + ", ".join(sorted(unauthorized))
            )

    def payload_low_level_max_time(
        self,
        payload: dict[str, Any],
    ) -> int:
        raw = payload.get(
            "lowLevelMaxTime",
            self.planner.low_level_max_time,
        )
        try:
            value = int(raw)
        except (TypeError, ValueError):
            value = self.planner.low_level_max_time
        return max(1, min(self.planner.low_level_max_time, value))

    def request_route_nodes(
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
                f"routeNodes for {robot_name} must span "
                f"{start_lm}->{goal_lm}"
            )
        for src, dst in zip(nodes, nodes[1:]):
            if dst not in self.planner.graph.get(src, []):
                raise ValueError(
                    f"routeNodes for {robot_name} contains "
                    f"non-edge {src}->{dst}"
                )
        return nodes

    def blocked_edges(
        self,
        payload: dict[str, Any],
    ) -> set[tuple[str, str]]:
        raw_edges = (
            payload.get("blocked_edges")
            or payload.get("blockedEdges")
            or []
        )
        if not isinstance(raw_edges, list):
            return set()
        blocked: set[tuple[str, str]] = set()
        for item in raw_edges:
            if isinstance(item, str) and "->" in item:
                src, dst = item.split("->", 1)
                blocked.add((src.strip(), dst.strip()))
            elif isinstance(item, dict):
                src = str(
                    item.get("from") or item.get("fromLm") or ""
                ).strip()
                dst = str(
                    item.get("to") or item.get("toLm") or ""
                ).strip()
                if src and dst:
                    blocked.add((src, dst))
        return blocked

    def reserved_vertex_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, str]]:
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
                raw_time = (
                    item["time"] if "time" in item else item.get("t")
                )
                time_tick = self.int_value(raw_time)
                node = str(
                    item.get("node") or item.get("lm") or ""
                ).strip()
                if time_tick is not None and node:
                    constraints.append((time_tick, node))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                time_tick = self.int_value(item[0])
                node = str(item[1]).strip()
                if time_tick is not None and node:
                    constraints.append((time_tick, node))
        return constraints

    def reserved_edge_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, str, str]]:
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
                raw_time = (
                    item["time"] if "time" in item else item.get("t")
                )
                time_tick = self.int_value(raw_time)
                src = str(
                    item.get("from") or item.get("src") or ""
                ).strip()
                dst = str(
                    item.get("to") or item.get("dst") or ""
                ).strip()
                if time_tick is not None and src and dst:
                    constraints.append((time_tick, src, dst))
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                time_tick = self.int_value(item[0])
                src = str(item[1]).strip()
                dst = str(item[2]).strip()
                if time_tick is not None and src and dst:
                    constraints.append((time_tick, src, dst))
        return constraints

    @staticmethod
    def reserved_edge_intervals(
        payload: dict[str, Any],
    ) -> list[dict[str, Any]]:
        raw_intervals = (
            payload.get("reserved_edge_intervals")
            or payload.get("reservedEdgeIntervals")
            or []
        )
        if not isinstance(raw_intervals, list):
            return []
        return [
            item
            for item in raw_intervals
            if isinstance(item, dict)
        ]

    @staticmethod
    def reserved_vertex_intervals(
        payload: dict[str, Any],
    ) -> list[dict[str, Any]]:
        raw_intervals = (
            payload.get("reserved_vertex_intervals")
            or payload.get("reservedVertexIntervals")
            or []
        )
        if not isinstance(raw_intervals, list):
            return []
        return [
            item
            for item in raw_intervals
            if isinstance(item, dict)
        ]

    def reserved_edge_interval_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, int, str, str, str]]:
        constraints: list[tuple[int, int, str, str, str]] = []
        for item in self.reserved_edge_intervals(payload):
            src = str(
                item.get("from") or item.get("src") or ""
            ).strip()
            dst = str(
                item.get("to") or item.get("dst") or ""
            ).strip()
            start = self.float_value(
                item.get("start") or item.get("startTime") or 0.0
            )
            end = self.float_value(
                item.get("end") or item.get("endTime") or 0.0
            )
            owner = str(
                item.get("robot") or item.get("owner") or ""
            ).strip()
            if not src or not dst or start is None or end is None:
                continue
            start_tick, end_tick = self.interval_seconds_to_ticks(
                start,
                end,
            )
            constraints.append(
                (start_tick, end_tick, src, dst, owner)
            )
        return constraints

    def reserved_vertex_interval_constraints(
        self,
        payload: dict[str, Any],
    ) -> list[tuple[int, int, str, str]]:
        constraints: list[tuple[int, int, str, str]] = []
        for item in self.reserved_vertex_intervals(payload):
            node = str(
                item.get("node") or item.get("lm") or ""
            ).strip()
            start = self.float_value(
                item.get("start") or item.get("startTime") or 0.0
            )
            end = self.float_value(
                item.get("end") or item.get("endTime") or 0.0
            )
            owner = str(
                item.get("robot") or item.get("owner") or ""
            ).strip()
            if not node or start is None or end is None:
                continue
            start_tick, end_tick = self.interval_seconds_to_ticks(
                start,
                end,
            )
            constraints.append((start_tick, end_tick, node, owner))
        return constraints

    def reserved_interval_blocked_edges(
        self,
        payload: dict[str, Any],
    ) -> set[tuple[str, str]]:
        blocked: set[tuple[str, str]] = set()
        if not self.planner.reserved_edge_detour_enabled:
            return blocked
        horizon = max(
            0.0,
            self.planner.reservation_detour_horizon_sec,
        )
        for item in self.reserved_edge_intervals(payload):
            src = str(
                item.get("from") or item.get("src") or ""
            ).strip()
            dst = str(
                item.get("to") or item.get("dst") or ""
            ).strip()
            start = self.float_value(
                item.get("start") or item.get("startTime") or 0.0
            )
            end = self.float_value(
                item.get("end") or item.get("endTime") or 0.0
            )
            if not src or not dst or start is None or end is None:
                continue
            start_s = min(start, end)
            end_s = max(start, end)
            if end_s < 0.0 or start_s > horizon:
                continue
            blocked.add((src, dst))
            blocked.add((dst, src))
        return blocked

    def interval_seconds_to_ticks(
        self,
        start: float,
        end: float,
    ) -> tuple[int, int]:
        start_s = max(0.0, min(float(start), float(end)))
        end_s = max(0.0, max(float(start), float(end)))
        step = max(0.001, self.planner.time_step_sec)
        return math.floor(start_s / step), math.ceil(end_s / step)

    @staticmethod
    def int_value(value: Any) -> int | None:
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def float_value(value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def route_speed(self, payload: dict[str, Any]) -> float:
        raw_speed = payload.get("speed")
        if raw_speed is None:
            navigation = self.planner.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_speed = navigation.get("route_speed")
        try:
            return max(0.02, float(raw_speed))
        except (TypeError, ValueError):
            return 0.35

    def route_acceleration(self, payload: dict[str, Any]) -> float:
        raw_value = (
            payload.get("acceleration")
            or payload.get("routeAcceleration")
            or payload.get("route_acceleration")
        )
        if raw_value is None:
            navigation = self.planner.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_value = navigation.get("route_acceleration")
        try:
            return max(0.0, float(raw_value))
        except (TypeError, ValueError):
            return 0.0

    def rotate_enabled(self, payload: dict[str, Any]) -> bool:
        raw_value = (
            payload.get("rotate")
            if "rotate" in payload
            else payload.get("simulateRotation")
            if "simulateRotation" in payload
            else payload.get("simulate_rotation")
        )
        if raw_value is None:
            navigation = self.planner.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_value = navigation.get("simulate_rotation", False)
        if isinstance(raw_value, str):
            return raw_value.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        return bool(raw_value)

    def turn_speed(self, payload: dict[str, Any]) -> float:
        raw_value = (
            payload.get("turnSpeed")
            or payload.get("turn_speed")
            or payload.get("rotationSpeed")
            or payload.get("rotation_speed")
        )
        if raw_value is None:
            navigation = self.planner.params.get("navigation", {})
            if isinstance(navigation, dict):
                raw_value = (
                    navigation.get("turn_speed")
                    or navigation.get("max_angular_speed")
                )
        try:
            return max(0.05, float(raw_value))
        except (TypeError, ValueError):
            return 0.9

    def stretch_motion(self, payload: dict[str, Any]) -> bool:
        raw_value = (
            payload.get("stretchMotionToReservationTicks")
            if "stretchMotionToReservationTicks" in payload
            else payload.get("stretch_motion_to_reservation_ticks")
        )
        if raw_value is None:
            return bool(
                self.planner.stretch_motion_to_reservation_ticks
            )
        if isinstance(raw_value, str):
            return raw_value.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        return bool(raw_value)

    def validate_start_pose_at_lm(
        self,
        robot_name: str,
        start_lm: str,
        start_pose: dict[str, float],
    ) -> None:
        landmark = self.planner.landmarks.get(start_lm)
        if landmark is None:
            raise ValueError(f"{robot_name}: unknown start LM: {start_lm}")
        distance = math.hypot(
            landmark.x - float(start_pose.get("x", 0.0) or 0.0),
            landmark.y - float(start_pose.get("y", 0.0) or 0.0),
        )
        if distance <= self.planner.start_pose_lm_tolerance_m:
            return
        raise ValueError(
            f"{robot_name}: start pose is {distance:.3f} m "
            f"from {start_lm}; off-graph approach is forbidden, "
            "replan at a landmark"
        )
