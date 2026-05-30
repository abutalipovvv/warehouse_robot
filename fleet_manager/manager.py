from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from time import time
from typing import Any

from fleet_manager.mapf import FleetMapfPlanner
from route_core import GraphEdge, Landmark, MapMetadata, PlannedRoute, WarehouseMapLoader


@dataclass
class FleetRobot:
    name: str
    current_lm: str
    target_lm: str = ""
    status: str = "IDLE"
    updated_at: float = field(default_factory=time)
    pose: dict[str, float] | None = None
    trajectory: list[dict[str, Any]] = field(default_factory=list)
    plan_nodes: list[str] = field(default_factory=list)
    route_started_at: float | None = None
    route_clock: float = 0.0
    last_tick_at: float | None = None
    last_reason: str = ""
    route_note: str = ""
    blocked_since: float | None = None
    last_replan_at: float | None = None
    trajectory_dirty: bool = False

    def to_dict(self, include_trajectory: bool = True) -> dict[str, Any]:
        return {
            "name": self.name,
            "currentLm": self.current_lm,
            "targetName": self.target_lm,
            "targetLm": self.target_lm,
            "status": self.status,
            "updatedAt": self.updated_at,
            "pose": self.pose,
            "trajectory": self.trajectory if include_trajectory or self.trajectory_dirty else [],
            "planNodes": self.plan_nodes,
            "routeClock": self.route_clock,
            "reason": self.last_reason,
            "routeNote": self.route_note,
            "blockedSince": self.blocked_since,
            "lastReplanAt": self.last_replan_at,
        }


@dataclass
class FleetEvent:
    stamp: float
    level: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp": self.stamp,
            "level": self.level,
            "message": self.message,
        }


class WebFleetManager:
    """No-ROS fleet manager used by the web operator panel."""

    def __init__(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        params: dict[str, Any] | None = None,
        map_dir: Path | None = None,
        map_metadata: MapMetadata | None = None,
    ) -> None:
        self.landmarks = landmarks
        self.edges = edges
        self.params = params or {}
        self.planner = FleetMapfPlanner(landmarks, edges, params=params)
        self.robots: dict[str, FleetRobot] = {}
        self.events: list[FleetEvent] = []
        self.obstacles: list[dict[str, float]] = []
        self.obstacle_areas: list[dict[str, float]] = []
        self.collision = FleetCollisionChecker(
            params=self.params,
            map_dir=map_dir,
            map_metadata=map_metadata,
        )

    def state(self, include_trajectories: bool = True) -> dict[str, Any]:
        self._advance_runtime()
        return {
            "ok": True,
            "robots": [
                robot.to_dict(include_trajectory=include_trajectories)
                for robot in self.robots.values()
            ],
            "events": [event.to_dict() for event in self.events[-80:]],
            "obstacles": self.obstacles,
            "obstacleAreas": self.obstacle_areas,
        }

    def tick(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._advance_runtime()
        state = {
            "ok": True,
            "robots": [
                robot.to_dict(include_trajectory=False)
                for robot in self.robots.values()
            ],
            "events": [event.to_dict() for event in self.events[-80:]],
            "obstacles": self.obstacles,
            "obstacleAreas": self.obstacle_areas,
        }
        for robot in self.robots.values():
            robot.trajectory_dirty = False
        return state

    def update_world(self, payload: dict[str, Any]) -> dict[str, Any]:
        obstacles = payload.get("obstacles", [])
        areas = payload.get("obstacleAreas", [])
        previous_counts = (len(self.obstacles), len(self.obstacle_areas))
        if isinstance(obstacles, list):
            self.obstacles = [
                self._clean_obstacle(item)
                for item in obstacles
                if isinstance(item, dict)
            ]
        if isinstance(areas, list):
            self.obstacle_areas = [
                self._clean_area(item)
                for item in areas
                if isinstance(item, dict)
            ]
        params = payload.get("params")
        if isinstance(params, dict):
            self.params = params
            self.collision.set_params(params)
        counts = (len(self.obstacles), len(self.obstacle_areas))
        if counts != previous_counts:
            self._event(
                "info",
                f"world synced: obstacles={counts[0]}, areas={counts[1]}",
            )
        return {"ok": True, "state": self.state()}

    def add_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        current_lm = str(payload.get("currentLm") or payload.get("spawnLm") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if not current_lm:
            raise ValueError("currentLm/spawnLm is required")
        if current_lm not in self.landmarks:
            raise ValueError(f"unknown LM: {current_lm}")

        robot = self.robots.get(name)
        if robot is None:
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                pose=self._pose_at_landmark(current_lm),
            )
            self.robots[name] = robot
            self._event("info", f"robot added: {name}@{current_lm}")
        else:
            robot.current_lm = current_lm
            robot.pose = self._pose_at_landmark(current_lm)
            robot.target_lm = ""
            robot.trajectory = []
            robot.plan_nodes = []
            robot.route_clock = 0.0
            robot.route_note = ""
            robot.trajectory_dirty = True
            robot.blocked_since = None
            robot.last_replan_at = None
            robot.updated_at = time()
            self._event("info", f"robot updated: {name}@{current_lm}")
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def remove_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        removed = self.robots.pop(name, None)
        if removed is not None:
            self._event("warn", f"robot removed: {name}")
        return {"ok": True, "removed": removed is not None, "state": self.state()}

    def stop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if name:
            robot = self.robots.get(name)
            if robot is None:
                raise ValueError(f"unknown robot: {name}")
            self._stop_robot(robot)
            self._event("warn", f"robot stopped: {name}")
        else:
            for robot in self.robots.values():
                self._stop_robot(robot)
            self._event("warn", "fleet stopped")
        return {"ok": True, "state": self.state()}

    def reset_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        target_names = [name] if name else list(self.robots)
        for robot_name in target_names:
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            spawn_lm = str(payload.get("spawnLm") or robot.current_lm or "").strip()
            if spawn_lm in self.landmarks:
                robot.current_lm = spawn_lm
                robot.pose = self._pose_at_landmark(spawn_lm)
            robot.target_lm = ""
            robot.status = "IDLE"
            robot.trajectory = []
            robot.plan_nodes = []
            robot.trajectory_dirty = True
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = None
            robot.blocked_since = None
            robot.last_replan_at = None
            robot.last_reason = "reset"
            robot.route_note = ""
            robot.updated_at = time()
            self._event("warn", f"robot reset: {robot.name}@{robot.current_lm}")
        return {"ok": True, "state": self.state()}

    def update_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            current_lm = str(payload.get("currentLm") or "").strip()
            if not current_lm:
                raise ValueError("unknown robot and currentLm is missing")
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                pose=self._pose_at_landmark(current_lm),
            )
            self.robots[name] = robot

        if "currentLm" in payload and payload["currentLm"]:
            robot.current_lm = str(payload["currentLm"] or "")
            if robot.current_lm and not robot.trajectory:
                robot.pose = self._pose_at_landmark(robot.current_lm)
        if "targetLm" in payload:
            robot.target_lm = str(payload["targetLm"] or "")
        if "status" in payload and payload["status"]:
            robot.status = str(payload["status"])
        if "pose" in payload and isinstance(payload["pose"], dict):
            pose = payload["pose"]
            robot.pose = {
                "x": float(pose.get("x", 0.0) or 0.0),
                "y": float(pose.get("y", 0.0) or 0.0),
                "yaw": float(pose.get("yaw", 0.0) or 0.0),
            }
        if robot.status in {"IDLE", "ARRIVED", "BLOCKED", "STOPPED"} and not robot.target_lm:
            robot.trajectory = []
            robot.plan_nodes = []
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = None
            robot.trajectory_dirty = True
        robot.updated_at = time()
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def plan(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Plan one or more orders.

        Compatible with the old /api/fleet/plan payload:
        {"robots": [{"name": "...", "startLm": "...", "goalLm": "..."}]}.
        """
        requests = payload.get("robots", [])
        if not isinstance(requests, list):
            raise ValueError("robots must be a list")

        valid_requests: list[dict[str, Any]] = []
        for request in requests:
            if not isinstance(request, dict):
                continue
            name = str(request.get("name", "")).strip()
            start_lm = str(request.get("startLm") or request.get("currentLm") or "").strip()
            goal_lm = str(request.get("goalLm") or request.get("targetLm") or "").strip()
            start_pose = request.get("startPose")
            if not name or not start_lm or not goal_lm:
                self._event("error", f"bad order ignored: name/start/goal is missing")
                continue
            if start_lm not in self.landmarks:
                self._block_order(name, start_lm, goal_lm, f"unknown start LM: {start_lm}")
                continue
            if goal_lm not in self.landmarks:
                self._block_order(name, start_lm, goal_lm, f"unknown goal LM: {goal_lm}")
                continue
            robot = self.robots.get(name)
            if robot is None:
                robot = FleetRobot(
                    name=name,
                    current_lm=start_lm,
                    pose=self._pose_at_landmark(start_lm),
                )
                self.robots[name] = robot
            robot.current_lm = start_lm
            robot.target_lm = goal_lm
            robot.status = "PLANNING"
            robot.last_reason = "order accepted"
            robot.blocked_since = None
            robot.updated_at = time()
            self._event("info", f"order accepted: {name} {start_lm}->{goal_lm}")
            clean_request: dict[str, Any] = {
                "name": name,
                "startLm": start_lm,
                "goalLm": goal_lm,
            }
            if isinstance(start_pose, dict):
                clean_request["startPose"] = {
                    "x": float(start_pose.get("x", 0.0) or 0.0),
                    "y": float(start_pose.get("y", 0.0) or 0.0),
                    "yaw": float(start_pose.get("yaw", 0.0) or 0.0),
                }
            elif robot.pose is not None:
                clean_request["startPose"] = dict(robot.pose)
            valid_requests.append(clean_request)

        if not valid_requests:
            self._event("error", "planner skipped: no valid orders")
            return {
                "ok": False,
                "debug": {
                    "reason": "no valid orders",
                    "conflictsResolved": 0,
                    "highLevelNodes": 0,
                    "expandedNodes": 0,
                },
                "timeStepSec": 0.0,
                "plans": [],
                "fleetState": self.state(),
            }

        result = self._plan_valid_requests(valid_requests, payload)
        planned_names = {str(plan.get("robot")) for plan in result.get("plans", []) if isinstance(plan, dict)}
        if result.get("ok"):
            now = time()
            self._apply_planner_result(result, now)
            self._event("info", f"planner accepted {len(planned_names)} order(s)")
        else:
            for request in valid_requests:
                if not isinstance(request, dict):
                    continue
                robot = self.robots.get(str(request.get("name", "")).strip())
                if robot is not None:
                    robot.status = "BLOCKED"
                    robot.last_reason = result.get("debug", {}).get("reason", "unknown")
                    robot.updated_at = time()
            reason = result.get("debug", {}).get("reason", "unknown")
            self._event("error", f"planner rejected: {reason}")

        return {
            **result,
            "fleetState": self.state(),
        }

    def _event(self, level: str, message: str) -> None:
        self.events.append(FleetEvent(stamp=time(), level=level, message=message))
        self.events = self.events[-200:]

    def _stop_robot(self, robot: FleetRobot) -> None:
        robot.status = "IDLE"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.last_tick_at = None
        robot.blocked_since = None
        robot.last_replan_at = None
        robot.last_reason = "stopped"
        robot.route_note = ""
        robot.updated_at = time()

    def _block_order(self, name: str, start_lm: str, goal_lm: str, reason: str) -> None:
        robot = self.robots.get(name)
        if robot is None:
            robot = FleetRobot(
                name=name,
                current_lm=start_lm,
                target_lm=goal_lm,
                status="BLOCKED",
                pose=self._pose_at_landmark(start_lm),
                last_reason=reason,
            )
            self.robots[name] = robot
        else:
            robot.current_lm = start_lm
            robot.target_lm = goal_lm
            robot.status = "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = time()
        self._event("error", f"{name} blocked: {reason}")

    def _plan_valid_requests(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        payload = payload or {}
        hard_blocked_lms = self._hard_blocked_lms(payload)
        blocked_edges = self._hard_blocked_edges(payload) | self._dynamic_blocked_edges()
        reserved_vertices, reserved_edges = self._reserved_constraints(valid_requests)
        soft_blocked_lms = self._soft_blocked_lms(valid_requests, hard_blocked_lms)
        planner_payload = {
            **payload,
            "robots": valid_requests,
            "blocked_edges": [
                {"from": src, "to": dst}
                for src, dst in sorted(blocked_edges)
            ],
            "reserved_vertex_constraints": [
                {"time": tick, "node": node}
                for tick, node in reserved_vertices
            ],
            "reserved_edge_constraints": [
                {"time": tick, "from": src, "to": dst}
                for tick, src, dst in reserved_edges
            ],
        }

        if soft_blocked_lms:
            result = self.planner.plan(
                {
                    **planner_payload,
                    "blocked_lms": sorted(hard_blocked_lms | soft_blocked_lms),
                }
            )
            if result.get("ok"):
                debug = result.setdefault("debug", {})
                debug["reason"] = f"{debug.get('reason', 'success')}:detour_soft_blocks"
                debug["softBlockedLms"] = sorted(soft_blocked_lms)
                self._event(
                    "info",
                    f"planner detour around occupied LM(s): {', '.join(sorted(soft_blocked_lms))}",
                )
                return result

            failed_reason = result.get("debug", {}).get("reason", "unknown")
            result = self.planner.plan(
                {
                    **planner_payload,
                    "blocked_lms": sorted(hard_blocked_lms),
                }
            )
            if result.get("ok"):
                debug = result.setdefault("debug", {})
                debug["reason"] = f"{debug.get('reason', 'success')}:fallback_wait"
                debug["softBlockedLms"] = sorted(soft_blocked_lms)
                debug["softBlockFailure"] = failed_reason
                self._event(
                    "warn",
                    "planner found no detour; using original route and runtime waiting",
                )
            return result

        return self.planner.plan(
            {
                **planner_payload,
                "blocked_lms": sorted(hard_blocked_lms),
            }
        )

    def _apply_planner_result(self, result: dict[str, Any], now: float | None = None) -> None:
        now = now or time()
        for plan in result.get("plans", []):
            if not isinstance(plan, dict):
                continue
            name = str(plan.get("robot", ""))
            robot = self.robots.get(name)
            if robot is None:
                continue
            trajectory = [
                item for item in plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            robot.status = "MOVING" if trajectory else "BLOCKED"
            robot.current_lm = str(plan.get("startLm") or robot.current_lm)
            robot.target_lm = str(plan.get("goalLm") or robot.target_lm)
            robot.trajectory = trajectory
            robot.trajectory_dirty = True
            robot.plan_nodes = [
                str(item) for item in plan.get("nodes", [])
            ]
            robot.route_started_at = now
            robot.route_clock = 0.0
            robot.last_tick_at = now
            robot.pose = self._pose_at_trajectory(robot.trajectory, 0.0) or robot.pose
            robot.route_note = self._plan_note(result)
            robot.last_reason = robot.route_note if trajectory else "empty trajectory"
            robot.blocked_since = None
            robot.last_replan_at = now
            robot.updated_at = now

    def _plan_note(self, result: dict[str, Any]) -> str:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return "planner accepted"
        reason = str(debug.get("reason", "") or "")
        blocked_edges = debug.get("blockedEdges", [])
        reserved_edges = int(debug.get("reservedEdges", 0) or 0)
        if "fallback_wait" in reason:
            return "FALLBACK_WAIT"
        if "detour_soft_blocks" in reason:
            return "DETOUR"
        if isinstance(blocked_edges, list) and blocked_edges:
            return "DETOUR: edge blocked"
        if reserved_edges > 0:
            return "DETOUR: reserved edge"
        return "planner accepted"

    def _dynamic_blocked_edges(self) -> set[tuple[str, str]]:
        if not self.obstacles and not self.obstacle_areas:
            return set()
        blocked: set[tuple[str, str]] = set()
        for edge in self.edges:
            route = PlannedRoute(
                nodes=[edge.from_name, edge.to_name],
                edges=[edge],
                length=edge.length,
            )
            try:
                samples = self.planner.route_planner.sample_route(route, sample_distance=0.15)
            except Exception:
                continue
            for sample in samples:
                pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                }
                reason = self.collision.dynamic_blocked_reason(
                    pose,
                    self.obstacles,
                    self.obstacle_areas,
                )
                if reason:
                    blocked.add((edge.from_name, edge.to_name))
                    break
        return blocked

    def _hard_blocked_edges(self, payload: dict[str, Any]) -> set[tuple[str, str]]:
        raw_edges = payload.get("blocked_edges") or payload.get("blockedEdges") or []
        if not isinstance(raw_edges, list):
            return set()
        blocked: set[tuple[str, str]] = set()
        for item in raw_edges:
            if isinstance(item, str) and "->" in item:
                src, dst = item.split("->", 1)
                src = src.strip()
                dst = dst.strip()
            elif isinstance(item, dict):
                src = str(item.get("from") or item.get("fromLm") or "").strip()
                dst = str(item.get("to") or item.get("toLm") or "").strip()
            else:
                continue
            if src in self.landmarks and dst in self.landmarks:
                blocked.add((src, dst))
        return blocked

    def _reserved_constraints(
        self,
        requests: list[dict[str, Any]],
    ) -> tuple[list[tuple[int, str]], list[tuple[int, str, str]]]:
        request_names = {str(request.get("name", "")) for request in requests}
        time_step = self._reservation_time_step()
        horizon = self._reservation_horizon()
        vertices: set[tuple[int, str]] = set()
        edges: set[tuple[int, str, str]] = set()

        for robot in self.robots.values():
            if robot.name in request_names:
                continue
            if not robot.trajectory or robot.status not in {"MOVING", "WAITING"}:
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            tick = 0
            offset = 0.0
            while offset <= horizon + 0.000001:
                sample_clock = min(final_time, robot.route_clock + offset)
                pose = self._pose_at_trajectory(robot.trajectory, sample_clock)
                if pose is None:
                    break
                lm_name = self._nearest_lm_for_pose(pose)
                if lm_name:
                    vertices.add((tick, lm_name))
                edge_id = self._edge_id_at_trajectory(robot.trajectory, sample_clock)
                edge = self._parse_edge_id(edge_id)
                if edge is not None:
                    src, dst = edge
                    edges.add((tick, src, dst))
                    edges.add((tick, dst, src))
                if sample_clock >= final_time:
                    break
                tick += 1
                offset += time_step
        return sorted(vertices), sorted(edges)

    def _reservation_time_step(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(0.25, float(fleet.get("reservation_time_step_sec", 1.0) or 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _reservation_horizon(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 8.0
        try:
            return max(1.0, float(fleet.get("reservation_horizon_sec", 8.0) or 8.0))
        except (TypeError, ValueError):
            return 8.0

    def _nearest_lm_for_pose(self, pose: dict[str, float]) -> str:
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(
                landmark.x - float(pose.get("x", 0.0) or 0.0),
                landmark.y - float(pose.get("y", 0.0) or 0.0),
            ),
        )
        return nearest.name

    def _edge_id_at_trajectory(self, trajectory: list[dict[str, Any]], elapsed: float) -> str:
        if not trajectory:
            return ""
        if len(trajectory) == 1 or elapsed <= float(trajectory[0].get("t", 0.0) or 0.0):
            return str(trajectory[0].get("edgeId", "") or "")
        for index in range(len(trajectory) - 1):
            start = trajectory[index]
            goal = trajectory[index + 1]
            start_t = float(start.get("t", 0.0) or 0.0)
            goal_t = float(goal.get("t", 0.0) or 0.0)
            if start_t <= elapsed <= goal_t:
                return str(goal.get("edgeId") or start.get("edgeId") or "")
        return str(trajectory[-1].get("edgeId", "") or "")

    def _parse_edge_id(self, edge_id: str) -> tuple[str, str] | None:
        if "->" not in edge_id or edge_id.startswith("CURRENT->"):
            return None
        src, dst = edge_id.split("->", 1)
        src = src.strip()
        dst = dst.strip()
        if not src or not dst or src == dst:
            return None
        if src not in self.landmarks or dst not in self.landmarks:
            return None
        return src, dst

    def _hard_blocked_lms(self, payload: dict[str, Any]) -> set[str]:
        blocked_lms = payload.get("blocked_lms", [])
        if not isinstance(blocked_lms, list):
            return set()
        return {
            str(name)
            for name in blocked_lms
            if isinstance(name, str) and name in self.landmarks
        }

    def _soft_blocked_lms(
        self,
        requests: list[dict[str, Any]],
        hard_blocked_lms: set[str],
    ) -> set[str]:
        request_names = {request["name"] for request in requests}
        protected_lms = set(hard_blocked_lms)
        for request in requests:
            protected_lms.add(request["startLm"])
            protected_lms.add(request["goalLm"])

        blocked: set[str] = set()
        for robot in self.robots.values():
            if robot.name in request_names:
                continue
            lm_name = self._nearest_lm_for_robot(robot)
            if not lm_name or lm_name in protected_lms:
                continue
            blocked.add(lm_name)
        return blocked

    def _nearest_lm_for_robot(self, robot: FleetRobot) -> str:
        pose = robot.pose
        if not pose:
            return robot.current_lm if robot.current_lm in self.landmarks else ""
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(
                landmark.x - float(pose.get("x", 0.0) or 0.0),
                landmark.y - float(pose.get("y", 0.0) or 0.0),
            ),
        )
        return nearest.name

    def _advance_runtime(self) -> None:
        now = time()
        for robot in self.robots.values():
            if robot.status in {"BLOCKED", "PLANNING"} and robot.target_lm:
                self._maybe_replan_robot(robot, now, "no active trajectory")
                robot.last_tick_at = now
                continue
            if robot.status not in {"MOVING", "WAITING"}:
                robot.last_tick_at = now
                continue
            if not robot.trajectory:
                if robot.target_lm:
                    self._maybe_replan_robot(robot, now, "empty trajectory")
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            last_tick_at = robot.last_tick_at or now
            dt = min(0.20, max(0.0, now - last_tick_at))
            robot.last_tick_at = now
            proposed_clock = min(final_time, robot.route_clock + dt)
            blocked_reason = self._blocked_ahead(robot, proposed_clock)
            if blocked_reason:
                if robot.status != "WAITING" or robot.last_reason != blocked_reason:
                    self._event("warn", f"{robot.name} waiting: {blocked_reason}")
                robot.status = "WAITING"
                robot.last_reason = blocked_reason
                if robot.blocked_since is None:
                    robot.blocked_since = now
                robot.updated_at = now
                if not self._is_robot_conflict(blocked_reason):
                    self._maybe_replan_robot(robot, now, blocked_reason)
                continue
            if robot.status != "MOVING":
                self._event("info", f"{robot.name} moving")
            robot.status = "MOVING"
            robot.last_reason = "moving"
            robot.blocked_since = None
            robot.route_clock = proposed_clock
            pose = self._pose_at_trajectory(robot.trajectory, robot.route_clock)
            if pose is not None:
                robot.pose = pose
            if final_time > 0.0 and robot.route_clock >= final_time:
                robot.current_lm = robot.target_lm or robot.current_lm
                robot.target_lm = ""
                robot.status = "ARRIVED"
                robot.trajectory = []
                robot.plan_nodes = []
                robot.trajectory_dirty = True
                robot.route_started_at = None
                robot.route_clock = 0.0
                robot.last_tick_at = None
                robot.blocked_since = None
                robot.last_reason = "arrived"
                robot.route_note = ""
                robot.updated_at = now
                self._event("info", f"{robot.name} arrived at {robot.current_lm}")

    def _maybe_replan_robot(self, robot: FleetRobot, now: float, reason: str) -> bool:
        if not robot.target_lm:
            return False
        interval = self._replan_interval()
        if robot.last_replan_at is not None and now - robot.last_replan_at < interval:
            return False

        robot.last_replan_at = now
        start_lm = self._nearest_lm_for_robot(robot)
        if not start_lm or start_lm not in self.landmarks:
            robot.status = "BLOCKED"
            robot.last_reason = "cannot find nearest LM for replan"
            robot.updated_at = now
            return False

        request = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": robot.target_lm,
            "startPose": dict(robot.pose) if robot.pose else self._pose_at_landmark(start_lm),
        }
        no_reverse_edges = self._no_reverse_edges(robot, start_lm)
        result = self._plan_valid_requests(
            [request],
            {
                "robots": [request],
                "blocked_edges": [
                    {"from": src, "to": dst}
                    for src, dst in sorted(no_reverse_edges)
                ],
            },
        )
        if result.get("ok") and result.get("plans"):
            self._apply_planner_result(result, now)
            robot.route_note = f"REPLAN: {self._plan_note(result)}"
            robot.last_reason = robot.route_note
            self._event("info", f"{robot.name} replanned after block: {reason}")
            return True

        robot.status = "WAITING" if robot.trajectory else "BLOCKED"
        robot.last_reason = result.get("debug", {}).get("reason", reason)
        robot.updated_at = now
        self._event("warn", f"{robot.name} replan pending: {robot.last_reason}")
        return False

    def _no_reverse_edges(self, robot: FleetRobot, start_lm: str) -> set[tuple[str, str]]:
        if not robot.plan_nodes or start_lm not in robot.plan_nodes:
            return set()
        index = robot.plan_nodes.index(start_lm)
        if index <= 0:
            return set()
        previous = robot.plan_nodes[index - 1]
        if previous not in self.landmarks:
            return set()
        return {(start_lm, previous)}

    def _replan_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(0.25, float(fleet.get("replan_interval_sec", 1.0) or 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _blocked_ahead(self, robot: FleetRobot, proposed_clock: float) -> str:
        if not robot.trajectory:
            return ""
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        lookahead = self.collision.lookahead_time()
        step = self.collision.sample_time_step()
        end_clock = min(final_time, proposed_clock + lookahead)
        checks = [proposed_clock]
        clock = proposed_clock + step
        while clock <= end_clock + 0.000001:
            checks.append(clock)
            clock += step
        for check_clock in checks:
            pose = self._pose_at_trajectory(robot.trajectory, check_clock)
            if pose is None:
                continue
            offset = max(0.0, check_clock - robot.route_clock)
            reason = self.collision.blocked_reason(
                pose=pose,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
            if reason:
                return reason
            for other in self.robots.values():
                if other.name == robot.name or other.pose is None:
                    continue
                other_pose = self._predicted_robot_pose(other, offset)
                if other_pose is None:
                    continue
                if self.collision.footprints_overlap(pose, other_pose):
                    reason = self._robot_conflict_reason(robot, other, pose, other_pose)
                    if reason:
                        return reason
        return ""

    def _predicted_robot_pose(self, robot: FleetRobot, offset: float) -> dict[str, float] | None:
        if robot.status != "MOVING" or not robot.trajectory:
            return robot.pose
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        return self._pose_at_trajectory(
            robot.trajectory,
            min(final_time, robot.route_clock + max(0.0, offset)),
        )

    def _robot_conflict_reason(
        self,
        robot: FleetRobot,
        other: FleetRobot,
        candidate_pose: dict[str, float],
        other_pose: dict[str, float],
    ) -> str:
        if other.pose is not None and self.collision.footprints_overlap(candidate_pose, other.pose):
            if (
                robot.pose is not None
                and self.collision.footprints_overlap(robot.pose, other.pose)
                and self._candidate_moves_away(robot.pose, candidate_pose, other.pose)
            ):
                return ""
            return f"occupied by {other.name}"

        if self._candidate_moves_away(robot.pose, candidate_pose, other_pose):
            return ""
        if self._should_yield_to(robot, other):
            return f"yield to {other.name}"
        return ""

    def _candidate_moves_away(
        self,
        current_pose: dict[str, float] | None,
        candidate_pose: dict[str, float],
        other_pose: dict[str, float],
    ) -> bool:
        if current_pose is None:
            return False
        current_distance = math.hypot(
            float(current_pose.get("x", 0.0) or 0.0) - float(other_pose.get("x", 0.0) or 0.0),
            float(current_pose.get("y", 0.0) or 0.0) - float(other_pose.get("y", 0.0) or 0.0),
        )
        candidate_distance = math.hypot(
            float(candidate_pose.get("x", 0.0) or 0.0) - float(other_pose.get("x", 0.0) or 0.0),
            float(candidate_pose.get("y", 0.0) or 0.0) - float(other_pose.get("y", 0.0) or 0.0),
        )
        return candidate_distance > current_distance + 0.015

    def _should_yield_to(self, robot: FleetRobot, other: FleetRobot) -> bool:
        if other.status == "WAITING" and robot.name in other.last_reason:
            return False
        if robot.status == "WAITING" and other.name in robot.last_reason:
            return True

        robot_started = robot.route_started_at or robot.updated_at
        other_started = other.route_started_at or other.updated_at
        if abs(robot_started - other_started) > 0.001:
            return robot_started > other_started
        return robot.name > other.name

    def _is_robot_conflict(self, reason: str) -> bool:
        return str(reason).startswith("yield to ") or str(reason).startswith("occupied by ")

    def _pose_at_landmark(self, lm_name: str) -> dict[str, float] | None:
        landmark = self.landmarks.get(lm_name)
        if landmark is None:
            return None
        return {"x": landmark.x, "y": landmark.y, "yaw": 0.0}

    def _pose_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> dict[str, float] | None:
        if not trajectory:
            return None
        if len(trajectory) == 1 or elapsed <= float(trajectory[0].get("t", 0.0) or 0.0):
            return self._pose_from_sample(trajectory[0])
        last = trajectory[-1]
        if elapsed >= float(last.get("t", 0.0) or 0.0):
            return self._pose_from_sample(last)

        index = 0
        while (
            index < len(trajectory) - 2
            and float(trajectory[index + 1].get("t", 0.0) or 0.0) < elapsed
        ):
            index += 1
        start = trajectory[index]
        goal = trajectory[index + 1]
        start_t = float(start.get("t", 0.0) or 0.0)
        goal_t = float(goal.get("t", 0.0) or 0.0)
        span = max(0.0001, goal_t - start_t)
        ratio = (elapsed - start_t) / span
        yaw = self._interpolate_angle(
            float(start.get("yaw", 0.0) or 0.0),
            float(goal.get("yaw", 0.0) or 0.0),
            ratio,
        )
        return {
            "x": float(start.get("x", 0.0) or 0.0)
            + ((float(goal.get("x", 0.0) or 0.0) - float(start.get("x", 0.0) or 0.0)) * ratio),
            "y": float(start.get("y", 0.0) or 0.0)
            + ((float(goal.get("y", 0.0) or 0.0) - float(start.get("y", 0.0) or 0.0)) * ratio),
            "yaw": yaw,
        }

    def _pose_from_sample(self, sample: dict[str, Any]) -> dict[str, float]:
        return {
            "x": float(sample.get("x", 0.0) or 0.0),
            "y": float(sample.get("y", 0.0) or 0.0),
            "yaw": float(sample.get("yaw", 0.0) or 0.0),
        }

    def _interpolate_angle(self, start: float, goal: float, ratio: float) -> float:
        delta = (goal - start + math.pi) % (2.0 * math.pi) - math.pi
        return start + (delta * ratio)

    def _clean_obstacle(self, item: dict[str, Any]) -> dict[str, float]:
        return {
            "x": float(item.get("x", 0.0) or 0.0),
            "y": float(item.get("y", 0.0) or 0.0),
            "radius": max(0.0, float(item.get("radius", 0.08) or 0.08)),
        }

    def _clean_area(self, item: dict[str, Any]) -> dict[str, float]:
        return {
            "x1": float(item.get("x1", 0.0) or 0.0),
            "y1": float(item.get("y1", 0.0) or 0.0),
            "x2": float(item.get("x2", 0.0) or 0.0),
            "y2": float(item.get("y2", 0.0) or 0.0),
        }


class FleetCollisionChecker:
    def __init__(
        self,
        params: dict[str, Any],
        map_dir: Path | None,
        map_metadata: MapMetadata | None,
    ) -> None:
        self.params = params
        self.map_metadata = map_metadata
        self.map_pixels: bytes | None = None
        self.map_width = 0
        self.map_height = 0
        if map_dir is not None:
            self._load_map_pixels(map_dir)

    def set_params(self, params: dict[str, Any]) -> None:
        self.params = params

    def lookahead_time(self) -> float:
        navigation = self._dict_param("navigation")
        speed = max(0.05, float(navigation.get("route_speed", 0.35) or 0.35))
        stop_distance = max(0.08, float(navigation.get("stop_distance", 0.40) or 0.40))
        return max(0.18, min(0.85, stop_distance / speed))

    def sample_time_step(self) -> float:
        manual = self._dict_param("manual")
        return max(0.04, min(0.14, float(manual.get("prediction_step", 0.10) or 0.10)))

    def blocked_reason(
        self,
        pose: dict[str, float],
        obstacles: list[dict[str, float]],
        obstacle_areas: list[dict[str, float]],
    ) -> str:
        points = self.pose_sample_points(pose)
        for point in points:
            if self.map_occupied(point):
                return "map occupancy under footprint"
            for area in obstacle_areas:
                if self.area_contains(area, point):
                    return "obstacle area under footprint"
        for obstacle in obstacles:
            if self.obstacle_hits_pose(obstacle, pose):
                return "point obstacle hits footprint"
        return ""

    def dynamic_blocked_reason(
        self,
        pose: dict[str, float],
        obstacles: list[dict[str, float]],
        obstacle_areas: list[dict[str, float]],
    ) -> str:
        points = self.pose_sample_points(pose)
        for point in points:
            for area in obstacle_areas:
                if self.area_contains(area, point):
                    return "obstacle area under footprint"
        for obstacle in obstacles:
            if self.obstacle_hits_pose(obstacle, pose):
                return "point obstacle hits footprint"
        return ""

    def footprints_overlap(self, first_pose: dict[str, float], second_pose: dict[str, float]) -> bool:
        return self.polygons_overlap(
            self.footprint_corners(first_pose),
            self.footprint_corners(second_pose),
            self.collision_margin(),
        )

    def pose_sample_points(self, pose: dict[str, float]) -> list[dict[str, float]]:
        footprint = self.footprint()
        margin = self.collision_margin()
        min_x = min(point["x"] for point in footprint) - margin
        max_x = max(point["x"] for point in footprint) + margin
        min_y = min(point["y"] for point in footprint) - margin
        max_y = max(point["y"] for point in footprint) + margin
        resolution = self.map_metadata.resolution if self.map_metadata is not None else 0.02
        step = max(0.04, resolution * 2.0)
        points: list[dict[str, float]] = []
        x = min_x
        while x <= max_x + 0.000001:
            y = min_y
            while y <= max_y + 0.000001:
                local = {"x": x, "y": y}
                if self.point_in_polygon(local, footprint) or self.distance_to_polygon(local, footprint) <= margin:
                    points.append(self.local_to_world(pose, local))
                y += step
            x += step
        points.append(self.local_to_world(pose, {"x": 0.0, "y": 0.0}))
        for point in footprint:
            points.append(self.local_to_world(pose, point))
        return points

    def obstacle_hits_pose(self, obstacle: dict[str, float], pose: dict[str, float]) -> bool:
        local = self.world_to_local(pose, obstacle)
        radius = float(obstacle.get("radius", 0.08) or 0.08) + self.collision_margin()
        footprint = self.footprint()
        return self.point_in_polygon(local, footprint) or self.distance_to_polygon(local, footprint) <= radius

    def map_occupied(self, point: dict[str, float]) -> bool:
        if self.map_pixels is None or self.map_metadata is None:
            return False
        pixel = self.world_to_image(point)
        if pixel["x"] < 0 or pixel["y"] < 0 or pixel["x"] >= self.map_width or pixel["y"] >= self.map_height:
            return True
        value = self.map_pixels[(pixel["y"] * self.map_width) + pixel["x"]]
        return value < 82

    def area_contains(self, area: dict[str, float], point: dict[str, float]) -> bool:
        x1 = min(area["x1"], area["x2"])
        x2 = max(area["x1"], area["x2"])
        y1 = min(area["y1"], area["y2"])
        y2 = max(area["y1"], area["y2"])
        return x1 <= point["x"] <= x2 and y1 <= point["y"] <= y2

    def footprint_corners(self, pose: dict[str, float]) -> list[dict[str, float]]:
        return [self.local_to_world(pose, point) for point in self.footprint()]

    def local_to_world(self, pose: dict[str, float], point: dict[str, float]) -> dict[str, float]:
        yaw = float(pose.get("yaw", 0.0) or 0.0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return {
            "x": float(pose.get("x", 0.0) or 0.0) + (point["x"] * cos_yaw) - (point["y"] * sin_yaw),
            "y": float(pose.get("y", 0.0) or 0.0) + (point["x"] * sin_yaw) + (point["y"] * cos_yaw),
        }

    def world_to_local(self, pose: dict[str, float], point: dict[str, float]) -> dict[str, float]:
        yaw = float(pose.get("yaw", 0.0) or 0.0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx = point["x"] - float(pose.get("x", 0.0) or 0.0)
        dy = point["y"] - float(pose.get("y", 0.0) or 0.0)
        return {
            "x": (dx * cos_yaw) + (dy * sin_yaw),
            "y": (-dx * sin_yaw) + (dy * cos_yaw),
        }

    def world_to_image(self, point: dict[str, float]) -> dict[str, int]:
        assert self.map_metadata is not None
        origin = self.map_metadata.origin
        resolution = self.map_metadata.resolution
        return {
            "x": round((point["x"] - float(origin[0])) / resolution),
            "y": round((self.map_metadata.height - 1) - ((point["y"] - float(origin[1])) / resolution)),
        }

    def polygons_overlap(
        self,
        first: list[dict[str, float]],
        second: list[dict[str, float]],
        margin: float,
    ) -> bool:
        axes = self.polygon_axes(first) + self.polygon_axes(second)
        for axis in axes:
            first_projection = self.project_polygon(first, axis)
            second_projection = self.project_polygon(second, axis)
            if (
                first_projection["max"] + margin < second_projection["min"]
                or second_projection["max"] + margin < first_projection["min"]
            ):
                return False
        return True

    def polygon_axes(self, polygon: list[dict[str, float]]) -> list[dict[str, float]]:
        axes: list[dict[str, float]] = []
        for index, start in enumerate(polygon):
            end = polygon[(index + 1) % len(polygon)]
            dx = end["x"] - start["x"]
            dy = end["y"] - start["y"]
            length = math.hypot(dx, dy)
            if length > 0.000001:
                axes.append({"x": -dy / length, "y": dx / length})
        return axes

    def project_polygon(self, polygon: list[dict[str, float]], axis: dict[str, float]) -> dict[str, float]:
        values = [(point["x"] * axis["x"]) + (point["y"] * axis["y"]) for point in polygon]
        return {"min": min(values), "max": max(values)}

    def point_in_polygon(self, point: dict[str, float], polygon: list[dict[str, float]]) -> bool:
        inside = False
        j = len(polygon) - 1
        for i, a in enumerate(polygon):
            b = polygon[j]
            crosses = (
                (a["y"] > point["y"]) != (b["y"] > point["y"])
                and point["x"]
                < ((b["x"] - a["x"]) * (point["y"] - a["y"])) / ((b["y"] - a["y"]) or 0.000001)
                + a["x"]
            )
            if crosses:
                inside = not inside
            j = i
        return inside

    def distance_to_polygon(self, point: dict[str, float], polygon: list[dict[str, float]]) -> float:
        return min(
            self.distance_to_segment(point, polygon[index], polygon[(index + 1) % len(polygon)])
            for index in range(len(polygon))
        )

    def distance_to_segment(
        self,
        point: dict[str, float],
        start: dict[str, float],
        end: dict[str, float],
    ) -> float:
        dx = end["x"] - start["x"]
        dy = end["y"] - start["y"]
        length_sq = (dx * dx) + (dy * dy)
        if length_sq <= 0.000001:
            return math.hypot(point["x"] - start["x"], point["y"] - start["y"])
        ratio = max(0.0, min(1.0, (((point["x"] - start["x"]) * dx) + ((point["y"] - start["y"]) * dy)) / length_sq))
        closest = {"x": start["x"] + (dx * ratio), "y": start["y"] + (dy * ratio)}
        return math.hypot(point["x"] - closest["x"], point["y"] - closest["y"])

    def footprint(self) -> list[dict[str, float]]:
        robot_model = self._dict_param("robot_model")
        raw_footprint = robot_model.get("footprint")
        if not isinstance(raw_footprint, list) or len(raw_footprint) < 3:
            raw_footprint = [
                {"x": 0.35, "y": 0.275},
                {"x": 0.35, "y": -0.275},
                {"x": -0.35, "y": -0.275},
                {"x": -0.35, "y": 0.275},
            ]
        return [
            {
                "x": float(point.get("x", 0.0) or 0.0),
                "y": float(point.get("y", 0.0) or 0.0),
            }
            for point in raw_footprint
            if isinstance(point, dict)
        ]

    def collision_margin(self) -> float:
        navigation = self._dict_param("navigation")
        return max(0.0, float(navigation.get("collision_margin", 0.04) or 0.04))

    def _dict_param(self, key: str) -> dict[str, Any]:
        value = self.params.get(key, {})
        return value if isinstance(value, dict) else {}

    def _load_map_pixels(self, map_dir: Path) -> None:
        try:
            loader = WarehouseMapLoader(map_dir)
            ros_map_yaml = loader._find_ros_map_yaml()
            ros_map = loader._read_yaml(ros_map_yaml)
            if not isinstance(ros_map, dict):
                return
            image_path = (map_dir / str(ros_map["image"])).resolve()
            width, height, pixels = loader._load_pgm(image_path)
        except Exception:
            return
        self.map_width = width
        self.map_height = height
        self.map_pixels = pixels
