from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from threading import Lock, Thread
from time import time
from typing import Any
from urllib.parse import urlparse

from fleet_manager.mapf import FleetMapfPlanner
from fleet_manager.route_core import (
    GraphEdge,
    Landmark,
    MapMetadata,
    PlannedRoute,
    WarehouseMapLoader,
)
from fleet_manager.robot_grpc_api.client import GrpcRobotAdapter
from fleet_manager.robot_grpc_api.contracts import DEFAULT_GRPC_PORT, normalize_grpc_endpoint


TERMINAL_ORDER_STATUSES = {"COMPLETED", "FAILED", "CANCELED"}
ORDER_SEQUENCE_KEYS = ("targets", "targetLms", "goals", "orders", "queue", "blocks")
ORDER_ID_KEYS = ("id", "orderId", "taskId")
ORDER_TARGET_KEYS = ("targetLm", "goalLm", "location", "target", "LM")
FLEET_CONTROL_OWNER_ID = "fleet-manager"
FLEET_CONTROL_OWNER_NAME = "Fleet Manager"
EXTERNAL_CONTROL_PAUSE_PREFIX = "external control active:"


@dataclass
class FleetRobot:
    name: str
    current_lm: str
    mode: str = "simulated"
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
    active_order_id: str = ""
    base_url: str = ""
    remote_id: str = ""
    remote_online: bool = True
    remote_error: str = ""
    remote_status: dict[str, Any] = field(default_factory=dict)
    remote_last_poll_at: float | None = None
    route_revision: int = 0
    route_chunk_index: int = 0
    route_chunk_goal_lm: str = ""
    route_final_lm: str = ""
    route_preview: list[dict[str, Any]] = field(default_factory=list)
    route_preview_dirty: bool = False
    has_executed_route: bool = False
    pending_route: dict[str, Any] | None = None
    retreat_target_clock: float | None = None
    retreat_target_lm: str = ""
    retreat_blocked_edges: list[tuple[str, str]] = field(default_factory=list)
    traffic_priority_until: float = 0.0
    wait_for_robot: str = ""
    wait_resource: str = ""
    wait_release_at: float = 0.0
    traffic_stall_since: float | None = None

    def to_dict(self, include_trajectory: bool = True) -> dict[str, Any]:
        # target_lm is the end of the currently committed rolling chunk.  The
        # operator, however, asked for route_final_lm and should not see the
        # short internal horizon waypoint as if it were the order destination.
        display_target_lm = self.route_final_lm or self.target_lm
        return {
            "name": self.name,
            "currentLm": self.current_lm,
            "mode": self.mode,
            "type": self.mode,
            "targetName": display_target_lm,
            "targetLm": display_target_lm,
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
            "activeOrderId": self.active_order_id,
            "baseUrl": self.base_url,
            "remoteId": self.remote_id,
            "online": self.remote_online,
            "remoteError": self.remote_error,
            "remoteStatus": self.remote_status,
            "remoteLastPollAt": self.remote_last_poll_at,
            "routeRevision": self.route_revision,
            "routeChunkIndex": self.route_chunk_index,
            "routeChunkGoalLm": self.route_chunk_goal_lm,
            "routeFinalLm": self.route_final_lm,
            "routePreview": self.route_preview if include_trajectory or self.route_preview_dirty else [],
            "trafficPriorityUntil": self.traffic_priority_until,
            "waitDependency": (
                {
                    "robot": self.wait_for_robot,
                    "resource": self.wait_resource,
                    "releaseAt": self.wait_release_at,
                }
                if self.wait_for_robot
                else None
            ),
            "trafficStallSince": self.traffic_stall_since,
        }

    def is_remote(self) -> bool:
        return self.mode in {"remote", "robot", "real", "grpc", "aivison_grpc", "real_grpc"}


@dataclass
class FleetOrder:
    order_id: str
    target_lm: str
    vehicle: str = ""
    priority: int = 0
    status: str = "QUEUED"
    created_at: float = field(default_factory=time)
    updated_at: float = field(default_factory=time)
    assigned_robot: str = ""
    start_lm: str = ""
    route_nodes: list[str] = field(default_factory=list)
    error: str = ""
    external_id: str = ""
    targets: list[str] = field(default_factory=list)
    step_index: int = 0
    speed: float = 0.0
    acceleration: float = 0.0
    rotate: bool = False
    turn_speed: float = 0.0
    stretch_motion_to_reservation_ticks: bool = True
    dispatch_failures: int = 0
    traffic_detour_edges: list[tuple[str, str]] = field(default_factory=list)
    traffic_detour_attempts: int = 0
    spatial_route_nodes: list[str] = field(default_factory=list)
    spatial_route_revision: int = 0
    traffic_blocked_since: float | None = None

    def to_dict(self) -> dict[str, Any]:
        targets = self.targets or ([self.target_lm] if self.target_lm else [])
        current_step = max(0, min(self.step_index, max(0, len(targets) - 1)))
        current_target = targets[current_step] if targets else self.target_lm
        return {
            "id": self.order_id,
            "orderId": self.order_id,
            "externalId": self.external_id,
            "vehicle": self.vehicle,
            "targetLm": current_target,
            "targets": targets,
            "currentStep": current_step,
            "totalSteps": len(targets),
            "steps": self._steps_payload(targets, current_step),
            "priority": self.priority,
            "status": self.status,
            "createdAt": self.created_at,
            "updatedAt": self.updated_at,
            "assignedRobot": self.assigned_robot,
            "startLm": self.start_lm,
            "routeNodes": self.route_nodes,
            "error": self.error,
            "speed": self.speed,
            "acceleration": self.acceleration,
            "rotate": self.rotate,
            "turnSpeed": self.turn_speed,
            "stretchMotionToReservationTicks": self.stretch_motion_to_reservation_ticks,
            "dispatchFailures": self.dispatch_failures,
            "trafficDetourAttempts": self.traffic_detour_attempts,
            "trafficDetourEdges": [f"{src}->{dst}" for src, dst in self.traffic_detour_edges],
            "spatialRouteNodes": self.spatial_route_nodes,
            "spatialRouteRevision": self.spatial_route_revision,
            "trafficBlockedSince": self.traffic_blocked_since,
        }

    def _steps_payload(self, targets: list[str], current_step: int) -> list[dict[str, Any]]:
        steps = []
        for index, target_lm in enumerate(targets):
            if self.status == "CANCELED" and index >= current_step:
                status = "CANCELED"
            elif self.status == "FAILED" and index >= current_step:
                status = "FAILED"
            elif self.status == "COMPLETED" or index < current_step:
                status = "COMPLETED"
            elif index == current_step:
                status = self.status
            else:
                status = "QUEUED"
            steps.append({
                "index": index,
                "targetLm": target_lm,
                "status": status,
            })
        return steps


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
        remote_adapter: Any | None = None,
    ) -> None:
        self.landmarks = landmarks
        self.edges = edges
        self.params = params or {}
        self.planner = FleetMapfPlanner(landmarks, edges, params=params)
        self.robots: dict[str, FleetRobot] = {}
        self.orders: dict[str, FleetOrder] = {}
        self.events: list[FleetEvent] = []
        self.obstacles: list[dict[str, float]] = []
        self.obstacle_areas: list[dict[str, float]] = []
        self.active_robot_modes: set[str] | None = None
        self.collision = FleetCollisionChecker(
            params=self.params,
            map_dir=map_dir,
            map_metadata=map_metadata,
        )
        self._static_blocked_edges = self._static_map_blocked_edges()
        if self._static_blocked_edges:
            self._event(
                "error",
                f"map audit blocked {len(self._static_blocked_edges)} graph edge(s) "
                "that intersect occupancy",
            )
        self._external_remote_adapter = remote_adapter
        self.remote_adapter = remote_adapter or GrpcRobotAdapter(timeout=self._remote_timeout())
        self._route_revision_seq = int(time() * 1000)
        self._planner_lock = Lock()
        self._dispatch_job_lock = Lock()
        self._dispatch_job: dict[str, Any] | None = None
        self._last_async_job_kind = ""
        self._coupled_replan_last_attempt: dict[tuple[str, ...], float] = {}
        self._coupled_replan_failures: dict[tuple[str, ...], int] = {}
        self._active_wait_cycles: dict[tuple[str, ...], float] = {}
        self.traffic_metrics: dict[str, int] = {
            "waitCyclesDetected": 0,
            "waitCyclesResolved": 0,
            "cycleReplans": 0,
            "coupledReplansStarted": 0,
            "coupledReplansSucceeded": 0,
            "coupledReplansFailed": 0,
            "priorityGrants": 0,
            "runtimeSafetyRollbacks": 0,
        }

    def set_active_robot_modes(self, modes: set[str] | list[str] | tuple[str, ...] | None) -> None:
        if modes is None:
            self.active_robot_modes = None
            return
        clean_modes = {str(mode or "").strip().lower() for mode in modes if str(mode or "").strip()}
        self.active_robot_modes = clean_modes or None

    def state(self, include_trajectories: bool = True) -> dict[str, Any]:
        self._advance_runtime()
        return self._state_snapshot(include_trajectories=include_trajectories)

    def advance_runtime(self) -> None:
        self._advance_runtime()

    def snapshot(self, include_trajectories: bool = True) -> dict[str, Any]:
        return self._state_snapshot(include_trajectories=include_trajectories)

    def _state_snapshot(
        self,
        include_trajectories: bool = True,
        route_revisions: dict[str, int] | None = None,
    ) -> dict[str, Any]:
        return {
            "ok": True,
            "robots": [
                robot.to_dict(
                    include_trajectory=(
                        include_trajectories
                        or (
                            route_revisions is not None
                            and int(route_revisions.get(robot.name, -1)) != robot.route_revision
                        )
                    )
                )
                for robot in self._runtime_robots()
            ],
            "events": [event.to_dict() for event in self.events[-80:]],
            "obstacles": self.obstacles,
            "obstacleAreas": self.obstacle_areas,
            "orders": self._orders_list(),
            "traffic": dict(self.traffic_metrics),
        }

    def tick(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._advance_runtime()
        return self.stream_tick()

    def stream_tick(self, route_revisions: dict[str, int] | None = None) -> dict[str, Any]:
        state = self._state_snapshot(
            include_trajectories=False,
            route_revisions=route_revisions,
        )
        for robot in self._runtime_robots():
            robot.trajectory_dirty = False
            robot.route_preview_dirty = False
        return state

    def _should_stream_trajectory(self, robot: FleetRobot) -> bool:
        return bool(
            robot.trajectory
            and robot.status in {"MOVING", "WAITING", "BLOCKED", "PLANNING"}
        )

    def _robot_mode_key(self, robot: FleetRobot) -> str:
        return "remote" if robot.is_remote() else "simulated"

    def _robot_enabled(self, robot: FleetRobot) -> bool:
        if self.active_robot_modes is None:
            return True
        return self._robot_mode_key(robot) in self.active_robot_modes

    def _runtime_robots(self) -> list[FleetRobot]:
        return [
            robot
            for robot in self.robots.values()
            if self._robot_enabled(robot)
        ]

    def _order_enabled(self, order: FleetOrder) -> bool:
        if self.active_robot_modes is None:
            return True
        robot_name = order.assigned_robot or order.vehicle
        if not robot_name:
            return True
        robot = self.robots.get(robot_name)
        return bool(robot is not None and self._robot_enabled(robot))

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
            if self._external_remote_adapter is None:
                self.remote_adapter = GrpcRobotAdapter(timeout=self._remote_timeout())
        counts = (len(self.obstacles), len(self.obstacle_areas))
        if counts != previous_counts:
            self._event(
                "info",
                f"world synced: obstacles={counts[0]}, areas={counts[1]}",
            )
        return {"ok": True, "state": self.state()}

    def check_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._advance_runtime()
        name = str(payload.get("name", "")).strip()
        poses = payload.get("poses", [])
        if not isinstance(poses, list):
            poses = []
        step = self.collision.sample_time_step()
        for index, item in enumerate(poses):
            if not isinstance(item, dict):
                continue
            pose = {
                "x": float(item.get("x", 0.0) or 0.0),
                "y": float(item.get("y", 0.0) or 0.0),
                "yaw": float(item.get("yaw", 0.0) or 0.0),
            }
            reason = self.collision.blocked_reason(
                pose=pose,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
            if reason:
                return {
                    "ok": True,
                    "blocked": True,
                    "reason": reason,
                    "index": index,
                    "pose": pose,
                }
            offset = index * step
            for other in self._runtime_robots():
                if other.name == name or other.pose is None:
                    continue
                other_pose = self._predicted_robot_pose(other, offset) or other.pose
                if other_pose is not None and self.collision.robot_footprints_conflict(pose, other_pose):
                    return {
                        "ok": True,
                        "blocked": True,
                        "reason": f"robot footprint conflict with {other.name}",
                        "index": index,
                        "pose": pose,
                    }
        return {"ok": True, "blocked": False, "reason": ""}

    def orders_payload(self) -> dict[str, Any]:
        self._advance_runtime()
        return {
            "ok": True,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def set_order(
        self,
        payload: dict[str, Any],
        *,
        dispatch: bool = True,
    ) -> dict[str, Any]:
        orders = self._build_orders(payload)
        if not orders:
            raise ValueError("no orders to queue")
        replace_active = bool(
            payload.get("replaceActive")
            or payload.get("replace_active")
            or payload.get("replace")
        )
        incoming_ids = set()
        for order in orders:
            if order.order_id in incoming_ids:
                raise ValueError(f"duplicate order id in payload: {order.order_id}")
            incoming_ids.add(order.order_id)
            existing = self.orders.get(order.order_id)
            if existing is not None and existing.status not in TERMINAL_ORDER_STATUSES:
                raise ValueError(f"active order already exists: {order.order_id}")

        if replace_active:
            for vehicle in sorted({order.vehicle for order in orders if order.vehicle}):
                self._replace_orders_for_robot(vehicle, "replaced by operator")

        for order in orders:
            self.orders[order.order_id] = order
            self._event(
                "info",
                f"order queued: {order.order_id} {order.vehicle or 'auto'}->{order.target_lm}",
            )
        if dispatch:
            self._dispatch_orders()
        if len(orders[0].targets or []) > 1:
            first = orders[0]
            self._event(
                "info",
                f"order sequence queued: {len(first.targets)} LM step(s) for {first.vehicle or 'auto'}",
            )
        return {
            "ok": True,
            "order": orders[0].to_dict(),
            "queuedOrders": [order.to_dict() for order in orders],
            "orders": self._orders_list(),
            "state": (
                self.state()
                if dispatch
                else self._state_snapshot(include_trajectories=True)
            ),
        }

    def dispatch_orders(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        dispatched = self._dispatch_orders(force=True)
        return {
            "ok": True,
            "dispatched": dispatched,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def cancel_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        self._cancel_order(order, "canceled by operator")
        return {
            "ok": True,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def pause_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        if order.status in TERMINAL_ORDER_STATUSES:
            raise ValueError(f"cannot pause terminal order: {order_id}")
        self._pause_order(order, "paused by operator")
        return {
            "ok": True,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def resume_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        if order.status in TERMINAL_ORDER_STATUSES:
            raise ValueError(f"cannot resume terminal order: {order_id}")
        order.status = "QUEUED"
        order.error = ""
        order.updated_at = time()
        self._event("info", f"order resumed: {order.order_id}")
        dispatched = self._dispatch_orders(force=True)
        return {
            "ok": True,
            "dispatched": dispatched,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def clear_orders(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        include_active = bool(payload.get("includeActive", False))
        canceled = 0
        for order in list(self.orders.values()):
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if include_active or order.status == "QUEUED":
                self._cancel_order(order, "cleared by operator")
                canceled += 1
        if canceled:
            self._event("warn", f"orders cleared: {canceled}")
        return {
            "ok": True,
            "canceled": canceled,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def add_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = self._robot_mode_from_payload(payload)
        requested_name = self._robot_name_from_payload(payload)
        name = requested_name
        base_url = ""
        remote_status: dict[str, Any] | None = None
        remote_identity: dict[str, Any] | None = None
        if mode != "simulated":
            base_url = self._remote_base_url_from_payload(payload)
            if not base_url:
                raise ValueError("robot IP is required for remote robot")
            try:
                remote_identity = self.remote_adapter.identity(base_url)
                remote_status = self.remote_adapter.status(base_url)
            except Exception as exc:
                raise ValueError(f"remote robot is not reachable: {exc}") from exc
        current_lm = "" if mode != "simulated" else str(payload.get("currentLm") or payload.get("spawnLm") or "").strip()
        remote_pose: dict[str, float] | None = None
        if remote_status is not None:
            status_robot = self._remote_status_robot(remote_status)
            current_lm = current_lm or str(
                status_robot.get("nearestLm")
                or status_robot.get("currentLm")
                or status_robot.get("currentLM")
                or status_robot.get("currentStation")
                or status_robot.get("current_station")
                or ""
            ).strip()
            remote_pose = self._remote_pose_from_status(status_robot)
            if current_lm not in self.landmarks and remote_pose is not None:
                current_lm = self._nearest_lm_for_pose(remote_pose)
            if not name:
                name = self._remote_robot_name(remote_identity, status_robot, base_url)
            name = self._remote_unique_robot_name(name, base_url)
        if not name:
            raise ValueError("robot name is required")
        if not current_lm:
            if mode != "simulated":
                raise ValueError("remote robot has no current LM or localized pose yet; wait for robot status")
            raise ValueError("currentLm/spawnLm is required")
        if current_lm not in self.landmarks:
            raise ValueError(f"unknown LM: {current_lm}")

        robot = self.robots.get(name)
        if robot is None:
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                mode=mode,
                pose=self._pose_at_landmark(current_lm),
                base_url=base_url,
                remote_id=self._remote_identity_id(remote_identity),
                remote_online=True,
            )
            self.robots[name] = robot
            if remote_status is not None:
                self._apply_remote_status(robot, remote_status, time())
            self._event("info", f"robot added: {name}@{current_lm}" + (f" remote={base_url}" if robot.is_remote() else ""))
        else:
            self._cancel_active_order_for_robot(robot, "robot respawned")
            if robot.is_remote():
                self._cancel_remote_route(robot, "robot respawned")
            robot.mode = mode
            robot.base_url = base_url
            robot.remote_id = self._remote_identity_id(remote_identity)
            robot.remote_online = True
            robot.remote_error = ""
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
            robot.route_revision = 0
            robot.route_chunk_index = 0
            robot.route_chunk_goal_lm = ""
            robot.route_final_lm = ""
            robot.route_preview = []
            robot.route_preview_dirty = True
            robot.updated_at = time()
            if remote_status is not None:
                self._apply_remote_status(robot, remote_status, time())
            self._event("info", f"robot updated: {name}@{current_lm}" + (f" remote={base_url}" if robot.is_remote() else ""))
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def remove_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        removed = self.robots.pop(name, None)
        if removed is not None:
            self._cancel_active_order_for_robot(removed, "robot removed")
            self._cancel_orders_for_robot(name, "robot removed")
            self._event("warn", f"robot removed: {name}")
        return {"ok": True, "removed": removed is not None, "state": self.state()}

    def stop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if name:
            robot = self.robots.get(name)
            if robot is None:
                raise ValueError(f"unknown robot: {name}")
            self._stop_robot(robot)
            self._cancel_orders_for_robot(name, "robot stopped")
            self._event("warn", f"robot stopped: {name}")
        else:
            for robot in self._runtime_robots():
                self._stop_robot(robot)
            self._cancel_all_orders("fleet stopped")
            self._event("warn", "fleet stopped")
        return {"ok": True, "state": self.state()}

    def teleop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            raise ValueError(f"unknown robot: {name}")
        if not robot.is_remote() or not robot.base_url:
            raise ValueError(f"{name} is not a remote robot")

        linear = float(payload.get("linear", 0.0) or 0.0)
        angular = float(payload.get("angular", 0.0) or 0.0)
        timeout_ms = max(80, int(payload.get("timeoutMs", 350) or 350))
        if robot.active_order_id:
            self._cancel_active_order_for_robot(robot, "manual control takeover")
        try:
            self._ensure_remote_control(robot, "manual control")
            response = self.remote_adapter.teleop(
                robot.base_url,
                linear=linear,
                angular=angular,
                timeout_ms=timeout_ms,
                owner_id=FLEET_CONTROL_OWNER_ID,
            )
            robot.remote_online = True
            robot.remote_error = ""
            status = response.get("status")
            if isinstance(status, dict):
                self._apply_remote_status(robot, status, time())
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote teleop failed: {exc}"
            robot.updated_at = time()
            raise ValueError(robot.last_reason) from exc

        robot.status = "MANUAL"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.trajectory_dirty = True
        robot.last_reason = "manual control active"
        self._clear_remote_route_metadata(robot)
        robot.updated_at = time()
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def teleop_stop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            raise ValueError(f"unknown robot: {name}")
        if not robot.is_remote() or not robot.base_url:
            raise ValueError(f"{name} is not a remote robot")
        try:
            self._ensure_remote_control(robot, "manual stop")
            response = self.remote_adapter.teleop_stop(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
            status = response.get("status")
            if isinstance(status, dict):
                self._apply_remote_status(robot, status, time())
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote teleop stop failed: {exc}"
            robot.updated_at = time()
            raise ValueError(robot.last_reason) from exc

        if robot.status == "MANUAL":
            robot.status = "IDLE"
            robot.last_reason = "manual control released"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.trajectory_dirty = True
        self._clear_remote_route_metadata(robot)
        robot.updated_at = time()
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def note_external_control_takeover(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
    ) -> bool:
        """Mirror a direct robot takeover into Fleet Manager immediately.

        The robot remains the source of truth for ownership.  This local note
        closes the short race where an operator can acquire and release again
        before the next remote status poll observes the foreign owner.
        """
        try:
            normalized = normalize_grpc_endpoint(endpoint, default_port=DEFAULT_GRPC_PORT)
        except Exception:
            normalized = str(endpoint or "").strip()
        parsed_endpoint = urlparse(normalized)
        endpoint_key = (
            str(parsed_endpoint.hostname or "").lower(),
            int(parsed_endpoint.port or DEFAULT_GRPC_PORT),
        )
        robot = next(
            (
                candidate
                for candidate in self.robots.values()
                if candidate.is_remote()
                and (
                    candidate.base_url == normalized
                    or (
                        str(urlparse(candidate.base_url).hostname or "").lower(),
                        int(urlparse(candidate.base_url).port or DEFAULT_GRPC_PORT),
                    ) == endpoint_key
                )
            ),
            None,
        )
        if robot is None:
            return False

        status = dict(robot.remote_status) if isinstance(robot.remote_status, dict) else {}
        control = dict(status.get("control")) if isinstance(status.get("control"), dict) else {}
        control.update({"state": "OWNED", "ownerId": owner_id, "ownerName": owner_name})
        status.update(
            {
                "control": control,
                "controlState": "OWNED",
                "controlOwner": owner_id,
                "controlOwnerName": owner_name,
            }
        )
        robot.remote_status = status
        now = time()
        if robot.active_order_id:
            order = self.orders.get(robot.active_order_id)
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                self._pause_order_for_external_control(robot, order, now, owner_name or owner_id)
                return True
        robot.status = "MANUAL"
        robot.last_reason = f"{EXTERNAL_CONTROL_PAUSE_PREFIX} {owner_name or owner_id}"
        robot.updated_at = now
        return True

    def reset_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        target_names = [name] if name else [robot.name for robot in self._runtime_robots()]
        for robot_name in target_names:
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            self._cancel_active_order_for_robot(robot, "robot reset")
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
            self._clear_remote_route_metadata(robot)
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
        if "mode" in payload or "type" in payload or "robotMode" in payload:
            robot.mode = self._robot_mode_from_payload(payload)
        if "baseUrl" in payload or "url" in payload or "host" in payload:
            robot.base_url = self._remote_base_url_from_payload(payload)
        if "status" in payload and payload["status"]:
            robot.status = str(payload["status"])
        if "pose" in payload and isinstance(payload["pose"], dict):
            pose = payload["pose"]
            robot.pose = {
                "x": float(pose.get("x", 0.0) or 0.0),
                "y": float(pose.get("y", 0.0) or 0.0),
                "yaw": float(pose.get("yaw", 0.0) or 0.0),
            }
        if robot.status in {"IDLE", "ARRIVED", "BLOCKED", "STOPPED", "MANUAL", "MANUAL_BLOCKED"} and not robot.target_lm:
            self._cancel_active_order_for_robot(robot, f"robot status {robot.status.lower()}")
            robot.trajectory = []
            robot.plan_nodes = []
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = None
            robot.trajectory_dirty = True
            self._clear_remote_route_metadata(robot)
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
            authoritative_pose = (
                dict(robot.pose)
                if robot is not None and isinstance(robot.pose, dict)
                else start_pose
            )
            if robot is not None and robot.trajectory:
                safe_start_lm = self._safe_replan_start_lm(robot)
                if not safe_start_lm:
                    self._event(
                        "warn",
                        f"order deferred for {name}: robot is between LMs; "
                        "keeping the current graph edge",
                    )
                    continue
                if safe_start_lm != start_lm:
                    self._event(
                        "info",
                        f"corrected stale start LM for {name}: {start_lm}->{safe_start_lm}",
                    )
                    start_lm = safe_start_lm
            if isinstance(authoritative_pose, dict) and not self._pose_is_at_lm(
                authoritative_pose,
                start_lm,
            ):
                self._event(
                    "error",
                    f"order rejected for {name}: pose is not at {start_lm}; "
                    "off-graph approach is forbidden",
                )
                continue
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
            if isinstance(authoritative_pose, dict):
                clean_request["startPose"] = {
                    "x": float(authoritative_pose.get("x", 0.0) or 0.0),
                    "y": float(authoritative_pose.get("y", 0.0) or 0.0),
                    "yaw": float(authoritative_pose.get("yaw", 0.0) or 0.0),
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
            deadlock = self._planner_deadlock_result(result)
            reason = self._planner_failure_reason(result)
            for request in valid_requests:
                if not isinstance(request, dict):
                    continue
                robot = self.robots.get(str(request.get("name", "")).strip())
                if robot is not None:
                    robot.status = "WAITING" if deadlock else "BLOCKED"
                    robot.last_reason = reason
                    if deadlock:
                        robot.blocked_since = time()
                        robot.trajectory = []
                        robot.plan_nodes = []
                        robot.trajectory_dirty = True
                        robot.route_started_at = None
                        robot.route_clock = 0.0
                        robot.last_tick_at = None
                        robot.route_note = "DEADLOCK"
                    robot.updated_at = time()
            self._event("error", f"planner rejected: {reason}")

        return {
            **result,
            "fleetState": self.state(),
        }

    def _event(self, level: str, message: str) -> None:
        self.events.append(FleetEvent(stamp=time(), level=level, message=message))
        self.events = self.events[-200:]

    def _orders_list(self) -> list[dict[str, Any]]:
        status_rank = {
            "EXECUTING": 0,
            "WAITING_TRAFFIC": 1,
            "WAITING_OBSTACLE": 1,
            "PLANNING": 2,
            "PAUSED": 2,
            "ASSIGNED": 2,
            "QUEUED": 3,
            "COMPLETED": 4,
            "FAILED": 5,
            "CANCELED": 6,
        }
        ordered = sorted(
            [order for order in self.orders.values() if self._order_enabled(order)],
            key=lambda item: (
                status_rank.get(item.status, 9),
                -int(item.priority or 0),
                item.created_at,
            ),
        )
        return [order.to_dict() for order in ordered[:120]]

    def _build_orders(self, payload: dict[str, Any]) -> list[FleetOrder]:
        return [self._build_order(payload)]

    def _build_order(self, payload: dict[str, Any]) -> FleetOrder:
        order_id = str(
            payload.get("id")
            or payload.get("orderId")
            or payload.get("taskId")
            or payload.get("externalId")
            or ""
        ).strip()
        if not order_id:
            order_id = f"order-{int(time() * 1000)}"

        targets = self._target_lms_from_payload(payload)
        if not targets:
            raise ValueError("targetLm/goalLm/location is required")
        for target_lm in targets:
            if target_lm not in self.landmarks:
                raise ValueError(f"unknown target LM: {target_lm}")

        vehicle = str(
            payload.get("vehicle")
            or payload.get("robot")
            or payload.get("robotName")
            or payload.get("name")
            or ""
        ).strip()
        if vehicle and vehicle not in self.robots:
            raise ValueError(f"unknown robot: {vehicle}")

        try:
            priority = int(payload.get("priority", 0) or 0)
        except (TypeError, ValueError):
            priority = 0
        external_id = str(payload.get("externalId") or payload.get("taskId") or "").strip()
        speed = self._float_payload(payload, ("speed", "routeSpeed"), 0.0)
        acceleration = self._float_payload(payload, ("acceleration", "routeAcceleration", "route_acceleration"), 0.0)
        rotate = self._bool_payload(payload, ("rotate", "simulateRotation", "simulate_rotation"), False)
        turn_speed = self._float_payload(payload, ("turnSpeed", "turn_speed", "rotationSpeed", "rotation_speed"), 0.0)
        stretch_motion = self._bool_payload(
            payload,
            ("stretchMotionToReservationTicks", "stretch_motion_to_reservation_ticks"),
            True,
        )
        return FleetOrder(
            order_id=order_id,
            target_lm=targets[0],
            vehicle=vehicle,
            priority=priority,
            external_id=external_id,
            targets=targets,
            speed=speed,
            acceleration=acceleration,
            rotate=rotate,
            turn_speed=turn_speed,
            stretch_motion_to_reservation_ticks=stretch_motion,
        )

    def _float_payload(self, payload: dict[str, Any], keys: tuple[str, ...], default: float = 0.0) -> float:
        for key in keys:
            if key not in payload:
                continue
            try:
                return float(payload.get(key) or default)
            except (TypeError, ValueError):
                return default
        return default

    def _bool_payload(self, payload: dict[str, Any], keys: tuple[str, ...], default: bool = False) -> bool:
        for key in keys:
            if key not in payload:
                continue
            value = payload.get(key)
            if isinstance(value, str):
                return value.strip().lower() in {"1", "true", "yes", "on"}
            return bool(value)
        return default

    def _target_lms_from_payload(self, payload: dict[str, Any]) -> list[str]:
        targets: list[str] = []
        for key in ORDER_SEQUENCE_KEYS:
            raw_sequence = payload.get(key)
            if not isinstance(raw_sequence, list):
                continue
            for item in raw_sequence:
                target_lm = self._target_lm_from_payload_item(item)
                if target_lm:
                    targets.append(target_lm)
            if targets:
                return targets

        target_lm = self._target_lm_from_payload_item(payload)
        return [target_lm] if target_lm else []

    def _target_lm_from_payload_item(self, item: Any) -> str:
        if isinstance(item, dict):
            for key in ORDER_TARGET_KEYS:
                target_lm = str(item.get(key) or "").strip()
                if target_lm:
                    return target_lm
            return ""
        return str(item or "").strip()

    def _dispatch_orders(
        self,
        force: bool = False,
        *,
        async_simulated: bool = False,
    ) -> int:
        dispatched = self._finish_async_simulated_dispatch() if async_simulated else 0
        now = time()
        queued_orders = [
            order for order in self.orders.values()
            if order.status == "QUEUED"
            and (
                force
                or not order.error
                or now - order.updated_at >= self._order_dispatch_retry_interval(order)
            )
        ]
        queued_orders.sort(
            key=lambda item: (
                bool(item.error),
                -int(item.priority or 0),
                item.updated_at if item.error else item.created_at,
                item.order_id,
            )
        )

        # Dynamic orders used to call MAPF independently for every idle robot.
        # Apart from being expensive, that made all other robots look parked
        # and produced a fleet full of wait-only plans.  Plan a small coupled
        # component at a time and cap synchronous work per runtime tick.
        handled: set[str] = set()
        ready = self._ready_simulated_order_entries(queued_orders)
        if async_simulated and not self._async_simulated_dispatch_active():
            prefetch = self._ready_rolling_prefetch_entry()
            # Alternate deadline work with already stopped robots.  Prefetch
            # must not permanently starve the recovery queue when the fleet is
            # denser than one planner worker can serve in a single horizon.
            if prefetch is not None and (self._last_async_job_kind != "prefetch" or not ready):
                self._start_async_rolling_prefetch(prefetch)
                return dispatched
        planning_budget = self._dispatch_plan_budget()
        batch_size = self._dispatch_batch_size()
        planning_calls = 0
        while ready and planning_calls < planning_budget:
            first = ready.pop(0)
            motion_key = self._order_motion_key(first[0])
            group = [first]
            failure_count = max(0, int(first[0].dispatch_failures or 0))
            group_limit = (
                1
                if failure_count >= 2
                else min(2, batch_size)
                if failure_count == 1
                else batch_size
            )
            # Initial assignment benefits from a small coupled group because
            # all robots are still parked.  A rolling continuation is a much
            # more frequent operation (N / horizon requests per second) and
            # already sees the other committed trajectories as reservations.
            # Keep those jobs deliberately small so 50 robots do not enter an
            # expensive CBS wave at the same horizon boundary.
            if first[1].has_executed_route:
                group_limit = min(group_limit, self._dispatch_rolling_batch_size())
            remaining: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]] = []
            for entry in ready:
                if (
                    len(group) < group_limit
                    and self._order_motion_key(entry[0]) == motion_key
                ):
                    group.append(entry)
                else:
                    remaining.append(entry)
            ready = remaining
            if async_simulated:
                if self._async_simulated_dispatch_active():
                    break
                self._start_async_simulated_dispatch(group)
                group_dispatched = 0
            else:
                group_dispatched, _ = self._dispatch_simulated_order_batch(group)
                dispatched += group_dispatched
            group_handled = {order.order_id for order, _, _, _ in group}
            handled.update(group_handled)
            planning_calls += 1
            if async_simulated:
                break

        # Keep support for remote, automatic and already-at-goal orders.  A
        # failed coupled group is deliberately not retried individually in the
        # same tick: doing so recreated both CPU starvation and wait-only plans.
        remaining_budget = max(0, planning_budget - planning_calls)
        for order in queued_orders:
            if order.order_id in handled:
                continue
            robot = self.robots.get(order.vehicle) if order.vehicle else None
            if async_simulated and robot is not None and not robot.is_remote():
                # Simulated MAPF is deliberately background-only on runtime
                # ticks.  Otherwise one leftover order here would put the
                # expensive planner back on the websocket/status path.
                continue
            if (
                robot is not None
                and not robot.is_remote()
                and self._robot_can_accept_order(robot, explicit=True)
                and self._safe_replan_start_lm(robot) != order.target_lm
            ):
                if remaining_budget <= 0:
                    continue
                remaining_budget -= 1
            if self._dispatch_order(order, force=force):
                dispatched += 1
        return dispatched

    def _ready_simulated_order_entries(
        self,
        orders: list[FleetOrder],
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]]:
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]] = []
        used_robots: set[str] = set()
        for order in orders:
            if not order.vehicle or order.vehicle in used_robots:
                continue
            if not self._order_is_robot_queue_head(order):
                continue
            robot = self.robots.get(order.vehicle)
            if robot is None or robot.is_remote():
                continue
            if not self._robot_can_accept_order(robot, explicit=True):
                continue
            final_goal = self._active_order_target(order)
            start_lm = self._safe_replan_start_lm(robot)
            if not start_lm or start_lm not in self.landmarks or start_lm == final_goal:
                continue
            planning_goal = self._rolling_planning_goal(start_lm, final_goal, order)
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
            }
            if robot.pose is not None:
                request["startPose"] = dict(robot.pose)
            self._attach_spatial_route_to_request(
                request,
                order,
                start_lm,
                planning_goal,
                final_goal,
            )
            entries.append((order, robot, request, final_goal))
            used_robots.add(robot.name)
        return entries

    def _order_is_robot_queue_head(self, order: FleetOrder) -> bool:
        """Keep generated lifelong orders FIFO for each robot.

        A later benchmark order is generated from the previous order's final
        LM. Dispatching it first (for example because it has higher priority)
        can make that otherwise distant goal only one edge from the robot's
        current pose. Priority still orders traffic between different robots.
        """
        if not order.order_id.startswith("dynamic-") or not order.vehicle:
            return True
        pending = [
            candidate
            for candidate in self.orders.values()
            if candidate.order_id.startswith("dynamic-")
            and candidate.vehicle == order.vehicle
            and candidate.status not in TERMINAL_ORDER_STATUSES
        ]
        if not pending:
            return True
        pending.sort(key=lambda candidate: (candidate.created_at, candidate.order_id))
        return pending[0] is order

    def _dispatch_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[int, set[str]]:
        if not entries:
            return 0, set()
        requests, payload = self._prepare_simulated_order_batch(entries)
        result = self._plan_valid_requests(requests, payload)
        return self._finish_simulated_order_batch(entries, result)

    def _prepare_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        requests = [dict(request) for _, _, request, _ in entries]
        for order, robot, request, _ in entries:
            self._set_order_status(
                order,
                "PLANNING",
                robot=robot,
                start_lm=str(request["startLm"]),
            )
        first_order = entries[0][0]
        payload = self._order_plan_payload(first_order, requests[0]) | {"robots": requests}
        return requests, payload

    def _finish_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
        result: dict[str, Any],
    ) -> tuple[int, set[str]]:
        handled = {order.order_id for order, _, _, _ in entries}
        final_goals = {
            robot.name: final_goal
            for _, robot, _, final_goal in entries
        }
        if not result.get("ok") or not result.get("plans"):
            reason = self._planner_failure_reason(result)
            for order, _, _, _ in entries:
                self._set_order_error(order, reason)
            return 0, handled

        result = self._rolling_result(result, final_goals)
        plans_by_robot = {
            str(plan.get("robot")): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        accepted: list[dict[str, Any]] = []
        accepted_entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, dict[str, Any]]] = []
        for order, robot, request, final_goal in entries:
            plan = plans_by_robot.get(robot.name)
            if plan is None:
                self._set_order_error(order, "planner did not return robot plan")
                continue
            if self._wait_only_rolling_plan(plan, final_goal):
                detour_queued = (
                    self._order_stall_allows_detour(order)
                    and self._queue_alternate_corridor_detour(
                        order,
                        str(request.get("startLm") or ""),
                        final_goal,
                    )
                )
                self._set_order_error(
                    order,
                    "traffic window has no progress; alternate corridor queued"
                    if detour_queued
                    else "traffic window has no progress; joint retry pending",
                )
                self._event(
                    "warn",
                    f"{robot.name} wait-only route rejected; order kept queued",
                )
                continue
            accepted.append(plan)
            accepted_entries.append((order, robot, request, final_goal, plan))

        if not accepted:
            return 0, handled
        accepted_result = {**result, "plans": accepted}
        now = time()
        self._apply_planner_result(accepted_result, now)
        for order, robot, request, _, plan in accepted_entries:
            order.route_nodes = [str(item) for item in plan.get("nodes", [])]
            robot.active_order_id = order.order_id
            self._apply_simulated_route_metadata(robot, order, plan, now)
            self._set_order_status(
                order,
                "EXECUTING",
                robot=robot,
                start_lm=str(request["startLm"]),
            )
            order.traffic_detour_edges = []
            self._event(
                "info",
                f"order dispatched: {order.order_id} {robot.name} "
                f"{request['startLm']}->{order.target_lm}",
            )
        return len(accepted_entries), handled

    def _async_simulated_dispatch_active(self) -> bool:
        with self._dispatch_job_lock:
            return self._dispatch_job is not None

    def _start_async_simulated_dispatch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> None:
        if not entries:
            return
        requests, payload = self._prepare_simulated_order_batch(entries)
        job: dict[str, Any] = {
            "kind": "dispatch",
            "entries": list(entries),
            "requests": requests,
            "payload": payload,
            "done": False,
            "result": None,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                for order, _, _, _ in entries:
                    if order.status == "PLANNING":
                        order.status = "QUEUED"
                return
            self._dispatch_job = job

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background planner failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-dispatch",
            daemon=True,
        ).start()

    def _ready_rolling_prefetch_entry(
        self,
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float] | None:
        lead = self._rolling_prefetch_lead()
        candidates: list[tuple[float, str, FleetOrder, FleetRobot, dict[str, Any], str]] = []
        for robot in self._runtime_robots():
            if (
                robot.is_remote()
                or robot.pending_route is not None
                or robot.status not in {"MOVING", "WAITING"}
                or not robot.active_order_id
                or not robot.route_chunk_goal_lm
                or not robot.trajectory
            ):
                continue
            order = self.orders.get(robot.active_order_id)
            if order is None or order.status in TERMINAL_ORDER_STATUSES:
                continue
            final_goal = self._active_order_target(order)
            if not final_goal or robot.route_chunk_goal_lm == final_goal:
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            remaining = max(0.0, final_time - robot.route_clock)
            if remaining > lead:
                continue
            start_lm = robot.route_chunk_goal_lm
            planning_goal = self._rolling_planning_goal(start_lm, final_goal, order)
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
                "startPose": self._pose_at_landmark(start_lm),
            }
            self._attach_spatial_route_to_request(
                request,
                order,
                start_lm,
                planning_goal,
                final_goal,
            )
            candidates.append((remaining, robot.name, order, robot, request, final_goal))
        if not candidates:
            return None
        remaining, _, order, robot, request, final_goal = min(candidates)
        return order, robot, request, final_goal, remaining

    def _start_async_rolling_prefetch(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str, float],
    ) -> None:
        order, robot, request, final_goal, offset = entry
        payload = self._order_plan_payload(order, request) | {
            "robots": [request],
            "reservationOffsetSec": offset,
        }
        job: dict[str, Any] = {
            "kind": "prefetch",
            "entry": entry,
            "route_revision": robot.route_revision,
            "result": None,
            "done": False,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return
            self._dispatch_job = job

        def run() -> None:
            try:
                result = self._plan_valid_requests([request], payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background prefetch failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-prefetch",
            daemon=True,
        ).start()

    def _finish_async_simulated_dispatch(self) -> int:
        with self._dispatch_job_lock:
            job = self._dispatch_job
            if job is None or not bool(job.get("done")):
                return 0
            self._dispatch_job = None
        self._last_async_job_kind = str(job.get("kind") or "dispatch")

        if job.get("kind") == "prefetch":
            return self._finish_async_rolling_prefetch(job)
        if job.get("kind") == "coupled_replan":
            return self._finish_async_coupled_replan(job)

        entries = [
            entry
            for entry in job.get("entries", [])
            if self._async_dispatch_entry_is_current(entry)
        ]
        if not entries:
            return 0
        result = job.get("result")
        if not isinstance(result, dict):
            result = {
                "ok": False,
                "plans": [],
                "debug": {"reason": "background planner returned no result"},
            }
        dispatched, _ = self._finish_simulated_order_batch(entries, result)
        return dispatched

    def _finish_async_rolling_prefetch(self, job: dict[str, Any]) -> int:
        entry = job.get("entry")
        if not isinstance(entry, tuple) or len(entry) != 5:
            return 0
        order, robot, request, final_goal, _ = entry
        if (
            self.orders.get(order.order_id) is not order
            or self.robots.get(robot.name) is not robot
            or robot.active_order_id != order.order_id
            or robot.route_revision != int(job.get("route_revision", -1))
            or robot.route_chunk_goal_lm != str(request.get("startLm") or "")
            or not robot.trajectory
        ):
            return 0
        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            return 0
        result = self._rolling_result(result, {robot.name: final_goal})
        plan = self._plan_for_robot(result, robot.name)
        if plan is None or self._wait_only_rolling_plan(plan, final_goal):
            return 0
        robot.pending_route = {
            "order_id": order.order_id,
            "start_lm": str(request.get("startLm") or ""),
            "final_goal": final_goal,
            "result": result,
        }
        return 0

    def _start_async_coupled_replan(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> bool:
        cycle_key = tuple(sorted(robot.name for robot in robots))
        if len(cycle_key) < 2:
            return False
        if len(cycle_key) > self.planner.local_cbs_max_robots:
            if cycle_key not in self._coupled_replan_failures:
                self._coupled_replan_failures[cycle_key] = 1
                self.traffic_metrics["coupledReplansFailed"] += 1
                self._event(
                    "warn",
                    f"wait cycle has {len(cycle_key)} robots; local CBS cap is "
                    f"{self.planner.local_cbs_max_robots}",
                )
            return False
        last_attempt = self._coupled_replan_last_attempt.get(cycle_key, 0.0)
        if now - last_attempt < self._deadlock_coupled_replan_interval():
            return False

        requests: list[dict[str, Any]] = []
        entries: list[dict[str, Any]] = []
        ordered = [winner] + sorted(
            (robot for robot in robots if robot.name != winner.name),
            key=lambda robot: robot.name,
        )
        for robot in ordered:
            order = self._active_order_for_robot(robot)
            start_lm = self._safe_replan_start_lm(robot)
            final_goal = (
                self._active_order_target(order)
                if order is not None
                else robot.route_final_lm or robot.target_lm
            )
            if (
                order is None
                or not start_lm
                or start_lm not in self.landmarks
                or final_goal not in self.landmarks
            ):
                return False
            planning_goal = self._rolling_planning_goal(start_lm, final_goal, order)
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
                "startPose": (
                    dict(robot.pose)
                    if robot.pose is not None
                    else self._pose_at_landmark(start_lm)
                ),
            }
            requests.append(request)
            entries.append(
                {
                    "robot": robot.name,
                    "order": order.order_id,
                    "start": start_lm,
                    "finalGoal": final_goal,
                    "routeRevision": robot.route_revision,
                }
            )

        first_order = self.orders[str(entries[0]["order"])]
        payload = self._order_plan_payload(first_order, requests[0]) | {
            "robots": requests,
            "plannerBackend": "cbs",
            "allowCbsFallback": True,
            "skipSoftBlockedDetour": True,
            "reservedEdgeDetourEnabled": False,
        }
        job: dict[str, Any] = {
            "kind": "coupled_replan",
            "cycle": cycle_key,
            "entries": entries,
            "requests": requests,
            "result": None,
            "done": False,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return False
            self._dispatch_job = job
        self._coupled_replan_last_attempt[cycle_key] = now
        self.traffic_metrics["coupledReplansStarted"] += 1

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background coupled CBS failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-coupled-cbs",
            daemon=True,
        ).start()
        self._event(
            "warn",
            f"local CBS started for wait cycle: {', '.join(cycle_key)}",
        )
        return True

    def _finish_async_coupled_replan(self, job: dict[str, Any]) -> int:
        cycle_key = tuple(str(name) for name in job.get("cycle", ()))
        entries = [entry for entry in job.get("entries", []) if isinstance(entry, dict)]
        current: list[tuple[FleetRobot, FleetOrder, dict[str, Any]]] = []
        for entry in entries:
            robot = self.robots.get(str(entry.get("robot") or ""))
            order = self.orders.get(str(entry.get("order") or ""))
            if (
                robot is None
                or order is None
                or robot.active_order_id != order.order_id
                or order.status in TERMINAL_ORDER_STATUSES
                or robot.route_revision != int(entry.get("routeRevision", -1))
                or self._safe_replan_start_lm(robot) != str(entry.get("start") or "")
            ):
                return 0
            current.append((robot, order, entry))

        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            self._record_coupled_replan_failure(cycle_key, result)
            return 0
        final_goals = {
            robot.name: str(entry.get("finalGoal") or "")
            for robot, _, entry in current
        }
        result = self._rolling_result(result, final_goals)
        plans = {
            str(plan.get("robot") or ""): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        if any(
            robot.name not in plans
            or self._wait_only_rolling_plan(plans[robot.name], final_goals[robot.name])
            for robot, _, _ in current
        ):
            self._record_coupled_replan_failure(cycle_key, result)
            return 0

        now = time()
        self._apply_planner_result(result, now)
        for robot, order, entry in current:
            plan = plans[robot.name]
            self._adopt_coupled_spatial_detour(
                order,
                plan,
                str(entry.get("finalGoal") or ""),
            )
            order.route_nodes = [str(node) for node in plan.get("nodes", [])]
            order.status = "EXECUTING"
            order.error = ""
            order.updated_at = now
            self._apply_simulated_route_metadata(robot, order, plan, now)
            robot.last_reason = "local coupled CBS committed"
            robot.last_replan_at = now
            robot.traffic_stall_since = None
            self._clear_wait_dependency(robot)
        self._coupled_replan_failures.pop(cycle_key, None)
        self._active_wait_cycles.pop(cycle_key, None)
        self.traffic_metrics["coupledReplansSucceeded"] += 1
        self.traffic_metrics["cycleReplans"] += 1
        self._event(
            "warn",
            f"local CBS committed for wait cycle: {', '.join(cycle_key)}",
        )
        return len(current)

    def _record_coupled_replan_failure(
        self,
        cycle_key: tuple[str, ...],
        result: Any,
    ) -> None:
        self._coupled_replan_failures[cycle_key] = (
            self._coupled_replan_failures.get(cycle_key, 0) + 1
        )
        self.traffic_metrics["coupledReplansFailed"] += 1
        reason = self._planner_failure_reason(result) if isinstance(result, dict) else "no result"
        self._event(
            "warn",
            f"local CBS pending for wait cycle {', '.join(cycle_key)}: {reason}",
        )

    def _async_dispatch_entry_is_current(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str],
    ) -> bool:
        order, robot, request, _ = entry
        if self.orders.get(order.order_id) is not order or order.status != "PLANNING":
            return False
        if self.robots.get(robot.name) is not robot or robot.active_order_id:
            order.status = "QUEUED"
            return False
        start_lm = self._safe_replan_start_lm(robot)
        if start_lm != str(request.get("startLm") or ""):
            order.status = "QUEUED"
            order.error = "robot moved while background plan was running"
            order.updated_at = time()
            return False
        return True

    def _order_motion_key(self, order: FleetOrder) -> tuple[float, float, bool, float, bool]:
        return (
            round(float(order.speed), 6),
            round(float(order.acceleration), 6),
            bool(order.rotate),
            round(float(order.turn_speed), 6),
            bool(order.stretch_motion_to_reservation_ticks),
        )

    def _dispatch_plan_budget(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 2
        try:
            return max(1, min(8, int(fleet.get("dispatch_plan_budget_per_tick", 2) or 2)))
        except (TypeError, ValueError):
            return 2

    def _dispatch_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 2
        try:
            return max(2, min(8, int(fleet.get("dispatch_joint_batch_size", 2) or 2)))
        except (TypeError, ValueError):
            return 2

    def _dispatch_rolling_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1
        try:
            return max(1, min(4, int(fleet.get("dispatch_rolling_batch_size", 1) or 1)))
        except (TypeError, ValueError):
            return 1

    def _rolling_prefetch_lead(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 3.0
        try:
            configured = float(fleet.get("rolling_prefetch_lead_sec", 3.0) or 3.0)
        except (TypeError, ValueError):
            configured = 3.0
        horizon = self._rolling_horizon()
        upper = max(0.5, horizon * 0.6) if horizon > 0.0 else 5.0
        return max(0.5, min(upper, configured))

    def _order_dispatch_retry_interval(self, order: FleetOrder | None = None) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            base = max(0.1, float(fleet.get("order_dispatch_retry_sec", 0.5) or 0.5))
        except (TypeError, ValueError):
            base = 0.5
        try:
            maximum = max(base, float(fleet.get("order_dispatch_retry_max_sec", 4.0) or 4.0))
        except (TypeError, ValueError):
            maximum = 4.0
        failures = max(0, int(order.dispatch_failures if order is not None else 0))
        return min(maximum, base * (2 ** min(4, max(0, failures - 1))))

    def _dispatch_order(self, order: FleetOrder, force: bool = False) -> bool:
        if not self._order_is_robot_queue_head(order):
            return False
        order.target_lm = self._active_order_target(order)
        candidates = self._candidate_robots_for_order(order)
        if not candidates:
            self._set_order_error(order, "no available robot")
            return False

        failed_reason = ""
        for robot in candidates:
            start_lm = self._safe_replan_start_lm(robot)
            if not start_lm or start_lm not in self.landmarks:
                failed_reason = "robot is between graph landmarks"
                continue
            if start_lm == order.target_lm and not robot.trajectory:
                robot.current_lm = order.target_lm
                robot.target_lm = ""
                robot.status = "ARRIVED"
                robot.active_order_id = ""
                robot.last_reason = "order already at target"
                robot.updated_at = time()
                if self._advance_or_complete_order(order, robot, time()):
                    self._event("info", f"order completed: {order.order_id} {robot.name}@{order.target_lm}")
                    return True
                return self._dispatch_order(order, force=force)

            planning_goal = (
                order.target_lm
                if robot.is_remote()
                else self._rolling_planning_goal(start_lm, order.target_lm, order)
            )
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
            }
            if robot.pose is not None:
                request["startPose"] = dict(robot.pose)
            if not robot.is_remote():
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    order.target_lm,
                )

            self._set_order_status(order, "PLANNING", robot=robot, start_lm=start_lm)
            result = self._plan_valid_requests([request], self._order_plan_payload(order, request))
            if result.get("ok") and result.get("plans"):
                now = time()
                plan = self._plan_for_robot(result, robot.name)
                remote_route: dict[str, Any] | None = None
                if robot.is_remote():
                    if plan is None:
                        failed_reason = "planner did not return robot plan"
                        continue
                    try:
                        remote_route = self._execute_remote_plan(robot, order, plan, result)
                    except Exception as exc:
                        failed_reason = f"remote execute failed: {exc}"
                        robot.remote_error = str(exc)
                        control_conflict = self._is_remote_control_conflict(exc)
                        robot.remote_online = control_conflict
                        robot.status = "MANUAL" if control_conflict else "OFFLINE"
                        robot.last_reason = failed_reason
                        robot.updated_at = now
                        continue
                if not robot.is_remote():
                    result = self._rolling_result(
                        result,
                        {robot.name: order.target_lm},
                    )
                    plan = self._plan_for_robot(result, robot.name)
                    if plan is None:
                        failed_reason = "planner did not return rolling route chunk"
                        continue
                    if self._wait_only_rolling_plan(plan, order.target_lm):
                        detour_queued = (
                            self._order_stall_allows_detour(order)
                            and self._queue_alternate_corridor_detour(
                                order,
                                start_lm,
                                order.target_lm,
                            )
                        )
                        failed_reason = (
                            "traffic window has no progress; alternate corridor queued"
                            if detour_queued
                            else "traffic window has no progress; joint retry pending"
                        )
                        self._event(
                            "warn",
                            f"{robot.name} wait-only route rejected; order kept queued",
                        )
                        continue
                order.route_nodes = [
                    str(item)
                    for plan in result.get("plans", [])
                    if isinstance(plan, dict)
                    for item in plan.get("nodes", [])
                ]
                self._apply_planner_result(result, now, order_id=order.order_id)
                if remote_route is not None:
                    self._apply_remote_route_metadata(robot, remote_route, now)
                elif plan is not None:
                    self._apply_simulated_route_metadata(robot, order, plan, now)
                self._set_order_status(order, "EXECUTING", robot=robot, start_lm=start_lm)
                order.traffic_detour_edges = []
                self._event(
                    "info",
                    f"order dispatched: {order.order_id} {robot.name} {start_lm}->{order.target_lm}",
                )
                return True
            failed_reason = self._planner_failure_reason(result)
            if self._planner_deadlock_result(result):
                robot.status = "WAITING"
                robot.last_reason = failed_reason
                robot.blocked_since = time()
                robot.updated_at = time()
            if order.vehicle:
                order.status = "QUEUED"
                order.assigned_robot = robot.name
                order.start_lm = start_lm
                order.updated_at = time()
            else:
                order.status = "QUEUED"
                order.start_lm = start_lm
                order.updated_at = time()

        self._set_order_error(order, failed_reason or "dispatch pending")
        return False

    def _order_plan_payload(self, order: FleetOrder, request: dict[str, Any]) -> dict[str, Any]:
        robot = self.robots.get(str(request.get("name") or ""))
        rolling_continuation = bool(robot is not None and robot.has_executed_route)
        traffic_detour = bool(order.traffic_detour_edges)
        payload: dict[str, Any] = {
            "robots": [request],
            # A 10 second rolling waypoint never needs the global 160 second
            # low-level search.  Bounding the background search prevents one
            # congested request from monopolising Python's GIL and freezing
            # the simulation clock.
            "lowLevelMaxTime": self._runtime_low_level_max_time(),
            # CBS coordinates a newly released group.  For a continuation the
            # other robots are fixed time reservations; CBS cannot move them,
            # so falling back from SIPP only burns several seconds.  Traffic
            # retries/deadlock priority leases handle that case on a fresh tick.
            "allowCbsFallback": not rolling_continuation,
            # Time reservations already model parked and moving peers. Static
            # detour probes duplicate the same work and can run the planner
            # twice, which is too expensive at N / horizon rolling requests.
            # The spatial route is committed once. Other robots are temporal
            # reservations, not static obstacles that silently rewrite that
            # route at every rolling chunk.
            "skipSoftBlockedDetour": not traffic_detour,
            "reservedEdgeDetourEnabled": traffic_detour,
        }
        if traffic_detour:
            payload["blocked_edges"] = [
                {"from": src, "to": dst}
                for src, dst in order.traffic_detour_edges
            ]
        if order.speed > 0.0:
            payload["speed"] = order.speed
        if order.acceleration > 0.0:
            payload["acceleration"] = order.acceleration
        payload["rotate"] = bool(order.rotate)
        payload["stretchMotionToReservationTicks"] = bool(order.stretch_motion_to_reservation_ticks)
        if order.turn_speed > 0.0:
            payload["turnSpeed"] = order.turn_speed
        return payload

    def _runtime_low_level_max_time(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured_max = max(8, int(fleet.get("cbs_low_level_max_time", 160) or 160))
        except (TypeError, ValueError):
            configured_max = 160
        # The request may spend one reservation horizon waiting and then one
        # rolling horizon moving.  A small guard absorbs rounding and safety
        # margins while keeping the state space proportional to the window.
        window_sec = self._rolling_horizon() + self._reservation_horizon()
        guard_sec = max(2.0, self._reservation_safety_time() * 4.0)
        ticks = math.ceil((window_sec + guard_sec) / self._reservation_time_step())
        return max(8, min(configured_max, int(ticks)))

    def _candidate_robots_for_order(self, order: FleetOrder) -> list[FleetRobot]:
        if order.vehicle:
            robot = self.robots.get(order.vehicle)
            if robot is None:
                return []
            return [robot] if self._robot_can_accept_order(robot, explicit=True) else []

        candidates = [
            robot for robot in self._runtime_robots()
            if self._robot_can_accept_order(robot, explicit=False)
        ]
        candidates.sort(
            key=lambda robot: (
                self._lm_distance(self._nearest_lm_for_robot(robot), order.target_lm),
                robot.name,
            )
        )
        return candidates

    def _robot_can_accept_order(self, robot: FleetRobot, explicit: bool = False) -> bool:
        if not self._robot_enabled(robot):
            return False
        if robot.is_remote():
            self._sync_remote_robot(robot, time(), force=False)
            if not robot.remote_online:
                return False
            owner_id, _ = self._remote_control_owner(robot)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return False
            if robot.status in {"LOCALIZING", "OFFLINE", "ERROR"}:
                return False
        if robot.active_order_id:
            return False
        if robot.target_lm or robot.trajectory:
            return False
        if robot.status == "STOPPED" and not explicit:
            return False
        if robot.status == "MANUAL" and not explicit:
            return False
        if robot.status in {"MOVING", "WAITING", "PLANNING", "BLOCKED"}:
            return False
        return True

    def _lm_distance(self, start_lm: str, goal_lm: str) -> float:
        start = self.landmarks.get(start_lm)
        goal = self.landmarks.get(goal_lm)
        if start is None or goal is None:
            return float("inf")
        return math.hypot(goal.x - start.x, goal.y - start.y)

    def _set_order_status(
        self,
        order: FleetOrder,
        status: str,
        robot: FleetRobot | None = None,
        start_lm: str = "",
        error: str = "",
    ) -> None:
        order.status = status
        order.updated_at = time()
        order.error = error
        if status in {"EXECUTING", "COMPLETED"}:
            order.dispatch_failures = 0
            order.traffic_blocked_since = None
        if robot is not None:
            order.assigned_robot = robot.name
            if not order.vehicle and status not in {"PLANNING", "QUEUED"}:
                order.vehicle = robot.name
        if start_lm:
            order.start_lm = start_lm

    def _set_order_error(self, order: FleetOrder, error: str) -> None:
        if order.error != error:
            self._event("warn", f"order pending: {order.order_id} {error}")
        order.status = "QUEUED"
        order.error = error
        order.dispatch_failures += 1
        error_value = str(error or "").lower()
        traffic_failure = any(
            marker in error_value
            for marker in (
                "no_sipp_path",
                "reserved_",
                "traffic window",
                "deadlock",
                "continuous reservation conflict",
                "blocked edge",
            )
        )
        if traffic_failure:
            now = time()
            if order.traffic_blocked_since is None:
                order.traffic_blocked_since = now
            elif now - order.traffic_blocked_since >= self._traffic_replan_after():
                self._queue_alternate_corridor_detour(
                    order,
                    order.start_lm,
                    self._active_order_target(order),
                )
                order.spatial_route_nodes = []
        if not order.vehicle:
            order.assigned_robot = ""
        order.updated_at = time()

    def _queue_alternate_corridor_detour(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
    ) -> bool:
        """Exclude the next corridor only when the same goal stays reachable."""
        if start_lm not in self.landmarks or final_goal_lm not in self.landmarks:
            return False
        # A detour is a one-chunk traffic decision, not permanent map editing.
        # Retry an existing exclusion after backoff instead of accumulating
        # enough exclusions to erode the graph of a long-running order.
        if order.traffic_detour_edges:
            return False
        try:
            route = self.planner.route_planner.find_route(
                start_lm,
                final_goal_lm,
            )
        except ValueError:
            return False
        if len(route.nodes) < 2:
            return False

        src, dst = str(route.nodes[0]), str(route.nodes[1])
        candidate = {(src, dst), (dst, src)}
        try:
            alternate = self.planner.route_planner.find_route(
                start_lm,
                final_goal_lm,
                blocked_edges=candidate,
            )
        except ValueError:
            # A single-exit landmark must wait; banning its only corridor
            # would turn temporary congestion into a permanent no-path error.
            return False
        if alternate.nodes == route.nodes:
            return False

        order.traffic_detour_edges = sorted(candidate)
        order.traffic_detour_attempts += 1
        order.spatial_route_nodes = []
        self._event(
            "warn",
            f"{order.vehicle or order.assigned_robot} alternate corridor queued: "
            f"avoid {src}<->{dst}, keep goal {final_goal_lm}",
        )
        return True

    def _order_stall_allows_detour(self, order: FleetOrder) -> bool:
        now = time()
        if order.traffic_blocked_since is None:
            order.traffic_blocked_since = now
            return False
        return now - order.traffic_blocked_since >= self._traffic_replan_after()

    def _cancel_order(self, order: FleetOrder, reason: str) -> None:
        for robot in self._runtime_robots():
            if robot.active_order_id == order.order_id:
                self._stop_robot(robot, cancel_active_order=False)
        self._set_order_status(order, "CANCELED", error=reason)
        self._event("warn", f"order canceled: {order.order_id}")

    def _pause_order(self, order: FleetOrder, reason: str) -> None:
        paused_robot: FleetRobot | None = None
        for robot in self._runtime_robots():
            if robot.active_order_id == order.order_id:
                paused_robot = robot
                break
        if paused_robot is None and order.assigned_robot:
            paused_robot = self.robots.get(order.assigned_robot)
        if paused_robot is None and order.vehicle:
            paused_robot = self.robots.get(order.vehicle)

        if paused_robot is not None:
            self._cancel_remote_route(paused_robot, reason)
            nearest_lm = self._nearest_lm_for_robot(paused_robot)
            if nearest_lm in self.landmarks:
                paused_robot.current_lm = nearest_lm
            paused_robot.target_lm = ""
            paused_robot.status = "PAUSED"
            paused_robot.trajectory = []
            paused_robot.plan_nodes = []
            paused_robot.trajectory_dirty = True
            paused_robot.route_started_at = None
            paused_robot.route_clock = 0.0
            paused_robot.last_tick_at = None
            paused_robot.blocked_since = None
            paused_robot.last_replan_at = None
            paused_robot.last_reason = reason
            paused_robot.route_note = ""
            paused_robot.active_order_id = ""
            self._clear_remote_route_metadata(paused_robot)
            paused_robot.updated_at = time()
            order.assigned_robot = paused_robot.name
            if not order.vehicle:
                order.vehicle = paused_robot.name
            order.start_lm = paused_robot.current_lm

        order.status = "PAUSED"
        order.error = reason
        order.updated_at = time()
        self._event("warn", f"order paused: {order.order_id}")

    def _cancel_active_order_for_robot(self, robot: FleetRobot, reason: str) -> None:
        if not robot.active_order_id:
            return
        self._cancel_remote_route(robot, reason)
        order = self.orders.get(robot.active_order_id)
        if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
            self._set_order_status(order, "CANCELED", error=reason)
            self._event("warn", f"order canceled: {order.order_id} {reason}")
        robot.active_order_id = ""

    def _cancel_orders_for_robot(self, robot_name: str, reason: str) -> None:
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.vehicle == robot_name or order.assigned_robot == robot_name:
                self._set_order_status(order, "CANCELED", error=reason)
                self._event("warn", f"order canceled: {order.order_id} {reason}")

    def _replace_orders_for_robot(self, robot_name: str, reason: str) -> None:
        robot = self.robots.get(robot_name)
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.vehicle == robot_name or order.assigned_robot == robot_name:
                self._set_order_status(order, "CANCELED", error=reason)
                self._event("warn", f"order canceled: {order.order_id} {reason}")

        if robot is None:
            return
        self._cancel_remote_route(robot, reason)
        nearest_lm = self._nearest_lm_for_robot(robot)
        if nearest_lm in self.landmarks:
            robot.current_lm = nearest_lm
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
        robot.last_reason = reason
        robot.route_note = ""
        robot.active_order_id = ""
        self._clear_remote_route_metadata(robot)
        robot.updated_at = time()

    def _cancel_all_orders(self, reason: str) -> None:
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            self._set_order_status(order, "CANCELED", error=reason)
            self._event("warn", f"order canceled: {order.order_id} {reason}")

    def _active_order_target(self, order: FleetOrder) -> str:
        targets = order.targets or ([order.target_lm] if order.target_lm else [])
        if not targets:
            return order.target_lm
        order.step_index = max(0, min(order.step_index, len(targets) - 1))
        order.target_lm = targets[order.step_index]
        return order.target_lm

    def _advance_or_complete_order(self, order: FleetOrder, robot: FleetRobot, now: float) -> bool:
        targets = order.targets or ([order.target_lm] if order.target_lm else [])
        if order.step_index + 1 >= len(targets):
            order.status = "COMPLETED"
            order.error = ""
            order.updated_at = now
            order.assigned_robot = robot.name
            order.route_nodes = list(robot.plan_nodes)
            robot.active_order_id = ""
            return True

        previous_target = targets[order.step_index]
        order.step_index += 1
        order.target_lm = targets[order.step_index]
        order.status = "QUEUED"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = robot.current_lm
        order.route_nodes = []
        order.spatial_route_nodes = []
        order.spatial_route_revision = 0
        order.traffic_blocked_since = None
        robot.active_order_id = ""
        self._event(
            "info",
            f"order step completed: {order.order_id} {robot.name}@{previous_target}; next {order.target_lm}",
        )
        return False

    def _complete_active_order(self, robot: FleetRobot, now: float) -> None:
        if not robot.active_order_id:
            return
        order = self.orders.get(robot.active_order_id)
        if order is None:
            robot.active_order_id = ""
            return
        order.route_nodes = list(robot.plan_nodes)
        self._advance_or_complete_order(order, robot, now)

    def _complete_simulated_route_chunk(self, robot: FleetRobot, now: float) -> bool:
        """Finish a committed rolling-horizon chunk without completing its order."""
        if not robot.active_order_id or not robot.route_chunk_goal_lm:
            return False
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return False
        final_target = self._active_order_target(order)
        if robot.current_lm != robot.route_chunk_goal_lm or robot.current_lm == final_target:
            return False

        order.status = "QUEUED"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = robot.current_lm
        order.route_nodes = list(robot.plan_nodes)
        robot.active_order_id = ""
        self._event(
            "info",
            f"route horizon reached: {order.order_id} {robot.name}@{robot.current_lm}; "
            f"rolling replan to {final_target}",
        )
        return True

    def _activate_rolling_prefetch(self, robot: FleetRobot, now: float) -> bool:
        pending = robot.pending_route
        if not isinstance(pending, dict) or not robot.active_order_id:
            return False
        order = self.orders.get(robot.active_order_id)
        if (
            order is None
            or order.status in TERMINAL_ORDER_STATUSES
            or str(pending.get("order_id") or "") != order.order_id
            or str(pending.get("start_lm") or "") != robot.current_lm
        ):
            robot.pending_route = None
            return False
        result = pending.get("result")
        if not isinstance(result, dict):
            robot.pending_route = None
            return False
        plan = self._plan_for_robot(result, robot.name)
        if plan is None:
            robot.pending_route = None
            return False

        # Switch at the exact graph LM.  The new trajectory starts at t=0 on
        # the same pose, so the browser sees one continuous route clock rather
        # than an IDLE frame between rolling chunks.
        robot.pending_route = None
        self._apply_planner_result(result, now, order_id=order.order_id)
        order.route_nodes = [str(item) for item in plan.get("nodes", [])]
        self._apply_simulated_route_metadata(robot, order, plan, now)
        self._set_order_status(
            order,
            "EXECUTING",
            robot=robot,
            start_lm=robot.current_lm,
        )
        robot.last_reason = "rolling route continued"
        self._event(
            "info",
            f"route prefetched: {order.order_id} {robot.name}@{robot.current_lm}",
        )
        return True

    def _update_active_order_from_robot(self, robot: FleetRobot) -> None:
        if not robot.active_order_id:
            return
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return
        if robot.status == "WAITING":
            if self._is_robot_conflict(robot.last_reason) or str(robot.last_reason).startswith(
                "planned traffic wait"
            ):
                status = "WAITING_TRAFFIC"
            else:
                status = "WAITING_OBSTACLE"
        elif robot.status == "MOVING":
            status = "EXECUTING"
        elif robot.status == "BLOCKED":
            status = "PAUSED"
        elif robot.status == "PLANNING":
            status = "PLANNING"
        elif robot.status == "OFFLINE":
            status = "QUEUED"
        else:
            status = order.status
        order.status = status
        order.error = "" if status == "EXECUTING" else robot.last_reason
        order.updated_at = time()
        order.route_nodes = list(robot.plan_nodes)

    def _stop_robot(self, robot: FleetRobot, cancel_active_order: bool = True) -> None:
        self._stop_remote_robot(robot)
        if cancel_active_order and robot.active_order_id:
            order = self.orders.get(robot.active_order_id)
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                self._set_order_status(order, "CANCELED", error="robot stopped")
                self._event("warn", f"order canceled: {order.order_id} robot stopped")
        robot.status = "STOPPED"
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
        robot.active_order_id = ""
        self._clear_remote_route_metadata(robot)
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

    def _robot_mode_from_payload(self, payload: dict[str, Any]) -> str:
        raw = str(payload.get("mode") or payload.get("type") or payload.get("robotMode") or "simulated").strip().lower()
        if raw in {"remote", "robot", "real", "grpc", "aivison_grpc", "real_grpc"}:
            return "remote"
        return "simulated"

    def _robot_name_from_payload(self, payload: dict[str, Any]) -> str:
        return str(
            payload.get("name")
            or payload.get("robotName")
            or payload.get("robot_name")
            or payload.get("alias")
            or ""
        ).strip()

    def _remote_base_url_from_payload(self, payload: dict[str, Any]) -> str:
        value = str(
            payload.get("baseUrl")
            or payload.get("url")
            or payload.get("host")
            or payload.get("ip")
            or payload.get("address")
            or ""
        ).strip()
        if not value:
            return ""
        if getattr(self.remote_adapter, "transport", "") == "grpc":
            if value.startswith("grpc://") or value.startswith("grpcs://"):
                return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
            if "://" in value:
                raise ValueError("unsupported robot endpoint scheme; use grpc://host:port")
            port_raw = payload.get("port") or payload.get("grpcPort") or payload.get("grpc_port")
            if port_raw is None:
                return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
            try:
                port = int(port_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid robot gRPC port") from exc
            return normalize_grpc_endpoint(f"grpc://{value}:{port}", default_port=DEFAULT_GRPC_PORT)
        if "://" in value:
            raise ValueError("unsupported robot endpoint scheme; use grpc://host:port")
        port_raw = payload.get("port") or payload.get("grpcPort") or payload.get("grpc_port")
        if port_raw is None:
            return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
        try:
            port = int(port_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid robot gRPC port") from exc
        return normalize_grpc_endpoint(f"grpc://{value}:{port}", default_port=DEFAULT_GRPC_PORT)

    def _remote_robot_name(
        self,
        identity_payload: dict[str, Any] | None,
        status_payload: dict[str, Any] | None,
        base_url: str,
    ) -> str:
        candidates: list[Any] = []

        def collect(payload: dict[str, Any] | None) -> None:
            if not isinstance(payload, dict):
                return
            for key in ("robotId", "robot_id", "name", "id", "vehicleId", "vehicle_id", "uuid", "serial"):
                candidates.append(payload.get(key))
            for nested_key in ("identity", "robot", "basic_info", "basicInfo", "robot_report", "robotReport"):
                nested = payload.get(nested_key)
                if isinstance(nested, dict):
                    for key in ("robotId", "robot_id", "name", "id", "vehicleId", "vehicle_id", "uuid", "serial"):
                        candidates.append(nested.get(key))

        collect(identity_payload)
        collect(status_payload)
        for value in candidates:
            text = str(value or "").strip()
            if text and text.lower() not in {"none", "null", "unknown", "-"}:
                return text
        parsed = urlparse(base_url)
        return str(parsed.hostname or parsed.netloc or "").strip()

    def _remote_unique_robot_name(self, name: str, base_url: str) -> str:
        clean_name = str(name or "").strip() or self._remote_name_from_endpoint(base_url)
        for existing in self.robots.values():
            if existing.is_remote() and existing.base_url == base_url:
                return existing.name
        existing = self.robots.get(clean_name)
        if existing is None or (existing.is_remote() and existing.base_url == base_url):
            return clean_name

        suffix = self._remote_name_from_endpoint(base_url)
        candidate = f"{clean_name}-{suffix}" if suffix and suffix != clean_name else f"{clean_name}-remote"
        index = 2
        while candidate in self.robots:
            candidate = f"{clean_name}-{suffix or 'remote'}-{index}"
            index += 1
        return candidate

    def _remote_name_from_endpoint(self, base_url: str) -> str:
        parsed = urlparse(base_url)
        host = str(parsed.hostname or parsed.netloc or "").strip()
        if not host:
            return "remote"
        parts = [part for part in host.replace(":", ".").split(".") if part]
        if len(parts) >= 4 and all(part.isdigit() for part in parts[-4:]):
            return f"robot-{parts[-1]}"
        return host.replace(".", "-")

    def _remote_identity_id(self, identity_payload: dict[str, Any] | None) -> str:
        if not isinstance(identity_payload, dict):
            return ""
        identity = identity_payload.get("identity")
        if isinstance(identity, dict):
            value = identity.get("robotId") or identity.get("id")
        else:
            value = identity_payload.get("robotId") or identity_payload.get("id")
        return str(value or "").strip()

    def _remote_status_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        robot_payload = payload.get("robot")
        if isinstance(robot_payload, dict):
            return robot_payload
        return payload if isinstance(payload, dict) else {}

    def _remote_pose_from_status(self, status_payload: dict[str, Any]) -> dict[str, float] | None:
        pose = status_payload.get("pose")
        if isinstance(pose, dict):
            try:
                return {
                    "x": float(pose.get("x", 0.0) or 0.0),
                    "y": float(pose.get("y", 0.0) or 0.0),
                    "yaw": float(pose.get("yaw", 0.0) or pose.get("angle", 0.0) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        if "x" in status_payload and "y" in status_payload:
            try:
                return {
                    "x": float(status_payload.get("x", 0.0) or 0.0),
                    "y": float(status_payload.get("y", 0.0) or 0.0),
                    "yaw": float(status_payload.get("yaw", status_payload.get("angle", 0.0)) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        robot_report = status_payload.get("robot_report") or status_payload.get("robotReport")
        if isinstance(robot_report, dict) and "x" in robot_report and "y" in robot_report:
            try:
                return {
                    "x": float(robot_report.get("x", 0.0) or 0.0),
                    "y": float(robot_report.get("y", 0.0) or 0.0),
                    "yaw": float(robot_report.get("angle", robot_report.get("yaw", 0.0)) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        return None

    def _remote_timeout(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.8
        try:
            return max(0.2, float(fleet.get("remote_timeout_sec", 0.8) or 0.8))
        except (TypeError, ValueError):
            return 0.8

    def _clear_remote_route_metadata(self, robot: FleetRobot) -> None:
        robot.route_revision = 0
        robot.route_chunk_index = 0
        robot.route_chunk_goal_lm = ""
        robot.route_final_lm = ""
        robot.route_preview = []
        robot.route_preview_dirty = True
        robot.pending_route = None
        self._clear_deadlock_retreat(robot)

    def _apply_simulated_route_metadata(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
        now: float,
    ) -> None:
        previous_final = robot.route_final_lm
        previous_chunk = robot.route_chunk_goal_lm
        chunk_goal = str(plan.get("goalLm") or order.target_lm).strip()
        final_goal = str(plan.get("finalGoalLm") or order.target_lm).strip()
        if previous_final == final_goal and previous_chunk == robot.current_lm:
            chunk_index = robot.route_chunk_index + 1
        else:
            chunk_index = 0
        robot.route_revision = self._next_route_revision()
        robot.route_chunk_index = chunk_index
        robot.route_chunk_goal_lm = chunk_goal
        robot.route_final_lm = final_goal
        self._update_route_preview(
            robot,
            robot.current_lm,
            final_goal,
            blocked_edges=set(order.traffic_detour_edges),
            committed_trajectory=plan.get("trajectory"),
            committed_nodes=plan.get("nodes"),
            spatial_route_nodes=order.spatial_route_nodes,
        )
        robot.has_executed_route = True
        robot.pending_route = None
        robot.target_lm = chunk_goal
        robot.updated_at = now

    def _update_route_preview(
        self,
        robot: FleetRobot,
        start_lm: str,
        final_goal_lm: str,
        blocked_edges: set[tuple[str, str]] | None = None,
        committed_trajectory: Any = None,
        committed_nodes: Any = None,
        spatial_route_nodes: Any = None,
    ) -> None:
        if start_lm not in self.landmarks or final_goal_lm not in self.landmarks:
            robot.route_preview = []
            robot.route_preview_dirty = True
            return
        committed_samples = [
            item
            for item in (committed_trajectory if isinstance(committed_trajectory, list) else [])
            if isinstance(item, dict)
        ]
        preview: list[dict[str, Any]] = [
            {
                "x": float(sample.get("x", 0.0) or 0.0),
                "y": float(sample.get("y", 0.0) or 0.0),
                "yaw": float(sample.get("yaw", 0.0) or 0.0),
                "phase": "committed",
            }
            for sample in committed_samples
        ]
        nodes = [
            str(node)
            for node in (committed_nodes if isinstance(committed_nodes, list) else [])
            if str(node) in self.landmarks
        ]
        continuation_start = nodes[-1] if nodes else start_lm
        if not preview:
            continuation_start = start_lm
        stable_nodes = [
            str(node)
            for node in (
                spatial_route_nodes
                if isinstance(spatial_route_nodes, list)
                else []
            )
            if str(node) in self.landmarks
        ]
        stable_suffix: list[str] = []
        if continuation_start in stable_nodes:
            stable_suffix = stable_nodes[stable_nodes.index(continuation_start):]
        if stable_suffix and stable_suffix[-1] != final_goal_lm:
            stable_suffix = []
        if continuation_start != final_goal_lm:
            try:
                route = (
                    self._planned_route_from_nodes(stable_suffix)
                    if len(stable_suffix) >= 2
                    else self.planner.route_planner.find_route(
                        continuation_start,
                        final_goal_lm,
                        blocked_edges=blocked_edges,
                    )
                )
                continuation = self.planner.route_planner.sample_route(
                    route,
                    sample_distance=0.50,
                )
            except (RuntimeError, ValueError):
                continuation = []
            for sample in continuation:
                if not isinstance(sample, dict):
                    continue
                point = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                    "phase": "forecast",
                }
                if preview and math.hypot(
                    point["x"] - float(preview[-1].get("x", 0.0) or 0.0),
                    point["y"] - float(preview[-1].get("y", 0.0) or 0.0),
                ) < 0.001:
                    continue
                preview.append(point)
        robot.route_preview = preview
        robot.route_preview_dirty = True

    def _planned_route_from_nodes(self, nodes: list[str]) -> PlannedRoute:
        edges: list[GraphEdge] = []
        for src, dst in zip(nodes, nodes[1:]):
            edge = self.planner.route_planner.get_edge(src, dst)
            if edge is None:
                raise ValueError(f"spatial route contains non-edge {src}->{dst}")
            edges.append(edge)
        return PlannedRoute(
            nodes=list(nodes),
            edges=edges,
            length=sum(float(edge.length) for edge in edges),
        )

    def _ensure_order_spatial_route(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
    ) -> list[str]:
        existing = [
            str(node)
            for node in order.spatial_route_nodes
            if str(node) in self.landmarks
        ]
        if start_lm in existing and existing and existing[-1] == final_goal_lm:
            suffix = existing[existing.index(start_lm):]
            if all(
                dst in self.planner.graph.get(src, [])
                for src, dst in zip(suffix, suffix[1:])
            ):
                return suffix

        blocked_edges = set(order.traffic_detour_edges) | self._dynamic_blocked_edges()
        route = self.planner.route_planner.find_route(
            start_lm,
            final_goal_lm,
            blocked_edges=blocked_edges,
        )
        order.spatial_route_nodes = [str(node) for node in route.nodes]
        order.spatial_route_revision = self._next_route_revision()
        order.traffic_blocked_since = None
        return list(order.spatial_route_nodes)

    def _attach_spatial_route_to_request(
        self,
        request: dict[str, Any],
        order: FleetOrder,
        start_lm: str,
        planning_goal_lm: str,
        final_goal_lm: str,
    ) -> None:
        try:
            suffix = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
            )
        except ValueError:
            return
        if planning_goal_lm not in suffix:
            return
        goal_index = suffix.index(planning_goal_lm)
        route_nodes = suffix[:goal_index + 1]
        if len(route_nodes) >= 2:
            request["routeNodes"] = route_nodes

    def _adopt_coupled_spatial_detour(
        self,
        order: FleetOrder,
        plan: dict[str, Any],
        final_goal_lm: str,
    ) -> None:
        plan_nodes: list[str] = []
        for value in plan.get("nodes", []):
            node = str(value)
            if node in self.landmarks and (not plan_nodes or plan_nodes[-1] != node):
                plan_nodes.append(node)
        if len(plan_nodes) < 2:
            return
        chunk_goal = plan_nodes[-1]
        old_nodes = [str(node) for node in order.spatial_route_nodes]
        if chunk_goal in old_nodes:
            suffix = old_nodes[old_nodes.index(chunk_goal) + 1:]
        else:
            try:
                suffix = self.planner.route_planner.find_route(
                    chunk_goal,
                    final_goal_lm,
                ).nodes[1:]
            except ValueError:
                suffix = []
        adopted = plan_nodes + [node for node in suffix if node != plan_nodes[-1]]
        if not adopted or adopted[-1] != final_goal_lm:
            return
        order.spatial_route_nodes = adopted
        order.spatial_route_revision = self._next_route_revision()

    def _rolling_planning_goal(
        self,
        start_lm: str,
        final_goal_lm: str,
        order: FleetOrder,
    ) -> str:
        """Choose the committed waypoint before running time-aware MAPF.

        Planning the complete lifelong order first defeats a rolling horizon:
        sufficiently distant goals exceed the low-level time bound before a
        usable prefix exists.  The spatial route is only used to select a graph
        waypoint; MAPF still owns all timing, waiting and conflict decisions.
        """
        horizon = self._rolling_horizon()
        step_limit = self._rolling_horizon_steps()
        if horizon <= 0.0 and step_limit <= 0:
            return final_goal_lm
        try:
            route_nodes = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
            )
        except ValueError:
            return final_goal_lm
        if len(route_nodes) < 2:
            return final_goal_lm

        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        if order.acceleration > 0.0:
            route_payload["acceleration"] = order.acceleration
        speed = self.planner._route_speed(route_payload)
        acceleration = self.planner._route_acceleration(route_payload)
        elapsed = 0.0
        selected_index = 1
        for index in range(1, len(route_nodes)):
            if step_limit > 0 and index > step_limit:
                break
            elapsed += (
                self.planner._edge_tick_cost(
                    route_nodes[index - 1],
                    route_nodes[index],
                    speed,
                    acceleration,
                )
                * max(0.001, self.planner.time_step_sec)
            )
            # Always choose at least the next graph LM.  A single long edge is
            # indivisible and must be committed as one safe graph segment.
            if index == 1 or horizon <= 0.0 or elapsed <= horizon + 0.000001:
                selected_index = index
                continue
            break
        return str(route_nodes[selected_index])

    def _wait_only_rolling_plan(
        self,
        plan: dict[str, Any],
        final_goal_lm: str,
    ) -> bool:
        nodes = [str(node) for node in plan.get("nodes", [])]
        if not nodes:
            return True
        start_lm = str(plan.get("startLm") or nodes[0])
        if final_goal_lm == start_lm:
            return False
        return all(node == start_lm for node in nodes)

    def _rolling_result(
        self,
        result: dict[str, Any],
        final_goals: dict[str, str],
    ) -> dict[str, Any]:
        """Commit only complete graph nodes inside the configured rolling horizon."""
        horizon = self._rolling_horizon()
        step_limit = self._rolling_horizon_steps()
        if horizon <= 0.0 and step_limit <= 0:
            return result
        time_step = max(0.001, float(result.get("timeStepSec", 1.0) or 1.0))
        for plan in result.get("plans", []):
            if not isinstance(plan, dict):
                continue
            robot_name = str(plan.get("robot") or "")
            final_goal = str(final_goals.get(robot_name) or plan.get("goalLm") or "").strip()
            nodes = [str(node) for node in plan.get("nodes", [])]
            times = [int(value) for value in plan.get("times", [])]
            if len(nodes) < 2 or len(times) != len(nodes):
                plan["finalGoalLm"] = final_goal
                continue

            final_index = len(nodes) - 1
            chunk_index = final_index
            if horizon > 0.0:
                allowed_tick = horizon / time_step
                progress_indices = [
                    index
                    for index in range(1, len(nodes))
                    if nodes[index] != nodes[0]
                ]
                eligible = [
                    index
                    for index in progress_indices
                    if float(times[index] - times[0]) <= allowed_tick + 0.000001
                ]
                # A turn can consume the whole short window. Commit the first
                # complete graph move as the minimum useful rolling chunk;
                # never publish a rotate-only plan as "no progress".
                chunk_index = (
                    max(eligible)
                    if eligible
                    else min(progress_indices, default=1)
                )
            if step_limit > 0:
                chunk_index = min(chunk_index, max(1, step_limit))
            chunk_index = min(final_index, max(1, chunk_index))

            chunk_goal = nodes[chunk_index]
            arrival_time = max(0.0, float(times[chunk_index] - times[0]) * time_step)
            trajectory = [
                sample for sample in plan.get("trajectory", [])
                if isinstance(sample, dict)
            ]
            trajectory_end = self._trajectory_chunk_end_index(
                trajectory,
                chunk_goal,
                arrival_time,
            )
            if trajectory_end is not None:
                plan["trajectory"] = trajectory[:trajectory_end + 1]
                if plan["trajectory"]:
                    arrival_time = float(plan["trajectory"][-1].get("t", arrival_time) or arrival_time)
            plan["nodes"] = nodes[:chunk_index + 1]
            plan["times"] = times[:chunk_index + 1]
            for key in ("yaws", "actions"):
                values = plan.get(key)
                if isinstance(values, list) and len(values) >= chunk_index + 1:
                    plan[key] = values[:chunk_index + 1]
            timed_segments = plan.get("timedSegments")
            if isinstance(timed_segments, list):
                plan["timedSegments"] = timed_segments[:chunk_index]
            plan["goalLm"] = chunk_goal
            plan["finalGoalLm"] = final_goal
            plan["arrivalTime"] = arrival_time
            plan["rollingChunk"] = chunk_index < final_index
        return result

    def _trajectory_chunk_end_index(
        self,
        trajectory: list[dict[str, Any]],
        chunk_goal: str,
        arrival_time: float,
    ) -> int | None:
        if not trajectory:
            return None
        candidates = [
            index
            for index, sample in enumerate(trajectory)
            if str(sample.get("lm") or "").strip() == chunk_goal
            and float(sample.get("t", 0.0) or 0.0) >= arrival_time - 0.001
        ]
        if candidates:
            return candidates[0]
        before = [
            index
            for index, sample in enumerate(trajectory)
            if float(sample.get("t", 0.0) or 0.0) <= arrival_time + 0.001
        ]
        return before[-1] if before else 0

    def _remote_poll_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.25
        try:
            return max(0.05, float(fleet.get("remote_poll_interval_sec", 0.25) or 0.25))
        except (TypeError, ValueError):
            return 0.25

    def _sync_remote_robot(self, robot: FleetRobot, now: float, force: bool = False) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        if (
            not force
            and robot.remote_last_poll_at is not None
            and now - robot.remote_last_poll_at < self._remote_poll_interval()
        ):
            return
        robot.remote_last_poll_at = now
        try:
            payload = self.remote_adapter.status(robot.base_url)
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote status failed: {exc}"
            robot.updated_at = now
            return
        self._apply_remote_status(robot, payload, now)

    def _apply_remote_status(self, robot: FleetRobot, payload: dict[str, Any], now: float) -> None:
        status_payload = self._remote_status_robot(payload)
        robot.remote_status = dict(status_payload)
        robot.remote_online = bool(status_payload.get("connected", True))
        robot.remote_error = ""
        robot.updated_at = now

        state = self._normalize_remote_state(str(status_payload.get("state") or "IDLE"))
        robot.status = state
        message = str(status_payload.get("message") or status_payload.get("reason") or state).strip()
        if message:
            robot.last_reason = message

        target_lm = str(status_payload.get("targetLm") or status_payload.get("target_lm") or "").strip()
        if target_lm or not robot.active_order_id:
            robot.target_lm = target_lm

        current_lm = str(
            status_payload.get("currentLm")
            or status_payload.get("currentLM")
            or status_payload.get("nearestLm")
            or status_payload.get("nearest_lm")
            or status_payload.get("currentStation")
            or status_payload.get("current_station")
            or ""
        ).strip()
        if current_lm in self.landmarks:
            robot.current_lm = current_lm

        pose = self._remote_pose_from_status(status_payload)
        if pose is not None:
            robot.pose = pose
            if current_lm not in self.landmarks:
                nearest_lm = self._nearest_lm_for_pose(pose)
                if nearest_lm:
                    robot.current_lm = nearest_lm
        elif robot.current_lm in self.landmarks and robot.pose is None:
            robot.pose = self._pose_at_landmark(robot.current_lm)

        progress = status_payload.get("routeProgress")
        if progress is not None and robot.trajectory:
            try:
                final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                robot.route_clock = max(0.0, min(final_time, final_time * float(progress)))
            except (TypeError, ValueError):
                pass
        elif robot.pose is not None and robot.trajectory:
            robot.route_clock = self._nearest_trajectory_clock(robot.trajectory, robot.pose)

    def _remote_control_owner(self, robot: FleetRobot) -> tuple[str, str]:
        status = robot.remote_status if isinstance(robot.remote_status, dict) else {}
        control = status.get("control")
        if not isinstance(control, dict):
            control = {}
        owner_id = str(
            status.get("controlOwner")
            or status.get("control_owner")
            or control.get("ownerId")
            or control.get("owner_id")
            or ""
        ).strip()
        owner_name = str(
            status.get("controlOwnerName")
            or status.get("control_owner_name")
            or control.get("ownerName")
            or control.get("owner_name")
            or owner_id
        ).strip()
        return owner_id, owner_name

    def _pause_order_for_external_control(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
        owner_name: str,
    ) -> None:
        reason = f"{EXTERNAL_CONTROL_PAUSE_PREFIX} {owner_name or 'another client'}"
        first_pause = not str(order.error or "").startswith(EXTERNAL_CONTROL_PAUSE_PREFIX)
        order.status = "PAUSED"
        order.error = reason
        order.updated_at = now
        robot.status = "MANUAL"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_clock = 0.0
        robot.route_started_at = None
        robot.blocked_since = None
        robot.last_reason = reason
        self._clear_remote_route_metadata(robot)
        robot.updated_at = now
        if first_pause:
            self._event("warn", f"order paused: {order.order_id}; {reason}")

    def _resume_order_after_external_control(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
    ) -> None:
        order.status = "QUEUED"
        order.error = ""
        order.start_lm = self._nearest_lm_for_robot(robot)
        order.updated_at = now
        robot.active_order_id = ""
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_clock = 0.0
        robot.route_started_at = None
        robot.blocked_since = None
        robot.last_reason = "external control released; replanning fleet order"
        self._clear_remote_route_metadata(robot)
        robot.updated_at = now
        self._event("info", f"order resumed after external control: {order.order_id}")
        self._dispatch_order(order, force=True)

    def _normalize_remote_state(self, state: str) -> str:
        value = state.strip().upper()
        if value in {"EXECUTING_ROUTE", "FOLLOWING_ROUTE", "RUNNING"}:
            return "MOVING"
        if value in {"WAITING_TRAFFIC", "WAITING_OBSTACLE", "WAITING"}:
            return "WAITING"
        if value in {"ARRIVED", "IDLE", "STOPPED", "LOCALIZING", "ERROR", "MANUAL"}:
            return value
        if value in {"BLOCKED", "PAUSED"}:
            return "BLOCKED"
        return value or "IDLE"

    def _advance_remote_robot_order(self, robot: FleetRobot, now: float) -> None:
        self._sync_remote_robot(robot, now, force=False)
        if not robot.active_order_id:
            robot.last_tick_at = now
            return
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            robot.active_order_id = ""
            robot.last_tick_at = now
            return

        owner_id, owner_name = self._remote_control_owner(robot)
        if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
            self._pause_order_for_external_control(robot, order, now, owner_name or owner_id)
            robot.last_tick_at = now
            return
        if (
            not owner_id
            and str(order.error or "").startswith(EXTERNAL_CONTROL_PAUSE_PREFIX)
            and robot.status in {"ARRIVED", "IDLE", "STOPPED"}
        ):
            self._resume_order_after_external_control(robot, order, now)
            robot.last_tick_at = now
            return

        target_lm = self._active_order_target(order)
        if robot.status in {"ARRIVED", "IDLE"} and target_lm and robot.current_lm == target_lm:
            order.route_nodes = list(robot.plan_nodes)
            completed = self._advance_or_complete_order(order, robot, now)
            robot.target_lm = ""
            robot.trajectory = []
            robot.plan_nodes = []
            robot.trajectory_dirty = True
            robot.route_clock = 0.0
            robot.route_started_at = None
            robot.blocked_since = None
            robot.last_reason = "arrived"
            if completed:
                robot.status = "ARRIVED"
                self._clear_remote_route_metadata(robot)
                self._event("info", f"order completed: {order.order_id} {robot.name}@{target_lm}")
            else:
                robot.status = "IDLE"
            robot.updated_at = now
        elif (
            robot.status in {"ARRIVED", "IDLE"}
            and target_lm
            and robot.route_chunk_goal_lm
            and robot.current_lm == robot.route_chunk_goal_lm
            and robot.current_lm != target_lm
        ):
            order.start_lm = robot.current_lm
            order.status = "QUEUED"
            order.error = ""
            order.updated_at = now
            robot.target_lm = ""
            robot.trajectory = []
            robot.plan_nodes = []
            robot.trajectory_dirty = True
            robot.route_clock = 0.0
            robot.route_started_at = None
            robot.blocked_since = None
            robot.last_reason = f"chunk arrived at {robot.current_lm}; planning next chunk"
            self._event(
                "info",
                f"route chunk completed: {order.order_id} {robot.name}@{robot.current_lm}; next {target_lm}",
            )
            robot.active_order_id = ""
            if not self._dispatch_order(order, force=True):
                robot.active_order_id = order.order_id
                order.status = "QUEUED"
            robot.updated_at = now
        elif robot.status in {"ERROR", "BLOCKED", "OFFLINE"}:
            order.status = "PAUSED" if robot.status != "OFFLINE" else "QUEUED"
            order.error = robot.last_reason or robot.remote_error or robot.status.lower()
            order.updated_at = now
        elif robot.status in {"MOVING", "WAITING"} and robot.trajectory:
            blocked_reason = self._blocked_ahead(robot, robot.route_clock)
            if blocked_reason and self._should_replan_for_blocked_reason(blocked_reason):
                self._maybe_replan_remote_robot_order(robot, order, now, blocked_reason)
            self._update_active_order_from_robot(robot)
        else:
            self._update_active_order_from_robot(robot)
        robot.last_tick_at = now

    def _plan_for_robot(self, result: dict[str, Any], robot_name: str) -> dict[str, Any] | None:
        for plan in result.get("plans", []):
            if isinstance(plan, dict) and str(plan.get("robot") or "") == robot_name:
                return plan
        return None

    def _execute_remote_plan(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
        result: dict[str, Any],
    ) -> dict[str, Any]:
        route = self._remote_route_payload(robot, order, plan)
        payload = {
            "route": route,
            "order": order.to_dict(),
            "ownerId": FLEET_CONTROL_OWNER_ID,
            "fleet": {
                "manager": "fleet_manager",
                "routeProtocol": "lm_route",
                "planNote": self._plan_note(result),
                "debug": result.get("debug", {}),
            },
        }
        self._ensure_remote_control(robot, "execute route")
        response = self.remote_adapter.execute_route(robot.base_url, payload)
        robot.remote_online = True
        robot.remote_error = ""
        status = response.get("status")
        if isinstance(status, dict):
            self._apply_remote_status(robot, status, time())
        return route

    def _remote_route_payload(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
    ) -> dict[str, Any]:
        full_nodes = [str(item) for item in plan.get("nodes", []) if str(item)]
        start_lm = str(plan.get("startLm") or order.start_lm or (full_nodes[0] if full_nodes else ""))
        final_goal_lm = str(plan.get("goalLm") or order.target_lm or (full_nodes[-1] if full_nodes else ""))
        if not full_nodes and start_lm and final_goal_lm:
            full_nodes = [start_lm, final_goal_lm] if start_lm != final_goal_lm else [start_lm]
        chunk_nodes = self._remote_route_chunk_nodes(full_nodes)
        chunk_start_lm = chunk_nodes[0] if chunk_nodes else start_lm
        chunk_goal_lm = chunk_nodes[-1] if chunk_nodes else final_goal_lm
        chunk_length = self._lm_path_length(chunk_nodes)
        full_length = float(plan.get("length", 0.0) or 0.0)
        revision = self._next_route_revision()
        chunk_index = self._next_remote_chunk_index(robot, order, chunk_start_lm)
        is_final = bool(chunk_nodes and full_nodes and chunk_nodes[-1] == full_nodes[-1])
        dispatch_epoch = time() + self._remote_dispatch_lead_time()
        timed_segments = self._timed_segments_from_trajectory(plan.get("trajectory", []))
        if not timed_segments:
            timed_segments = [
                dict(item)
                for item in plan.get("timedSegments", [])
                if isinstance(item, dict)
            ]
        return {
            "routeId": f"{order.order_id}:{order.step_index}",
            "protocol": "lm_route",
            "protocolVersion": 2,
            "revision": revision,
            "orderId": order.order_id,
            "startLm": chunk_start_lm,
            "goalLm": chunk_goal_lm,
            "finalGoalLm": final_goal_lm,
            "nodes": chunk_nodes,
            "fullNodes": full_nodes,
            "length": chunk_length if chunk_length > 0.0 else full_length,
            "fullLength": full_length,
            "replaceMode": "immediate",
            "dispatchEpochSec": dispatch_epoch,
            "timedSegments": timed_segments,
            "chunk": {
                "index": chunk_index,
                "stepIndex": order.step_index,
                "offset": 0,
                "startLm": chunk_start_lm,
                "goalLm": chunk_goal_lm,
                "finalGoalLm": final_goal_lm,
                "nodes": chunk_nodes,
                "fullNodes": full_nodes,
                "isFinal": is_final,
            },
        }

    def _remote_dispatch_lead_time(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.75
        try:
            return max(0.25, float(fleet.get("remote_dispatch_lead_sec", 0.75) or 0.75))
        except (TypeError, ValueError):
            return 0.75

    def _timed_segments_from_trajectory(self, raw_trajectory: Any) -> list[dict[str, Any]]:
        if not isinstance(raw_trajectory, list):
            return []
        trajectory = [item for item in raw_trajectory if isinstance(item, dict)]
        segments: list[dict[str, Any]] = []
        for index in range(len(trajectory) - 1):
            start = trajectory[index]
            end = trajectory[index + 1]
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            start_time = float(start.get("t", 0.0) or 0.0)
            end_time = float(end.get("t", start_time) or start_time)
            if edge_id.startswith("WAIT@ROTATE:"):
                node = str(end.get("lm") or start.get("lm") or "").strip()
                segments.append(
                    {
                        "kind": "rotate",
                        "node": node,
                        "fromYaw": float(start.get("yaw", 0.0) or 0.0),
                        "toYaw": float(end.get("yaw", 0.0) or 0.0),
                        "notBeforeSec": start_time,
                        "plannedArrivalSec": end_time,
                    }
                )
                continue
            parsed = self._parse_edge_id(edge_id)
            if parsed is None or edge_id.startswith("WAIT@") or parsed[0] == parsed[1]:
                continue
            src, dst = parsed
            if (
                segments
                and segments[-1].get("from") == src
                and segments[-1].get("to") == dst
                and abs(float(segments[-1].get("plannedArrivalSec", 0.0)) - start_time) < 1e-6
            ):
                segments[-1]["plannedArrivalSec"] = end_time
                continue
            segments.append(
                {
                    "kind": "move",
                    "from": src,
                    "to": dst,
                    "notBeforeSec": start_time,
                    "plannedArrivalSec": end_time,
                }
            )
        return segments

    def _remote_route_chunk_nodes(self, nodes: list[str]) -> list[str]:
        if len(nodes) <= 2:
            return list(nodes)
        limit = self._remote_route_chunk_lms()
        if limit <= 0 or limit >= len(nodes):
            return list(nodes)
        return list(nodes[:max(2, limit)])

    def _remote_route_chunk_lms(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 5
        try:
            return max(0, int(fleet.get("remote_route_chunk_lms", 5) or 0))
        except (TypeError, ValueError):
            return 5

    def _next_route_revision(self) -> int:
        now_ms = int(time() * 1000)
        self._route_revision_seq = max(self._route_revision_seq + 1, now_ms)
        return self._route_revision_seq

    def _next_remote_chunk_index(self, robot: FleetRobot, order: FleetOrder, chunk_start_lm: str) -> int:
        if robot.route_final_lm == order.target_lm:
            if robot.route_chunk_goal_lm and robot.current_lm == robot.route_chunk_goal_lm:
                return max(0, robot.route_chunk_index + 1)
            if robot.active_order_id != order.order_id:
                return 0
            if chunk_start_lm == robot.current_lm:
                return max(0, robot.route_chunk_index)
        return 0

    def _apply_remote_route_metadata(self, robot: FleetRobot, route: dict[str, Any], now: float) -> None:
        chunk = route.get("chunk")
        if not isinstance(chunk, dict):
            chunk = {}
        robot.route_revision = int(route.get("revision", 0) or 0)
        robot.route_chunk_index = int(chunk.get("index", 0) or 0)
        robot.route_chunk_goal_lm = str(chunk.get("goalLm") or route.get("goalLm") or "").strip()
        robot.route_final_lm = str(
            chunk.get("finalGoalLm")
            or route.get("finalGoalLm")
            or route.get("goalLm")
            or ""
        ).strip()
        self._update_route_preview(
            robot,
            robot.current_lm,
            robot.route_final_lm,
            committed_trajectory=robot.trajectory,
            committed_nodes=route.get("nodes"),
        )
        if robot.route_chunk_goal_lm:
            robot.target_lm = robot.route_chunk_goal_lm
        robot.updated_at = now

    def _lm_path_length(self, nodes: list[str]) -> float:
        if len(nodes) < 2:
            return 0.0
        length = 0.0
        edge_lengths = {
            (edge.from_name, edge.to_name): float(edge.length)
            for edge in self.edges
        }
        for start_lm, goal_lm in zip(nodes, nodes[1:]):
            if start_lm == goal_lm:
                continue
            edge_length = edge_lengths.get((start_lm, goal_lm))
            if edge_length is None:
                return 0.0
            length += edge_length
        return length

    def _cancel_remote_route(self, robot: FleetRobot, reason: str) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        try:
            self._ensure_remote_control(robot, "cancel route")
            self.remote_adapter.cancel_route(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
        except Exception as exc:
            robot.remote_online = self._is_remote_control_conflict(exc)
            robot.remote_error = str(exc)
            self._event("warn", f"{robot.name} remote cancel failed: {exc}")
        robot.last_reason = reason

    def _stop_remote_robot(self, robot: FleetRobot) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        try:
            self._ensure_remote_control(robot, "stop")
            self.remote_adapter.stop(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
        except Exception as exc:
            robot.remote_online = self._is_remote_control_conflict(exc)
            robot.remote_error = str(exc)
            self._event("warn", f"{robot.name} remote stop failed: {exc}")

    def _ensure_remote_control(self, robot: FleetRobot, action: str) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        try:
            self.remote_adapter.acquire_control(
                robot.base_url,
                owner_id=FLEET_CONTROL_OWNER_ID,
                owner_name=FLEET_CONTROL_OWNER_NAME,
                # A Fleet Manager background tick must never steal a robot
                # from an operator who explicitly took manual control.
                force=False,
                lease_ms=0,
            )
            robot.remote_online = True
            robot.remote_error = ""
        except Exception as exc:
            robot.remote_online = self._is_remote_control_conflict(exc)
            robot.remote_error = str(exc)
            raise ValueError(f"remote control acquire failed before {action}: {exc}") from exc

    @staticmethod
    def _is_remote_control_conflict(error: Exception | str) -> bool:
        text = str(error or "").strip().lower()
        return "control is owned by" in text or "control owner" in text

    def _nearest_trajectory_clock(self, trajectory: list[dict[str, Any]], pose: dict[str, float]) -> float:
        best_time = float(trajectory[0].get("t", 0.0) or 0.0) if trajectory else 0.0
        best_dist = float("inf")
        px = float(pose.get("x", 0.0) or 0.0)
        py = float(pose.get("y", 0.0) or 0.0)
        for sample in trajectory:
            dist = math.hypot(
                px - float(sample.get("x", 0.0) or 0.0),
                py - float(sample.get("y", 0.0) or 0.0),
            )
            if dist < best_dist:
                best_dist = dist
                best_time = float(sample.get("t", 0.0) or 0.0)
        return best_time

    def _plan_valid_requests(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        # FleetMapfPlanner keeps reusable graph/planner objects.  Dynamic
        # dispatch runs in one background thread, while explicit operator
        # requests may still arrive from the HTTP server, so serialize planner
        # use without serializing runtime ticks and rendering.
        with self._planner_lock:
            return self._plan_valid_requests_unlocked(valid_requests, payload)

    def _plan_valid_requests_unlocked(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        payload = payload or {}
        try:
            reservation_offset = max(0.0, float(payload.get("reservationOffsetSec", 0.0) or 0.0))
        except (TypeError, ValueError):
            reservation_offset = 0.0
        hard_blocked_lms = self._hard_blocked_lms(payload)
        blocked_edges = self._hard_blocked_edges(payload) | self._dynamic_blocked_edges()
        release_owners = self._release_blocker_names_for_requests(valid_requests)
        release_start_lms = {
            str(request.get("startLm", "")).strip()
            for request in valid_requests
            if str(request.get("startLm", "")).strip() in self.landmarks
        }
        reserved_edge_intervals = self._reserved_edge_intervals(
            valid_requests,
            ignore_robot_names=release_owners,
            prediction_offset=reservation_offset,
        )
        reserved_vertex_intervals = self._reserved_vertex_intervals(
            valid_requests,
            ignore_robot_names=release_owners,
            ignore_nodes=release_start_lms,
            prediction_offset=reservation_offset,
        )
        soft_blocked_lms = (
            set()
            if bool(payload.get("skipSoftBlockedDetour", False))
            else self._soft_blocked_lms(valid_requests, hard_blocked_lms)
        )
        planner_payload = {
            **payload,
            "robots": valid_requests,
            "blocked_edges": [
                {"from": src, "to": dst}
                for src, dst in sorted(blocked_edges)
            ],
            "reserved_vertex_constraints": [],
            "reserved_edge_constraints": [],
            "reserved_vertex_intervals": [
                {
                    "node": node,
                    "start": start,
                    "end": end,
                    "robot": robot_name,
                }
                for node, start, end, robot_name in reserved_vertex_intervals
            ],
            "reserved_edge_intervals": [
                {
                    "from": src,
                    "to": dst,
                    "start": start,
                    "end": end,
                    "robot": robot_name,
                }
                for src, dst, start, end, robot_name in reserved_edge_intervals
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
                result = self._apply_continuous_reservation_waits(
                    result,
                    ignore_robot_names=release_owners,
                    prediction_offset=reservation_offset,
                )
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
                result = self._apply_continuous_reservation_waits(
                    result,
                    ignore_robot_names=release_owners,
                    prediction_offset=reservation_offset,
                )
                self._event(
                    "warn",
                    "planner found no detour; using original route and runtime waiting",
                )
            return result

        result = self.planner.plan(
            {
                **planner_payload,
                "blocked_lms": sorted(hard_blocked_lms),
            }
        )
        if result.get("ok"):
            result = self._apply_continuous_reservation_waits(
                result,
                ignore_robot_names=release_owners,
                prediction_offset=reservation_offset,
            )
        return result

    def _apply_planner_result(
        self,
        result: dict[str, Any],
        now: float | None = None,
        order_id: str | None = None,
    ) -> None:
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
            if robot.is_remote() and robot.pose:
                robot.route_clock = self._nearest_trajectory_clock(trajectory, robot.pose)
            else:
                robot.route_clock = 0.0
            robot.last_tick_at = now
            if not robot.is_remote():
                robot.pose = self._pose_at_trajectory(robot.trajectory, 0.0) or robot.pose
            robot.route_note = self._plan_note(result)
            robot.last_reason = robot.route_note if trajectory else "empty trajectory"
            robot.blocked_since = None
            robot.traffic_stall_since = None
            self._clear_wait_dependency(robot)
            self._clear_deadlock_retreat(robot)
            if order_id is not None:
                robot.active_order_id = order_id
            robot.updated_at = now

    def _plan_note(self, result: dict[str, Any]) -> str:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return "planner accepted"
        reason = str(debug.get("reason", "") or "")
        blocked_edges = debug.get("hardBlockedEdges") or debug.get("blockedEdges", [])
        reserved_detour_edges = debug.get("reservedDetourEdges", [])
        reserved_edges = int(debug.get("reservedEdges", 0) or 0)
        continuous_waits = int(debug.get("continuousWaits", 0) or 0)
        if "fallback_wait" in reason or "reserved_interval_fallback_wait" in reason:
            return "FALLBACK_WAIT"
        if "reserved_edge_detour" in reason:
            return "DETOUR: reserved edge"
        if "detour_soft_blocks" in reason:
            return "DETOUR"
        if isinstance(blocked_edges, list) and blocked_edges:
            return "DETOUR: edge blocked"
        if continuous_waits > 0:
            return "WAIT: reserved corridor"
        if reserved_edges > 0 or (isinstance(reserved_detour_edges, list) and reserved_detour_edges):
            return "DETOUR: reserved edge"
        return "planner accepted"

    def _planner_deadlock_result(self, result: dict[str, Any]) -> bool:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return False
        if bool(debug.get("deadlock")):
            return True
        try:
            return int(debug.get("continuousUnresolved", 0) or 0) > 0
        except (TypeError, ValueError):
            return False

    def _planner_failure_reason(self, result: dict[str, Any]) -> str:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return "planner rejected"
        if self._planner_deadlock_result(result):
            detail = str(debug.get("deadlockReason") or "").strip()
            return f"deadlock: {detail or 'planner could not resolve robot traffic; robots hold position'}"
        return str(debug.get("reason") or "planner rejected")

    def _apply_continuous_reservation_waits(
        self,
        result: dict[str, Any],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> dict[str, Any]:
        plans = result.get("plans", [])
        if not isinstance(plans, list) or not plans:
            return result

        total_wait = 0.0
        total_conflicts = 0
        wait_count = 0
        unresolved_count = 0
        planned_names = {
            str(plan.get("robot", ""))
            for plan in plans
            if isinstance(plan, dict) and str(plan.get("robot", ""))
        }
        ignored_names = planned_names | (ignore_robot_names or set())
        for plan in plans:
            if not isinstance(plan, dict):
                continue
            robot_name = str(plan.get("robot", ""))
            trajectory = [
                item for item in plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            if not robot_name or len(trajectory) < 2:
                continue
            trajectory, stats = self._schedule_trajectory_against_corridors(
                robot_name,
                trajectory,
                ignore_robot_names=ignored_names,
                prediction_offset=prediction_offset,
            )
            if stats["conflicts"] > 0:
                plan["trajectory"] = trajectory
                plan["arrivalTime"] = float(trajectory[-1].get("t", 0.0) or 0.0)
                total_wait += stats["wait"]
                total_conflicts += stats["conflicts"]
                wait_count += stats["waits"]
            unresolved_count += int(stats.get("unresolved", 0) or 0)

        batch_trajectory_stats = self._schedule_batch_trajectories(plans)
        total_wait += batch_trajectory_stats["wait"]
        total_conflicts += batch_trajectory_stats["conflicts"]
        wait_count += batch_trajectory_stats["waits"]
        unresolved_count += int(batch_trajectory_stats.get("unresolved", 0) or 0)

        if total_conflicts > 0 or unresolved_count > 0:
            debug = result.setdefault("debug", {})
            debug["continuousConflicts"] = int(debug.get("continuousConflicts", 0) or 0) + total_conflicts
            debug["continuousWaits"] = int(debug.get("continuousWaits", 0) or 0) + wait_count
            debug["continuousWaitSec"] = round(float(debug.get("continuousWaitSec", 0.0) or 0.0) + total_wait, 3)
            debug["continuousUnresolved"] = int(debug.get("continuousUnresolved", 0) or 0) + unresolved_count
            if batch_trajectory_stats["conflicts"] > 0:
                debug["batchContinuousConflicts"] = int(debug.get("batchContinuousConflicts", 0) or 0) + batch_trajectory_stats["conflicts"]
                debug["batchContinuousWaits"] = int(debug.get("batchContinuousWaits", 0) or 0) + batch_trajectory_stats["waits"]
                debug["batchContinuousWaitSec"] = round(float(debug.get("batchContinuousWaitSec", 0.0) or 0.0) + batch_trajectory_stats["wait"], 3)
            debug["reason"] = f"{debug.get('reason', 'success')}:reserved_corridor_wait"
            if unresolved_count > 0:
                debug["reason"] = f"{debug.get('reason', 'success')}:continuous_conflict_unresolved"
                debug["deadlock"] = True
                debug["deadlockReason"] = (
                    "continuous reservation conflict could not be resolved; robots will hold position"
                )
                debug["rejectedPlanCount"] = len(plans)
                result["ok"] = False
                result["plans"] = []
        return result

    def _schedule_trajectory_against_corridors(
        self,
        robot_name: str,
        trajectory: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> tuple[list[dict[str, Any]], dict[str, float | int]]:
        conflicts = 0
        waits = 0
        total_wait = 0.0
        max_iterations = 10
        ignored = ignore_robot_names or set()
        for _ in range(max_iterations):
            conflict = self._first_continuous_corridor_conflict(
                robot_name,
                trajectory,
                ignore_robot_names=ignored,
                prediction_offset=prediction_offset,
            )
            if conflict is None:
                break
            conflicts += 1
            wait_point = self._wait_insert_point(
                trajectory,
                max(0.0, float(conflict["time"]) - self._reservation_safety_time()),
            )
            wait_duration = self._wait_duration_for_conflict(
                trajectory,
                float(conflict["time"]),
                str(conflict["other"]),
                prediction_offset=prediction_offset,
            )
            if (
                float(conflict["time"]) <= self._continuous_collision_step()
                and wait_duration >= self._reservation_horizon()
            ):
                # A stationary robot already occupies the first pose. Adding
                # the same horizon-sized wait ten times cannot resolve it and
                # was a major source of visible planner stalls/event spam.
                self._event(
                    "warn",
                    f"{robot_name} immediate corridor block by {conflict['other']}; detour required",
                )
                break
            trajectory = self._insert_trajectory_wait(
                trajectory,
                wait_point["index"],
                wait_duration,
            )
            waits += 1
            total_wait += wait_duration
            self._event(
                "warn",
                (
                    f"{robot_name} reservation wait: t={float(conflict['time']):.2f}s "
                    f"edge={conflict['edge']} other={conflict['other']} "
                    f"wait={wait_duration:.2f}s"
                ),
            )
        remaining_conflict = self._first_continuous_corridor_conflict(
            robot_name,
            trajectory,
            ignore_robot_names=ignored,
            prediction_offset=prediction_offset,
        )
        if remaining_conflict is not None:
            self._event(
                "error",
                (
                    f"{robot_name} reservation deadlock: "
                    f"t={float(remaining_conflict['time']):.2f}s "
                    f"edge={remaining_conflict['edge']} "
                    f"other={remaining_conflict['other']}"
                ),
            )
            return trajectory, {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 1}
        return trajectory, {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 0}

    def _schedule_batch_trajectories(
        self,
        plans: list[Any],
    ) -> dict[str, float | int]:
        scheduled = [
            plan for plan in plans
            if isinstance(plan, dict)
            and str(plan.get("robot", ""))
            and isinstance(plan.get("trajectory"), list)
            and len(plan.get("trajectory", [])) >= 2
        ]
        if len(scheduled) < 2:
            return {"conflicts": 0, "waits": 0, "wait": 0.0, "unresolved": 0}

        conflicts = 0
        waits = 0
        total_wait = 0.0
        max_iterations = self._batch_wait_max_iterations()
        for _ in range(max_iterations):
            conflict = self._first_batch_trajectory_conflict(scheduled)
            if conflict is None:
                break
            waiting_plan = scheduled[int(conflict["waitIndex"])]
            priority_plan = scheduled[int(conflict["priorityIndex"])]
            trajectory = [
                item for item in waiting_plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            priority_trajectory = [
                item for item in priority_plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            if len(trajectory) < 2 or len(priority_trajectory) < 2:
                break

            wait_point = self._wait_insert_point(trajectory, float(conflict["time"]))
            wait_duration = self._wait_duration_for_peer_conflict(
                trajectory,
                priority_trajectory,
                float(conflict["time"]),
            )
            trajectory = self._insert_trajectory_wait(
                trajectory,
                int(wait_point["index"]),
                wait_duration,
            )
            waiting_plan["trajectory"] = trajectory
            waiting_plan["arrivalTime"] = float(trajectory[-1].get("t", 0.0) or 0.0)
            conflicts += 1
            waits += 1
            total_wait += wait_duration
            self._event(
                "warn",
                (
                    f"{waiting_plan.get('robot')} batch reservation wait: "
                    f"t={float(conflict['time']):.2f}s "
                    f"edge={conflict['edge']} "
                    f"priority={priority_plan.get('robot')} "
                    f"wait={wait_duration:.2f}s"
                ),
            )
        remaining_conflict = self._first_batch_trajectory_conflict(scheduled)
        if remaining_conflict is not None:
            self._event(
                "error",
                (
                    "batch reservation unresolved: "
                    f"t={float(remaining_conflict['time']):.2f}s "
                    f"edge={remaining_conflict['edge']}"
                ),
            )
            return {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 1}
        return {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 0}

    def _first_batch_trajectory_conflict(
        self,
        plans: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        step = self._continuous_collision_step()
        final_time = 0.0
        for plan in plans:
            trajectory = plan.get("trajectory", [])
            if isinstance(trajectory, list) and trajectory:
                final_time = max(final_time, float(trajectory[-1].get("t", 0.0) or 0.0))
        horizon = min(final_time, self._batch_collision_horizon(final_time))
        t = 0.0
        while t <= horizon + 0.000001:
            # Pose interpolation used to run once for every robot pair.  At 20
            # robots that means 190 scans through sampled trajectories for
            # every time slice.  Cache the N poses for this slice and keep the
            # same exact footprint checks below.
            poses = [
                self._pose_at_trajectory(
                    plan.get("trajectory", [])
                    if isinstance(plan.get("trajectory", []), list)
                    else [],
                    t,
                )
                for plan in plans
            ]
            cell_size = max(0.05, self.collision.robot_broadphase_distance())
            grid: dict[tuple[int, int], list[int]] = {}
            candidate_pairs: set[tuple[int, int]] = set()
            for index, pose in enumerate(poses):
                if pose is None:
                    continue
                cell = (
                    math.floor(float(pose.get("x", 0.0) or 0.0) / cell_size),
                    math.floor(float(pose.get("y", 0.0) or 0.0) / cell_size),
                )
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for other_index in grid.get((cell[0] + dx, cell[1] + dy), ()):
                            candidate_pairs.add((other_index, index))
                grid.setdefault(cell, []).append(index)

            for priority_index, wait_index in sorted(candidate_pairs):
                priority_trajectory = plans[priority_index].get("trajectory", [])
                if not isinstance(priority_trajectory, list):
                    continue
                priority_pose = poses[priority_index]
                if priority_pose is None:
                    continue
                waiting_trajectory = plans[wait_index].get("trajectory", [])
                if not isinstance(waiting_trajectory, list):
                    continue
                waiting_pose = poses[wait_index]
                if waiting_pose is None:
                    continue
                if self.collision.robot_footprints_conflict(priority_pose, waiting_pose):
                    priority_edge = self._edge_id_at_trajectory(priority_trajectory, t) or "unknown"
                    waiting_edge = self._edge_id_at_trajectory(waiting_trajectory, t) or "unknown"
                    if waiting_edge.startswith("WAIT@") and not priority_edge.startswith("WAIT@"):
                        return {
                            "time": t,
                            "priorityIndex": wait_index,
                            "waitIndex": priority_index,
                            "edge": priority_edge,
                        }
                    priority_entry = self._edge_start_time_at_trajectory(priority_trajectory, t)
                    waiting_entry = self._edge_start_time_at_trajectory(waiting_trajectory, t)
                    if priority_entry > waiting_entry + step:
                        return {
                            "time": t,
                            "priorityIndex": wait_index,
                            "waitIndex": priority_index,
                            "edge": priority_edge,
                        }
                    return {
                        "time": t,
                        "priorityIndex": priority_index,
                        "waitIndex": wait_index,
                        "edge": waiting_edge,
                    }
            t += step
        return None

    def _edge_start_time_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> float:
        if not trajectory:
            return 0.0
        segment_index = 0
        for index in range(len(trajectory) - 1):
            start_t = float(trajectory[index].get("t", 0.0) or 0.0)
            end_t = float(trajectory[index + 1].get("t", 0.0) or 0.0)
            if start_t <= elapsed <= end_t:
                segment_index = index
                break
        edge_id = str(
            trajectory[min(segment_index + 1, len(trajectory) - 1)].get("edgeId")
            or trajectory[segment_index].get("edgeId")
            or ""
        )
        insert_index = segment_index
        while insert_index > 0:
            previous_edge = str(trajectory[insert_index].get("edgeId", "") or "")
            if previous_edge != edge_id:
                break
            insert_index -= 1
        return float(trajectory[max(0, insert_index)].get("t", 0.0) or 0.0)

    def _wait_duration_for_peer_conflict(
        self,
        trajectory: list[dict[str, Any]],
        priority_trajectory: list[dict[str, Any]],
        conflict_time: float,
    ) -> float:
        conflict_pose = self._pose_at_trajectory(trajectory, conflict_time)
        if conflict_pose is None:
            return self._reservation_safety_time()

        step = self._continuous_collision_step()
        safety = self._reservation_safety_time()
        max_wait = max(2.0, self._reservation_horizon())
        wait = 0.0
        while wait <= max_wait + 0.000001:
            priority_pose = self._pose_at_trajectory(priority_trajectory, conflict_time + wait)
            if priority_pose is None or not self.collision.robot_footprints_conflict(conflict_pose, priority_pose):
                return max(safety, wait + safety)
            wait += step
        return max_wait + safety

    def _first_continuous_corridor_conflict(
        self,
        robot_name: str,
        trajectory: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> dict[str, Any] | None:
        ignored = ignore_robot_names or set()
        corridor_robots = [
            robot
            for robot in self._runtime_robots()
            if robot.name != robot_name
            and robot.name not in ignored
            and robot.pose is not None
        ]
        if not corridor_robots:
            return None
        final_time = float(trajectory[-1].get("t", 0.0) or 0.0)
        step = self._continuous_collision_step()
        horizon = min(final_time, self._reservation_horizon())
        t = 0.0
        while t <= horizon + 0.000001:
            pose = self._pose_at_trajectory(trajectory, t)
            if pose is None:
                t += step
                continue
            for other in corridor_robots:
                other_pose = self._predicted_robot_pose(other, prediction_offset + t)
                if other_pose is None:
                    continue
                if self.collision.robot_footprints_conflict(pose, other_pose):
                    edge = self._edge_id_at_trajectory(trajectory, t)
                    return {
                        "time": t,
                        "other": other.name,
                        "edge": edge or "unknown",
                    }
            t += step
        return None

    def _wait_duration_for_conflict(
        self,
        trajectory: list[dict[str, Any]],
        conflict_time: float,
        other_name: str,
        prediction_offset: float = 0.0,
    ) -> float:
        conflict_pose = self._pose_at_trajectory(trajectory, conflict_time)
        other = self.robots.get(other_name)
        if conflict_pose is None or other is None:
            return self._reservation_safety_time()

        step = self._continuous_collision_step()
        safety = self._reservation_safety_time()
        max_wait = max(2.0, self._reservation_horizon())
        wait = 0.0
        while wait <= max_wait + 0.000001:
            other_pose = self._predicted_robot_pose(
                other,
                prediction_offset + conflict_time + wait,
            )
            if other_pose is None or not self.collision.robot_footprints_conflict(conflict_pose, other_pose):
                return max(safety, wait + safety)
            wait += step
        return max_wait + safety

    def _wait_insert_point(
        self,
        trajectory: list[dict[str, Any]],
        conflict_time: float,
        clamp_to_edge_start: bool = True,
    ) -> dict[str, float | int]:
        if conflict_time <= 0.0 or len(trajectory) <= 1:
            return {"index": 0, "time": 0.0}
        segment_index = 0
        for index in range(len(trajectory) - 1):
            start_t = float(trajectory[index].get("t", 0.0) or 0.0)
            end_t = float(trajectory[index + 1].get("t", 0.0) or 0.0)
            if start_t <= conflict_time <= end_t:
                segment_index = index
                break
        if not clamp_to_edge_start:
            return {
                "index": max(0, segment_index),
                "time": float(trajectory[max(0, segment_index)].get("t", 0.0) or 0.0),
            }
        edge_id = str(
            trajectory[min(segment_index + 1, len(trajectory) - 1)].get("edgeId")
            or trajectory[segment_index].get("edgeId")
            or ""
        )
        insert_index = segment_index
        while insert_index > 0:
            previous_edge = str(trajectory[insert_index].get("edgeId", "") or "")
            if previous_edge != edge_id:
                break
            insert_index -= 1
        return {
            "index": max(0, insert_index),
            "time": float(trajectory[max(0, insert_index)].get("t", 0.0) or 0.0),
        }

    def _insert_trajectory_wait(
        self,
        trajectory: list[dict[str, Any]],
        insert_index: int,
        wait_duration: float,
    ) -> list[dict[str, Any]]:
        if wait_duration <= 0.0 or not trajectory:
            return trajectory
        insert_index = max(0, min(insert_index, len(trajectory) - 1))
        wait_duration = max(0.0, wait_duration)
        anchor = dict(trajectory[insert_index])
        anchor_time = float(anchor.get("t", 0.0) or 0.0)
        hold = {
            **anchor,
            "t": anchor_time + wait_duration,
            "edgeId": f"WAIT@{anchor.get('edgeId', 'route')}",
        }
        shifted = [
            {
                **sample,
                "t": float(sample.get("t", 0.0) or 0.0) + wait_duration,
            }
            for sample in trajectory[insert_index + 1:]
        ]
        return trajectory[: insert_index + 1] + [hold] + shifted

    def _dynamic_blocked_edges(self) -> set[tuple[str, str]]:
        blocked = set(self._static_blocked_edges)
        if not self.obstacles and not self.obstacle_areas:
            return blocked
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

    def _static_map_blocked_edges(self) -> set[tuple[str, str]]:
        if self.collision.map_pixels is None or self.collision.map_metadata is None:
            return set()
        blocked: set[tuple[str, str]] = set()
        for edge in self.edges:
            route = PlannedRoute(
                nodes=[edge.from_name, edge.to_name],
                edges=[edge],
                length=edge.length,
            )
            try:
                samples = self.planner.route_planner.sample_route(
                    route,
                    sample_distance=0.10,
                )
            except Exception:
                blocked.add((edge.from_name, edge.to_name))
                continue
            for sample in samples:
                pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                }
                if self.collision.blocked_reason(pose, [], []):
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

        for robot in self._runtime_robots():
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

    def _reserved_edge_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, str, float, float, str]]:
        request_names = {str(request.get("name", "")) for request in requests}
        ignored = ignore_robot_names or set()
        horizon = self._reservation_horizon()
        safety = self._reservation_safety_time()
        intervals: list[tuple[str, str, float, float, str]] = []
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.name in ignored:
                continue
            if robot.status not in {"MOVING", "WAITING"} or len(robot.trajectory) < 2:
                continue
            active_edge: tuple[str, str] | None = None
            active_start = 0.0
            active_end = 0.0

            def flush_edge() -> None:
                nonlocal active_edge, active_start, active_end
                if active_edge is None:
                    return
                start_time = active_start - safety
                end_time = active_end + safety
                if end_time >= 0.0 and start_time <= horizon:
                    src, dst = active_edge
                    intervals.append(
                        (
                            src,
                            dst,
                            max(0.0, start_time),
                            min(horizon, end_time),
                            robot.name,
                        )
                    )
                active_edge = None

            for index in range(len(robot.trajectory) - 1):
                start = robot.trajectory[index]
                end = robot.trajectory[index + 1]
                edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
                edge = self._parse_edge_id(edge_id)
                if edge is None:
                    flush_edge()
                    continue
                future_clock = robot.route_clock + max(0.0, prediction_offset)
                start_time = float(start.get("t", 0.0) or 0.0) - future_clock
                end_time = float(end.get("t", 0.0) or 0.0) - future_clock
                if edge != active_edge:
                    flush_edge()
                    active_edge = edge
                    active_start = start_time
                    active_end = end_time
                else:
                    active_end = end_time
            flush_edge()
        return intervals

    def _reserved_vertex_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        ignore_nodes: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, float, float, str]]:
        request_names = {str(request.get("name", "")) for request in requests}
        ignored = ignore_robot_names or set()
        skipped_nodes = ignore_nodes or set()
        horizon = self._reservation_horizon()
        safety = self._reservation_safety_time()
        intervals: list[tuple[str, float, float, str]] = []

        def add_interval(node: str, start: float, end: float, owner: str) -> None:
            if node not in self.landmarks:
                return
            if node in skipped_nodes:
                return
            start_time = max(0.0, min(start, end))
            end_time = min(horizon, max(start, end))
            if end_time < 0.0 or start_time > horizon:
                return
            intervals.append((node, start_time, end_time, owner))

        for robot in self._runtime_robots():
            if robot.name in request_names or robot.name in ignored:
                continue

            future_pose = self._predicted_robot_pose(robot, max(0.0, prediction_offset))
            current_lm = (
                self._nearest_lm_for_pose(future_pose)
                if future_pose is not None
                else self._nearest_lm_for_robot(robot)
            )
            if current_lm:
                add_interval(current_lm, 0.0, safety * 2.0, robot.name)

            if robot.status not in {"MOVING", "WAITING"} or len(robot.trajectory) < 2:
                if current_lm:
                    add_interval(current_lm, 0.0, horizon, robot.name)
                continue

            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            future_clock = robot.route_clock + max(0.0, prediction_offset)
            if future_clock >= final_time:
                if current_lm:
                    add_interval(current_lm, 0.0, horizon, robot.name)
                continue

            active_edge: tuple[str, str] | None = None
            active_start = 0.0
            active_end = 0.0

            def flush_edge_vertices() -> None:
                nonlocal active_edge
                if active_edge is None:
                    return
                src, dst = active_edge
                add_interval(src, active_start - safety, active_start + safety, robot.name)
                add_interval(dst, active_end - safety, active_end + safety, robot.name)
                active_edge = None

            for index in range(len(robot.trajectory) - 1):
                start = robot.trajectory[index]
                end = robot.trajectory[index + 1]
                start_time = float(start.get("t", 0.0) or 0.0) - future_clock
                end_time = float(end.get("t", 0.0) or 0.0) - future_clock
                if end_time < -safety or start_time > horizon + safety:
                    continue

                edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
                edge = self._parse_edge_id(edge_id)
                if edge is not None:
                    if edge != active_edge:
                        flush_edge_vertices()
                        active_edge = edge
                        active_start = start_time
                        active_end = end_time
                    else:
                        active_end = end_time
                    continue

                flush_edge_vertices()
                wait_lm = self._lm_from_wait_segment(start, end)
                if wait_lm:
                    add_interval(wait_lm, start_time - safety, end_time + safety, robot.name)
            flush_edge_vertices()

            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0) - robot.route_clock
            final_lm = robot.target_lm if robot.target_lm in self.landmarks else self._nearest_lm_for_robot(robot)
            if final_lm and final_time <= horizon:
                add_interval(final_lm, final_time - safety, horizon, robot.name)
        return intervals

    def _lm_from_wait_segment(
        self,
        start: dict[str, Any],
        end: dict[str, Any],
    ) -> str:
        for sample in (end, start):
            lm = str(sample.get("lm") or "").strip()
            if lm in self.landmarks:
                return lm
        edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
        if edge_id.startswith("WAIT@"):
            edge_id = edge_id[5:]
        if "->" in edge_id:
            src, dst = edge_id.split("->", 1)
            src = src.strip()
            dst = dst.strip()
            if src == dst and src in self.landmarks:
                return src
        pose = self._pose_from_sample(end)
        return self._nearest_lm_for_pose(pose)

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

    def _rolling_horizon(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 10.0
        try:
            return max(0.0, float(fleet.get("rolling_horizon_sec", 10.0) or 0.0))
        except (TypeError, ValueError):
            return 10.0

    def _rolling_horizon_steps(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0
        try:
            return max(0, int(fleet.get("rolling_horizon_steps", 0) or 0))
        except (TypeError, ValueError):
            return 0

    def _reservation_safety_time(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.35
        try:
            return max(0.05, float(fleet.get("reservation_safety_time_sec", 0.35) or 0.35))
        except (TypeError, ValueError):
            return 0.35

    def _continuous_collision_step(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.10
        try:
            return max(0.04, min(0.25, float(fleet.get("continuous_collision_step_sec", 0.10) or 0.10)))
        except (TypeError, ValueError):
            return 0.10

    def _batch_collision_horizon(self, final_time: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return max(1.0, final_time)
        raw_value = fleet.get("batch_collision_horizon_sec")
        if raw_value is None:
            return max(1.0, final_time)
        try:
            return max(1.0, float(raw_value))
        except (TypeError, ValueError):
            return max(1.0, final_time)

    def _batch_wait_max_iterations(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 60
        try:
            return max(1, int(fleet.get("batch_wait_max_iterations", 60) or 60))
        except (TypeError, ValueError):
            return 60

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
        if "->" not in edge_id:
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
        for robot in self._runtime_robots():
            if robot.name in request_names:
                continue
            # Moving robots are represented by time-indexed vertex/edge
            # reservations below.  Also turning their current LM into a static
            # obstacle forces needless detours and often a second planner run.
            if robot.status in {"MOVING", "WAITING"} and robot.trajectory:
                continue
            lm_name = self._nearest_lm_for_robot(robot)
            if not lm_name or lm_name in protected_lms:
                continue
            blocked.add(lm_name)
        return blocked

    def _release_blocker_names_for_requests(self, requests: list[dict[str, Any]]) -> set[str]:
        request_names = {
            str(request.get("name", "")).strip()
            for request in requests
            if str(request.get("name", "")).strip()
        }
        if not request_names:
            return set()

        release_owners: set[str] = set()
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.status != "WAITING":
                continue
            if robot.wait_for_robot in request_names:
                release_owners.add(robot.name)
                continue
            reason = str(robot.last_reason or "")
            for request_name in request_names:
                if reason.endswith(request_name) and (
                    reason.startswith("yield to ")
                    or reason.startswith("keep clearance from ")
                ):
                    release_owners.add(robot.name)
                    break
        return release_owners

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
        # Commit a completed background result before deadlock arbitration.
        # Otherwise dispatch immediately occupies the single worker again and
        # a waiting coupled component can starve forever without a CBS slot.
        self._finish_async_simulated_dispatch()
        now = time()
        safety_snapshots = {
            robot.name: self._runtime_safety_snapshot(robot)
            for robot in self._runtime_robots()
            if not robot.is_remote()
        }
        for robot in self._runtime_robots():
            if robot.is_remote():
                self._advance_remote_robot_order(robot, now)
                continue
            route_clock_before = robot.route_clock
            if robot.retreat_target_clock is not None:
                self._advance_deadlock_retreat(robot, now)
                self._update_active_order_from_robot(robot)
                continue
            if (
                robot.status in {"MOVING", "WAITING"}
                and not robot.trajectory
                and not robot.target_lm
            ):
                # A robot without a committed route cannot own right of way.
                # In particular, deadlock recovery may return its order to the
                # background queue while the wait-cycle snapshot still holds a
                # reference to the robot.  Never leave that stale snapshot as
                # a permanent MOVING/WAITING blocker in the traffic graph.
                if robot.active_order_id and self._queue_active_order_for_background_replan(
                    robot,
                    now,
                    "missing active trajectory",
                ):
                    self._update_active_order_from_robot(robot)
                    continue
                robot.status = "BLOCKED" if robot.active_order_id else "IDLE"
                robot.blocked_since = None
                robot.traffic_priority_until = 0.0
                robot.last_reason = (
                    "missing active trajectory"
                    if robot.active_order_id
                    else "idle: no active trajectory"
                )
                robot.last_tick_at = now
                robot.updated_at = now
                self._update_active_order_from_robot(robot)
                continue
            if self._is_deadlock_reason(robot.last_reason) and not robot.trajectory:
                robot.status = "WAITING"
                robot.blocked_since = robot.blocked_since or now
                robot.last_tick_at = now
                robot.updated_at = now
                self._update_active_order_from_robot(robot)
                continue
            if robot.status in {"BLOCKED", "PLANNING"} and robot.target_lm:
                self._schedule_runtime_replan(robot, now, "no active trajectory")
                self._update_active_order_from_robot(robot)
                robot.last_tick_at = now
                continue
            if robot.status not in {"MOVING", "WAITING"}:
                self._update_active_order_from_robot(robot)
                robot.last_tick_at = now
                continue
            if not robot.trajectory:
                if robot.target_lm:
                    self._schedule_runtime_replan(robot, now, "empty trajectory")
                    self._update_active_order_from_robot(robot)
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            last_tick_at = robot.last_tick_at or now
            dt = min(0.20, max(0.0, now - last_tick_at))
            robot.last_tick_at = now
            blocked_reason = ""
            if dt > 0.000001 and robot.route_clock < final_time:
                remaining = dt
                preflight_pending = True
                # Never jump over the interval between websocket ticks.  Each
                # small move is checked at its actual future pose, which also
                # prevents two robots from swapping sides between snapshots.
                while remaining > 0.000001 and robot.route_clock < final_time:
                    motion_dt = min(self._runtime_motion_step(), remaining)
                    proposed_clock = min(final_time, robot.route_clock + motion_dt)
                    # Before moving at all during this physics tick, scan far
                    # enough ahead to stop outside the shared clearance zone.
                    # Subsequent substeps are already covered by that scan and
                    # only need the immediate invariant check.
                    blocked_reason = (
                        self._blocked_ahead(robot, proposed_clock)
                        if preflight_pending
                        else self._blocked_at_clock(robot, proposed_clock)
                    )
                    preflight_pending = False
                    if blocked_reason:
                        break
                    robot.route_clock = proposed_clock
                    remaining -= motion_dt
            if blocked_reason:
                pose = self._pose_at_trajectory(robot.trajectory, robot.route_clock)
                if pose is not None:
                    robot.pose = pose
                self._update_current_lm_from_trajectory(robot)
                if robot.status != "WAITING" or robot.last_reason != blocked_reason:
                    self._event("warn", f"{robot.name} waiting: {blocked_reason}")
                robot.status = "WAITING"
                robot.last_reason = blocked_reason
                self._set_wait_dependency(robot, blocked_reason, now)
                if robot.blocked_since is None:
                    robot.blocked_since = now
                if robot.traffic_stall_since is None:
                    robot.traffic_stall_since = now
                robot.updated_at = now
                if (
                    self._should_replan_for_blocked_reason(blocked_reason)
                    and not self._wait_expected_to_clear(robot)
                    and robot.traffic_stall_since is not None
                    and now - robot.traffic_stall_since >= self._traffic_replan_after()
                ):
                    self._schedule_runtime_replan(robot, now, blocked_reason)
                self._update_active_order_from_robot(robot)
                continue
            pose = self._pose_at_trajectory(robot.trajectory, robot.route_clock)
            if pose is not None:
                robot.pose = pose
            self._update_current_lm_from_trajectory(robot)
            planned_wait_lm = self._planned_wait_lm_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            if planned_wait_lm:
                wait_reason = f"planned traffic wait at {planned_wait_lm}"
                if robot.status != "WAITING" or robot.last_reason != wait_reason:
                    self._event("info", f"{robot.name} {wait_reason}")
                robot.status = "WAITING"
                robot.last_reason = wait_reason
                robot.blocked_since = None
                robot.traffic_stall_since = None
                self._clear_wait_dependency(robot)
            else:
                if robot.status != "MOVING":
                    self._event("info", f"{robot.name} moving")
                robot.status = "MOVING"
                robot.last_reason = "moving"
                robot.blocked_since = None
                self._clear_wait_dependency(robot)
                if robot.route_clock > route_clock_before + 0.000001:
                    self._record_traffic_progress(robot)
            self._update_active_order_from_robot(robot)
            if final_time > 0.0 and robot.route_clock >= final_time:
                robot.current_lm = robot.target_lm or robot.current_lm
                if self._activate_rolling_prefetch(robot, now):
                    continue
                rolling_chunk = self._complete_simulated_route_chunk(robot, now)
                if not rolling_chunk:
                    self._complete_active_order(robot, now)
                robot.target_lm = ""
                robot.status = "IDLE" if rolling_chunk else "ARRIVED"
                robot.trajectory = []
                robot.plan_nodes = []
                robot.trajectory_dirty = True
                robot.route_started_at = None
                robot.route_clock = 0.0
                robot.last_tick_at = None
                robot.blocked_since = None
                robot.last_reason = "rolling horizon reached" if rolling_chunk else "arrived"
                robot.route_note = ""
                robot.updated_at = now
                if not rolling_chunk:
                    self._clear_remote_route_metadata(robot)
                    self._event("info", f"{robot.name} arrived at {robot.current_lm}")
        self._enforce_runtime_safety_invariant(safety_snapshots, now)
        self._resolve_runtime_wait_cycles(now)
        self._dispatch_orders(async_simulated=True)

    def _advance_deadlock_retreat(self, robot: FleetRobot, now: float) -> None:
        target_clock = robot.retreat_target_clock
        if target_clock is None or not robot.trajectory:
            self._clear_deadlock_retreat(robot)
            return
        # Planning can temporarily monopolise the GIL on a dense fleet.  Keep
        # the evacuation lease alive until the graph-safe reverse traversal is
        # actually complete instead of letting it expire in wall-clock time.
        robot.traffic_priority_until = max(
            robot.traffic_priority_until,
            now + self._deadlock_priority_lease(),
        )
        last_tick_at = robot.last_tick_at or now
        dt = min(0.20, max(0.0, now - last_tick_at))
        robot.last_tick_at = now
        remaining = dt
        blocked_reason = ""
        while remaining > 0.000001 and robot.route_clock > target_clock + 0.000001:
            motion_dt = min(self._runtime_motion_step(), remaining)
            proposed_clock = max(target_clock, robot.route_clock - motion_dt)
            blocked_reason = self._blocked_at_clock(robot, proposed_clock)
            if blocked_reason:
                break
            robot.route_clock = proposed_clock
            remaining -= motion_dt

        pose = self._pose_at_trajectory(robot.trajectory, robot.route_clock)
        if pose is not None:
            robot.pose = pose
        self._update_current_lm_from_trajectory(robot)
        robot.updated_at = now
        if blocked_reason:
            robot.status = "RETREATING"
            robot.last_reason = f"deadlock retreat waiting: {blocked_reason}"
            return
        if robot.route_clock > target_clock + 0.000001:
            robot.status = "RETREATING"
            robot.last_reason = f"deadlock retreat to {robot.retreat_target_lm}"
            return

        target_lm = robot.retreat_target_lm
        blocked_edges = list(robot.retreat_blocked_edges)
        if target_lm in self.landmarks:
            robot.current_lm = target_lm
            robot.pose = self._pose_at_landmark(target_lm)
        self._clear_deadlock_retreat(robot)
        order = self._active_order_for_robot(robot)
        if order is not None:
            order.traffic_detour_edges = list(dict.fromkeys(blocked_edges))
            order.traffic_detour_attempts += 1
        if self._queue_active_order_for_background_replan(
            robot,
            now,
            "deadlock corridor evacuated; alternate route required",
        ):
            self.traffic_metrics["cycleReplans"] += 1
            self._event(
                "warn",
                f"{robot.name} retreated to {target_lm}; detour to the same goal queued",
            )
            return
        robot.status = "WAITING"
        robot.last_reason = "deadlock retreat complete; detour queue pending"

    def _clear_deadlock_retreat(self, robot: FleetRobot) -> None:
        robot.retreat_target_clock = None
        robot.retreat_target_lm = ""
        robot.retreat_blocked_edges = []

    def _runtime_safety_snapshot(self, robot: FleetRobot) -> dict[str, Any]:
        active_order = self.orders.get(robot.active_order_id) if robot.active_order_id else None
        return {
            "pose": dict(robot.pose) if robot.pose is not None else None,
            "current_lm": robot.current_lm,
            "target_lm": robot.target_lm,
            "status": robot.status,
            "trajectory": robot.trajectory,
            "plan_nodes": robot.plan_nodes,
            "route_started_at": robot.route_started_at,
            "route_clock": robot.route_clock,
            "last_reason": robot.last_reason,
            "route_note": robot.route_note,
            "blocked_since": robot.blocked_since,
            "last_replan_at": robot.last_replan_at,
            "route_revision": robot.route_revision,
            "route_chunk_index": robot.route_chunk_index,
            "route_chunk_goal_lm": robot.route_chunk_goal_lm,
            "route_final_lm": robot.route_final_lm,
            "route_preview": robot.route_preview,
            "route_preview_dirty": robot.route_preview_dirty,
            "traffic_priority_until": robot.traffic_priority_until,
            "wait_for_robot": robot.wait_for_robot,
            "wait_resource": robot.wait_resource,
            "wait_release_at": robot.wait_release_at,
            "traffic_stall_since": robot.traffic_stall_since,
            "active_order_id": robot.active_order_id,
            "active_order": (
                {
                    "status": active_order.status,
                    "updated_at": active_order.updated_at,
                    "assigned_robot": active_order.assigned_robot,
                    "start_lm": active_order.start_lm,
                    "route_nodes": list(active_order.route_nodes),
                    "error": active_order.error,
                    "target_lm": active_order.target_lm,
                    "step_index": active_order.step_index,
                    "spatial_route_nodes": list(active_order.spatial_route_nodes),
                    "spatial_route_revision": active_order.spatial_route_revision,
                    "traffic_blocked_since": active_order.traffic_blocked_since,
                }
                if active_order is not None
                else None
            ),
        }

    def _restore_runtime_safety_snapshot(
        self,
        robot: FleetRobot,
        snapshot: dict[str, Any],
        now: float,
    ) -> None:
        robot.pose = dict(snapshot["pose"]) if snapshot["pose"] is not None else None
        robot.current_lm = str(snapshot["current_lm"])
        robot.target_lm = str(snapshot["target_lm"])
        robot.status = str(snapshot["status"])
        robot.trajectory = snapshot["trajectory"]
        robot.plan_nodes = snapshot["plan_nodes"]
        robot.route_started_at = snapshot["route_started_at"]
        robot.route_clock = float(snapshot["route_clock"])
        robot.last_reason = str(snapshot["last_reason"])
        robot.route_note = str(snapshot["route_note"])
        robot.blocked_since = snapshot["blocked_since"]
        robot.last_replan_at = snapshot["last_replan_at"]
        robot.route_revision = int(snapshot["route_revision"])
        robot.route_chunk_index = int(snapshot["route_chunk_index"])
        robot.route_chunk_goal_lm = str(snapshot["route_chunk_goal_lm"])
        robot.route_final_lm = str(snapshot["route_final_lm"])
        robot.route_preview = snapshot["route_preview"]
        robot.route_preview_dirty = bool(snapshot["route_preview_dirty"])
        robot.traffic_priority_until = float(snapshot["traffic_priority_until"])
        robot.wait_for_robot = str(snapshot.get("wait_for_robot") or "")
        robot.wait_resource = str(snapshot.get("wait_resource") or "")
        robot.wait_release_at = float(snapshot.get("wait_release_at", 0.0) or 0.0)
        robot.traffic_stall_since = snapshot.get("traffic_stall_since")
        robot.active_order_id = str(snapshot["active_order_id"])
        order_snapshot = snapshot.get("active_order")
        order = self.orders.get(robot.active_order_id) if robot.active_order_id else None
        if order is not None and isinstance(order_snapshot, dict):
            order.status = str(order_snapshot["status"])
            order.updated_at = float(order_snapshot["updated_at"])
            order.assigned_robot = str(order_snapshot["assigned_robot"])
            order.start_lm = str(order_snapshot["start_lm"])
            order.route_nodes = list(order_snapshot["route_nodes"])
            order.error = str(order_snapshot["error"])
            order.target_lm = str(order_snapshot["target_lm"])
            order.step_index = int(order_snapshot["step_index"])
            order.spatial_route_nodes = list(order_snapshot.get("spatial_route_nodes", []))
            order.spatial_route_revision = int(
                order_snapshot.get("spatial_route_revision", 0) or 0
            )
            order.traffic_blocked_since = order_snapshot.get("traffic_blocked_since")
        robot.last_tick_at = now
        robot.trajectory_dirty = True
        robot.updated_at = now

    def _enforce_runtime_safety_invariant(
        self,
        snapshots: dict[str, dict[str, Any]],
        now: float,
    ) -> None:
        robots = [
            robot for robot in self._runtime_robots()
            if not robot.is_remote() and robot.pose is not None and robot.name in snapshots
        ]
        unsafe_names: set[str] = set()
        unsafe_pairs: list[tuple[str, str]] = []
        for index, robot in enumerate(robots):
            for other in robots[index + 1:]:
                previous_robot_pose = snapshots[robot.name].get("pose")
                previous_other_pose = snapshots[other.name].get("pose")
                if (
                    not self.collision.footprints_overlap(robot.pose, other.pose)
                    and not self._swept_footprints_overlap(
                        previous_robot_pose,
                        robot.pose,
                        previous_other_pose,
                        other.pose,
                    )
                ):
                    continue
                if (
                    previous_robot_pose is not None
                    and previous_other_pose is not None
                    and self.collision.footprints_overlap(
                        previous_robot_pose,
                        previous_other_pose,
                    )
                ):
                    # The invariant cannot repair an overlap that existed
                    # before this tick; do not create an endless rollback loop.
                    continue
                unsafe_names.update((robot.name, other.name))
                unsafe_pairs.append((robot.name, other.name))
        if not unsafe_names:
            return

        involved = [self.robots[name] for name in sorted(unsafe_names)]
        for robot in involved:
            self._restore_runtime_safety_snapshot(robot, snapshots[robot.name], now)

        def priority_key(robot: FleetRobot) -> tuple[int, str]:
            order = self._active_order_for_robot(robot)
            return (-int(order.priority if order is not None else 0), robot.name)

        winner = min(involved, key=priority_key)
        winner.traffic_priority_until = now + self._deadlock_priority_lease()
        winner.status = "MOVING" if winner.trajectory else "WAITING"
        winner.last_reason = "runtime safety rollback; priority granted"
        winner.blocked_since = now
        winner.traffic_stall_since = winner.traffic_stall_since or now
        for robot in involved:
            if robot.name == winner.name:
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {winner.name}"
            robot.blocked_since = now
            robot.traffic_stall_since = robot.traffic_stall_since or now
            self._update_active_order_from_robot(robot)
        self._update_active_order_from_robot(winner)
        self.traffic_metrics["runtimeSafetyRollbacks"] += 1
        pairs = ", ".join(f"{first}/{second}" for first, second in unsafe_pairs)
        self._event(
            "error",
            f"runtime safety invariant prevented footprint overlap: {pairs}; "
            f"rolled back, priority {winner.name}",
        )

    def _swept_footprints_overlap(
        self,
        first_start: dict[str, float] | None,
        first_end: dict[str, float],
        second_start: dict[str, float] | None,
        second_end: dict[str, float],
    ) -> bool:
        if first_start is None or second_start is None:
            return False

        relative_x = float(first_start.get("x", 0.0) or 0.0) - float(second_start.get("x", 0.0) or 0.0)
        relative_y = float(first_start.get("y", 0.0) or 0.0) - float(second_start.get("y", 0.0) or 0.0)
        relative_dx = (
            float(first_end.get("x", 0.0) or 0.0) - float(first_start.get("x", 0.0) or 0.0)
            - float(second_end.get("x", 0.0) or 0.0) + float(second_start.get("x", 0.0) or 0.0)
        )
        relative_dy = (
            float(first_end.get("y", 0.0) or 0.0) - float(first_start.get("y", 0.0) or 0.0)
            - float(second_end.get("y", 0.0) or 0.0) + float(second_start.get("y", 0.0) or 0.0)
        )
        relative_speed_sq = (relative_dx * relative_dx) + (relative_dy * relative_dy)
        closest_ratio = 0.0
        if relative_speed_sq > 0.000000001:
            closest_ratio = max(
                0.0,
                min(1.0, -((relative_x * relative_dx) + (relative_y * relative_dy)) / relative_speed_sq),
            )
        closest_distance = math.hypot(
            relative_x + (relative_dx * closest_ratio),
            relative_y + (relative_dy * closest_ratio),
        )
        if closest_distance > self.collision.robot_broadphase_distance():
            return False

        first_travel = math.hypot(
            float(first_end.get("x", 0.0) or 0.0) - float(first_start.get("x", 0.0) or 0.0),
            float(first_end.get("y", 0.0) or 0.0) - float(first_start.get("y", 0.0) or 0.0),
        )
        second_travel = math.hypot(
            float(second_end.get("x", 0.0) or 0.0) - float(second_start.get("x", 0.0) or 0.0),
            float(second_end.get("y", 0.0) or 0.0) - float(second_start.get("y", 0.0) or 0.0),
        )
        first_turn = abs(math.atan2(
            math.sin(float(first_end.get("yaw", 0.0) or 0.0) - float(first_start.get("yaw", 0.0) or 0.0)),
            math.cos(float(first_end.get("yaw", 0.0) or 0.0) - float(first_start.get("yaw", 0.0) or 0.0)),
        ))
        second_turn = abs(math.atan2(
            math.sin(float(second_end.get("yaw", 0.0) or 0.0) - float(second_start.get("yaw", 0.0) or 0.0)),
            math.cos(float(second_end.get("yaw", 0.0) or 0.0) - float(second_start.get("yaw", 0.0) or 0.0)),
        ))
        linear_samples = int(math.ceil(max(first_travel, second_travel) / 0.025))
        angular_samples = int(math.ceil(max(first_turn, second_turn) / 0.05))
        samples = max(2, min(40, max(linear_samples, angular_samples)))
        for index in range(1, samples):
            ratio = index / samples
            first_pose = self._interpolate_pose(first_start, first_end, ratio)
            second_pose = self._interpolate_pose(second_start, second_end, ratio)
            if self.collision.footprints_overlap(first_pose, second_pose):
                return True
        return False

    def _interpolate_pose(
        self,
        start: dict[str, float],
        end: dict[str, float],
        ratio: float,
    ) -> dict[str, float]:
        return {
            "x": float(start.get("x", 0.0) or 0.0)
            + ((float(end.get("x", 0.0) or 0.0) - float(start.get("x", 0.0) or 0.0)) * ratio),
            "y": float(start.get("y", 0.0) or 0.0)
            + ((float(end.get("y", 0.0) or 0.0) - float(start.get("y", 0.0) or 0.0)) * ratio),
            "yaw": self._interpolate_angle(
                float(start.get("yaw", 0.0) or 0.0),
                float(end.get("yaw", 0.0) or 0.0),
                ratio,
            ),
        }

    def _runtime_motion_step(self) -> float:
        return max(0.02, min(0.05, self._continuous_collision_step() / 2.0))

    def _schedule_runtime_replan(self, robot: FleetRobot, now: float, reason: str) -> bool:
        if self._reason_requires_spatial_replan(reason):
            order = self._active_order_for_robot(robot)
            start_lm = self._safe_replan_start_lm(robot)
            if order is not None and start_lm:
                self._queue_alternate_corridor_detour(
                    order,
                    start_lm,
                    self._active_order_target(order),
                )
        if self._queue_active_order_for_background_replan(robot, now, reason):
            return True
        # Manual/ad-hoc routes have no order that can be returned to the
        # dispatcher, so retain the synchronous compatibility path for those
        # uncommon requests only.
        return self._maybe_replan_robot(robot, now, reason)

    def _queue_active_order_for_background_replan(
        self,
        robot: FleetRobot,
        now: float,
        reason: str,
    ) -> bool:
        if robot.is_remote() or not robot.active_order_id:
            return False
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return False
        start_lm = self._safe_replan_start_lm(robot)
        if not start_lm:
            return False

        order.status = "QUEUED"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = start_lm
        order.route_nodes = list(robot.plan_nodes)
        if self._reason_requires_spatial_replan(reason):
            order.spatial_route_nodes = []
            order.traffic_blocked_since = now
        robot.current_lm = start_lm
        robot.active_order_id = ""
        robot.target_lm = ""
        robot.status = "IDLE"
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.last_tick_at = now
        robot.blocked_since = None
        robot.traffic_stall_since = None
        robot.traffic_priority_until = 0.0
        robot.last_replan_at = now
        robot.last_reason = f"background replan queued: {reason}"
        robot.route_note = ""
        robot.updated_at = now
        self._event("warn", f"{robot.name} background replan queued: {reason}")
        return True

    def _reason_requires_spatial_replan(self, reason: str) -> bool:
        value = str(reason or "").lower()
        return bool(
            self._is_parked_robot_conflict(reason)
            or "alternate route required" in value
            or "traffic wait timeout" in value
            or "obstacle" in value
            or "blocked edge" in value
        )

    def _maybe_replan_robot(self, robot: FleetRobot, now: float, reason: str) -> bool:
        if not robot.target_lm:
            return False
        interval = self._replan_interval()
        if robot.last_replan_at is not None and now - robot.last_replan_at < interval:
            return False

        start_lm = self._safe_replan_start_lm(robot)
        if not start_lm or start_lm not in self.landmarks:
            already_deferred = robot.status == "WAITING" and robot.last_reason == reason
            robot.last_replan_at = now
            robot.status = "WAITING" if robot.trajectory else "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = now
            if robot.trajectory and not already_deferred:
                self._event(
                    "info",
                    f"{robot.name} replan deferred until the next LM (holding current edge)",
                )
            return False

        robot.last_replan_at = now
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
            plan_note = self._plan_note(result)
            if (
                plan_note == "FALLBACK_WAIT"
                and robot.trajectory
                and self._is_parked_robot_conflict(reason)
            ):
                robot.status = "WAITING"
                robot.last_reason = reason
                robot.updated_at = now
                self._event("warn", f"{robot.name} replan pending: no detour around parked robot")
                return False
            self._apply_planner_result(result, now)
            robot.route_note = f"REPLAN: {plan_note}"
            robot.last_reason = robot.route_note
            self._event("info", f"{robot.name} replanned after block: {reason}")
            return True

        robot.status = "WAITING" if robot.trajectory else "BLOCKED"
        if self._is_parked_robot_conflict(reason):
            robot.last_reason = reason
        else:
            robot.last_reason = result.get("debug", {}).get("reason", reason)
        robot.updated_at = now
        self._event("warn", f"{robot.name} replan pending: {robot.last_reason}")
        return False

    def _maybe_replan_remote_robot_order(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
        reason: str,
    ) -> bool:
        if not robot.is_remote() or not robot.base_url:
            return False
        target_lm = self._active_order_target(order)
        if not target_lm:
            return False
        interval = self._replan_interval()
        if robot.last_replan_at is not None and now - robot.last_replan_at < interval:
            return False

        start_lm = self._safe_replan_start_lm(robot)
        if not start_lm or start_lm not in self.landmarks:
            robot.last_replan_at = now
            robot.status = "WAITING" if robot.trajectory else "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = now
            return False

        robot.last_replan_at = now
        request = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": target_lm,
            "startPose": dict(robot.pose) if robot.pose else self._pose_at_landmark(start_lm),
        }
        result = self._plan_valid_requests([request], {"robots": [request]})
        if not result.get("ok") or not result.get("plans"):
            robot.status = "WAITING" if robot.trajectory else "BLOCKED"
            debug = result.get("debug", {})
            if isinstance(debug, dict):
                robot.last_reason = str(debug.get("reason") or reason)
            else:
                robot.last_reason = reason
            robot.updated_at = now
            self._event("warn", f"{robot.name} remote replan pending: {robot.last_reason}")
            return False

        plan = self._plan_for_robot(result, robot.name)
        if plan is None:
            robot.status = "WAITING"
            robot.last_reason = "remote replan did not return robot plan"
            robot.updated_at = now
            return False

        try:
            remote_route = self._execute_remote_plan(robot, order, plan, result)
        except Exception as exc:
            robot.remote_error = str(exc)
            robot.remote_online = False
            robot.status = "OFFLINE"
            robot.last_reason = f"remote replan execute failed: {exc}"
            robot.updated_at = now
            return False

        order.route_nodes = [str(item) for item in plan.get("nodes", []) if str(item)]
        self._apply_planner_result(result, now, order_id=order.order_id)
        self._apply_remote_route_metadata(robot, remote_route, now)
        self._set_order_status(order, "EXECUTING", robot=robot, start_lm=start_lm)
        robot.route_note = f"REPLAN: {self._plan_note(result)}"
        robot.last_reason = robot.route_note
        self._event("info", f"{robot.name} remote route revision {robot.route_revision}: {reason}")
        return True

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

    def _safe_replan_start_lm(self, robot: FleetRobot) -> str:
        if robot.pose is None:
            return robot.current_lm if robot.current_lm in self.landmarks else ""
        nearest_lm = self._nearest_lm_for_robot(robot)
        landmark = self.landmarks.get(nearest_lm)
        if landmark is None:
            return ""
        distance = math.hypot(
            landmark.x - float(robot.pose.get("x", 0.0) or 0.0),
            landmark.y - float(robot.pose.get("y", 0.0) or 0.0),
        )
        if distance > self._runtime_replan_lm_tolerance():
            return ""
        return nearest_lm

    def _pose_is_at_lm(self, pose: dict[str, Any], lm_name: str) -> bool:
        landmark = self.landmarks.get(lm_name)
        if landmark is None:
            return False
        return math.hypot(
            landmark.x - float(pose.get("x", 0.0) or 0.0),
            landmark.y - float(pose.get("y", 0.0) or 0.0),
        ) <= self._runtime_replan_lm_tolerance()

    def _update_current_lm_from_trajectory(self, robot: FleetRobot) -> None:
        latest_lm = ""
        for sample in robot.trajectory:
            sample_time = float(sample.get("t", 0.0) or 0.0)
            if sample_time > robot.route_clock + 0.000001:
                break
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name in self.landmarks:
                latest_lm = lm_name
        if latest_lm:
            robot.current_lm = latest_lm

    def _planned_wait_lm_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> str:
        for index in range(len(trajectory) - 1):
            start = trajectory[index]
            end = trajectory[index + 1]
            start_time = float(start.get("t", 0.0) or 0.0)
            end_time = float(end.get("t", start_time) or start_time)
            if end_time <= start_time or not (start_time <= elapsed < end_time):
                continue
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            if edge_id.startswith("WAIT@ROTATE:"):
                return ""
            if edge_id.startswith("WAIT@"):
                return self._lm_from_wait_segment(start, end)
            if "->" not in edge_id:
                return ""
            source, target = (value.strip() for value in edge_id.split("->", 1))
            if source == target and source in self.landmarks:
                return source
            return ""
        return ""

    def _runtime_replan_lm_tolerance(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.10
        try:
            return max(
                0.03,
                float(fleet.get("runtime_replan_lm_tolerance_m", 0.10) or 0.10),
            )
        except (TypeError, ValueError):
            return 0.10

    def _resolve_runtime_wait_cycles(self, now: float) -> None:
        waiting = {
            robot.name: robot
            for robot in self._runtime_robots()
            if (
                robot.status == "WAITING"
                and bool(robot.trajectory)
                and self._is_robot_conflict(robot.last_reason)
            )
        }
        wait_for = {
            name: robot.wait_for_robot or self._robot_name_from_conflict_reason(robot.last_reason)
            for name, robot in waiting.items()
        }
        handled: set[str] = set()
        cycle_members: set[str] = set()
        for start_name in sorted(wait_for):
            chain: list[str] = []
            positions: dict[str, int] = {}
            current = start_name
            while current in wait_for and current not in handled:
                if current in positions:
                    cycle = chain[positions[current]:]
                    cycle_key = tuple(sorted(cycle))
                    new_episode = cycle_key not in self._active_wait_cycles
                    if new_episode:
                        self._active_wait_cycles[cycle_key] = min(
                            (
                                robot.traffic_stall_since
                                or robot.blocked_since
                                or now
                                for robot in (
                                    waiting[name]
                                    for name in cycle
                                    if name in waiting
                                )
                            ),
                            default=now,
                        )
                        self.traffic_metrics["waitCyclesDetected"] += 1
                    if not self._maintain_runtime_wait_cycle_lease(cycle, waiting, now):
                        self._break_runtime_wait_cycle(
                            cycle,
                            waiting,
                            now,
                            new_episode=new_episode,
                        )
                    cycle_members.update(cycle)
                    handled.update(cycle)
                    break
                positions[current] = len(chain)
                chain.append(current)
                current = wait_for[current]
            handled.update(chain)

        timeout = self._traffic_replan_after()
        for robot in waiting.values():
            if robot.name in cycle_members:
                continue
            if robot.blocked_since is None or now - robot.blocked_since < timeout:
                continue
            if self._wait_expected_to_clear(robot):
                continue
            if self._safe_replan_start_lm(robot):
                self._schedule_runtime_replan(robot, now, "traffic wait timeout")

    def _record_traffic_progress(self, robot: FleetRobot) -> None:
        robot.traffic_stall_since = None
        for cycle_key in list(self._active_wait_cycles):
            if robot.name not in cycle_key:
                continue
            self._active_wait_cycles.pop(cycle_key, None)
            self._coupled_replan_failures.pop(cycle_key, None)

    def _break_runtime_wait_cycle(
        self,
        cycle: list[str],
        waiting: dict[str, FleetRobot],
        now: float,
        *,
        new_episode: bool = True,
    ) -> None:
        robots = [waiting[name] for name in cycle if name in waiting]
        if len(robots) < 2:
            return

        def priority_key(robot: FleetRobot) -> tuple[int, float, float, str]:
            order = self._active_order_for_robot(robot)
            priority = int(order.priority if order is not None else 0)
            waited = now - (robot.blocked_since or now)
            # If a lease did not create progress, rotate equal-priority cycles
            # instead of granting the same robot forever.
            return (-priority, robot.traffic_priority_until, -waited, robot.name)

        winner = min(robots, key=priority_key)
        cycle_key = tuple(sorted(robot.name for robot in robots))
        cycle_started = self._active_wait_cycles.get(
            cycle_key,
            min(
                (robot.traffic_stall_since or robot.blocked_since or now for robot in robots),
                default=now,
            ),
        )
        cycle_wait = max(0.0, now - cycle_started)
        for robot in robots:
            robot.traffic_stall_since = robot.traffic_stall_since or cycle_started
        if cycle_wait >= self._deadlock_coupled_replan_after():
            self._start_async_coupled_replan(robots, winner, now)
        # Retreat changes the spatial route and is intentionally the final
        # recovery level. Give dependency ordering and at least one local CBS
        # attempt time to resolve the coupled component first.
        evacuating_name = ""
        if (
            cycle_wait >= self._deadlock_retreat_after()
            and self._coupled_replan_failures.get(cycle_key, 0) > 0
        ):
            evacuating_name = self._start_deadlock_corridor_evacuation(
                robots,
                winner,
                now,
            )
        # Breaking a wait-for cycle must be immediate and deterministic.  A
        # synchronous joint replan here used to freeze the whole status stream
        # while every robot was already stopped.  The winner receives a short
        # lease; any later rolling replan is handled by the background queue.
        lease_until = now + self._deadlock_priority_lease()
        evacuating_robot = self.robots.get(evacuating_name) if evacuating_name else None
        # An at-LM evacuation queues a background detour immediately, which
        # intentionally clears that robot's trajectory and changes it to IDLE.
        # It must not be changed back to MOVING merely because this resolver is
        # still operating on the old wait-cycle snapshot.
        priority_robot = (
            evacuating_robot
            if (
                evacuating_robot is not None
                and evacuating_robot.status == "RETREATING"
                and bool(evacuating_robot.trajectory)
            )
            else winner
        )
        if not priority_robot.trajectory:
            eligible = [robot for robot in robots if robot.trajectory]
            if not eligible:
                return
            priority_robot = min(eligible, key=priority_key)
        priority_robot.traffic_priority_until = max(
            priority_robot.traffic_priority_until,
            lease_until,
        )
        if priority_robot.status != "RETREATING":
            priority_robot.status = "MOVING"
            priority_robot.last_reason = "deadlock priority granted"
        self._clear_wait_dependency(priority_robot)
        priority_robot.blocked_since = priority_robot.blocked_since or now
        priority_robot.updated_at = now
        for robot in robots:
            if robot.name == priority_robot.name:
                continue
            if not robot.trajectory:
                # Preserve IDLE/background-replan state established during
                # corridor evacuation; it is no longer part of this cycle.
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {priority_robot.name}"
            robot.wait_for_robot = priority_robot.name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = lease_until
            robot.blocked_since = robot.blocked_since or now
            robot.updated_at = now
        if new_episode:
            # This records one arbitration episode. Actual spatial progress
            # closes the episode in _record_traffic_progress().
            self.traffic_metrics["waitCyclesResolved"] += 1
        self.traffic_metrics["priorityGrants"] += 1
        self._event(
            "warn",
            f"traffic wait cycle resolved: priority granted to {priority_robot.name}",
        )

    def _start_deadlock_corridor_evacuation(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> str:
        candidates: list[tuple[float, int, str, FleetRobot, float, str]] = []
        for robot in robots:
            if robot.name == winner.name or not robot.trajectory or not robot.active_order_id:
                continue
            retreat = self._previous_trajectory_lm(robot)
            if retreat is None:
                continue
            target_clock, target_lm = retreat
            if self._deadlock_retreat_target_blocker(robot, target_clock):
                # Reversing toward an occupied landmark cannot evacuate the
                # corridor and creates a permanent RETREATING state. Try a
                # different member of the cycle; if none is clear, grant the
                # forward winner the lease and retry on a later tick.
                continue
            order = self._active_order_for_robot(robot)
            priority = int(order.priority if order is not None else 0)
            distance = max(0.0, robot.route_clock - target_clock)
            candidates.append((distance, priority, robot.name, robot, target_clock, target_lm))
        if not candidates:
            return ""

        _, _, _, robot, target_clock, target_lm = min(candidates)
        blocked_edges = self._deadlock_detour_edges(robot)
        if not blocked_edges:
            return ""
        if abs(robot.route_clock - target_clock) <= 0.000001:
            order = self._active_order_for_robot(robot)
            if order is None:
                return ""
            order.traffic_detour_edges = blocked_edges
            order.traffic_detour_attempts += 1
            if self._queue_active_order_for_background_replan(
                robot,
                now,
                "deadlock at LM; alternate corridor required",
            ):
                self.traffic_metrics["cycleReplans"] += 1
                self._event(
                    "warn",
                    f"{robot.name}@{target_lm} queued for alternate route to the same goal",
                )
                return robot.name
            return ""

        robot.pending_route = None
        robot.retreat_target_clock = target_clock
        robot.retreat_target_lm = target_lm
        robot.retreat_blocked_edges = blocked_edges
        robot.status = "RETREATING"
        robot.last_reason = f"deadlock retreat to {target_lm} before detour"
        robot.last_tick_at = now
        robot.updated_at = now
        self._event(
            "warn",
            f"{robot.name} evacuating narrow corridor back to {target_lm}",
        )
        return robot.name

    def _deadlock_retreat_target_blocker(
        self,
        robot: FleetRobot,
        target_clock: float,
    ) -> str:
        target_pose = self._pose_at_trajectory(robot.trajectory, target_clock)
        if target_pose is None:
            return "invalid retreat pose"
        for other in self._runtime_robots():
            if other.name == robot.name or other.pose is None:
                continue
            if self.collision.robot_footprints_conflict(target_pose, other.pose):
                return other.name
        return ""

    def _previous_trajectory_lm(
        self,
        robot: FleetRobot,
    ) -> tuple[float, str] | None:
        candidate: tuple[float, str] | None = None
        for sample in robot.trajectory:
            sample_time = float(sample.get("t", 0.0) or 0.0)
            if sample_time > robot.route_clock + 0.000001:
                break
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name not in self.landmarks:
                continue
            candidate = (sample_time, lm_name)
        return candidate

    def _deadlock_detour_edges(self, robot: FleetRobot) -> list[tuple[str, str]]:
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, robot.route_clock)
        )
        if edge is None:
            for sample in robot.trajectory:
                if float(sample.get("t", 0.0) or 0.0) + 0.000001 < robot.route_clock:
                    continue
                edge = self._parse_edge_id(str(sample.get("edgeId") or ""))
                if edge is not None:
                    break
        if edge is None:
            return []
        src, dst = edge
        return [(src, dst), (dst, src)]

    def _maintain_runtime_wait_cycle_lease(
        self,
        cycle: list[str],
        waiting: dict[str, FleetRobot],
        now: float,
    ) -> bool:
        robots = [waiting[name] for name in cycle if name in waiting]
        leased = [robot for robot in robots if robot.traffic_priority_until > now]
        if not leased:
            return False
        winner = max(
            leased,
            key=lambda robot: (robot.traffic_priority_until, robot.name),
        )
        # The runtime collision check may mark the winner WAITING again during
        # the same geometric encounter.  Reassert the existing lease without
        # counting/resolving the identical cycle at every 10 Hz physics tick.
        winner.status = "MOVING" if winner.trajectory else "WAITING"
        winner.last_reason = "deadlock priority active"
        self._clear_wait_dependency(winner)
        winner.updated_at = now
        for robot in robots:
            if robot.name == winner.name:
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {winner.name}"
            robot.wait_for_robot = winner.name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = winner.traffic_priority_until
            robot.blocked_since = robot.blocked_since or now
            robot.updated_at = now
        return True

    def _deadlock_priority_lease(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 2.5
        try:
            return max(0.5, float(fleet.get("deadlock_priority_lease_sec", 2.5) or 2.5))
        except (TypeError, ValueError):
            return 2.5

    def _deadlock_wait_timeout(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 3.0

    def _traffic_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 8.0
        try:
            return max(
                self._deadlock_wait_timeout(),
                float(fleet.get("traffic_replan_after_sec", 8.0) or 8.0),
            )
        except (TypeError, ValueError):
            return 8.0
        try:
            return max(1.0, float(fleet.get("deadlock_wait_timeout_sec", 3.0) or 3.0))
        except (TypeError, ValueError):
            return 3.0

    def _deadlock_coupled_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 5.0
        try:
            return max(
                self._deadlock_wait_timeout(),
                float(fleet.get("deadlock_coupled_replan_after_sec", 5.0) or 5.0),
            )
        except (TypeError, ValueError):
            return 5.0

    def _deadlock_coupled_replan_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 4.0
        try:
            return max(
                1.0,
                float(fleet.get("deadlock_coupled_replan_interval_sec", 4.0) or 4.0),
            )
        except (TypeError, ValueError):
            return 4.0

    def _deadlock_retreat_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 12.0
        try:
            return max(
                self._deadlock_coupled_replan_after(),
                float(fleet.get("deadlock_retreat_after_sec", 12.0) or 12.0),
            )
        except (TypeError, ValueError):
            return 12.0

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
        robot_candidates = self._lookahead_robot_candidates(robot, lookahead)
        relative_speed = self._runtime_robot_speed(robot) + max(
            (self._runtime_robot_speed(other) for other in robot_candidates),
            default=0.0,
        )
        broadphase = self.collision.robot_broadphase_distance()
        safe_step = (
            broadphase / relative_speed
            if relative_speed > 0.000001
            else lookahead
        )
        # Bound far-horizon work to roughly ten samples while keeping relative
        # travel below one clearance radius between samples.
        step = min(
            max(self.collision.sample_time_step(), lookahead / 10.0),
            max(self.collision.sample_time_step(), safe_step),
        )
        end_clock = min(final_time, proposed_clock + lookahead)
        checks = [proposed_clock]
        clock = proposed_clock + step
        while clock <= end_clock + 0.000001:
            checks.append(clock)
            clock += step
        for check_clock in checks:
            reason = self._blocked_at_clock(
                robot,
                check_clock,
                robot_candidates=robot_candidates,
            )
            if reason:
                return reason
        return ""

    def _lookahead_robot_candidates(
        self,
        robot: FleetRobot,
        lookahead: float,
    ) -> list[FleetRobot]:
        if robot.pose is None:
            return []
        robot_speed = self._runtime_robot_speed(robot)
        broadphase = self.collision.robot_broadphase_distance()
        candidates: list[FleetRobot] = []
        for other in self._runtime_robots():
            if other.name == robot.name or other.pose is None:
                continue
            other_speed = self._runtime_robot_speed(other) if other.status == "MOVING" else 0.0
            reachable_distance = broadphase + ((robot_speed + other_speed) * lookahead) + 0.05
            center_distance = math.hypot(
                float(robot.pose.get("x", 0.0) or 0.0)
                - float(other.pose.get("x", 0.0) or 0.0),
                float(robot.pose.get("y", 0.0) or 0.0)
                - float(other.pose.get("y", 0.0) or 0.0),
            )
            if center_distance <= reachable_distance:
                candidates.append(other)
        return candidates

    def _runtime_robot_speed(self, robot: FleetRobot) -> float:
        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        try:
            speed = max(0.05, float(navigation.get("route_speed", 0.35) or 0.35))
        except (TypeError, ValueError):
            speed = 0.35
        order = self._active_order_for_robot(robot)
        if order is not None and order.speed > 0.0:
            speed = max(speed, float(order.speed))
        return speed

    def _blocked_at_clock(
        self,
        robot: FleetRobot,
        check_clock: float,
        robot_candidates: list[FleetRobot] | None = None,
    ) -> str:
        pose = self._pose_at_trajectory(robot.trajectory, check_clock)
        if pose is None:
            return ""
        offset = max(0.0, check_clock - robot.route_clock)
        future_prediction = offset > self._runtime_motion_step() + 0.000001
        if future_prediction:
            reason = (
                self.collision.dynamic_blocked_reason(
                    pose=pose,
                    obstacles=self.obstacles,
                    obstacle_areas=self.obstacle_areas,
                )
                if self.obstacles or self.obstacle_areas
                else ""
            )
        else:
            # Static map occupancy is checked on the immediate motion step.
            # Graph trajectories were already audited against the map, so
            # repeating the expensive footprint raster scan at every distant
            # lookahead sample adds CPU load without adding safety.
            reason = self.collision.blocked_reason(
                pose=pose,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
        if reason:
            return reason
        others = robot_candidates if robot_candidates is not None else self._runtime_robots()
        for other in others:
            if other.name == robot.name or other.pose is None:
                continue
            other_pose = self._predicted_robot_pose(other, offset)
            if other_pose is None:
                continue
            if future_prediction:
                center_distance = math.hypot(
                    float(pose.get("x", 0.0) or 0.0)
                    - float(other_pose.get("x", 0.0) or 0.0),
                    float(pose.get("y", 0.0) or 0.0)
                    - float(other_pose.get("y", 0.0) or 0.0),
                )
                if center_distance > self.collision.robot_broadphase_distance():
                    continue
                # The circumscribed circle is reject-only. Adjacent graph
                # lanes can be closer than that circle while the oriented
                # rectangular bodies (including their traffic margin) remain
                # disjoint. Treating broadphase as a collision authority was
                # the main source of fleet-wide false waits.
                if not self.collision.robot_footprints_conflict(pose, other_pose):
                    continue
                reason = self._robot_conflict_reason(
                    robot,
                    other,
                    pose,
                    other_pose,
                    prediction_offset=offset,
                )
                if reason:
                    return reason
                continue
            if not self.collision.robot_footprints_conflict(pose, other_pose):
                continue
            reason = self._robot_conflict_reason(
                robot,
                other,
                pose,
                other_pose,
                prediction_offset=offset,
            )
            if reason:
                return reason
        return ""

    def _predicted_robot_pose(self, robot: FleetRobot, offset: float) -> dict[str, float] | None:
        if (
            robot.status == "RETREATING"
            and robot.trajectory
            and robot.retreat_target_clock is not None
        ):
            return self._pose_at_trajectory(
                robot.trajectory,
                max(
                    robot.retreat_target_clock,
                    robot.route_clock - max(0.0, offset),
                ),
            )
        follows_committed_timeline = (
            robot.status == "MOVING"
            or (
                robot.status == "WAITING"
                and str(robot.last_reason).startswith("planned traffic wait")
            )
        )
        if not follows_committed_timeline or not robot.trajectory:
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
        prediction_offset: float = 0.0,
    ) -> str:
        other_current_is_authoritative = (
            other.status != "MOVING"
            or prediction_offset <= self._runtime_motion_step() + 0.000001
        )
        if (
            other_current_is_authoritative
            and other.pose is not None
            and self.collision.footprints_overlap(candidate_pose, other.pose)
        ):
            return f"occupied by {other.name}"
        if self.collision.footprints_overlap(candidate_pose, other_pose):
            # This is an anticipated collision, not an overlap that already
            # exists. Resolve it before motion using deterministic right of
            # way: the loser waits, while the winner's future prediction is
            # evaluated against the loser's stationary pose on later ticks.
            if self._has_right_of_way(robot, other):
                return ""
            if self._is_active_traffic(other):
                return f"yield to {other.name}"
            return f"occupied by {other.name}"
        if robot.pose is not None and self._candidate_stays_put(robot.pose, candidate_pose):
            return ""
        if (
            robot.status == "RETREATING"
            and robot.traffic_priority_until > time()
            and self._is_active_traffic(other)
        ):
            # The evacuation robot owns the corridor until it reaches the
            # previous LM.  Physical footprint overlap was checked above;
            # a soft clearance envelope must not recreate the same deadlock.
            return ""
        if (
            robot.status == "RETREATING"
            and other.pose is not None
            and self._candidate_moves_away(robot.pose, candidate_pose, other.pose)
        ):
            return ""

        if other.pose is not None and self.collision.robot_footprints_conflict(candidate_pose, other.pose):
            if self._has_right_of_way(robot, other):
                return ""
            if self._is_active_traffic(other):
                return f"yield to {other.name}"
            return f"keep clearance from {other.name}"

        if self._has_right_of_way(robot, other):
            return ""
        if self._is_active_traffic(other):
            return f"yield to {other.name}"
        return f"keep clearance from {other.name}"

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

    def _candidate_stays_put(
        self,
        current_pose: dict[str, float],
        candidate_pose: dict[str, float],
    ) -> bool:
        return math.hypot(
            float(current_pose.get("x", 0.0) or 0.0) - float(candidate_pose.get("x", 0.0) or 0.0),
            float(current_pose.get("y", 0.0) or 0.0) - float(candidate_pose.get("y", 0.0) or 0.0),
        ) < 0.005

    def _has_right_of_way(self, robot: FleetRobot, other: FleetRobot) -> bool:
        if not self._is_active_traffic(robot):
            return False
        if not self._is_active_traffic(other):
            return False
        now = time()
        robot_lease = robot.traffic_priority_until > now
        other_lease = other.traffic_priority_until > now
        if robot_lease != other_lease:
            return robot_lease
        if self._is_yielding_to(other, robot):
            return True
        if self._is_yielding_to(robot, other):
            return False

        robot_order = self._active_order_for_robot(robot)
        other_order = self._active_order_for_robot(other)
        robot_priority = int(robot_order.priority if robot_order is not None else 0)
        other_priority = int(other_order.priority if other_order is not None else 0)
        if robot_priority != other_priority:
            return robot_priority > other_priority

        if robot.status != other.status:
            if robot.status == "MOVING":
                return True
            if other.status == "MOVING":
                return False

        robot_started = robot.route_started_at or robot.updated_at
        other_started = other.route_started_at or other.updated_at
        if abs(robot_started - other_started) > 0.001:
            return robot_started < other_started
        return robot.name < other.name

    def _is_active_traffic(self, robot: FleetRobot) -> bool:
        return bool(
            robot.active_order_id
            or (robot.target_lm and robot.trajectory)
            or robot.status in {"MOVING", "WAITING", "PLANNING", "RETREATING"}
        )

    def _is_yielding_to(self, robot: FleetRobot, other: FleetRobot) -> bool:
        if robot.status != "WAITING":
            return False
        if robot.wait_for_robot:
            return robot.wait_for_robot == other.name
        reason = str(robot.last_reason or "")
        return reason.endswith(other.name) and (
            reason.startswith("yield to ")
            or reason.startswith("keep clearance from ")
            or reason.startswith("occupied by ")
        )

    def _active_order_for_robot(self, robot: FleetRobot) -> FleetOrder | None:
        if robot.active_order_id:
            order = self.orders.get(robot.active_order_id)
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                return order
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.assigned_robot == robot.name or order.vehicle == robot.name:
                return order
        return None

    def _is_robot_conflict(self, reason: str) -> bool:
        value = str(reason)
        return (
            value.startswith("yield to ")
            or value.startswith("occupied by ")
            or value.startswith("keep clearance from ")
        )

    def _is_deadlock_reason(self, reason: str) -> bool:
        value = str(reason or "")
        return value.startswith("deadlock:") or "continuous_conflict_unresolved" in value

    def _should_replan_for_blocked_reason(self, reason: str) -> bool:
        if self._is_deadlock_reason(reason):
            return False
        if not self._is_robot_conflict(reason):
            return True
        return self._is_parked_robot_conflict(reason)

    def _wait_expected_to_clear(self, robot: FleetRobot) -> bool:
        blocker_name = robot.wait_for_robot or self._robot_name_from_conflict_reason(
            robot.last_reason,
        )
        blocker = self.robots.get(blocker_name)
        if blocker is None or not blocker.trajectory:
            return False
        if not (
            blocker.status == "MOVING"
            or str(blocker.last_reason).startswith("planned traffic wait")
        ):
            return False

        current_pose = robot.pose
        if current_pose is None or not robot.trajectory:
            return False
        candidate_pose: dict[str, float] | None = None
        step = self._continuous_collision_step()
        horizon = self._reservation_horizon()
        offset = step
        while offset <= horizon + 0.000001:
            pose = self._pose_at_trajectory(
                robot.trajectory,
                min(
                    float(robot.trajectory[-1].get("t", 0.0) or 0.0),
                    robot.route_clock + offset,
                ),
            )
            if pose is not None and math.hypot(
                float(pose.get("x", 0.0) or 0.0) - float(current_pose.get("x", 0.0) or 0.0),
                float(pose.get("y", 0.0) or 0.0) - float(current_pose.get("y", 0.0) or 0.0),
            ) >= 0.02:
                candidate_pose = pose
                break
            offset += step
        if candidate_pose is None:
            return True

        wait = 0.0
        while wait <= horizon + 0.000001:
            blocker_pose = self._predicted_robot_pose(blocker, wait)
            if blocker_pose is None or not self.collision.robot_footprints_conflict(
                candidate_pose,
                blocker_pose,
            ):
                return True
            wait += step
        return False

    def _is_parked_robot_conflict(self, reason: str) -> bool:
        other_name = self._robot_name_from_conflict_reason(reason)
        other = self.robots.get(other_name)
        if other is None:
            return False
        return not self._is_active_traffic(other)

    def _robot_name_from_conflict_reason(self, reason: str) -> str:
        value = str(reason or "")
        for prefix in ("yield to ", "occupied by ", "keep clearance from "):
            if value.startswith(prefix):
                return value[len(prefix):].strip()
        return ""

    def _set_wait_dependency(self, robot: FleetRobot, reason: str, now: float) -> None:
        blocker = self._robot_name_from_conflict_reason(reason)
        if not blocker:
            self._clear_wait_dependency(robot)
            return
        robot.wait_for_robot = blocker
        robot.wait_resource = self._edge_id_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        )
        robot.wait_release_at = now + self.collision.lookahead_time()

    @staticmethod
    def _clear_wait_dependency(robot: FleetRobot) -> None:
        robot.wait_for_robot = ""
        robot.wait_resource = ""
        robot.wait_release_at = 0.0

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

        low = 0
        high = len(trajectory) - 1
        while low + 1 < high:
            middle = (low + high) // 2
            middle_time = float(trajectory[middle].get("t", 0.0) or 0.0)
            if middle_time < elapsed:
                low = middle
            else:
                high = middle
        index = low
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
        fleet = self._dict_param("fleet")
        speed = max(0.05, float(navigation.get("route_speed", 0.35) or 0.35))
        stop_distance = max(0.08, float(navigation.get("stop_distance", 0.40) or 0.40))
        footprint_lookahead = max(
            stop_distance,
            float(navigation.get("footprint_lookahead", 0.80) or 0.80),
        )
        try:
            configured = float(fleet.get("runtime_collision_lookahead_sec") or 0.0)
        except (TypeError, ValueError):
            configured = 0.0
        if configured > 0.0:
            return max(0.25, min(4.0, configured))

        # Auto mode covers both the configured spatial preview and a full
        # braking interval. At warehouse speed this is intentionally much
        # earlier than the final footprint-overlap guard.
        try:
            acceleration = max(
                0.10,
                float(navigation.get("route_acceleration", 0.60) or 0.60),
            )
        except (TypeError, ValueError):
            acceleration = 0.60
        braking_time = speed / acceleration
        distance_time = footprint_lookahead / speed
        reaction_time = max(
            0.05,
            float(fleet.get("continuous_collision_step_sec", 0.10) or 0.10),
        )
        return max(0.35, min(3.0, max(braking_time, distance_time) + reaction_time))

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
        margin = self.collision_margin()
        if not self._footprint_bounds_may_overlap(first_pose, second_pose, margin):
            return False
        return self.polygons_overlap(
            self.footprint_corners(first_pose),
            self.footprint_corners(second_pose),
            margin,
        )

    def robot_footprints_conflict(self, first_pose: dict[str, float], second_pose: dict[str, float]) -> bool:
        margin = self.robot_collision_margin()
        if not self._footprint_bounds_may_overlap(first_pose, second_pose, margin):
            return False
        return self.polygons_overlap(
            self.footprint_corners(first_pose),
            self.footprint_corners(second_pose),
            margin,
        )

    def _footprint_bounds_may_overlap(
        self,
        first_pose: dict[str, float],
        second_pose: dict[str, float],
        margin: float,
    ) -> bool:
        center_distance = math.hypot(
            float(first_pose.get("x", 0.0) or 0.0)
            - float(second_pose.get("x", 0.0) or 0.0),
            float(first_pose.get("y", 0.0) or 0.0)
            - float(second_pose.get("y", 0.0) or 0.0),
        )
        return center_distance <= self._footprint_diameter() + max(0.0, margin)

    def robot_broadphase_distance(self) -> float:
        return self._footprint_diameter() + self.robot_collision_margin()

    def physical_broadphase_distance(self) -> float:
        return self._footprint_diameter() + self.collision_margin()

    def _footprint_diameter(self) -> float:
        radius = max(
            (
                math.hypot(point["x"], point["y"])
                for point in self.footprint()
            ),
            default=0.22,
        )
        return radius * 2.0

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
        resolution = self.map_metadata.resolution
        return {
            "x": round(point["x"] / resolution),
            "y": round(point["y"] / resolution),
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
                {"x": 0.220000, "y": 0.000000},
                {"x": 0.203253, "y": 0.084190},
                {"x": 0.155563, "y": 0.155563},
                {"x": 0.084190, "y": 0.203253},
                {"x": 0.000000, "y": 0.220000},
                {"x": -0.084190, "y": 0.203253},
                {"x": -0.155563, "y": 0.155563},
                {"x": -0.203253, "y": 0.084190},
                {"x": -0.220000, "y": 0.000000},
                {"x": -0.203253, "y": -0.084190},
                {"x": -0.155563, "y": -0.155563},
                {"x": -0.084190, "y": -0.203253},
                {"x": 0.000000, "y": -0.220000},
                {"x": 0.084190, "y": -0.203253},
                {"x": 0.155563, "y": -0.155563},
                {"x": 0.203253, "y": -0.084190},
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

    def robot_collision_margin(self) -> float:
        fleet = self._dict_param("fleet")
        try:
            clearance = float(fleet.get("robot_clearance_m", 0.35) or 0.35)
        except (TypeError, ValueError):
            clearance = 0.35
        return self.collision_margin() + max(0.0, clearance)

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
