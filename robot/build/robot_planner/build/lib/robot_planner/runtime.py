from __future__ import annotations

from dataclasses import dataclass, field
from threading import RLock
from time import monotonic, time
from typing import Any
import uuid


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def to_dict(self) -> dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
        }


@dataclass
class Velocity2D:
    linear: float = 0.0
    angular: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {
            "linear": self.linear,
            "angular": self.angular,
        }


@dataclass
class RoutePoint:
    x: float
    y: float
    yaw: float
    edge_id: str
    motion_direction: str = "forward"

    def to_dict(self) -> dict[str, float | str]:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "edgeId": self.edge_id,
            "motionDirection": self.motion_direction,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "RoutePoint":
        return cls(
            x=float(payload.get("x", 0.0) or 0.0),
            y=float(payload.get("y", 0.0) or 0.0),
            yaw=float(payload.get("yaw", 0.0) or 0.0),
            edge_id=str(payload.get("edgeId") or payload.get("edge_id") or ""),
            motion_direction=str(payload.get("motionDirection") or payload.get("motion_direction") or "forward"),
        )


@dataclass
class PlannedRobotRoute:
    route_id: str
    start_lm: str
    goal_lm: str
    nodes: list[str]
    trajectory: list[RoutePoint]
    length: float
    current_index: int = 0
    created_at: float = field(default_factory=time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "routeId": self.route_id,
            "startLm": self.start_lm,
            "goalLm": self.goal_lm,
            "nodes": list(self.nodes),
            "length": self.length,
            "trajectory": [point.to_dict() for point in self.trajectory],
            "currentIndex": self.current_index,
        }

    @classmethod
    def create(
        cls,
        start_lm: str,
        goal_lm: str,
        nodes: list[str],
        trajectory: list[RoutePoint],
        length: float,
    ) -> "PlannedRobotRoute":
        return cls(
            route_id=f"route-{uuid.uuid4().hex[:12]}",
            start_lm=start_lm,
            goal_lm=goal_lm,
            nodes=list(nodes),
            trajectory=list(trajectory),
            length=float(length),
        )

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "PlannedRobotRoute":
        route_id = str(payload.get("routeId") or payload.get("route_id") or "").strip()
        if not route_id:
            route_id = f"route-{uuid.uuid4().hex[:12]}"
        trajectory_payload = payload.get("trajectory", [])
        if not isinstance(trajectory_payload, list):
            trajectory_payload = []
        return cls(
            route_id=route_id,
            start_lm=str(payload.get("startLm") or payload.get("start_lm") or ""),
            goal_lm=str(payload.get("goalLm") or payload.get("goal_lm") or ""),
            nodes=[str(item) for item in payload.get("nodes", []) if str(item)],
            trajectory=[
                RoutePoint.from_dict(item)
                for item in trajectory_payload
                if isinstance(item, dict)
            ],
            length=float(payload.get("length", 0.0) or 0.0),
            current_index=int(payload.get("currentIndex", 0) or 0),
        )


@dataclass
class RobotEvent:
    stamp: float
    level: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp": self.stamp,
            "level": self.level,
            "message": self.message,
        }


class RobotRuntime:
    def __init__(self, robot_id: str, map_id: str) -> None:
        self.robot_id = robot_id
        self.map_id = map_id
        self._lock = RLock()
        self._pose: Pose2D | None = None
        self._pose_updated_at: float | None = None
        self._velocity = Velocity2D()
        self._state = "LOCALIZING"
        self._message = "Waiting for amcl pose."
        self._nearest_lm = ""
        self._target_lm = ""
        self._current_edge_id = ""
        self._route_progress = 0.0
        self._active_route: PlannedRobotRoute | None = None
        self._events: list[RobotEvent] = []
        self.add_event("info", "robot runtime initialized")

    def add_event(self, level: str, message: str) -> None:
        with self._lock:
            self._events.append(RobotEvent(stamp=time(), level=level, message=message))
            self._events = self._events[-120:]

    def set_map(self, map_id: str) -> None:
        with self._lock:
            self.map_id = str(map_id)
            self._active_route = None
            self._target_lm = ""
            self._current_edge_id = ""
            self._route_progress = 0.0
            self._state = "LOCALIZING"
            self._message = f"Map changed to {self.map_id}. Waiting for localization."

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._pose = Pose2D(x=x, y=y, yaw=yaw)
            self._pose_updated_at = monotonic()

    def set_velocity(self, linear: float, angular: float) -> None:
        with self._lock:
            self._velocity = Velocity2D(linear=linear, angular=angular)

    def set_nearest_lm(self, name: str) -> None:
        with self._lock:
            self._nearest_lm = name

    def latest_pose(self) -> Pose2D | None:
        with self._lock:
            if self._pose is None:
                return None
            return Pose2D(x=self._pose.x, y=self._pose.y, yaw=self._pose.yaw)

    def localization_age(self, now: float | None = None) -> float:
        with self._lock:
            if self._pose_updated_at is None:
                return float("inf")
            current = now if now is not None else monotonic()
            return max(0.0, current - self._pose_updated_at)

    def set_state(self, state: str, message: str) -> None:
        with self._lock:
            self._state = state
            self._message = message

    def set_route(self, route: PlannedRobotRoute) -> None:
        with self._lock:
            self._active_route = route
            self._target_lm = route.goal_lm
            self._current_edge_id = route.trajectory[0].edge_id if route.trajectory else ""
            self._route_progress = 0.0
            self._state = "EXECUTING_ROUTE"
            self._message = f"Executing route to {route.goal_lm}."

    def active_route(self) -> PlannedRobotRoute | None:
        with self._lock:
            if self._active_route is None:
                return None
            return PlannedRobotRoute(
                route_id=self._active_route.route_id,
                start_lm=self._active_route.start_lm,
                goal_lm=self._active_route.goal_lm,
                nodes=list(self._active_route.nodes),
                trajectory=list(self._active_route.trajectory),
                length=self._active_route.length,
                current_index=self._active_route.current_index,
                created_at=self._active_route.created_at,
            )

    def update_route_progress(self, index: int, edge_id: str, progress: float) -> None:
        with self._lock:
            if self._active_route is None:
                return
            self._active_route.current_index = max(0, index)
            self._current_edge_id = edge_id
            self._route_progress = max(0.0, min(1.0, progress))

    def cancel_route(self, message: str = "Route canceled.") -> None:
        with self._lock:
            self._active_route = None
            self._target_lm = ""
            self._current_edge_id = ""
            self._route_progress = 0.0
            self._state = "IDLE"
            self._message = message

    def finish_route(self, arrived: bool, message: str) -> None:
        with self._lock:
            goal_lm = self._target_lm
            self._active_route = None
            self._target_lm = ""
            self._current_edge_id = ""
            self._route_progress = 1.0 if arrived else self._route_progress
            self._state = "ARRIVED" if arrived else "ERROR"
            if arrived and goal_lm:
                self._nearest_lm = goal_lm
            self._message = message

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            route = self._active_route.to_dict() if self._active_route is not None else None
            pose = self._pose.to_dict() if self._pose is not None else None
            velocity = self._velocity.to_dict()
            return {
                "robotId": self.robot_id,
                "mapId": self.map_id,
                "pose": pose,
                "velocity": velocity,
                "state": self._state,
                "message": self._message,
                "nearestLm": self._nearest_lm,
                "targetLm": self._target_lm,
                "currentEdgeId": self._current_edge_id,
                "routeProgress": self._route_progress,
                "route": route,
                "localizationAgeSec": self.localization_age(),
                "events": [item.to_dict() for item in self._events[-120:]],
            }
