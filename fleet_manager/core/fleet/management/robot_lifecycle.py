"""Robot registration, reset, stop and lifecycle transitions."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.fleet.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.fleet.domain.models import FleetRobot

from .state import runtime_command


class FleetManagerRobotLifecycleMixin:
    """Create, update and retire fleet robots and their local state."""

    @runtime_command
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
            self.clear_robot_ephemeral_state(name)
            now = self._now()
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                mode=mode,
                pose=self._pose_at_landmark(current_lm),
                base_url=base_url,
                remote_id=self._remote_identity_id(remote_identity),
                remote_online=True,
                updated_at=now,
            )
            self.robots[name] = robot
            if remote_status is not None:
                self._apply_remote_status(robot, remote_status, self._now())
            self._event("info", f"robot added: {name}@{current_lm}" + (f" remote={base_url}" if robot.is_remote() else ""))
        else:
            self.clear_robot_ephemeral_state(name)
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
            robot.updated_at = self._now()
            if remote_status is not None:
                self._apply_remote_status(robot, remote_status, self._now())
            self._event("info", f"robot updated: {name}@{current_lm}" + (f" remote={base_url}" if robot.is_remote() else ""))
        self._advance_planning_revision(f"robot registered: {name}")
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    @runtime_command
    def remove_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        removed = self.robots.pop(name, None)
        if removed is not None:
            self.clear_robot_ephemeral_state(name)
            self._cancel_active_order_for_robot(removed, "robot removed")
            self._cancel_orders_for_robot(name, "robot removed")
            self._event("warn", f"robot removed: {name}")
            self._advance_planning_revision(f"robot removed: {name}")
        return {"ok": True, "removed": removed is not None, "state": self.state()}

    @runtime_command
    def stop_robot(
        self,
        payload: dict[str, Any],
        *,
        include_state: bool = True,
    ) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        stopped_robot: FleetRobot | None = None
        if name:
            robot = self.robots.get(name)
            if robot is None:
                raise ValueError(f"unknown robot: {name}")
            self._stop_robot(robot)
            stopped_robot = robot
            self._cancel_orders_for_robot(name, "robot stopped")
            self._event("warn", f"robot stopped: {name}")
        else:
            for robot in self._runtime_robots():
                self._stop_robot(robot)
            self._cancel_all_orders("fleet stopped")
            self._event("warn", "fleet stopped")
        self._advance_planning_revision(
            f"robot stopped: {name}" if name else "fleet stopped"
        )
        return {
            "ok": True,
            "robot": stopped_robot.to_dict() if stopped_robot is not None else None,
            "state": self.state() if include_state else None,
        }


    @runtime_command
    def reset_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        target_names = [name] if name else [robot.name for robot in self._runtime_robots()]
        reset_count = 0
        for robot_name in target_names:
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            reset_count += 1
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
            robot.updated_at = self._now()
            self._event("warn", f"robot reset: {robot.name}@{robot.current_lm}")
        if reset_count:
            self._advance_planning_revision("robot state reset")
        return {"ok": True, "state": self.state()}

    @runtime_command
    def update_robot(
        self,
        payload: dict[str, Any],
        *,
        include_state: bool = True,
    ) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        planning_before = self._planning_state_fingerprint()
        robot = self.robots.get(name)
        if robot is None:
            current_lm = str(payload.get("currentLm") or "").strip()
            if not current_lm:
                raise ValueError("unknown robot and currentLm is missing")
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                pose=self._pose_at_landmark(current_lm),
                updated_at=self._now(),
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
        robot.updated_at = self._now()
        if self._planning_state_fingerprint() != planning_before:
            self._advance_planning_revision(f"robot state updated: {name}")
        return {
            "ok": True,
            "robot": robot.to_dict(),
            "state": self.state() if include_state else None,
        }


    def _stop_robot(self, robot: FleetRobot, cancel_active_order: bool = True) -> None:
        self._stop_remote_robot(robot)
        self._runtime_replans.pop(robot.name, None)
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
        robot.updated_at = self._now()

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
                updated_at=self._now(),
            )
            self.robots[name] = robot
        else:
            robot.current_lm = start_lm
            robot.target_lm = goal_lm
            robot.status = "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = self._now()
        self._event("error", f"{name} blocked: {reason}")
