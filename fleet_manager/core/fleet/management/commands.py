"""Fleet API commands for paths, orders and direct planning."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.fleet.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.fleet.management.state import _manager_wall_time
from fleet_manager.core.fleet.domain.models import FleetEvent, FleetRobot

from .state import runtime_command


class FleetManagerCommandMixin:
    """Execute order and planner commands against the shared runtime."""

    @runtime_command
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

    @runtime_command
    def orders_payload(self) -> dict[str, Any]:
        self._advance_runtime()
        return {
            "ok": True,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    @runtime_command
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

        self._prune_terminal_order_history()
        if replace_active:
            for vehicle in sorted({order.vehicle for order in orders if order.vehicle}):
                self._replace_orders_for_robot(vehicle, "replaced by operator")

        for order in orders:
            # Reusing an external order id is valid once its previous record
            # is terminal.  Its old stationary-blocker quarantine is not: it
            # describes another task and could otherwise suppress the fresh
            # order indefinitely while the same parked bodies remain nearby.
            self._stationary_order_retry_state.pop(order.order_id, None)
            self.orders[order.order_id] = order
            self._event(
                "info",
                f"order queued: {order.order_id} {order.vehicle or 'auto'}->{order.target_lm}",
            )
        self._advance_planning_revision("order queued")
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

    @runtime_command
    def dispatch_orders(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        dispatched = self._dispatch_orders(force=True)
        return {
            "ok": True,
            "dispatched": dispatched,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    @runtime_command
    def cancel_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        self._cancel_order(order, "canceled by operator")
        self._advance_planning_revision(f"order canceled: {order_id}")
        return {
            "ok": True,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    @runtime_command
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
        self._advance_planning_revision(f"order paused: {order_id}")
        return {
            "ok": True,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    @runtime_command
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
        order.updated_at = self._now()
        self._event("info", f"order resumed: {order.order_id}")
        self._advance_planning_revision(f"order resumed: {order_id}")
        dispatched = self._dispatch_orders(force=True)
        return {
            "ok": True,
            "dispatched": dispatched,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    @runtime_command
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
            self._advance_planning_revision("orders cleared")
        return {
            "ok": True,
            "canceled": canceled,
            "orders": self._orders_list(),
            "state": self.state(),
        }


    @runtime_command
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
            clean_request = self._prepare_manual_plan_request(request)
            if clean_request is not None:
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
            now = self._now()
            self._apply_planner_result(result, now)
            self._event("info", f"planner accepted {len(planned_names)} order(s)")
        else:
            self._apply_manual_plan_failure(result, valid_requests)

        return {
            **result,
            "fleetState": self.state(),
        }

    def _prepare_manual_plan_request(
        self,
        request: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Validate one legacy API request and mark its robot as planning."""
        name = str(request.get("name", "")).strip()
        start_lm = str(
            request.get("startLm") or request.get("currentLm") or ""
        ).strip()
        goal_lm = str(
            request.get("goalLm") or request.get("targetLm") or ""
        ).strip()
        start_pose = request.get("startPose")
        if not name or not start_lm or not goal_lm:
            self._event(
                "error",
                "bad order ignored: name/start/goal is missing",
            )
            return None
        if start_lm not in self.landmarks:
            self._block_order(
                name,
                start_lm,
                goal_lm,
                f"unknown start LM: {start_lm}",
            )
            return None
        if goal_lm not in self.landmarks:
            self._block_order(
                name,
                start_lm,
                goal_lm,
                f"unknown goal LM: {goal_lm}",
            )
            return None

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
                return None
            if safe_start_lm != start_lm:
                self._event(
                    "info",
                    f"corrected stale start LM for {name}: "
                    f"{start_lm}->{safe_start_lm}",
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
            return None
        if robot is None:
            robot = FleetRobot(
                name=name,
                current_lm=start_lm,
                pose=self._pose_at_landmark(start_lm),
                updated_at=self._now(),
            )
            self.robots[name] = robot
        self._accept_manual_plan_robot(robot, start_lm, goal_lm)
        return self._clean_manual_plan_request(
            name,
            start_lm,
            goal_lm,
            authoritative_pose,
            robot,
        )

    def _accept_manual_plan_robot(
        self,
        robot: FleetRobot,
        start_lm: str,
        goal_lm: str,
    ) -> None:
        robot.current_lm = start_lm
        robot.target_lm = goal_lm
        robot.status = "PLANNING"
        robot.last_reason = "order accepted"
        robot.blocked_since = None
        robot.updated_at = self._now()
        self._event(
            "info",
            f"order accepted: {robot.name} {start_lm}->{goal_lm}",
        )

    @staticmethod
    def _clean_manual_plan_request(
        name: str,
        start_lm: str,
        goal_lm: str,
        authoritative_pose: Any,
        robot: FleetRobot,
    ) -> dict[str, Any]:
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
        return clean_request

    def _apply_manual_plan_failure(
        self,
        result: dict[str, Any],
        valid_requests: list[dict[str, Any]],
    ) -> None:
        """Apply a shared planner rejection consistently to admitted robots."""
        deadlock = self._planner_deadlock_result(result)
        reason = self._planner_failure_reason(result)
        for request in valid_requests:
            if not isinstance(request, dict):
                continue
            robot = self.robots.get(str(request.get("name", "")).strip())
            if robot is None:
                continue
            robot.status = "WAITING" if deadlock else "BLOCKED"
            robot.last_reason = reason
            if deadlock:
                robot.blocked_since = self._now()
                robot.trajectory = []
                robot.plan_nodes = []
                robot.trajectory_dirty = True
                robot.route_started_at = None
                robot.route_clock = 0.0
                robot.last_tick_at = None
                robot.route_note = "DEADLOCK"
            robot.updated_at = self._now()
        self._event("error", f"planner rejected: {reason}")

    def _event(self, level: str, message: str) -> None:
        # Event log timestamps stay in wall time for a human operator even
        # while the simulated traffic clock is running faster than real time.
        self.events.append(
            FleetEvent(
                stamp=_manager_wall_time(),
                level=level,
                message=message,
            )
        )
        self.events = self.events[-200:]
