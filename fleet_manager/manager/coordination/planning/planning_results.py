"""Application and presentation of traffic-planner results."""

from __future__ import annotations

from typing import Any


class TrafficPlanResultMixin:
    """Apply accepted plans to runtime robots and explain planner outcomes."""

    def _apply_planner_result(
        self,
        result: dict[str, Any],
        now: float | None = None,
        order_id: str | None = None,
    ) -> None:
        now = now or self._now()
        applied = False
        for plan in result.get("plans", []):
            if not isinstance(plan, dict):
                continue
            name = str(plan.get("robot", ""))
            robot = self.robots.get(name)
            if robot is None:
                continue
            applied = True
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
        if applied:
            advance_revision = getattr(
                self,
                "_advance_planning_revision",
                None,
            )
            if callable(advance_revision):
                advance_revision("planner result committed")

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
