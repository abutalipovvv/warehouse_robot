"""Simulation route metadata, rolling handoff and pose sampling."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.domain.models import FleetOrder, FleetRobot


class FleetManagerRouteMetadataMixin:
    """Maintain stable route revisions, previews and sampled poses."""

    def _next_route_revision(self) -> int:
        """Allocate a revision shared by simulation and gRPC routes."""
        now_ms = int(self._now() * 1000)
        self._route_revision_seq = max(self._route_revision_seq + 1, now_ms)
        return self._route_revision_seq


    def _complete_simulated_route_chunk(self, robot: FleetRobot, now: float) -> bool:
        """Hold a completed safe chunk until its continuation is ready.

        Clearing the trajectory and active order here exposed an artificial
        IDLE robot to both the browser and subsequent MAPF requests. Keeping
        the terminal trajectory preserves the LM reservation and permits an
        atomic append without resetting the route clock.
        """
        if not robot.active_order_id or not robot.route_chunk_goal_lm:
            return False
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return False
        if robot.route_note == "manual graph reconnect":
            order.status = "QUEUED"
            order.error = ""
            order.updated_at = now
            order.assigned_robot = robot.name
            order.start_lm = robot.current_lm
            order.route_nodes = []
            robot.active_order_id = ""
            robot.target_lm = ""
            robot.status = "IDLE"
            robot.trajectory = []
            robot.trajectory_dirty = True
            robot.plan_nodes = []
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = now
            robot.last_reason = "manual graph reconnect complete; route queued"
            robot.route_note = ""
            self._clear_remote_route_metadata(robot)
            robot.updated_at = now
            self._event(
                "info",
                f"manual graph reconnect complete: {robot.name}@{robot.current_lm}",
            )
            return True
        final_target = self._active_order_target(order)
        if robot.current_lm != robot.route_chunk_goal_lm or robot.current_lm == final_target:
            return False

        first_boundary_tick = robot.rolling_boundary_since is None
        if first_boundary_tick:
            robot.rolling_boundary_since = now
            self._rolling_prefetch_eligible_since.setdefault(robot.name, now)
        order.status = "PLANNING"
        order.error = "rolling continuation pending"
        # Do not erase the real waiting age on every 10 Hz physics tick.
        if first_boundary_tick:
            order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = robot.current_lm
        order.route_nodes = list(robot.plan_nodes)
        if robot.status != "WAITING" or robot.last_reason != "rolling continuation pending":
            self._event(
                "info",
                f"route continuation pending: {order.order_id} "
                f"{robot.name}@{robot.current_lm}->{final_target}",
            )
        robot.status = "WAITING"
        robot.last_reason = "rolling continuation pending"
        robot.blocked_since = None
        robot.traffic_stall_since = None
        self._clear_wait_dependency(robot)
        robot.updated_at = now
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
        order.updated_at = self._now()
        order.route_nodes = list(robot.plan_nodes)


    def _clear_remote_route_metadata(self, robot: FleetRobot) -> None:
        self._clear_rolling_prefetch_state(robot.name)
        self._runtime_replans.pop(robot.name, None)
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
        # A successful full dispatch/replan starts a new continuation episode.
        # Failure/backoff from an older route must not poison this route later.
        self._runtime_replans.pop(robot.name, None)
        self._clear_rolling_prefetch_state(robot.name)
        previous_final = robot.route_final_lm
        previous_chunk = robot.route_chunk_goal_lm
        planned_chunk_goal = str(
            plan.get("goalLm") or order.target_lm
        ).strip()
        trajectory = [
            sample
            for sample in plan.get("trajectory", [])
            if isinstance(sample, dict)
        ]
        trajectory_goal = (
            str(trajectory[-1].get("lm") or "").strip()
            if trajectory
            else ""
        )
        chunk_goal = (
            trajectory_goal
            if trajectory_goal in self.landmarks
            else planned_chunk_goal
        )
        final_goal = str(plan.get("finalGoalLm") or order.target_lm).strip()
        plan_nodes = [
            str(node)
            for node in plan.get("nodes", [])
            if str(node) in self.landmarks
        ]
        trajectory_nodes: list[str] = []
        for sample in trajectory:
            sample_lm = str(sample.get("lm") or "").strip()
            if (
                sample_lm in self.landmarks
                and (
                    not trajectory_nodes
                    or trajectory_nodes[-1] != sample_lm
                )
            ):
                trajectory_nodes.append(sample_lm)
        if (
            chunk_goal
            and plan_nodes
            and chunk_goal not in plan_nodes
            and trajectory_nodes
            and trajectory_nodes[-1] == chunk_goal
        ):
            plan_nodes = trajectory_nodes
        if (
            chunk_goal
            and plan_nodes
            and plan_nodes[-1] != chunk_goal
            and chunk_goal in plan_nodes
        ):
            # Keep every consumer on the same executable prefix.  This is a
            # defensive boundary for old/in-flight planner results; new
            # rolling results are already cut on the exact LM sample.
            terminal_index = max(
                index
                for index, node in enumerate(plan_nodes)
                if node == chunk_goal
            )
            plan_nodes = plan_nodes[: terminal_index + 1]
            plan["nodes"] = list(plan_nodes)
        if chunk_goal:
            plan["goalLm"] = chunk_goal
            robot.plan_nodes = list(plan_nodes)
            order.route_nodes = list(plan_nodes)
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



__all__ = ["FleetManagerRouteMetadataMixin"]

