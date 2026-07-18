"""Fleet motion advancement, safety commit and replanning runtime."""

from __future__ import annotations

import math
from time import time
from typing import Any

from fleet_manager.core.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.models import FleetOrder, FleetRobot


class FleetMotionRuntimeMixin:
    """Advance robot motion without coupling it to UI or robot transport."""

    def _advance_runtime(self) -> None:
        # Commit a completed background result before deadlock arbitration.
        # Otherwise dispatch immediately occupies the single worker again and
        # a waiting coupled component can starve forever without a CBS slot.
        self._finish_async_simulated_dispatch()
        now = self._now()
        safety_snapshots = {
            robot.name: self._runtime_safety_snapshot(robot)
            for robot in self._runtime_robots()
            if not robot.is_remote()
        }
        self._runtime_tick_route_clocks = {
            name: float(snapshot["route_clock"])
            for name, snapshot in safety_snapshots.items()
        }
        self._prepare_controlled_corridor_admissions(now)
        self._prepare_traffic_zone_admissions(now)
        for robot in self._runtime_robots():
            if robot.is_remote():
                self._advance_remote_robot_order(robot, now)
                continue
            if (
                robot.status == "IDLE"
                and not robot.active_order_id
                and not robot.trajectory
                and str(robot.last_reason or "").startswith(
                    "background replan queued:"
                )
            ):
                # Normalize states produced by an interrupted/older runtime.
                # A stale chunk target makes _robot_can_accept_order reject the
                # robot forever even though its replacement order is QUEUED.
                robot.target_lm = ""
                robot.last_reason = (
                    "route replan queued"
                    if self._active_order_for_robot(robot) is not None
                    else "idle: no active order"
                )
                robot.updated_at = now
            self._refresh_runtime_priority_lease(robot, now)
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
            if final_time > 0.0 and robot.route_clock >= final_time - 0.000001:
                endpoint = self._pose_at_trajectory(robot.trajectory, final_time)
                if endpoint is not None:
                    robot.pose = endpoint
                self._update_current_lm_from_trajectory(robot)
                robot.current_lm = robot.target_lm or robot.current_lm
                if self._activate_rolling_prefetch(robot, now):
                    final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                elif self._complete_simulated_route_chunk(robot, now):
                    robot.last_tick_at = now
                    continue
            last_tick_at = robot.last_tick_at or now
            dt = min(
                0.20 * self.simulation_time_scale(),
                max(0.0, now - last_tick_at),
            )
            robot.last_tick_at = now
            blocked_reason = ""
            moved_during_tick = False
            preflight_pending = self._runtime_collision_preflight_due(
                robot,
                now,
            )
            if (
                robot.status == "WAITING"
                and self._is_robot_conflict(robot.last_reason)
                and not preflight_pending
            ):
                # A far-horizon scan already selected this safe stop point.
                # Do not creep forward using only the immediate check between
                # refreshes; hold steadily until the next preflight decides
                # that the dependency has cleared.
                robot.updated_at = now
                self._update_active_order_from_robot(robot)
                continue
            if dt > 0.000001:
                remaining = dt
                handoffs = 0
                # Never jump over the interval between physics ticks. Each
                # small move is checked at its actual future pose, which also
                # prevents two robots from swapping sides between snapshots.
                #
                # A prefetched rolling route is adopted inside this loop. Any
                # part of the current physics slice left after reaching the
                # old chunk LM is therefore consumed on the new trajectory.
                # Previously that remainder was discarded, producing a
                # visible stop at every otherwise identical route revision.
                while remaining > 0.000001 and robot.trajectory:
                    final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                    if final_time <= 0.0 or robot.route_clock >= final_time - 0.000001:
                        robot.route_clock = max(0.0, final_time)
                        endpoint = self._pose_at_trajectory(robot.trajectory, robot.route_clock)
                        if endpoint is not None:
                            robot.pose = endpoint
                        self._update_current_lm_from_trajectory(robot)
                        robot.current_lm = robot.target_lm or robot.current_lm
                        if handoffs >= 4 or not self._activate_rolling_prefetch(robot, now):
                            break
                        handoffs += 1
                        # The adopted trajectory has its own zero-based clock.
                        # Pairwise prediction for the remainder of this tick
                        # must use that new clock as well.
                        self._runtime_tick_route_clocks[robot.name] = robot.route_clock
                        preflight_pending = True
                        continue

                    motion_dt = min(
                        self._runtime_motion_step(),
                        remaining,
                        max(0.0, final_time - robot.route_clock),
                    )
                    if motion_dt <= 0.000001:
                        break
                    proposed_clock = robot.route_clock + motion_dt
                    # Before moving at all on each committed chunk, scan far
                    # enough ahead to stop outside the shared clearance zone.
                    # Subsequent substeps are already covered by that scan and
                    # only need the immediate invariant check.
                    if preflight_pending:
                        blocked_reason = self._blocked_ahead(robot, proposed_clock)
                        self._mark_runtime_collision_preflight(robot, now)
                    else:
                        blocked_reason = self._blocked_at_clock(robot, proposed_clock)
                    preflight_pending = False
                    if blocked_reason:
                        break
                    robot.route_clock = proposed_clock
                    remaining -= motion_dt
                    moved_during_tick = True
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
                    and now - robot.traffic_stall_since
                    >= self._blocked_replan_after(blocked_reason)
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
                if moved_during_tick or robot.route_clock > route_clock_before + 0.000001:
                    self._record_traffic_progress(robot)
            self._update_active_order_from_robot(robot)
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            if final_time > 0.0 and robot.route_clock >= final_time:
                robot.current_lm = robot.target_lm or robot.current_lm
                if self._activate_rolling_prefetch(robot, now):
                    continue
                rolling_chunk = self._complete_simulated_route_chunk(robot, now)
                if rolling_chunk:
                    robot.last_tick_at = now
                    continue
                self._complete_active_order(robot, now)
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
                self._clear_remote_route_metadata(robot)
                self._event("info", f"{robot.name} arrived at {robot.current_lm}")
        self._enforce_runtime_safety_invariant(safety_snapshots, now)
        self._runtime_tick_route_clocks = {}
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
        dt = min(
            0.20 * self.simulation_time_scale(),
            max(0.0, now - last_tick_at),
        )
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
                avoid_lm = ""
                if self._is_parked_robot_conflict(reason):
                    blocker = self.robots.get(
                        self._robot_name_from_conflict_reason(reason)
                    )
                    if blocker is not None:
                        avoid_lm = self._traffic_lm_for_robot(blocker)
                self._queue_alternate_corridor_detour(
                    order,
                    start_lm,
                    self._active_order_target(order),
                    avoid_lm=avoid_lm,
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
        # The detailed cause remains in the event/order history. At runtime
        # this is no longer an active deadlock, so do not leave a stale
        # "deadlock resolving" state above an IDLE robot.
        robot.last_reason = "route replan queued"
        robot.route_note = ""
        robot.pending_route = None
        robot.route_preview = []
        robot.route_preview_dirty = True
        self._clear_wait_dependency(robot)
        robot.updated_at = now
        self._event("warn", f"{robot.name} background replan queued: {reason}")
        return True

    def _reason_requires_spatial_replan(self, reason: str) -> bool:
        value = str(reason or "").lower()
        return bool(
            self._is_parked_robot_conflict(reason)
            or "alternate route required" in value
            or "traffic admission timeout" in value
            or "corridor admission timeout" in value
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
        index = self._trajectory_sample_index_at_or_before(
            robot.trajectory,
            robot.route_clock + 0.000001,
        )
        while index >= 0:
            lm_name = str(robot.trajectory[index].get("lm") or "").strip()
            if lm_name in self.landmarks:
                robot.current_lm = lm_name
                return
            index -= 1

    def _planned_wait_lm_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> str:
        if len(trajectory) < 2:
            return ""
        index = self._trajectory_segment_index(trajectory, elapsed)
        start = trajectory[index]
        end = trajectory[index + 1]
        start_time = float(start.get("t", 0.0) or 0.0)
        end_time = float(end.get("t", start_time) or start_time)
        if end_time <= start_time or not (start_time <= elapsed < end_time):
            return ""
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
