"""Fleet time-step advancement and route completion."""

from __future__ import annotations

from dataclasses import dataclass
import math

from fleet_manager.core.fleet.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.fleet.domain.models import FleetRobot


@dataclass(frozen=True, slots=True)
class _MotionClockResult:
    """Outcome of consuming one robot's physics slice."""

    blocked_reason: str = ""
    moved: bool = False
    handled: bool = False


class FleetMotionStepMixin:
    """Advance local and remote robots through one runtime tick."""

    def _advance_runtime(self) -> None:
        self._synchronize_planning_revision()
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
            self._advance_runtime_robot(robot, now)
        self._enforce_runtime_safety_invariant(safety_snapshots, now)
        self._runtime_tick_route_clocks = {}
        self._resolve_runtime_wait_cycles(now)
        self._dispatch_orders(async_simulated=True)
        self._synchronize_planning_revision()

    def _advance_runtime_robot(
        self,
        robot: FleetRobot,
        now: float,
    ) -> None:
        """Advance one robot while preserving its committed route contract."""

        if robot.is_remote():
            self._advance_remote_robot_order(robot, now)
            return
        self._normalize_interrupted_runtime_state(robot, now)
        self._refresh_runtime_priority_lease(robot, now)
        if self._settle_degenerate_simulated_route(robot, now):
            return
        route_clock_before = robot.route_clock
        if self._hold_robot_for_runtime_replan(robot, now):
            return
        if robot.retreat_target_clock is not None:
            self._advance_deadlock_retreat(robot, now)
            self._update_active_order_from_robot(robot)
            return
        if self._handle_robot_without_active_motion(robot, now):
            return

        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        if final_time > 0.0 and robot.route_clock >= final_time - 0.000001:
            endpoint = self._pose_at_trajectory(robot.trajectory, final_time)
            if endpoint is not None:
                robot.pose = endpoint
            self._settle_completed_trajectory_endpoint(robot, now)
            if self._activate_rolling_prefetch(robot, now):
                final_time = float(
                    robot.trajectory[-1].get("t", 0.0) or 0.0
                )
            elif self._complete_simulated_route_chunk(robot, now):
                robot.last_tick_at = now
                return

        clock_result = self._advance_runtime_route_clock(robot, now)
        if clock_result.handled:
            return
        if clock_result.blocked_reason:
            self._hold_robot_for_runtime_collision(
                robot,
                now,
                clock_result.blocked_reason,
            )
            return
        self._commit_runtime_motion_state(
            robot,
            now,
            route_clock_before=route_clock_before,
            moved_during_tick=clock_result.moved,
        )

    def _normalize_interrupted_runtime_state(
        self,
        robot: FleetRobot,
        now: float,
    ) -> None:
        if (
            robot.status != "IDLE"
            or robot.active_order_id
            or robot.trajectory
            or not str(robot.last_reason or "").startswith(
                "background replan queued:"
            )
        ):
            return
        robot.target_lm = ""
        robot.last_reason = (
            "route replan queued"
            if self._active_order_for_robot(robot) is not None
            else "idle: no active order"
        )
        robot.updated_at = now

    def _hold_robot_for_runtime_replan(
        self,
        robot: FleetRobot,
        now: float,
    ) -> bool:
        if not self._runtime_replan_holds_robot(robot):
            return False
        state = self._runtime_replans.get(robot.name, {})
        reason = str(state.get("reason") or "traffic changed")
        stage = str(state.get("stage") or "")
        robot.status = "WAITING"
        if stage == "deadlock_escalated":
            blocker_name = str(
                state.get("escalated_blocker") or ""
            ).strip()
            if blocker_name:
                robot.wait_for_robot = blocker_name
                robot.wait_resource = str(
                    state.get("escalated_resource") or ""
                )
                robot.wait_release_at = 0.0
                robot.last_reason = f"occupied by {blocker_name}"
            else:
                robot.last_reason = (
                    "deadlock replan awaiting safe evacuation"
                )
        else:
            robot.last_reason = f"replanning route while holding: {reason}"
        robot.last_tick_at = now
        robot.updated_at = now
        self._update_active_order_from_robot(robot)
        return True

    def _handle_robot_without_active_motion(
        self,
        robot: FleetRobot,
        now: float,
    ) -> bool:
        """Handle every state that cannot consume trajectory time."""

        if (
            robot.status in {"MOVING", "WAITING"}
            and not robot.trajectory
            and not robot.target_lm
        ):
            if (
                robot.active_order_id
                and self._queue_active_order_for_background_replan(
                    robot,
                    now,
                    "missing active trajectory",
                )
            ):
                self._update_active_order_from_robot(robot)
                return True
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
            return True
        if self._is_deadlock_reason(robot.last_reason) and not robot.trajectory:
            robot.status = "WAITING"
            robot.blocked_since = robot.blocked_since or now
            robot.last_tick_at = now
            robot.updated_at = now
            self._update_active_order_from_robot(robot)
            return True
        if robot.status in {"BLOCKED", "PLANNING"} and robot.target_lm:
            self._schedule_runtime_replan(robot, now, "no active trajectory")
            self._update_active_order_from_robot(robot)
            robot.last_tick_at = now
            return True
        if robot.status not in {"MOVING", "WAITING"}:
            self._update_active_order_from_robot(robot)
            robot.last_tick_at = now
            return True
        if robot.trajectory:
            return False
        if robot.target_lm:
            self._schedule_runtime_replan(robot, now, "empty trajectory")
            self._update_active_order_from_robot(robot)
        return True

    def _advance_runtime_route_clock(
        self,
        robot: FleetRobot,
        now: float,
    ) -> _MotionClockResult:
        last_tick_at = robot.last_tick_at or now
        remaining = min(
            0.20 * self.simulation_time_scale(),
            max(0.0, now - last_tick_at),
        )
        robot.last_tick_at = now
        preflight_pending = self._runtime_collision_preflight_due(robot, now)
        if (
            robot.status == "WAITING"
            and self._is_robot_conflict(robot.last_reason)
            and not preflight_pending
        ):
            robot.updated_at = now
            self._update_active_order_from_robot(robot)
            return _MotionClockResult(handled=True)

        blocked_reason = ""
        moved_during_tick = False
        handoffs = 0
        while remaining > 0.000001 and robot.trajectory:
            final_time = float(
                robot.trajectory[-1].get("t", 0.0) or 0.0
            )
            if (
                final_time <= 0.0
                or robot.route_clock >= final_time - 0.000001
            ):
                robot.route_clock = max(0.0, final_time)
                endpoint = self._pose_at_trajectory(
                    robot.trajectory,
                    robot.route_clock,
                )
                if endpoint is not None:
                    robot.pose = endpoint
                self._settle_completed_trajectory_endpoint(robot, now)
                if (
                    handoffs >= 4
                    or not self._activate_rolling_prefetch(robot, now)
                ):
                    break
                handoffs += 1
                self._runtime_tick_route_clocks[robot.name] = (
                    robot.route_clock
                )
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
            if preflight_pending:
                blocked_reason = self._blocked_ahead(robot, proposed_clock)
                self._mark_runtime_collision_preflight(robot, now)
            else:
                blocked_reason = self._blocked_at_clock(
                    robot,
                    proposed_clock,
                )
            preflight_pending = False
            if blocked_reason:
                break
            robot.route_clock = proposed_clock
            remaining -= motion_dt
            moved_during_tick = True
        return _MotionClockResult(
            blocked_reason=blocked_reason,
            moved=moved_during_tick,
        )

    def _hold_robot_for_runtime_collision(
        self,
        robot: FleetRobot,
        now: float,
        blocked_reason: str,
    ) -> None:
        pose = self._pose_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        )
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
            and not self._central_corridor_manages_wait(robot)
            and not self._wait_expected_to_clear(robot)
            and robot.traffic_stall_since is not None
            and now - robot.traffic_stall_since
            >= self._blocked_replan_after(blocked_reason)
        ):
            self._schedule_runtime_replan(robot, now, blocked_reason)
        self._update_active_order_from_robot(robot)

    def _commit_runtime_motion_state(
        self,
        robot: FleetRobot,
        now: float,
        *,
        route_clock_before: float,
        moved_during_tick: bool,
    ) -> None:
        pose = self._pose_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        )
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
            if (
                moved_during_tick
                or robot.route_clock > route_clock_before + 0.000001
            ):
                self._discard_runtime_replan_after_progress(robot)
                self._record_traffic_progress(robot)
                robot.updated_at = now
        self._update_active_order_from_robot(robot)
        self._complete_runtime_route_if_due(robot, now)

    def _complete_runtime_route_if_due(
        self,
        robot: FleetRobot,
        now: float,
    ) -> None:
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        if final_time <= 0.0 or robot.route_clock < final_time:
            return
        self._settle_completed_trajectory_endpoint(robot, now)
        if self._activate_rolling_prefetch(robot, now):
            return
        if self._complete_simulated_route_chunk(robot, now):
            robot.last_tick_at = now
            return
        self._finish_simulated_route_at_target(robot, now)

    def _settle_completed_trajectory_endpoint(
        self,
        robot: FleetRobot,
        now: float,
    ) -> None:
        """Commit the physical trajectory endpoint as the rolling boundary.

        A target label is intent; the terminal trajectory sample is the
        executable contract.  Keeping an older target after a rolling trim
        makes continuation wait for an LM the robot never reached.  Repair
        such legacy/in-flight metadata atomically at the graph boundary.
        """
        self._update_current_lm_from_trajectory(robot)
        if not robot.trajectory:
            return
        endpoint_lm = str(robot.trajectory[-1].get("lm") or "").strip()
        if endpoint_lm not in self.landmarks:
            return
        if robot.pose is not None and not self._pose_is_at_lm(
            robot.pose,
            endpoint_lm,
        ):
            return
        robot.current_lm = endpoint_lm
        expected_lm = str(
            robot.route_chunk_goal_lm or robot.target_lm or ""
        ).strip()
        if (
            not robot.active_order_id
            or not expected_lm
            or expected_lm == endpoint_lm
        ):
            return

        robot.target_lm = endpoint_lm
        robot.route_chunk_goal_lm = endpoint_lm
        if endpoint_lm in robot.plan_nodes:
            terminal_index = max(
                index
                for index, node in enumerate(robot.plan_nodes)
                if node == endpoint_lm
            )
            robot.plan_nodes = robot.plan_nodes[: terminal_index + 1]
        order = self.orders.get(robot.active_order_id)
        if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
            order.route_nodes = list(robot.plan_nodes)
            order.start_lm = endpoint_lm
            order.updated_at = now
        robot.pending_route = None
        robot.route_revision = self._next_route_revision()
        robot.trajectory_dirty = True
        robot.route_preview_dirty = True
        robot.updated_at = now
        self._clear_rolling_prefetch_state(robot.name)
        self._event(
            "warn",
            f"{robot.name} rolling endpoint normalised: "
            f"{expected_lm}->{endpoint_lm}",
        )

    def _settle_degenerate_simulated_route(
        self,
        robot: FleetRobot,
        now: float,
    ) -> bool:
        """Resolve committed routes which have no executable time interval.

        A one-node SIPP result is a valid representation of "already here",
        but it is not motion.  Treating every non-empty trajectory as MOVING
        left the order EXECUTING forever because the ordinary completion path
        only ran for a positive final timestamp.  The stuck robot then kept a
        false traffic reservation and blocked otherwise healthy followers.

        Return ``True`` when this tick has been fully handled.  A prefetched
        positive-duration continuation returns ``False`` so normal motion may
        consume it immediately in the same physics tick.
        """
        if not robot.trajectory:
            return False

        handoffs = 0
        while robot.trajectory:
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            if final_time > 0.000001:
                return False

            endpoint_lm = self._safe_degenerate_endpoint_lm(robot)
            if endpoint_lm:
                robot.current_lm = endpoint_lm
                endpoint_pose = self._pose_from_sample(robot.trajectory[-1])
                # A zero-time route may confirm the graph position, but it
                # cannot physically perform an instantaneous rotation.  Keep
                # the already committed heading; an actual turn must have a
                # positive-duration trajectory of its own.
                if robot.pose is not None:
                    endpoint_pose["yaw"] = float(
                        robot.pose.get("yaw", 0.0) or 0.0
                    )
                robot.pose = endpoint_pose

            order = (
                self.orders.get(robot.active_order_id)
                if robot.active_order_id
                else None
            )
            if order is not None and order.status in TERMINAL_ORDER_STATUSES:
                order = None
            final_target = (
                self._active_order_target(order)
                if order is not None
                else str(robot.route_final_lm or robot.target_lm or "").strip()
            )
            if final_target and self._degenerate_route_reached_lm(
                robot,
                final_target,
                endpoint_lm,
            ):
                robot.current_lm = final_target
                self._finish_simulated_route_at_target(robot, now)
                return True

            chunk_target = str(
                robot.route_chunk_goal_lm or robot.target_lm or ""
            ).strip()
            if (
                order is not None
                and chunk_target
                and self._degenerate_route_reached_lm(
                    robot,
                    chunk_target,
                    endpoint_lm,
                )
            ):
                robot.current_lm = chunk_target
                if handoffs < 4 and self._activate_rolling_prefetch(robot, now):
                    handoffs += 1
                    continue
                if self._complete_simulated_route_chunk(robot, now):
                    robot.last_tick_at = now
                    return True

            reason = "degenerate route does not reach active target"
            previous_status = robot.status
            previous_reason = robot.last_reason
            replanning = self._queue_active_order_for_background_replan(
                robot,
                now,
                reason,
            )
            if not replanning:
                robot.status = "WAITING" if order is not None else "BLOCKED"
                robot.last_reason = f"holding for replan: {reason}"
                robot.last_tick_at = now
                robot.blocked_since = robot.blocked_since or now
                robot.traffic_priority_until = 0.0
                self._clear_wait_dependency(robot)
                robot.updated_at = now
                self._update_active_order_from_robot(robot)
            if previous_status == "MOVING" or previous_reason != robot.last_reason:
                self._event(
                    "warn",
                    f"{robot.name} rejected zero-duration motion: {reason}",
                )
            return True
        return False

    def _safe_degenerate_endpoint_lm(self, robot: FleetRobot) -> str:
        """Return a zero-duration endpoint only when it is physically local.

        Multiple different poses at the same timestamp are an invalid instant
        teleport, not an arrival.  A single sample (the normal already-there
        result), or co-located duplicate samples, can safely identify the LM.
        """
        if not robot.trajectory:
            return ""
        endpoint = robot.trajectory[-1]
        endpoint_lm = str(endpoint.get("lm") or "").strip()
        if endpoint_lm not in self.landmarks:
            return ""
        endpoint_pose = self._pose_from_sample(endpoint)
        tolerance = self._runtime_replan_lm_tolerance()
        if robot.pose is not None and math.hypot(
            endpoint_pose["x"] - float(robot.pose.get("x", 0.0) or 0.0),
            endpoint_pose["y"] - float(robot.pose.get("y", 0.0) or 0.0),
        ) > tolerance:
            return ""
        for sample in robot.trajectory:
            sample_pose = self._pose_from_sample(sample)
            if math.hypot(
                endpoint_pose["x"] - sample_pose["x"],
                endpoint_pose["y"] - sample_pose["y"],
            ) > tolerance:
                return ""
        return endpoint_lm

    def _degenerate_route_reached_lm(
        self,
        robot: FleetRobot,
        target_lm: str,
        endpoint_lm: str,
    ) -> bool:
        if target_lm not in self.landmarks:
            return False
        if robot.current_lm == target_lm or endpoint_lm == target_lm:
            return True
        return bool(robot.pose and self._pose_is_at_lm(robot.pose, target_lm))

    def _finish_simulated_route_at_target(
        self,
        robot: FleetRobot,
        now: float,
    ) -> None:
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
        robot.traffic_stall_since = None
        robot.traffic_priority_until = 0.0
        robot.last_reason = "arrived"
        robot.route_note = ""
        robot.updated_at = now
        self._clear_wait_dependency(robot)
        self._clear_remote_route_metadata(robot)
        self._event("info", f"{robot.name} arrived at {robot.current_lm}")
