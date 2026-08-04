"""Deadlock retreat advancement and recovery lifecycle."""

from __future__ import annotations

from dataclasses import dataclass

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot


@dataclass(slots=True)
class _BlockedRetreatRecovery:
    """Stable causal inputs for one blocked-retreat recovery decision."""

    blocker_name: str
    blocker: FleetRobot | None
    start_lm: str
    reverse_edges: list[tuple[str, str]]
    escape_blocked_edges: list[tuple[str, str]]


class FleetDeadlockRetreatMixin:
    """Run, recover and clear deadlock-retreat transactions."""

    def _advance_deadlock_retreat(self, robot: FleetRobot, now: float) -> None:
        target_clock = robot.retreat_target_clock
        if target_clock is None or not robot.trajectory:
            self._clear_deadlock_retreat(robot)
            return
        if self._retreat_waits_for_priority(robot, now):
            robot.last_tick_at = now
            robot.updated_at = now
            return
        robot.traffic_priority_until = max(
            robot.traffic_priority_until,
            now + self._deadlock_priority_lease(),
        )
        blocked_reason = self._consume_deadlock_retreat_clock(
            robot,
            target_clock,
            now,
        )
        pose = self._pose_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        )
        if pose is not None:
            robot.pose = pose
        self._update_current_lm_from_trajectory(robot)
        robot.updated_at = now
        if blocked_reason:
            self._hold_blocked_deadlock_retreat(
                robot,
                blocked_reason,
                now,
            )
            return
        if robot.route_clock > target_clock + 0.000001:
            robot.status = "RETREATING"
            robot.last_reason = (
                f"deadlock retreat to {robot.retreat_target_lm}"
            )
            return
        self._finish_deadlock_retreat(robot, now, pose)

    @staticmethod
    def _retreat_waits_for_priority(
        robot: FleetRobot,
        now: float,
    ) -> bool:
        return bool(
            robot.status == "WAITING"
            and str(robot.last_reason or "").startswith("yield to ")
            and robot.wait_for_robot
            and now < robot.wait_release_at
        )

    def _consume_deadlock_retreat_clock(
        self,
        robot: FleetRobot,
        target_clock: float,
        now: float,
    ) -> str:
        """Consume a bounded reverse-motion slice and return any blocker."""

        last_tick_at = robot.last_tick_at or now
        remaining = min(
            0.20 * self.simulation_time_scale(),
            max(0.0, now - last_tick_at),
        )
        robot.last_tick_at = now
        blocked_reason = ""
        retreat_clock_before = float(robot.route_clock)
        while (
            remaining > 0.000001
            and robot.route_clock > target_clock + 0.000001
        ):
            motion_dt = min(self._runtime_motion_step(), remaining)
            proposed_clock = max(target_clock, robot.route_clock - motion_dt)
            blocked_reason = self._blocked_at_clock(
                robot,
                proposed_clock,
                ignore_admission=True,
            )
            if blocked_reason:
                break
            robot.route_clock = proposed_clock
            remaining -= motion_dt

        if robot.route_clock < retreat_clock_before - 0.000001:
            # A blocker encountered later on the reverse path is a new stall,
            # not a continuation of the wait cycle which armed the retreat.
            robot.blocked_since = None
            robot.traffic_stall_since = None
            self._clear_wait_dependency(robot)
        return blocked_reason

    def _hold_blocked_deadlock_retreat(
        self,
        robot: FleetRobot,
        blocked_reason: str,
        now: float,
    ) -> None:
        robot.status = "RETREATING"
        robot.last_reason = f"deadlock retreat waiting: {blocked_reason}"
        self._set_wait_dependency(robot, blocked_reason, now)
        robot.blocked_since = robot.blocked_since or now
        robot.traffic_stall_since = robot.traffic_stall_since or now
        if (
            now - robot.blocked_since
            >= self._deadlock_retreat_block_timeout()
        ):
            self._recover_blocked_deadlock_retreat(
                robot,
                blocked_reason,
                now,
            )

    def _finish_deadlock_retreat(
        self,
        robot: FleetRobot,
        now: float,
        pose: dict[str, float] | None,
    ) -> None:
        """Commit a graph-safe retreat endpoint and queue same-goal recovery."""

        target_lm = robot.retreat_target_lm
        blocked_edges = list(robot.retreat_blocked_edges)
        corridor_hold = (
            dict(robot.retreat_corridor_hold)
            if isinstance(robot.retreat_corridor_hold, dict)
            else None
        )
        causal_blocker_signatures = (
            self._retreat_causal_blocker_signatures(robot)
        )
        if target_lm in self.landmarks:
            robot.current_lm = target_lm
            landmark_pose = self._pose_at_landmark(target_lm)
            # The trajectory sample carries the orientation in which the
            # robot physically reached this LM.  Replacing it with the
            # landmark helper's synthetic yaw=0 can rotate a rectangular
            # footprint by 90 degrees in one frame and manufacture an overlap
            # with a robot on the neighbouring lane. Snap only the position
            # to the graph LM and preserve the physical arrival orientation.
            if pose is not None and landmark_pose is not None:
                robot.pose = {
                    "x": float(landmark_pose["x"]),
                    "y": float(landmark_pose["y"]),
                    "yaw": float(pose.get("yaw", 0.0) or 0.0),
                }
            elif pose is None:
                robot.pose = landmark_pose
        self._clear_deadlock_retreat(robot)
        order = self._active_order_for_robot(robot)
        if order is not None:
            order.traffic_detour_edges = list(dict.fromkeys(blocked_edges))
            order.traffic_detour_attempts += 1
        replan_handled, replan_started = (
            self._queue_background_replan_recovery_action(
                robot,
                now,
                "deadlock corridor evacuated; alternate route required",
            )
        )
        if causal_blocker_signatures:
            self._attach_retreat_blocker_dependencies(
                robot,
                order,
                target_lm,
                causal_blocker_signatures,
            )
        if corridor_hold:
            replan_state = self._runtime_replans.get(robot.name)
            if isinstance(replan_state, dict):
                # Keep the cleared pocket occupied until the owner has
                # physically left the local corridor resource.  Planning the
                # original goal sooner can send the tail straight back into
                # the queue which it has just opened.
                replan_state["corridor_clearance_hold"] = corridor_hold
        if replan_handled:
            if replan_started:
                self.traffic_metrics["cycleReplans"] += 1
                self._event(
                    "warn",
                    f"{robot.name} retreated to {target_lm}; detour to the same goal queued",
                )
            return
        robot.status = "WAITING"
        robot.last_reason = "deadlock retreat complete; detour queue pending"

    def _retreat_causal_blocker_signatures(
        self,
        robot: FleetRobot,
    ) -> tuple[tuple[str, str, int], ...]:
        return tuple(
            (
                str(signature[0]),
                str(signature[1]),
                int(signature[2]),
            )
            for signature in robot.retreat_blocker_signatures
            if (
                isinstance(signature, (list, tuple))
                and len(signature) == 3
                and str(signature[0]) in self.robots
                and str(signature[0]) != robot.name
            )
        )

    def _attach_retreat_blocker_dependencies(
        self,
        robot: FleetRobot,
        order: FleetOrder | None,
        target_lm: str,
        signatures: tuple[tuple[str, str, int], ...],
    ) -> None:
        replan_state = self._runtime_replans.get(robot.name)
        if not isinstance(replan_state, dict):
            return
        replan_state["causal_blocker_signatures"] = signatures
        replan_state["blocker_names"] = tuple(
            signature[0] for signature in signatures
        )
        retry_state = (
            self._stationary_order_retry_state.get(order.order_id)
            if order is not None
            else None
        )
        if not isinstance(retry_state, dict):
            return
        staged_signatures = tuple(
            retry_state.get("waiter_escape_in_flight", ())
        )
        staged_target = str(
            retry_state.get("waiter_escape_target_lm") or ""
        )
        if staged_signatures != signatures or staged_target != target_lm:
            return

        for blocker_name, blocker_lm, blocker_revision in signatures:
            blocker = self.robots.get(blocker_name)
            if (
                not self._inactive_stationary_clearance_candidate(
                    blocker,
                    exclude_name=robot.name,
                )
                or self._traffic_lm_for_robot(blocker) != blocker_lm
                or int(blocker.route_revision) != blocker_revision
            ):
                continue
            if self._queue_stationary_clearance_relocation(
                robot,
                blocker,
                cause=f"staged waiter escape completed at {target_lm}",
            ):
                replan_state["clearance_blocker_names"] = (blocker_name,)
                replan_state["stage"] = "queued"
                break
        retry_state.pop("waiter_escape_in_flight", None)
        retry_state.pop("waiter_escape_target_lm", None)

    def _deadlock_retreat_block_timeout(self) -> float:
        """Maximum stable wait before replacing an unusable retreat."""
        return max(
            1.0,
            self._deadlock_wait_timeout(),
            self._deadlock_priority_lease(),
        )

    def _recover_blocked_deadlock_retreat(
        self,
        robot: FleetRobot,
        blocked_reason: str,
        now: float,
    ) -> bool:
        """Replace a reverse path blocked after it was committed.

        Other robots can enter an old trajectory after the initial deadlock
        decision.  A reverse traversal must therefore have a bounded runtime
        failure mode as well as pre-commit validation.  At a graph LM we first
        try a short escape to a legal waiting pocket, then transactionally
        replan the same active order.  If neither operation is safe (for
        example the body is still mid-edge), abort only the retreat marker and
        let the unchanged forward trajectory pass through normal collision
        preflight on the next tick.
        """
        recovery = self._blocked_retreat_recovery(robot, blocked_reason)
        if self._replace_blocked_retreat_with_graph_escape(
            robot,
            recovery,
            now,
        ):
            return True

        order = self._active_order_for_robot(robot)
        if order is not None and recovery.reverse_edges:
            # Retaining both the old exit ban and the newly blocked reverse
            # edge can disconnect a degree-two aisle completely.
            order.traffic_detour_edges = list(recovery.reverse_edges)
        if self._replace_blocked_retreat_with_replan(
            robot,
            recovery,
            order,
            now,
        ):
            return True
        self._abort_blocked_deadlock_retreat(robot, recovery.blocker_name, now)
        return True

    def _blocked_retreat_recovery(
        self,
        robot: FleetRobot,
        blocked_reason: str,
    ) -> _BlockedRetreatRecovery:
        """Capture the blocker and edge evidence before changing state."""
        blocker_name = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(blocked_reason)
        )
        blocker = self.robots.get(blocker_name)
        start_lm = self._safe_replan_start_lm(robot)
        reverse_edges = self._blocked_retreat_segment_edges(robot)
        old_blocked_edges = list(robot.retreat_blocked_edges)
        escape_blocked_edges = list(
            dict.fromkeys([*old_blocked_edges, *reverse_edges])
        )
        return _BlockedRetreatRecovery(
            blocker_name=blocker_name,
            blocker=blocker,
            start_lm=start_lm,
            reverse_edges=reverse_edges,
            escape_blocked_edges=escape_blocked_edges,
        )

    def _replace_blocked_retreat_with_graph_escape(
        self,
        robot: FleetRobot,
        recovery: _BlockedRetreatRecovery,
        now: float,
    ) -> bool:
        """Try the legal pocket escape before altering the active order."""
        if not recovery.start_lm or recovery.blocker is None:
            return False
        blocker_lm = self._traffic_lm_for_robot(recovery.blocker)
        blocker_edges = (
            self._blocked_edges_for_lms({blocker_lm})
            if blocker_lm in self.landmarks
            else set()
        )
        escape_route = self._stationary_clearance_route(
            recovery.blocker,
            robot,
            forbidden_lms={blocker_lm},
            extra_blocked_edges=blocker_edges,
            avoid_controlled_regions=True,
            start_lm_override=recovery.start_lm,
        )
        if (
            len(escape_route) < 2
            or escape_route[0] != recovery.start_lm
            or not self._install_graph_escape_retreat(
                robot,
                escape_route,
                recovery.escape_blocked_edges,
                now,
            )
        ):
            return False
        current_blocker = self.robots.get(recovery.blocker_name)
        robot.retreat_blocker_signatures = (
            [(
                recovery.blocker_name,
                self._traffic_lm_for_robot(current_blocker),
                int(current_blocker.route_revision),
            )]
            if current_blocker is not None
            and recovery.blocker_name != robot.name
            else []
        )
        self._release_mutual_retreat_wait(
            robot,
            recovery.blocker_name,
            now,
        )
        self._event(
            "warn",
            f"{robot.name} blocked retreat replaced with graph escape "
            f"to {escape_route[-1]}",
        )
        return True

    def _replace_blocked_retreat_with_replan(
        self,
        robot: FleetRobot,
        recovery: _BlockedRetreatRecovery,
        order: FleetOrder | None,
        now: float,
    ) -> bool:
        """Fall back to a same-goal replan from a graph-safe pose."""
        if not recovery.start_lm:
            return False
        replan_handled, replan_started = (
            self._queue_background_replan_recovery_action(
                robot,
                now,
                "deadlock retreat blocked; alternate route required",
            )
        )
        if not replan_handled:
            return False
        self._clear_deadlock_retreat(robot)
        self._release_mutual_retreat_wait(
            robot,
            recovery.blocker_name,
            now,
        )
        if replan_started:
            if order is not None:
                order.traffic_detour_attempts += 1
            self.traffic_metrics["cycleReplans"] += 1
        self._event(
            "warn",
            f"{robot.name} blocked retreat replaced with same-goal replan",
        )
        return True

    def _abort_blocked_deadlock_retreat(
        self,
        robot: FleetRobot,
        blocker_name: str,
        now: float,
    ) -> None:
        """Keep the forward contract when no graph-safe recovery can start."""
        self._clear_deadlock_retreat(robot)
        robot.status = "WAITING"
        robot.last_reason = "blocked retreat aborted; collision preflight pending"
        robot.blocked_since = None
        robot.traffic_stall_since = None
        robot.traffic_priority_until = 0.0
        robot.collision_preflight_due_at = 0.0
        robot.last_tick_at = now
        self._clear_wait_dependency(robot)
        robot.updated_at = now
        self._release_mutual_retreat_wait(robot, blocker_name, now)
        self._event(
            "warn",
            f"{robot.name} blocked retreat aborted; committed route retained",
        )

    def _blocked_retreat_segment_edges(
        self,
        robot: FleetRobot,
    ) -> list[tuple[str, str]]:
        target_clock = (
            float(robot.retreat_target_clock)
            if robot.retreat_target_clock is not None
            else float(robot.route_clock)
        )
        check_clock = max(
            target_clock,
            float(robot.route_clock) - self._runtime_motion_step(),
        )
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, check_clock)
        )
        if edge is None:
            return []
        source, destination = edge
        return list(dict.fromkeys([
            (source, destination),
            (destination, source),
        ]))

    def _release_mutual_retreat_wait(
        self,
        robot: FleetRobot,
        blocker_name: str,
        now: float,
    ) -> None:
        """Invalidate the peer half of an obsolete mutual-yield decision."""
        blocker = self.robots.get(blocker_name)
        if blocker is None:
            return
        blocker_dependency = (
            str(blocker.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(blocker.last_reason)
        )
        if blocker_dependency != robot.name:
            return
        blocker.status = "WAITING"
        blocker.last_reason = "retreat dependency changed; collision preflight pending"
        blocker.blocked_since = None
        blocker.traffic_stall_since = None
        blocker.collision_preflight_due_at = 0.0
        self._clear_wait_dependency(blocker)
        blocker.updated_at = now

    def _clear_deadlock_retreat(self, robot: FleetRobot) -> None:
        robot.retreat_target_clock = None
        robot.retreat_target_lm = ""
        robot.retreat_blocked_edges = []
        robot.retreat_blocker_signatures = []
        robot.retreat_corridor_hold = None
