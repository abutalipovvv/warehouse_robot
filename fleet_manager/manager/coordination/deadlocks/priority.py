"""Deterministic traffic priority grants and progress tracking."""

from __future__ import annotations

from fleet_manager.robot.model import FleetRobot


class DeadlockPriorityMixin:
    """Track progress and grant bounded priority to stalled robots and chains."""

    def _record_traffic_progress(self, robot: FleetRobot) -> None:
        robot.traffic_stall_since = None
        for cycle_key in list(self._active_wait_cycles):
            if robot.name not in cycle_key:
                continue
            peers = [
                peer
                for name in cycle_key
                if name != robot.name
                and (peer := self.robots.get(name)) is not None
            ]
            final_time = (
                float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                if robot.trajectory
                else robot.route_clock
            )
            remaining = max(0.0, final_time - robot.route_clock)
            if (
                peers
                and remaining > 0.000001
                and self._cycle_forward_clearance(
                    robot,
                    [robot, *peers],
                )
                + self._runtime_motion_step()
                < remaining
            ):
                # A tiny priority-step toward the same stationary body is not
                # deadlock progress. Preserve the episode age so the entrant
                # reaches deterministic corridor evacuation instead of
                # receiving the same grant forever.
                robot.traffic_stall_since = self._active_wait_cycles[cycle_key]
                continue
            if cycle_key in self._wait_cycle_grant_signatures:
                # A priority grant is only an attempted arbitration. Count a
                # resolved cycle after the selected robot has produced enough
                # spatial progress to change the component geometry.
                self.traffic_metrics["waitCyclesResolved"] += 1
            self._active_wait_cycles.pop(cycle_key, None)
            self._wait_cycle_last_arbitration.pop(cycle_key, None)
            self._wait_cycle_grant_signatures.pop(cycle_key, None)
            self._clear_coupled_replan_attempts_for_members(cycle_key)
            self._clear_wait_cycle_recovery_attempts(cycle_key)
            # The geometry of the complete component changed. Give its peers
            # a fresh bounded arbitration window instead of carrying an old
            # stall timestamp into a new cycle and arming another retreat on
            # the very next 10 Hz tick.
            for peer_name in cycle_key:
                peer = self.robots.get(peer_name)
                if peer is not None:
                    peer.traffic_stall_since = None
                    peer.blocked_since = None

    def _refresh_runtime_priority_lease(self, robot: FleetRobot, now: float) -> None:
        """Make a traffic grant survive until it produces a physics step.

        A background Python planner can hold the GIL longer than a wall-clock
        lease. Re-arm only the still-unconsumed grant; the normal MOVING branch
        changes ``last_reason`` after the first real route-clock advance, so a
        successful robot does not retain priority indefinitely.
        """
        if (
            robot.status == "MOVING"
            and robot.trajectory
            and str(robot.last_reason or "") in {
                "deadlock priority active",
                "deadlock priority granted",
                "starvation priority active",
            }
        ):
            robot.traffic_priority_until = max(
                robot.traffic_priority_until,
                now + self._deadlock_priority_lease(),
            )

    def _grant_starvation_priority(
        self,
        winner: FleetRobot,
        blocker: FleetRobot,
        now: float,
    ) -> bool:
        """Let a mid-edge robot clear the conflict when it cannot replan yet."""
        if (
            winner.name == blocker.name
            or not winner.trajectory
            or not blocker.trajectory
            or not self._is_active_traffic(blocker)
        ):
            return False
        participants = [winner, blocker]
        downstream_clearer = self._controlled_corridor_downstream_clearer(
            participants,
        )
        if (
            self._controlled_corridor_scheduler is not None
            and downstream_clearer is None
            and (
                str(winner.last_reason or "").startswith(
                    "corridor admission wait at "
                )
                or str(blocker.last_reason or "").startswith(
                    "corridor admission wait at "
                )
            )
        ):
            # A scheduled red light is not starvation. The central calendar
            # already owns the order and entry time; a generic priority lease
            # would create a second, contradictory dispatcher.
            return True
        lease_until = now + self._deadlock_priority_lease()
        if downstream_clearer is not None:
            winner = downstream_clearer
            blocker = next(
                robot
                for robot in participants
                if robot.name != winner.name
            )
        # A→B where B is stopped at a corridor signal is not resolved by
        # granting A more priority: A still has B's body directly ahead.
        # Transfer the corridor signal to B (when this pair owns the complete
        # local conflict) and let B clear the stop line first.
        elif (
            str(blocker.last_reason or "").startswith(
                "corridor admission wait at "
            )
            and self._transfer_controlled_corridor_lease(
                blocker,
                participants,
                now,
            )
        ):
            winner, blocker = blocker, winner
        else:
            self._transfer_controlled_corridor_lease(
                winner,
                participants,
                now,
            )
        winner.status = "MOVING"
        winner.last_reason = "starvation priority active"
        winner.traffic_priority_until = max(
            winner.traffic_priority_until,
            lease_until,
        )
        self._clear_wait_dependency(winner)
        winner.updated_at = now

        blocker.status = "WAITING"
        blocker.last_reason = f"yield to {winner.name}"
        blocker.traffic_priority_until = 0.0
        blocker.wait_for_robot = winner.name
        blocker.wait_resource = self._edge_id_at_trajectory(
            blocker.trajectory,
            blocker.route_clock,
        )
        blocker.wait_release_at = lease_until
        # This is a new, explicit yield episode. Reset its age so priority does
        # not bounce straight back on the same frame merely because the former
        # winner carried an old cycle timestamp.
        blocker.blocked_since = now
        blocker.traffic_stall_since = now
        blocker.updated_at = now
        self.traffic_metrics["priorityGrants"] += 1
        self._event(
            "warn",
            f"traffic starvation resolved: priority transferred to {winner.name}; "
            f"{blocker.name} yields",
        )
        return True

    def _grant_wait_chain_priority(
        self,
        participants: list[FleetRobot],
        terminal: FleetRobot,
        now: float,
    ) -> bool:
        """Collapse A→B→C waits into one explicit component grant."""
        robots = list({robot.name: robot for robot in participants}.values())
        if len(robots) < 2 or any(not robot.trajectory for robot in robots):
            return False

        if terminal.traffic_priority_until > now:
            # A gate may have changed the visible WAIT reason since the
            # previous arbitration, but the unexpired sink lease is still the
            # same decision. Preserve/reapply it without counting a new grant.
            self._transfer_controlled_corridor_lease(terminal, robots, now)
            return True

        terminal_reason = str(terminal.last_reason or "")
        if (
            terminal_reason == "deadlock recovery pending"
            or terminal_reason.startswith(
                ("traffic admission wait at ", "corridor admission wait at ")
            )
        ):
            # Admission control or the wait-cycle recovery debounce already
            # owns this sink. Reissuing a physical priority lease cannot open
            # the occupied region and would recreate the same cycle.
            return True

        # In an acyclic wait chain only the sink can create space. Granting an
        # upstream robot with a longer geometric suffix merely commands it to
        # drive into the stationary body ahead (the live A→B→C failure mode).
        # Preserve an existing lease only when it already belongs to the sink.
        terminal_stall = terminal.traffic_stall_since or terminal.blocked_since
        terminal_grant_is_fresh = (
            terminal_stall is None
            or now - terminal_stall < self._deadlock_coupled_replan_after()
        )
        if terminal.traffic_priority_until > now and terminal_grant_is_fresh:
            return True
        terminal.traffic_priority_until = 0.0
        for robot in robots:
            if robot.name != terminal.name:
                robot.traffic_priority_until = 0.0

        # An exhausted rolling trajectory cannot consume a motion lease. Keep
        # the complete dependency chain pointed at that terminal and make its
        # continuation the dispatcher's next direct prefetch target.
        if self._robot_waits_at_rolling_boundary(terminal):
            self._rolling_prefetch_retry_at.pop(terminal.name, None)
            terminal.rolling_boundary_since = (
                terminal.rolling_boundary_since or now
            )
            self._rolling_prefetch_eligible_since.setdefault(
                terminal.name,
                terminal.rolling_boundary_since,
            )
            terminal.status = "WAITING"
            terminal.last_reason = "rolling continuation pending"
            terminal.traffic_priority_until = 0.0
            self._clear_wait_dependency(terminal)
            terminal.updated_at = now
            for robot in robots:
                if robot.name == terminal.name:
                    continue
                robot.status = "WAITING"
                robot.last_reason = f"yield to {terminal.name}"
                robot.wait_for_robot = terminal.name
                robot.wait_resource = self._edge_id_at_trajectory(
                    robot.trajectory,
                    robot.route_clock,
                )
                robot.wait_release_at = 0.0
                robot.blocked_since = robot.blocked_since or now
                robot.traffic_stall_since = robot.traffic_stall_since or now
                robot.updated_at = now
            return True

        winner = terminal
        lease_until = now + self._deadlock_priority_lease()
        self._transfer_controlled_corridor_lease(winner, robots, now)
        winner.status = "MOVING"
        winner.last_reason = "starvation priority active"
        winner.traffic_priority_until = max(winner.traffic_priority_until, lease_until)
        self._clear_wait_dependency(winner)
        winner.updated_at = now
        for robot in robots:
            if robot.name == winner.name:
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {winner.name}"
            robot.traffic_priority_until = 0.0
            robot.wait_for_robot = winner.name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = lease_until
            robot.blocked_since = robot.blocked_since or now
            robot.traffic_stall_since = robot.traffic_stall_since or now
            robot.updated_at = now
        self.traffic_metrics["priorityGrants"] += 1
        self._event(
            "warn",
            f"traffic wait chain resolved: priority granted to {winner.name}; "
            f"{len(robots) - 1} robots yield",
        )
        return True

    def _evacuate_for_failed_stationary_departure(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        now: float,
    ) -> bool:
        """Break a head-on where a queued robot cannot leave its LM."""
        pending = self._active_order_for_robot(blocker)
        if (
            pending is None
            or pending.status not in {"QUEUED", "PLANNING"}
            or max(
                int(pending.dispatch_failures or 0),
                int(self._rolling_prefetch_failures.get(blocker.name, 0) or 0),
            )
            < 2
            or not waiter.trajectory
            or not waiter.active_order_id
        ):
            return False
        stalled_since = waiter.traffic_stall_since or waiter.blocked_since or now
        if now - stalled_since < self._deadlock_coupled_replan_after():
            return False
        # ``_start_deadlock_corridor_evacuation`` deliberately reports an
        # already queued transactional detour as handled.  That keeps the
        # ordinary starvation branch from competing with the same recovery,
        # but it is not a *new* evacuation.  At accelerated simulation time
        # the recovery debounce expires every few wall-clock frames; treating
        # that idempotent result as new used to refresh the waiter's priority
        # lease and emit ``evacuating for queued departure`` forever while its
        # route clock remained at zero.
        before_replan = self._runtime_replans.get(waiter.name)
        before_status = waiter.status
        before_retreat = (
            waiter.retreat_target_clock,
            waiter.retreat_target_lm,
        )
        pending_waiter_order = self._active_order_for_robot(waiter)
        before_detour_attempts = int(
            pending_waiter_order.traffic_detour_attempts
            if pending_waiter_order is not None
            else 0
        )
        evacuated_name = self._start_deadlock_corridor_evacuation(
            [blocker, waiter],
            blocker,
            now,
        )
        if evacuated_name != waiter.name:
            return False
        after_replan = self._runtime_replans.get(waiter.name)
        new_recovery_action = bool(
            after_replan is not before_replan
            or (
                waiter.status == "RETREATING"
                and (
                    before_status != "RETREATING"
                    or before_retreat
                    != (waiter.retreat_target_clock, waiter.retreat_target_lm)
                )
            )
            or (
                pending_waiter_order is not None
                and int(pending_waiter_order.traffic_detour_attempts)
                > before_detour_attempts
            )
        )
        if not new_recovery_action:
            # The existing transaction still owns this dependency.  Report it
            # as handled without renewing right-of-way or duplicating events.
            return True
        waiter.traffic_priority_until = max(
            waiter.traffic_priority_until,
            now + self._deadlock_priority_lease(),
        )
        self._event(
            "warn",
            f"{waiter.name} evacuating for queued departure {blocker.name}",
        )
        return True

    def _failed_rolling_boundary_departure(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether a stopped rolling handoff has repeatedly failed."""
        return (
            self._robot_waits_at_rolling_boundary(robot)
            and int(self._rolling_prefetch_failures.get(robot.name, 0) or 0)
            >= 2
        )

    def _wait_cycle_grant_signature(
        self,
        robots: list[FleetRobot],
    ) -> tuple[tuple[str, str, str, int], ...]:
        """Describe geometry for which one priority nudge was already tried."""
        return tuple(
            sorted(
                (
                    robot.name,
                    self._traffic_lm_for_robot(robot),
                    str(robot.active_order_id or ""),
                    int(robot.route_revision),
                )
                for robot in robots
            )
        )
