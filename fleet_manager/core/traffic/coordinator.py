"""Runtime traffic arbitration and deadlock resolution."""

from __future__ import annotations

import math
from time import time
from typing import Any

from fleet_manager.core.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import PlannedRoute


class TrafficCoordinatorMixin:
    """Coordinate waits, priorities, collision look-ahead and deadlocks."""

    def _resolve_runtime_wait_cycles(self, now: float) -> None:
        waiting = {
            robot.name: robot
            for robot in self._runtime_robots()
            if (
                robot.status == "WAITING"
                and bool(robot.trajectory)
                and self._is_robot_conflict(robot.last_reason)
                and not self._central_corridor_manages_wait(robot)
                and not (
                    robot.wait_resource == "blocked_retreat"
                    and now < robot.wait_release_at
                )
            )
        }
        wait_for = {
            name: robot.wait_for_robot or self._robot_name_from_conflict_reason(robot.last_reason)
            for name, robot in waiting.items()
        }
        handled: set[str] = set()
        cycle_members: set[str] = set()
        observed_cycle_keys: set[tuple[str, ...]] = set()
        for start_name in sorted(wait_for):
            chain: list[str] = []
            positions: dict[str, int] = {}
            current = start_name
            while current in wait_for and current not in handled:
                if current in positions:
                    cycle = chain[positions[current]:]
                    cycle_key = tuple(sorted(cycle))
                    observed_cycle_keys.add(cycle_key)
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

        # A queued detour or a completed rolling chunk can dissolve a cycle
        # without advancing one of its old trajectories.  Do not retain that
        # component's age/cooldown and misclassify a later encounter as the
        # same deadlock episode.
        for cycle_key in list(self._active_wait_cycles):
            if cycle_key in observed_cycle_keys:
                continue
            # A granted winner is MOVING for at least one physics frame, so
            # the wait-for cycle temporarily disappears even when it advances
            # only a few centimetres and blocks again at the same corridor
            # mouth. Keep that episode until the lease produces genuine
            # clearance (handled by _record_traffic_progress) or expires.
            if any(
                (
                    (member := self.robots.get(robot_name)) is not None
                    and member.traffic_priority_until > now
                )
                for robot_name in cycle_key
            ):
                continue
            if self._runtime_replan_holds_wait_cycle_geometry(cycle_key):
                # A single-robot replacement plan temporarily changes its UI
                # reason to "replanning route while holding", hiding one edge
                # of an otherwise unchanged physical wait cycle.  Keep the
                # episode age/CBS result until that exact transaction or graph
                # geometry changes; otherwise every failed same-goal attempt
                # restarts arbitration from zero and can postpone the already
                # armed graph escape forever.
                self._resume_hidden_wait_cycle_coupled_replan(
                    cycle_key,
                    now,
                )
                continue
            self._active_wait_cycles.pop(cycle_key, None)
            self._wait_cycle_last_arbitration.pop(cycle_key, None)
            self._wait_cycle_grant_signatures.pop(cycle_key, None)
            self._clear_coupled_replan_attempts_for_members(cycle_key)

        timeout = self._traffic_replan_after()
        chain_members: set[str] = set()
        for start_name, start_robot in waiting.items():
            if (
                start_name in cycle_members
                or start_name in chain_members
                or start_robot.blocked_since is None
                or now - start_robot.blocked_since < timeout
            ):
                continue
            chain: list[str] = []
            current = start_name
            while current in waiting and current not in chain:
                chain.append(current)
                current = wait_for.get(current, "")
            if any(name in cycle_members for name in chain):
                # This is an upstream tail feeding a cycle already handled by
                # _break_runtime_wait_cycle(). A second acyclic grant on the
                # stale beginning-of-tick snapshot would immediately override
                # that cycle winner and repeat on every physics frame.
                chain_members.update(chain)
                continue
            terminal = self.robots.get(current)
            if len(chain) < 2 or terminal is None:
                continue
            if not terminal.trajectory:
                # A→B→C where C is a queued/parked body has no motion lease
                # that an upstream robot can consume.  Mark the whole tail as
                # handled so the individual starvation branch below does not
                # repeatedly command A into B.  Release the immediate follower
                # once C's departure has demonstrably failed; for a genuinely
                # parked C, use the same bounded corridor evacuation directly.
                chain_members.update(chain)
                immediate = waiting[chain[-1]]
                if self._robot_departure_pending(terminal):
                    self._evacuate_for_failed_stationary_departure(
                        immediate,
                        terminal,
                        now,
                    )
                else:
                    stalled_since = (
                        immediate.traffic_stall_since
                        or immediate.blocked_since
                        or now
                    )
                    if now - stalled_since >= self._deadlock_retreat_after():
                        self._start_deadlock_corridor_evacuation(
                            [terminal, immediate],
                            terminal,
                            now,
                        )
                continue
            participants = [waiting[name] for name in chain] + [terminal]
            if self._grant_wait_chain_priority(participants, terminal, now):
                chain_members.update(chain)

        for robot in waiting.values():
            if robot.name in cycle_members or robot.name in chain_members:
                continue
            if robot.blocked_since is None or now - robot.blocked_since < timeout:
                continue
            blocker_name = robot.wait_for_robot or self._robot_name_from_conflict_reason(
                robot.last_reason,
            )
            blocker = self.robots.get(blocker_name)
            if blocker is not None and self._robot_departure_pending(blocker):
                # Do not turn a temporary dependency into a second parked
                # obstacle. The dispatcher gives this blocker the next
                # recovery slot. If repeated MAPF attempts could not release
                # it, however, this is a physical head-on rather than a
                # temporary wait: evacuate the incoming robot to its previous
                # graph LM so the commanded departure can actually move.
                evacuated = self._evacuate_for_failed_stationary_departure(
                    robot,
                    blocker,
                    now,
                )
                if evacuated or not self._failed_rolling_boundary_departure(
                    blocker,
                ):
                    continue
            replanned = False
            if self._safe_replan_start_lm(robot):
                replanned = self._schedule_runtime_replan(
                    robot,
                    now,
                    "traffic wait timeout",
                )
            if replanned:
                continue
            if blocker is not None:
                stalled_since = (
                    robot.traffic_stall_since
                    or robot.blocked_since
                    or now
                )
                if (
                    now - stalled_since >= self._deadlock_retreat_after()
                    and self._start_deadlock_corridor_evacuation(
                        [blocker, robot],
                        blocker,
                        now,
                    )
                ):
                    continue
                self._grant_starvation_priority(robot, blocker, now)

        # Admission control is a traffic light, not a permanent reservation.
        # A robot first waits at the graph LM outside the hot zone; if no slot
        # appears within the bounded interval it invalidates the spatial suffix
        # and tries another corridor to the same goal. This prevents an
        # over-capacity zone from freezing every upstream queue forever.
        admission_timeout = self._traffic_zone_replan_after()
        for robot in self._runtime_robots():
            if (
                robot.status != "WAITING"
                or not robot.trajectory
                or not str(robot.last_reason or "").startswith(
                    "traffic admission wait at "
                )
                or robot.blocked_since is None
                or now - robot.blocked_since < admission_timeout
            ):
                continue
            if self._safe_replan_start_lm(robot):
                self._schedule_runtime_replan(
                    robot,
                    now,
                    f"traffic admission timeout: {robot.last_reason}",
                )

        corridor_timeout = self._controlled_corridor_replan_after()
        for robot in self._runtime_robots():
            if (
                robot.status != "WAITING"
                or not robot.trajectory
                or not str(robot.last_reason or "").startswith(
                    "corridor admission wait at "
                )
                or robot.blocked_since is None
                or now - robot.blocked_since < corridor_timeout
            ):
                continue
            owner_name = self._robot_name_from_conflict_reason(
                robot.last_reason
            )
            owner = self.robots.get(owner_name)
            # A normal corridor queue is resolved by the stable passage
            # grant/FIFO ageing. Replanning every waiter after eight seconds
            # cleared valid trajectories, reset queue age and eventually
            # turned the whole fleet into route-less obstacles. Spatial
            # detour is reserved for a genuinely parked owner with no pending
            # departure; moving or commanded owners are allowed to clear.
            if (
                owner is not None
                and not self._is_active_traffic(owner)
                and not self._robot_departure_pending(owner)
                and self._safe_replan_start_lm(robot)
            ):
                self._schedule_runtime_replan(
                    robot,
                    now,
                    f"corridor admission timeout: {robot.last_reason}",
                )

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

        if str(terminal.last_reason or "").startswith(
            ("traffic admission wait at ", "corridor admission wait at ")
        ):
            # Admission control already owns this sink. Reissuing a physical
            # priority lease cannot open the occupied region and used to
            # count the identical upstream chain again on every physics tick.
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

    def _runtime_replan_holds_wait_cycle_geometry(
        self,
        cycle_key: tuple[str, ...],
    ) -> bool:
        """Keep a hidden wait episode only while its exact transaction is live."""
        robots = [
            robot
            for name in cycle_key
            if (robot := self.robots.get(name)) is not None
        ]
        if len(robots) != len(cycle_key):
            return False
        captured_grant = self._wait_cycle_grant_signatures.get(cycle_key)
        if (
            captured_grant is None
            or captured_grant != self._wait_cycle_grant_signature(robots)
        ):
            return False

        cycle_names = set(cycle_key)
        for robot in robots:
            state = self._runtime_replans.get(robot.name)
            if (
                not isinstance(state, dict)
                or not self._runtime_replan_state_is_current(
                    robot,
                    state,
                    allowed_stages={
                        "queued",
                        "planning",
                        "retry",
                        "deadlock_escalated",
                    },
                )
            ):
                continue
            blocker_names = {
                str(name)
                for name in state.get("blocker_names", ())
                if str(name)
            }
            escalated_blocker = str(
                state.get("escalated_blocker") or ""
            ).strip()
            if escalated_blocker:
                blocker_names.add(escalated_blocker)
            dynamic_signature = state.get("dynamic_conflict_signature")
            if isinstance(dynamic_signature, (list, tuple)) and dynamic_signature:
                blocker_names.add(str(dynamic_signature[0]))
            if blocker_names.intersection(cycle_names - {robot.name}):
                return True
        return False

    def _resume_hidden_wait_cycle_coupled_replan(
        self,
        cycle_key: tuple[str, ...],
        now: float,
    ) -> bool:
        """Run atomic MAPF after a transaction hides one wait-cycle edge.

        The retained trajectory is a rollback/safety snapshot once a corridor
        evacuation supersedes it. Planning each participant separately still
        reserves the other snapshots as executable futures, creating a false
        circular lock. The existing coupled planner removes those futures for
        the complete request set and commits all replacements atomically.
        """
        cycle_started = float(self._active_wait_cycles.get(cycle_key, now))
        if now - cycle_started < self._deadlock_coupled_replan_after():
            return False
        if self._async_simulated_dispatch_active():
            return False
        robots = [
            robot
            for name in cycle_key
            if (robot := self.robots.get(name)) is not None
        ]
        if len(robots) != len(cycle_key):
            return False

        promoted_transactions = [
            state
            for robot in robots
            if isinstance(
                state := self._runtime_replans.get(robot.name),
                dict,
            )
            and bool(state.get("retained_route_superseded"))
            and self._runtime_replan_state_is_current(
                robot,
                state,
                allowed_stages={
                    "queued",
                    "planning",
                    "retry",
                    "deadlock_escalated",
                },
            )
        ]
        if not promoted_transactions:
            return False
        newest_transaction = max(
            float(
                state.get("promoted_at")
                or state.get("queued_at")
                or 0.0
            )
            for state in promoted_transactions
        )
        previous_attempt = (
            self._coupled_replan_latest_attempt_for_members(cycle_key)
        )
        if (
            self._coupled_replan_failure_count_for_members(cycle_key) > 0
            and newest_transaction > previous_attempt + 0.000001
        ):
            # A graph-safe evacuation created a genuinely new start snapshot.
            # Permit exactly one CBS/SIPP attempt for that new transaction;
            # an unchanged failed transaction remains latched to avoid spin.
            self._clear_coupled_replan_attempts_for_members(cycle_key)

        corridor_owner = self._controlled_corridor_cycle_owner(robots)
        downstream_clearer = self._controlled_corridor_downstream_clearer(
            robots,
        )
        winner = downstream_clearer or corridor_owner or min(
            robots,
            key=lambda robot: (
                float(
                    (
                        self._runtime_replans.get(robot.name) or {}
                    ).get("queued_at", now)
                    or now
                ),
                robot.name,
            ),
        )
        return self._start_async_coupled_replan(robots, winner, now)

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

        def priority_key(robot: FleetRobot) -> tuple[float, int, float, float, str]:
            order = self._active_order_for_robot(robot)
            priority = int(order.priority if order is not None else 0)
            waited = now - (robot.blocked_since or now)
            forward_clearance = self._cycle_forward_clearance(robot, robots)
            # If a lease did not create progress, rotate equal-priority cycles
            # instead of granting the same robot forever. Geometry comes first:
            # granting a high-priority robot whose forward path ends in the
            # stationary loser's footprint cannot resolve the intersection.
            return (
                -forward_clearance,
                -priority,
                robot.traffic_priority_until,
                -waited,
                robot.name,
            )

        corridor_owner = self._controlled_corridor_cycle_owner(robots)
        downstream_clearer = self._controlled_corridor_downstream_clearer(
            robots,
        )
        winner = downstream_clearer or corridor_owner or min(
            robots,
            key=priority_key,
        )
        corridor_handoff_required = False
        if corridor_owner is not None:
            for robot in robots:
                if robot.name == corridor_owner.name:
                    continue
                passage = self._controlled_corridor_passages.get(robot.name)
                if (
                    isinstance(passage, dict)
                    and self._controlled_corridor_follower_yields_to(
                        robot,
                        corridor_owner,
                        passage,
                    )
                ):
                    corridor_handoff_required = True
                    break
        cycle_key = tuple(sorted(robot.name for robot in robots))
        grant_signature = self._wait_cycle_grant_signature(robots)
        previous_grant_signature = self._wait_cycle_grant_signatures.get(
            cycle_key
        )
        grant_already_failed = bool(
            not new_episode
            and (
                previous_grant_signature == grant_signature
                or (
                    previous_grant_signature is None
                    and cycle_key in self._wait_cycle_last_arbitration
                )
            )
        )
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

        # Once the first lease has demonstrably failed, the lease maintainer
        # deliberately expires it so CBS/retreat can run.  The same unchanged
        # wait-for snapshot is still visible at 10 Hz, however; without this
        # gate it would receive another nominal grant on every frame.  One
        # arbitration per lease interval is enough to react immediately while
        # leaving time for the chosen robot or background recovery to move.
        last_arbitration = self._wait_cycle_last_arbitration.get(cycle_key, 0.0)
        if (
            not new_episode
            and now - last_arbitration
            < self._deadlock_priority_lease()
        ):
            return
        self._wait_cycle_last_arbitration[cycle_key] = now

        if (
            corridor_owner is None
            and cycle_wait >= self._deadlock_coupled_replan_after()
        ):
            self._start_async_coupled_replan(robots, winner, now)
        # Retreat changes the spatial route and is intentionally the final
        # recovery level. Give dependency ordering and at least one local CBS
        # attempt time to resolve the coupled component first.
        evacuating_name = ""
        if (
            corridor_owner is not None
            and cycle_wait >= self._deadlock_retreat_after()
        ):
            # A complete wait cycle proves that the selected passage owner did
            # not clear. Scheduled WAIT/ROTATE samples can make geometric
            # forward-clearance appear positive even though the first eventual
            # motion intersects the losing portal body, so elapsed no-progress
            # time—not a zero-distance test—is the authoritative trigger.
            evacuating_name = self._start_deadlock_corridor_evacuation(
                robots,
                winner,
                now,
            )
        elif (
            cycle_wait >= self._deadlock_retreat_after()
            and self._coupled_replan_failure_count_for_members(cycle_key) > 0
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
        cycle_names = {robot.name for robot in robots}
        if (
            evacuating_robot is not None
            and evacuating_robot.name not in cycle_names
        ):
            # The portal itself cannot move until a physical queue outside the
            # cycle has been opened from its tail.  That tail now owns one
            # bounded reverse manoeuvre, but it is not the corridor winner and
            # must never receive/steal the owner's passage lease.  Keep the
            # original reciprocal pair steadily stopped until the next
            # arbitration observes real queue progress.
            for robot in robots:
                robot.traffic_priority_until = 0.0
                robot.updated_at = now
            return
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
        if grant_already_failed and not (
            evacuating_robot is not None
            and evacuating_robot.status == "RETREATING"
            and bool(evacuating_robot.trajectory)
        ) and not corridor_handoff_required:
            # The same winner already consumed a complete lease without a
            # graph/route change. Keep the component steadily stopped while CBS
            # or a transactional recovery is pending; another nudge can only
            # recreate the same centimetre-scale twitch and metric/event spam.
            for robot in robots:
                robot.traffic_priority_until = 0.0
            return
        if not priority_robot.trajectory:
            eligible = [robot for robot in robots if robot.trajectory]
            if not eligible:
                return
            priority_robot = min(eligible, key=priority_key)
        lease_transferred = self._transfer_controlled_corridor_lease(
            priority_robot,
            robots,
            now,
        )
        if corridor_handoff_required and not lease_transferred:
            # Never tell the physical leader to move while the follower still
            # owns the atomic passage. That would bypass the corridor gate and
            # retain two contradictory authorities until the next tick.
            for robot in robots:
                robot.traffic_priority_until = 0.0
                robot.updated_at = now
            return
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
        self._wait_cycle_grant_signatures[cycle_key] = (
            self._wait_cycle_grant_signature(robots)
        )
        self.traffic_metrics["priorityGrants"] += 1
        self._event(
            "warn",
            f"traffic wait cycle resolved: priority granted to {priority_robot.name}",
        )

    def _controlled_corridor_cycle_owner(
        self,
        robots: list[FleetRobot],
    ) -> FleetRobot | None:
        """Return a physical or central-calendar corridor cycle owner."""
        by_name = {robot.name: robot for robot in robots}
        physically_owned_regions: dict[str, set[str]] = {
            name: set()
            for name in by_name
        }
        for region_id, owner_names in self._controlled_corridor_occupancy.items():
            for owner_name in owner_names:
                if owner_name in physically_owned_regions:
                    physically_owned_regions[owner_name].add(str(region_id))

        physical_owners: list[FleetRobot] = []
        for robot in robots:
            physical_regions = set(
                self._controlled_regions_for_robot(robot)
            )
            physical_regions.update(
                physically_owned_regions.get(robot.name, set())
            )
            if physical_regions:
                physical_owners.append(robot)
        if len(physical_owners) == 1:
            return physical_owners[0]
        if self._controlled_corridor_scheduler is None:
            return None

        active_calendar_owners: list[
            tuple[float, str, FleetRobot]
        ] = []
        schedule = self._controlled_corridor_schedule
        for robot in robots:
            slot = (
                schedule.slot_for(robot.name)
                if schedule is not None
                else None
            )
            if (
                slot is not None
                and self._controlled_corridor_has_grant(
                    robot.name,
                    slot.regions,
                )
            ):
                first_resource_entry = min(
                    (
                        slot.entry_time
                        + window.entry_offset_sec
                        for window in slot.resource_windows
                    ),
                    default=slot.entry_time,
                )
                active_calendar_owners.append(
                    (
                        first_resource_entry,
                        slot.robot_id,
                        robot,
                    )
                )
        if active_calendar_owners:
            # A same-flow convoy can hold several committed slots. The first
            # resource entrant remains the physical queue leader.
            return min(active_calendar_owners)[2]
        return None

    def _controlled_corridor_downstream_clearer(
        self,
        robots: list[FleetRobot],
    ) -> FleetRobot | None:
        """Return the robot which can open a physical corridor exit.

        A corridor owner normally keeps right of way. The one exception is an
        external body already occupying its exit pocket: commanding the owner
        forward only tightens the blockage. If that body has a committed
        trajectory which moves away from the owner, it receives the short
        local lease first. This is still one central decision; it does not
        alter corridor admission or permit a new entrant.
        """
        by_name = {robot.name: robot for robot in robots}
        candidates: list[tuple[float, str, FleetRobot]] = []
        schedule = self._controlled_corridor_schedule
        for owner_name, blocker_name in (
            self._controlled_corridor_blockers.items()
        ):
            owner = by_name.get(owner_name)
            blocker = by_name.get(blocker_name)
            if (
                owner is None
                or blocker is None
                or owner.pose is None
                or blocker.pose is None
                or not blocker.trajectory
            ):
                continue
            physical_regions = set(
                self._controlled_regions_for_robot(owner)
            )
            physical_regions.update(
                region_id
                for region_id, owner_names
                in self._controlled_corridor_occupancy.items()
                if owner_name in owner_names
            )
            if not physical_regions:
                continue
            moves_away = False
            for sample in blocker.trajectory:
                sample_clock = float(sample.get("t", 0.0) or 0.0)
                if sample_clock <= blocker.route_clock + 0.000001:
                    continue
                candidate_pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(
                        sample.get(
                            "yaw",
                            blocker.pose.get("yaw", 0.0),
                        )
                        or 0.0
                    ),
                }
                if self._candidate_moves_away(
                    blocker.pose,
                    candidate_pose,
                    owner.pose,
                ):
                    moves_away = True
                    break
            if not moves_away:
                continue
            slot = (
                schedule.slot_for(owner_name)
                if schedule is not None
                else None
            )
            candidates.append(
                (
                    float(slot.entry_time if slot is not None else 0.0),
                    blocker.name,
                    blocker,
                )
            )
        return min(candidates)[2] if candidates else None

    def _controlled_corridor_follower_yields_to(
        self,
        follower: FleetRobot,
        leader: FleetRobot,
        passage: dict[str, Any],
    ) -> bool:
        """Return whether an external passage owner is behind its dependency."""
        if follower.name == leader.name or bool(passage.get("entered")):
            return False
        regions = {
            str(region_id)
            for region_id in passage.get("regions", ())
            if str(region_id)
        }
        if (
            not regions
            or self._controlled_regions_for_robot(follower).intersection(
                regions
            )
        ):
            return False
        dependency_name = (
            str(follower.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(
                follower.last_reason
            )
        )
        direct_reason = str(follower.last_reason or "").strip().lower()
        if (
            dependency_name != leader.name
            or not direct_reason.startswith(
                f"occupied by {leader.name.lower()}"
            )
        ):
            return False
        follower_entry = self._next_controlled_corridor_entry(follower)
        leader_entry = self._next_controlled_corridor_entry(leader)
        if not isinstance(follower_entry, dict) or not isinstance(
            leader_entry,
            dict,
        ):
            return False
        follower_regions = set(
            self._controlled_corridor_entry_regions(follower_entry)
        )
        leader_regions = set(
            self._controlled_corridor_entry_regions(leader_entry)
        )
        entry_lm = str(follower_entry.get("src") or "")
        entry = self.landmarks.get(entry_lm)
        if (
            entry is None
            or follower.pose is None
            or leader.pose is None
        ):
            return False
        follower_distance = math.hypot(
            float(follower.pose.get("x", 0.0)) - float(entry.x),
            float(follower.pose.get("y", 0.0)) - float(entry.y),
        )
        leader_distance = math.hypot(
            float(leader.pose.get("x", 0.0)) - float(entry.x),
            float(leader.pose.get("y", 0.0)) - float(entry.y),
        )
        return bool(
            regions.intersection(follower_regions, leader_regions)
            and str(follower_entry.get("src") or "")
            == str(leader_entry.get("src") or "")
            and str(follower_entry.get("dst") or "")
            == str(leader_entry.get("dst") or "")
            and leader_distance + 0.001 < follower_distance
        )

    def _cycle_forward_clearance(
        self,
        robot: FleetRobot,
        cycle_robots: list[FleetRobot],
    ) -> float:
        """Measure how far this candidate can move past stationary cycle peers."""
        if not robot.trajectory:
            return 0.0
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        # Arbitration must see beyond the ordinary braking preview. In a
        # perpendicular crossing both candidates can look clear for the next
        # ~2 s, while only one can finish its committed rolling chunk without
        # entering the stationary peer's footprint. Chunks are already bounded
        # by the rolling horizon, so inspecting the remainder is cheap.
        horizon = final_time
        step = max(self._runtime_motion_step(), self.collision.sample_time_step())
        clock = min(horizon, robot.route_clock + step)
        while clock <= horizon + 0.000001:
            candidate = self._pose_at_trajectory(robot.trajectory, clock)
            if candidate is None:
                break
            if any(
                other.name != robot.name
                and other.pose is not None
                and self.collision.footprints_overlap(candidate, other.pose)
                for other in cycle_robots
            ):
                return max(0.0, clock - robot.route_clock - step)
            clock += step
        return max(0.0, horizon - robot.route_clock)

    def _wait_cycle_recovery_signature(
        self,
        action: str,
        selected: FleetRobot,
        robots: list[FleetRobot],
    ) -> tuple[str, str, tuple[tuple[str, str, str, str], ...]]:
        """Describe the graph-stable state for one spatial recovery action."""
        members = tuple(
            sorted(
                (
                    robot.name,
                    self._traffic_lm_for_robot(robot),
                    str(robot.active_order_id or ""),
                    str(robot.route_final_lm or robot.target_lm or ""),
                )
                for robot in robots
            )
        )
        return str(action), selected.name, members

    def _wait_cycle_recovery_cooldown(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            stationary_retry = float(
                fleet.get("stationary_recovery_retry_sec", 4.0) or 4.0
            )
        except (TypeError, ValueError):
            stationary_retry = 4.0
        default = max(
            self._deadlock_priority_lease(),
            stationary_retry,
        )
        try:
            configured = float(
                fleet.get("deadlock_recovery_cooldown_sec", default) or default
            )
        except (TypeError, ValueError):
            configured = default
        return max(self._deadlock_priority_lease(), min(30.0, configured))

    def _wait_cycle_recovery_ready(
        self,
        signature: tuple[
            str,
            str,
            tuple[tuple[str, str, str, str], ...],
        ],
        now: float,
    ) -> bool:
        """Gate identical recovery attempts across transient wait states."""
        cooldown = self._wait_cycle_recovery_cooldown()
        # Order ids are intentionally part of the signature so a new task gets
        # an independent recovery episode. Bound old signatures nevertheless:
        # lifelong package benchmarks must not accumulate one key per order.
        retention = max(60.0, cooldown * 8.0)
        for old_signature, attempted_at in list(
            self._wait_cycle_recovery_attempts.items()
        ):
            if now - attempted_at >= retention:
                self._wait_cycle_recovery_attempts.pop(old_signature, None)
        attempted_at = self._wait_cycle_recovery_attempts.get(signature)
        return attempted_at is None or now - attempted_at >= cooldown

    def _record_wait_cycle_recovery_attempt(
        self,
        signature: tuple[
            str,
            str,
            tuple[tuple[str, str, str, str], ...],
        ],
        now: float,
    ) -> None:
        self._wait_cycle_recovery_attempts[signature] = now

    def _clear_wait_cycle_recovery_attempts(
        self,
        robot_names: tuple[str, ...] | list[str] | set[str],
    ) -> None:
        names = set(robot_names)
        if not names:
            return
        for signature in list(self._wait_cycle_recovery_attempts):
            if any(member[0] in names for member in signature[2]):
                self._wait_cycle_recovery_attempts.pop(signature, None)

    def _deadlock_portal_queue_limits(self) -> tuple[int, float]:
        """Return bounded breadth/time for physical portal-queue recovery."""
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured_max = int(
                fleet.get(
                    "deadlock_portal_queue_max_robots",
                    self.planner.local_cbs_max_robots,
                )
                or self.planner.local_cbs_max_robots
            )
        except (TypeError, ValueError):
            configured_max = self.planner.local_cbs_max_robots
        try:
            lookahead = float(
                fleet.get("deadlock_portal_queue_lookahead_sec", 4.0)
                or 4.0
            )
        except (TypeError, ValueError):
            lookahead = 4.0
        return (
            max(
                2,
                min(
                    12,
                    int(self.planner.local_cbs_max_robots),
                    configured_max,
                ),
            ),
            max(1.0, min(10.0, lookahead)),
        )

    def _controlled_corridor_portal_queue_component(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
    ) -> tuple[list[FleetRobot], dict[str, int]]:
        """Discover only the physical tail feeding a blocked corridor mouth.

        The wait-for cycle contains the entered owner and the body directly at
        its exit.  Admission losers behind that body commonly all point to the
        *owner*, so dependency strings alone either miss the tail or pull in
        every remote queue for a bundled corridor.  Instead follow current
        bodies along each waiter's short committed forward trajectory.  Each
        newly discovered waiter must physically run into an already selected
        external member before it can reach the portal.
        """
        component: list[FleetRobot] = []
        by_name: dict[str, FleetRobot] = {}
        for robot in robots:
            if robot.name in by_name:
                continue
            by_name[robot.name] = robot
            component.append(robot)
        depths = {
            robot.name: (-1 if robot.name == winner.name else 0)
            for robot in component
        }
        external_names = {
            robot.name for robot in component if robot.name != winner.name
        }
        if not external_names:
            return component, depths

        max_robots, lookahead = self._deadlock_portal_queue_limits()
        while len(component) < max_robots:
            additions: list[tuple[int, str, FleetRobot]] = []
            for candidate in self._runtime_robots():
                if (
                    candidate.name in by_name
                    or candidate.status != "WAITING"
                    or not candidate.trajectory
                    or not candidate.active_order_id
                    or not self._is_robot_conflict(candidate.last_reason)
                ):
                    continue
                final_clock = float(
                    candidate.trajectory[-1].get("t", candidate.route_clock)
                    or candidate.route_clock
                )
                check_until = min(
                    final_clock,
                    float(candidate.route_clock) + lookahead,
                )
                blocker_name = self._trajectory_current_body_blocker(
                    candidate,
                    candidate.trajectory,
                    float(candidate.route_clock),
                    check_until,
                )
                if blocker_name not in external_names:
                    continue
                additions.append((
                    depths.get(blocker_name, 0) + 1,
                    candidate.name,
                    candidate,
                ))
            if not additions:
                break
            added = False
            for depth, _, candidate in sorted(additions):
                if candidate.name in by_name or len(component) >= max_robots:
                    continue
                by_name[candidate.name] = candidate
                component.append(candidate)
                depths[candidate.name] = max(1, int(depth))
                external_names.add(candidate.name)
                added = True
            if not added:
                break
        return component, depths

    def _previous_clearance_trajectory_lm(
        self,
        robot: FleetRobot,
        queue_depth: int,
    ) -> tuple[float, str] | None:
        """Return an old safe LM far enough to make room for a queue slot."""
        if not robot.trajectory:
            return None
        current_pose = robot.pose or self._pose_at_trajectory(
            robot.trajectory,
            robot.route_clock,
        )
        if current_pose is None:
            return None
        graph = self._controlled_corridor_graph
        required = self.collision.robot_broadphase_distance() * max(
            1.0,
            float(queue_depth) + 0.25,
        )
        fallback: tuple[float, str] | None = None
        seen: set[str] = set()
        for sample in reversed(robot.trajectory):
            sample_time = float(sample.get("t", 0.0) or 0.0)
            if sample_time >= robot.route_clock - 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name in seen or lm_name not in self.landmarks:
                continue
            seen.add(lm_name)
            if graph is not None:
                vertex = graph.vertices.get(lm_name)
                if (
                    vertex is None
                    or not vertex.can_wait
                    or vertex.controlled_region_ids
                ):
                    continue
            landmark = self.landmarks[lm_name]
            distance = math.hypot(
                float(landmark.x) - float(current_pose.get("x", 0.0) or 0.0),
                float(landmark.y) - float(current_pose.get("y", 0.0) or 0.0),
            )
            fallback = (sample_time, lm_name)
            if distance + 0.000001 >= required:
                return fallback
        # Short rolling history is common at a fresh chunk boundary.  Moving
        # to the farthest retained safe LM is still useful, even when it
        # cannot provide the ideal multi-slot distance in one action.
        return fallback

    def _graph_escape_route_current_body_blocker(
        self,
        robot: FleetRobot,
        route_nodes: list[str],
        *,
        only_robot_names: set[str] | None = None,
    ) -> str:
        """Return a present body crossed by a proposed graph escape route.

        A graph edge is not merely a centre-line segment.  Before entering it
        the rectangular body may have to turn in place, and authored
        ``backward``/``not_specified`` motion changes the body yaw without
        changing the direction in which its centre travels.  Auditing only
        landmark interpolation therefore accepted an escape which SIPP could
        commit but runtime preflight stopped during its first turn.

        Build the same oriented edge samples as the route planner, select the
        minimum-turn orientation for unspecified edges, and audit every
        initial/intermediate rotation as well as translation.  Translation
        retains the established move-away exception: a robot already inside a
        soft clearance envelope must still be able to leave it.  A turn keeps
        the centre fixed, so physical footprint overlap remains authoritative.
        """
        nodes = [node for node in route_nodes if node in self.landmarks]
        if len(nodes) < 2:
            return ""
        current_pose = robot.pose or self._pose_at_landmark(nodes[0])
        if current_pose is None:
            return ""
        broadphase = max(0.1, self.collision.robot_broadphase_distance())
        sample_step = max(0.04, min(0.10, broadphase / 8.0))

        def normalize_yaw(value: float) -> float:
            return (float(value) + math.pi) % (2.0 * math.pi) - math.pi

        def turn_distance(first: float, second: float) -> float:
            return abs(normalize_yaw(float(second) - float(first)))

        rotate_enabled = bool(self.planner._rotate_enabled({}))

        # Each unspecified edge permits the body to face either along the
        # tangent or opposite it while its centre follows the same geometry.
        # A tiny dynamic programme (at most two states per edge) mirrors the
        # kinematic planner's minimum accumulated turn choice, including turns
        # at intermediate LMs.  This is more accurate than greedily selecting
        # an orientation independently for every segment.
        edge_variants: list[list[list[dict[str, float]]]] = []
        for src, dst in zip(nodes, nodes[1:]):
            edge = self.planner.route_planner.get_edge(src, dst)
            if edge is None:
                return "invalid graph escape"
            raw_samples = self.planner.route_planner.sample_route(
                PlannedRoute(
                    nodes=[src, dst],
                    edges=[edge],
                    length=float(edge.length),
                ),
                sample_distance=sample_step,
            )
            if len(raw_samples) < 2:
                return "invalid graph escape"
            base = [
                {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": normalize_yaw(
                        float(sample.get("yaw", 0.0) or 0.0)
                    ),
                }
                for sample in raw_samples
            ]
            variants = [base]
            if edge.motion_direction_code() == -1:
                variants.append([
                    {
                        **sample,
                        "yaw": normalize_yaw(sample["yaw"] + math.pi),
                    }
                    for sample in base
                ])
            edge_variants.append(variants)

        # State: accumulated rotation, deterministic variant indices, final
        # yaw, and the selected samples.  Movement cost is identical for both
        # orientations, so only turn cost distinguishes the alternatives.
        orientation_states: list[
            tuple[
                float,
                tuple[int, ...],
                float,
                list[list[dict[str, float]]],
            ]
        ] = [(
            0.0,
            (),
            float(current_pose.get("yaw", 0.0) or 0.0),
            [],
        )]
        for variants in edge_variants:
            next_states: list[
                tuple[
                    float,
                    tuple[int, ...],
                    float,
                    list[list[dict[str, float]]],
                ]
            ] = []
            for variant_index, samples in enumerate(variants):
                candidates = [
                    (
                        cost
                        + (
                            turn_distance(previous_yaw, samples[0]["yaw"])
                            if rotate_enabled
                            else 0.0
                        ),
                        (*indices, variant_index),
                        samples[-1]["yaw"],
                        [*selected, samples],
                    )
                    for cost, indices, previous_yaw, selected
                    in orientation_states
                ]
                next_states.append(min(candidates, key=lambda item: (item[0], item[1])))
            orientation_states = next_states
        _, _, _, selected_edges = min(
            orientation_states,
            key=lambda item: (item[0], item[1]),
        )

        motion_samples: list[tuple[dict[str, float], bool]] = []
        anchor = {
            "x": float(current_pose.get("x", 0.0) or 0.0),
            "y": float(current_pose.get("y", 0.0) or 0.0),
            "yaw": float(current_pose.get("yaw", 0.0) or 0.0),
        }
        rotation_step = math.radians(2.0)
        for edge_samples in selected_edges:
            target_yaw = edge_samples[0]["yaw"]
            yaw_delta = normalize_yaw(target_yaw - anchor["yaw"])
            if rotate_enabled and abs(yaw_delta) >= math.radians(2.0):
                steps = max(1, int(math.ceil(abs(yaw_delta) / rotation_step)))
                for index in range(1, steps + 1):
                    motion_samples.append((
                        {
                            "x": anchor["x"],
                            "y": anchor["y"],
                            "yaw": normalize_yaw(
                                anchor["yaw"] + (yaw_delta * index / steps)
                            ),
                        },
                        True,
                    ))
            # The first route sample is the LM anchor.  Runtime starts at the
            # robot's measured pose (within the graph tolerance), then consumes
            # the remaining planner samples without teleporting the centre.
            for sample in edge_samples[1:]:
                motion_samples.append((sample, False))
            anchor = dict(edge_samples[-1])

        checks = 0
        for candidate_pose, is_rotation in motion_samples:
            checks += 1
            if checks > 512:
                return "bounded escape audit"
            for other in self._runtime_robots():
                if other.name == robot.name or other.pose is None:
                    continue
                if (
                    only_robot_names is not None
                    and other.name not in only_robot_names
                ):
                    continue
                if is_rotation:
                    # Runtime permits a stationary turn inside a neighbour's
                    # soft clearance envelope, but never a physical body
                    # overlap.  Match that exact distinction here.
                    if self.collision.footprints_overlap(
                        candidate_pose,
                        other.pose,
                    ):
                        return other.name
                    continue
                if not self.collision.robot_footprints_conflict(
                    candidate_pose,
                    other.pose,
                ):
                    continue
                if self._candidate_moves_away(
                    current_pose,
                    candidate_pose,
                    other.pose,
                ):
                    continue
                return other.name
        return ""

    def _corridor_clearance_hold_for(
        self,
        winner: FleetRobot,
        winner_regions: set[str],
    ) -> dict[str, Any] | None:
        """Capture the local resource an evacuated portal tail must await."""
        physical_regions = set(self._controlled_regions_for_robot(winner))
        for region_id, owners in self._controlled_corridor_occupancy.items():
            if winner.name in owners:
                physical_regions.add(str(region_id))
        regions = physical_regions or set(winner_regions)
        if not regions:
            return None
        return {
            "owner": winner.name,
            "owner_order_id": str(winner.active_order_id or ""),
            "regions": tuple(sorted(regions)),
            "physical_only": bool(physical_regions),
        }

    def _corridor_clearance_hold_active(
        self,
        hold: dict[str, Any],
        cleared_robot_name: str = "",
    ) -> bool:
        """Return whether a captured passage owner still occupies its mouth."""
        owner_name = str(hold.get("owner") or "")
        owner = self.robots.get(owner_name)
        if owner is None:
            return False
        owner_order_id = str(hold.get("owner_order_id") or "")
        if owner_order_id and owner.active_order_id != owner_order_id:
            return False
        owner_dependency = (
            str(owner.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(owner.last_reason)
        )
        if (
            cleared_robot_name
            and owner_dependency == cleared_robot_name
        ):
            # The selected historical LM was graph-safe, but the owner's
            # actual exit suffix may continue along the same external aisle.
            # In that geometry the held robot is still the direct blocker;
            # keeping it frozen until the owner exits is circular. Release its
            # transactional replan so SIPP can move it to another branch.
            return False
        regions = {
            str(region_id)
            for region_id in hold.get("regions", ())
            if str(region_id)
        }
        if not regions:
            return False
        physical = set(self._controlled_regions_for_robot(owner))
        for region_id, owners in self._controlled_corridor_occupancy.items():
            if owner_name in owners:
                physical.add(str(region_id))
        if physical.intersection(regions):
            return True
        if bool(hold.get("physical_only")):
            return False
        passage = self._controlled_corridor_passages.get(owner_name)
        if not isinstance(passage, dict):
            return False
        passage_regions = {
            str(region_id)
            for region_id in passage.get("regions", ())
            if str(region_id)
        }
        return bool(
            passage_regions.intersection(regions)
            and (
                bool(passage.get("entered"))
                or bool(passage.get("committed"))
            )
        )

    def _controlled_corridor_recovery_physical_regions(
        self,
        robot: FleetRobot,
    ) -> set[str]:
        """Return authored regions which this body physically owns."""
        regions = set(self._controlled_regions_for_robot(robot))
        regions.update(
            str(region_id)
            for region_id, owners
            in self._controlled_corridor_occupancy.items()
            if robot.name in owners
        )
        passage = self._controlled_corridor_passages.get(robot.name)
        if (
            isinstance(passage, dict)
            and (
                bool(passage.get("entered"))
                or bool(passage.get("past_commit_point"))
            )
        ):
            regions.update(
                str(region_id)
                for region_id in passage.get("regions", ())
                if str(region_id)
            )
        return regions

    def _prune_controlled_corridor_recovery_latches(self) -> None:
        for key in list(self._controlled_corridor_recovery_latches):
            region_ids, owner_name, order_id, route_revision = key
            owner = self.robots.get(owner_name)
            if (
                owner is None
                or str(owner.active_order_id or "") != order_id
                or int(owner.route_revision) != route_revision
                or not set(region_ids).intersection(
                    self._controlled_corridor_recovery_physical_regions(
                        owner
                    )
                )
            ):
                self._controlled_corridor_recovery_latches.pop(key, None)

    @staticmethod
    def _controlled_corridor_recovery_latch_key(
        owner: FleetRobot,
        region_ids: set[str],
    ) -> tuple[tuple[str, ...], str, str, int]:
        return (
            tuple(sorted(region_ids)),
            owner.name,
            str(owner.active_order_id or ""),
            int(owner.route_revision),
        )

    def _latch_controlled_corridor_recovery(
        self,
        key: tuple[tuple[str, ...], str, str, int] | None,
        victim_name: str,
    ) -> None:
        if key is not None and victim_name:
            self._controlled_corridor_recovery_latches[key] = victim_name

    def _start_deadlock_corridor_evacuation(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> str:
        candidates: list[
            tuple[
                float,
                int,
                str,
                FleetRobot,
                float,
                str,
                bool,
                list[str],
                list[tuple[str, str]],
            ]
        ] = []
        self._prune_controlled_corridor_recovery_latches()
        winner_regions = (
            self._controlled_corridor_recovery_physical_regions(winner)
        )
        # A pre-entry committed/tentative slot is calendar authority, not
        # proof that the body occupies the narrow resource. Only physical
        # regions create the stable one-recovery-per-owner latch.
        recovery_latch_key = (
            self._controlled_corridor_recovery_latch_key(
                winner,
                winner_regions,
            )
            if winner_regions
            else None
        )
        existing_recovery = (
            self._controlled_corridor_recovery_latches.get(
                recovery_latch_key
            )
            if recovery_latch_key is not None
            else ""
        )
        if existing_recovery:
            existing_robot = self.robots.get(existing_recovery)
            return (
                existing_recovery
                if (
                    existing_robot is not None
                    and existing_robot.status == "RETREATING"
                )
                else ""
            )
        portal_queue_depths: dict[str, int] = {
            robot.name: (-1 if robot.name == winner.name else 0)
            for robot in robots
        }
        # A physical queue can make the direct loser impossible to retreat
        # even on an ordinary graph aisle.  Previously the bounded tail
        # discovery only ran when ``winner`` owned an annotated corridor, so
        # an open-lane head-on A<->B with C->B, D->C behind it kept granting
        # priority to A/B forever: B's reverse sweep hit C, while C and D were
        # outside the recovery component.  Discover the same short physical
        # dependency tail for every wait cycle.  Corridor metadata still adds
        # the portal-specific staging/ownership rules below; in open space the
        # depth is used only to select a graph-safe tail retreat/escape.
        robots, portal_queue_depths = (
            self._controlled_corridor_portal_queue_component(
                robots,
                winner,
            )
        )
        for robot in robots:
            if robot.name == winner.name or not robot.trajectory or not robot.active_order_id:
                continue
            retreat = self._previous_trajectory_lm(robot)
            if retreat is None:
                continue
            portal_queue_depth = max(
                0,
                int(portal_queue_depths.get(robot.name, 0)),
            )
            if portal_queue_depth > 0:
                clearance_retreat = self._previous_clearance_trajectory_lm(
                    robot,
                    portal_queue_depth,
                )
                if clearance_retreat is not None:
                    retreat = clearance_retreat
            corridor_graph = self._controlled_corridor_graph
            robot_regions = self._controlled_regions_for_robot(robot)
            current_vertex = (
                corridor_graph.vertices.get(self._traffic_lm_for_robot(robot))
                if corridor_graph is not None
                else None
            )
            upcoming = (
                self._next_controlled_corridor_entry(robot)
                if callable(getattr(corridor_graph, "lane_for", None))
                else None
            )
            upcoming_regions = set(
                self._controlled_corridor_entry_regions(upcoming)
            )
            graph_escape_required = False
            retreats_from_occupied_portal = bool(
                winner_regions
                and upcoming_regions.intersection(winner_regions)
                and not robot_regions
            )
            if winner_regions and robot_regions:
                # Atomic passage ownership makes the body/bodies already in
                # the resource authoritative. Moving one of them backwards
                # cannot create room at the external portal and risks
                # clearing the only executable exit timeline.
                continue
            if retreats_from_occupied_portal:
                # The current passage owner cannot leave while an opposing
                # admission loser is parked on its external endpoint.  The
                # ordinary "previous LM" at that point is the portal itself,
                # which produces either a no-op detour or another collision.
                # Reuse the admission controller's broadphase-safe staging LM
                # from the retained trajectory and reverse the outside robot
                # all the way there.  If that sample no longer exists, leave
                # both routes intact; never evacuate/clear the internal owner.
                staging_lm = str(
                    upcoming.get("holding_lm")
                    if isinstance(upcoming, dict)
                    else ""
                )
                staging_clock = float(
                    (
                        upcoming.get("staging_clock", robot.route_clock)
                        if isinstance(upcoming, dict)
                        else robot.route_clock
                    )
                    or 0.0
                )
                if (
                    staging_lm not in self.landmarks
                    or staging_clock >= robot.route_clock - 0.000001
                ):
                    # A rolling chunk may begin exactly on the portal. There is
                    # then no historical staging sample to reverse toward; the
                    # generic graph escape below finds a real external pocket.
                    graph_escape_required = True
                else:
                    retreat = (staging_clock, staging_lm)
            if (
                current_vertex is not None
                and current_vertex.controlled_region_ids
            ):
                # A retreat that ends at another internal/no-wait LM merely
                # re-arms the original route and produces an endless
                # forward/backward oscillation. Evacuate only when the
                # retained trajectory contains a genuinely safe external
                # holding LM; otherwise keep the passage owner moving and let
                # atomic admission prevent new entrants.
                safe_retreat = self._previous_safe_trajectory_lm(robot)
                if safe_retreat is None:
                    continue
                retreat = safe_retreat
            target_clock, target_lm = retreat
            retreat_is_noop_at_current_lm = (
                target_lm == robot.current_lm
                and robot.pose is not None
                and self._pose_is_at_lm(robot.pose, target_lm)
            )
            robot_dependency = (
                str(robot.wait_for_robot or "").strip()
                or self._robot_name_from_conflict_reason(robot.last_reason)
            )
            winner_dependency = (
                str(winner.wait_for_robot or "").strip()
                or self._robot_name_from_conflict_reason(winner.last_reason)
            )
            reciprocal_blocker = bool(
                robot_dependency == winner.name
                and winner_dependency == robot.name
            )
            reciprocal_external_blocker = bool(
                winner_regions
                and not robot_regions
                and reciprocal_blocker
            )
            if retreat_is_noop_at_current_lm and reciprocal_blocker:
                # A fresh rolling chunk commonly starts with one or more wait
                # samples on the robot's present LM.  Its "previous" tagged LM
                # is therefore the same physical point.  This used to trigger
                # a same-LM global replan forever for an ordinary, unannotated
                # A<->B head-on: graph escape was enabled only when ``winner``
                # happened to own a controlled corridor.
                #
                # A reciprocal dependency proves that a priority nudge cannot
                # make either body pass through the other.  Move the loser to
                # a real graph-safe holding pocket first, regardless of map
                # annotations.  The selector and installer below still audit
                # directed edge rules, turns, static geometry and every
                # current robot footprint before committing the manoeuvre.
                graph_escape_required = True
            if retreat_is_noop_at_current_lm and portal_queue_depth > 0:
                # A fresh rolling chunk may contain no history before this
                # queue member's current LM.  It still needs a real external
                # pocket rather than the same-goal no-op transaction.
                graph_escape_required = True
            historical_retreat_blocker = ""
            if not retreat_is_noop_at_current_lm:
                historical_retreat_blocker = (
                    self._deadlock_retreat_path_blocker(
                        robot,
                        target_clock,
                    )
                )
                if (
                    historical_retreat_blocker
                    and retreats_from_occupied_portal
                ):
                    # A motion-orientation manoeuvre can carry an admission
                    # loser through the portal and leave its earlier staging
                    # LM on the owner's side.  Reversing that history would
                    # cross the very owner it must release.  Treat this like a
                    # fresh portal boundary and find a free external branch
                    # from the robot's current physical LM instead.
                    graph_escape_required = True
            graph_escape_route: list[str] = []
            portal_blocked_edges: list[tuple[str, str]] = []
            if graph_escape_required:
                # ``current_lm`` deliberately remains the source LM while a
                # robot traverses an edge.  At a stop line the physical pose
                # can already be within centimetres of the following staging
                # LM, however.  A fresh MAPF escape must use that safe nearest
                # LM or start-pose validation rejects the transaction and the
                # two corridor participants wait forever.
                escape_start_lm = self._safe_replan_start_lm(robot)
                portal_src = str(
                    (
                        upcoming.get("src")
                        if isinstance(upcoming, dict)
                        else ""
                    )
                    or ""
                )
                portal_dst = str(
                    (
                        upcoming.get("dst")
                        if isinstance(upcoming, dict)
                        else ""
                    )
                    or ""
                )
                if portal_src in self.landmarks and portal_dst in self.landmarks:
                    portal_blocked_edges = [
                        (portal_src, portal_dst),
                        (portal_dst, portal_src),
                    ]
                elif reciprocal_blocker:
                    # With no corridor entry ahead, forbid the exact suffix
                    # that currently points this robot into its reciprocal
                    # blocker. This applies to ordinary graph aisles as well
                    # as the external side of an annotated portal and leaves
                    # every other branch available to the pocket selector.
                    portal_blocked_edges = self._deadlock_detour_edges(robot)
                if escape_start_lm:
                    graph_escape_route = self._stationary_clearance_route(
                        winner,
                        robot,
                        extra_blocked_edges=set(portal_blocked_edges),
                        avoid_controlled_regions=True,
                        start_lm_override=escape_start_lm,
                        # In an ordinary head-on, moving one body by a single LM
                        # is useful only if the other robot can then reach its
                        # real goal without crossing that new holding pocket.
                        # Corridor owners use their separately captured passage
                        # authority; applying this graph-cut proof to them could
                        # reject the authored external stop line itself.
                        require_waiter_release=bool(
                            reciprocal_blocker and not winner_regions
                        ),
                    )
                # A robot stopped midway between graph LMs cannot begin a
                # fresh MAPF transaction without violating its start-pose
                # contract.  _stationary_clearance_route() deliberately falls
                # back to current_lm for ordinary callers, but doing that here
                # produced a plausible pocket which
                # _install_graph_escape_retreat() then rejected forever.
                # Leave graph_escape_route empty and reuse the already
                # committed trajectory backwards to the previous LM first;
                # completion of that bounded retreat queues the same-goal
                # detour from a valid graph anchor.
                if len(graph_escape_route) >= 2:
                    target_clock = float(robot.route_clock)
                    target_lm = str(graph_escape_route[-1])
                    retreat_is_noop_at_current_lm = True
                    if self._graph_escape_route_current_body_blocker(
                        robot,
                        graph_escape_route,
                    ):
                        # This is the production tail-to-portal case: the
                        # spatial pocket is valid, but reaching it would cross
                        # a body already queued behind the portal.  Do not
                        # submit that known-impossible route or fall back to
                        # the identical goal replan; a later candidate in the
                        # discovered component is the one that must move.
                        continue
            if (
                not graph_escape_route
                and not retreat_is_noop_at_current_lm
                and historical_retreat_blocker
            ):
                # The target LM can be clear while an intermediate LM on a
                # multi-edge portal escape is occupied.  Arming that reverse
                # traversal leaves the robot permanently RETREATING halfway
                # through the path.  Validate the complete current->target
                # sweep against the fleet's current bodies before committing
                # the recovery action.
                continue
            order = self._active_order_for_robot(robot)
            priority = int(order.priority if order is not None else 0)
            distance = max(0.0, robot.route_clock - target_clock)
            candidates.append(
                (
                    distance,
                    priority,
                    robot.name,
                    robot,
                    target_clock,
                    target_lm,
                    retreat_is_noop_at_current_lm,
                    graph_escape_route,
                    portal_blocked_edges,
                )
            )
        if not candidates:
            return ""

        (
            _,
            _,
            _,
            robot,
            target_clock,
            target_lm,
            retreat_is_noop_at_current_lm,
            graph_escape_route,
            portal_blocked_edges,
        ) = min(candidates)
        blocked_edges = self._deadlock_detour_edges(robot)
        if portal_blocked_edges:
            blocked_edges = list(
                dict.fromkeys([*blocked_edges, *portal_blocked_edges])
            )
        if graph_escape_route:
            # When the body has almost reached a staging LM, the edge reported
            # at ``route_clock`` is still its upstream approach edge.  The
            # escape correctly uses that edge in reverse to make room for the
            # corridor owner.  Do not hand MAPF a forced route while also
            # marking one of its directed segments as forbidden.
            escape_edges = set(zip(graph_escape_route, graph_escape_route[1:]))
            blocked_edges = [
                edge for edge in blocked_edges if edge not in escape_edges
            ]
        if not blocked_edges:
            return ""
        recovery_action = (
            f"detour:{target_lm}"
            if (
                abs(robot.route_clock - target_clock) <= 0.000001
                or retreat_is_noop_at_current_lm
            )
            else f"retreat:{target_lm}"
        )
        recovery_signature = self._wait_cycle_recovery_signature(
            recovery_action,
            robot,
            robots,
        )
        if not self._wait_cycle_recovery_ready(recovery_signature, now):
            return ""
        if graph_escape_route:
            if self._install_graph_escape_retreat(
                robot,
                graph_escape_route,
                blocked_edges,
                now,
            ):
                causal_blocker = (
                    str(robot.wait_for_robot or "").strip()
                    or self._robot_name_from_conflict_reason(robot.last_reason)
                    or winner.name
                )
                blocker = self.robots.get(causal_blocker)
                robot.retreat_blocker_signatures = (
                    [(
                        causal_blocker,
                        self._traffic_lm_for_robot(blocker),
                        int(blocker.route_revision),
                    )]
                    if blocker is not None and causal_blocker != robot.name
                    else []
                )
                robot.retreat_corridor_hold = (
                    self._corridor_clearance_hold_for(
                        winner,
                        winner_regions,
                    )
                )
                self._record_wait_cycle_recovery_attempt(
                    recovery_signature,
                    now,
                )
                self._latch_controlled_corridor_recovery(
                    recovery_latch_key,
                    robot.name,
                )
                self._event(
                    "warn",
                    f"{robot.name} clearing corridor portal toward {target_lm}",
                )
                return robot.name
            # A valid spatial pocket can still lose a transient planner-lock
            # race or fail fresh temporal scheduling.  Do not leave the exact
            # wait cycle inert: below, queue a transactional global detour of
            # the same active order while preserving the current trajectory
            # and the internal passage owner's authority.
            graph_escape_route = []
        if (
            abs(robot.route_clock - target_clock) <= 0.000001
            or retreat_is_noop_at_current_lm
        ):
            order = self._active_order_for_robot(robot)
            if order is None:
                return ""
            parked_tail = self._queue_deadlock_portal_tail_clearance(
                winner,
                robot,
                robots,
                portal_blocked_edges,
                now,
            )
            if parked_tail:
                # Do not debounce the admission loser itself. Once the hidden
                # maintenance order has opened the external arm, the unchanged
                # pair must be eligible to install its graph escape immediately.
                self._latch_controlled_corridor_recovery(
                    recovery_latch_key,
                    parked_tail,
                )
                return parked_tail
            replan_handled, replan_started = (
                self._queue_background_replan_recovery_action(
                    robot,
                    now,
                    "deadlock at LM; alternate corridor required",
                    # This is the last fallback after both historical retreat
                    # and a graph-safe escape failed. The retained spatial
                    # suffix is therefore evidence of the deadlock, not a route
                    # which may be re-armed after a transient planner failure.
                    supersede_retained_route=True,
                )
            )
            if replan_handled:
                self._record_wait_cycle_recovery_attempt(
                    recovery_signature,
                    now,
                )
                self._latch_controlled_corridor_recovery(
                    recovery_latch_key,
                    robot.name,
                )
                if replan_started:
                    order.traffic_detour_edges = blocked_edges
                    order.traffic_detour_attempts += 1
                    self.traffic_metrics["cycleReplans"] += 1
                    self._event(
                        "warn",
                        f"{robot.name}@{target_lm} queued for alternate route to the same goal",
                    )
                return robot.name
            return ""

        self._record_wait_cycle_recovery_attempt(recovery_signature, now)
        robot.pending_route = None
        robot.retreat_target_clock = target_clock
        robot.retreat_target_lm = target_lm
        robot.retreat_blocked_edges = blocked_edges
        causal_blocker = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(robot.last_reason)
            or winner.name
        )
        blocker = self.robots.get(causal_blocker)
        robot.retreat_blocker_signatures = (
            [(
                causal_blocker,
                self._traffic_lm_for_robot(blocker),
                int(blocker.route_revision),
            )]
            if blocker is not None and causal_blocker != robot.name
            else []
        )
        robot.retreat_corridor_hold = self._corridor_clearance_hold_for(
            winner,
            winner_regions,
        )
        robot.status = "RETREATING"
        robot.last_reason = f"deadlock retreat to {target_lm} before detour"
        robot.blocked_since = None
        robot.traffic_stall_since = None
        self._clear_wait_dependency(robot)
        robot.last_tick_at = now
        robot.updated_at = now
        self._event(
            "warn",
            f"{robot.name} evacuating narrow corridor back to {target_lm}",
        )
        self._latch_controlled_corridor_recovery(
            recovery_latch_key,
            robot.name,
        )
        return robot.name

    def _queue_deadlock_portal_tail_clearance(
        self,
        corridor_owner: FleetRobot,
        admission_loser: FleetRobot,
        component: list[FleetRobot],
        portal_blocked_edges: list[tuple[str, str]],
        now: float,
    ) -> str:
        """Move an inactive body which seals a corridor loser's escape arm.

        The ordinary wait graph only contains commanded robots. A completed
        robot parked two LMs behind an admission loser is therefore absent even
        when it turns the loser's external aisle into a graph cut. Replanning
        the loser cannot help: the controlled portal is occupied on one side
        and the parked body closes the other.

        Prove causality by selecting a normal graph-safe pocket while
        *prospectively* removing one inactive body. The candidate is accepted
        only when its current footprint actually intersects that hypothetical
        escape. Its relocation is then queued through the existing hidden
        traffic-clearance order, so normal MAPF, motion rules and collision
        checks still own the physical move.
        """
        start_lm = self._safe_replan_start_lm(admission_loser)
        if (
            start_lm not in self.landmarks
            or not portal_blocked_edges
            or not admission_loser.active_order_id
        ):
            return ""

        component_names = {robot.name for robot in component}
        candidates: list[tuple[float, str, FleetRobot, bool]] = []
        for candidate in self._runtime_robots():
            if candidate.name in component_names:
                continue
            relocation_state = self._stationary_clearance_relocations.get(
                candidate.name
            )
            relocation_order = (
                self.orders.get(str(relocation_state.get("order_id") or ""))
                if isinstance(relocation_state, dict)
                else None
            )
            relocation_active = bool(
                relocation_order is not None
                and relocation_order.status not in TERMINAL_ORDER_STATUSES
            )
            if not relocation_active and not (
                self._inactive_stationary_clearance_candidate(
                    candidate,
                    exclude_name=admission_loser.name,
                )
            ):
                continue
            candidate_lm = self._traffic_lm_for_robot(candidate)
            if candidate_lm not in self.landmarks:
                continue
            candidates.append((
                self._lm_distance(start_lm, candidate_lm),
                candidate.name,
                candidate,
                relocation_active,
            ))

        # This proof is only a deadlock-path operation, nevertheless keep it
        # bounded for a very large real fleet. Nearest bodies on the escape arm
        # are the only plausible cuts and are checked first.
        for _, _, candidate, relocation_active in sorted(candidates)[:32]:
            hypothetical_escape = self._stationary_clearance_route(
                corridor_owner,
                admission_loser,
                extra_blocked_edges=set(portal_blocked_edges),
                avoid_controlled_regions=True,
                start_lm_override=start_lm,
                prospectively_vacated_robot_names={candidate.name},
            )
            if len(hypothetical_escape) < 2:
                continue
            if (
                self._graph_escape_route_current_body_blocker(
                    admission_loser,
                    hypothetical_escape,
                    only_robot_names={candidate.name},
                )
                != candidate.name
            ):
                continue

            if not relocation_active and not (
                self._queue_stationary_clearance_relocation(
                    admission_loser,
                    candidate,
                    cause=(
                        f"parked body seals the external escape from "
                        f"{start_lm}"
                    ),
                )
            ):
                continue

            admission_loser.status = "WAITING"
            admission_loser.last_reason = (
                f"waiting for {candidate.name} to clear corridor approach"
            )
            admission_loser.blocked_since = (
                admission_loser.blocked_since or now
            )
            admission_loser.traffic_stall_since = (
                admission_loser.traffic_stall_since or now
            )
            admission_loser.wait_for_robot = candidate.name
            admission_loser.wait_resource = (
                f"portal-tail:{start_lm}"
            )
            admission_loser.wait_release_at = 0.0
            admission_loser.updated_at = now
            self._update_active_order_from_robot(admission_loser)
            if not relocation_active:
                self._event(
                    "warn",
                    f"{candidate.name} clearing parked tail behind "
                    f"{admission_loser.name} at {start_lm}",
                )
            return candidate.name
        return ""

    def _install_graph_escape_retreat(
        self,
        robot: FleetRobot,
        escape_route: list[str],
        blocked_edges: list[tuple[str, str]],
        now: float,
    ) -> bool:
        """Install a short reverse-clock escape when history starts at a portal.

        Ordinary retreat reuses an already committed trajectory. A fresh
        lifelong-order chunk can start on the contested portal itself, however,
        so there is no earlier sample to return to. Plan a normal graph/motion-
        rule compliant path from the portal to an external holding pocket, then
        store its time-reversed representation. The existing retreat runtime can
        execute it by decreasing ``route_clock`` and, on arrival, transactionally
        replan the same active order to its original goal.
        """
        route_nodes = [
            str(node)
            for node in escape_route
            if str(node) in self.landmarks
        ]
        safe_start_lm = self._safe_replan_start_lm(robot)
        if len(route_nodes) < 2 or route_nodes[0] != safe_start_lm:
            return False
        order = self._active_order_for_robot(robot)
        if order is None or order.status in {"COMPLETED", "CANCELED", "FAILED"}:
            return False

        request: dict[str, Any] = {
            "name": robot.name,
            "startLm": route_nodes[0],
            "goalLm": route_nodes[-1],
            "routeNodes": route_nodes,
        }
        if robot.pose is not None:
            request["startPose"] = dict(robot.pose)
        payload = self._order_plan_payload(order, request) | {
            "robots": [request],
            "allowCbsFallback": False,
            "blocked_edges": [
                {"from": src, "to": dst}
                for src, dst in blocked_edges
            ],
        }

        # Never stall the physics thread behind an unrelated long CBS job. The
        # graph-stable cycle is retried on the next arbitration interval when
        # the shared planner is busy.
        if not self._planner_lock.acquire(blocking=False):
            return False
        try:
            result = self._plan_valid_requests_unlocked([request], payload)
            if not result.get("ok") or not result.get("plans"):
                # The escape selector already excludes controlled regions,
                # current bodies and static obstacles. Temporal reservations of
                # the cycle itself may still reject the only outward edge; for
                # this explicit right-of-way action, generate its kinematics
                # without those stale future reservations. Runtime swept-
                # footprint checks remain the final motion authority.
                result = self.planner.plan({
                    **payload,
                    "robots": [request],
                    "blocked_lms": [],
                    "reserved_vertex_constraints": [],
                    "reserved_edge_constraints": [],
                    "reserved_vertex_intervals": [],
                    "reserved_edge_intervals": [],
                })
        finally:
            self._planner_lock.release()

        plan = self._plan_for_robot(result, robot.name)
        if plan is None:
            return False
        forward = [
            dict(sample)
            for sample in plan.get("trajectory", [])
            if isinstance(sample, dict)
        ]
        if len(forward) < 2:
            return False
        duration = float(forward[-1].get("t", 0.0) or 0.0)
        if duration <= 0.000001:
            return False
        if self._trajectory_current_body_blocker(
            robot,
            forward,
            0.0,
            duration,
        ):
            # A spatial pocket is insufficient when the path to that pocket
            # crosses a robot that is currently stopped on an intermediate
            # LM.  Runtime collision checks would stop the escape safely, but
            # without this transaction-level audit it could never finish.
            return False
        retreat_trajectory: list[dict[str, Any]] = []
        for sample in reversed(forward):
            transformed = dict(sample)
            transformed["t"] = max(
                0.0,
                duration - float(sample.get("t", 0.0) or 0.0),
            )
            retreat_trajectory.append(transformed)
        retreat_trajectory.sort(
            key=lambda sample: float(sample.get("t", 0.0) or 0.0)
        )

        # Supersede any failed same-goal transaction at this unchanged portal.
        self._runtime_replans.pop(robot.name, None)
        old_passage = self._controlled_corridor_passages.pop(robot.name, None)
        old_regions = {
            str(item)
            for item in (
                old_passage.get("regions", ())
                if isinstance(old_passage, dict)
                else ()
            )
            if str(item)
        }
        for region_id in old_regions:
            lease = self._controlled_corridor_leases.get(region_id)
            if lease and lease[0] == robot.name:
                self._controlled_corridor_leases.pop(region_id, None)
        self._controlled_corridor_winners.pop(robot.name, None)

        order.status = "EXECUTING"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.traffic_detour_edges = list(dict.fromkeys(blocked_edges))
        order.route_nodes = list(route_nodes)
        robot.trajectory = retreat_trajectory
        robot.plan_nodes = list(reversed(route_nodes))
        robot.route_clock = duration
        robot.route_started_at = now
        robot.route_revision = self._next_route_revision()
        robot.pending_route = None
        robot.retreat_target_clock = 0.0
        robot.retreat_target_lm = route_nodes[-1]
        robot.retreat_blocked_edges = list(dict.fromkeys(blocked_edges))
        robot.status = "RETREATING"
        robot.last_reason = f"deadlock portal escape to {route_nodes[-1]}"
        robot.blocked_since = None
        robot.traffic_stall_since = None
        robot.last_tick_at = now
        robot.traffic_priority_until = now + self._deadlock_priority_lease()
        robot.collision_preflight_due_at = 0.0
        robot.trajectory_dirty = True
        robot.route_preview_dirty = True
        self._clear_wait_dependency(robot)
        robot.updated_at = now
        return True

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

    def _deadlock_retreat_path_blocker(
        self,
        robot: FleetRobot,
        target_clock: float,
    ) -> str:
        """Return the first current body intersecting a reverse retreat path.

        A retreat deliberately reuses an old trajectory in reverse, so its old
        temporal reservations are no longer authoritative.  Checking only the
        destination misses a robot parked on an intermediate graph LM (the
        production failure was a clear target four LMs away with a waiter on
        the second LM).  This bounded dense sweep is only run while resolving
        a deadlock, not on ordinary physics ticks.
        """
        return self._trajectory_current_body_blocker(
            robot,
            robot.trajectory,
            float(robot.route_clock),
            float(target_clock),
        )

    def _trajectory_current_body_blocker(
        self,
        robot: FleetRobot,
        trajectory: list[dict[str, Any]],
        start_clock: float,
        target_clock: float,
    ) -> str:
        if not trajectory or abs(target_clock - start_clock) <= 0.000001:
            return ""
        current_pose = robot.pose or self._pose_at_trajectory(
            trajectory,
            start_clock,
        )
        if current_pose is None:
            return "invalid retreat pose"

        span = abs(target_clock - start_clock)
        # At most 512 checks keeps a pathological long rolling trajectory
        # bounded while remaining much denser than one robot footprint at
        # normal fleet speeds.
        step = max(self._runtime_motion_step(), span / 512.0)
        direction = 1.0 if target_clock > start_clock else -1.0
        clocks: list[float] = []
        clock = start_clock + (direction * step)
        while (
            clock < target_clock - 0.000001
            if direction > 0.0
            else clock > target_clock + 0.000001
        ):
            clocks.append(clock)
            clock += direction * step
        clocks.append(target_clock)

        for check_clock in clocks:
            candidate_pose = self._pose_at_trajectory(
                trajectory,
                check_clock,
            )
            if candidate_pose is None:
                continue
            # Waiting beside another body can begin inside the softer traffic
            # envelope.  A path monotonically increasing their separation is
            # an escape, not a collision; all physical-overlap checks still
            # run during execution.
            if self._candidate_stays_put(current_pose, candidate_pose):
                continue
            for other in self._runtime_robots():
                if other.name == robot.name or other.pose is None:
                    continue
                if not self.collision.robot_footprints_conflict(
                    candidate_pose,
                    other.pose,
                ):
                    continue
                if self._candidate_moves_away(
                    current_pose,
                    candidate_pose,
                    other.pose,
                ):
                    continue
                return other.name
        return ""

    def _previous_trajectory_lm(
        self,
        robot: FleetRobot,
    ) -> tuple[float, str] | None:
        candidate: tuple[float, str] | None = None
        previous_distinct: tuple[float, str] | None = None
        for sample in robot.trajectory:
            sample_time = float(sample.get("t", 0.0) or 0.0)
            if sample_time > robot.route_clock + 0.000001:
                break
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name not in self.landmarks:
                continue
            if candidate is not None and candidate[1] != lm_name:
                previous_distinct = candidate
            candidate = (sample_time, lm_name)
        if candidate is None or not robot.trajectory:
            return candidate
        final_sample = robot.trajectory[-1]
        final_time = float(final_sample.get("t", 0.0) or 0.0)
        final_lm = str(final_sample.get("lm") or "").strip()
        if (
            robot.route_clock >= final_time - 0.000001
            and final_lm == candidate[1]
            and previous_distinct is not None
        ):
            # At an exhausted chunk the last tagged LM is the robot's current
            # physical endpoint. Retreating to that same clock is a no-op and
            # only queues another replan. Use the most recent distinct LM
            # instead; repeated wait/rotation samples at either endpoint do
            # not change which graph segment must be reversed.
            return previous_distinct
        return candidate

    def _previous_safe_trajectory_lm(
        self,
        robot: FleetRobot,
    ) -> tuple[float, str] | None:
        graph = self._controlled_corridor_graph
        if graph is None:
            return self._previous_trajectory_lm(robot)
        candidate: tuple[float, str] | None = None
        for sample in robot.trajectory:
            sample_time = float(sample.get("t", 0.0) or 0.0)
            if sample_time > robot.route_clock + 0.000001:
                break
            lm_name = str(sample.get("lm") or "").strip()
            vertex = graph.vertices.get(lm_name)
            if (
                vertex is not None
                and bool(getattr(vertex, "can_wait", True))
                and not getattr(vertex, "controlled_region_ids", ())
            ):
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
        cycle_stall_started = min(
            (robot.traffic_stall_since or robot.blocked_since or now for robot in robots),
            default=now,
        )
        if now - cycle_stall_started >= self._deadlock_coupled_replan_after():
            # The leased winner failed to produce even one route-clock advance.
            # Stop renewing it as soon as the local-CBS deadline is reached;
            # otherwise the short priority lease can be refreshed forever and
            # a visibly stopped group never reaches coupled replanning.
            for robot in leased:
                robot.traffic_priority_until = 0.0
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
            return 1.5
        try:
            return max(0.5, float(fleet.get("deadlock_priority_lease_sec", 1.5) or 1.5))
        except (TypeError, ValueError):
            return 1.5

    def _deadlock_wait_timeout(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(
                0.5,
                float(fleet.get("deadlock_wait_timeout_sec", 1.0) or 1.0),
            )
        except (TypeError, ValueError):
            return 1.0

    def _traffic_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 3.0
        try:
            return max(
                self._deadlock_wait_timeout(),
                float(fleet.get("traffic_replan_after_sec", 3.0) or 3.0),
            )
        except (TypeError, ValueError):
            return 3.0

    def _blocked_replan_after(self, reason: str) -> float:
        if not self._is_parked_robot_conflict(reason):
            return self._traffic_replan_after()
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(
                0.25,
                float(
                    fleet.get("parked_robot_replan_after_sec", 1.0)
                    or 1.0
                ),
            )
        except (TypeError, ValueError):
            return 1.0

    def _traffic_zone_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 6.0
        try:
            # Give the starvation-aware phase selector one normal opportunity
            # before switching corridors, but still impose a hard upper bound.
            return max(
                self._traffic_zone_param("traffic_zone_starvation_sec", 5.0),
                float(fleet.get("traffic_zone_replan_after_sec", 6.0) or 6.0),
            )
        except (TypeError, ValueError):
            return 6.0

    def _controlled_corridor_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 8.0
        try:
            return max(
                self._controlled_corridor_param(
                    "controlled_corridor_starvation_sec",
                    8.0,
                ),
                float(
                    fleet.get(
                        "controlled_corridor_replan_after_sec",
                        8.0,
                    )
                    or 8.0
                ),
            )
        except (TypeError, ValueError):
            return 8.0

    def _deadlock_coupled_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.5
        try:
            return max(
                self._deadlock_wait_timeout(),
                float(fleet.get("deadlock_coupled_replan_after_sec", 1.5) or 1.5),
            )
        except (TypeError, ValueError):
            return 1.5

    def _deadlock_coupled_replan_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.5
        try:
            return max(
                0.5,
                float(fleet.get("deadlock_coupled_replan_interval_sec", 1.5) or 1.5),
            )
        except (TypeError, ValueError):
            return 1.5

    def _deadlock_retreat_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 4.5
        try:
            return max(
                self._deadlock_coupled_replan_after(),
                float(fleet.get("deadlock_retreat_after_sec", 4.5) or 4.5),
            )
        except (TypeError, ValueError):
            return 4.5

    def _replan_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(0.25, float(fleet.get("replan_interval_sec", 1.0) or 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _runtime_collision_preflight_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.20
        try:
            return max(
                0.10,
                min(
                    0.50,
                    float(
                        fleet.get(
                            "runtime_collision_preflight_interval_sec",
                            0.20,
                        )
                        or 0.20
                    ),
                ),
            )
        except (TypeError, ValueError):
            return 0.20

    def _runtime_collision_preflight_due(
        self,
        robot: FleetRobot,
        now: float,
    ) -> bool:
        return (
            robot.collision_preflight_revision != robot.route_revision
            or now + 0.000001 >= robot.collision_preflight_due_at
        )

    def _mark_runtime_collision_preflight(
        self,
        robot: FleetRobot,
        now: float,
    ) -> None:
        interval = self._runtime_collision_preflight_interval()
        # A stable per-robot phase prevents all fifty lookahead scans from
        # becoming due on the same physics tick after a batch plan commit.
        phase = (sum(ord(char) for char in robot.name) % 7) / 7.0
        robot.collision_preflight_revision = robot.route_revision
        robot.collision_preflight_due_at = now + (interval * (0.85 + (phase * 0.30)))

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
        if self._central_corridor_owner_is_clearing(robot):
            # Admission already guarantees exclusive/compatible traffic in
            # the atomic bundle.  Looking several seconds beyond its external
            # exit used to freeze an owner while its rear footprint was still
            # inside: a robot parked in the downstream aisle could therefore
            # hold the whole corridor forever.  Immediate checks below remain
            # authoritative and stop before any actual footprint overlap.
            end_clock = proposed_clock
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
        *,
        ignore_admission: bool = False,
    ) -> str:
        pose = self._pose_at_trajectory(robot.trajectory, check_clock)
        if pose is None:
            return ""
        if not ignore_admission:
            corridor_reason = self._controlled_corridor_admission_reason(
                robot,
                check_clock,
            )
            if corridor_reason:
                return corridor_reason
            zone_reason = self._traffic_zone_admission_reason(robot, check_clock)
            if zone_reason:
                return zone_reason
        # Use elapsed time from the common beginning-of-tick clock. Earlier
        # robots in the loop may already have advanced their mutable clock;
        # basing the offset on that value makes pair predictions asynchronous.
        robot_clock = self._runtime_tick_route_clocks.get(
            robot.name,
            robot.route_clock,
        )
        offset = max(0.0, check_clock - robot_clock)
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
            incremental_dt = max(0.0, check_clock - robot.route_clock)
            segment_start_pose = (
                self._pose_at_trajectory(
                    robot.trajectory,
                    robot.route_clock,
                )
                or robot.pose
            )
            other_segment_start = self._predicted_robot_pose(
                other,
                max(0.0, offset - incremental_dt),
            )
            if (
                incremental_dt
                <= self._runtime_motion_step() + 0.000001
                and segment_start_pose is not None
                and other_segment_start is not None
            ):
                # Endpoint-only checks can miss two rectangular bodies whose
                # swept areas cross between 50 ms motion samples. The global
                # invariant catches that only after movement and visibly
                # rolls both robots back. Resolve the same crossing here.
                #
                # Pair prediction must also model the arbitration decision:
                # if this robot wins, the lower-priority peer may stop later
                # in the same tick. Never let the winner rely on that peer
                # continuing to move out of its way.
                predicted_sweep_conflict = self._swept_footprints_overlap(
                    segment_start_pose,
                    pose,
                    other_segment_start,
                    other_pose,
                )
                has_right_of_way = self._has_right_of_way(robot, other)
                # The peer prediction is conditional: that peer can be
                # stopped by its own collision/admission check later in this
                # same sequential tick.  The immediate substep therefore may
                # never consume the peer's *current* physical body, even when
                # the moving robot has (or lacks) traffic right of way.  This
                # is the pre-commit counterpart of the global swept-footprint
                # invariant and prevents a stop-then-rollback storm when a
                # predicted leader does not actually advance.
                stationary_endpoint_conflict = self.collision.footprints_overlap(
                    pose,
                    other.pose,
                )
                stationary_sweep_conflict = self._swept_footprints_overlap(
                    segment_start_pose,
                    pose,
                    other.pose,
                    other.pose,
                )
                existing_overlap_escape = bool(
                    self.collision.footprints_overlap(
                        segment_start_pose,
                        other.pose,
                    )
                    and self._candidate_moves_away(
                        segment_start_pose,
                        pose,
                        other.pose,
                    )
                )
                if (
                    (
                        stationary_endpoint_conflict
                        or stationary_sweep_conflict
                    )
                    and not existing_overlap_escape
                ) or (
                    predicted_sweep_conflict
                    and not has_right_of_way
                    and not existing_overlap_escape
                ):
                    if self._is_active_traffic(other):
                        return f"yield to {other.name}"
                    return f"occupied by {other.name}"
            rotation_reason = self._rotation_sweep_conflict_reason(
                robot,
                other,
                check_clock,
                pose,
                other_pose,
                offset,
            )
            if rotation_reason:
                return rotation_reason
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

    def _rotation_sweep_conflict_reason(
        self,
        robot: FleetRobot,
        other: FleetRobot,
        check_clock: float,
        candidate_pose: dict[str, float],
        other_pose: dict[str, float],
        prediction_offset: float,
    ) -> str:
        robot_rotating = self._is_rotation_at_trajectory(
            robot.trajectory,
            check_clock,
        )
        other_clock = self._runtime_tick_route_clocks.get(
            other.name,
            other.route_clock,
        ) + max(0.0, prediction_offset)
        other_rotating = bool(
            other.trajectory
            and self._is_rotation_at_trajectory(other.trajectory, other_clock)
        )
        # The coarse circumscribed-radius rule exists only to serialize two
        # simultaneous adjacent turns. A stationary or translating neighbour
        # is checked below by the exact oriented footprint geometry; treating
        # it as occupying the complete circle creates artificial grid walls.
        if not (robot_rotating and other_rotating):
            return ""
        distance = math.hypot(
            float(candidate_pose.get("x", 0.0) or 0.0)
            - float(other_pose.get("x", 0.0) or 0.0),
            float(candidate_pose.get("y", 0.0) or 0.0)
            - float(other_pose.get("y", 0.0) or 0.0),
        )
        threshold = max(
            0.0,
            float(self.planner.rotation_min_robot_center_distance_m),
        )
        if threshold <= 0.0 or distance >= threshold:
            return ""

        # The rotation resource must obey the same deterministic grant as a
        # translational crossing. Otherwise both adjacent robots return
        # ``yield`` forever even after the deadlock resolver selected a winner.
        # Exact oriented-footprint checks below remain authoritative.
        if self._has_right_of_way(robot, other):
            return ""
        if self._is_active_traffic(other):
            return f"yield to {other.name}"
        return f"keep clearance from {other.name}"

    def _is_rotation_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> bool:
        if len(trajectory) < 2:
            return False
        index = self._trajectory_segment_index(trajectory, elapsed)
        start = trajectory[index]
        end = trajectory[index + 1]
        start_time = float(start.get("t", 0.0) or 0.0)
        end_time = float(end.get("t", start_time) or start_time)
        if end_time <= start_time or not (start_time <= elapsed < end_time):
            return False
        edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
        return edge_id.startswith("WAIT@ROTATE:")

    def _predicted_robot_pose(self, robot: FleetRobot, offset: float) -> dict[str, float] | None:
        route_clock = self._runtime_tick_route_clocks.get(
            robot.name,
            robot.route_clock,
        )
        if (
            robot.status == "RETREATING"
            and robot.trajectory
            and robot.retreat_target_clock is not None
        ):
            return self._pose_at_trajectory(
                robot.trajectory,
                max(
                    robot.retreat_target_clock,
                    route_clock - max(0.0, offset),
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
            min(final_time, route_clock + max(0.0, offset)),
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
        existing_overlap_escape = bool(
            robot.pose is not None
            and other.pose is not None
            and self.collision.footprints_overlap(robot.pose, other.pose)
            and self._candidate_moves_away(
                robot.pose,
                candidate_pose,
                other.pose,
            )
        )
        if (
            other_current_is_authoritative
            and other.pose is not None
            and self.collision.footprints_overlap(candidate_pose, other.pose)
            and not existing_overlap_escape
        ):
            return f"occupied by {other.name}"
        if self.collision.footprints_overlap(candidate_pose, other_pose):
            predicted_overlap_escape = bool(
                existing_overlap_escape
                and robot.pose is not None
                and self._candidate_moves_away(
                    robot.pose,
                    candidate_pose,
                    other_pose,
                )
            )
            # This is an anticipated collision, not an overlap that already
            # exists. Resolve it before motion using deterministic right of
            # way: the loser waits, while the winner's future prediction is
            # evaluated against the loser's stationary pose on later ticks.
            if not predicted_overlap_escape:
                if self._has_right_of_way(robot, other):
                    return ""
                if self._is_active_traffic(other):
                    return f"yield to {other.name}"
                return f"occupied by {other.name}"
        if robot.pose is not None and self._candidate_stays_put(robot.pose, candidate_pose):
            return ""
        if (
            robot.pose is not None
            and other.pose is not None
            and self._candidate_moves_away(robot.pose, candidate_pose, other.pose)
        ):
            # The physical-overlap guards above remain authoritative. A robot
            # that already starts inside the softer traffic-clearance envelope
            # must be allowed to leave it, otherwise adjacent graph cells form
            # an artificial wall and every replan fails at t=0.
            return ""
        if (
            robot.status == "RETREATING"
            and robot.traffic_priority_until > self._now()
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
        now = self._now()
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
        return self.task_manager.active_for_robot(
            robot.name,
            preferred_order_id=robot.active_order_id,
        )

    def _is_robot_conflict(self, reason: str) -> bool:
        value = str(reason)
        return (
            value.startswith("yield to ")
            or value.startswith("occupied by ")
            or value.startswith("keep clearance from ")
            or (
                value.startswith("corridor admission wait at ")
                and bool(self._robot_name_from_conflict_reason(value))
            )
        )

    def _is_deadlock_reason(self, reason: str) -> bool:
        value = str(reason or "")
        return value.startswith("deadlock:") or "continuous_conflict_unresolved" in value

    def _should_replan_for_blocked_reason(self, reason: str) -> bool:
        if self._is_deadlock_reason(reason):
            return False
        if str(reason or "").startswith("traffic admission wait at "):
            return False
        if str(reason or "").startswith("corridor admission wait at "):
            return False
        if not self._is_robot_conflict(reason):
            return True
        return self._is_parked_robot_conflict(reason)

    def _wait_expected_to_clear(self, robot: FleetRobot) -> bool:
        blocker_name = robot.wait_for_robot or self._robot_name_from_conflict_reason(
            robot.last_reason,
        )
        blocker = self.robots.get(blocker_name)
        if blocker is None:
            return False
        if self._robot_departure_pending(blocker):
            return True
        if not blocker.trajectory:
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

    def _robot_departure_pending(self, robot: FleetRobot) -> bool:
        order = self._active_order_for_robot(robot)
        if order is None or order.status not in {"QUEUED", "PLANNING"}:
            return False
        target_lm = self._active_order_target(order)
        return bool(target_lm and target_lm != self._traffic_lm_for_robot(robot))

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
        if value.startswith("corridor admission wait at ") and "; owner " in value:
            return value.rsplit("; owner ", 1)[1].strip()
        return ""

    def _set_wait_dependency(self, robot: FleetRobot, reason: str, now: float) -> None:
        blocker = self._robot_name_from_conflict_reason(reason)
        if not blocker or blocker == robot.name:
            # Admission diagnostics can briefly report the robot's own lease
            # while corridor state is rebuilt at a rolling/turn boundary.
            # Self-dependencies are never actionable wait-for edges: retaining
            # one manufactures a one-node deadlock and prevents ordinary
            # progress/re-evaluation from clearing the stale status.
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
