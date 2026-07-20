"""Runtime traffic arbitration and deadlock resolution."""

from __future__ import annotations

import math
from time import time
from typing import Any

from fleet_manager.core.models import FleetOrder, FleetRobot


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
            self._active_wait_cycles.pop(cycle_key, None)
            self._wait_cycle_last_arbitration.pop(cycle_key, None)
            self._coupled_replan_failures.pop(cycle_key, None)

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
            if self._safe_replan_start_lm(robot):
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
            self._coupled_replan_failures.pop(cycle_key, None)
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
        lease_until = now + self._deadlock_priority_lease()
        participants = [winner, blocker]
        # A→B where B is stopped at a corridor signal is not resolved by
        # granting A more priority: A still has B's body directly ahead.
        # Transfer the corridor signal to B (when this pair owns the complete
        # local conflict) and let B clear the stop line first.
        if (
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
        evacuated_name = self._start_deadlock_corridor_evacuation(
            [blocker, waiter],
            blocker,
            now,
        )
        if evacuated_name != waiter.name:
            return False
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
        winner = corridor_owner or min(robots, key=priority_key)
        cycle_key = tuple(sorted(robot.name for robot in robots))
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
            and cycle_wait >= self._deadlock_wait_timeout()
        ):
            # An entrant parked on the mouth of an occupied one-lane region
            # must clear for the current owner. A priority flip or local CBS
            # cannot change that physical ordering.
            evacuating_name = self._start_deadlock_corridor_evacuation(
                robots,
                winner,
                now,
            )
        elif (
            cycle_wait >= self._deadlock_retreat_after()
            and self._coupled_replan_failures.get(cycle_key, 0) > 0
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
        if not priority_robot.trajectory:
            eligible = [robot for robot in robots if robot.trajectory]
            if not eligible:
                return
            priority_robot = min(eligible, key=priority_key)
        priority_robot.traffic_priority_until = max(
            priority_robot.traffic_priority_until,
            lease_until,
        )
        self._transfer_controlled_corridor_lease(
            priority_robot,
            robots,
            now,
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
        self.traffic_metrics["priorityGrants"] += 1
        self._event(
            "warn",
            f"traffic wait cycle resolved: priority granted to {priority_robot.name}",
        )

    def _controlled_corridor_cycle_owner(
        self,
        robots: list[FleetRobot],
    ) -> FleetRobot | None:
        """Prefer the robot physically inside a corridor over its entrant."""
        by_name = {robot.name: robot for robot in robots}
        prefix = "corridor admission wait at "
        marker = " for "
        for entrant in robots:
            reason = str(entrant.last_reason or "")
            if not reason.startswith(prefix) or marker not in reason:
                continue
            region_id = reason.split(marker, 1)[1].split(
                "; owner ",
                1,
            )[0].strip()
            for owner_name in self._controlled_corridor_occupancy.get(
                region_id,
                [],
            ):
                owner = by_name.get(owner_name)
                if owner is not None and owner.name != entrant.name:
                    return owner
        return None

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

    def _start_deadlock_corridor_evacuation(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> str:
        candidates: list[
            tuple[float, int, str, FleetRobot, float, str, bool]
        ] = []
        for robot in robots:
            if robot.name == winner.name or not robot.trajectory or not robot.active_order_id:
                continue
            retreat = self._previous_trajectory_lm(robot)
            if retreat is None:
                continue
            target_clock, target_lm = retreat
            retreat_is_noop_at_current_lm = (
                target_lm == robot.current_lm
                and robot.pose is not None
                and self._pose_is_at_lm(robot.pose, target_lm)
            )
            if (
                not retreat_is_noop_at_current_lm
                and self._deadlock_retreat_target_blocker(robot, target_clock)
            ):
                # Reversing toward an occupied landmark cannot evacuate the
                # corridor and creates a permanent RETREATING state. Try a
                # different member of the cycle; if none is clear, grant the
                # forward winner the lease and retry on a later tick.
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
        ) = min(candidates)
        blocked_edges = self._deadlock_detour_edges(robot)
        if not blocked_edges:
            return ""
        if (
            abs(robot.route_clock - target_clock) <= 0.000001
            or retreat_is_noop_at_current_lm
        ):
            order = self._active_order_for_robot(robot)
            if order is None:
                return ""
            order.traffic_detour_edges = blocked_edges
            order.traffic_detour_attempts += 1
            if self._queue_active_order_for_background_replan(
                robot,
                now,
                "deadlock at LM; alternate corridor required",
                # This is narrower than an ordinary traffic wait inside a
                # controlled corridor: the selected reverse target is the
                # robot's present LM/pose, so retaining the same trajectory
                # can only arm the identical no-op retreat on the next tick.
                allow_controlled_corridor_replan=retreat_is_noop_at_current_lm,
            ):
                self.traffic_metrics["cycleReplans"] += 1
                self._event(
                    "warn",
                    f"{robot.name}@{target_lm} queued for alternate route to the same goal",
                )
                return robot.name
            return ""

        robot.pending_route = None
        robot.retreat_target_clock = target_clock
        robot.retreat_target_lm = target_lm
        robot.retreat_blocked_edges = blocked_edges
        robot.status = "RETREATING"
        robot.last_reason = f"deadlock retreat to {target_lm} before detour"
        robot.last_tick_at = now
        robot.updated_at = now
        self._event(
            "warn",
            f"{robot.name} evacuating narrow corridor back to {target_lm}",
        )
        return robot.name

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
        if (
            other_current_is_authoritative
            and other.pose is not None
            and self.collision.footprints_overlap(candidate_pose, other.pose)
        ):
            return f"occupied by {other.name}"
        if self.collision.footprints_overlap(candidate_pose, other_pose):
            # This is an anticipated collision, not an overlap that already
            # exists. Resolve it before motion using deterministic right of
            # way: the loser waits, while the winner's future prediction is
            # evaluated against the loser's stationary pose on later ticks.
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
        if not blocker:
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
