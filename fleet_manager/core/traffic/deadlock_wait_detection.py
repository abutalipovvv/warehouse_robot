"""Wait-cycle snapshot, detection and top-level resolution stages."""

from __future__ import annotations

from dataclasses import dataclass

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.traffic.wait_graph import WaitForGraph


@dataclass(frozen=True, slots=True)
class _RuntimeWaitSnapshot:
    """Filtered robots and their immutable-for-this-tick dependency graph."""

    waiting: dict[str, FleetRobot]
    graph: WaitForGraph


class WaitCycleDetectionMixin:
    """Capture wait dependencies and dispatch cycle, chain and starvation handling."""

    def _resolve_runtime_wait_cycles(self, now: float) -> None:
        snapshot = self._runtime_wait_snapshot(now)
        cycle_members, observed_cycle_keys = self._arbitrate_wait_cycles(
            snapshot,
            now,
        )
        self._expire_wait_cycle_episodes(observed_cycle_keys, now)
        timeout = self._traffic_replan_after()
        chain_members = self._resolve_wait_chains(
            snapshot,
            cycle_members,
            timeout,
            now,
        )
        self._resolve_starved_waiters(
            snapshot.waiting,
            cycle_members,
            chain_members,
            timeout,
            now,
        )
        self._replan_expired_admission_waits(now)

    def _runtime_wait_snapshot(self, now: float) -> _RuntimeWaitSnapshot:
        """Capture only actionable robot-conflict waits for this runtime tick."""
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
            name: (
                robot.wait_for_robot
                or self._robot_name_from_conflict_reason(robot.last_reason)
            )
            for name, robot in waiting.items()
        }
        return _RuntimeWaitSnapshot(
            waiting=waiting,
            graph=WaitForGraph(wait_for),
        )

    def _arbitrate_wait_cycles(
        self,
        snapshot: _RuntimeWaitSnapshot,
        now: float,
    ) -> tuple[set[str], set[tuple[str, ...]]]:
        """Detect cycles, retain their episode age and run one arbitration."""
        waiting = snapshot.waiting
        wait_graph = snapshot.graph
        cycle_members: set[str] = set()
        observed_cycle_keys: set[tuple[str, ...]] = set()
        for cycle in wait_graph.cycles():
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
            if not self._maintain_runtime_wait_cycle_lease(
                cycle,
                waiting,
                now,
            ):
                self._break_runtime_wait_cycle(
                    cycle,
                    waiting,
                    now,
                    new_episode=new_episode,
                )
            cycle_members.update(cycle)
        return cycle_members, observed_cycle_keys

    def _expire_wait_cycle_episodes(
        self,
        observed_cycle_keys: set[tuple[str, ...]],
        now: float,
    ) -> None:
        """Forget episodes only after their geometry and active lease vanish."""
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

    def _resolve_wait_chains(
        self,
        snapshot: _RuntimeWaitSnapshot,
        cycle_members: set[str],
        timeout: float,
        now: float,
    ) -> set[str]:
        """Handle acyclic tails without overriding an already resolved cycle."""
        waiting = snapshot.waiting
        wait_graph = snapshot.graph
        chain_members: set[str] = set()
        for start_name, start_robot in waiting.items():
            if (
                start_name in cycle_members
                or start_name in chain_members
                or start_robot.blocked_since is None
                or now - start_robot.blocked_since < timeout
            ):
                continue
            wait_chain = wait_graph.walk(start_name)
            chain = list(wait_chain.members)
            current = wait_chain.terminal
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
        return chain_members

    def _resolve_starved_waiters(
        self,
        waiting: dict[str, FleetRobot],
        cycle_members: set[str],
        chain_members: set[str],
        timeout: float,
        now: float,
    ) -> None:
        """Escalate old single dependencies after cycle and chain handling."""
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

    def _replan_expired_admission_waits(self, now: float) -> None:
        """Replan only admission waits whose owner cannot clear normally."""
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


__all__ = ['WaitCycleDetectionMixin']
