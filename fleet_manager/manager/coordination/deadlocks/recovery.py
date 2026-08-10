"""Coupled replanning and deterministic wait-cycle recovery."""

from __future__ import annotations

from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.coordination.deadlocks.models import (
    _WaitCycleDecisionContext,
)


class WaitCycleRecoveryMixin:
    """Retain hidden cycle geometry and escalate an unresolved cycle safely."""

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
        context = self._wait_cycle_decision_context(
            cycle,
            waiting,
            now,
            new_episode=new_episode,
        )
        if context is None:
            return
        if not self._wait_cycle_arbitration_due(context):
            self._publish_wait_cycle_recovery_hold(context)
            return
        evacuating_name = self._escalate_wait_cycle_recovery(context)
        # Preserve the legacy evaluation point: this lease is read even when
        # evacuation makes the cycle ineligible for a direct priority grant.
        lease_until = now + self._deadlock_priority_lease()
        granted = self._grant_wait_cycle_priority(
            context,
            evacuating_name,
            lease_until,
        )
        if not granted:
            self._publish_wait_cycle_recovery_hold(context)

    def _publish_wait_cycle_recovery_hold(
        self,
        context: _WaitCycleDecisionContext,
    ) -> None:
        """Publish an acyclic safe hold while the next recovery is debounced.

        A failed lease or unavailable corridor handoff must keep every robot
        stopped. Leaving their old mutual ``yield to`` reasons untouched,
        however, exposes a wait-for cycle to the operator for one or more
        ticks. Point the component at one stationary recovery root instead.
        """
        root = context.winner
        root.status = "WAITING"
        root.last_reason = "deadlock recovery pending"
        root.traffic_priority_until = 0.0
        self._clear_wait_dependency(root)
        root.updated_at = context.now
        for robot in context.robots:
            if robot.name == root.name or not robot.trajectory:
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {root.name}"
            robot.traffic_priority_until = 0.0
            robot.wait_for_robot = root.name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = 0.0
            robot.blocked_since = robot.blocked_since or context.now
            robot.updated_at = context.now

    def _wait_cycle_decision_context(
        self,
        cycle: list[str],
        waiting: dict[str, FleetRobot],
        now: float,
        *,
        new_episode: bool,
    ) -> _WaitCycleDecisionContext | None:
        """Capture winner, corridor handoff and episode facts once."""
        robots = [waiting[name] for name in cycle if name in waiting]
        if len(robots) < 2:
            return None
        corridor_owner = self._controlled_corridor_cycle_owner(robots)
        downstream_clearer = self._controlled_corridor_downstream_clearer(
            robots,
        )
        winner = downstream_clearer or corridor_owner or min(
            robots,
            key=lambda robot: self._wait_cycle_priority_key(
                robot,
                robots,
                now,
            ),
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
                (
                    robot.traffic_stall_since
                    or robot.blocked_since
                    or now
                    for robot in robots
                ),
                default=now,
            ),
        )
        cycle_wait = max(0.0, now - cycle_started)
        for robot in robots:
            robot.traffic_stall_since = (
                robot.traffic_stall_since or cycle_started
            )
        return _WaitCycleDecisionContext(
            robots=robots,
            now=now,
            new_episode=new_episode,
            corridor_owner=corridor_owner,
            winner=winner,
            corridor_handoff_required=corridor_handoff_required,
            cycle_key=cycle_key,
            grant_already_failed=grant_already_failed,
            cycle_wait=cycle_wait,
        )

    def _wait_cycle_priority_key(
        self,
        robot: FleetRobot,
        robots: list[FleetRobot],
        now: float,
    ) -> tuple[float, int, float, float, str]:
        """Rank geometry before order priority, lease age and identity."""
        order = self._active_order_for_robot(robot)
        priority = int(order.priority if order is not None else 0)
        waited = now - (robot.blocked_since or now)
        forward_clearance = self._cycle_forward_clearance(robot, robots)
        return (
            -forward_clearance,
            -priority,
            robot.traffic_priority_until,
            -waited,
            robot.name,
        )

    def _wait_cycle_arbitration_due(
        self,
        context: _WaitCycleDecisionContext,
    ) -> bool:
        """Allow at most one arbitration during each priority lease."""
        last_arbitration = self._wait_cycle_last_arbitration.get(
            context.cycle_key,
            0.0,
        )
        if (
            not context.new_episode
            and context.now - last_arbitration
            < self._deadlock_priority_lease()
        ):
            return False
        self._wait_cycle_last_arbitration[context.cycle_key] = context.now
        return True

    def _escalate_wait_cycle_recovery(
        self,
        context: _WaitCycleDecisionContext,
    ) -> str:
        """Start coupled replanning and, when due, bounded evacuation."""
        if (
            context.corridor_owner is None
            and context.cycle_wait >= self._deadlock_coupled_replan_after()
        ):
            self._start_async_coupled_replan(
                context.robots,
                context.winner,
                context.now,
            )
        if (
            context.corridor_owner is not None
            and context.cycle_wait >= self._deadlock_retreat_after()
        ):
            return self._start_deadlock_corridor_evacuation(
                context.robots,
                context.winner,
                context.now,
            )
        if (
            context.cycle_wait >= self._deadlock_retreat_after()
            and self._coupled_replan_failure_count_for_members(
                context.cycle_key
            ) > 0
        ):
            return self._start_deadlock_corridor_evacuation(
                context.robots,
                context.winner,
                context.now,
            )
        return ""

    def _grant_wait_cycle_priority(
        self,
        context: _WaitCycleDecisionContext,
        evacuating_name: str,
        lease_until: float,
    ) -> bool:
        """Select, authorize and publish one deterministic priority grant."""
        evacuating_robot = (
            self.robots.get(evacuating_name)
            if evacuating_name
            else None
        )
        cycle_names = {robot.name for robot in context.robots}
        if (
            evacuating_robot is not None
            and evacuating_robot.name not in cycle_names
        ):
            for robot in context.robots:
                robot.traffic_priority_until = 0.0
                robot.updated_at = context.now
            return False

        active_evacuation = bool(
            evacuating_robot is not None
            and evacuating_robot.status == "RETREATING"
            and bool(evacuating_robot.trajectory)
        )
        priority_robot = (
            evacuating_robot
            if active_evacuation
            else context.winner
        )
        if (
            context.grant_already_failed
            and not active_evacuation
            and not context.corridor_handoff_required
        ):
            for robot in context.robots:
                robot.traffic_priority_until = 0.0
            return False
        if not priority_robot.trajectory:
            eligible = [
                robot for robot in context.robots if robot.trajectory
            ]
            if not eligible:
                return False
            priority_robot = min(
                eligible,
                key=lambda robot: self._wait_cycle_priority_key(
                    robot,
                    context.robots,
                    context.now,
                ),
            )
        lease_transferred = self._transfer_controlled_corridor_lease(
            priority_robot,
            context.robots,
            context.now,
        )
        if (
            context.corridor_handoff_required
            and not lease_transferred
        ):
            for robot in context.robots:
                robot.traffic_priority_until = 0.0
                robot.updated_at = context.now
            return False

        priority_robot.traffic_priority_until = max(
            priority_robot.traffic_priority_until,
            lease_until,
        )
        if priority_robot.status != "RETREATING":
            priority_robot.status = "MOVING"
            priority_robot.last_reason = "deadlock priority granted"
        self._clear_wait_dependency(priority_robot)
        priority_robot.blocked_since = (
            priority_robot.blocked_since or context.now
        )
        priority_robot.updated_at = context.now
        for robot in context.robots:
            if robot.name == priority_robot.name:
                continue
            if not robot.trajectory:
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {priority_robot.name}"
            robot.wait_for_robot = priority_robot.name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = lease_until
            robot.blocked_since = robot.blocked_since or context.now
            robot.updated_at = context.now
        self._wait_cycle_grant_signatures[context.cycle_key] = (
            self._wait_cycle_grant_signature(context.robots)
        )
        self.traffic_metrics["priorityGrants"] += 1
        self._event(
            "warn",
            "traffic wait cycle arbitration: priority granted to "
            f"{priority_robot.name}",
        )
        return True
