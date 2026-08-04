"""Evacuation selection, activation and parked-tail clearance."""

from __future__ import annotations

from fleet_manager.core.fleet.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.fleet.domain.models import FleetRobot
from fleet_manager.core.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_models import (
    _EvacuationCandidate,
)


RecoveryLatchKey = tuple[tuple[str, ...], str, str, int]


class EvacuationActivationMixin:
    """Choose one candidate and install its bounded recovery state transition."""

    def _start_deadlock_corridor_evacuation(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> str:
        """Choose and activate one bounded evacuation for a stalled component."""
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
        component, candidates = self._build_deadlock_evacuation_candidates(
            robots,
            winner,
            winner_regions,
        )
        if not candidates:
            return ""
        return self._activate_deadlock_evacuation(
            min(candidates),
            component,
            winner,
            winner_regions,
            recovery_latch_key,
            now,
        )

    def _activate_deadlock_evacuation(
        self,
        candidate: _EvacuationCandidate,
        robots: list[FleetRobot],
        winner: FleetRobot,
        winner_regions: set[str],
        recovery_latch_key: RecoveryLatchKey | None,
        now: float,
    ) -> str:
        """Install the chosen escape, detour transaction or reverse retreat."""
        robot = candidate.robot
        blocked_edges = self._deadlock_evacuation_blocked_edges(candidate)
        if not blocked_edges:
            return ""
        retreat_is_noop = self._deadlock_evacuation_is_noop(candidate)
        recovery_signature = self._wait_cycle_recovery_signature(
            (
                f"detour:{candidate.target_lm}"
                if retreat_is_noop
                else f"retreat:{candidate.target_lm}"
            ),
            robot,
            robots,
        )
        if not self._wait_cycle_recovery_ready(recovery_signature, now):
            return ""

        if candidate.graph_escape_route:
            graph_escape_installed = self._activate_graph_escape_recovery(
                candidate,
                winner,
                winner_regions,
                blocked_edges,
                recovery_signature,
                recovery_latch_key,
                now,
            )
            if graph_escape_installed:
                return robot.name
        if retreat_is_noop:
            return self._activate_deadlock_replan_recovery(
                robot,
                candidate,
                robots,
                winner,
                blocked_edges,
                recovery_signature,
                recovery_latch_key,
                now,
            )
        return self._activate_reverse_deadlock_retreat(
            candidate,
            winner,
            winner_regions,
            blocked_edges,
            recovery_signature,
            recovery_latch_key,
            now,
        )

    def _deadlock_evacuation_blocked_edges(
        self,
        candidate: _EvacuationCandidate,
    ) -> list[tuple[str, str]]:
        blocked_edges = self._deadlock_detour_edges(candidate.robot)
        if candidate.portal_blocked_edges:
            blocked_edges = list(dict.fromkeys([
                *blocked_edges,
                *candidate.portal_blocked_edges,
            ]))
        if not candidate.graph_escape_route:
            return blocked_edges

        # A forced escape may legitimately reverse the current approach edge.
        # Do not submit the same directed segment as both required and blocked.
        escape_edges = set(zip(
            candidate.graph_escape_route,
            candidate.graph_escape_route[1:],
        ))
        return [
            edge
            for edge in blocked_edges
            if edge not in escape_edges
        ]

    @staticmethod
    def _deadlock_evacuation_is_noop(
        candidate: _EvacuationCandidate,
    ) -> bool:
        return bool(
            abs(
                candidate.robot.route_clock - candidate.target_clock
            )
            <= 0.000001
            or candidate.retreat_is_noop_at_current_lm
        )

    def _activate_graph_escape_recovery(
        self,
        candidate: _EvacuationCandidate,
        winner: FleetRobot,
        winner_regions: set[str],
        blocked_edges: list[tuple[str, str]],
        recovery_signature: tuple[object, ...],
        recovery_latch_key: RecoveryLatchKey | None,
        now: float,
    ) -> bool:
        robot = candidate.robot
        if not self._install_graph_escape_retreat(
            robot,
            candidate.graph_escape_route,
            blocked_edges,
            now,
        ):
            # A spatial pocket can lose a transient planner-lock race. The
            # caller falls through to the transactional global replan path.
            return False
        self._set_deadlock_evacuation_cause(
            robot,
            winner,
            winner_regions,
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
            f"{robot.name} clearing corridor portal toward "
            f"{candidate.target_lm}",
        )
        return True

    def _activate_deadlock_replan_recovery(
        self,
        robot: FleetRobot,
        candidate: _EvacuationCandidate,
        robots: list[FleetRobot],
        winner: FleetRobot,
        blocked_edges: list[tuple[str, str]],
        recovery_signature: tuple[object, ...],
        recovery_latch_key: RecoveryLatchKey | None,
        now: float,
    ) -> str:
        order = self._active_order_for_robot(robot)
        if order is None:
            return ""
        parked_tail = self._queue_deadlock_portal_tail_clearance(
            winner,
            robot,
            robots,
            candidate.portal_blocked_edges,
            now,
        )
        if parked_tail:
            # Keep the loser eligible after the hidden maintenance order opens
            # the external arm; latch the body that is actually being moved.
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
                supersede_retained_route=True,
            )
        )
        if not replan_handled:
            return ""
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
                f"{robot.name}@{candidate.target_lm} queued for "
                "alternate route to the same goal",
            )
        return robot.name

    def _activate_reverse_deadlock_retreat(
        self,
        candidate: _EvacuationCandidate,
        winner: FleetRobot,
        winner_regions: set[str],
        blocked_edges: list[tuple[str, str]],
        recovery_signature: tuple[object, ...],
        recovery_latch_key: RecoveryLatchKey | None,
        now: float,
    ) -> str:
        robot = candidate.robot
        self._record_wait_cycle_recovery_attempt(recovery_signature, now)
        robot.pending_route = None
        robot.retreat_target_clock = candidate.target_clock
        robot.retreat_target_lm = candidate.target_lm
        robot.retreat_blocked_edges = blocked_edges
        self._set_deadlock_evacuation_cause(
            robot,
            winner,
            winner_regions,
        )
        robot.status = "RETREATING"
        robot.last_reason = (
            f"deadlock retreat to {candidate.target_lm} before detour"
        )
        robot.blocked_since = None
        robot.traffic_stall_since = None
        self._clear_wait_dependency(robot)
        robot.last_tick_at = now
        robot.updated_at = now
        self._event(
            "warn",
            f"{robot.name} evacuating narrow corridor back to "
            f"{candidate.target_lm}",
        )
        self._latch_controlled_corridor_recovery(
            recovery_latch_key,
            robot.name,
        )
        return robot.name

    def _set_deadlock_evacuation_cause(
        self,
        robot: FleetRobot,
        winner: FleetRobot,
        winner_regions: set[str],
    ) -> None:
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
