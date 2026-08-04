"""Candidate discovery for bounded deadlock evacuation."""

from __future__ import annotations

from fleet_manager.core.fleet.domain.models import FleetRobot
from fleet_manager.core.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_candidate_models import (
    _EvacuationCandidateProbe,
    _EvacuationSearchContext,
)
from fleet_manager.core.traffic.deadlocks.recovery.evacuation.deadlock_evacuation_models import (
    _EvacuationCandidate,
)


class EvacuationCandidateMixin:
    """Build deterministic historical-retreat and graph-escape candidates."""

    def _build_deadlock_evacuation_candidates(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        winner_regions: set[str],
    ) -> tuple[list[FleetRobot], list[_EvacuationCandidate]]:
        """Find graph-safe historical retreats or fresh escape pockets."""
        context = self._evacuation_search_context(
            robots,
            winner,
            winner_regions,
        )
        candidates: list[_EvacuationCandidate] = []
        build_probe = self._evacuation_candidate_probe
        audit_candidate = self._audit_evacuation_candidate
        append_candidate = candidates.append
        for robot in context.robots:
            probe = build_probe(context, robot)
            if probe is None:
                continue
            candidate = audit_candidate(context, probe)
            if candidate is not None:
                append_candidate(candidate)
        return context.robots, candidates

    def _evacuation_search_context(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        winner_regions: set[str],
    ) -> _EvacuationSearchContext:
        """Expand the physical queue and capture its stable depth map."""
        portal_queue_depths: dict[str, int] = {
            robot.name: (-1 if robot.name == winner.name else 0)
            for robot in robots
        }
        # Discover a bounded physical tail for both controlled portals and
        # ordinary graph-aisle head-ons.
        robots, portal_queue_depths = (
            self._controlled_corridor_portal_queue_component(
                robots,
                winner,
            )
        )
        return _EvacuationSearchContext(
            robots=robots,
            winner=winner,
            winner_regions=winner_regions,
            portal_queue_depths=portal_queue_depths,
        )

    def _evacuation_candidate_probe(
        self,
        context: _EvacuationSearchContext,
        robot: FleetRobot,
    ) -> _EvacuationCandidateProbe | None:
        """Resolve historical retreat, portal and reciprocal-wait facts."""
        winner = context.winner
        winner_regions = context.winner_regions
        if (
            robot.name == winner.name
            or not robot.trajectory
            or not robot.active_order_id
        ):
            return None
        retreat = self._previous_trajectory_lm(robot)
        if retreat is None:
            return None

        portal_queue_depth = max(
            0,
            int(context.portal_queue_depths.get(robot.name, 0)),
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
            corridor_graph.vertices.get(
                self._traffic_lm_for_robot(robot)
            )
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
        if (
            winner_regions
            and robot_regions
            and not winner_regions.intersection(robot_regions)
        ):
            # Separate physical resources in one bundle keep their owners.
            return None

        if retreats_from_occupied_portal:
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
                graph_escape_required = True
            else:
                retreat = (staging_clock, staging_lm)

        if (
            current_vertex is not None
            and current_vertex.controlled_region_ids
        ):
            safe_retreat = self._previous_safe_trajectory_lm(robot)
            if safe_retreat is None:
                return None
            retreat = safe_retreat

        target_clock, target_lm = retreat
        retreat_is_noop_at_current_lm = (
            target_lm == robot.current_lm
            and robot.pose is not None
            and self._pose_is_at_lm(robot.pose, target_lm)
        )
        reciprocal_blocker = self._candidate_has_reciprocal_blocker(
            context,
            robot,
        )
        if retreat_is_noop_at_current_lm and reciprocal_blocker:
            graph_escape_required = True
        if (
            retreat_is_noop_at_current_lm
            and portal_queue_depth > 0
        ):
            graph_escape_required = True

        return _EvacuationCandidateProbe(
            robot=robot,
            target_clock=target_clock,
            target_lm=target_lm,
            portal_queue_depth=portal_queue_depth,
            robot_regions=robot_regions,
            upcoming=upcoming,
            retreats_from_occupied_portal=(
                retreats_from_occupied_portal
            ),
            retreat_is_noop_at_current_lm=(
                retreat_is_noop_at_current_lm
            ),
            reciprocal_blocker=reciprocal_blocker,
            graph_escape_required=graph_escape_required,
        )

    def _candidate_has_reciprocal_blocker(
        self,
        context: _EvacuationSearchContext,
        robot: FleetRobot,
    ) -> bool:
        """Return whether winner and candidate directly wait for each other."""
        robot_dependency = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(robot.last_reason)
        )
        winner = context.winner
        winner_dependency = (
            str(winner.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(winner.last_reason)
        )
        return bool(
            robot_dependency == winner.name
            and winner_dependency == robot.name
        )

    def _audit_evacuation_candidate(
        self,
        context: _EvacuationSearchContext,
        probe: _EvacuationCandidateProbe,
    ) -> _EvacuationCandidate | None:
        """Audit a probe and materialize it only when the route is safe."""
        if not probe.retreat_is_noop_at_current_lm:
            probe.historical_retreat_blocker = (
                self._deadlock_retreat_path_blocker(
                    probe.robot,
                    probe.target_clock,
                )
            )
            if (
                probe.historical_retreat_blocker
                and (
                    probe.retreats_from_occupied_portal
                    or probe.reciprocal_blocker
                )
            ):
                probe.graph_escape_required = True

        if (
            probe.graph_escape_required
            and not self._build_candidate_graph_escape(
                context,
                probe,
            )
        ):
            return None
        if (
            not probe.graph_escape_route
            and not probe.retreat_is_noop_at_current_lm
            and probe.historical_retreat_blocker
        ):
            return None

        order = self._active_order_for_robot(probe.robot)
        priority = int(order.priority if order is not None else 0)
        distance = max(
            0.0,
            probe.robot.route_clock - probe.target_clock,
        )
        return _EvacuationCandidate(
            distance=distance,
            priority=priority,
            robot_name=probe.robot.name,
            robot=probe.robot,
            target_clock=probe.target_clock,
            target_lm=probe.target_lm,
            retreat_is_noop_at_current_lm=(
                probe.retreat_is_noop_at_current_lm
            ),
            graph_escape_route=probe.graph_escape_route,
            portal_blocked_edges=probe.portal_blocked_edges,
        )

    def _build_candidate_graph_escape(
        self,
        context: _EvacuationSearchContext,
        probe: _EvacuationCandidateProbe,
    ) -> bool:
        """Find and audit a fresh external holding pocket."""
        robot = probe.robot
        escape_start_lm = self._safe_replan_start_lm(robot)
        upcoming = probe.upcoming
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
        if (
            portal_src in self.landmarks
            and portal_dst in self.landmarks
        ):
            probe.portal_blocked_edges = [
                (portal_src, portal_dst),
                (portal_dst, portal_src),
            ]
        elif probe.reciprocal_blocker:
            probe.portal_blocked_edges = self._deadlock_detour_edges(
                robot
            )

        if escape_start_lm:
            probe.graph_escape_route = self._stationary_clearance_route(
                context.winner,
                robot,
                extra_blocked_edges=set(probe.portal_blocked_edges),
                avoid_controlled_regions=True,
                start_lm_override=escape_start_lm,
                require_waiter_release=bool(
                    probe.reciprocal_blocker
                    and not context.winner_regions
                ),
            )
        if len(probe.graph_escape_route) < 2:
            return True

        probe.target_clock = float(robot.route_clock)
        probe.target_lm = str(probe.graph_escape_route[-1])
        probe.retreat_is_noop_at_current_lm = True
        return not bool(
            self._graph_escape_route_current_body_blocker(
                robot,
                probe.graph_escape_route,
            )
        )
