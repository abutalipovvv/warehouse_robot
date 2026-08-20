"""Evaluate grants, ordered waits, blockers and admission reasons."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorDecisionStatus,
    CorridorSlotState,
)

from .models import (
    _CentralCorridorWaitContext,
)


class ControlledCorridorAdmissionDecisionMixin:
    """Evaluate grants, ordered waits, blockers and admission reasons."""

    def _controlled_corridor_has_grant(
        self,
        robot_name: str,
        regions: tuple[str, ...] | list[str],
    ) -> bool:
        """Return whether the central calendar admits this corridor passage."""
        required = {str(region) for region in regions if str(region)}
        if not required:
            return True
        if self._controlled_corridor_scheduler is None:
            return False

        robot = self.robots.get(robot_name)
        if (
            robot is not None
            and required.issubset(
                self._controlled_regions_for_robot(robot)
            )
        ):
            return True
        schedule = self._controlled_corridor_schedule
        slot = (
            schedule.slot_for(robot_name)
            if schedule is not None
            else None
        )
        if (
            slot is None
            or slot.state is not CorridorSlotState.COMMITTED
            or not required.issubset(slot.regions)
            or robot is None
            or int(slot.route_revision) != int(robot.route_revision)
        ):
            return False
        upcoming = self._next_controlled_corridor_entry(robot)
        if (
            upcoming is not None
            and self._controlled_corridor_entry_regions(upcoming)
            and slot.direction
            != self._controlled_corridor_flow_direction(upcoming)
        ):
            return False
        immediate_window = max(
            self._runtime_motion_step(),
            self._continuous_collision_step(),
        )
        return bool(
            slot.entry_time
            <= self._controlled_corridor_tick_now
            + immediate_window
            + 0.000001
        )

    def _controlled_corridor_entry_lookahead(self) -> float:
        return max(
            1.0,
            self._controlled_corridor_param(
                "controlled_corridor_schedule_horizon_sec",
                self._controlled_corridor_param(
                    "controlled_corridor_entry_lookahead_sec",
                    max(30.0, self._rolling_horizon()),
                ),
            ),
        )

    def _retained_route_is_superseded(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether a valid transaction has retired this route suffix.

        The trajectory remains attached while its replacement is planned so
        collision prediction and browser rendering never see an unsafe empty
        frame. It must not, however, renew an external corridor admission after
        recovery has explicitly declared that spatial suffix unusable.
        """
        state = self._runtime_replans.get(robot.name)
        if not isinstance(state, dict) or not bool(
            state.get("retained_route_superseded")
        ):
            return False
        order = self.orders.get(str(state.get("order_id") or ""))
        return bool(
            order is not None
            and robot.active_order_id == order.order_id
            and int(state.get("route_revision", -1))
            == int(robot.route_revision)
            and abs(
                float(state.get("route_clock", 0.0) or 0.0)
                - float(robot.route_clock)
            ) <= 0.000001
            and str(state.get("start_lm") or "")
            == self._safe_replan_start_lm(robot)
        )

    def _central_corridor_manages_wait(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether the central calendar already orders this wait.

        The central calendar owns admission at an *external* stop line.  It
        also owns a physical queue outside that line when the named blocker
        has an earlier slot on the same resource.  Treating that expected
        queue as a generic deadlock made followers retreat and globally
        replan after only a few seconds, even though their leader was already
        clearing the corridor.  Unexpected conflicts, bodies after entry and
        blockers absent from the calendar still go to the local wait-for
        resolver.
        """
        context = self._central_corridor_wait_context(robot)
        if context is None:
            return False
        deferred_queue_wait = bool(
            not context.admission_wait
            and context.decision is not None
            and context.decision.status
            is CorridorDecisionStatus.DEFERRED
            and self._controlled_corridor_queue_predecessor(
                robot,
                context.entry,
                context.regions,
                context.direction,
            )
            == context.blocker_name
        )
        if (
            (context.admission_wait or deferred_queue_wait)
            and context.decision is not None
            and context.decision.status
            in {
                CorridorDecisionStatus.GRANTED,
                CorridorDecisionStatus.DEFERRED,
            }
        ):
            blocker = self.robots.get(context.blocker_name)
            if self._central_corridor_wait_closes_cycle(
                robot,
                blocker,
            ):
                # A red-light waiter can close a dependency cycle either
                # directly or through bodies queued at the corridor exit.
                # Hiding this edge as an "expected queue" turns the cycle
                # into an acyclic chain and leaves a physical owner stopped
                # inside the no-wait passage. Expose the complete component
                # to deterministic local arbitration.
                return False
            if context.decision.slot is None:
                # A deferred request is still an intentional calendar
                # decision (predecessor, downstream box or horizon), not a
                # planner failure. It must remain at the authored stop line
                # without spawning a global detour every few seconds.
                return True
        slot = context.schedule.slot_for(robot.name)
        if not (
            slot is not None
            and int(slot.route_revision) == int(robot.route_revision)
            and set(context.regions).issubset(slot.regions)
            and slot.direction == context.direction
        ):
            return False
        if context.admission_wait:
            return True

        blocker = self.robots.get(context.blocker_name)
        blocker_slot = (
            context.schedule.slot_for(context.blocker_name)
            if blocker is not None
            else None
        )
        if (
            blocker is None
            or blocker_slot is None
            or int(blocker_slot.route_revision)
            != int(blocker.route_revision)
        ):
            return False
        return self._central_corridor_blocker_precedes(
            slot,
            blocker_slot,
            context.regions,
        )

    def _central_corridor_wait_context(
        self,
        robot: FleetRobot,
    ) -> _CentralCorridorWaitContext | None:
        """Collect calendar facts needed by external wait arbitration."""
        scheduler = self._controlled_corridor_scheduler
        schedule = self._controlled_corridor_schedule
        if scheduler is None or schedule is None or not robot.trajectory:
            return None
        reason = str(robot.last_reason or "")
        admission_wait = bool(
            reason.startswith("corridor admission wait at ")
            or reason.startswith(
                "corridor admission timeout: corridor admission wait at "
            )
        )
        if not admission_wait and not self._is_robot_conflict(reason):
            return None
        if (
            self._controlled_regions_for_robot(robot)
            & set(scheduler.controlled_regions)
        ):
            # A body which already crossed into a controlled resource belongs
            # to physical wait-for recovery, never to an external red light.
            return None
        entry = self._next_controlled_corridor_entry(robot)
        if entry is None:
            return None
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            not regions
            or not set(regions).issubset(scheduler.controlled_regions)
        ):
            return None
        staging_clock = float(
            entry.get("staging_clock", entry.get("entry_clock", 0.0))
            or 0.0
        )
        if (
            float(robot.route_clock) + self._runtime_motion_step()
            < staging_clock
        ):
            return None
        return _CentralCorridorWaitContext(
            schedule=schedule,
            entry=entry,
            regions=regions,
            direction=self._controlled_corridor_flow_direction(entry),
            admission_wait=admission_wait,
            decision=schedule.decisions.get(robot.name),
            blocker_name=(
                str(robot.wait_for_robot or "").strip()
                or self._robot_name_from_conflict_reason(reason)
            ),
        )

    def _central_corridor_wait_closes_cycle(
        self,
        robot: FleetRobot,
        blocker: FleetRobot | None,
    ) -> bool:
        """Return whether the named owner chain leads back to the waiter."""
        current = blocker
        visited = {robot.name}
        while current is not None and current.status == "WAITING":
            if current.name in visited:
                return current.name == robot.name
            visited.add(current.name)
            dependency = (
                str(current.wait_for_robot or "").strip()
                or self._robot_name_from_conflict_reason(
                    current.last_reason
                )
            )
            if not dependency:
                return False
            if dependency == robot.name:
                return True
            if dependency in visited:
                return False
            current = self.robots.get(dependency)
        return False

    @staticmethod
    def _central_corridor_blocker_precedes(
        slot: Any,
        blocker_slot: Any,
        regions: tuple[str, ...],
    ) -> bool:
        """Return whether calendar windows order the named blocker first."""
        robot_windows = {
            window.region_id: (
                slot.entry_time + window.entry_offset_sec,
                slot.entry_time + window.exit_offset_sec,
                window.direction,
            )
            for window in slot.resource_windows
        }
        blocker_windows = {
            window.region_id: (
                blocker_slot.entry_time + window.entry_offset_sec,
                blocker_slot.entry_time + window.exit_offset_sec,
                window.direction,
            )
            for window in blocker_slot.resource_windows
        }
        for region_id in set(regions).intersection(blocker_windows):
            robot_entry, robot_exit, robot_direction = robot_windows[
                region_id
            ]
            blocker_entry, blocker_exit, blocker_direction = blocker_windows[
                region_id
            ]
            if robot_direction == blocker_direction:
                if (
                    blocker_entry <= robot_entry + 0.000001
                    and blocker_exit <= robot_exit + 0.000001
                ):
                    return True
            elif blocker_exit <= robot_entry + 0.000001:
                return True
        return False

    def _central_corridor_owner_is_clearing(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether this body must reach the external exit first.

        A committed owner that has entered an atomic passage may not obey a
        far-horizon collision forecast *beyond* the passage exit by stopping
        inside the narrow resource.  Runtime still checks every immediate
        20–50 ms motion step and the global swept-footprint invariant; this
        flag only suppresses the distant preflight freeze until the complete
        body clears the authored corridor.
        """
        passage = self._controlled_corridor_passages.get(robot.name)
        if not isinstance(passage, dict):
            return False
        try:
            passage_revision = int(
                passage.get("route_revision", -1)
            )
        except (TypeError, ValueError):
            return False
        if passage_revision != int(robot.route_revision):
            return False
        return bool(
            passage.get("entered")
            or passage.get("past_commit_point")
        )

    def _controlled_corridor_downstream_blocker(
        self,
        robot: FleetRobot,
        exit_lm: str,
        exit_clock: float,
    ) -> str:
        """Return a body not proven to clear the exit before our arrival."""
        candidate_pose = self._pose_at_trajectory(
            robot.trajectory,
            exit_clock,
        )
        if candidate_pose is None:
            exit_landmark = self.landmarks.get(exit_lm)
            if exit_landmark is None:
                return ""
            candidate_pose = {
                "x": float(exit_landmark.x),
                "y": float(exit_landmark.y),
                "yaw": float(robot.pose.get("yaw", 0.0))
                if robot.pose is not None
                else 0.0,
            }
        broadphase_distance = self.collision.robot_broadphase_distance()
        robot_entry = self._next_controlled_corridor_entry(robot)
        robot_regions = set(
            self._controlled_corridor_entry_regions(robot_entry)
        )

        def may_overlap(pose: dict[str, Any] | None) -> bool:
            if pose is None:
                return False
            return math.hypot(
                float(candidate_pose.get("x", 0.0) or 0.0)
                - float(pose.get("x", 0.0) or 0.0),
                float(candidate_pose.get("y", 0.0) or 0.0)
                - float(pose.get("y", 0.0) or 0.0),
            ) <= broadphase_distance + 0.000001

        indexed_robots = self._controlled_corridor_downstream_candidates(
            candidate_pose,
            broadphase_distance,
        )
        for other in indexed_robots:
            if other.name == robot.name or other.pose is None:
                continue
            current_conflict = bool(
                may_overlap(other.pose)
                and self.collision.robot_footprints_conflict(
                    candidate_pose,
                    other.pose,
                )
            )
            if current_conflict:
                other_entry = self._next_controlled_corridor_entry(other)
                other_regions = set(
                    self._controlled_corridor_entry_regions(other_entry)
                )
                other_stop_lm = str(
                    (other_entry or {}).get("holding_lm")
                    or (other_entry or {}).get("src")
                    or ""
                )
                if (
                    robot_regions.intersection(other_regions)
                    and not self._controlled_regions_for_robot(other)
                    and self._controlled_corridor_pose_is_at_lm(
                        other.pose,
                        other_stop_lm,
                    )
                ):
                    # Two externally staged requests for the same resource
                    # are ordered by the central calendar. Authored staging
                    # clearance keeps the losing body outside the exit box.
                    continue
                # Strict no-box admission: a body which occupies the exit
                # pocket must clear it physically before a new robot enters
                # the no-wait passage. A predicted departure is not enough;
                # that departure can itself be delayed by downstream traffic
                # after admission and strand the new owner inside.
                return other.name
            prediction_offset = max(
                0.0,
                float(exit_clock) - float(robot.route_clock),
            )
            predicted_at_terminal = False
            if other.trajectory:
                final_clock = float(
                    other.trajectory[-1].get("t", 0.0) or 0.0
                )
                predicted_at_terminal = (
                    float(other.route_clock) + prediction_offset
                    >= final_clock - self._runtime_motion_step()
                )
            if not predicted_at_terminal:
                # Only terminal arrivals can become a new hard downstream
                # blocker. Moving/moving crossings stay in temporal SIPP.
                continue
            predicted_pose = self._predicted_robot_pose(
                other,
                prediction_offset,
            )
            predicted_conflict = bool(
                may_overlap(predicted_pose)
                and self.collision.robot_footprints_conflict(
                    candidate_pose,
                    predicted_pose,
                )
            )
            # Future moving/moving crossings remain SIPP's responsibility.
            # A trajectory which *ends* in the exit pocket is different:
            # there is no committed departure after its arrival, so admitting
            # this corridor would knowingly block the box.
            if predicted_conflict:
                return other.name
        return ""

    def _controlled_corridor_downstream_candidates(
        self,
        candidate_pose: dict[str, Any],
        broadphase_distance: float,
    ) -> list[FleetRobot]:
        """Return the exact broadphase superset near one corridor exit."""

        bucket_size = (
            self.traffic_state.controlled_corridor_downstream_bucket_size
        )
        buckets = self.traffic_state.controlled_corridor_downstream_buckets
        if bucket_size <= 0.0:
            return list(self._runtime_robots())
        cell_x = math.floor(
            float(candidate_pose.get("x", 0.0) or 0.0) / bucket_size
        )
        cell_y = math.floor(
            float(candidate_pose.get("y", 0.0) or 0.0) / bucket_size
        )
        cell_radius = max(
            1,
            int(math.ceil(broadphase_distance / bucket_size)),
        )
        candidates: dict[str, FleetRobot] = {}
        for offset_x in range(-cell_radius, cell_radius + 1):
            for offset_y in range(-cell_radius, cell_radius + 1):
                candidates.update(
                    buckets.get((cell_x + offset_x, cell_y + offset_y), {})
                )
        return [candidates[name] for name in sorted(candidates)]

    def _controlled_corridor_physical_exit_time(
        self,
        robot: FleetRobot,
        exit_lm: str,
        now: float,
    ) -> float:
        """Project an entered owner's safe exit from its live route clock.

        Reusing an overdue pre-entry slot as physical occupancy collapsed its
        exit to ``now + occupancy_recheck`` on every tick.  The first future
        trajectory sample at the atomic bundle's external exit is a safer and
        much more stable estimate: while a robot moves, simulation time and
        route time advance together; while it waits, the estimate moves forward
        and the corridor remains protected.
        """
        remaining = self._runtime_motion_step()
        route_clock = float(robot.route_clock)
        for sample in robot.trajectory:
            if not isinstance(sample, dict):
                continue
            sample_clock = float(sample.get("t", 0.0) or 0.0)
            if sample_clock + 0.000001 < route_clock:
                continue
            if str(sample.get("lm") or "") != exit_lm:
                continue
            remaining = max(remaining, sample_clock - route_clock)
            break
        return float(now) + remaining

    def _controlled_corridor_admission_reason(
        self,
        robot: FleetRobot,
        check_clock: float,
    ) -> str:
        graph = self._controlled_corridor_graph
        if (
            graph is None
            or not robot.trajectory
            or robot.status == "RETREATING"
        ):
            return ""
        inside = self._controlled_regions_for_robot(robot)
        upcoming = self._next_controlled_corridor_entry(robot)
        if upcoming is not None:
            region_id = str(upcoming["region"])
            regions = self._controlled_corridor_entry_regions(upcoming)
            if inside.intersection(regions):
                # Admission is an external stop-line decision. Once any part
                # of this atomic passage is physically occupied, the robot is
                # past that decision and must keep clearing toward the safe
                # exit. The live calendar deliberately drops resource windows
                # which are already behind the footprint; requiring the
                # original full bundle again here would revoke a valid grant
                # between adjacent corridor sections and stop the robot in
                # the no-wait passage.
                return ""
            entry_lm = str(upcoming["src"])
            holding_lm = str(upcoming.get("holding_lm") or "")
            stop_lm = holding_lm or entry_lm
            at_stop_line = self._controlled_corridor_pose_is_at_lm(
                robot.pose,
                stop_lm,
            )
            # A physics substep may straddle a graph sample without ever
            # placing the pose inside the old 3 cm stop-line tolerance.  Gate
            # the crossing interval itself, not just the sampled position.
            staging_clock = float(
                upcoming.get(
                    "staging_clock",
                    upcoming.get("entry_clock", robot.route_clock),
                )
                or 0.0
            )
            immediate_window = max(
                self._runtime_motion_step(),
                self._continuous_collision_step(),
            )
            crossing_boundary = bool(
                check_clock >= staging_clock - 0.000001
                and robot.route_clock <= staging_clock + 0.000001
                and check_clock - robot.route_clock
                <= immediate_window + 0.000001
            )
            staging_motion_complete = bool(
                float(robot.route_clock)
                >= staging_clock - 0.000001
            )
            passed_staging = bool(upcoming.get("passed_staging"))
            if (
                region_id not in inside
                and (
                    (at_stop_line and staging_motion_complete)
                    or crossing_boundary
                    or passed_staging
                )
                and not self._controlled_corridor_has_grant(
                    robot.name,
                    regions,
                )
            ):
                return self._controlled_corridor_wait_reason(
                    robot,
                    stop_lm,
                    region_id,
                )
            # A complete atomic passage was identified, so its route-clock
            # stop line is authoritative. The lane-level compatibility
            # fallback below would otherwise see the same edge while the
            # robot is merely rotating at its holding LM and stop that turn.
            return ""
        if (
            check_clock - robot.route_clock
            > max(
                self._runtime_motion_step(),
                self._continuous_collision_step(),
            )
            + 0.000001
        ):
            return ""
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, check_clock)
        )
        if edge is None:
            return ""
        src, dst = edge
        lane = graph.lane_for(src, dst)
        if lane is None or not lane.controlled_region_ids:
            return ""
        src_vertex = graph.vertices.get(src)
        lane_regions = tuple(lane.controlled_region_ids)
        for region_id in lane_regions:
            if region_id in inside:
                continue
            if src_vertex is not None and (
                not src_vertex.can_wait
                or src_vertex.controlled_region_ids
            ):
                # A legacy robot already inside a no-wait passage must clear
                # it; new entrants are protected by the atomic bundle above.
                # This also covers auto-detected transfer junctions which are
                # deliberately non-waitable but do not belong to either
                # adjacent corridor resource themselves.
                continue
            if self._controlled_corridor_has_grant(
                robot.name,
                lane_regions,
            ):
                continue
            return self._controlled_corridor_wait_reason(
                robot,
                src,
                region_id,
            )
        return ""

    def _transfer_controlled_corridor_lease(
        self,
        winner: FleetRobot,
        participants: list[FleetRobot],
        now: float,
    ) -> bool:
        """Confirm a central-calendar owner during deadlock arbitration."""
        del participants, now
        if self._controlled_corridor_scheduler is None:
            return False

        # Generic wait-cycle arbitration may confirm the calendar owner, but
        # must never repaint corridor authority independently.
        upcoming = self._next_controlled_corridor_entry(winner)
        regions = list(
            self._controlled_corridor_entry_regions(upcoming)
        )
        for current_region in self._controlled_regions_for_robot(winner):
            if current_region not in regions:
                regions.append(current_region)
        return bool(
            regions
            and self._controlled_corridor_has_grant(
                winner.name,
                regions,
            )
        )

    def _controlled_corridor_wait_reason(
        self,
        robot: FleetRobot,
        stop_lm: str,
        region_id: str,
    ) -> str:
        owners = self._controlled_corridor_occupancy.get(region_id, [])
        lease = self._controlled_corridor_leases.get(region_id, ("", 0.0))
        owner = (
            owners[0]
            if owners
            else lease[0]
            or self._controlled_corridor_blockers.get(robot.name, "")
        )
        suffix = f"; owner {owner}" if owner else ""
        reason = (
            f"corridor admission wait at {stop_lm} for {region_id}{suffix}"
        )
        if robot.last_reason != reason:
            self.traffic_metrics["corridorAdmissionWaits"] += 1
        return reason
