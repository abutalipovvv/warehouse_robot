"""Runtime collision look-ahead and right-of-way rules."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot


class RuntimeConflictMixin:
    """Evaluate predicted conflicts and maintain explicit wait dependencies."""

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

    def _next_safe_holding_clock(
        self,
        robot: FleetRobot,
    ) -> float | None:
        """Return the end of the next indivisible graph passage.

        Runtime collision lookahead used to be bounded only by seconds.  A
        slow turn or a long reservation-stretched edge could therefore leave
        a graph LM before its destination was checked, then stop halfway when
        the same conflict entered the time window.  A warehouse robot may
        wait at a graph-safe LM, never by deliberately filling an edge.

        On ordinary graph sections the next LM is the passage end.  Inside an
        authored controlled corridor, internal/portal LMs are non-waitable, so
        the passage extends atomically to the first external waitable LM.
        """
        if not robot.trajectory or robot.pose is None:
            return None
        start_lm = self._safe_replan_start_lm(robot)
        if start_lm not in self.landmarks:
            return None

        graph = self._controlled_corridor_graph
        if graph is not None:
            start_vertex = graph.vertices.get(start_lm)
            if (
                start_vertex is not None
                and (
                    not start_vertex.can_wait
                    or start_vertex.controlled_region_ids
                )
            ):
                # The robot has already crossed the last safe stop line.
                return None

        for sample in robot.trajectory:
            sample_clock = float(sample.get("t", 0.0) or 0.0)
            if sample_clock <= robot.route_clock + 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name not in self.landmarks or lm_name == start_lm:
                continue
            if graph is not None:
                vertex = graph.vertices.get(lm_name)
                if (
                    vertex is not None
                    and (
                        not vertex.can_wait
                        or vertex.controlled_region_ids
                    )
                ):
                    continue
            return sample_clock
        return None

    def _blocked_ahead(self, robot: FleetRobot, proposed_clock: float) -> str:
        if not robot.trajectory:
            return ""
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        lookahead = self.collision.lookahead_time()
        safe_holding_clock = self._next_safe_holding_clock(robot)
        if safe_holding_clock is not None:
            # Admission is decided while the robot is still centred on its
            # previous safe LM.  Cover the complete edge, turn, or authored
            # no-wait corridor even when it is longer than the time lookahead.
            lookahead = max(
                lookahead,
                safe_holding_clock - proposed_clock,
            )
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
        elif safe_holding_clock is not None:
            end_clock = max(
                end_clock,
                min(final_time, safe_holding_clock),
            )
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
            admission_reason = self._runtime_admission_reason(
                robot,
                check_clock,
            )
            if admission_reason:
                return admission_reason
        # Use elapsed time from the common beginning-of-tick clock. Earlier
        # robots in the loop may already have advanced their mutable clock;
        # basing the offset on that value makes pair predictions asynchronous.
        robot_clock = self._runtime_tick_route_clocks.get(
            robot.name,
            robot.route_clock,
        )
        offset = max(0.0, check_clock - robot_clock)
        future_prediction = offset > self._runtime_motion_step() + 0.000001
        reason = self._runtime_obstacle_reason(
            pose,
            future_prediction=future_prediction,
        )
        if reason:
            return reason
        others = (
            robot_candidates
            if robot_candidates is not None
            else self._runtime_robots()
        )
        for other in others:
            reason = self._runtime_peer_conflict_reason(
                robot,
                other,
                check_clock,
                pose,
                offset,
                future_prediction=future_prediction,
            )
            if reason:
                return reason
        return ""

    def _runtime_admission_reason(
        self,
        robot: FleetRobot,
        check_clock: float,
    ) -> str:
        corridor_reason = self._controlled_corridor_admission_reason(
            robot,
            check_clock,
        )
        if corridor_reason:
            return corridor_reason
        return self._traffic_zone_admission_reason(robot, check_clock)

    def _runtime_obstacle_reason(
        self,
        pose: dict[str, Any],
        *,
        future_prediction: bool,
    ) -> str:
        if future_prediction:
            if not self.obstacles and not self.obstacle_areas:
                return ""
            return self.collision.dynamic_blocked_reason(
                pose=pose,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
        # Static map occupancy is checked on the immediate motion step.
        # Graph trajectories were already audited against the map, so
        # repeating the expensive footprint raster scan at every distant
        # lookahead sample adds CPU load without adding safety.
        return self.collision.blocked_reason(
            pose=pose,
            obstacles=self.obstacles,
            obstacle_areas=self.obstacle_areas,
        )

    def _runtime_peer_conflict_reason(
        self,
        robot: FleetRobot,
        other: FleetRobot,
        check_clock: float,
        pose: dict[str, Any],
        offset: float,
        *,
        future_prediction: bool,
    ) -> str:
        if other.name == robot.name or other.pose is None:
            return ""
        other_pose = self._predicted_robot_pose(other, offset)
        if other_pose is None:
            return ""

        sweep_reason = self._immediate_sweep_conflict_reason(
            robot,
            other,
            check_clock,
            pose,
            other_pose,
            offset,
        )
        if sweep_reason:
            return sweep_reason

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
                return ""
            # The circumscribed circle is reject-only. Adjacent graph lanes
            # can be closer while the oriented rectangular bodies remain
            # disjoint, so exact footprint geometry stays authoritative.
            if not self.collision.robot_footprints_conflict(pose, other_pose):
                return ""
        elif not self.collision.robot_footprints_conflict(pose, other_pose):
            return ""

        return self._robot_conflict_reason(
            robot,
            other,
            pose,
            other_pose,
            prediction_offset=offset,
        )

    def _immediate_sweep_conflict_reason(
        self,
        robot: FleetRobot,
        other: FleetRobot,
        check_clock: float,
        pose: dict[str, Any],
        other_pose: dict[str, Any],
        offset: float,
    ) -> str:
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
            incremental_dt > self._runtime_motion_step() + 0.000001
            or segment_start_pose is None
            or other_segment_start is None
        ):
            return ""

        # Endpoint-only checks can miss two rectangular bodies whose swept
        # areas cross between motion samples. Pair prediction must also assume
        # that the peer can stop later in the same sequential tick.
        predicted_sweep_conflict = self._swept_footprints_overlap(
            segment_start_pose,
            pose,
            other_segment_start,
            other_pose,
        )
        has_right_of_way = self._has_right_of_way(robot, other)
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
        unsafe_stationary_body = (
            stationary_endpoint_conflict
            or stationary_sweep_conflict
        ) and not existing_overlap_escape
        unsafe_predicted_sweep = (
            predicted_sweep_conflict
            and not has_right_of_way
            and not existing_overlap_escape
        )
        if not unsafe_stationary_body and not unsafe_predicted_sweep:
            return ""
        if self._is_active_traffic(other):
            return f"yield to {other.name}"
        return f"occupied by {other.name}"

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

        if (
            other.pose is not None
            and self.collision.robot_footprints_conflict(
                candidate_pose,
                other.pose,
            )
        ):
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
