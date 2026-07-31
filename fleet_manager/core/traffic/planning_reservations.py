"""Traffic reservation construction and trajectory lookup helpers."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.route_core.models import PlannedRoute


class TrafficReservationMixin:
    """Build graph reservations from live robot trajectories and obstacles."""

    def _dynamic_blocked_edges(self) -> set[tuple[str, str]]:
        blocked = set(self._static_blocked_edges)
        if not self.obstacles and not self.obstacle_areas:
            return blocked
        for edge in self.edges:
            route = PlannedRoute(
                nodes=[edge.from_name, edge.to_name],
                edges=[edge],
                length=edge.length,
            )
            try:
                samples = self.planner.route_planner.sample_route(route, sample_distance=0.15)
            except Exception:
                continue
            for sample in samples:
                pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                }
                reason = self.collision.dynamic_blocked_reason(
                    pose,
                    self.obstacles,
                    self.obstacle_areas,
                )
                if reason:
                    blocked.add((edge.from_name, edge.to_name))
                    break
        return blocked

    def _static_map_blocked_edges(self) -> set[tuple[str, str]]:
        if self.collision.map_pixels is None or self.collision.map_metadata is None:
            return set()
        blocked: set[tuple[str, str]] = set()
        for edge in self.edges:
            route = PlannedRoute(
                nodes=[edge.from_name, edge.to_name],
                edges=[edge],
                length=edge.length,
            )
            try:
                samples = self.planner.route_planner.sample_route(
                    route,
                    sample_distance=0.10,
                )
            except Exception:
                blocked.add((edge.from_name, edge.to_name))
                continue
            for sample in samples:
                pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                }
                if self.collision.blocked_reason(pose, [], []):
                    blocked.add((edge.from_name, edge.to_name))
                    break
        return blocked

    def _hard_blocked_edges(self, payload: dict[str, Any]) -> set[tuple[str, str]]:
        raw_edges = payload.get("blocked_edges") or payload.get("blockedEdges") or []
        if not isinstance(raw_edges, list):
            return set()
        blocked: set[tuple[str, str]] = set()
        for item in raw_edges:
            if isinstance(item, str) and "->" in item:
                src, dst = item.split("->", 1)
                src = src.strip()
                dst = dst.strip()
            elif isinstance(item, dict):
                src = str(item.get("from") or item.get("fromLm") or "").strip()
                dst = str(item.get("to") or item.get("toLm") or "").strip()
            else:
                continue
            if src in self.landmarks and dst in self.landmarks:
                blocked.add((src, dst))
        return blocked

    def _reserved_constraints(
        self,
        requests: list[dict[str, Any]],
    ) -> tuple[list[tuple[int, str]], list[tuple[int, str, str]]]:
        request_names = {str(request.get("name", "")) for request in requests}
        time_step = self._reservation_time_step()
        horizon = self._reservation_horizon()
        vertices: set[tuple[int, str]] = set()
        edges: set[tuple[int, str, str]] = set()

        for robot in self._runtime_robots():
            if robot.name in request_names:
                continue
            if not robot.trajectory or robot.status not in {"MOVING", "WAITING"}:
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            tick = 0
            offset = 0.0
            while offset <= horizon + 0.000001:
                sample_clock = min(final_time, robot.route_clock + offset)
                pose = self._pose_at_trajectory(robot.trajectory, sample_clock)
                if pose is None:
                    break
                lm_name = self._nearest_lm_for_pose(pose)
                if lm_name:
                    vertices.add((tick, lm_name))
                edge_id = self._edge_id_at_trajectory(robot.trajectory, sample_clock)
                edge = self._parse_edge_id(edge_id)
                if edge is not None:
                    src, dst = edge
                    edges.add((tick, src, dst))
                    edges.add((tick, dst, src))
                if sample_clock >= final_time:
                    break
                tick += 1
                offset += time_step
        return sorted(vertices), sorted(edges)

    def _reserved_edge_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, str, float, float, str]]:
        request_names = {str(request.get("name", "")) for request in requests}
        ignored = ignore_robot_names or set()
        horizon = self._reservation_horizon()
        safety = self._reservation_safety_time()
        intervals: list[tuple[str, str, float, float, str]] = []
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.name in ignored:
                continue
            if robot.status not in {"MOVING", "WAITING"} or len(robot.trajectory) < 2:
                continue
            trajectory = robot.trajectory
            future_clock = robot.route_clock + max(0.0, prediction_offset)
            first_relevant_clock = max(
                float(trajectory[0].get("t", 0.0) or 0.0),
                future_clock - safety,
            )
            first_index = max(
                0,
                self._trajectory_segment_index(
                    trajectory,
                    first_relevant_clock,
                    boundary_belongs_to_previous=True,
                ) - 1,
            )
            active_edge: tuple[str, str] | None = None
            active_start = 0.0
            active_end = 0.0

            def flush_edge() -> None:
                nonlocal active_edge, active_start, active_end
                if active_edge is None:
                    return
                start_time = active_start - safety
                end_time = active_end + safety
                if end_time >= 0.0 and start_time <= horizon:
                    src, dst = active_edge
                    intervals.append(
                        (
                            src,
                            dst,
                            max(0.0, start_time),
                            min(horizon, end_time),
                            robot.name,
                        )
                    )
                active_edge = None

            for index in range(first_index, len(trajectory) - 1):
                start = trajectory[index]
                end = trajectory[index + 1]
                start_time = float(start.get("t", 0.0) or 0.0) - future_clock
                end_time = float(end.get("t", 0.0) or 0.0) - future_clock
                if end_time < -safety:
                    continue
                if start_time > horizon + safety:
                    break
                edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
                edge = self._parse_edge_id(edge_id)
                if edge is None:
                    flush_edge()
                    continue
                if edge != active_edge:
                    flush_edge()
                    active_edge = edge
                    active_start = start_time
                    active_end = end_time
                else:
                    active_end = end_time
            flush_edge()
        return intervals

    def _reserved_vertex_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        ignore_nodes: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, float, float, str]]:
        request_names = {str(request.get("name", "")) for request in requests}
        ignored = ignore_robot_names or set()
        skipped_nodes = ignore_nodes or set()
        horizon = self._reservation_horizon()
        safety = self._reservation_safety_time()
        intervals: list[tuple[str, float, float, str]] = []

        def add_interval(node: str, start: float, end: float, owner: str) -> None:
            if node not in self.landmarks:
                return
            if node in skipped_nodes:
                return
            start_time = max(0.0, min(start, end))
            end_time = min(horizon, max(start, end))
            if end_time < 0.0 or start_time > horizon:
                return
            intervals.append((node, start_time, end_time, owner))

        for robot in self._runtime_robots():
            if robot.name in request_names or robot.name in ignored:
                continue

            future_pose = self._predicted_robot_pose(robot, max(0.0, prediction_offset))
            current_lm = (
                self._nearest_lm_for_pose(future_pose)
                if future_pose is not None
                else self._nearest_lm_for_robot(robot)
            )
            if current_lm:
                add_interval(current_lm, 0.0, safety * 2.0, robot.name)

            if robot.status not in {"MOVING", "WAITING"} or len(robot.trajectory) < 2:
                if current_lm:
                    add_interval(current_lm, 0.0, horizon, robot.name)
                continue

            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            future_clock = robot.route_clock + max(0.0, prediction_offset)
            if future_clock >= final_time:
                if current_lm:
                    add_interval(current_lm, 0.0, horizon, robot.name)
                continue

            active_edge: tuple[str, str] | None = None
            active_start = 0.0
            active_end = 0.0
            first_relevant_clock = max(
                float(robot.trajectory[0].get("t", 0.0) or 0.0),
                future_clock - safety,
            )
            first_index = max(
                0,
                self._trajectory_segment_index(
                    robot.trajectory,
                    first_relevant_clock,
                    boundary_belongs_to_previous=True,
                ) - 1,
            )

            def flush_edge_vertices() -> None:
                nonlocal active_edge
                if active_edge is None:
                    return
                src, dst = active_edge
                add_interval(src, active_start - safety, active_start + safety, robot.name)
                add_interval(dst, active_end - safety, active_end + safety, robot.name)
                active_edge = None

            for index in range(first_index, len(robot.trajectory) - 1):
                start = robot.trajectory[index]
                end = robot.trajectory[index + 1]
                start_time = float(start.get("t", 0.0) or 0.0) - future_clock
                end_time = float(end.get("t", 0.0) or 0.0) - future_clock
                if end_time < -safety:
                    continue
                if start_time > horizon + safety:
                    break

                edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
                edge = self._parse_edge_id(edge_id)
                if edge is not None:
                    if edge != active_edge:
                        flush_edge_vertices()
                        active_edge = edge
                        active_start = start_time
                        active_end = end_time
                    else:
                        active_end = end_time
                    continue

                flush_edge_vertices()
                wait_lm = self._lm_from_wait_segment(start, end)
                if wait_lm:
                    add_interval(wait_lm, start_time - safety, end_time + safety, robot.name)
            flush_edge_vertices()

            final_time = (
                float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                - future_clock
            )
            final_lm = robot.target_lm if robot.target_lm in self.landmarks else self._nearest_lm_for_robot(robot)
            if final_lm and final_time <= horizon:
                add_interval(final_lm, final_time - safety, horizon, robot.name)
        return intervals

    def _lm_from_wait_segment(
        self,
        start: dict[str, Any],
        end: dict[str, Any],
    ) -> str:
        for sample in (end, start):
            lm = str(sample.get("lm") or "").strip()
            if lm in self.landmarks:
                return lm
        edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
        if edge_id.startswith("WAIT@"):
            edge_id = edge_id[5:]
        if "->" in edge_id:
            src, dst = edge_id.split("->", 1)
            src = src.strip()
            dst = dst.strip()
            if src == dst and src in self.landmarks:
                return src
        pose = self._pose_from_sample(end)
        return self._nearest_lm_for_pose(pose)

    def _reservation_time_step(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(0.25, float(fleet.get("reservation_time_step_sec", 1.0) or 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _reservation_horizon(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            configured = 8.0
        else:
            try:
                configured = max(
                    1.0,
                    float(fleet.get("reservation_horizon_sec", 8.0) or 8.0),
                )
            except (TypeError, ValueError):
                configured = 8.0
        corridor_ticks = self.planner.controlled_corridor_max_ticks()
        if corridor_ticks <= 0:
            return configured
        # A committed robot must remain visible until it has left the longest
        # controlled region. Clipping its edge reservations to the ordinary
        # rolling window lets a later request schedule an entry while the
        # first robot is still physically inside the corridor.
        corridor_sec = corridor_ticks * self._reservation_time_step()
        return max(
            configured,
            corridor_sec + (2.0 * self._reservation_safety_time()),
        )

    def _rolling_horizon(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 10.0
        try:
            configured = max(
                0.0,
                float(fleet.get("rolling_horizon_sec", 10.0) or 0.0),
            )
        except (TypeError, ValueError):
            configured = 10.0
        raw_scale_enabled = fleet.get(
            "rolling_horizon_scale_with_simulation_time",
            True,
        )
        if isinstance(raw_scale_enabled, str):
            scale_enabled = raw_scale_enabled.strip().lower() not in {
                "",
                "0",
                "false",
                "no",
                "off",
                "disabled",
            }
        else:
            scale_enabled = bool(raw_scale_enabled)
        if not scale_enabled or configured <= 0.0:
            return configured
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        try:
            maximum = max(
                configured,
                float(fleet.get("rolling_horizon_max_sec", 120.0) or 120.0),
            )
        except (TypeError, ValueError):
            maximum = max(configured, 120.0)
        # Motion clocks accelerate, MAPF computation does not. Without this
        # wall-time invariant window, 10 simulated seconds become only 2.5
        # real seconds at 4x and the fleet reaches every rolling boundary
        # before the serialized planner can prepare its continuation.
        return min(maximum, configured * time_scale)

    def _rolling_horizon_steps(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0
        try:
            return max(0, int(fleet.get("rolling_horizon_steps", 0) or 0))
        except (TypeError, ValueError):
            return 0

    def _reservation_safety_time(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.35
        try:
            return max(0.05, float(fleet.get("reservation_safety_time_sec", 0.35) or 0.35))
        except (TypeError, ValueError):
            return 0.35

    def _continuous_collision_step(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.10
        try:
            return max(0.04, min(0.25, float(fleet.get("continuous_collision_step_sec", 0.10) or 0.10)))
        except (TypeError, ValueError):
            return 0.10

    def _batch_collision_horizon(self, final_time: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return max(1.0, final_time)
        raw_value = fleet.get("batch_collision_horizon_sec")
        if raw_value is None:
            return max(1.0, final_time)
        try:
            return max(1.0, float(raw_value))
        except (TypeError, ValueError):
            return max(1.0, final_time)

    def _batch_wait_max_iterations(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 60
        try:
            return max(1, int(fleet.get("batch_wait_max_iterations", 60) or 60))
        except (TypeError, ValueError):
            return 60

    def _nearest_lm_for_pose(self, pose: dict[str, float]) -> str:
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(
                landmark.x - float(pose.get("x", 0.0) or 0.0),
                landmark.y - float(pose.get("y", 0.0) or 0.0),
            ),
        )
        return nearest.name

    def _edge_id_at_trajectory(self, trajectory: list[dict[str, Any]], elapsed: float) -> str:
        if not trajectory:
            return ""
        if len(trajectory) == 1 or elapsed <= float(trajectory[0].get("t", 0.0) or 0.0):
            return str(trajectory[0].get("edgeId", "") or "")
        index = self._trajectory_segment_index(
            trajectory,
            elapsed,
            boundary_belongs_to_previous=True,
        )
        start = trajectory[index]
        goal = trajectory[index + 1]
        return str(goal.get("edgeId") or start.get("edgeId") or "")

    @staticmethod
    def _trajectory_segment_index(
        trajectory: list[dict[str, Any]],
        elapsed: float,
        *,
        boundary_belongs_to_previous: bool = False,
    ) -> int:
        """Find the active monotonic trajectory segment in O(log N)."""
        if len(trajectory) < 2:
            return 0
        low = 0
        high = len(trajectory) - 1
        while low + 1 < high:
            middle = (low + high) // 2
            middle_time = float(trajectory[middle].get("t", 0.0) or 0.0)
            before_or_at = (
                middle_time < elapsed
                if boundary_belongs_to_previous
                else middle_time <= elapsed
            )
            if before_or_at:
                low = middle
            else:
                high = middle
        return min(low, len(trajectory) - 2)

    @staticmethod
    def _trajectory_sample_index_at_or_before(
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> int:
        low = 0
        high = len(trajectory)
        while low < high:
            middle = (low + high) // 2
            middle_time = float(trajectory[middle].get("t", 0.0) or 0.0)
            if middle_time <= elapsed:
                low = middle + 1
            else:
                high = middle
        return low - 1

    def _parse_edge_id(self, edge_id: str) -> tuple[str, str] | None:
        if "->" not in edge_id:
            return None
        src, dst = edge_id.split("->", 1)
        src = src.strip()
        dst = dst.strip()
        if not src or not dst or src == dst:
            return None
        if src not in self.landmarks or dst not in self.landmarks:
            return None
        return src, dst

    def _hard_blocked_lms(self, payload: dict[str, Any]) -> set[str]:
        blocked_lms = payload.get("blocked_lms", [])
        if not isinstance(blocked_lms, list):
            return set()
        return {
            str(name)
            for name in blocked_lms
            if isinstance(name, str) and name in self.landmarks
        }

    def _soft_blocked_lms(
        self,
        requests: list[dict[str, Any]],
        hard_blocked_lms: set[str],
    ) -> set[str]:
        request_names = {request["name"] for request in requests}
        protected_lms = set(hard_blocked_lms)
        for request in requests:
            protected_lms.add(request["startLm"])

        # Goal LMs are intentionally not protected: arriving through a robot
        # parked on the goal is still a collision. The order waits until that
        # body moves instead of receiving a physically impossible route.
        return self._stationary_robot_blocked_lms(
            exclude_robot_names=request_names,
        ) - protected_lms

    def _stationary_blockers_named_by_failure(
        self,
        reason: str,
        candidate_lms: set[str],
        *,
        request_names: set[str],
    ) -> list[str]:
        """Resolve a failed resource to an actual inactive robot body.

        Planner failure strings name the constrained LM (for example
        ``resource_constrained:S003013@7``), but do not always include the
        owner robot. Only an LM that is both named in that diagnostic and in
        the soft stationary occupancy set is causal enough to relocate.
        """
        text = str(reason or "")
        mentioned_lms = {
            lm_name
            for lm_name in candidate_lms
            if lm_name and lm_name in text
        }
        if not mentioned_lms:
            return []
        blockers: list[str] = []
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.trajectory:
                continue
            if (
                robot.status in {"IDLE", "ARRIVED"}
                and self._robot_departure_pending(robot)
            ):
                # A commanded chain sink must receive its own departure plan,
                # never be mistaken for warehouse storage and relocated by a
                # different order's recovery.
                continue
            if self._nearest_lm_for_robot(robot) in mentioned_lms:
                blockers.append(robot.name)
        return sorted(blockers)

    def _release_blocker_names_for_requests(self, requests: list[dict[str, Any]]) -> set[str]:
        request_names = {
            str(request.get("name", "")).strip()
            for request in requests
            if str(request.get("name", "")).strip()
        }
        if not request_names:
            return set()

        # Dependencies point from an upstream waiter to the robot whose
        # departure will release it.  The old one-hop lookup correctly held
        # A for a request that moved B, but kept A's upstream waiters' *future*
        # suffixes reserved.  In A->B->C->terminal those stale reservations
        # can occupy the terminal's only exit even though every upstream body
        # is physically stopped.  Walk the complete wait-for closure: keep
        # each current body as a full-horizon vertex reservation (the caller
        # does that via ``_held_blocker_vertex_intervals``), while omitting all
        # of their unexecutable future edge/vertex reservations.
        release_owners: set[str] = set()
        dependency_frontier = set(request_names)

        # Rolling SIPP can identify an external future reservation before the
        # runtime wait graph has observed the reciprocal hold. If that owner
        # is already WAITING, its advertised suffix is not executable: using
        # it again can reserve the requester's current LM and create the
        # artificial cycle "A cannot leave because B plans to arrive at A;
        # B cannot move because it is yielding to A". Preserve B's physical
        # body at its current LM, but omit that stale future on the next
        # attempt. Moving owners keep their timelines unchanged and remain
        # ordinary temporal reservations.
        for requester_name in sorted(request_names):
            for blocker_name in self._valid_rolling_prefetch_blockers(
                requester_name
            ):
                blocker = self.robots.get(blocker_name)
                if (
                    blocker is None
                    or blocker.status != "WAITING"
                    or not blocker.trajectory
                ):
                    continue
                release_owners.add(blocker.name)
                dependency_frontier.add(blocker.name)

        changed = True
        while changed:
            changed = False
            for robot in self._runtime_robots():
                if (
                    robot.name in request_names
                    or robot.name in release_owners
                    or robot.status != "WAITING"
                ):
                    continue
                dependencies: set[str] = set()
                if robot.wait_for_robot:
                    dependencies.add(str(robot.wait_for_robot))

                replan_state = self._runtime_replans.get(robot.name)
                if isinstance(replan_state, dict):
                    order = self.orders.get(
                        str(replan_state.get("order_id") or "")
                    )
                    valid_transaction = bool(
                        order is not None
                        and robot.active_order_id == order.order_id
                        and int(replan_state.get("route_revision", -1))
                        == int(robot.route_revision)
                        and abs(
                            float(replan_state.get("route_clock", 0.0) or 0.0)
                            - float(robot.route_clock)
                        ) <= 0.000001
                        and str(replan_state.get("start_lm") or "")
                        == self._safe_replan_start_lm(robot)
                    )
                    if valid_transaction:
                        dependencies.update(
                            str(name)
                            for name in replan_state.get("blocker_names", ())
                            if str(name)
                        )

                reason = str(robot.last_reason or "")
                reason_blocker = self._robot_name_from_conflict_reason(reason)
                if reason_blocker and (
                    self._is_robot_conflict(reason)
                    or reason.startswith("keep clearance from ")
                ):
                    dependencies.add(reason_blocker)

                if not dependencies.intersection(dependency_frontier):
                    continue
                release_owners.add(robot.name)
                dependency_frontier.add(robot.name)
                changed = True
        return release_owners

    def _nearest_lm_for_robot(self, robot: FleetRobot) -> str:
        pose = robot.pose
        if not pose:
            return robot.current_lm if robot.current_lm in self.landmarks else ""
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(
                landmark.x - float(pose.get("x", 0.0) or 0.0),
                landmark.y - float(pose.get("y", 0.0) or 0.0),
            ),
        )
        return nearest.name

    def _traffic_lm_for_robot(self, robot: FleetRobot) -> str:
        """Return the graph-authoritative LM without an O(|V|) pose scan.

        Runtime updates ``current_lm`` only after a tagged trajectory sample,
        so while traversing an edge it deliberately remains the source LM.
        That is exactly the occupancy side required by admission control.
        """
        if robot.current_lm in self.landmarks:
            return robot.current_lm
        return self._nearest_lm_for_robot(robot)
__all__ = ["TrafficReservationMixin"]
