"""Geometric validation and historical retreat target selection."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.route_core.models import PlannedRoute


class EvacuationGeometryMixin:
    """Validate graph escapes and select safe historical retreat geometry."""

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
        physically_at_candidate = bool(
            robot.pose is not None
            and candidate[1] in self.landmarks
            and self._pose_is_at_lm(robot.pose, candidate[1])
        )
        if (
            (
                (
                    robot.route_clock >= final_time - 0.000001
                    and final_lm == candidate[1]
                )
                or physically_at_candidate
            )
            and previous_distinct is not None
        ):
            # At an exhausted chunk—or during a planned wait after arriving
            # at a graph LM—the last tagged LM is the robot's current physical
            # position. Retreating to that same clock is a no-op and only
            # queues another identical replan. Use the most recent distinct
            # LM instead; repeated wait/rotation samples at either endpoint do
            # not change which graph segment must be reversed.
            if self._controlled_corridor_graph is not None:
                safe_previous = self._previous_safe_trajectory_lm(
                    robot,
                    exclude_lm=candidate[1],
                )
                if (
                    safe_previous is not None
                    and safe_previous[1] != candidate[1]
                ):
                    return safe_previous
                # Never choose an internal/no-wait LM as a deliberate stop.
                # The caller will request an outward graph escape instead.
                return candidate
            return previous_distinct
        return candidate

    def _previous_safe_trajectory_lm(
        self,
        robot: FleetRobot,
        *,
        exclude_lm: str = "",
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
            if exclude_lm and lm_name == exclude_lm:
                continue
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
        if edge is not None and edge[0] == edge[1]:
            edge = None
        if edge is None:
            for sample in robot.trajectory:
                if float(sample.get("t", 0.0) or 0.0) + 0.000001 < robot.route_clock:
                    continue
                candidate = self._parse_edge_id(
                    str(sample.get("edgeId") or "")
                )
                if candidate is not None and candidate[0] != candidate[1]:
                    edge = candidate
                    break
        if edge is None:
            return []
        src, dst = edge
        return [(src, dst), (dst, src)]


__all__ = ['EvacuationGeometryMixin']
