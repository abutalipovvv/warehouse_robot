"""Geometric validation and historical retreat target selection."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.mapping.maps.models import PlannedRoute


EscapePose = dict[str, float]
EscapeEdgeSamples = list[EscapePose]


@dataclass(frozen=True, slots=True)
class _EscapeOrientationState:
    """One deterministic dynamic-programming state for edge orientation."""

    turn_cost: float
    variant_indices: tuple[int, ...]
    final_yaw: float
    selected_edges: tuple[EscapeEdgeSamples, ...]


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
        rotate_enabled = bool(self.planner._rotate_enabled({}))
        edge_variants = self._graph_escape_edge_variants(
            nodes,
            sample_step=sample_step,
        )
        if edge_variants is None:
            return "invalid graph escape"
        selected_edges = self._select_graph_escape_orientations(
            edge_variants,
            initial_yaw=float(current_pose.get("yaw", 0.0) or 0.0),
            rotate_enabled=rotate_enabled,
        )
        motion_samples = self._graph_escape_motion_samples(
            current_pose,
            selected_edges,
            rotate_enabled=rotate_enabled,
        )
        return self._graph_escape_motion_blocker(
            robot,
            current_pose,
            motion_samples,
            only_robot_names=only_robot_names,
        )

    def _graph_escape_edge_variants(
        self,
        nodes: list[str],
        *,
        sample_step: float,
    ) -> list[list[EscapeEdgeSamples]] | None:
        """Sample every route edge in each legal body orientation."""
        edge_variants: list[list[EscapeEdgeSamples]] = []
        for src, dst in zip(nodes, nodes[1:]):
            edge = self.planner.route_planner.get_edge(src, dst)
            if edge is None:
                return None
            raw_samples = self.planner.route_planner.sample_route(
                PlannedRoute(
                    nodes=[src, dst],
                    edges=[edge],
                    length=float(edge.length),
                ),
                sample_distance=sample_step,
            )
            if len(raw_samples) < 2:
                return None
            base = [
                {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": self._normalize_escape_yaw(
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
                        "yaw": self._normalize_escape_yaw(
                            sample["yaw"] + math.pi
                        ),
                    }
                    for sample in base
                ])
            edge_variants.append(variants)
        return edge_variants

    def _select_graph_escape_orientations(
        self,
        edge_variants: list[list[EscapeEdgeSamples]],
        *,
        initial_yaw: float,
        rotate_enabled: bool,
    ) -> tuple[EscapeEdgeSamples, ...]:
        """Choose the minimum-turn orientation sequence with stable ties."""
        states = [
            _EscapeOrientationState(
                turn_cost=0.0,
                variant_indices=(),
                final_yaw=initial_yaw,
                selected_edges=(),
            )
        ]
        for variants in edge_variants:
            next_states: list[_EscapeOrientationState] = []
            for variant_index, samples in enumerate(variants):
                candidates = [
                    _EscapeOrientationState(
                        turn_cost=(
                            state.turn_cost
                            + (
                                self._escape_turn_distance(
                                    state.final_yaw,
                                    samples[0]["yaw"],
                                )
                                if rotate_enabled
                                else 0.0
                            )
                        ),
                        variant_indices=(
                            *state.variant_indices,
                            variant_index,
                        ),
                        final_yaw=samples[-1]["yaw"],
                        selected_edges=(*state.selected_edges, samples),
                    )
                    for state in states
                ]
                next_states.append(min(
                    candidates,
                    key=lambda state: (
                        state.turn_cost,
                        state.variant_indices,
                    ),
                ))
            states = next_states
        selected = min(
            states,
            key=lambda state: (
                state.turn_cost,
                state.variant_indices,
            ),
        )
        return selected.selected_edges

    def _graph_escape_motion_samples(
        self,
        current_pose: dict[str, Any],
        selected_edges: tuple[EscapeEdgeSamples, ...],
        *,
        rotate_enabled: bool,
    ) -> list[tuple[EscapePose, bool]]:
        """Insert the same bounded in-place turns used by runtime motion."""
        motion_samples: list[tuple[EscapePose, bool]] = []
        anchor = {
            "x": float(current_pose.get("x", 0.0) or 0.0),
            "y": float(current_pose.get("y", 0.0) or 0.0),
            "yaw": float(current_pose.get("yaw", 0.0) or 0.0),
        }
        rotation_step = math.radians(2.0)
        for edge_samples in selected_edges:
            target_yaw = edge_samples[0]["yaw"]
            yaw_delta = self._normalize_escape_yaw(
                target_yaw - anchor["yaw"]
            )
            if rotate_enabled and abs(yaw_delta) >= rotation_step:
                steps = max(
                    1,
                    int(math.ceil(abs(yaw_delta) / rotation_step)),
                )
                for index in range(1, steps + 1):
                    motion_samples.append((
                        {
                            "x": anchor["x"],
                            "y": anchor["y"],
                            "yaw": self._normalize_escape_yaw(
                                anchor["yaw"]
                                + (yaw_delta * index / steps)
                            ),
                        },
                        True,
                    ))
            # Runtime starts at the measured pose and consumes samples after
            # the first LM anchor without teleporting the body centre.
            motion_samples.extend(
                (sample, False)
                for sample in edge_samples[1:]
            )
            anchor = dict(edge_samples[-1])
        return motion_samples

    def _graph_escape_motion_blocker(
        self,
        robot: FleetRobot,
        current_pose: dict[str, Any],
        motion_samples: list[tuple[EscapePose, bool]],
        *,
        only_robot_names: set[str] | None,
    ) -> str:
        """Audit generated motion against current physical robot bodies."""
        for checks, (candidate_pose, is_rotation) in enumerate(
            motion_samples,
            start=1,
        ):
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

    @staticmethod
    def _normalize_escape_yaw(value: float) -> float:
        return (float(value) + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def _escape_turn_distance(cls, first: float, second: float) -> float:
        return abs(cls._normalize_escape_yaw(float(second) - float(first)))

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

        target_pose = self._pose_at_trajectory(trajectory, target_clock)
        path_poses = [current_pose]
        if target_pose is not None:
            path_poses.append(target_pose)
        lower_clock = min(start_clock, target_clock)
        upper_clock = max(start_clock, target_clock)
        path_poses.extend(
            sample
            for sample in trajectory
            if (
                lower_clock - 0.000001
                <= float(sample.get("t", 0.0) or 0.0)
                <= upper_clock + 0.000001
            )
        )
        margin = max(
            self.collision.robot_broadphase_distance(),
            float(self.planner.rotation_min_robot_center_distance_m),
        ) + 0.05
        min_x = min(float(pose.get("x", 0.0) or 0.0) for pose in path_poses)
        max_x = max(float(pose.get("x", 0.0) or 0.0) for pose in path_poses)
        min_y = min(float(pose.get("y", 0.0) or 0.0) for pose in path_poses)
        max_y = max(float(pose.get("y", 0.0) or 0.0) for pose in path_poses)
        other_candidates = [
            other
            for other in self._runtime_robots()
            if (
                other.name != robot.name
                and other.pose is not None
                and min_x - margin
                <= float(other.pose.get("x", 0.0) or 0.0)
                <= max_x + margin
                and min_y - margin
                <= float(other.pose.get("y", 0.0) or 0.0)
                <= max_y + margin
            )
        ]
        if not other_candidates:
            return ""

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
            for other in other_candidates:
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
