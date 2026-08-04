"""Motion mathematics and trajectory construction for fleet plans."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from fleet_manager.core.mapping.maps.models import GraphEdge, PlannedRoute


class FleetMotionModel:
    """Shared timing and heading mathematics for CBS, SIPP and trajectories."""

    def __init__(self, planner: Any) -> None:
        self.planner = planner

    def edge_tick_cost(
        self,
        src: str,
        dst: str,
        speed: float,
        acceleration: float | None = None,
    ) -> int:
        # Scheduling landmarks are not physical stop points. Consecutive MOVE
        # edges therefore run at their speed limit; WAIT and ROTATE own ticks.
        del acceleration
        owner = self.planner
        if src == dst:
            return 1
        edge = owner.edge_by_key.get((src, dst))
        if edge is None:
            start = owner.landmarks.get(src)
            goal = owner.landmarks.get(dst)
            if start is None or goal is None:
                return 1
            length = math.hypot(goal.x - start.x, goal.y - start.y)
            edge_speed = speed
        else:
            length = max(float(edge.length), 0.0)
            edge_speed = self.edge_speed(edge, speed)
        travel_time = length / max(0.02, edge_speed)
        return max(
            1,
            math.ceil(
                travel_time / max(owner.time_step_sec, 0.001)
            ),
        )

    @staticmethod
    def edge_speed(edge: GraphEdge, default_speed: float) -> float:
        properties = (
            edge.properties
            if isinstance(edge.properties, dict)
            else {}
        )
        raw_speed = None
        for key in (
            "max_speed",
            "maxSpeed",
            "maxspeed",
            "speed",
            "speedLimit",
        ):
            if key in properties:
                raw_speed = properties.get(key)
                break
        if raw_speed is None:
            return max(0.02, default_speed)
        try:
            return max(0.02, min(default_speed, float(raw_speed)))
        except (TypeError, ValueError):
            return max(0.02, default_speed)

    @staticmethod
    def travel_time(
        distance: float,
        speed: float,
        acceleration: float | None = None,
    ) -> float:
        distance = max(0.0, float(distance or 0.0))
        speed = max(0.02, float(speed or 0.02))
        acceleration = max(0.0, float(acceleration or 0.0))
        if distance <= 0.0:
            return 0.0
        if acceleration <= 0.0:
            return distance / speed
        ramp_distance = (speed * speed) / acceleration
        if distance <= ramp_distance:
            return 2.0 * math.sqrt(distance / acceleration)
        return (
            (2.0 * speed / acceleration)
            + ((distance - ramp_distance) / speed)
        )

    def heuristic_ticks(self, start_lm: str, goal_lm: str) -> float:
        owner = self.planner
        key = (start_lm, goal_lm)
        if key in owner._heuristic_cache:
            return owner._heuristic_cache[key]
        try:
            route = owner.route_planner.find_route(start_lm, goal_lm)
        except ValueError:
            return 0.0
        if route.length <= 0:
            return 0.0
        speed = owner._route_speed({})
        acceleration = owner._route_acceleration({})
        value = max(
            1.0,
            owner._travel_time(
                route.length,
                speed,
                acceleration,
            )
            / max(owner.time_step_sec, 1e-6),
        )
        owner._bounded_cache_store(
            owner._heuristic_cache,
            key,
            value,
            owner.heuristic_cache_max_entries,
        )
        return value

    def rotation_duration(
        self,
        from_yaw: float,
        to_yaw: float,
        turn_speed: float,
    ) -> float:
        delta = abs(
            self.normalize_angle(
                float(to_yaw or 0.0) - float(from_yaw or 0.0)
            )
        )
        if delta < math.radians(2.0):
            return 0.0
        return delta / max(0.05, float(turn_speed or 0.05))

    def rotation_tick_cost(
        self,
        from_yaw: float,
        to_yaw: float,
        turn_speed: float,
    ) -> int:
        duration = self.rotation_duration(
            from_yaw,
            to_yaw,
            turn_speed,
        )
        if duration <= 0.0:
            return 0
        return max(
            1,
            int(
                math.ceil(
                    duration
                    / max(self.planner.time_step_sec, 1e-6)
                )
            ),
        )

    def edge_heading(self, from_lm: str, to_lm: str) -> float:
        owner = self.planner
        start = owner.landmarks.get(from_lm)
        goal = owner.landmarks.get(to_lm)
        if start is None or goal is None:
            return 0.0
        delta_x = goal.x - start.x
        delta_y = goal.y - start.y
        edge = owner.edge_by_key.get((from_lm, to_lm))
        if (
            edge is not None
            and edge.geometry is not None
            and str(edge.geometry.geometry).lower() == "bezier"
            and len(edge.geometry.control_points) >= 2
        ):
            first = edge.geometry.control_points[0]
            second = edge.geometry.control_points[1]
            tangent_x = second.x - first.x
            tangent_y = second.y - first.y
            if math.hypot(tangent_x, tangent_y) > 1e-9:
                delta_x = tangent_x
                delta_y = tangent_y
        heading = math.atan2(delta_y, delta_x)
        if edge is not None and edge.motion_direction_code() == 1:
            heading += math.pi
        return self.normalize_angle(heading)

    def edge_heading_options(
        self,
        from_lm: str,
        to_lm: str,
    ) -> tuple[float, ...]:
        heading = self.edge_heading(from_lm, to_lm)
        edge = self.planner.edge_by_key.get((from_lm, to_lm))
        if edge is None or edge.motion_direction_code() != -1:
            return (heading,)
        return (
            heading,
            self.normalize_angle(heading + math.pi),
        )

    @staticmethod
    def normalize_angle(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


@dataclass(slots=True)
class _TrajectoryOptions:
    speed: float
    acceleration: float
    rotate_enabled: bool
    turn_speed: float
    stretch_motion: bool
    times: list[int] | None
    yaws: list[float] | None
    actions: list[str] | None
    has_kinematic_timing: bool


@dataclass(slots=True)
class _TrajectoryState:
    points: list[dict[str, float | str]]
    current_time: float
    last_yaw: float


@dataclass(slots=True)
class _SampledMovement:
    points: list[dict[str, float | str]]
    length: float
    yaw: float
    reverse_unspecified: bool
    continuous_duration: float


class TrajectoryBuilder:
    """Convert landmark/time plans into sampled continuous trajectories."""

    def __init__(self, planner: Any) -> None:
        self.planner = planner

    def build(
        self,
        nodes: list[str],
        speed: float,
        times: list[int] | None = None,
        *,
        acceleration: float = 0.0,
        rotate_enabled: bool = False,
        turn_speed: float = 0.9,
        stretch_motion_to_reservation_ticks: bool | None = None,
        start_yaw: float = 0.0,
        yaws: list[float] | None = None,
        actions: list[str] | None = None,
    ) -> list[dict[str, float | str]]:
        if not nodes:
            return []

        owner = self.planner
        first = owner.landmarks[nodes[0]]
        state = _TrajectoryState(
            points=[
                {
                    "t": 0.0,
                    "x": first.x,
                    "y": first.y,
                    "yaw": float(start_yaw),
                    "edgeId": f"{first.name}->{first.name}",
                    "lm": first.name,
                    "motionDirection": "not_specified",
                }
            ],
            current_time=0.0,
            last_yaw=float(start_yaw),
        )
        options = _TrajectoryOptions(
            speed=speed,
            acceleration=acceleration,
            rotate_enabled=rotate_enabled,
            turn_speed=turn_speed,
            stretch_motion=(
                owner.stretch_motion_to_reservation_ticks
                if stretch_motion_to_reservation_ticks is None
                else stretch_motion_to_reservation_ticks
            ),
            times=times,
            yaws=yaws,
            actions=actions,
            has_kinematic_timing=bool(
                actions
                and len(actions) == len(nodes)
                and yaws
                and len(yaws) == len(nodes)
            ),
        )
        for index in range(1, len(nodes)):
            self._append_step(state, options, nodes, index)

        state.points[-1]["lm"] = nodes[-1]
        return state.points

    def _append_step(
        self,
        state: _TrajectoryState,
        options: _TrajectoryOptions,
        nodes: list[str],
        index: int,
    ) -> None:
        from_lm = nodes[index - 1]
        to_lm = nodes[index]
        planned_duration = self.planned_segment_duration(
            options.times,
            index,
        )
        planned_yaw = (
            float(options.yaws[index])
            if options.yaws is not None
            and index < len(options.yaws)
            else state.last_yaw
        )
        if from_lm == to_lm:
            planned_action = (
                str(options.actions[index]).strip().lower()
                if options.actions is not None
                and index < len(options.actions)
                else ""
            )
            self._append_stationary(
                state,
                from_lm=from_lm,
                planned_duration=planned_duration,
                planned_yaw=planned_yaw,
                is_rotation=planned_action == "rotate",
            )
            return

        movement = self._sample_movement(
            options,
            from_lm=from_lm,
            to_lm=to_lm,
            planned_yaw=planned_yaw,
        )
        rotate_duration = self._append_pre_move_rotation(
            state,
            options,
            movement,
            from_lm=from_lm,
            to_lm=to_lm,
            planned_duration=planned_duration,
        )
        duration = self._movement_duration(
            options,
            movement,
            planned_duration=planned_duration,
            rotate_duration=rotate_duration,
        )
        self._append_movement_samples(
            state,
            movement,
            duration=duration,
        )
        state.current_time += duration
        state.points[-1]["lm"] = to_lm

    def _append_stationary(
        self,
        state: _TrajectoryState,
        *,
        from_lm: str,
        planned_duration: float | None,
        planned_yaw: float,
        is_rotation: bool,
    ) -> None:
        owner = self.planner
        state.current_time += max(
            0.05,
            planned_duration or owner.wait_time_sec,
        )
        landmark = owner.landmarks[from_lm]
        state.points.append(
            {
                "t": state.current_time,
                "x": landmark.x,
                "y": landmark.y,
                "yaw": (
                    planned_yaw
                    if is_rotation
                    else state.last_yaw
                ),
                "edgeId": (
                    f"WAIT@ROTATE:{from_lm}"
                    if is_rotation
                    else f"{from_lm}->{from_lm}"
                ),
                "lm": from_lm,
                "motionDirection": (
                    "rotate"
                    if is_rotation
                    else "not_specified"
                ),
            }
        )
        if is_rotation:
            state.last_yaw = planned_yaw

    def _sample_movement(
        self,
        options: _TrajectoryOptions,
        *,
        from_lm: str,
        to_lm: str,
        planned_yaw: float,
    ) -> _SampledMovement:
        owner = self.planner
        route = self.direct_route(from_lm, to_lm)
        samples = owner.route_planner.sample_route(route)
        points = self.annotate_sample_distances(samples)
        length = max(
            points[-1]["s"] if points else 0.0,
            1e-6,
        )
        yaw = float(
            points[1]["yaw"]
            if len(points) > 1
            else points[0]["yaw"]
        )
        edge = owner.edge_by_key.get((from_lm, to_lm))
        reverse_unspecified = bool(
            options.has_kinematic_timing
            and edge is not None
            and edge.motion_direction_code() == -1
            and abs(
                abs(
                    FleetMotionModel.normalize_angle(
                        planned_yaw - yaw
                    )
                )
                - math.pi
            )
            <= 0.000001
        )
        continuous_duration = (
            length / max(0.02, options.speed)
            if options.has_kinematic_timing
            else owner._motion_model.travel_time(
                length,
                options.speed,
                options.acceleration,
            )
        )
        return _SampledMovement(
            points=points,
            length=length,
            yaw=yaw,
            reverse_unspecified=reverse_unspecified,
            continuous_duration=continuous_duration,
        )

    def _append_pre_move_rotation(
        self,
        state: _TrajectoryState,
        options: _TrajectoryOptions,
        movement: _SampledMovement,
        *,
        from_lm: str,
        to_lm: str,
        planned_duration: float | None,
    ) -> float:
        if (
            not options.rotate_enabled
            or options.has_kinematic_timing
        ):
            return 0.0
        duration = self.planner._motion_model.rotation_duration(
            state.last_yaw,
            movement.yaw,
            options.turn_speed,
        )
        if options.stretch_motion and planned_duration is not None:
            reservation_slack = max(
                0.0,
                planned_duration - movement.continuous_duration,
            )
            if duration > reservation_slack + 0.000001:
                duration = 0.0
        if duration <= 0.001:
            return duration

        state.current_time += duration
        anchor = state.points[-1]
        state.points.append(
            {
                "t": state.current_time,
                "x": float(anchor.get("x", 0.0) or 0.0),
                "y": float(anchor.get("y", 0.0) or 0.0),
                "yaw": movement.yaw,
                "edgeId": f"WAIT@ROTATE:{from_lm}->{to_lm}",
                "lm": from_lm,
                "motionDirection": "rotate",
            }
        )
        state.last_yaw = movement.yaw
        return duration

    @staticmethod
    def _movement_duration(
        options: _TrajectoryOptions,
        movement: _SampledMovement,
        *,
        planned_duration: float | None,
        rotate_duration: float,
    ) -> float:
        if options.has_kinematic_timing:
            return max(
                movement.continuous_duration,
                planned_duration or 0.0,
                0.05,
            )
        if options.stretch_motion:
            return max(
                movement.continuous_duration,
                (planned_duration or 0.0) - rotate_duration,
                0.05,
            )
        return max(movement.continuous_duration, 0.05)

    @staticmethod
    def _append_movement_samples(
        state: _TrajectoryState,
        movement: _SampledMovement,
        *,
        duration: float,
    ) -> None:
        current_time = state.current_time
        segment_length = movement.length
        reverse_unspecified = movement.reverse_unspecified
        trajectory = state.points
        normalize_angle = FleetMotionModel.normalize_angle
        last_yaw = state.last_yaw
        for sample in movement.points[1:]:
            time_value = (
                current_time
                + (float(sample["s"]) / segment_length)
                * duration
            )
            last_yaw = normalize_angle(
                float(sample["yaw"])
                + (
                    math.pi
                    if reverse_unspecified
                    else 0.0
                )
            )
            trajectory.append(
                {
                    "t": time_value,
                    "x": float(sample["x"]),
                    "y": float(sample["y"]),
                    "yaw": last_yaw,
                    "edgeId": str(sample["edgeId"]),
                    "motionDirection": (
                        "backward"
                        if reverse_unspecified
                        else str(
                            sample.get(
                                "motionDirection",
                                "not_specified",
                            )
                        )
                    ),
                }
            )
        state.last_yaw = last_yaw

    def planned_segment_duration(
        self,
        times: list[int] | None,
        index: int,
    ) -> float | None:
        if not times or index <= 0 or index >= len(times):
            return None
        return max(
            0.0,
            (
                int(times[index])
                - int(times[index - 1])
            )
            * self.planner.time_step_sec,
        )

    def direct_route(self, from_lm: str, to_lm: str) -> PlannedRoute:
        edge = self.planner.edge_by_key.get((from_lm, to_lm))
        if edge is None:
            raise ValueError(
                "MAPF returned non-adjacent landmarks: "
                f"{from_lm}->{to_lm}"
            )
        return PlannedRoute(
            nodes=[from_lm, to_lm],
            edges=[edge],
            length=edge.length,
        )

    @staticmethod
    def annotate_sample_distances(
        samples: list[dict[str, float | str]],
    ) -> list[dict[str, float | str]]:
        distance = 0.0
        annotated: list[dict[str, float | str]] = []
        for index, sample in enumerate(samples):
            if index > 0:
                previous = samples[index - 1]
                distance += math.hypot(
                    float(sample["x"]) - float(previous["x"]),
                    float(sample["y"]) - float(previous["y"]),
                )
            annotated.append({**sample, "s": distance})
        return annotated
