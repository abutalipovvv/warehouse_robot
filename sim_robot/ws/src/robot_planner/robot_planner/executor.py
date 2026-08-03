from __future__ import annotations

from dataclasses import dataclass
import math
from time import time
from typing import Any, Callable

from .route_planner import RobotTrajectoryPlanner, clamp, normalize_angle
from .runtime import PlannedRobotRoute, Pose2D, RobotRuntime, RoutePoint


@dataclass(frozen=True, slots=True)
class RouteControlParameters:
    """Normalized parameters for one pure-pursuit control step."""

    route_speed: float
    lookahead: float
    stop_distance: float
    angular_gain: float
    max_angular: float
    rotate_in_place_angle: float
    curve_speed_limit: float
    rejoin_speed_limit: float
    hard_rejoin_speed_limit: float
    on_route_tolerance: float
    yaw_tolerance: float

    @classmethod
    def from_payload(cls, params: dict[str, Any]) -> RouteControlParameters:
        navigation = params.get("navigation", {})
        planner = params.get("planner", {})
        localization = params.get("localization", {})
        if not isinstance(navigation, dict):
            navigation = {}
        if not isinstance(planner, dict):
            planner = {}
        if not isinstance(localization, dict):
            localization = {}
        return cls(
            route_speed=max(
                0.05,
                float(navigation.get("route_speed", 0.35) or 0.35),
            ),
            lookahead=max(
                0.10,
                float(navigation.get("footprint_lookahead", 0.8) or 0.8),
            ),
            stop_distance=max(
                0.04,
                float(navigation.get("stop_distance", 0.1) or 0.1),
            ),
            angular_gain=max(
                0.4,
                float(navigation.get("angular_gain", 2.2) or 2.2),
            ),
            max_angular=max(
                0.35,
                float(navigation.get("max_angular_speed", 0.9) or 0.9),
            ),
            rotate_in_place_angle=math.radians(
                max(
                    10.0,
                    float(
                        navigation.get("rotate_in_place_angle_deg", 32.0)
                        or 32.0
                    ),
                )
            ),
            curve_speed_limit=max(
                0.05,
                float(navigation.get("curve_speed_limit", 0.25) or 0.25),
            ),
            rejoin_speed_limit=max(
                0.05,
                float(navigation.get("rejoin_speed_limit", 0.16) or 0.16),
            ),
            hard_rejoin_speed_limit=max(
                0.04,
                float(
                    navigation.get("hard_rejoin_speed_limit", 0.06) or 0.06
                ),
            ),
            on_route_tolerance=max(
                0.05,
                float(planner.get("on_route_tolerance", 0.12) or 0.12),
            ),
            yaw_tolerance=math.radians(
                max(
                    0.5,
                    float(
                        localization.get("allowed_yaw_error_deg", 4.0)
                        or 4.0
                    ),
                )
            ),
        )


@dataclass(slots=True)
class RouteProgress:
    """Goal-relative progress calculated from a path projection."""

    distance_to_goal: float
    final_yaw_error: float
    remaining_distance: float


@dataclass(slots=True)
class RouteSteeringState:
    """Geometric errors used by the velocity policy."""

    curvature_hint: float
    drive_sign: float
    path_heading_error: float
    target_heading_error: float
    cross_track_error: float
    steering_error: float
    off_route: bool
    hard_rejoin: bool


class RouteExecutor:
    def __init__(
        self,
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        publish_cmd_vel: Callable[[float, float], None],
    ) -> None:
        self.runtime = runtime
        self.route_planner = route_planner
        self._publish_cmd_vel = publish_cmd_vel
        self._control_params_payload: dict[str, Any] | None = None
        self._control_params: RouteControlParameters | None = None

    def control_step(self, status: dict[str, Any]) -> None:
        self.route_planner.reload_params_from_disk()
        route = self.runtime.active_route()
        if route is None:
            if status.get("localizationOk", False) and status.get("state") in {"ARRIVED", "EXECUTING_ROUTE"}:
                self.runtime.set_state("IDLE", "Ready.")
            return

        if self.runtime.route_paused():
            self.runtime.set_route_paused(True)
            self._publish_cmd_vel(0.0, 0.0)
            return

        pose_payload = status.get("pose")
        if not isinstance(pose_payload, dict):
            self.runtime.finish_route(False, "Robot pose is not available.")
            self.runtime.add_event(
                "error",
                "Route execution error: robot pose is not available. Planning or tracking cannot continue.",
            )
            self._publish_cmd_vel(0.0, 0.0)
            return

        pose = Pose2D(
            x=float(pose_payload.get("x", 0.0) or 0.0),
            y=float(pose_payload.get("y", 0.0) or 0.0),
            yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
        )
        self._follow_route(route, pose)

    def _follow_route(self, route: PlannedRobotRoute, pose: Pose2D) -> None:
        control = self._route_control_parameters()

        if not route.trajectory:
            self.runtime.finish_route(False, "Route is empty.")
            self.runtime.add_event(
                "error",
                "Route execution error: the planned route is empty.",
            )
            self._publish_cmd_vel(0.0, 0.0)
            return

        points = route.trajectory
        distances = self._path_distances(points)
        projection = self._project_pose_to_path(points, distances, pose, route.current_index)
        current_index = int(projection["index"])
        total_length = max(1e-6, distances[-1])
        progress = float(projection["s"]) / total_length
        self.runtime.update_route_progress(current_index, str(projection["edge_id"]), progress)

        if self._wait_for_route_gate(points, current_index):
            return

        route_progress = self._route_goal_progress(
            points,
            pose,
            projection,
            total_length,
        )
        if (
            route_progress.remaining_distance <= control.stop_distance
            and route_progress.distance_to_goal <= control.stop_distance
            and abs(route_progress.final_yaw_error) <= control.yaw_tolerance
        ):
            self._publish_cmd_vel(0.0, 0.0)
            self.runtime.finish_route(True, f"Arrived at {route.goal_lm}.")
            self.runtime.add_event("info", f"arrived at {route.goal_lm}")
            return

        steering = self._route_steering_state(
            points,
            distances,
            pose,
            projection,
            current_index,
            total_length,
            control,
        )
        linear, angular, message = self._route_drive_command(
            route,
            route_progress,
            steering,
            control,
        )
        self.runtime.set_state("EXECUTING_ROUTE", message)
        if steering.off_route:
            linear = steering.drive_sign * min(
                abs(linear),
                control.rejoin_speed_limit,
            )
        if steering.hard_rejoin:
            linear = steering.drive_sign * min(
                abs(linear),
                control.hard_rejoin_speed_limit,
            )
        self._publish_cmd_vel(
            linear,
            self.route_planner.map_angular_to_ros(angular),
        )

    def _route_control_parameters(self) -> RouteControlParameters:
        """Normalize once; planner hot reload replaces the params mapping."""
        payload = self.route_planner.current_params()
        if payload is not self._control_params_payload:
            self._control_params_payload = payload
            self._control_params = RouteControlParameters.from_payload(payload)
        if self._control_params is None:
            self._control_params = RouteControlParameters.from_payload(payload)
        return self._control_params

    def _wait_for_route_gate(
        self,
        points: list[RoutePoint],
        current_index: int,
    ) -> bool:
        """Stop before entering a segment whose reservation starts later."""
        gate_index = min(len(points) - 1, current_index + 1)
        gate_time = max(
            points[current_index].not_before,
            points[gate_index].not_before,
        )
        if not gate_time > time():
            return False
        self.runtime.set_state(
            "WAITING_TRAFFIC",
            f"Waiting for reserved route window ({gate_time - time():.2f}s).",
        )
        self._publish_cmd_vel(0.0, 0.0)
        return True

    @staticmethod
    def _route_goal_progress(
        points: list[RoutePoint],
        pose: Pose2D,
        projection: dict[str, Any],
        total_length: float,
    ) -> RouteProgress:
        final_point = points[-1]
        distance_to_goal = math.hypot(
            final_point.x - pose.x,
            final_point.y - pose.y,
        )
        desired_final_yaw = normalize_angle(final_point.yaw)
        return RouteProgress(
            distance_to_goal=distance_to_goal,
            final_yaw_error=normalize_angle(desired_final_yaw - pose.yaw),
            remaining_distance=max(
                0.0,
                total_length - float(projection["s"]),
            ),
        )

    def _route_steering_state(
        self,
        points: list[RoutePoint],
        distances: list[float],
        pose: Pose2D,
        projection: dict[str, Any],
        current_index: int,
        total_length: float,
        control: RouteControlParameters,
    ) -> RouteSteeringState:
        preview_target = self._interpolate_path_point(
            points,
            distances,
            min(total_length, float(projection["s"]) + control.lookahead),
            current_index,
        )
        curvature_hint = abs(
            normalize_angle(preview_target.yaw - float(projection["yaw"]))
        )
        effective_lookahead = clamp(
            control.lookahead
            * (
                1.0
                - (0.35 * min(1.0, curvature_hint / 1.1))
                - (
                    0.30
                    * min(
                        1.0,
                        abs(float(projection["cross_track"]))
                        / max(0.2, control.on_route_tolerance * 2.0),
                    )
                )
            ),
            0.12,
            control.lookahead,
        )
        target = self._interpolate_path_point(
            points,
            distances,
            min(total_length, float(projection["s"]) + effective_lookahead),
            current_index,
        )
        drive_sign = -1.0 if target.motion_direction == "backward" else 1.0
        path_heading_error = normalize_angle(target.yaw - pose.yaw)
        target_bearing = math.atan2(target.y - pose.y, target.x - pose.x)
        target_heading_error = (
            path_heading_error
            if drive_sign < 0.0
            else normalize_angle(target_bearing - pose.yaw)
        )
        cross_track_error = float(projection["cross_track"])
        cross_track_term = math.atan2(
            1.9 * cross_track_error,
            max(control.route_speed, 0.10),
        )
        steering_error = (
            path_heading_error - cross_track_term
            if drive_sign < 0.0
            else (
                (0.45 * path_heading_error)
                + (0.55 * target_heading_error)
                - cross_track_term
            )
        )
        return RouteSteeringState(
            curvature_hint=curvature_hint,
            drive_sign=drive_sign,
            path_heading_error=path_heading_error,
            target_heading_error=target_heading_error,
            cross_track_error=cross_track_error,
            steering_error=steering_error,
            off_route=(
                abs(cross_track_error) > control.on_route_tolerance
            ),
            hard_rejoin=(
                abs(cross_track_error)
                > max(0.32, control.on_route_tolerance * 2.5)
            ),
        )

    @staticmethod
    def _route_drive_command(
        route: PlannedRobotRoute,
        progress: RouteProgress,
        steering: RouteSteeringState,
        control: RouteControlParameters,
    ) -> tuple[float, float, str]:
        """Apply geometric penalties and return map-frame velocity."""
        linear = steering.drive_sign * control.route_speed
        heading_penalty = min(abs(steering.steering_error) / 1.25, 0.90)
        lateral_penalty = min(
            abs(steering.cross_track_error)
            / max(0.32, control.on_route_tolerance * 2.5),
            0.85,
        )
        curvature_penalty = min(steering.curvature_hint / 1.10, 0.70)
        linear_scale = max(
            0.12,
            1.0
            - (0.45 * heading_penalty)
            - (0.35 * lateral_penalty)
            - (0.25 * curvature_penalty),
        )
        linear *= linear_scale

        if steering.off_route:
            linear = steering.drive_sign * min(
                abs(linear),
                max(0.08, control.route_speed * 0.55),
            )
        if steering.hard_rejoin and abs(steering.target_heading_error) > 0.45:
            linear = steering.drive_sign * min(abs(linear), 0.06)
        if abs(steering.steering_error) > 1.25:
            linear = steering.drive_sign * min(abs(linear), 0.05)
        if progress.distance_to_goal < 0.50:
            linear = steering.drive_sign * min(
                abs(linear),
                max(0.08, progress.distance_to_goal),
            )
        if progress.distance_to_goal < control.stop_distance:
            linear = steering.drive_sign * min(abs(linear), 0.05)
        steering_error = steering.steering_error
        if (
            progress.distance_to_goal <= control.stop_distance
            and abs(progress.final_yaw_error) > control.yaw_tolerance
        ):
            linear = 0.0
            steering_error = progress.final_yaw_error
        elif (
            steering.hard_rejoin
            and abs(steering.target_heading_error)
            > control.rotate_in_place_angle
        ):
            linear = 0.0
            steering_error = (
                steering.path_heading_error
                if steering.drive_sign < 0.0
                else steering.target_heading_error
            )
        elif (
            steering.off_route
            and abs(steering_error) > (control.rotate_in_place_angle * 1.15)
        ):
            linear = 0.0

        angular = clamp(
            control.angular_gain * steering_error,
            -control.max_angular,
            control.max_angular,
        )
        if steering.hard_rejoin:
            message = f"Returning to route toward {route.goal_lm}."
        elif steering.off_route:
            message = f"Rejoining route to {route.goal_lm}."
        else:
            message = f"Driving to {route.goal_lm}."
        if steering.curvature_hint > 0.38:
            linear = steering.drive_sign * min(
                abs(linear),
                control.curve_speed_limit,
            )
        return linear, angular, message

    def _path_distances(self, points: list[RoutePoint]) -> list[float]:
        distances = [0.0]
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            distances.append(distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y))
        return distances

    def _project_pose_to_path(
        self,
        points: list[RoutePoint],
        distances: list[float],
        pose: Pose2D,
        hint_index: int,
    ) -> dict[str, Any]:
        if len(points) == 1:
            point = points[0]
            return {
                "index": 0,
                "s": 0.0,
                "x": point.x,
                "y": point.y,
                "yaw": point.yaw,
                "cross_track": math.hypot(point.x - pose.x, point.y - pose.y),
                "edge_id": point.edge_id,
            }

        best = self._project_pose_to_path_range(
            points=points,
            distances=distances,
            pose=pose,
            start_index=max(0, hint_index - 4),
            stop_index=min(len(points) - 1, hint_index + 72),
        )
        if best is None or abs(float(best["cross_track"])) > 0.75:
            fallback = self._project_pose_to_path_range(
                points=points,
                distances=distances,
                pose=pose,
                start_index=0,
                stop_index=len(points) - 1,
            )
            if fallback is not None:
                best = fallback

        return best or {
            "index": 0,
            "s": 0.0,
            "x": points[0].x,
            "y": points[0].y,
            "yaw": points[0].yaw,
            "cross_track": math.hypot(points[0].x - pose.x, points[0].y - pose.y),
            "edge_id": points[0].edge_id,
        }

    def _project_pose_to_path_range(
        self,
        points: list[RoutePoint],
        distances: list[float],
        pose: Pose2D,
        start_index: int,
        stop_index: int,
    ) -> dict[str, Any] | None:
        segment_count = max(0, len(points) - 1)
        if segment_count == 0:
            return None
        start = min(max(0, start_index), segment_count - 1)
        stop = min(max(start + 1, stop_index), segment_count)
        best: dict[str, Any] | None = None
        best_distance = float("inf")
        for index in range(start, stop):
            first = points[index]
            second = points[index + 1]
            dx = second.x - first.x
            dy = second.y - first.y
            seg_len_sq = (dx * dx) + (dy * dy)
            if seg_len_sq <= 1e-9:
                ratio = 0.0
                proj_x = first.x
                proj_y = first.y
            else:
                ratio = clamp((((pose.x - first.x) * dx) + ((pose.y - first.y) * dy)) / seg_len_sq, 0.0, 1.0)
                proj_x = first.x + (dx * ratio)
                proj_y = first.y + (dy * ratio)
            interpolated_yaw = normalize_angle(first.yaw + (normalize_angle(second.yaw - first.yaw) * ratio))
            cross_track = (-math.sin(interpolated_yaw) * (pose.x - proj_x)) + (math.cos(interpolated_yaw) * (pose.y - proj_y))
            distance = math.hypot(pose.x - proj_x, pose.y - proj_y)
            if distance < best_distance:
                best_distance = distance
                best = {
                    "index": index if ratio < 0.5 else min(index + 1, len(points) - 1),
                    "s": distances[index] + (math.sqrt(seg_len_sq) * ratio),
                    "x": proj_x,
                    "y": proj_y,
                    "yaw": interpolated_yaw,
                    "cross_track": cross_track,
                    "edge_id": second.edge_id if ratio > 0.5 and second.edge_id else first.edge_id,
                }
        return best

    def _interpolate_path_point(
        self,
        points: list[RoutePoint],
        distances: list[float],
        target_s: float,
        hint_index: int = 0,
    ) -> RoutePoint:
        if target_s <= 0.0 or len(points) == 1:
            return points[0]
        if target_s >= distances[-1]:
            return points[-1]

        index = max(0, min(hint_index, len(points) - 2))
        if distances[index] > target_s:
            while index > 0 and distances[index] > target_s:
                index -= 1
        else:
            while index < len(points) - 2 and distances[index + 1] < target_s:
                index += 1

        first = points[index]
        second = points[index + 1]
        span = max(1e-6, distances[index + 1] - distances[index])
        ratio = clamp((target_s - distances[index]) / span, 0.0, 1.0)
        return RoutePoint(
            x=first.x + ((second.x - first.x) * ratio),
            y=first.y + ((second.y - first.y) * ratio),
            yaw=normalize_angle(first.yaw + (normalize_angle(second.yaw - first.yaw) * ratio)),
            edge_id=second.edge_id if ratio > 0.5 and second.edge_id else first.edge_id,
            motion_direction=second.motion_direction if ratio > 0.5 else first.motion_direction,
        )
