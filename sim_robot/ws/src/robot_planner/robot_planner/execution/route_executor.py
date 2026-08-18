from __future__ import annotations

from dataclasses import dataclass
import math
from time import monotonic, time
from typing import Any, Callable, Protocol

from ..control import (
    ArrivalMonitor,
    ArrivalParameters,
    PidController,
    PidParameters,
    SpeedProfileParameters,
    SpeedProfiler,
)
from ..math import PathProjection, TrajectoryArray, TrajectoryMath
from ..planning import RobotTrajectoryPlanner
from ..runtime import PlannedRobotRoute, Pose2D, RobotRuntime, RoutePoint


@dataclass(frozen=True, slots=True)
class RouteControlParameters:
    """Normalized parameters for one trajectory tracking control step."""

    route_speed: float
    lookahead: float
    stop_distance: float
    angular_gain: float
    max_angular: float
    rotate_in_place_angle: float
    curve_speed_limit: float
    curve_heading_threshold: float
    curve_curvature_threshold: float
    curve_preview_distance: float
    curve_angular_reserve: float
    curve_min_speed: float
    rejoin_speed_limit: float
    hard_rejoin_speed_limit: float
    strict_edge_tracking: bool
    strict_speed_limit: float
    edge_transition_tolerance: float
    drive_alignment_angle: float
    lateral_error_gain: float
    rejoin_lateral_error_gain: float
    precision_lateral_tolerance: float
    precision_speed_limit: float
    on_route_tolerance: float
    yaw_tolerance: float
    odom_tracking_enabled: bool

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
                float(navigation.get("curve_speed_limit", 0.22) or 0.22),
            ),
            curve_heading_threshold=math.radians(
                max(
                    1.0,
                    float(
                        navigation.get("curve_heading_threshold_deg", 5.0)
                        or 5.0
                    ),
                )
            ),
            curve_curvature_threshold=max(
                0.001,
                float(
                    navigation.get("curve_curvature_threshold", 0.05)
                    or 0.05
                ),
            ),
            curve_preview_distance=max(
                0.03,
                float(
                    navigation.get("curve_preview_distance", 0.25)
                    or 0.25
                ),
            ),
            curve_angular_reserve=min(
                0.75,
                max(
                    0.10,
                    float(
                        navigation.get("curve_angular_reserve", 0.35)
                        or 0.35
                    ),
                ),
            ),
            curve_min_speed=max(
                0.03,
                float(navigation.get("curve_min_speed", 0.08) or 0.08),
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
            strict_edge_tracking=bool(
                navigation.get("strict_edge_tracking", True)
            ),
            strict_speed_limit=max(
                0.05,
                float(navigation.get("strict_speed_limit", 0.40) or 0.40),
            ),
            edge_transition_tolerance=max(
                0.01,
                float(
                    navigation.get("edge_transition_tolerance", 0.04)
                    or 0.04
                ),
            ),
            drive_alignment_angle=math.radians(
                max(
                    2.0,
                    float(
                        navigation.get("drive_alignment_angle_deg", 12.0)
                        or 12.0
                    ),
                )
            ),
            lateral_error_gain=max(
                0.1,
                float(
                    navigation.get("lateral_error_gain", 3.0)
                    or 3.0
                ),
            ),
            rejoin_lateral_error_gain=max(
                0.1,
                float(
                    navigation.get("rejoin_lateral_error_gain", 1.5)
                    or 1.5
                ),
            ),
            precision_lateral_tolerance=max(
                0.005,
                float(
                    navigation.get(
                        "precision_lateral_tolerance",
                        localization.get("allowed_lateral_error", 0.02),
                    )
                    or 0.02
                ),
            ),
            precision_speed_limit=max(
                0.05,
                float(
                    navigation.get("precision_speed_limit", 0.18)
                    or 0.18
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
            odom_tracking_enabled=bool(
                navigation.get("odom_tracking_enabled", True)
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
    lateral_control_error: float = 0.0
    heading_control_error: float = 0.0
    path_curvature: float = 0.0
    preview_curvature: float = 0.0
    tracking_speed_reference: float = 0.0


@dataclass(frozen=True, slots=True)
class RouteEdgeTransition:
    edge_end_index: int
    next_index: int
    boundary_distance: float
    next_yaw_error: float
    requires_stop: bool


class RouteLogger(Protocol):
    def debug(self, message: str) -> None: ...

    def info(self, message: str) -> None: ...

    def warning(self, message: str) -> None: ...


class RouteExecutor:
    def __init__(
        self,
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        publish_cmd_vel: Callable[[float, float], None],
        logger: RouteLogger | None = None,
    ) -> None:
        self.runtime = runtime
        self.route_planner = route_planner
        self._publish_cmd_vel = publish_cmd_vel
        self._logger = logger
        self._control_params_payload: dict[str, Any] | None = None
        self._control_params: RouteControlParameters | None = None
        self._speed_profile_params: SpeedProfileParameters | None = None
        self._arrival_params: ArrivalParameters | None = None
        self._pid = PidController(PidParameters.from_payload({}))
        self._speed_profiler = SpeedProfiler()
        self._arrival_monitor = ArrivalMonitor()
        self._trajectory_cache_key: tuple[Any, ...] | None = None
        self._trajectory_cache: TrajectoryArray | None = None
        self._odom_anchor_route_key: tuple[str, int] | None = None
        self._odom_anchor_map_pose: Pose2D | None = None
        self._odom_anchor_pose: Pose2D | None = None
        self._last_tracking_pose: Pose2D | None = None
        self._last_logged_route: tuple[str, int] | None = None
        self._last_logged_edge = ""
        self._was_off_route = False
        self._last_debug_log_at = 0.0
        self._measured_linear = 0.0
        self._measured_angular = 0.0

    def control_step(self, status: dict[str, Any]) -> None:
        self.route_planner.reload_params_from_disk()
        route = self.runtime.active_route()
        route_key = (
            (route.route_id, route.revision)
            if route is not None
            else ("", 0)
        )
        control = self._route_control_parameters()
        self._measured_linear, self._measured_angular = (
            self._velocity_from_payload(status.get("velocity"))
        )
        localized_pose = self._pose_from_payload(status.get("pose"))
        pose = (
            self._tracking_pose(
                route_key,
                localized_pose,
                status.get("odomPose"),
                enabled=control.odom_tracking_enabled,
            )
            if localized_pose is not None
            else None
        )
        if pose is not None:
            self._last_tracking_pose = pose

        if route is None:
            self._reset_tracking_state(clear_trajectory=True)
            self._last_logged_route = None
            self._last_logged_edge = ""
            self._was_off_route = False
            if status.get("localizationOk", False) and status.get("state") in {"ARRIVED", "EXECUTING_ROUTE"}:
                self.runtime.set_state("IDLE", "Ready.")
            return

        if self.runtime.route_paused():
            self._reset_tracking_state()
            self.runtime.set_route_paused(True)
            self._publish_cmd_vel(0.0, 0.0)
            return

        if pose is None:
            self._reset_tracking_state(clear_trajectory=True)
            self.runtime.finish_route(False, "Robot pose is not available.")
            self.runtime.add_event(
                "error",
                "Route execution error: robot pose is not available. Planning or tracking cannot continue.",
            )
            self._publish_cmd_vel(0.0, 0.0)
            return

        if route_key != self._last_logged_route:
            self._last_logged_route = route_key
            self._log_info(
                f"route started: id={route.route_id} rev={route.revision} "
                f"goal={route.goal_lm} nodes={len(route.nodes)} length={route.length:.2f}m "
                f"tracking={'odom' if self._odom_anchor_route_key == route_key else 'map'}"
            )
        self._follow_route(route, pose)

    def _follow_route(self, route: PlannedRobotRoute, pose: Pose2D) -> None:
        control = self._route_control_parameters()

        if not route.trajectory:
            self._reset_tracking_state(clear_trajectory=True)
            self.runtime.finish_route(False, "Route is empty.")
            self.runtime.add_event(
                "error",
                "Route execution error: the planned route is empty.",
            )
            self._log_warning("route rejected: trajectory is empty")
            self._publish_cmd_vel(0.0, 0.0)
            return

        points = route.trajectory
        trajectory = self._trajectory_array(route)
        projection, _, edge_end_index = self._project_pose_to_active_edge(
            trajectory,
            pose,
            route.current_index,
        )
        transition = self._edge_transition(
            trajectory,
            pose,
            edge_end_index,
            control,
        )
        transition_tolerance = self._edge_transition_tolerance(control)
        if (
            transition is not None
            and self._transition_boundary_reached(
                trajectory,
                projection,
                transition,
                transition_tolerance,
                strict_edge_tracking=control.strict_edge_tracking,
            )
        ):
            boundary_progress = float(trajectory.s[edge_end_index]) / max(
                1e-6,
                trajectory.length,
            )
            current_edge_id = str(trajectory.edge_ids[edge_end_index])
            self.runtime.update_route_progress(
                edge_end_index,
                current_edge_id,
                boundary_progress,
            )
            if self._wait_for_route_gate(points, edge_end_index):
                return
            alignment_tolerance = max(control.yaw_tolerance, math.radians(4.0))
            if (
                transition.requires_stop
                and abs(transition.next_yaw_error) > alignment_tolerance
            ):
                self._pid.reset()
                self._speed_profiler.reset()
                angular = TrajectoryMath.clamp(
                    control.angular_gain * transition.next_yaw_error,
                    -control.max_angular,
                    control.max_angular,
                )
                self.runtime.set_state(
                    "EXECUTING_ROUTE",
                    f"Aligning for the next route segment to {route.goal_lm}.",
                )
                self._publish_cmd_vel(
                    0.0,
                    self.route_planner.map_angular_to_ros(angular),
                )
                return
            projection, _, edge_end_index = self._project_pose_to_active_edge(
                trajectory,
                pose,
                transition.next_index,
            )
            transition = self._edge_transition(
                trajectory,
                pose,
                edge_end_index,
                control,
            )
        current_index = projection.index
        total_length = max(1e-6, trajectory.length)
        progress = projection.s / total_length
        self.runtime.update_route_progress(current_index, projection.edge_id, progress)

        if self._wait_for_route_gate(points, current_index):
            return

        route_progress = self._route_goal_progress(
            points,
            pose,
            projection,
            total_length,
        )
        arrival = self._arrival_parameters()
        arrived = self._arrival_monitor.update(
            remaining_distance=route_progress.remaining_distance,
            goal_position_error=route_progress.distance_to_goal,
            goal_yaw_error=route_progress.final_yaw_error,
            linear_velocity=self._measured_linear,
            angular_velocity=self._measured_angular,
            params=arrival,
        )
        goal_position_reached = (
            route_progress.remaining_distance <= arrival.position_tolerance
            and route_progress.distance_to_goal <= arrival.position_tolerance
        )
        goal_yaw_reached = (
            abs(route_progress.final_yaw_error) <= arrival.yaw_tolerance
        )
        if arrived:
            self._record_tracking_metrics(
                projection=projection,
                route_progress=route_progress,
                heading_error=route_progress.final_yaw_error,
                linear=0.0,
                angular=0.0,
            )
            self._reset_tracking_state(clear_trajectory=True)
            self._publish_cmd_vel(0.0, 0.0)
            self.runtime.finish_route(
                True,
                f"Arrived at {route.goal_lm} within "
                f"{arrival.position_tolerance:.3f} m.",
            )
            self.runtime.add_event("info", f"arrived at {route.goal_lm}")
            self._log_info(
                f"route arrived: id={route.route_id} goal={route.goal_lm} "
                f"position_error={route_progress.distance_to_goal:.4f}m "
                f"yaw_error={math.degrees(route_progress.final_yaw_error):.2f}deg"
            )
            return
        if goal_position_reached and goal_yaw_reached:
            self._pid.reset()
            self._speed_profiler.reset()
            self.runtime.set_state(
                "EXECUTING_ROUTE",
                f"Stabilizing at {route.goal_lm} "
                f"({self._arrival_monitor.stable_cycles}/{arrival.stable_cycles}).",
            )
            self._record_tracking_metrics(
                projection=projection,
                route_progress=route_progress,
                heading_error=route_progress.final_yaw_error,
                linear=0.0,
                angular=0.0,
            )
            self._publish_cmd_vel(0.0, 0.0)
            return

        steering = self._route_steering_state_array(
            trajectory,
            pose,
            projection,
            current_index,
            total_length,
            control,
            target_s_limit=(
                float(trajectory.s[edge_end_index])
                if control.strict_edge_tracking
                or (transition is not None and transition.requires_stop)
                else None
            ),
        )
        linear, angular, message = self._route_drive_command(
            route,
            route_progress,
            steering,
            control,
        )
        if transition is not None and transition.requires_stop:
            braking_distance = max(
                0.0,
                transition.boundary_distance - transition_tolerance,
            )
            transition_speed_limit = max(
                0.05,
                math.sqrt(2.0 * 0.50 * braking_distance),
            )
            linear = steering.drive_sign * min(
                abs(linear),
                transition_speed_limit,
            )
            message = f"Approaching a route turn toward {route.goal_lm}."
        linear = self._speed_profiler.step(
            linear,
            max(
                route_progress.distance_to_goal,
                route_progress.remaining_distance,
            ),
            self._speed_profile_parameters(),
        )
        if abs(linear) > 1e-6 and self._pid.params.enabled:
            angular = self._pid.step(
                (
                    steering.lateral_control_error,
                    steering.heading_control_error,
                ),
                # Curvature is parameterized by positive progress along the
                # graph edge. A backward chassis has yaw=tangent+pi, but its
                # yaw derivative keeps the same sign as the edge tangent.
                feedforward=abs(linear) * steering.path_curvature,
                max_output=control.max_angular,
            )
        else:
            self._pid.reset()
        self.runtime.set_state("EXECUTING_ROUTE", message)
        if projection.edge_id != self._last_logged_edge:
            self._last_logged_edge = projection.edge_id
            self._log_info(
                f"tracking edge: {projection.edge_id} "
                f"direction={'backward' if steering.drive_sign < 0.0 else 'forward'}"
            )
        if steering.off_route and not self._was_off_route:
            self._was_off_route = True
            self._log_warning(
                f"left edge corridor: edge={projection.edge_id} "
                f"cte={steering.cross_track_error:.3f}m"
            )
        elif (
            self._was_off_route
            and abs(steering.cross_track_error)
            <= control.on_route_tolerance * 0.8
        ):
            self._was_off_route = False
            self._log_info(
                f"edge corridor recovered: edge={projection.edge_id} "
                f"cte={steering.cross_track_error:.3f}m"
            )
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
        published_angular = self.route_planner.map_angular_to_ros(angular)
        self._record_tracking_metrics(
            projection=projection,
            route_progress=route_progress,
            heading_error=steering.heading_control_error,
            linear=linear,
            angular=published_angular,
        )
        self._publish_cmd_vel(
            linear,
            published_angular,
        )
        now = monotonic()
        if now - self._last_debug_log_at >= 2.0:
            self._last_debug_log_at = now
            terms = self._pid.last_terms
            self._log_debug(
                f"tracking: edge={projection.edge_id} progress={progress:.1%} "
                f"cte={steering.cross_track_error:.3f}m "
                f"heading={math.degrees(steering.heading_control_error):.1f}deg "
                f"curve={steering.path_curvature:.3f}/{steering.preview_curvature:.3f}m^-1 "
                f"goal={route_progress.distance_to_goal:.4f}m "
                f"v={linear:.3f}m/s w={angular:.3f}rad/s "
                f"pid=[p={terms.proportional:.3f}, i={terms.integral:.3f}, "
                f"d={terms.derivative:.3f}, ff={terms.feedforward:.3f}]"
            )

    def _route_control_parameters(self) -> RouteControlParameters:
        """Normalize once; planner hot reload replaces the params mapping."""
        payload = self.route_planner.current_params()
        if payload is not self._control_params_payload:
            self._control_params_payload = payload
            self._control_params = RouteControlParameters.from_payload(payload)
            self._speed_profile_params = SpeedProfileParameters.from_payload(
                payload
            )
            self._arrival_params = ArrivalParameters.from_payload(payload)
            self._pid.configure(PidParameters.from_payload(payload))
        if self._control_params is None:
            self._control_params = RouteControlParameters.from_payload(payload)
        return self._control_params

    def _speed_profile_parameters(self) -> SpeedProfileParameters:
        self._route_control_parameters()
        if self._speed_profile_params is None:
            self._speed_profile_params = SpeedProfileParameters.from_payload({})
        return self._speed_profile_params

    def _arrival_parameters(self) -> ArrivalParameters:
        self._route_control_parameters()
        if self._arrival_params is None:
            self._arrival_params = ArrivalParameters.from_payload({})
        return self._arrival_params

    def _trajectory_array(self, route: PlannedRobotRoute) -> TrajectoryArray:
        points = route.trajectory
        first = points[0]
        last = points[-1]
        cache_key = (
            route.route_id,
            route.revision,
            route.created_at,
            len(points),
            first.x,
            first.y,
            first.not_before,
            last.x,
            last.y,
            last.not_before,
        )
        if self._trajectory_cache is None or cache_key != self._trajectory_cache_key:
            self._trajectory_cache = TrajectoryArray.from_route_points(points)
            self._trajectory_cache_key = cache_key
            self._pid.reset()
            self._speed_profiler.reset()
            self._arrival_monitor.reset()
        return self._trajectory_cache

    def _reset_tracking_state(self, *, clear_trajectory: bool = False) -> None:
        self._pid.reset()
        self._speed_profiler.reset()
        self._arrival_monitor.reset()
        if clear_trajectory:
            self._trajectory_cache = None
            self._trajectory_cache_key = None

    def latest_tracking_pose(self) -> Pose2D | None:
        pose = self._last_tracking_pose
        if pose is None:
            return None
        return Pose2D(x=pose.x, y=pose.y, yaw=pose.yaw)

    def reset_pose_anchor(self) -> None:
        self._odom_anchor_route_key = None
        self._odom_anchor_map_pose = None
        self._odom_anchor_pose = None
        self._last_tracking_pose = None

    def reanchor_pose(self, map_pose: Pose2D, odom_payload: Any) -> bool:
        """Anchor smooth odometry to an explicitly supplied map pose."""

        self.reset_pose_anchor()
        odom_pose = self._pose_from_payload(odom_payload)
        if odom_pose is None:
            return False
        self._odom_anchor_map_pose = Pose2D(
            x=float(map_pose.x),
            y=float(map_pose.y),
            yaw=TrajectoryMath.normalize_angle(float(map_pose.yaw)),
        )
        self._odom_anchor_pose = odom_pose
        self._last_tracking_pose = self._odom_anchor_map_pose
        return True

    def _tracking_pose(
        self,
        route_key: tuple[str, int],
        localized_pose: Pose2D,
        odom_payload: Any,
        *,
        enabled: bool,
    ) -> Pose2D:
        """Use map localization as an anchor and odom for the local loop."""

        odom_pose = self._pose_from_payload(odom_payload)
        if not enabled or odom_pose is None:
            if not enabled:
                self._odom_anchor_route_key = None
                self._odom_anchor_map_pose = None
                self._odom_anchor_pose = None
            return localized_pose

        if self._odom_anchor_map_pose is None or self._odom_anchor_pose is None:
            self._odom_anchor_map_pose = localized_pose
            self._odom_anchor_pose = odom_pose
        self._odom_anchor_route_key = route_key

        x, y, yaw = TrajectoryMath.map_pose_from_odom_anchor(
            (
                self._odom_anchor_map_pose.x,
                self._odom_anchor_map_pose.y,
                self._odom_anchor_map_pose.yaw,
            ),
            (
                self._odom_anchor_pose.x,
                self._odom_anchor_pose.y,
                self._odom_anchor_pose.yaw,
            ),
            (odom_pose.x, odom_pose.y, odom_pose.yaw),
        )
        return Pose2D(x=x, y=y, yaw=yaw)

    @staticmethod
    def _pose_from_payload(payload: Any) -> Pose2D | None:
        if not isinstance(payload, dict):
            return None
        try:
            values = (
                float(payload.get("x")),
                float(payload.get("y")),
                float(payload.get("yaw")),
            )
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return Pose2D(x=values[0], y=values[1], yaw=values[2])

    @staticmethod
    def _velocity_from_payload(payload: Any) -> tuple[float, float]:
        if not isinstance(payload, dict):
            return 0.0, 0.0
        try:
            values = (
                float(payload.get("linear", 0.0) or 0.0),
                float(payload.get("angular", 0.0) or 0.0),
            )
        except (TypeError, ValueError):
            return 0.0, 0.0
        if not all(math.isfinite(value) for value in values):
            return 0.0, 0.0
        return values

    def _record_tracking_metrics(
        self,
        *,
        projection: PathProjection,
        route_progress: RouteProgress,
        heading_error: float,
        linear: float,
        angular: float,
    ) -> None:
        arrival = self._arrival_parameters()
        self.runtime.update_tracking_metrics(
            cross_track_error=projection.cross_track,
            heading_error=heading_error,
            remaining_distance=route_progress.remaining_distance,
            goal_position_error=route_progress.distance_to_goal,
            goal_yaw_error=route_progress.final_yaw_error,
            commanded_linear=linear,
            commanded_angular=angular,
            arrival_stable_cycles=self._arrival_monitor.stable_cycles,
            arrival_required_cycles=arrival.stable_cycles,
        )

    def _log_debug(self, message: str) -> None:
        if self._logger is not None:
            self._logger.debug(message)

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)

    def _log_warning(self, message: str) -> None:
        if self._logger is not None:
            self._logger.warning(message)

    @staticmethod
    def _project_pose_to_active_edge(
        trajectory: TrajectoryArray,
        pose: Pose2D,
        active_index: int,
    ) -> tuple[PathProjection, int, int]:
        """Project only onto the current graph edge to prevent route shortcuts."""

        index = max(0, min(trajectory.size - 1, int(active_index)))
        edge_start, edge_end = trajectory.edge_span(index)
        projection = trajectory.project_range(
            (pose.x, pose.y),
            edge_start,
            edge_end,
        )
        if projection is None:
            sample = trajectory.sample_at(float(trajectory.s[index]))
            offset_x = pose.x - sample.x
            offset_y = pose.y - sample.y
            cross_track = (
                (-math.sin(sample.yaw) * offset_x)
                + (math.cos(sample.yaw) * offset_y)
            )
            projection = PathProjection(
                index=index,
                s=sample.s,
                x=sample.x,
                y=sample.y,
                yaw=sample.yaw,
                cross_track=cross_track,
                distance=math.hypot(offset_x, offset_y),
                edge_id=str(trajectory.edge_ids[index]),
            )
        projection_index = max(index, min(edge_end, projection.index))
        return (
            PathProjection(
                index=projection_index,
                s=projection.s,
                x=projection.x,
                y=projection.y,
                yaw=projection.yaw,
                cross_track=projection.cross_track,
                distance=projection.distance,
                edge_id=str(trajectory.edge_ids[index]),
            ),
            edge_start,
            edge_end,
        )

    @staticmethod
    def _edge_transition(
        trajectory: TrajectoryArray,
        pose: Pose2D,
        edge_end_index: int,
        control: RouteControlParameters,
    ) -> RouteEdgeTransition | None:
        next_index = edge_end_index + 1
        if next_index >= trajectory.size:
            return None
        boundary = trajectory.xy[edge_end_index]
        heading_jump = abs(
            TrajectoryMath.normalize_angle(
                float(trajectory.yaw[next_index] - trajectory.yaw[edge_end_index])
            )
        )
        direction_changed = (
            trajectory.motion_directions[next_index]
            != trajectory.motion_directions[edge_end_index]
        )
        return RouteEdgeTransition(
            edge_end_index=edge_end_index,
            next_index=next_index,
            boundary_distance=math.hypot(
                pose.x - float(boundary[0]),
                pose.y - float(boundary[1]),
            ),
            next_yaw_error=TrajectoryMath.normalize_angle(
                float(trajectory.yaw[next_index]) - pose.yaw
            ),
            requires_stop=(
                direction_changed
                or heading_jump > control.rotate_in_place_angle
            ),
        )

    @staticmethod
    def _edge_transition_tolerance(
        control: RouteControlParameters,
    ) -> float:
        return min(
            control.stop_distance,
            control.edge_transition_tolerance,
        )

    @staticmethod
    def _transition_boundary_reached(
        trajectory: TrajectoryArray,
        projection: PathProjection,
        transition: RouteEdgeTransition,
        tolerance: float,
        *,
        strict_edge_tracking: bool = False,
    ) -> bool:
        """Decide when the active graph edge may advance.

        A discontinuous heading or motion-direction change must happen at the
        LM itself, because the chassis has to stop and align there. For a
        geometrically continuous edge, reaching the end in along-track
        progress is sufficient. Requiring the same tiny Euclidean tolerance
        for both cases deadlocked tracking after even a modest lateral error:
        the projection stayed on the old endpoint forever while the robot had
        already entered the next edge.
        """

        if transition.requires_stop:
            return transition.boundary_distance <= tolerance
        edge_end_s = float(trajectory.s[transition.edge_end_index])
        progress_tolerance = (
            min(tolerance, 0.002)
            if strict_edge_tracking
            else tolerance
        )
        return projection.s >= edge_end_s - progress_tolerance

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
        self._pid.reset()
        self._speed_profiler.reset()
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
        projection: PathProjection | dict[str, Any],
        total_length: float,
    ) -> RouteProgress:
        final_point = points[-1]
        distance_to_goal = math.hypot(
            final_point.x - pose.x,
            final_point.y - pose.y,
        )
        desired_final_yaw = TrajectoryMath.normalize_angle(final_point.yaw)
        projected_s = projection.s if isinstance(projection, PathProjection) else float(projection["s"])
        return RouteProgress(
            distance_to_goal=distance_to_goal,
            final_yaw_error=TrajectoryMath.normalize_angle(
                desired_final_yaw - pose.yaw
            ),
            remaining_distance=max(
                0.0,
                total_length - projected_s,
            ),
        )

    def _route_steering_state_array(
        self,
        trajectory: TrajectoryArray,
        pose: Pose2D,
        projection: PathProjection,
        current_index: int,
        total_length: float,
        control: RouteControlParameters,
        target_s_limit: float | None = None,
    ) -> RouteSteeringState:
        del current_index
        tracking_limit = total_length
        if target_s_limit is not None:
            tracking_limit = min(
                total_length,
                max(projection.s, float(target_s_limit)),
            )
        preview_target = trajectory.sample_at(
            min(total_length, projection.s + control.lookahead)
        )
        curvature_hint = abs(
            TrajectoryMath.normalize_angle(preview_target.yaw - projection.yaw)
        )
        local_reference = trajectory.sample_at(projection.s)
        drive_sign = (
            -1.0
            if local_reference.motion_direction == "backward"
            else 1.0
        )
        if control.strict_edge_tracking:
            path_heading_error = TrajectoryMath.normalize_angle(
                projection.yaw - pose.yaw
            )
            target_heading_error = path_heading_error
            path_curvature = local_reference.curvature
        else:
            effective_lookahead = TrajectoryMath.clamp(
                control.lookahead
                * (
                    1.0
                    - (0.35 * min(1.0, curvature_hint / 1.1))
                    - (
                        0.30
                        * min(
                            1.0,
                            abs(projection.cross_track)
                            / max(0.2, control.on_route_tolerance * 2.0),
                        )
                    )
                ),
                0.12,
                control.lookahead,
            )
            target = trajectory.sample_at(
                min(tracking_limit, projection.s + effective_lookahead)
            )
            drive_sign = (
                -1.0 if target.motion_direction == "backward" else 1.0
            )
            path_heading_error = TrajectoryMath.normalize_angle(
                target.yaw - pose.yaw
            )
            target_bearing = math.atan2(target.y - pose.y, target.x - pose.x)
            target_heading_error = (
                path_heading_error
                if drive_sign < 0.0
                else TrajectoryMath.normalize_angle(target_bearing - pose.yaw)
            )
            path_curvature = target.curvature
        curvature_preview_end = min(
            tracking_limit,
            projection.s + control.curve_preview_distance,
        )
        preview_curvature = trajectory.max_abs_curvature_between(
            projection.s,
            curvature_preview_end,
        )
        tracking_speed_reference = min(
            control.route_speed,
            control.strict_speed_limit
            if control.strict_edge_tracking
            else control.route_speed,
        )
        curve_detected = (
            curvature_hint > control.curve_heading_threshold
            or preview_curvature > control.curve_curvature_threshold
        )
        if curve_detected:
            tracking_speed_reference = min(
                tracking_speed_reference,
                control.curve_speed_limit,
            )
        if preview_curvature > control.curve_curvature_threshold:
            angular_feedforward_budget = (
                control.max_angular * (1.0 - control.curve_angular_reserve)
            )
            tracking_speed_reference = min(
                tracking_speed_reference,
                max(
                    control.curve_min_speed,
                    angular_feedforward_budget / preview_curvature,
                ),
            )
        # Projection yaw is the desired body yaw.  On a backward edge it is
        # rotated by pi relative to the direction in which path progress grows,
        # so its signed normal is inverted.  PID lateral error must stay in the
        # travel frame; otherwise reverse motion steers away from the edge.
        cross_track_error = drive_sign * projection.cross_track
        if (
            control.strict_edge_tracking
            and abs(cross_track_error) > control.precision_lateral_tolerance
        ):
            tracking_speed_reference = min(
                tracking_speed_reference,
                control.precision_speed_limit,
            )
        if abs(cross_track_error) > control.on_route_tolerance:
            tracking_speed_reference = min(
                tracking_speed_reference,
                control.rejoin_speed_limit,
            )
        if abs(cross_track_error) > max(
            0.32,
            control.on_route_tolerance * 2.5,
        ):
            tracking_speed_reference = min(
                tracking_speed_reference,
                control.hard_rejoin_speed_limit,
            )
        precision_blend = TrajectoryMath.clamp(
            1.0
            - (
                abs(cross_track_error)
                / max(control.on_route_tolerance, 1e-6)
            ),
            0.0,
            1.0,
        )
        lateral_gain = (
            control.rejoin_lateral_error_gain
            + precision_blend
            * (
                control.lateral_error_gain
                - control.rejoin_lateral_error_gain
            )
        )
        cross_track_term = math.atan2(
            lateral_gain * cross_track_error,
            max(tracking_speed_reference, control.curve_min_speed),
        )
        if control.strict_edge_tracking or drive_sign < 0.0:
            heading_control_error = path_heading_error
        else:
            heading_control_error = (
                (0.45 * path_heading_error)
                + (0.55 * target_heading_error)
            )
        lateral_control_error = -cross_track_term
        steering_error = heading_control_error + lateral_control_error
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
            lateral_control_error=lateral_control_error,
            heading_control_error=heading_control_error,
            path_curvature=path_curvature,
            preview_curvature=preview_curvature,
            tracking_speed_reference=tracking_speed_reference,
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
        elif not steering.off_route and abs(steering.steering_error) > (
            control.drive_alignment_angle
            if control.strict_edge_tracking
            else control.rotate_in_place_angle
        ):
            linear = 0.0
            steering_error = steering.steering_error
        elif (
            steering.off_route
            and abs(steering_error) > (control.rotate_in_place_angle * 1.15)
        ):
            linear = 0.0

        angular = TrajectoryMath.clamp(
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
        if (
            steering.curvature_hint > control.curve_heading_threshold
            or steering.preview_curvature
            > control.curve_curvature_threshold
        ):
            linear = steering.drive_sign * min(
                abs(linear),
                control.curve_speed_limit,
            )
        if steering.tracking_speed_reference > 0.0:
            linear = steering.drive_sign * min(
                abs(linear),
                steering.tracking_speed_reference,
            )
        if control.strict_edge_tracking:
            linear = steering.drive_sign * min(
                abs(linear),
                control.strict_speed_limit,
            )
        if (
            control.strict_edge_tracking
            and abs(steering.cross_track_error)
            > control.precision_lateral_tolerance
        ):
            linear = steering.drive_sign * min(
                abs(linear),
                control.precision_speed_limit,
            )
        return linear, angular, message
