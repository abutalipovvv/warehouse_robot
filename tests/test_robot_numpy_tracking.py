from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest


ROBOT_PLANNER_SRC = (
    Path(__file__).resolve().parents[1]
    / "sim_robot"
    / "ws"
    / "src"
    / "robot_planner"
)
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))

from robot_planner.control import (
    ArrivalMonitor,
    ArrivalParameters,
    LqrController,
    LqrParameters,
    PidController,
    PidParameters,
    SpeedProfileParameters,
    SpeedProfiler,
    TrajectorySpeedParameters,
    TrajectorySpeedProfile,
)
from robot_planner.execution import (
    RouteControlParameters,
    RouteExecutor,
    RouteProgress,
    RouteSteeringState,
)
from robot_planner.route_core import EdgeGeometry, GraphEdge, Landmark, LmRoutePlanner, WorldPoint
from robot_planner.runtime import PlannedRobotRoute, Pose2D, RobotRuntime, RoutePoint
from robot_planner.math import TrajectoryArray, TrajectoryMath
from robot_planner.planning import MapfRoutePlan, RobotTrajectoryPlanner


def _straight_points() -> list[RoutePoint]:
    return [
        RoutePoint(0.0, 0.0, 0.0, "A->B", not_before=10.0),
        RoutePoint(1.0, 0.0, 0.0, "A->B", not_before=20.0),
        RoutePoint(2.0, 0.0, 0.0, "B->C", not_before=30.0),
    ]


def test_numpy_trajectory_projects_pose_and_interpolates_metadata() -> None:
    trajectory = TrajectoryArray.from_route_points(_straight_points())

    projection = trajectory.project((0.75, 0.20))
    sample = trajectory.sample_at(1.75)

    assert trajectory.xy.shape == (3, 2)
    assert trajectory.s.tolist() == pytest.approx([0.0, 1.0, 2.0])
    assert projection.index == 1
    assert projection.s == pytest.approx(0.75)
    assert projection.cross_track == pytest.approx(0.20)
    assert projection.distance == pytest.approx(0.20)
    assert sample.x == pytest.approx(1.75)
    assert sample.y == pytest.approx(0.0)
    assert sample.edge_id == "B->C"
    assert sample.not_before == pytest.approx(30.0)
    assert not trajectory.xy.flags.writeable


def test_numpy_trajectory_estimates_signed_curve_curvature() -> None:
    diagonal = math.sqrt(0.5)
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(1.0, 0.0, math.pi / 2.0, "A->B"),
            RoutePoint(diagonal, diagonal, 3.0 * math.pi / 4.0, "A->B"),
            RoutePoint(0.0, 1.0, math.pi, "A->B"),
        ]
    )

    assert trajectory.curvature == pytest.approx(np.ones(3), rel=1e-6)


def test_cubic_bezier_is_sampled_by_matrix_basis() -> None:
    points, derivatives = TrajectoryMath.sample_cubic_bezier(
        ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)),
        steps=4,
    )

    assert points.shape == (5, 2)
    assert derivatives.shape == (5, 2)
    assert points[0] == pytest.approx((0.0, 0.0))
    assert points[-1] == pytest.approx((2.0, 1.0))
    assert derivatives[0] == pytest.approx((3.0, 0.0))
    assert derivatives[-1] == pytest.approx((3.0, 0.0))


def test_cubic_bezier_arc_length_samples_are_evenly_spaced() -> None:
    points, _ = TrajectoryMath.sample_cubic_bezier_by_arc_length(
        ((0.0, 0.0), (0.0, 2.0), (3.0, 2.0), (3.0, 0.0)),
        sample_distance=0.01,
    )

    spacing = np.linalg.norm(np.diff(points, axis=0), axis=1)

    assert spacing.max() <= 0.0101
    assert spacing.min() >= 0.0097


def test_reference_map_lm280_to_lm131_is_not_flattened_to_line() -> None:
    workspace_src = ROBOT_PLANNER_SRC.parent
    planner = RobotTrajectoryPlanner(
        workspace_src
        / "robot_map_manager"
        / "maps_out"
        / "22.05.26_smap.smap",
        workspace_src / "params.yaml",
    )
    start = planner.loaded_map.landmarks["LM280"]
    route = planner.plan_from_pose(
        Pose2D(x=start.x, y=start.y, yaw=0.0),
        "LM131",
        start_lm="LM280",
    )

    trajectory = np.asarray(
        [(point.x, point.y) for point in route.trajectory],
        dtype=np.float64,
    )
    chord = trajectory[-1] - trajectory[0]
    relative = trajectory - trajectory[0]
    distance_from_chord = np.abs(
        (chord[0] * relative[:, 1]) - (chord[1] * relative[:, 0])
    ) / np.linalg.norm(chord)

    assert route.nodes == ["LM280", "LM131"]
    assert len(route.trajectory) >= 300
    assert {point.edge_id for point in route.trajectory} == {"LM280->LM131"}
    assert float(distance_from_chord.max()) > 0.5
    spacing = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
    assert spacing.max() <= 0.0101


def test_mapf_contract_keeps_geometry_and_control_robot_local() -> None:
    workspace_src = ROBOT_PLANNER_SRC.parent
    planner = RobotTrajectoryPlanner(
        workspace_src
        / "robot_map_manager"
        / "maps_out"
        / "22.05.26_smap.smap",
        workspace_src / "params.yaml",
    )
    start = planner.loaded_map.landmarks["LM280"]
    payload = {
        "protocol": "lm_route",
        "protocolVersion": 2,
        "routeId": "mapf-route-1",
        "revision": 3,
        "startLm": "LM280",
        "goalLm": "LM131",
        "nodes": ["LM280", "LM131"],
        "replaceMode": "immediate",
        "dispatchEpochSec": 100.0,
        "timedSegments": [
            {
                "kind": "move",
                "from": "LM280",
                "to": "LM131",
                "motionDirection": "forward",
                "notBeforeSec": 2.0,
                "plannedArrivalSec": 8.0,
            }
        ],
    }

    route = planner.plan_from_lm_route(
        Pose2D(x=start.x, y=start.y, yaw=0.0),
        payload,
    )

    assert route.route_id == "mapf-route-1"
    assert route.revision == 3
    assert route.nodes == ["LM280", "LM131"]
    assert len(route.trajectory) >= 300
    assert {point.edge_id for point in route.trajectory} == {
        "LM280->LM131"
    }
    assert {point.not_before for point in route.trajectory} == {102.0}


def test_mapf_contract_rejects_inconsistent_goal_and_direction() -> None:
    with pytest.raises(ValueError, match="does not match last node"):
        MapfRoutePlan.from_payload(
            {
                "protocol": "lm_route",
                "protocolVersion": 2,
                "goalLm": "C",
                "nodes": ["A", "B"],
            }
        )

    workspace_src = ROBOT_PLANNER_SRC.parent
    planner = RobotTrajectoryPlanner(
        workspace_src
        / "robot_map_manager"
        / "maps_out"
        / "22.05.26_smap.smap",
        workspace_src / "params.yaml",
    )
    start = planner.loaded_map.landmarks["LM280"]
    with pytest.raises(ValueError, match="motion direction mismatch"):
        planner.plan_from_lm_route(
            Pose2D(x=start.x, y=start.y, yaw=0.0),
            {
                "protocol": "lm_route",
                "protocolVersion": 2,
                "goalLm": "LM131",
                "nodes": ["LM280", "LM131"],
                "timedSegments": [
                    {
                        "kind": "move",
                        "from": "LM280",
                        "to": "LM131",
                        "motionDirection": "backward",
                    }
                ],
            },
        )


def test_mapf_contract_never_connects_to_start_lm_with_a_straight_shortcut() -> None:
    workspace_src = ROBOT_PLANNER_SRC.parent
    planner = RobotTrajectoryPlanner(
        workspace_src
        / "robot_map_manager"
        / "maps_out"
        / "22.05.26_smap.smap",
        workspace_src / "params.yaml",
    )

    with pytest.raises(ValueError, match="strict LM route rejected"):
        planner.plan_from_lm_route(
            Pose2D(x=1_000.0, y=1_000.0, yaw=0.0),
            {
                "protocol": "lm_route",
                "protocolVersion": 2,
                "routeId": "off-graph-route",
                "revision": 1,
                "startLm": "LM280",
                "goalLm": "LM131",
                "nodes": ["LM280", "LM131"],
                "replaceMode": "immediate",
            },
        )


def test_speed_profiler_slows_to_millimetre_goal_without_speed_floor() -> None:
    params = SpeedProfileParameters.from_payload(
        {
            "navigation": {
                "control_period": 0.05,
                "max_linear_acceleration": 0.5,
                "max_linear_deceleration": 0.8,
                "goal_precision_speed_limit": 0.08,
                "goal_precision_min_speed": 0.01,
                "goal_precision_linear_gain": 1.2,
            },
            "planner": {"precision_start_distance": 0.10},
            "localization": {"goal_position_tolerance": 0.005},
        }
    )
    profiler = SpeedProfiler()

    assert profiler.step(0.5, 2.0, params) == pytest.approx(0.025)
    for _ in range(30):
        profiler.step(0.5, 2.0, params)
    precision_speed = profiler.step(0.5, 0.020, params)

    assert precision_speed == pytest.approx(0.018)
    assert profiler.step(0.5, 0.005, params) == pytest.approx(0.0)


def test_trajectory_speed_profile_brakes_before_curve_and_goal() -> None:
    points = [
        RoutePoint(float(x), 0.0, 0.0, "A->B")
        for x in np.linspace(0.0, 1.0, 101)
    ]
    points.extend(
        RoutePoint(
            1.0 + 0.5 * math.sin(angle),
            0.5 - 0.5 * math.cos(angle),
            angle,
            "B->C",
        )
        for angle in np.linspace(0.01, math.pi / 2.0, 157)
    )
    trajectory = TrajectoryArray.from_route_points(points)
    params = TrajectorySpeedParameters.from_payload(
        {
            "navigation": {
                "route_speed": 1.0,
                "strict_speed_limit": 1.0,
                "max_linear_acceleration": 0.5,
                "max_linear_deceleration": 0.8,
                "curve_speed_limit": 0.3,
                "trajectory_speed_profile": {
                    "max_forward_speed": 1.0,
                    "max_backward_speed": 0.4,
                    "max_lateral_acceleration": 0.2,
                    "max_jerk": 1.5,
                },
            }
        }
    )

    profile = TrajectorySpeedProfile.build(trajectory, params)

    assert profile.speed_at(0.2) == pytest.approx(1.0)
    assert profile.speed_at(0.8) < 0.65
    assert profile.speed_at(1.2) <= 0.3
    assert profile.speed_at(trajectory.length) == pytest.approx(0.0)
    assert float(profile.acceleration.max()) <= 0.5 + 1e-9
    assert float(profile.acceleration.min()) >= -0.8 - 1e-9
    assert not profile.speed_limits.flags.writeable


def test_trajectory_speed_profile_has_separate_backward_limit() -> None:
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(
                index * 0.1,
                0.0,
                math.pi,
                "A->B",
                motion_direction="backward",
            )
            for index in range(21)
        ]
    )
    params = TrajectorySpeedParameters.from_payload(
        {
            "navigation": {
                "route_speed": 1.0,
                "strict_speed_limit": 1.0,
                "trajectory_speed_profile": {
                    "max_forward_speed": 0.8,
                    "max_backward_speed": 0.3,
                },
            }
        }
    )

    profile = TrajectorySpeedProfile.build(trajectory, params)

    assert profile.speed_at(0.0) == pytest.approx(0.3)
    assert float(profile.speed_limits.max()) <= 0.3


def test_arrival_requires_five_stationary_precision_cycles() -> None:
    params = ArrivalParameters.from_payload(
        {
            "localization": {
                "goal_position_tolerance": 0.005,
                "allowed_yaw_error_deg": 1.0,
                "goal_linear_velocity_tolerance": 0.01,
                "goal_angular_velocity_tolerance": 0.03,
                "arrival_stable_cycles": 5,
            }
        }
    )
    monitor = ArrivalMonitor()

    assert not monitor.update(
        remaining_distance=0.004,
        goal_position_error=0.004,
        goal_yaw_error=math.radians(0.5),
        linear_velocity=0.02,
        angular_velocity=0.0,
        params=params,
    )
    for _ in range(4):
        assert not monitor.update(
            remaining_distance=0.004,
            goal_position_error=0.004,
            goal_yaw_error=math.radians(0.5),
            linear_velocity=0.0,
            angular_velocity=0.0,
            params=params,
        )
    assert monitor.update(
        remaining_distance=0.004,
        goal_position_error=0.004,
        goal_yaw_error=math.radians(0.5),
        linear_velocity=0.0,
        angular_velocity=0.0,
        params=params,
    )


@pytest.mark.parametrize(
    ("motion_direction", "start_yaw"),
    (("forward", 0.0), ("backward", math.pi)),
)
@pytest.mark.parametrize("tracking_controller", ("pid", "lqr"))
def test_executor_arrives_within_five_millimetres(
    motion_direction: str,
    start_yaw: float,
    tracking_controller: str,
) -> None:
    payload = {
        "navigation": {
            "route_speed": 0.5,
            "tracking_controller": tracking_controller,
            "strict_speed_limit": 0.5,
            "control_period": 0.05,
            "max_linear_acceleration": 0.5,
            "max_linear_deceleration": 0.8,
            "goal_precision_speed_limit": 0.08,
            "goal_precision_min_speed": 0.01,
            "goal_precision_linear_gain": 1.2,
        },
        "planner": {
            "on_route_tolerance": 0.1,
            "precision_start_distance": 0.1,
        },
        "localization": {
            "goal_position_tolerance": 0.005,
            "allowed_yaw_error_deg": 1.0,
            "goal_linear_velocity_tolerance": 0.01,
            "goal_angular_velocity_tolerance": 0.03,
            "arrival_stable_cycles": 5,
        },
    }

    class Planner:
        def current_params(self):
            return payload

        @staticmethod
        def map_angular_to_ros(angular: float) -> float:
            return -angular

    points = [
        RoutePoint(
            index * 0.01,
            0.0,
            start_yaw,
            "A->B",
            motion_direction=motion_direction,
        )
        for index in range(101)
    ]
    route = PlannedRobotRoute.create(
        start_lm="A",
        goal_lm="B",
        nodes=["A", "B"],
        trajectory=points,
        length=1.0,
    )
    runtime = RobotRuntime(robot_id="robot", map_id="map")
    runtime.set_route(route)
    commands: list[tuple[float, float]] = []
    executor = RouteExecutor(
        runtime,
        Planner(),
        lambda linear, angular: commands.append((linear, angular)),
    )
    pose = Pose2D(x=0.0, y=0.0, yaw=start_yaw)
    dt = 0.05

    for _ in range(1000):
        active_route = runtime.active_route()
        if active_route is None:
            break
        executor._follow_route(active_route, pose)
        linear, ros_angular = commands[-1]
        executor._measured_linear = linear
        executor._measured_angular = ros_angular
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = TrajectoryMath.normalize_angle(
            pose.yaw - (ros_angular * dt)
        )

    snapshot = runtime.snapshot()
    assert runtime.active_route() is None
    assert snapshot["state"] == "ARRIVED"
    assert abs(1.0 - pose.x) <= 0.005
    assert abs(pose.y) <= 0.005
    assert snapshot["tracking"]["goalPositionError"] <= 0.005
    assert snapshot["tracking"]["arrivalStableCycles"] == 5


def test_odom_anchor_propagates_pose_in_left_handed_graph_frame() -> None:
    odom_yaw = 1.0
    forward = 0.5
    left = 0.2
    odom_delta = np.asarray(
        (
            (math.cos(odom_yaw), -math.sin(odom_yaw)),
            (math.sin(odom_yaw), math.cos(odom_yaw)),
        )
    ) @ np.asarray((forward, left))

    pose = TrajectoryMath.map_pose_from_odom_anchor(
        (10.0, 5.0, 0.0),
        (2.0, 3.0, odom_yaw),
        (2.0 + odom_delta[0], 3.0 + odom_delta[1], odom_yaw + 0.3),
    )

    assert pose == pytest.approx((10.5, 4.8, -0.3))


def test_route_tracking_pose_uses_fixed_map_anchor_and_smooth_odom() -> None:
    executor = object.__new__(RouteExecutor)
    executor._odom_anchor_route_key = None
    executor._odom_anchor_map_pose = None
    executor._odom_anchor_pose = None
    route_key = ("route", 0)
    localized = Pose2D(x=18.0, y=7.0, yaw=math.pi)

    anchored = executor._tracking_pose(
        route_key,
        localized,
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        enabled=True,
    )
    propagated = executor._tracking_pose(
        route_key,
        Pose2D(x=99.0, y=99.0, yaw=0.0),
        {"x": 0.5, "y": 0.0, "yaw": -0.2},
        enabled=True,
    )
    next_route = executor._tracking_pose(
        ("next-route", 0),
        Pose2D(x=99.0, y=99.0, yaw=0.0),
        {"x": 0.6, "y": 0.0, "yaw": -0.2},
        enabled=True,
    )

    assert anchored == localized
    assert propagated.x == pytest.approx(17.5)
    assert propagated.y == pytest.approx(7.0)
    assert propagated.yaw == pytest.approx(
        TrajectoryMath.normalize_angle(math.pi + 0.2)
    )
    assert next_route.x == pytest.approx(17.4)
    assert next_route.y == pytest.approx(7.0)


def test_explicit_relocation_reanchors_odom_tracking() -> None:
    executor = object.__new__(RouteExecutor)
    executor._odom_anchor_route_key = ("old-route", 0)
    executor._odom_anchor_map_pose = Pose2D(x=1.0, y=2.0, yaw=0.0)
    executor._odom_anchor_pose = Pose2D(x=3.0, y=4.0, yaw=0.0)
    executor._last_tracking_pose = Pose2D(x=1.0, y=2.0, yaw=0.0)

    anchored = executor.reanchor_pose(
        Pose2D(x=10.0, y=20.0, yaw=0.5),
        {"x": 7.0, "y": 8.0, "yaw": 0.0},
    )
    propagated = executor._tracking_pose(
        ("new-route", 0),
        Pose2D(x=99.0, y=99.0, yaw=0.0),
        {"x": 7.2, "y": 8.0, "yaw": 0.1},
        enabled=True,
    )

    assert anchored is True
    assert executor._odom_anchor_route_key == ("new-route", 0)
    assert propagated.x == pytest.approx(10.0 + 0.2 * math.cos(0.5))
    assert propagated.y == pytest.approx(20.0 + 0.2 * math.sin(0.5))
    assert propagated.yaw == pytest.approx(0.4)


def test_route_planner_preserves_backward_heading_on_numpy_bezier() -> None:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=2.0, y=1.0),
    }
    controls = (
        WorldPoint(0.0, 0.0),
        WorldPoint(1.0, 0.0),
        WorldPoint(1.0, 1.0),
        WorldPoint(2.0, 1.0),
    )
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=2.5,
        kind="bezier",
        edge_type="Bezier",
        world_points=(controls[0], controls[-1]),
        geometry=EdgeGeometry(
            geometry="bezier",
            control_points=controls,
            curve_type="Bezier",
        ),
        properties={"direction": 1},
    )
    planner = LmRoutePlanner(
        landmarks,
        [edge],
        params={"planner": {"trajectory_sample_distance": 0.25}},
    )

    samples = planner._sample_edge(edge, 0.25)

    assert samples[0]["x"] == pytest.approx(0.0)
    assert samples[-1]["x"] == pytest.approx(2.0)
    assert samples[0]["motionDirection"] == "backward"
    assert abs(abs(float(samples[0]["yaw"])) - math.pi) < 1e-9


def test_pid_combines_lateral_and_heading_states() -> None:
    params = PidParameters.from_payload(
        {
            "navigation": {
                "tracking_pid": {
                    "kp": [2.0, 3.0],
                    "ki": [0.1, 0.2],
                    "kd": [0.0, 0.0],
                    "integral_limit": [1.0, 1.0],
                }
            }
        }
    )
    controller = PidController(params)

    output = controller.step(
        (0.2, -0.1),
        feedforward=0.0,
        max_output=1.0,
        dt=0.1,
    )

    assert output == pytest.approx(0.1)
    assert controller.last_terms.proportional == pytest.approx(0.1)
    assert controller.last_terms.integral == pytest.approx(0.0)


def test_pid_inherits_existing_angular_gain_when_kp_is_not_set() -> None:
    params = PidParameters.from_payload(
        {
            "navigation": {
                "angular_gain": 3.1,
                "tracking_pid": {"ki": [0.0, 0.0], "kd": [0.0, 0.0]},
            }
        }
    )

    assert params.kp == pytest.approx((3.1, 3.1))
    assert params.curvature_feedforward_gain == pytest.approx(1.0)


def test_pid_anti_windup_does_not_integrate_into_saturation() -> None:
    params = PidParameters.from_payload(
        {
            "navigation": {
                "tracking_pid": {
                    "kp": [10.0, 0.0],
                    "ki": [1.0, 0.0],
                    "kd": [0.0, 0.0],
                    "integral_limit": [1.0, 1.0],
                }
            }
        }
    )
    controller = PidController(params)

    output = controller.step(
        (1.0, 0.0),
        feedforward=0.0,
        max_output=0.5,
        dt=0.1,
    )

    assert output == pytest.approx(0.5)
    assert controller.last_terms.integral == pytest.approx(0.0)


def test_lqr_uses_lateral_heading_and_curvature_terms() -> None:
    controller = LqrController(LqrParameters.from_payload({}))

    lateral_output = controller.step(
        cross_track_error=0.2,
        heading_error=0.0,
        linear_speed=0.3,
        curvature=0.0,
        max_output=1.0,
        dt=0.05,
    )
    heading_output = controller.step(
        cross_track_error=0.0,
        heading_error=0.2,
        linear_speed=0.3,
        curvature=0.0,
        max_output=1.0,
        dt=0.05,
    )
    feedforward_output = controller.step(
        cross_track_error=0.0,
        heading_error=0.0,
        linear_speed=0.3,
        curvature=1.0,
        max_output=1.0,
        dt=0.05,
    )

    assert lateral_output < 0.0
    assert heading_output > 0.0
    assert feedforward_output == pytest.approx(0.3)


@pytest.mark.parametrize(
    ("motion_direction", "start_yaw"),
    (("forward", 0.0), ("backward", math.pi)),
)
def test_lqr_tracking_controller_returns_robot_to_straight_path(
    motion_direction: str,
    start_yaw: float,
) -> None:
    payload = {
        "navigation": {
            "route_speed": 0.30,
            "tracking_controller": "lqr",
            "max_angular_speed": 0.9,
        },
        "planner": {"on_route_tolerance": 0.12},
    }
    control = RouteControlParameters.from_payload(payload)
    controller = LqrController(LqrParameters.from_payload(payload))
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(
                index * 0.05,
                0.0,
                start_yaw,
                "A->B",
                motion_direction=motion_direction,
            )
            for index in range(121)
        ]
    )
    executor = object.__new__(RouteExecutor)
    route = type("Route", (), {"goal_lm": "B"})()
    pose = Pose2D(x=0.0, y=0.25, yaw=start_yaw)
    hint_index = 0
    dt = 0.05

    for _ in range(300):
        projection = trajectory.project((pose.x, pose.y), hint_index)
        hint_index = projection.index
        steering = executor._route_steering_state_array(
            trajectory,
            pose,
            projection,
            hint_index,
            trajectory.length,
            control,
        )
        linear, angular, _ = executor._route_drive_command(
            route,
            RouteProgress(
                distance_to_goal=math.hypot(6.0 - pose.x, pose.y),
                final_yaw_error=0.0,
                remaining_distance=trajectory.length - projection.s,
            ),
            steering,
            control,
        )
        if abs(linear) > 1e-6:
            angular = controller.step(
                cross_track_error=steering.cross_track_error,
                heading_error=steering.heading_control_error,
                linear_speed=linear,
                curvature=steering.path_curvature,
                max_output=control.max_angular,
                dt=dt,
            )
        else:
            controller.reset()
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = TrajectoryMath.normalize_angle(pose.yaw + angular * dt)

    assert control.tracking_controller == "lqr"
    assert pose.x > 2.5
    assert abs(pose.y) < 0.005
    assert abs(TrajectoryMath.normalize_angle(pose.yaw - start_yaw)) < 0.005


def test_pid_tracking_controller_returns_robot_to_straight_path() -> None:
    payload = {
        "navigation": {
            "route_speed": 0.35,
            "angular_gain": 2.2,
            "max_angular_speed": 0.9,
        },
        "planner": {"on_route_tolerance": 0.12},
    }
    control = RouteControlParameters.from_payload(payload)
    controller = PidController(PidParameters.from_payload(payload))
    trajectory = TrajectoryArray.from_route_points(
        [RoutePoint(index * 0.05, 0.0, 0.0, "A->B") for index in range(121)]
    )
    executor = object.__new__(RouteExecutor)
    route = type("Route", (), {"goal_lm": "B"})()
    pose = Pose2D(x=0.0, y=0.25, yaw=0.0)
    hint_index = 0
    dt = 0.05

    for _ in range(160):
        projection = trajectory.project((pose.x, pose.y), hint_index)
        hint_index = projection.index
        steering = executor._route_steering_state_array(
            trajectory,
            pose,
            projection,
            hint_index,
            trajectory.length,
            control,
        )
        progress = RouteProgress(
            distance_to_goal=math.hypot(6.0 - pose.x, pose.y),
            final_yaw_error=0.0,
            remaining_distance=trajectory.length - projection.s,
        )
        linear, angular, _ = executor._route_drive_command(
            route,
            progress,
            steering,
            control,
        )
        if abs(linear) > 1e-6:
            angular = controller.step(
                (steering.lateral_control_error, steering.heading_control_error),
                feedforward=abs(linear) * steering.path_curvature,
                max_output=control.max_angular,
                dt=dt,
            )
        else:
            controller.reset()
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = math.atan2(
            math.sin(pose.yaw + angular * dt),
            math.cos(pose.yaw + angular * dt),
        )

    assert pose.x > 1.5
    assert abs(pose.y) < 0.01
    assert abs(pose.yaw) < 0.01


def test_pid_tracking_controller_returns_backward_robot_to_straight_path() -> None:
    payload = {
        "navigation": {
            "route_speed": 0.30,
            "angular_gain": 2.2,
            "max_angular_speed": 0.9,
        },
        "planner": {"on_route_tolerance": 0.12},
    }
    control = RouteControlParameters.from_payload(payload)
    controller = PidController(PidParameters.from_payload(payload))
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(
                index * 0.05,
                0.0,
                math.pi,
                "A->B",
                motion_direction="backward",
            )
            for index in range(121)
        ]
    )
    executor = object.__new__(RouteExecutor)
    route = type("Route", (), {"goal_lm": "B"})()
    pose = Pose2D(x=0.0, y=0.25, yaw=math.pi)
    hint_index = 0
    dt = 0.05

    for _ in range(190):
        projection = trajectory.project((pose.x, pose.y), hint_index)
        hint_index = projection.index
        steering = executor._route_steering_state_array(
            trajectory,
            pose,
            projection,
            hint_index,
            trajectory.length,
            control,
        )
        linear, _, _ = executor._route_drive_command(
            route,
            RouteProgress(
                distance_to_goal=math.hypot(6.0 - pose.x, pose.y),
                final_yaw_error=0.0,
                remaining_distance=trajectory.length - projection.s,
            ),
            steering,
            control,
        )
        angular = controller.step(
            (steering.lateral_control_error, steering.heading_control_error),
            feedforward=linear * steering.path_curvature,
            max_output=control.max_angular,
            dt=dt,
        )
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = TrajectoryMath.normalize_angle(pose.yaw + angular * dt)

    assert pose.x > 1.5
    assert abs(pose.y) < 0.015
    assert abs(TrajectoryMath.normalize_angle(pose.yaw - math.pi)) < 0.015


def test_backward_curve_feedforward_keeps_path_curvature_sign() -> None:
    linear = -0.25
    curvature = 0.8

    assert abs(linear) * curvature == pytest.approx(0.2)


def test_backward_pid_tracks_curve_without_lateral_drift() -> None:
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(
                math.sin(angle),
                1.0 - math.cos(angle),
                TrajectoryMath.normalize_angle(angle + math.pi),
                "A->B",
                motion_direction="backward",
            )
            for angle in np.linspace(0.0, math.pi / 2.0, 101)
        ]
    )
    payload = {
        "navigation": {
            "route_speed": 0.4,
            "curve_speed_limit": 0.25,
            "max_angular_speed": 0.9,
            "angular_gain": 2.2,
        },
        "planner": {"on_route_tolerance": 0.1},
    }
    control = RouteControlParameters.from_payload(payload)
    controller = PidController(PidParameters.from_payload(payload))
    executor = object.__new__(RouteExecutor)
    route = type("Route", (), {"goal_lm": "B"})()
    pose = Pose2D(x=0.0, y=0.0, yaw=math.pi)
    hint_index = 0
    maximum_cross_track = 0.0
    dt = 0.05

    for _ in range(120):
        projection = trajectory.project((pose.x, pose.y), hint_index)
        hint_index = projection.index
        maximum_cross_track = max(maximum_cross_track, abs(projection.cross_track))
        steering = executor._route_steering_state_array(
            trajectory,
            pose,
            projection,
            hint_index,
            trajectory.length,
            control,
            target_s_limit=trajectory.length,
        )
        linear, _, _ = executor._route_drive_command(
            route,
            RouteProgress(10.0, 0.0, trajectory.length - projection.s),
            steering,
            control,
        )
        angular = controller.step(
            (steering.lateral_control_error, steering.heading_control_error),
            feedforward=abs(linear) * steering.path_curvature,
            max_output=control.max_angular,
            dt=dt,
        )
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = TrajectoryMath.normalize_angle(pose.yaw + angular * dt)

    assert maximum_cross_track < 0.005
    assert pose.x == pytest.approx(1.0, abs=0.01)
    assert pose.y > 0.90


def test_strict_tracking_slows_but_does_not_turn_before_curve_edge() -> None:
    control = RouteControlParameters.from_payload(
        {
            "navigation": {
                "route_speed": 0.8,
                "curve_speed_limit": 0.25,
                "footprint_lookahead": 0.8,
            }
        }
    )
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(0.0, 0.0, 0.0, "A->B"),
            RoutePoint(0.5, 0.0, 0.0, "A->B"),
            RoutePoint(1.0, 0.0, 0.0, "A->B"),
            RoutePoint(1.03, 0.00, 0.10, "B->C"),
            RoutePoint(1.25, 0.08, 0.60, "B->C"),
            RoutePoint(1.40, 0.25, 0.90, "B->C"),
        ]
    )
    pose = Pose2D(x=0.5, y=0.0, yaw=0.0)
    projection, _, edge_end = RouteExecutor._project_pose_to_active_edge(
        trajectory,
        pose,
        1,
    )
    executor = object.__new__(RouteExecutor)
    steering = executor._route_steering_state_array(
        trajectory,
        pose,
        projection,
        projection.index,
        trajectory.length,
        control,
        target_s_limit=float(trajectory.s[edge_end]),
    )
    linear, angular, _ = executor._route_drive_command(
        type("Route", (), {"goal_lm": "C"})(),
        RouteProgress(
            distance_to_goal=2.0,
            final_yaw_error=0.0,
            remaining_distance=2.0,
        ),
        steering,
        control,
    )
    controller = PidController(PidParameters.from_payload({}))
    angular = controller.step(
        (steering.lateral_control_error, steering.heading_control_error),
        feedforward=abs(linear) * steering.path_curvature,
        max_output=control.max_angular,
        dt=0.05,
    )

    assert control.strict_edge_tracking is True
    assert steering.curvature_hint > control.curve_heading_threshold
    assert steering.path_heading_error == pytest.approx(0.0)
    assert steering.cross_track_error == pytest.approx(0.0)
    assert steering.path_curvature == pytest.approx(0.0)
    assert linear == pytest.approx(control.curve_speed_limit)
    assert angular == pytest.approx(0.0)


def test_strict_tracking_uses_precision_speed_outside_two_centimeters() -> None:
    control = RouteControlParameters.from_payload(
        {
            "navigation": {
                "route_speed": 1.0,
                "strict_edge_tracking": True,
                "precision_lateral_tolerance": 0.02,
                "precision_speed_limit": 0.25,
            },
            "planner": {"on_route_tolerance": 0.10},
        }
    )
    steering = RouteSteeringState(
        curvature_hint=0.0,
        drive_sign=1.0,
        path_heading_error=0.0,
        target_heading_error=0.0,
        cross_track_error=0.03,
        steering_error=-0.09,
        off_route=False,
        hard_rejoin=False,
    )
    linear, _, _ = RouteExecutor._route_drive_command(
        type("Route", (), {"goal_lm": "B"})(),
        RouteProgress(2.0, 0.0, 2.0),
        steering,
        control,
    )

    assert linear == pytest.approx(0.25)


def test_strict_curve_tracking_uses_real_capped_speed_for_lateral_feedback() -> None:
    control = RouteControlParameters.from_payload(
        {
            "navigation": {
                "route_speed": 1.0,
                "strict_edge_tracking": True,
                "strict_speed_limit": 0.4,
                "curve_speed_limit": 0.2,
                "curve_preview_distance": 0.25,
                "precision_speed_limit": 0.2,
                "max_angular_speed": 0.9,
            },
            "planner": {"on_route_tolerance": 0.1},
        }
    )
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(
                math.sin(angle),
                1.0 - math.cos(angle),
                angle,
                "A->B",
            )
            for angle in np.linspace(0.0, math.pi / 2.0, 101)
        ]
    )
    pose = Pose2D(x=0.20, y=0.05, yaw=0.20)
    projection = trajectory.project((pose.x, pose.y))
    executor = object.__new__(RouteExecutor)

    steering = executor._route_steering_state_array(
        trajectory,
        pose,
        projection,
        projection.index,
        trajectory.length,
        control,
        target_s_limit=trajectory.length,
    )

    assert steering.preview_curvature == pytest.approx(1.0, rel=2e-3)
    assert steering.tracking_speed_reference == pytest.approx(0.2)
    precision_blend = max(
        0.0,
        1.0 - abs(steering.cross_track_error) / control.on_route_tolerance,
    )
    lateral_gain = control.rejoin_lateral_error_gain + precision_blend * (
        control.lateral_error_gain - control.rejoin_lateral_error_gain
    )
    expected = -math.atan2(lateral_gain * steering.cross_track_error, 0.2)
    assert steering.lateral_control_error == pytest.approx(expected)


def test_curve_speed_preserves_angular_budget_for_pid_correction() -> None:
    control = RouteControlParameters.from_payload(
        {
            "navigation": {
                "route_speed": 1.0,
                "strict_edge_tracking": True,
                "strict_speed_limit": 0.4,
                "curve_speed_limit": 0.3,
                "curve_angular_reserve": 0.35,
                "max_angular_speed": 0.9,
            }
        }
    )
    speed_reference = (
        control.max_angular
        * (1.0 - control.curve_angular_reserve)
        / 2.5
    )
    steering = RouteSteeringState(
        curvature_hint=0.4,
        drive_sign=1.0,
        path_heading_error=0.0,
        target_heading_error=0.0,
        cross_track_error=0.0,
        steering_error=0.0,
        off_route=False,
        hard_rejoin=False,
        path_curvature=2.5,
        preview_curvature=2.5,
        tracking_speed_reference=speed_reference,
    )

    linear, _, _ = RouteExecutor._route_drive_command(
        type("Route", (), {"goal_lm": "B"})(),
        RouteProgress(2.0, 0.0, 2.0),
        steering,
        control,
    )

    assert linear * steering.preview_curvature == pytest.approx(
        control.max_angular * (1.0 - control.curve_angular_reserve)
    )


def test_strict_tracking_caps_nominal_straight_speed() -> None:
    control = RouteControlParameters.from_payload(
        {
            "navigation": {
                "route_speed": 1.0,
                "strict_edge_tracking": True,
                "strict_speed_limit": 0.4,
            }
        }
    )
    steering = RouteSteeringState(
        curvature_hint=0.0,
        drive_sign=1.0,
        path_heading_error=0.0,
        target_heading_error=0.0,
        cross_track_error=0.0,
        steering_error=0.0,
        off_route=False,
        hard_rejoin=False,
    )

    linear, _, _ = RouteExecutor._route_drive_command(
        type("Route", (), {"goal_lm": "B"})(),
        RouteProgress(2.0, 0.0, 2.0),
        steering,
        control,
    )

    assert linear == pytest.approx(0.4)


def test_edge_curvature_does_not_leak_into_previous_straight_edge() -> None:
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(0.0, 0.0, 0.0, "A->B"),
            RoutePoint(0.5, 0.0, 0.0, "A->B"),
            RoutePoint(1.0, 0.0, 0.0, "A->B"),
            RoutePoint(1.0, 0.1, math.pi / 2.0, "B->C"),
            RoutePoint(1.1, 0.2, math.pi / 4.0, "B->C"),
            RoutePoint(1.2, 0.2, 0.0, "B->C"),
        ]
    )

    assert trajectory.curvature[:3] == pytest.approx(np.zeros(3))
    assert np.max(np.abs(trajectory.curvature[3:])) > 0.1


@pytest.mark.parametrize("tracking_controller", ("pid", "lqr"))
def test_strict_tracking_follows_curve_without_cutting_inside(
    tracking_controller: str,
) -> None:
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(
                math.sin(angle),
                1.0 - math.cos(angle),
                angle,
                "A->B",
            )
            for angle in np.linspace(0.0, math.pi / 2.0, 101)
        ]
    )
    payload = {
        "navigation": {
            "route_speed": 0.4,
            "tracking_controller": tracking_controller,
            "curve_speed_limit": 0.25,
            "footprint_lookahead": 0.65,
            "max_angular_speed": 0.9,
            "angular_gain": 2.2,
        },
        "planner": {"on_route_tolerance": 0.1},
    }
    control = RouteControlParameters.from_payload(payload)
    pid = PidController(PidParameters.from_payload(payload))
    lqr = LqrController(LqrParameters.from_payload(payload))
    executor = object.__new__(RouteExecutor)
    route = type("Route", (), {"goal_lm": "B"})()
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    hint_index = 0
    maximum_cross_track = 0.0
    dt = 0.05

    for _ in range(120):
        projection = trajectory.project((pose.x, pose.y), hint_index)
        hint_index = projection.index
        maximum_cross_track = max(
            maximum_cross_track,
            abs(projection.cross_track),
        )
        steering = executor._route_steering_state_array(
            trajectory,
            pose,
            projection,
            hint_index,
            trajectory.length,
            control,
            target_s_limit=trajectory.length,
        )
        linear, _, _ = executor._route_drive_command(
            route,
            RouteProgress(
                distance_to_goal=10.0,
                final_yaw_error=0.0,
                remaining_distance=trajectory.length - projection.s,
            ),
            steering,
            control,
        )
        if tracking_controller == "lqr":
            angular = lqr.step(
                cross_track_error=steering.cross_track_error,
                heading_error=steering.heading_control_error,
                linear_speed=linear,
                curvature=steering.path_curvature,
                max_output=control.max_angular,
                dt=dt,
            )
        else:
            angular = pid.step(
                (
                    steering.lateral_control_error,
                    steering.heading_control_error,
                ),
                feedforward=abs(linear) * steering.path_curvature,
                max_output=control.max_angular,
                dt=dt,
            )
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = math.atan2(
            math.sin(pose.yaw + angular * dt),
            math.cos(pose.yaw + angular * dt),
        )

    assert maximum_cross_track < 0.005
    assert pose.x == pytest.approx(1.0, abs=0.01)
    assert pose.y > 0.90


def test_active_edge_projection_cannot_jump_to_closer_later_edge() -> None:
    points = [
        RoutePoint(0.30, 0.00, 2.20, "CURRENT->A"),
        RoutePoint(0.20, 0.13, 2.20, "CURRENT->A"),
        RoutePoint(0.10, 0.27, 2.20, "CURRENT->A"),
        RoutePoint(0.00, 0.40, 2.20, "CURRENT->A"),
        RoutePoint(0.00, 0.30, -math.pi / 2.0, "A->B"),
        RoutePoint(0.00, 0.00, -math.pi / 2.0, "A->B"),
        RoutePoint(0.00, -0.30, -math.pi / 2.0, "A->B"),
    ]
    trajectory = TrajectoryArray.from_route_points(points)
    pose = Pose2D(x=0.12, y=0.02, yaw=math.pi)

    global_projection = trajectory.project((pose.x, pose.y), 0)
    active_projection, edge_start, edge_end = RouteExecutor._project_pose_to_active_edge(
        trajectory,
        pose,
        0,
    )

    assert global_projection.edge_id == "A->B"
    assert (edge_start, edge_end) == (0, 3)
    assert active_projection.edge_id == "CURRENT->A"
    assert active_projection.index <= edge_end


def test_sharp_edge_transition_is_reached_and_aligned_before_switch() -> None:
    payload = {
        "navigation": {
            "route_speed": 0.5,
            "footprint_lookahead": 0.3,
            "stop_distance": 0.08,
            "angular_gain": 2.2,
            "max_angular_speed": 1.2,
            "rotate_in_place_angle_deg": 32.0,
        },
        "planner": {"on_route_tolerance": 0.06},
        "localization": {"allowed_yaw_error_deg": 2.0},
    }

    class Planner:
        def current_params(self):
            return payload

        @staticmethod
        def map_angular_to_ros(angular: float) -> float:
            return -angular

    start = (0.30, 0.00)
    boundary = (0.00, 0.40)
    connector_yaw = math.atan2(
        boundary[1] - start[1],
        boundary[0] - start[0],
    )
    trajectory = [
        RoutePoint(
            start[0] + (boundary[0] - start[0]) * (index / 17.0),
            start[1] + (boundary[1] - start[1]) * (index / 17.0),
            connector_yaw,
            "CURRENT->A",
        )
        for index in range(18)
    ]
    trajectory.extend(
        RoutePoint(
            0.0,
            boundary[1] + (-1.0 - boundary[1]) * (index / 47.0),
            -math.pi / 2.0,
            "A->B",
        )
        for index in range(1, 48)
    )
    route = PlannedRobotRoute.create(
        start_lm="A",
        goal_lm="B",
        nodes=["CURRENT", "A", "B"],
        trajectory=trajectory,
        length=1.9,
    )
    runtime = RobotRuntime(robot_id="robot", map_id="map")
    runtime.set_route(route)
    commands: list[tuple[float, float]] = []
    executor = RouteExecutor(runtime, Planner(), lambda v, w: commands.append((v, w)))
    pose = Pose2D(x=start[0], y=start[1], yaw=-1.1)
    switched_at: tuple[float, float, float] | None = None
    previous_edge = "CURRENT->A"
    dt = 0.05

    for _ in range(500):
        active_route = runtime.active_route()
        if active_route is None:
            break
        executor._follow_route(active_route, pose)
        snapshot = runtime.snapshot()
        current_edge = str(snapshot["currentEdgeId"])
        if current_edge == "A->B" and previous_edge != current_edge:
            switched_at = (pose.x, pose.y, pose.yaw)
        previous_edge = current_edge
        linear, ros_angular = commands[-1]
        map_angular = -ros_angular
        pose.x += linear * math.cos(pose.yaw) * dt
        pose.y += linear * math.sin(pose.yaw) * dt
        pose.yaw = math.atan2(
            math.sin(pose.yaw + map_angular * dt),
            math.cos(pose.yaw + map_angular * dt),
        )

    assert commands[0][0] == pytest.approx(0.0)
    assert switched_at is not None
    switch_x, switch_y, switch_yaw = switched_at
    assert math.hypot(switch_x - boundary[0], switch_y - boundary[1]) < 0.08
    assert abs(math.atan2(math.sin(switch_yaw + math.pi / 2.0), math.cos(switch_yaw + math.pi / 2.0))) < 0.08
    assert runtime.active_route() is None
    assert pose.y < -0.90


def test_continuous_edge_transition_uses_along_track_progress() -> None:
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(0.0, 0.0, 0.0, "A->B"),
            RoutePoint(1.0, 0.0, 0.0, "A->B"),
            RoutePoint(1.5, 0.0, 0.0, "B->C"),
            RoutePoint(2.0, 0.0, 0.0, "B->C"),
        ]
    )
    pose = Pose2D(x=1.02, y=0.12, yaw=0.0)
    projection, _, edge_end = RouteExecutor._project_pose_to_active_edge(
        trajectory,
        pose,
        0,
    )
    control = RouteControlParameters.from_payload({})
    transition = RouteExecutor._edge_transition(
        trajectory,
        pose,
        edge_end,
        control,
    )

    assert transition is not None
    assert transition.requires_stop is False
    assert transition.boundary_distance > 0.04
    assert RouteExecutor._transition_boundary_reached(
        trajectory,
        projection,
        transition,
        0.04,
    )

    early_pose = Pose2D(x=0.97, y=0.0, yaw=0.0)
    early_projection, _, _ = RouteExecutor._project_pose_to_active_edge(
        trajectory,
        early_pose,
        0,
    )
    assert not RouteExecutor._transition_boundary_reached(
        trajectory,
        early_projection,
        transition,
        0.04,
        strict_edge_tracking=True,
    )


def test_direction_change_still_requires_reaching_landmark() -> None:
    trajectory = TrajectoryArray.from_route_points(
        [
            RoutePoint(0.0, 0.0, 0.0, "A->B", motion_direction="forward"),
            RoutePoint(1.0, 0.0, 0.0, "A->B", motion_direction="forward"),
            RoutePoint(1.5, 0.0, math.pi, "B->C", motion_direction="backward"),
            RoutePoint(2.0, 0.0, math.pi, "B->C", motion_direction="backward"),
        ]
    )
    pose = Pose2D(x=1.02, y=0.12, yaw=0.0)
    projection, _, edge_end = RouteExecutor._project_pose_to_active_edge(
        trajectory,
        pose,
        0,
    )
    control = RouteControlParameters.from_payload({})
    transition = RouteExecutor._edge_transition(
        trajectory,
        pose,
        edge_end,
        control,
    )

    assert transition is not None
    assert transition.requires_stop is True
    assert not RouteExecutor._transition_boundary_reached(
        trajectory,
        projection,
        transition,
        0.04,
    )
