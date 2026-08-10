from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any

import pytest

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.movement.motion import FleetMotionRuntimeMixin
from fleet_manager.manager.movement.kinematics import (
    FleetMotionKinematicsMixin,
)
from fleet_manager.manager.movement.lifecycle import (
    FleetRuntimeLifecycleMixin,
)
from fleet_manager.manager.movement.replanning import (
    FleetRuntimeReplanMixin,
)
from fleet_manager.manager.movement.deadlock_retreat import (
    FleetDeadlockRetreatMixin,
)
from fleet_manager.manager.movement.safety import FleetMotionSafetyMixin
from fleet_manager.manager.movement.step import FleetMotionStepMixin


class FleetSettingsStub:
    def __init__(self) -> None:
        self.number_calls: list[tuple[str, float]] = []
        self.integer_calls: list[tuple[str, int]] = []

    def number(
        self,
        name: str,
        default: float,
        **kwargs: Any,
    ) -> float:
        self.number_calls.append((name, default))
        return 0.12

    def integer(
        self,
        name: str,
        default: int,
        **kwargs: Any,
    ) -> int:
        self.integer_calls.append((name, default))
        return 3


class KinematicsHarness(FleetMotionKinematicsMixin):
    def __init__(self) -> None:
        self.settings = SimpleNamespace(fleet=FleetSettingsStub())
        self.nearest_lm_calls = 0
        self.landmarks = {
            "A": SimpleNamespace(name="A", x=0.0, y=0.0),
            "B": SimpleNamespace(name="B", x=1.0, y=0.0),
        }

    @staticmethod
    def _interpolate_angle(
        start: float,
        end: float,
        ratio: float,
    ) -> float:
        delta = math.atan2(
            math.sin(end - start),
            math.cos(end - start),
        )
        return start + (delta * ratio)

    @staticmethod
    def _continuous_collision_step() -> float:
        return 0.2

    def _nearest_lm_for_robot(self, robot: FleetRobot) -> str:
        self.nearest_lm_calls += 1
        return "A"

    @staticmethod
    def _trajectory_segment_index(
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> int:
        for index in range(len(trajectory) - 1):
            if elapsed < float(trajectory[index + 1]["t"]):
                return index
        return max(0, len(trajectory) - 2)

    @staticmethod
    def _trajectory_sample_index_at_or_before(
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> int:
        result = -1
        for index, sample in enumerate(trajectory):
            if float(sample["t"]) <= elapsed:
                result = index
        return result

    @staticmethod
    def _lm_from_wait_segment(
        start: dict[str, Any],
        end: dict[str, Any],
    ) -> str:
        return str(end.get("lm") or start.get("lm") or "")


class StepHarness(FleetMotionStepMixin):
    def __init__(self) -> None:
        self.landmarks = {"A": SimpleNamespace(x=0.0, y=0.0)}

    @staticmethod
    def _pose_from_sample(
        sample: dict[str, Any],
    ) -> dict[str, float]:
        return {
            key: float(sample.get(key, 0.0) or 0.0)
            for key in ("x", "y", "yaw")
        }

    @staticmethod
    def _runtime_replan_lm_tolerance() -> float:
        return 0.1

    @staticmethod
    def _pose_is_at_lm(
        pose: dict[str, Any],
        lm_name: str,
    ) -> bool:
        return (
            lm_name == "A"
            and math.hypot(
                float(pose.get("x", 0.0) or 0.0),
                float(pose.get("y", 0.0) or 0.0),
            )
            <= 0.1
        )


class CircleCollision:
    @staticmethod
    def robot_broadphase_distance() -> float:
        return 0.8

    @staticmethod
    def footprints_overlap(
        first: dict[str, float],
        second: dict[str, float],
    ) -> bool:
        return (
            math.hypot(
                float(first["x"]) - float(second["x"]),
                float(first["y"]) - float(second["y"]),
            )
            < 0.25
        )


class SafetyHarness(
    FleetMotionSafetyMixin,
    FleetMotionKinematicsMixin,
):
    def __init__(self) -> None:
        self.orders: dict[str, FleetOrder] = {}
        self.collision = CircleCollision()

    @staticmethod
    def _interpolate_angle(
        start: float,
        end: float,
        ratio: float,
    ) -> float:
        delta = math.atan2(
            math.sin(end - start),
            math.cos(end - start),
        )
        return start + (delta * ratio)


class ReplanHarness(FleetRuntimeReplanMixin):
    def __init__(self) -> None:
        self.settings = SimpleNamespace(fleet=FleetSettingsStub())
        self._runtime_replans: dict[str, dict[str, Any]] = {}
        self.robots: dict[str, FleetRobot] = {}
        self.orders: dict[str, FleetOrder] = {}

    @staticmethod
    def _safe_replan_start_lm(robot: FleetRobot) -> str:
        return robot.current_lm

    @staticmethod
    def _corridor_clearance_hold_active(
        hold: dict[str, Any],
        robot_name: str,
    ) -> bool:
        return bool(hold.get("active"))

    @staticmethod
    def _is_parked_robot_conflict(reason: str) -> bool:
        return False


def test_facade_composes_motion_runtime_components() -> None:
    assert issubclass(FleetMotionRuntimeMixin, FleetMotionStepMixin)
    assert issubclass(
        FleetMotionRuntimeMixin,
        FleetMotionKinematicsMixin,
    )
    assert issubclass(FleetMotionRuntimeMixin, FleetMotionSafetyMixin)
    assert issubclass(
        FleetMotionRuntimeMixin,
        FleetRuntimeLifecycleMixin,
    )
    assert issubclass(
        FleetRuntimeLifecycleMixin,
        FleetDeadlockRetreatMixin,
    )
    assert issubclass(
        FleetRuntimeLifecycleMixin,
        FleetRuntimeReplanMixin,
    )
    assert (
        FleetMotionRuntimeMixin._advance_runtime
        is FleetMotionStepMixin._advance_runtime
    )


def test_time_step_rejects_zero_duration_teleport() -> None:
    harness = StepHarness()
    robot = FleetRobot(
        "robot",
        "A",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.7},
    )
    robot.trajectory = [
        {"t": 0.0, "x": 0.0, "y": 0.0, "lm": "A"},
    ]

    assert harness._safe_degenerate_endpoint_lm(robot) == "A"

    robot.trajectory.append(
        {"t": 0.0, "x": 0.4, "y": 0.0, "lm": "A"}
    )
    assert harness._safe_degenerate_endpoint_lm(robot) == ""


def test_kinematics_uses_shortest_angle_and_bounded_step() -> None:
    harness = KinematicsHarness()
    pose = harness._interpolate_pose(
        {"x": 0.0, "y": 2.0, "yaw": math.radians(170.0)},
        {"x": 4.0, "y": 6.0, "yaw": math.radians(-170.0)},
        0.5,
    )

    assert pose["x"] == 2.0
    assert pose["y"] == 4.0
    assert abs(abs(pose["yaw"]) - math.pi) < 1e-9
    assert harness._runtime_motion_step() == 0.05
    assert harness._runtime_replan_lm_tolerance() == 0.12
    assert harness.settings.fleet.number_calls == [
        ("runtime_replan_lm_tolerance_m", 0.1)
    ]


def test_kinematics_distinguishes_wait_from_rotation() -> None:
    harness = KinematicsHarness()
    wait = [
        {"t": 0.0, "edgeId": "A->B", "lm": "A"},
        {"t": 1.0, "edgeId": "WAIT@A->B", "lm": "A"},
    ]
    rotation = [
        {"t": 0.0, "edgeId": "A->B", "lm": "A"},
        {
            "t": 1.0,
            "edgeId": "WAIT@ROTATE:A",
            "lm": "A",
        },
    ]

    assert harness._planned_wait_lm_at_trajectory(wait, 0.5) == "A"
    assert (
        harness._planned_wait_lm_at_trajectory(rotation, 0.5)
        == ""
    )


def test_safe_replan_start_uses_current_landmark_before_full_scan() -> None:
    harness = KinematicsHarness()
    robot = FleetRobot(
        "robot",
        "A",
        pose={"x": 0.02, "y": 0.0, "yaw": 0.0},
    )

    assert harness._safe_replan_start_lm(robot) == "A"
    assert harness.nearest_lm_calls == 0

    robot.current_lm = "B"
    assert harness._safe_replan_start_lm(robot) == "A"
    assert harness.nearest_lm_calls == 1


def test_safety_snapshot_restores_robot_and_order_atomically() -> None:
    harness = SafetyHarness()
    order = FleetOrder(
        "order",
        "B",
        status="EXECUTING",
        assigned_robot="robot",
        route_nodes=["A", "B"],
    )
    harness.orders[order.order_id] = order
    robot = FleetRobot(
        "robot",
        "A",
        target_lm="B",
        status="MOVING",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[{"t": 0.0}, {"t": 2.0}],
        plan_nodes=["A", "B"],
        active_order_id=order.order_id,
        route_revision=4,
    )
    robot.wait_for_robot = "peer"
    snapshot = harness._runtime_safety_snapshot(robot)

    robot.pose = {"x": 9.0, "y": 9.0, "yaw": 1.0}
    robot.current_lm = "B"
    robot.status = "BLOCKED"
    robot.route_clock = 8.0
    robot.route_revision = 5
    order.status = "FAILED"
    order.route_nodes = []

    harness._restore_runtime_safety_snapshot(
        robot,
        snapshot,
        now=12.5,
    )

    assert robot.pose == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert robot.current_lm == "A"
    assert robot.status == "MOVING"
    assert robot.route_clock == 0.0
    assert robot.route_revision == 4
    assert robot.wait_for_robot == "peer"
    assert robot.last_tick_at == 12.5
    assert robot.updated_at == 12.5
    assert robot.trajectory_dirty
    assert order.status == "EXECUTING"
    assert order.route_nodes == ["A", "B"]


def test_safety_detects_crossing_sweeps_but_rejects_far_pairs() -> None:
    harness = SafetyHarness()
    crossing = harness._swept_footprints_overlap(
        {"x": -1.0, "y": 0.0, "yaw": 0.0},
        {"x": 1.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": -1.0, "yaw": math.pi / 2.0},
        {"x": 0.0, "y": 1.0, "yaw": math.pi / 2.0},
    )
    far = harness._swept_footprints_overlap(
        {"x": -1.0, "y": 0.0, "yaw": 0.0},
        {"x": 1.0, "y": 0.0, "yaw": 0.0},
        {"x": -1.0, "y": 3.0, "yaw": 0.0},
        {"x": 1.0, "y": 3.0, "yaw": 0.0},
    )

    assert crossing
    assert not far


def test_retreat_component_clears_complete_transaction_state() -> None:
    robot = FleetRobot("robot", "A")
    robot.retreat_target_clock = 1.5
    robot.retreat_target_lm = "B"
    robot.retreat_blocked_edges = [("A", "B")]
    robot.retreat_blocker_signatures = [("peer", "B", 3)]
    robot.retreat_corridor_hold = {"resource": "corridor"}

    FleetDeadlockRetreatMixin()._clear_deadlock_retreat(robot)

    assert robot.retreat_target_clock is None
    assert robot.retreat_target_lm == ""
    assert robot.retreat_blocked_edges == []
    assert robot.retreat_blocker_signatures == []
    assert robot.retreat_corridor_hold is None


def test_replan_component_validates_transaction_identity() -> None:
    harness = ReplanHarness()
    robot = FleetRobot(
        "robot",
        "A",
        active_order_id="order",
        route_revision=3,
        route_clock=1.25,
    )
    order = FleetOrder("order", "B", status="PLANNING")
    harness.robots[robot.name] = robot
    harness.orders[order.order_id] = order
    harness._runtime_replans[robot.name] = {
        "order_id": order.order_id,
        "route_revision": 3,
        "route_clock": 1.25,
        "start_lm": "A",
        "stage": "planning",
    }

    assert harness._runtime_replan_holds_robot(robot)

    robot.route_revision = 4
    assert not harness._runtime_replan_holds_robot(robot)
    assert robot.name not in harness._runtime_replans
    assert harness._superseded_runtime_replan_limit() == 3
    assert harness.settings.fleet.integer_calls == [
        ("max_superseded_runtime_replans", 2)
    ]


@pytest.mark.parametrize(
    ("reason", "expected"),
    [
        ("traffic admission timeout", True),
        ("blocked edge A->B", True),
        ("temporary temporal reservation", False),
    ],
)
def test_replan_component_classifies_spatial_failures(
    reason: str,
    expected: bool,
) -> None:
    assert ReplanHarness()._reason_requires_spatial_replan(reason) is expected
