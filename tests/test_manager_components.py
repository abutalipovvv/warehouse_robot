from __future__ import annotations

import math
from threading import Lock

import pytest

import fleet_manager.manager.manager as manager_module
from fleet_manager.manager.manager import FleetManagerCore
from fleet_manager.manager.commands import FleetManagerCommandMixin
from fleet_manager.manager.remote_control import (
    FleetManagerRemoteControlMixin,
)
from fleet_manager.manager.robot_lifecycle import (
    FleetManagerRobotLifecycleMixin,
)
from fleet_manager.manager.route_metadata import (
    FleetManagerRouteMetadataMixin,
)
from fleet_manager.manager.snapshots import (
    FleetManagerSnapshotMixin,
)
from fleet_manager.manager.runtime_state import (
    FleetManagerRuntimeStateMixin,
)
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.movement.motion import FleetMotionRuntimeMixin
from fleet_manager.core.mapping.maps.models import (
    GraphEdge,
    Landmark,
    WorldPoint,
)
from fleet_manager.manager.coordination.coordinator import (
    TrafficCoordinatorMixin,
)
from fleet_manager.manager.coordination.planning.planning import TrafficPlanningMixin
from fleet_manager.manager.coordination.routing.routing import TrafficRoutingMixin
from fleet_manager.manager.tasks.dispatch import FleetTaskDispatchMixin


def _landmarks() -> dict[str, Landmark]:
    return {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }


def _edges(
    landmarks: dict[str, Landmark],
) -> list[GraphEdge]:
    return [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                WorldPoint(
                    landmarks[source].x,
                    landmarks[source].y,
                ),
                WorldPoint(
                    landmarks[target].x,
                    landmarks[target].y,
                ),
            ),
            properties={"direction": 2},
        )
        for source, target in (("A", "B"), ("B", "A"))
    ]


@pytest.fixture
def manager(
    monkeypatch: pytest.MonkeyPatch,
) -> FleetManagerCore:
    monkeypatch.setattr(manager_module, "time", lambda: 1_000.0)
    landmarks = _landmarks()
    instance = FleetManagerCore(
        landmarks,
        _edges(landmarks),
        params={
            "fleet": {
                "controlled_corridors_enabled": False,
                "traffic_zone_control_enabled": False,
            }
        },
    )
    try:
        yield instance
    finally:
        instance.close()


def test_facade_composes_capabilities_before_original_policy_mro() -> None:
    capabilities = (
        FleetManagerRuntimeStateMixin,
        FleetManagerSnapshotMixin,
        FleetManagerCommandMixin,
        FleetManagerRobotLifecycleMixin,
        FleetManagerRemoteControlMixin,
        FleetManagerRouteMetadataMixin,
    )
    for capability in capabilities:
        assert issubclass(FleetManagerCore, capability)

    mro = FleetManagerCore.__mro__
    old_policy = (
        FleetMotionRuntimeMixin,
        TrafficCoordinatorMixin,
        TrafficRoutingMixin,
        TrafficPlanningMixin,
        FleetTaskDispatchMixin,
    )
    positions = [mro.index(base) for base in old_policy]
    assert positions == sorted(positions)
    assert (
        FleetManagerCore._state_snapshot
        is FleetManagerSnapshotMixin._state_snapshot
    )
    assert (
        FleetManagerCore._apply_simulated_route_metadata
        is FleetManagerRouteMetadataMixin._apply_simulated_route_metadata
    )


def test_runtime_state_owns_distinct_locks_and_state_groups(
    manager: FleetManagerCore,
) -> None:
    assert isinstance(manager._simulation_clock_lock, type(Lock()))
    assert isinstance(manager._planner_lock, type(Lock()))
    assert isinstance(manager._dispatch_job_lock, type(Lock()))
    assert len(
        {
            id(manager._simulation_clock_lock),
            id(manager._planner_lock),
            id(manager._dispatch_job_lock),
        }
    ) == 3
    assert manager._simulation_clock == 1_000.0
    assert manager._simulation_clock_wall_at == 1_000.0

    expected_state_groups = (
        "_runtime_replans",
        "_rolling_prefetch_retry_at",
        "_stationary_order_retry_state",
        "_dispatch_conflict_dependencies",
        "_active_wait_cycles",
        "_controlled_corridor_wait_since",
        "_controlled_corridor_entry_cache",
        "_traffic_zone_wait_since",
        "_runtime_tick_route_clocks",
        "traffic_metrics",
    )
    for name in expected_state_groups:
        assert hasattr(manager, name)
        assert isinstance(getattr(manager, name), dict)

    task_manager = manager.task_manager
    replacement = {
        "order": FleetOrder("order", "B", vehicle="robot")
    }
    manager.orders = replacement
    assert manager.task_manager is task_manager
    assert manager.orders == replacement


def test_snapshot_preserves_json_and_robot_insertion_order(
    manager: FleetManagerCore,
) -> None:
    first = FleetRobot(
        "second",
        "A",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    second = FleetRobot(
        "first",
        "B",
        pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
    )
    second.trajectory = [
        {"t": 0.0, "x": 1.0, "y": 0.0, "yaw": math.pi},
        {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": math.pi},
    ]
    second.trajectory_dirty = True
    second.route_revision = 7
    manager.robots = {first.name: first, second.name: second}
    manager.orders["assigned"] = FleetOrder(
        "assigned",
        "A",
        vehicle=second.name,
        assigned_robot=second.name,
        status="QUEUED",
    )
    manager._event("info", "snapshot-order")

    snapshot = manager.snapshot()

    assert list(snapshot) == [
        "ok",
        "robots",
        "simulationTimeScale",
        "simulationTimeScaleMax",
        "events",
        "obstacles",
        "obstacleAreas",
        "orders",
        "traffic",
        "lastRuntimeSafetyRollback",
        "trafficFlow",
    ]
    assert [item["name"] for item in snapshot["robots"]] == [
        "second",
        "first",
    ]
    assert snapshot["events"][-1]["message"] == "snapshot-order"
    assigned = snapshot["robots"][1]
    assert list(assigned)[-4:] == [
        "assignedOrderId",
        "assignedOrderStatus",
        "assignedOrderTargetLm",
        "orderQueueDepth",
    ]

    changed = manager.stream_tick({"first": 6})
    assert changed["robots"][1]["trajectory"] == second.trajectory
    assert not second.trajectory_dirty
    unchanged = manager.stream_tick({"first": 7})
    assert unchanged["robots"][1]["trajectory"] == []


def test_order_command_keeps_response_field_order(
    manager: FleetManagerCore,
) -> None:
    manager.robots["robot"] = FleetRobot(
        "robot",
        "A",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )

    response = manager.set_order(
        {
            "id": "order",
            "vehicle": "robot",
            "targetLm": "B",
        },
        dispatch=False,
    )

    assert list(response) == [
        "ok",
        "order",
        "queuedOrders",
        "orders",
        "state",
    ]
    assert response["order"]["id"] == "order"
    assert manager.orders["order"].status == "QUEUED"
    assert manager.events[-1].message == "order queued: order robot->B"


def test_robot_lifecycle_resets_and_removes_ephemeral_state(
    manager: FleetManagerCore,
) -> None:
    added = manager.add_robot(
        {
            "name": "robot",
            "spawnLm": "A",
            "mode": "simulated",
        }
    )
    robot = manager.robots["robot"]
    manager._runtime_replans[robot.name] = {"stage": "queued"}
    robot.target_lm = "B"
    robot.trajectory = [{"t": 0.0}, {"t": 1.0}]

    updated = manager.update_robot(
        {
            "name": robot.name,
            "status": "IDLE",
            "targetLm": "",
        },
        include_state=False,
    )
    removed = manager.remove_robot({"name": robot.name})

    assert added["robot"]["currentLm"] == "A"
    assert updated["robot"]["status"] == "IDLE"
    assert updated["robot"]["trajectory"] == []
    assert removed["removed"]
    assert robot.name not in manager._runtime_replans
    assert robot.name not in manager.robots


def test_remote_identity_and_takeover_are_transport_stable(
    manager: FleetManagerCore,
) -> None:
    identity = {
        "identity": {
            "robotId": "remote-7",
        }
    }
    status = {
        "robot": {
            "pose": {
                "x": "1.25",
                "y": "-0.5",
                "angle": "0.75",
            }
        }
    }
    endpoint = "grpc://robot.example:50051"

    assert (
        manager._remote_robot_name(
            identity,
            manager._remote_status_robot(status),
            endpoint,
        )
        == "remote-7"
    )
    assert manager._remote_pose_from_status(status["robot"]) == {
        "x": 1.25,
        "y": -0.5,
        "yaw": 0.75,
    }

    robot = FleetRobot(
        "remote-7",
        "A",
        mode="remote",
        base_url=endpoint,
    )
    manager.robots[robot.name] = robot
    assert manager.note_external_control_takeover(
        "robot.example:50051",
        owner_id="operator",
        owner_name="Alice",
    )
    assert robot.status == "MANUAL"
    assert robot.remote_status["controlOwner"] == "operator"
    assert robot.last_reason.endswith("Alice")


def test_simulation_route_metadata_commits_revision_and_preview(
    manager: FleetManagerCore,
) -> None:
    robot = FleetRobot(
        "robot",
        "A",
        target_lm="B",
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    order = FleetOrder(
        "order",
        "B",
        vehicle=robot.name,
        assigned_robot=robot.name,
        status="EXECUTING",
        spatial_route_nodes=["A", "B"],
    )
    manager.robots[robot.name] = robot
    manager.orders[order.order_id] = order
    plan = {
        "goalLm": "B",
        "finalGoalLm": "B",
        "nodes": ["A", "B"],
        "trajectory": [
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "lm": "A",
                "edgeId": "A->B",
            },
            {
                "t": 1.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "lm": "B",
                "edgeId": "A->B",
            },
        ],
    }

    manager._apply_simulated_route_metadata(
        robot,
        order,
        plan,
        now=1_001.0,
    )

    assert robot.route_revision > 0
    assert robot.route_chunk_index == 0
    assert robot.route_chunk_goal_lm == "B"
    assert robot.route_final_lm == "B"
    assert robot.target_lm == "B"
    assert robot.plan_nodes == ["A", "B"]
    assert [point["phase"] for point in robot.route_preview] == [
        "committed",
        "committed",
    ]
    assert robot.route_preview_dirty
    assert robot.has_executed_route


def test_pose_sampling_interpolates_shortest_yaw(
    manager: FleetManagerCore,
) -> None:
    trajectory = [
        {
            "t": 0.0,
            "x": 0.0,
            "y": 0.0,
            "yaw": math.radians(170.0),
        },
        {
            "t": 2.0,
            "x": 2.0,
            "y": 4.0,
            "yaw": math.radians(-170.0),
        },
    ]

    pose = manager._pose_at_trajectory(trajectory, 1.0)

    assert pose is not None
    assert pose["x"] == 1.0
    assert pose["y"] == 2.0
    assert abs(abs(pose["yaw"]) - math.pi) < 1e-9
