"""Readable composition root for fleet runtime capabilities."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from threading import Lock
from time import time as _system_time
from typing import Any

from fleet_manager.core.mapf.fleet.fleet_planner import FleetMapfPlanner
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, MapMetadata
from fleet_manager.core.traffic.collision import FleetCollisionChecker
from fleet_manager.core.traffic.corridors.scheduling.corridor_scheduler import (
    CentralCorridorScheduler,
)
from fleet_manager.manager.commands import FleetManagerCommandMixin
from fleet_manager.manager.events import FleetEvent
from fleet_manager.manager.planning import (
    PlanCommitService,
    PlanningJobRecord,
    PlanningSolverService,
)
from fleet_manager.manager.ports import UnavailableRobotGateway
from fleet_manager.manager.remote_control import (
    FleetManagerRemoteControlMixin,
)
from fleet_manager.manager.robot_lifecycle import (
    FleetManagerRobotLifecycleMixin,
)
from fleet_manager.manager.route_metadata import (
    FleetManagerRouteMetadataMixin,
)
from fleet_manager.manager.snapshots import FleetManagerSnapshotMixin
from fleet_manager.manager.runtime_state import FleetManagerRuntimeStateMixin
from fleet_manager.manager.scheduler import PlanningWorker
from fleet_manager.manager.settings import FleetSettings
from fleet_manager.manager.state import (
    FleetState,
    PlanningSnapshotFactory,
    PlanningState,
    RecoveryState,
    TrafficState,
)
from fleet_manager.manager.movement.motion import FleetMotionRuntimeMixin
from fleet_manager.manager.tasks.dispatch import FleetTaskDispatchMixin
from fleet_manager.manager.tasks.manager import FleetTaskManager
from fleet_manager.manager.tasks.order_admission import OrderAdmissionService
from fleet_manager.manager.tasks.replanning import ReplanningService
from fleet_manager.manager.tasks.rolling_continuation import (
    RollingContinuationService,
)
from fleet_manager.manager.coordination.coordinator import TrafficCoordinatorMixin
from fleet_manager.manager.coordination.planning.planning import TrafficPlanningMixin
from fleet_manager.manager.coordination.routing.routing import TrafficRoutingMixin
from fleet_manager.robot.model import FleetRobot


# Tests and embedded runtimes may replace this clock before manager creation.
time = _system_time


class FleetManagerCore(
    FleetManagerRuntimeStateMixin,
    FleetManagerSnapshotMixin,
    FleetManagerCommandMixin,
    FleetManagerRobotLifecycleMixin,
    FleetManagerRemoteControlMixin,
    FleetManagerRouteMetadataMixin,
    FleetMotionRuntimeMixin,
    TrafficCoordinatorMixin,
    TrafficRoutingMixin,
    TrafficPlanningMixin,
    FleetTaskDispatchMixin,
):
    """Compose fleet state, commands, traffic policy and transport hooks."""

    MAX_SIMULATION_TIME_SCALE = 4.0
    runtime_kind = "core"

    def __init__(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        params: dict[str, Any] | None = None,
        map_dir: Path | None = None,
        map_metadata: MapMetadata | None = None,
        remote_adapter: Any | None = None,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self._clock = clock or time
        wall_now = self._create_core_components(
            landmarks,
            edges,
            params=params,
            map_dir=map_dir,
            map_metadata=map_metadata,
            remote_adapter=remote_adapter,
        )
        self._create_planning_components(wall_now)

        self._create_runtime_state()
        self._planning_input_fingerprint = self._planning_state_fingerprint()

    def _create_core_components(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        *,
        params: dict[str, Any] | None,
        map_dir: Path | None,
        map_metadata: MapMetadata | None,
        remote_adapter: Any | None,
    ) -> float:
        self.fleet_state = FleetState()
        self.traffic_state = TrafficState()
        self.planning_state = PlanningState()
        self.recovery_state = RecoveryState()
        self._runtime_command_executor = None

        self.landmarks = landmarks
        self.edges = edges
        self.params = params or {}
        self.settings = FleetSettings(self.params)
        wall_now = self._wall_time()
        self._simulation_clock_lock = Lock()
        self._simulation_clock = wall_now
        self._simulation_clock_wall_at = wall_now
        self._simulation_time_scale = self._configured_simulation_time_scale()

        self.planner = FleetMapfPlanner(landmarks, edges, params=params)
        self.collision = FleetCollisionChecker(
            params=self.params,
            map_dir=map_dir,
            map_metadata=map_metadata,
        )
        collision = self.collision
        planning_landmarks = self.landmarks

        def rotation_is_clear(
            node: str,
            from_yaw: float,
            to_yaw: float,
        ) -> bool:
            landmark = planning_landmarks.get(node)
            if landmark is None:
                return False
            return collision.rotation_is_clear(
                {
                    "x": landmark.x,
                    "y": landmark.y,
                    "yaw": from_yaw,
                },
                to_yaw,
            )

        self.planner.set_rotation_validator(rotation_is_clear)
        self.robots: dict[str, FleetRobot] = {}
        self.task_manager = FleetTaskManager()
        self.events: list[FleetEvent] = []
        self.obstacles: list[dict[str, float]] = []
        self.obstacle_areas: list[dict[str, float]] = []
        self.active_robot_modes: set[str] | None = None

        self._static_blocked_edges = self._static_map_blocked_edges()
        if self._static_blocked_edges:
            self._event(
                "error",
                f"map audit blocked {len(self._static_blocked_edges)} graph edge(s) "
                "that intersect occupancy",
            )
        self._external_remote_adapter = remote_adapter
        self.remote_adapter = remote_adapter or UnavailableRobotGateway()
        self.robot_gateway = self.remote_adapter
        return wall_now

    def _create_planning_components(self, wall_now: float) -> None:
        self._route_revision_seq = int(wall_now * 1000)
        self._planner_lock = Lock()
        self._dispatch_job_lock = Lock()
        self._dispatch_job: PlanningJobRecord | None = None

        self._planning_snapshot_factory = PlanningSnapshotFactory(
            self.fleet_state,
            self.traffic_state,
        )
        self._planning_solver_service = PlanningSolverService(
            self.planner.plan,
            self._planner_lock,
            accepts_control=True,
        )
        self._plan_commit_service = PlanCommitService(
            lambda: self.planning_revision,
        )
        queue_size = int(self.settings.fleet.number(
            "planning_queue_max_size",
            2,
            minimum=1.0,
        ))
        self._planning_worker = PlanningWorker(
            name="fleet-mapf-worker",
            max_queue_size=min(8, queue_size),
        )
        self._planning_worker.set_completion_consumer(
            self._collect_completed_planning_candidates,
        )

        self._order_admission_service = OrderAdmissionService(
            self.fleet_state,
            self.landmarks,
            lambda: self._now(),
            robot_enabled=lambda robot: self._robot_enabled(robot),
            refresh_remote=lambda robot: self._sync_remote_robot(
                robot,
                self._now(),
                force=False,
            ),
            remote_owner=lambda robot: self._remote_control_owner(robot)[0],
            robot_landmark=lambda robot: self._nearest_lm_for_robot(robot),
        )
        self._rolling_continuation_service = RollingContinuationService(
            self.fleet_state,
            self.planning_state,
            lambda order: self._order_dispatch_retry_interval(order),
            lambda: self._now(),
            robot_enabled=lambda robot: self._robot_enabled(robot),
            active_order_target=lambda order: self._active_order_target(order),
            planning_goal=lambda start, goal, order: self._rolling_planning_goal(
                start,
                goal,
                order,
            ),
            pose_at_trajectory=lambda trajectory, elapsed: self._pose_at_trajectory(
                trajectory,
                elapsed,
            ),
            pose_at_landmark=lambda landmark: self._pose_at_landmark(landmark),
            attach_spatial_route=lambda request, order, start, goal, final: (
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start,
                    goal,
                    final,
                )
            ),
            valid_blockers=lambda name: self._valid_rolling_prefetch_blockers(
                name
            ),
            waits_at_boundary=lambda robot: self._robot_waits_at_rolling_boundary(
                robot
            ),
            prefetch_lead=lambda: self._rolling_prefetch_lead(),
            buffer_policy=lambda: self._rolling_buffer_policy(),
        )
        self._replanning_service = ReplanningService(
            self.fleet_state,
            self.planning_state,
            self.recovery_state,
            lambda robot: self._safe_replan_start_lm(robot),
            lambda: self._now(),
            lambda order: self._order_dispatch_retry_interval(order),
        )
        self._last_async_job_kind = ""

    def _create_runtime_state(self) -> None:
        """Build map-derived traffic state after core services exist."""

        self._runtime_tick_route_clocks: dict[str, float] = {}
        default_speed = self.planner._route_speed({})
        graph = self.planner._traffic_graph(default_speed)
        regions = graph.controlled_region_ids()
        if regions:
            self.traffic_state.controlled_corridor_graph = graph
            self.traffic_state.controlled_corridor_region_bounds = (
                self._build_controlled_corridor_region_bounds(graph)
            )
            self.traffic_state.controlled_corridor_scheduler = (
                CentralCorridorScheduler(
                    regions,
                    config=self._controlled_corridor_scheduler_config(),
                )
            )
        self.traffic_state.traffic_zone_by_lm = (
            self._build_traffic_zone_index()
        )

    def _wall_time(self) -> float:
        """Return the injectable wall clock used by manager state."""

        return float(self._clock())
