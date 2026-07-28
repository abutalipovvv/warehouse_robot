from __future__ import annotations

import math
from pathlib import Path
from threading import Lock
from time import time
from typing import Any
from urllib.parse import urlparse

from fleet_manager.core.geometry.collision import FleetCollisionChecker
from fleet_manager.core.mapf.fleet_planner import FleetMapfPlanner
from fleet_manager.core.constants import (
    EXTERNAL_CONTROL_PAUSE_PREFIX,
    FLEET_CONTROL_OWNER_ID,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.models import FleetEvent, FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import (
    GraphEdge,
    Landmark,
    MapMetadata,
)
from fleet_manager.runtime.grpc.api.contracts import DEFAULT_GRPC_PORT, normalize_grpc_endpoint
from fleet_manager.core.motion import FleetMotionRuntimeMixin
from fleet_manager.core.tasks.dispatch import FleetTaskDispatchMixin
from fleet_manager.core.tasks.manager import FleetTaskManager
from fleet_manager.core.traffic.coordinator import TrafficCoordinatorMixin
from fleet_manager.core.traffic.corridor_scheduler import (
    CentralCorridorScheduler,
    CorridorSchedule,
    CorridorSchedulerConfig,
)
from fleet_manager.core.traffic.planning import TrafficPlanningMixin
from fleet_manager.core.traffic.routing import TrafficRoutingMixin
from fleet_manager.runtime.gateways.base import UnavailableRobotGateway


class FleetManagerCore(
    FleetMotionRuntimeMixin,
    TrafficCoordinatorMixin,
    TrafficRoutingMixin,
    TrafficPlanningMixin,
    FleetTaskDispatchMixin,
):
    """Shared fleet policy, MAPF, task and traffic runtime.

    Transport-specific subclasses select either simulated execution or the
    production gRPC robot gateway. The core owns decisions, never UI concerns.
    """

    MAX_SIMULATION_TIME_SCALE = 4.0
    runtime_kind = "core"

    @property
    def orders(self) -> dict[str, FleetOrder]:
        return self.task_manager.orders

    @orders.setter
    def orders(self, value: dict[str, FleetOrder]) -> None:
        if hasattr(self, "task_manager"):
            self.task_manager.replace_storage(value)
        else:  # Defensive compatibility for unusual subclass initializers.
            self.task_manager = FleetTaskManager(value)

    def __init__(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        params: dict[str, Any] | None = None,
        map_dir: Path | None = None,
        map_metadata: MapMetadata | None = None,
        remote_adapter: Any | None = None,
    ) -> None:
        self.landmarks = landmarks
        self.edges = edges
        self.params = params or {}
        wall_now = time()
        self._simulation_clock_lock = Lock()
        self._simulation_clock = wall_now
        self._simulation_clock_wall_at = wall_now
        self._simulation_time_scale = self._configured_simulation_time_scale()
        self.planner = FleetMapfPlanner(landmarks, edges, params=params)
        self.robots: dict[str, FleetRobot] = {}
        self.task_manager = FleetTaskManager()
        self.events: list[FleetEvent] = []
        self.obstacles: list[dict[str, float]] = []
        self.obstacle_areas: list[dict[str, float]] = []
        self.active_robot_modes: set[str] | None = None
        self.collision = FleetCollisionChecker(
            params=self.params,
            map_dir=map_dir,
            map_metadata=map_metadata,
        )
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
        self._route_revision_seq = int(wall_now * 1000)
        self._planner_lock = Lock()
        self._dispatch_job_lock = Lock()
        self._dispatch_job: dict[str, Any] | None = None
        self._last_async_job_kind = ""
        # Runtime detours are transactional.  An executing robot keeps its
        # committed timeline while the shared background planner prepares a
        # replacement from the graph-safe LM at which it is holding.  The
        # state is separate from ``pending_route`` because that field belongs
        # to rolling *future* chunks and may be consumed at a chunk boundary.
        self._runtime_replans: dict[str, dict[str, Any]] = {}
        self._rolling_prefetch_retry_at: dict[str, float] = {}
        # Continuations share one planner worker with new order dispatch.
        # Preserve admission age and the last service turn explicitly so a
        # continuous stream of fresh robots cannot starve a stopped holder.
        self._rolling_prefetch_eligible_since: dict[str, float] = {}
        self._rolling_prefetch_last_attempt_at: dict[str, float] = {}
        # An impossible departure around the same route-less parked bodies
        # must not consume the sole planner worker forever. The dispatcher
        # records the exact blocker occupancy and retries only after it changes.
        self._stationary_order_retry_state: dict[str, dict[str, Any]] = {}
        # A moving committed robot can also make one fresh departure
        # temporarily impossible.  Keep the exact structured dependency so
        # that only that requester sleeps until the blocker reaches another
        # graph state; never let its failure contaminate unrelated orders.
        self._dispatch_conflict_dependencies: dict[
            str,
            dict[str, Any],
        ] = {}
        # A terminal robot may be the only physical obstacle left on an
        # otherwise viable route.  Traffic-clearance moves are normal internal
        # orders, while this compact index supplies deduplication/cooldown and
        # lets their hidden order records be pruned after completion.
        self._stationary_clearance_relocations: dict[str, dict[str, Any]] = {}
        # A continuation normally remains a cheap one/two-robot SIPP request.
        # Failed boundary holders rotate through cheap pair attempts, then the
        # fast SIPP recovery wave grows to include the coupled stopped group.
        # Exponential CBS remains capped to small local groups.
        self._rolling_prefetch_failures: dict[str, int] = {}
        # Low-level SIPP and the continuous footprint validator know the exact
        # robot which rejected a rolling continuation.  Preserve that evidence
        # across the short retry boundary so the next recovery request contains
        # the real coupled component instead of rediscovering the same
        # singleton conflict.  Every record carries route revisions and is
        # discarded as soon as either participant advances.
        self._rolling_prefetch_blockers: dict[str, dict[str, Any]] = {}
        # A fully stopped rolling cohort is released in dependency order. If
        # its dependency graph is cyclic, one robot is sent to a free waiting
        # pocket. Do not retry a pocket that already failed while every robot
        # is still at the exact same route revision.
        self._rolling_vacancy_recovery_signature: tuple[
            tuple[str, str, int], ...
        ] = ()
        self._rolling_vacancy_recovery_blacklist: set[
            tuple[tuple[tuple[str, str, int], ...], str, str]
        ] = set()
        # A queued stationary departure uses the same safe-pocket concept, but
        # it is a different liveness episode from an exhausted rolling cohort.
        # Keep its failed pockets independent so alternating recovery classes
        # cannot erase one another's bounded retry history.
        self._commanded_sink_vacancy_signatures: dict[
            str,
            tuple[tuple[str, str, int], ...],
        ] = {}
        self._commanded_sink_vacancy_blacklist: set[
            tuple[
                str,
                tuple[tuple[str, str, int], ...],
                str,
                str,
            ]
        ] = set()
        self._coupled_replan_last_attempt: dict[tuple[str, ...], float] = {}
        self._coupled_replan_failures: dict[tuple[str, ...], int] = {}
        self._active_wait_cycles: dict[tuple[str, ...], float] = {}
        # Runtime collision checks run at 10 Hz, while a right-of-way lease
        # intentionally lasts seconds.  Remember the last arbitration per
        # component so a stalled lease can escalate without being re-granted
        # and logged on every physics frame.
        self._wait_cycle_last_arbitration: dict[tuple[str, ...], float] = {}
        # One right-of-way nudge is useful only once for an unchanged physical
        # cycle.  If its lease expires without graph progress, issuing the same
        # winner again merely makes the robots twitch and inflates metrics.
        # Route revisions are part of the signature, so a real replan gets one
        # fresh arbitration attempt while an unchanged snapshot proceeds to
        # CBS/corridor evacuation instead.
        self._wait_cycle_grant_signatures: dict[
            tuple[str, ...],
            tuple[tuple[str, str, str, int], ...],
        ] = {}
        # A cycle can briefly disappear while a transactional detour is being
        # planned, then reappear at the exact same graph landmarks when the
        # retained route is retried.  ``_active_wait_cycles`` deliberately
        # follows the live wait-for graph and therefore cannot debounce that
        # cross-state transition.  Keep a short-lived spatial recovery
        # signature separately so the unchanged component cannot enqueue the
        # same retreat/detour (and inflate its metrics) on every physics tick.
        self._wait_cycle_recovery_attempts: dict[
            tuple[str, str, tuple[tuple[str, str, str, str], ...]],
            float,
        ] = {}
        # A changing portal tail must not create a new evacuation transaction
        # every few seconds for the same physical corridor owner.  The key is
        # stable across transient wait-graph membership changes and remains
        # latched until that owner physically clears or changes task/route.
        self._controlled_corridor_recovery_latches: dict[
            tuple[tuple[str, ...], str, str, int],
            str,
        ] = {}
        # Populated only while a physics tick is advancing. All pairwise
        # predictions must use the same clocks from the start of that tick;
        # otherwise iteration order makes an already-updated peer appear one
        # extra step ahead and can admit a crossing that is rolled back later.
        self._runtime_tick_route_clocks: dict[str, float] = {}
        default_speed = self.planner._route_speed({})
        controlled_graph = self.planner._traffic_graph(default_speed)
        self._controlled_corridor_graph = (
            controlled_graph
            if controlled_graph.controlled_region_ids()
            else None
        )
        self._controlled_corridor_region_bounds = (
            self._build_controlled_corridor_region_bounds(
                controlled_graph
            )
            if self._controlled_corridor_graph is not None
            else {}
        )
        scheduler_regions = (
            controlled_graph.controlled_region_ids()
            if self._controlled_corridor_graph is not None
            else ()
        )
        self._controlled_corridor_scheduler = (
            CentralCorridorScheduler(
                scheduler_regions,
                config=self._controlled_corridor_scheduler_config(),
            )
            if scheduler_regions
            else None
        )
        self._controlled_corridor_schedule: CorridorSchedule | None = None
        self._controlled_corridor_wait_since: dict[
            tuple[str, str, int, str],
            float,
        ] = {}
        self._controlled_corridor_leases: dict[str, tuple[str, float]] = {}
        # One admission covers the complete no-wait passage between two safe
        # holding LMs. A passage can contain several authored rectangle zones;
        # acquiring them independently recreates hold-and-wait deadlocks at
        # zone-to-zone edges.
        self._controlled_corridor_passages: dict[str, dict[str, Any]] = {}
        # A rolling continuation exists before its trajectory is committed.
        # Register its first authored-corridor passage here so the central
        # calendar can issue a slot before SIPP chooses temporal waits.
        self._controlled_corridor_prefetch_intents: dict[
            str,
            dict[str, Any],
        ] = {}
        # Runtime admission checks ask for the same next authored passage
        # several times during one physics slice. Keep only the latest result
        # for each live robot; any trajectory, clock, pose or lookahead change
        # invalidates it immediately.
        self._controlled_corridor_entry_cache: dict[
            str,
            tuple[
                list[dict[str, Any]],
                tuple[Any, ...],
                dict[str, Any] | None,
            ],
        ] = {}
        # Approach-only corridor chunks form a deterministic queue on distinct
        # graph-safe LMs. A reservation lives only for the current route
        # revision, so a completed/replaced chunk releases its queue cell
        # without an accumulating history.
        self._controlled_corridor_approach_holds: dict[
            str,
            dict[str, Any],
        ] = {}
        self._controlled_corridor_winners: dict[str, str] = {}
        self._controlled_corridor_occupancy: dict[str, list[str]] = {}
        self._controlled_corridor_queues: dict[str, list[str]] = {}
        self._controlled_corridor_blockers: dict[str, str] = {}
        self._controlled_corridor_tick_now = 0.0
        self._traffic_zone_by_lm = self._build_traffic_zone_index()
        self._traffic_zone_wait_since: dict[tuple[str, str], float] = {}
        self._traffic_zone_leases: dict[tuple[str, str], float] = {}
        self._traffic_zone_phase: dict[str, tuple[str, float]] = {}
        self._traffic_zone_emergency_until: dict[str, float] = {}
        self._traffic_zone_winners: dict[str, str] = {}
        self._traffic_zone_demand: dict[str, int] = {}
        self._traffic_zone_occupancy: dict[str, int] = {}
        self._traffic_zone_queues: dict[str, list[str]] = {}
        self._traffic_zone_tick_now = 0.0
        self.traffic_metrics: dict[str, int] = {
            "waitCyclesDetected": 0,
            "waitCyclesResolved": 0,
            "cycleReplans": 0,
            "coupledReplansStarted": 0,
            "coupledReplansSucceeded": 0,
            "coupledReplansFailed": 0,
            "priorityGrants": 0,
            "runtimeSafetyRollbacks": 0,
            "corridorAdmissionWaits": 0,
            "corridorAdmissionsGranted": 0,
            "zoneAdmissionWaits": 0,
            "zoneAdmissionsGranted": 0,
        }
        # Structured evidence for the most recent exceptional safety
        # transaction.  Event messages are intentionally bounded and can be
        # displaced quickly by planner traffic, while this single record is
        # constant-space and survives until the benchmark is reset.
        self._last_runtime_safety_rollback: dict[str, Any] | None = None

    def _build_controlled_corridor_region_bounds(
        self,
        graph: Any,
    ) -> dict[str, tuple[float, float, float, float]]:
        """Approximate authored rectangle bounds from annotated vertices."""
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = fleet.get(
                "controlled_corridor_footprint_zone_padding_m",
                0.2,
            )
            padding = max(
                0.0,
                float(0.2 if configured is None else configured),
            )
        except (TypeError, ValueError):
            padding = 0.2
        points: dict[str, list[tuple[float, float]]] = {}
        for vertex in graph.vertices.values():
            for region_id in vertex.controlled_region_ids:
                points.setdefault(str(region_id), []).append(
                    (float(vertex.x), float(vertex.y))
                )
        return {
            region_id: (
                min(point[0] for point in region_points) - padding,
                min(point[1] for point in region_points) - padding,
                max(point[0] for point in region_points) + padding,
                max(point[1] for point in region_points) + padding,
            )
            for region_id, region_points in points.items()
            if region_points
        }

    def _next_route_revision(self) -> int:
        """Allocate a revision shared by simulation and gRPC routes."""
        now_ms = int(self._now() * 1000)
        self._route_revision_seq = max(self._route_revision_seq + 1, now_ms)
        return self._route_revision_seq

    def _cancel_remote_route(self, robot: FleetRobot, reason: str) -> bool:
        """Cancel a transport route and report whether it is safe to retire.

        Simulation has no independent transport state, so cancellation is
        acknowledged immediately.  Remote runtimes override this hook and
        must return ``False`` while the physical robot may still be executing
        the route.
        """
        del robot, reason
        return True

    def _stop_remote_robot(self, robot: FleetRobot) -> None:
        """No-op transport hook overridden by the gRPC runtime."""
        del robot

    def set_active_robot_modes(self, modes: set[str] | list[str] | tuple[str, ...] | None) -> None:
        if modes is None:
            self.active_robot_modes = None
            return
        clean_modes = {str(mode or "").strip().lower() for mode in modes if str(mode or "").strip()}
        self.active_robot_modes = clean_modes or None

    def reset_traffic_flow_state(self) -> None:
        if self._controlled_corridor_scheduler is not None:
            self._controlled_corridor_scheduler.reset()
        self._controlled_corridor_schedule = None
        self._controlled_corridor_wait_since.clear()
        self._controlled_corridor_leases.clear()
        self._controlled_corridor_passages.clear()
        self._controlled_corridor_prefetch_intents.clear()
        self._controlled_corridor_entry_cache.clear()
        self._controlled_corridor_approach_holds.clear()
        self._controlled_corridor_winners.clear()
        self._controlled_corridor_occupancy.clear()
        self._controlled_corridor_queues.clear()
        self._controlled_corridor_blockers.clear()
        self._controlled_corridor_tick_now = 0.0
        self._traffic_zone_wait_since.clear()
        self._traffic_zone_leases.clear()
        self._traffic_zone_phase.clear()
        self._traffic_zone_emergency_until.clear()
        self._traffic_zone_winners.clear()
        self._traffic_zone_demand.clear()
        self._traffic_zone_occupancy.clear()
        self._traffic_zone_queues.clear()
        self._traffic_zone_tick_now = 0.0

    def _controlled_corridor_scheduler_config(
        self,
    ) -> CorridorSchedulerConfig:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}

        def value(name: str, default: float) -> float:
            try:
                configured = fleet.get(name, default)
                return max(
                    0.0,
                    float(default if configured is None else configured),
                )
            except (TypeError, ValueError):
                return default

        max_direction_batch = max(
            1,
            int(
                value(
                    "controlled_corridor_max_direction_batch",
                    3.0,
                )
            ),
        )
        max_adaptive_direction_batch = max(
            max_direction_batch,
            int(
                value(
                    "controlled_corridor_max_adaptive_direction_batch",
                    12.0,
                )
            ),
        )
        return CorridorSchedulerConfig(
            horizon_sec=max(
                1.0,
                value("controlled_corridor_schedule_horizon_sec", 120.0),
            ),
            commit_horizon_sec=value(
                "controlled_corridor_commit_horizon_sec",
                2.0,
            ),
            headway_sec=max(
                0.05,
                value(
                    "controlled_corridor_slot_headway_sec",
                    1.0,
                ),
            ),
            direction_change_sec=value(
                "controlled_corridor_direction_change_sec",
                0.9,
            ),
            starvation_sec=max(
                1.0,
                value("controlled_corridor_starvation_sec", 8.0),
            ),
            direction_switch_cost_sec=value(
                "controlled_corridor_direction_switch_cost_sec",
                1.5,
            ),
            priority_cost_sec=value(
                "controlled_corridor_priority_cost_sec",
                0.05,
            ),
            wait_age_cost_sec=value(
                "controlled_corridor_wait_age_cost_sec",
                0.03,
            ),
            tentative_change_penalty_sec=value(
                "controlled_corridor_schedule_hysteresis_sec",
                2.0,
            ),
            occupancy_recheck_sec=max(
                0.1,
                value("controlled_corridor_occupancy_recheck_sec", 0.1),
            ),
            starvation_age_quantum_sec=max(
                0.1,
                value(
                    "controlled_corridor_starvation_age_quantum_sec",
                    2.0,
                ),
            ),
            max_direction_batch=max_direction_batch,
            max_adaptive_direction_batch=max_adaptive_direction_batch,
            phase_amortization_sec=max(
                0.05,
                value(
                    "controlled_corridor_phase_amortization_sec",
                    4.0,
                ),
            ),
            max_phase_extension_sec=value(
                "controlled_corridor_max_phase_extension_sec",
                30.0,
            ),
        )

    def _clear_rolling_prefetch_state(self, robot_name: str) -> None:
        """Forget continuation backoff once a robot receives a fresh route."""
        name = str(robot_name or "").strip()
        if not name:
            return
        self._rolling_prefetch_retry_at.pop(name, None)
        self._rolling_prefetch_failures.pop(name, None)
        self._rolling_prefetch_blockers.pop(name, None)
        self._controlled_corridor_prefetch_intents.pop(name, None)
        for requester_name, evidence in list(
            self._rolling_prefetch_blockers.items()
        ):
            blockers = evidence.get("blockers")
            if not isinstance(blockers, dict) or name not in blockers:
                continue
            blockers.pop(name, None)
            if not blockers:
                self._rolling_prefetch_blockers.pop(requester_name, None)
        self._rolling_prefetch_eligible_since.pop(name, None)
        self._rolling_prefetch_last_attempt_at.pop(name, None)
        robot = self.robots.get(name)
        if robot is not None:
            robot.rolling_boundary_since = None

    def _terminal_order_history_limit(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = int(
                fleet.get("terminal_order_history_limit", 120) or 120
            )
        except (TypeError, ValueError):
            configured = 120
        return max(0, min(5000, configured))

    def _prune_terminal_order_history(self) -> tuple[str, ...]:
        """Bound completed task storage before admitting more lifelong work.

        Pruning on order admission is deliberate: benchmark accounting pumps
        and external clients have already observed the previous completions,
        while a just-completed fleet wave is not modified mid-snapshot.
        """
        removed = self.task_manager.prune_terminal_history(
            self._terminal_order_history_limit(),
        )
        for order_id in removed:
            self._stationary_order_retry_state.pop(order_id, None)
        return removed

    def clear_robot_ephemeral_state(self, robot_name: str) -> None:
        """Remove name-keyed arbitration state for a removed/respawned robot."""
        name = str(robot_name or "").strip()
        if not name:
            return
        self._clear_rolling_prefetch_state(name)
        self._runtime_replans.pop(name, None)
        self._stationary_clearance_relocations.pop(name, None)
        self._runtime_tick_route_clocks.pop(name, None)
        for order_id in list(self._stationary_order_retry_state):
            order = self.orders.get(order_id)
            if order is None or name in {
                str(order.vehicle or ""),
                str(order.assigned_robot or ""),
            }:
                self._stationary_order_retry_state.pop(order_id, None)

        if any(item[0] == name for item in self._rolling_vacancy_recovery_signature):
            self._rolling_vacancy_recovery_signature = ()
            self._rolling_vacancy_recovery_blacklist.clear()
        else:
            self._rolling_vacancy_recovery_blacklist = {
                item
                for item in self._rolling_vacancy_recovery_blacklist
                if item[1] != name
            }
        removed_vacancy_sinks = {
            sink_name
            for sink_name, signature
            in self._commanded_sink_vacancy_signatures.items()
            if sink_name == name
            or any(item[0] == name for item in signature)
        }
        for sink_name in removed_vacancy_sinks:
            self._commanded_sink_vacancy_signatures.pop(sink_name, None)
        self._commanded_sink_vacancy_blacklist = {
            item
            for item in self._commanded_sink_vacancy_blacklist
            if item[0] not in removed_vacancy_sinks
            and item[2] != name
        }

        for state in (
            self._active_wait_cycles,
            self._wait_cycle_last_arbitration,
            self._wait_cycle_grant_signatures,
            self._coupled_replan_last_attempt,
            self._coupled_replan_failures,
        ):
            for cycle_key in list(state):
                if name in cycle_key:
                    state.pop(cycle_key, None)

        for signature in list(self._wait_cycle_recovery_attempts):
            members = signature[2] if len(signature) > 2 else ()
            if any(member and member[0] == name for member in members):
                self._wait_cycle_recovery_attempts.pop(signature, None)
        for key, victim_name in list(
            self._controlled_corridor_recovery_latches.items()
        ):
            if key[1] == name or victim_name == name:
                self._controlled_corridor_recovery_latches.pop(key, None)

        for key in list(self._controlled_corridor_wait_since):
            if key and key[-1] == name:
                self._controlled_corridor_wait_since.pop(key, None)
        for key in list(self._traffic_zone_wait_since):
            if len(key) > 1 and key[1] == name:
                self._traffic_zone_wait_since.pop(key, None)
        self._controlled_corridor_winners.pop(name, None)
        self._controlled_corridor_passages.pop(name, None)
        self._controlled_corridor_prefetch_intents.pop(name, None)
        self._controlled_corridor_entry_cache.pop(name, None)
        self._controlled_corridor_approach_holds.pop(name, None)
        self._traffic_zone_winners.pop(name, None)
        for region_id, lease in list(self._controlled_corridor_leases.items()):
            if isinstance(lease, tuple) and lease and lease[0] == name:
                self._controlled_corridor_leases.pop(region_id, None)
        for key in list(self._traffic_zone_leases):
            if len(key) > 1 and key[1] == name:
                self._traffic_zone_leases.pop(key, None)
        for membership in (
            self._controlled_corridor_occupancy,
            self._controlled_corridor_queues,
            self._traffic_zone_queues,
        ):
            for region_id, robot_names in list(membership.items()):
                filtered = [item for item in robot_names if item != name]
                if filtered:
                    membership[region_id] = filtered
                else:
                    membership.pop(region_id, None)

    def reset_planning_runtime_state(self) -> None:
        """Reset transient planner/arbitration state without racing a worker.

        A Python planning thread cannot be force-cancelled safely. Mark the
        current result stale and let that one worker finish; the dispatcher
        will discard it before it can mutate a newly reset benchmark.
        """
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                if self._dispatch_job.get("kind") == "dispatch":
                    for entry in self._dispatch_job.get("entries", []):
                        if not isinstance(entry, tuple) or not entry:
                            continue
                        order = entry[0]
                        if (
                            isinstance(order, FleetOrder)
                            and self.orders.get(order.order_id) is order
                            and order.status == "PLANNING"
                        ):
                            order.status = "QUEUED"
                            order.error = ""
                            order.updated_at = self._now()
                self._dispatch_job["discard"] = True
        self._last_async_job_kind = ""
        self._runtime_replans.clear()
        self._rolling_prefetch_retry_at.clear()
        self._rolling_prefetch_failures.clear()
        self._rolling_prefetch_blockers.clear()
        self._rolling_prefetch_eligible_since.clear()
        self._rolling_prefetch_last_attempt_at.clear()
        for robot in self.robots.values():
            robot.rolling_boundary_since = None
        self._stationary_order_retry_state.clear()
        self._dispatch_conflict_dependencies.clear()
        self._stationary_clearance_relocations.clear()
        self._rolling_vacancy_recovery_signature = ()
        self._rolling_vacancy_recovery_blacklist.clear()
        self._commanded_sink_vacancy_signatures.clear()
        self._commanded_sink_vacancy_blacklist.clear()
        self._coupled_replan_last_attempt.clear()
        self._coupled_replan_failures.clear()
        self._active_wait_cycles.clear()
        self._wait_cycle_last_arbitration.clear()
        self._wait_cycle_grant_signatures.clear()
        self._wait_cycle_recovery_attempts.clear()
        self._controlled_corridor_recovery_latches.clear()
        self._runtime_tick_route_clocks.clear()
        self.reset_traffic_flow_state()

    def simulation_time(self) -> float:
        """Return the accelerated clock used by simulated fleet runtime."""
        return self._now()

    def simulation_time_scale(self) -> float:
        with self._simulation_clock_lock:
            return self._simulation_time_scale

    def set_simulation_time_scale(self, value: Any) -> float:
        try:
            requested = float(value)
        except (TypeError, ValueError):
            requested = 1.0
        if not math.isfinite(requested):
            requested = 1.0
        maximum = self._simulation_time_scale_limit()
        requested = max(1.0, min(maximum, requested))
        with self._simulation_clock_lock:
            wall_now = time()
            elapsed = max(0.0, wall_now - self._simulation_clock_wall_at)
            self._simulation_clock += elapsed * self._simulation_time_scale
            self._simulation_clock_wall_at = wall_now
            self._simulation_time_scale = requested
        return requested

    def _now(self) -> float:
        with self._simulation_clock_lock:
            wall_now = time()
            elapsed = max(0.0, wall_now - self._simulation_clock_wall_at)
            self._simulation_clock += elapsed * self._simulation_time_scale
            self._simulation_clock_wall_at = wall_now
            return self._simulation_clock

    def _configured_simulation_time_scale(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            value = float(fleet.get("simulation_time_scale", 1.0) or 1.0)
        except (TypeError, ValueError):
            return 1.0
        if not math.isfinite(value):
            return 1.0
        return max(1.0, min(self._simulation_time_scale_limit(), value))

    def _simulation_time_scale_limit(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return self.MAX_SIMULATION_TIME_SCALE
        try:
            configured = float(
                fleet.get(
                    "simulation_time_scale_max",
                    self.MAX_SIMULATION_TIME_SCALE,
                )
                or self.MAX_SIMULATION_TIME_SCALE
            )
        except (TypeError, ValueError):
            configured = self.MAX_SIMULATION_TIME_SCALE
        if not math.isfinite(configured):
            configured = self.MAX_SIMULATION_TIME_SCALE
        return max(1.0, min(self.MAX_SIMULATION_TIME_SCALE, configured))

    def state(self, include_trajectories: bool = True) -> dict[str, Any]:
        self._advance_runtime()
        return self._state_snapshot(include_trajectories=include_trajectories)

    def advance_runtime(self) -> None:
        self._advance_runtime()

    def snapshot(self, include_trajectories: bool = True) -> dict[str, Any]:
        return self._state_snapshot(include_trajectories=include_trajectories)

    def _state_snapshot(
        self,
        include_trajectories: bool = True,
        route_revisions: dict[str, int] | None = None,
        include_runtime_details: bool = True,
    ) -> dict[str, Any]:
        pending_by_robot = self.task_manager.pending_by_robot()
        state = {
            "ok": True,
            "robots": [
                self._robot_snapshot_payload(
                    robot,
                    include_trajectory=(
                        include_trajectories
                        or (
                            route_revisions is not None
                            and int(route_revisions.get(robot.name, -1)) != robot.route_revision
                        )
                    ),
                    pending_orders=pending_by_robot.get(robot.name, []),
                )
                for robot in self._runtime_robots()
            ],
            "simulationTimeScale": self.simulation_time_scale(),
            "simulationTimeScaleMax": self._simulation_time_scale_limit(),
        }
        if include_runtime_details:
            state.update(
                {
                    "events": [event.to_dict() for event in self.events[-80:]],
                    "obstacles": self.obstacles,
                    "obstacleAreas": self.obstacle_areas,
                    "orders": self._orders_list(),
                    "traffic": dict(self.traffic_metrics),
                    "lastRuntimeSafetyRollback": (
                        self._last_runtime_safety_rollback
                    ),
                    "trafficFlow": self._traffic_flow_payload(),
                }
            )
        return state

    def _robot_snapshot_payload(
        self,
        robot: FleetRobot,
        *,
        include_trajectory: bool,
        pending_orders: list[FleetOrder] | None = None,
    ) -> dict[str, Any]:
        payload = robot.to_dict(include_trajectory=include_trajectory)
        if pending_orders is None:
            pending_orders = self.task_manager.pending_for_robot(robot.name)
        if not pending_orders:
            payload.update(
                {
                    "assignedOrderId": "",
                    "assignedOrderStatus": "",
                    "assignedOrderTargetLm": "",
                    "orderQueueDepth": 0,
                }
            )
            return payload

        assigned = next(
            (
                order
                for order in pending_orders
                if order.order_id == robot.active_order_id
            ),
            pending_orders[0],
        )
        target_lm = self._active_order_target(assigned)
        payload.update(
            {
                "assignedOrderId": assigned.order_id,
                "assignedOrderStatus": assigned.status,
                "assignedOrderTargetLm": target_lm,
                "orderQueueDepth": len(pending_orders),
            }
        )
        # A queued order already belongs to this robot even before its MAPF
        # route is committed. Expose that destination without overloading
        # activeOrderId, whose execution semantics are used by the motion
        # controller and browser interpolation clock.
        if not str(payload.get("targetLm") or ""):
            payload["targetName"] = target_lm
            payload["targetLm"] = target_lm
        return payload

    def tick(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._advance_runtime()
        return self.stream_tick()

    def stream_tick(
        self,
        route_revisions: dict[str, int] | None = None,
        *,
        include_runtime_details: bool = True,
    ) -> dict[str, Any]:
        state = self._state_snapshot(
            include_trajectories=False,
            route_revisions=route_revisions,
            include_runtime_details=include_runtime_details,
        )
        for robot in self._runtime_robots():
            robot.trajectory_dirty = False
            robot.route_preview_dirty = False
        return state

    def _should_stream_trajectory(self, robot: FleetRobot) -> bool:
        return bool(
            robot.trajectory
            and robot.status in {"MOVING", "WAITING", "BLOCKED", "PLANNING"}
        )

    def _robot_mode_key(self, robot: FleetRobot) -> str:
        return "remote" if robot.is_remote() else "simulated"

    def _robot_enabled(self, robot: FleetRobot) -> bool:
        if self.active_robot_modes is None:
            return True
        return self._robot_mode_key(robot) in self.active_robot_modes

    def _runtime_robots(self) -> list[FleetRobot]:
        return [
            robot
            for robot in self.robots.values()
            if self._robot_enabled(robot)
        ]

    def _order_enabled(self, order: FleetOrder) -> bool:
        if order.internal_kind:
            return False
        if self.active_robot_modes is None:
            return True
        robot_name = order.assigned_robot or order.vehicle
        if not robot_name:
            return True
        robot = self.robots.get(robot_name)
        return bool(robot is not None and self._robot_enabled(robot))

    def update_world(self, payload: dict[str, Any]) -> dict[str, Any]:
        obstacles = payload.get("obstacles", [])
        areas = payload.get("obstacleAreas", [])
        previous_counts = (len(self.obstacles), len(self.obstacle_areas))
        if isinstance(obstacles, list):
            self.obstacles = [
                self._clean_obstacle(item)
                for item in obstacles
                if isinstance(item, dict)
            ]
        if isinstance(areas, list):
            self.obstacle_areas = [
                self._clean_area(item)
                for item in areas
                if isinstance(item, dict)
            ]
        params = payload.get("params")
        if isinstance(params, dict):
            self.params = params
            self.collision.set_params(params)
            if self._external_remote_adapter is None:
                self._configure_robot_gateway()
        counts = (len(self.obstacles), len(self.obstacle_areas))
        if counts != previous_counts:
            self._event(
                "info",
                f"world synced: obstacles={counts[0]}, areas={counts[1]}",
            )
        return {"ok": True, "state": self.state()}

    def _configure_robot_gateway(self) -> None:
        """Let a transport-specific subclass rebuild its params-bound gateway."""

    def check_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._advance_runtime()
        name = str(payload.get("name", "")).strip()
        poses = payload.get("poses", [])
        if not isinstance(poses, list):
            poses = []
        step = self.collision.sample_time_step()
        for index, item in enumerate(poses):
            if not isinstance(item, dict):
                continue
            pose = {
                "x": float(item.get("x", 0.0) or 0.0),
                "y": float(item.get("y", 0.0) or 0.0),
                "yaw": float(item.get("yaw", 0.0) or 0.0),
            }
            reason = self.collision.blocked_reason(
                pose=pose,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
            if reason:
                return {
                    "ok": True,
                    "blocked": True,
                    "reason": reason,
                    "index": index,
                    "pose": pose,
                }
            offset = index * step
            for other in self._runtime_robots():
                if other.name == name or other.pose is None:
                    continue
                other_pose = self._predicted_robot_pose(other, offset) or other.pose
                if other_pose is not None and self.collision.robot_footprints_conflict(pose, other_pose):
                    return {
                        "ok": True,
                        "blocked": True,
                        "reason": f"robot footprint conflict with {other.name}",
                        "index": index,
                        "pose": pose,
                    }
        return {"ok": True, "blocked": False, "reason": ""}

    def orders_payload(self) -> dict[str, Any]:
        self._advance_runtime()
        return {
            "ok": True,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def set_order(
        self,
        payload: dict[str, Any],
        *,
        dispatch: bool = True,
    ) -> dict[str, Any]:
        orders = self._build_orders(payload)
        if not orders:
            raise ValueError("no orders to queue")
        replace_active = bool(
            payload.get("replaceActive")
            or payload.get("replace_active")
            or payload.get("replace")
        )
        incoming_ids = set()
        for order in orders:
            if order.order_id in incoming_ids:
                raise ValueError(f"duplicate order id in payload: {order.order_id}")
            incoming_ids.add(order.order_id)
            existing = self.orders.get(order.order_id)
            if existing is not None and existing.status not in TERMINAL_ORDER_STATUSES:
                raise ValueError(f"active order already exists: {order.order_id}")

        self._prune_terminal_order_history()
        if replace_active:
            for vehicle in sorted({order.vehicle for order in orders if order.vehicle}):
                self._replace_orders_for_robot(vehicle, "replaced by operator")

        for order in orders:
            # Reusing an external order id is valid once its previous record
            # is terminal.  Its old stationary-blocker quarantine is not: it
            # describes another task and could otherwise suppress the fresh
            # order indefinitely while the same parked bodies remain nearby.
            self._stationary_order_retry_state.pop(order.order_id, None)
            self.orders[order.order_id] = order
            self._event(
                "info",
                f"order queued: {order.order_id} {order.vehicle or 'auto'}->{order.target_lm}",
            )
        if dispatch:
            self._dispatch_orders()
        if len(orders[0].targets or []) > 1:
            first = orders[0]
            self._event(
                "info",
                f"order sequence queued: {len(first.targets)} LM step(s) for {first.vehicle or 'auto'}",
            )
        return {
            "ok": True,
            "order": orders[0].to_dict(),
            "queuedOrders": [order.to_dict() for order in orders],
            "orders": self._orders_list(),
            "state": (
                self.state()
                if dispatch
                else self._state_snapshot(include_trajectories=True)
            ),
        }

    def dispatch_orders(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        dispatched = self._dispatch_orders(force=True)
        return {
            "ok": True,
            "dispatched": dispatched,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def cancel_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        self._cancel_order(order, "canceled by operator")
        return {
            "ok": True,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def pause_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        if order.status in TERMINAL_ORDER_STATUSES:
            raise ValueError(f"cannot pause terminal order: {order_id}")
        self._pause_order(order, "paused by operator")
        return {
            "ok": True,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def resume_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        order_id = str(payload.get("id") or payload.get("orderId") or "").strip()
        if not order_id:
            raise ValueError("order id is required")
        order = self.orders.get(order_id)
        if order is None:
            raise ValueError(f"unknown order: {order_id}")
        if order.status in TERMINAL_ORDER_STATUSES:
            raise ValueError(f"cannot resume terminal order: {order_id}")
        order.status = "QUEUED"
        order.error = ""
        order.updated_at = self._now()
        self._event("info", f"order resumed: {order.order_id}")
        dispatched = self._dispatch_orders(force=True)
        return {
            "ok": True,
            "dispatched": dispatched,
            "order": order.to_dict(),
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def clear_orders(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        include_active = bool(payload.get("includeActive", False))
        canceled = 0
        for order in list(self.orders.values()):
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if include_active or order.status == "QUEUED":
                self._cancel_order(order, "cleared by operator")
                canceled += 1
        if canceled:
            self._event("warn", f"orders cleared: {canceled}")
        return {
            "ok": True,
            "canceled": canceled,
            "orders": self._orders_list(),
            "state": self.state(),
        }

    def add_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = self._robot_mode_from_payload(payload)
        requested_name = self._robot_name_from_payload(payload)
        name = requested_name
        base_url = ""
        remote_status: dict[str, Any] | None = None
        remote_identity: dict[str, Any] | None = None
        if mode != "simulated":
            base_url = self._remote_base_url_from_payload(payload)
            if not base_url:
                raise ValueError("robot IP is required for remote robot")
            try:
                remote_identity = self.remote_adapter.identity(base_url)
                remote_status = self.remote_adapter.status(base_url)
            except Exception as exc:
                raise ValueError(f"remote robot is not reachable: {exc}") from exc
        current_lm = "" if mode != "simulated" else str(payload.get("currentLm") or payload.get("spawnLm") or "").strip()
        remote_pose: dict[str, float] | None = None
        if remote_status is not None:
            status_robot = self._remote_status_robot(remote_status)
            current_lm = current_lm or str(
                status_robot.get("nearestLm")
                or status_robot.get("currentLm")
                or status_robot.get("currentLM")
                or status_robot.get("currentStation")
                or status_robot.get("current_station")
                or ""
            ).strip()
            remote_pose = self._remote_pose_from_status(status_robot)
            if current_lm not in self.landmarks and remote_pose is not None:
                current_lm = self._nearest_lm_for_pose(remote_pose)
            if not name:
                name = self._remote_robot_name(remote_identity, status_robot, base_url)
            name = self._remote_unique_robot_name(name, base_url)
        if not name:
            raise ValueError("robot name is required")
        if not current_lm:
            if mode != "simulated":
                raise ValueError("remote robot has no current LM or localized pose yet; wait for robot status")
            raise ValueError("currentLm/spawnLm is required")
        if current_lm not in self.landmarks:
            raise ValueError(f"unknown LM: {current_lm}")

        robot = self.robots.get(name)
        if robot is None:
            self.clear_robot_ephemeral_state(name)
            now = self._now()
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                mode=mode,
                pose=self._pose_at_landmark(current_lm),
                base_url=base_url,
                remote_id=self._remote_identity_id(remote_identity),
                remote_online=True,
                updated_at=now,
            )
            self.robots[name] = robot
            if remote_status is not None:
                self._apply_remote_status(robot, remote_status, self._now())
            self._event("info", f"robot added: {name}@{current_lm}" + (f" remote={base_url}" if robot.is_remote() else ""))
        else:
            self.clear_robot_ephemeral_state(name)
            self._cancel_active_order_for_robot(robot, "robot respawned")
            if robot.is_remote():
                self._cancel_remote_route(robot, "robot respawned")
            robot.mode = mode
            robot.base_url = base_url
            robot.remote_id = self._remote_identity_id(remote_identity)
            robot.remote_online = True
            robot.remote_error = ""
            robot.current_lm = current_lm
            robot.pose = self._pose_at_landmark(current_lm)
            robot.target_lm = ""
            robot.trajectory = []
            robot.plan_nodes = []
            robot.route_clock = 0.0
            robot.route_note = ""
            robot.trajectory_dirty = True
            robot.blocked_since = None
            robot.last_replan_at = None
            robot.route_revision = 0
            robot.route_chunk_index = 0
            robot.route_chunk_goal_lm = ""
            robot.route_final_lm = ""
            robot.route_preview = []
            robot.route_preview_dirty = True
            robot.updated_at = self._now()
            if remote_status is not None:
                self._apply_remote_status(robot, remote_status, self._now())
            self._event("info", f"robot updated: {name}@{current_lm}" + (f" remote={base_url}" if robot.is_remote() else ""))
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def remove_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        removed = self.robots.pop(name, None)
        if removed is not None:
            self.clear_robot_ephemeral_state(name)
            self._cancel_active_order_for_robot(removed, "robot removed")
            self._cancel_orders_for_robot(name, "robot removed")
            self._event("warn", f"robot removed: {name}")
        return {"ok": True, "removed": removed is not None, "state": self.state()}

    def stop_robot(
        self,
        payload: dict[str, Any],
        *,
        include_state: bool = True,
    ) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        stopped_robot: FleetRobot | None = None
        if name:
            robot = self.robots.get(name)
            if robot is None:
                raise ValueError(f"unknown robot: {name}")
            self._stop_robot(robot)
            stopped_robot = robot
            self._cancel_orders_for_robot(name, "robot stopped")
            self._event("warn", f"robot stopped: {name}")
        else:
            for robot in self._runtime_robots():
                self._stop_robot(robot)
            self._cancel_all_orders("fleet stopped")
            self._event("warn", "fleet stopped")
        return {
            "ok": True,
            "robot": stopped_robot.to_dict() if stopped_robot is not None else None,
            "state": self.state() if include_state else None,
        }

    def teleop_robot(
        self,
        payload: dict[str, Any],
        *,
        include_state: bool = True,
    ) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            raise ValueError(f"unknown robot: {name}")
        if not robot.is_remote() or not robot.base_url:
            raise ValueError(f"{name} is not a remote robot")

        linear = float(payload.get("linear", 0.0) or 0.0)
        angular = float(payload.get("angular", 0.0) or 0.0)
        timeout_ms = max(80, int(payload.get("timeoutMs", 350) or 350))
        if robot.active_order_id:
            self._cancel_active_order_for_robot(robot, "manual control takeover")
        try:
            self._ensure_remote_control(robot, "manual control")
            response = self.remote_adapter.teleop(
                robot.base_url,
                linear=linear,
                angular=angular,
                timeout_ms=timeout_ms,
                owner_id=FLEET_CONTROL_OWNER_ID,
            )
            robot.remote_online = True
            robot.remote_error = ""
            status = response.get("status")
            if isinstance(status, dict):
                self._apply_remote_status(robot, status, self._now())
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote teleop failed: {exc}"
            robot.updated_at = self._now()
            raise ValueError(robot.last_reason) from exc

        robot.status = "MANUAL"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.trajectory_dirty = True
        robot.last_reason = "manual control active"
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()
        return {
            "ok": True,
            "robot": robot.to_dict(),
            "state": self.state() if include_state else None,
        }

    def teleop_stop_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            raise ValueError(f"unknown robot: {name}")
        if not robot.is_remote() or not robot.base_url:
            raise ValueError(f"{name} is not a remote robot")
        try:
            self._ensure_remote_control(robot, "manual stop")
            response = self.remote_adapter.teleop_stop(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
            status = response.get("status")
            if isinstance(status, dict):
                self._apply_remote_status(robot, status, self._now())
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote teleop stop failed: {exc}"
            robot.updated_at = self._now()
            raise ValueError(robot.last_reason) from exc

        if robot.status == "MANUAL":
            robot.status = "IDLE"
            robot.last_reason = "manual control released"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.trajectory_dirty = True
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()
        return {"ok": True, "robot": robot.to_dict(), "state": self.state()}

    def note_external_control_takeover(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
    ) -> bool:
        """Mirror a direct robot takeover into Fleet Manager immediately.

        The robot remains the source of truth for ownership.  This local note
        closes the short race where an operator can acquire and release again
        before the next remote status poll observes the foreign owner.
        """
        try:
            normalized = normalize_grpc_endpoint(endpoint, default_port=DEFAULT_GRPC_PORT)
        except Exception:
            normalized = str(endpoint or "").strip()
        parsed_endpoint = urlparse(normalized)
        endpoint_key = (
            str(parsed_endpoint.hostname or "").lower(),
            int(parsed_endpoint.port or DEFAULT_GRPC_PORT),
        )
        robot = next(
            (
                candidate
                for candidate in self.robots.values()
                if candidate.is_remote()
                and (
                    candidate.base_url == normalized
                    or (
                        str(urlparse(candidate.base_url).hostname or "").lower(),
                        int(urlparse(candidate.base_url).port or DEFAULT_GRPC_PORT),
                    ) == endpoint_key
                )
            ),
            None,
        )
        if robot is None:
            return False

        status = dict(robot.remote_status) if isinstance(robot.remote_status, dict) else {}
        control = dict(status.get("control")) if isinstance(status.get("control"), dict) else {}
        control.update({"state": "OWNED", "ownerId": owner_id, "ownerName": owner_name})
        status.update(
            {
                "control": control,
                "controlState": "OWNED",
                "controlOwner": owner_id,
                "controlOwnerName": owner_name,
            }
        )
        robot.remote_status = status
        now = self._now()
        if robot.active_order_id:
            order = self.orders.get(robot.active_order_id)
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                self._pause_order_for_external_control(robot, order, now, owner_name or owner_id)
                return True
        robot.status = "MANUAL"
        robot.last_reason = f"{EXTERNAL_CONTROL_PAUSE_PREFIX} {owner_name or owner_id}"
        robot.updated_at = now
        return True

    def reset_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        target_names = [name] if name else [robot.name for robot in self._runtime_robots()]
        for robot_name in target_names:
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            self._cancel_active_order_for_robot(robot, "robot reset")
            spawn_lm = str(payload.get("spawnLm") or robot.current_lm or "").strip()
            if spawn_lm in self.landmarks:
                robot.current_lm = spawn_lm
                robot.pose = self._pose_at_landmark(spawn_lm)
            robot.target_lm = ""
            robot.status = "IDLE"
            robot.trajectory = []
            robot.plan_nodes = []
            robot.trajectory_dirty = True
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = None
            robot.blocked_since = None
            robot.last_replan_at = None
            robot.last_reason = "reset"
            robot.route_note = ""
            self._clear_remote_route_metadata(robot)
            robot.updated_at = self._now()
            self._event("warn", f"robot reset: {robot.name}@{robot.current_lm}")
        return {"ok": True, "state": self.state()}

    def update_robot(
        self,
        payload: dict[str, Any],
        *,
        include_state: bool = True,
    ) -> dict[str, Any]:
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("robot name is required")
        robot = self.robots.get(name)
        if robot is None:
            current_lm = str(payload.get("currentLm") or "").strip()
            if not current_lm:
                raise ValueError("unknown robot and currentLm is missing")
            robot = FleetRobot(
                name=name,
                current_lm=current_lm,
                pose=self._pose_at_landmark(current_lm),
                updated_at=self._now(),
            )
            self.robots[name] = robot

        if "currentLm" in payload and payload["currentLm"]:
            robot.current_lm = str(payload["currentLm"] or "")
            if robot.current_lm and not robot.trajectory:
                robot.pose = self._pose_at_landmark(robot.current_lm)
        if "targetLm" in payload:
            robot.target_lm = str(payload["targetLm"] or "")
        if "mode" in payload or "type" in payload or "robotMode" in payload:
            robot.mode = self._robot_mode_from_payload(payload)
        if "baseUrl" in payload or "url" in payload or "host" in payload:
            robot.base_url = self._remote_base_url_from_payload(payload)
        if "status" in payload and payload["status"]:
            robot.status = str(payload["status"])
        if "pose" in payload and isinstance(payload["pose"], dict):
            pose = payload["pose"]
            robot.pose = {
                "x": float(pose.get("x", 0.0) or 0.0),
                "y": float(pose.get("y", 0.0) or 0.0),
                "yaw": float(pose.get("yaw", 0.0) or 0.0),
            }
        if robot.status in {"IDLE", "ARRIVED", "BLOCKED", "STOPPED", "MANUAL", "MANUAL_BLOCKED"} and not robot.target_lm:
            self._cancel_active_order_for_robot(robot, f"robot status {robot.status.lower()}")
            robot.trajectory = []
            robot.plan_nodes = []
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = None
            robot.trajectory_dirty = True
            self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()
        return {
            "ok": True,
            "robot": robot.to_dict(),
            "state": self.state() if include_state else None,
        }

    def plan(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Plan one or more orders.

        Compatible with the old /api/fleet/plan payload:
        {"robots": [{"name": "...", "startLm": "...", "goalLm": "..."}]}.
        """
        requests = payload.get("robots", [])
        if not isinstance(requests, list):
            raise ValueError("robots must be a list")

        valid_requests: list[dict[str, Any]] = []
        for request in requests:
            if not isinstance(request, dict):
                continue
            name = str(request.get("name", "")).strip()
            start_lm = str(request.get("startLm") or request.get("currentLm") or "").strip()
            goal_lm = str(request.get("goalLm") or request.get("targetLm") or "").strip()
            start_pose = request.get("startPose")
            if not name or not start_lm or not goal_lm:
                self._event("error", f"bad order ignored: name/start/goal is missing")
                continue
            if start_lm not in self.landmarks:
                self._block_order(name, start_lm, goal_lm, f"unknown start LM: {start_lm}")
                continue
            if goal_lm not in self.landmarks:
                self._block_order(name, start_lm, goal_lm, f"unknown goal LM: {goal_lm}")
                continue
            robot = self.robots.get(name)
            authoritative_pose = (
                dict(robot.pose)
                if robot is not None and isinstance(robot.pose, dict)
                else start_pose
            )
            if robot is not None and robot.trajectory:
                safe_start_lm = self._safe_replan_start_lm(robot)
                if not safe_start_lm:
                    self._event(
                        "warn",
                        f"order deferred for {name}: robot is between LMs; "
                        "keeping the current graph edge",
                    )
                    continue
                if safe_start_lm != start_lm:
                    self._event(
                        "info",
                        f"corrected stale start LM for {name}: {start_lm}->{safe_start_lm}",
                    )
                    start_lm = safe_start_lm
            if isinstance(authoritative_pose, dict) and not self._pose_is_at_lm(
                authoritative_pose,
                start_lm,
            ):
                self._event(
                    "error",
                    f"order rejected for {name}: pose is not at {start_lm}; "
                    "off-graph approach is forbidden",
                )
                continue
            if robot is None:
                robot = FleetRobot(
                    name=name,
                    current_lm=start_lm,
                    pose=self._pose_at_landmark(start_lm),
                    updated_at=self._now(),
                )
                self.robots[name] = robot
            robot.current_lm = start_lm
            robot.target_lm = goal_lm
            robot.status = "PLANNING"
            robot.last_reason = "order accepted"
            robot.blocked_since = None
            robot.updated_at = self._now()
            self._event("info", f"order accepted: {name} {start_lm}->{goal_lm}")
            clean_request: dict[str, Any] = {
                "name": name,
                "startLm": start_lm,
                "goalLm": goal_lm,
            }
            if isinstance(authoritative_pose, dict):
                clean_request["startPose"] = {
                    "x": float(authoritative_pose.get("x", 0.0) or 0.0),
                    "y": float(authoritative_pose.get("y", 0.0) or 0.0),
                    "yaw": float(authoritative_pose.get("yaw", 0.0) or 0.0),
                }
            elif robot.pose is not None:
                clean_request["startPose"] = dict(robot.pose)
            valid_requests.append(clean_request)

        if not valid_requests:
            self._event("error", "planner skipped: no valid orders")
            return {
                "ok": False,
                "debug": {
                    "reason": "no valid orders",
                    "conflictsResolved": 0,
                    "highLevelNodes": 0,
                    "expandedNodes": 0,
                },
                "timeStepSec": 0.0,
                "plans": [],
                "fleetState": self.state(),
            }

        result = self._plan_valid_requests(valid_requests, payload)
        planned_names = {str(plan.get("robot")) for plan in result.get("plans", []) if isinstance(plan, dict)}
        if result.get("ok"):
            now = self._now()
            self._apply_planner_result(result, now)
            self._event("info", f"planner accepted {len(planned_names)} order(s)")
        else:
            deadlock = self._planner_deadlock_result(result)
            reason = self._planner_failure_reason(result)
            for request in valid_requests:
                if not isinstance(request, dict):
                    continue
                robot = self.robots.get(str(request.get("name", "")).strip())
                if robot is not None:
                    robot.status = "WAITING" if deadlock else "BLOCKED"
                    robot.last_reason = reason
                    if deadlock:
                        robot.blocked_since = self._now()
                        robot.trajectory = []
                        robot.plan_nodes = []
                        robot.trajectory_dirty = True
                        robot.route_started_at = None
                        robot.route_clock = 0.0
                        robot.last_tick_at = None
                        robot.route_note = "DEADLOCK"
                    robot.updated_at = self._now()
            self._event("error", f"planner rejected: {reason}")

        return {
            **result,
            "fleetState": self.state(),
        }

    def _event(self, level: str, message: str) -> None:
        # Event log timestamps stay in wall time for a human operator even
        # while the simulated traffic clock is running faster than real time.
        self.events.append(FleetEvent(stamp=time(), level=level, message=message))
        self.events = self.events[-200:]

    def _complete_simulated_route_chunk(self, robot: FleetRobot, now: float) -> bool:
        """Hold a completed safe chunk until its continuation is ready.

        Clearing the trajectory and active order here exposed an artificial
        IDLE robot to both the browser and subsequent MAPF requests. Keeping
        the terminal trajectory preserves the LM reservation and permits an
        atomic append without resetting the route clock.
        """
        if not robot.active_order_id or not robot.route_chunk_goal_lm:
            return False
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return False
        if robot.route_note == "manual graph reconnect":
            order.status = "QUEUED"
            order.error = ""
            order.updated_at = now
            order.assigned_robot = robot.name
            order.start_lm = robot.current_lm
            order.route_nodes = []
            robot.active_order_id = ""
            robot.target_lm = ""
            robot.status = "IDLE"
            robot.trajectory = []
            robot.trajectory_dirty = True
            robot.plan_nodes = []
            robot.route_started_at = None
            robot.route_clock = 0.0
            robot.last_tick_at = now
            robot.last_reason = "manual graph reconnect complete; route queued"
            robot.route_note = ""
            self._clear_remote_route_metadata(robot)
            robot.updated_at = now
            self._event(
                "info",
                f"manual graph reconnect complete: {robot.name}@{robot.current_lm}",
            )
            return True
        final_target = self._active_order_target(order)
        if robot.current_lm != robot.route_chunk_goal_lm or robot.current_lm == final_target:
            return False

        first_boundary_tick = robot.rolling_boundary_since is None
        if first_boundary_tick:
            robot.rolling_boundary_since = now
            self._rolling_prefetch_eligible_since.setdefault(robot.name, now)
        order.status = "PLANNING"
        order.error = "rolling continuation pending"
        # Do not erase the real waiting age on every 10 Hz physics tick.
        if first_boundary_tick:
            order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = robot.current_lm
        order.route_nodes = list(robot.plan_nodes)
        if robot.status != "WAITING" or robot.last_reason != "rolling continuation pending":
            self._event(
                "info",
                f"route continuation pending: {order.order_id} "
                f"{robot.name}@{robot.current_lm}->{final_target}",
            )
        robot.status = "WAITING"
        robot.last_reason = "rolling continuation pending"
        robot.blocked_since = None
        robot.traffic_stall_since = None
        self._clear_wait_dependency(robot)
        robot.updated_at = now
        return True

    def _activate_rolling_prefetch(self, robot: FleetRobot, now: float) -> bool:
        pending = robot.pending_route
        if not isinstance(pending, dict) or not robot.active_order_id:
            return False
        order = self.orders.get(robot.active_order_id)
        if (
            order is None
            or order.status in TERMINAL_ORDER_STATUSES
            or str(pending.get("order_id") or "") != order.order_id
            or str(pending.get("start_lm") or "") != robot.current_lm
        ):
            robot.pending_route = None
            return False
        result = pending.get("result")
        if not isinstance(result, dict):
            robot.pending_route = None
            return False
        plan = self._plan_for_robot(result, robot.name)
        if plan is None:
            robot.pending_route = None
            return False

        # Switch at the exact graph LM.  The new trajectory starts at t=0 on
        # the same pose, so the browser sees one continuous route clock rather
        # than an IDLE frame between rolling chunks.
        robot.pending_route = None
        self._apply_planner_result(result, now, order_id=order.order_id)
        order.route_nodes = [str(item) for item in plan.get("nodes", [])]
        self._apply_simulated_route_metadata(robot, order, plan, now)
        self._set_order_status(
            order,
            "EXECUTING",
            robot=robot,
            start_lm=robot.current_lm,
        )
        robot.last_reason = "rolling route continued"
        self._event(
            "info",
            f"route prefetched: {order.order_id} {robot.name}@{robot.current_lm}",
        )
        return True

    def _update_active_order_from_robot(self, robot: FleetRobot) -> None:
        if not robot.active_order_id:
            return
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return
        if robot.status == "WAITING":
            if self._is_robot_conflict(robot.last_reason) or str(robot.last_reason).startswith(
                "planned traffic wait"
            ):
                status = "WAITING_TRAFFIC"
            else:
                status = "WAITING_OBSTACLE"
        elif robot.status == "MOVING":
            status = "EXECUTING"
        elif robot.status == "BLOCKED":
            status = "PAUSED"
        elif robot.status == "PLANNING":
            status = "PLANNING"
        elif robot.status == "OFFLINE":
            status = "QUEUED"
        else:
            status = order.status
        order.status = status
        order.error = "" if status == "EXECUTING" else robot.last_reason
        order.updated_at = self._now()
        order.route_nodes = list(robot.plan_nodes)

    def _stop_robot(self, robot: FleetRobot, cancel_active_order: bool = True) -> None:
        self._stop_remote_robot(robot)
        self._runtime_replans.pop(robot.name, None)
        if cancel_active_order and robot.active_order_id:
            order = self.orders.get(robot.active_order_id)
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                self._set_order_status(order, "CANCELED", error="robot stopped")
                self._event("warn", f"order canceled: {order.order_id} robot stopped")
        robot.status = "STOPPED"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.last_tick_at = None
        robot.blocked_since = None
        robot.last_replan_at = None
        robot.last_reason = "stopped"
        robot.route_note = ""
        robot.active_order_id = ""
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()

    def _block_order(self, name: str, start_lm: str, goal_lm: str, reason: str) -> None:
        robot = self.robots.get(name)
        if robot is None:
            robot = FleetRobot(
                name=name,
                current_lm=start_lm,
                target_lm=goal_lm,
                status="BLOCKED",
                pose=self._pose_at_landmark(start_lm),
                last_reason=reason,
                updated_at=self._now(),
            )
            self.robots[name] = robot
        else:
            robot.current_lm = start_lm
            robot.target_lm = goal_lm
            robot.status = "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = self._now()
        self._event("error", f"{name} blocked: {reason}")

    def _robot_mode_from_payload(self, payload: dict[str, Any]) -> str:
        raw = str(payload.get("mode") or payload.get("type") or payload.get("robotMode") or "simulated").strip().lower()
        if raw in {"remote", "robot", "real", "grpc", "aivison_grpc", "real_grpc"}:
            return "remote"
        return "simulated"

    def _robot_name_from_payload(self, payload: dict[str, Any]) -> str:
        return str(
            payload.get("name")
            or payload.get("robotName")
            or payload.get("robot_name")
            or payload.get("alias")
            or ""
        ).strip()

    def _remote_base_url_from_payload(self, payload: dict[str, Any]) -> str:
        value = str(
            payload.get("baseUrl")
            or payload.get("url")
            or payload.get("host")
            or payload.get("ip")
            or payload.get("address")
            or ""
        ).strip()
        if not value:
            return ""
        if getattr(self.remote_adapter, "transport", "") == "grpc":
            if value.startswith("grpc://") or value.startswith("grpcs://"):
                return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
            if "://" in value:
                raise ValueError("unsupported robot endpoint scheme; use grpc://host:port")
            port_raw = payload.get("port") or payload.get("grpcPort") or payload.get("grpc_port")
            if port_raw is None:
                return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
            try:
                port = int(port_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid robot gRPC port") from exc
            return normalize_grpc_endpoint(f"grpc://{value}:{port}", default_port=DEFAULT_GRPC_PORT)
        if "://" in value:
            raise ValueError("unsupported robot endpoint scheme; use grpc://host:port")
        port_raw = payload.get("port") or payload.get("grpcPort") or payload.get("grpc_port")
        if port_raw is None:
            return normalize_grpc_endpoint(value, default_port=DEFAULT_GRPC_PORT)
        try:
            port = int(port_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid robot gRPC port") from exc
        return normalize_grpc_endpoint(f"grpc://{value}:{port}", default_port=DEFAULT_GRPC_PORT)

    def _remote_robot_name(
        self,
        identity_payload: dict[str, Any] | None,
        status_payload: dict[str, Any] | None,
        base_url: str,
    ) -> str:
        candidates: list[Any] = []

        def collect(payload: dict[str, Any] | None) -> None:
            if not isinstance(payload, dict):
                return
            for key in ("robotId", "robot_id", "name", "id", "vehicleId", "vehicle_id", "uuid", "serial"):
                candidates.append(payload.get(key))
            for nested_key in ("identity", "robot", "basic_info", "basicInfo", "robot_report", "robotReport"):
                nested = payload.get(nested_key)
                if isinstance(nested, dict):
                    for key in ("robotId", "robot_id", "name", "id", "vehicleId", "vehicle_id", "uuid", "serial"):
                        candidates.append(nested.get(key))

        collect(identity_payload)
        collect(status_payload)
        for value in candidates:
            text = str(value or "").strip()
            if text and text.lower() not in {"none", "null", "unknown", "-"}:
                return text
        parsed = urlparse(base_url)
        return str(parsed.hostname or parsed.netloc or "").strip()

    def _remote_unique_robot_name(self, name: str, base_url: str) -> str:
        clean_name = str(name or "").strip() or self._remote_name_from_endpoint(base_url)
        for existing in self.robots.values():
            if existing.is_remote() and existing.base_url == base_url:
                return existing.name
        existing = self.robots.get(clean_name)
        if existing is None or (existing.is_remote() and existing.base_url == base_url):
            return clean_name

        suffix = self._remote_name_from_endpoint(base_url)
        candidate = f"{clean_name}-{suffix}" if suffix and suffix != clean_name else f"{clean_name}-remote"
        index = 2
        while candidate in self.robots:
            candidate = f"{clean_name}-{suffix or 'remote'}-{index}"
            index += 1
        return candidate

    def _remote_name_from_endpoint(self, base_url: str) -> str:
        parsed = urlparse(base_url)
        host = str(parsed.hostname or parsed.netloc or "").strip()
        if not host:
            return "remote"
        parts = [part for part in host.replace(":", ".").split(".") if part]
        if len(parts) >= 4 and all(part.isdigit() for part in parts[-4:]):
            return f"robot-{parts[-1]}"
        return host.replace(".", "-")

    def _remote_identity_id(self, identity_payload: dict[str, Any] | None) -> str:
        if not isinstance(identity_payload, dict):
            return ""
        identity = identity_payload.get("identity")
        if isinstance(identity, dict):
            value = identity.get("robotId") or identity.get("id")
        else:
            value = identity_payload.get("robotId") or identity_payload.get("id")
        return str(value or "").strip()

    def _remote_status_robot(self, payload: dict[str, Any]) -> dict[str, Any]:
        robot_payload = payload.get("robot")
        if isinstance(robot_payload, dict):
            return robot_payload
        return payload if isinstance(payload, dict) else {}

    def _remote_pose_from_status(self, status_payload: dict[str, Any]) -> dict[str, float] | None:
        pose = status_payload.get("pose")
        if isinstance(pose, dict):
            try:
                return {
                    "x": float(pose.get("x", 0.0) or 0.0),
                    "y": float(pose.get("y", 0.0) or 0.0),
                    "yaw": float(pose.get("yaw", 0.0) or pose.get("angle", 0.0) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        if "x" in status_payload and "y" in status_payload:
            try:
                return {
                    "x": float(status_payload.get("x", 0.0) or 0.0),
                    "y": float(status_payload.get("y", 0.0) or 0.0),
                    "yaw": float(status_payload.get("yaw", status_payload.get("angle", 0.0)) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        robot_report = status_payload.get("robot_report") or status_payload.get("robotReport")
        if isinstance(robot_report, dict) and "x" in robot_report and "y" in robot_report:
            try:
                return {
                    "x": float(robot_report.get("x", 0.0) or 0.0),
                    "y": float(robot_report.get("y", 0.0) or 0.0),
                    "yaw": float(robot_report.get("angle", robot_report.get("yaw", 0.0)) or 0.0),
                }
            except (TypeError, ValueError):
                return None
        return None

    def _remote_timeout(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.8
        try:
            return max(0.2, float(fleet.get("remote_timeout_sec", 0.8) or 0.8))
        except (TypeError, ValueError):
            return 0.8

    def _clear_remote_route_metadata(self, robot: FleetRobot) -> None:
        self._clear_rolling_prefetch_state(robot.name)
        self._runtime_replans.pop(robot.name, None)
        robot.route_revision = 0
        robot.route_chunk_index = 0
        robot.route_chunk_goal_lm = ""
        robot.route_final_lm = ""
        robot.route_preview = []
        robot.route_preview_dirty = True
        robot.pending_route = None
        self._clear_deadlock_retreat(robot)

    def _apply_simulated_route_metadata(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
        now: float,
    ) -> None:
        # A successful full dispatch/replan starts a new continuation episode.
        # Failure/backoff from an older route must not poison this route later.
        self._runtime_replans.pop(robot.name, None)
        self._clear_rolling_prefetch_state(robot.name)
        previous_final = robot.route_final_lm
        previous_chunk = robot.route_chunk_goal_lm
        planned_chunk_goal = str(
            plan.get("goalLm") or order.target_lm
        ).strip()
        trajectory = [
            sample
            for sample in plan.get("trajectory", [])
            if isinstance(sample, dict)
        ]
        trajectory_goal = (
            str(trajectory[-1].get("lm") or "").strip()
            if trajectory
            else ""
        )
        chunk_goal = (
            trajectory_goal
            if trajectory_goal in self.landmarks
            else planned_chunk_goal
        )
        final_goal = str(plan.get("finalGoalLm") or order.target_lm).strip()
        plan_nodes = [
            str(node)
            for node in plan.get("nodes", [])
            if str(node) in self.landmarks
        ]
        trajectory_nodes: list[str] = []
        for sample in trajectory:
            sample_lm = str(sample.get("lm") or "").strip()
            if (
                sample_lm in self.landmarks
                and (
                    not trajectory_nodes
                    or trajectory_nodes[-1] != sample_lm
                )
            ):
                trajectory_nodes.append(sample_lm)
        if (
            chunk_goal
            and plan_nodes
            and chunk_goal not in plan_nodes
            and trajectory_nodes
            and trajectory_nodes[-1] == chunk_goal
        ):
            plan_nodes = trajectory_nodes
        if (
            chunk_goal
            and plan_nodes
            and plan_nodes[-1] != chunk_goal
            and chunk_goal in plan_nodes
        ):
            # Keep every consumer on the same executable prefix.  This is a
            # defensive boundary for old/in-flight planner results; new
            # rolling results are already cut on the exact LM sample.
            terminal_index = max(
                index
                for index, node in enumerate(plan_nodes)
                if node == chunk_goal
            )
            plan_nodes = plan_nodes[: terminal_index + 1]
            plan["nodes"] = list(plan_nodes)
        if chunk_goal:
            plan["goalLm"] = chunk_goal
            robot.plan_nodes = list(plan_nodes)
            order.route_nodes = list(plan_nodes)
        if previous_final == final_goal and previous_chunk == robot.current_lm:
            chunk_index = robot.route_chunk_index + 1
        else:
            chunk_index = 0
        robot.route_revision = self._next_route_revision()
        robot.route_chunk_index = chunk_index
        robot.route_chunk_goal_lm = chunk_goal
        robot.route_final_lm = final_goal
        self._update_route_preview(
            robot,
            robot.current_lm,
            final_goal,
            blocked_edges=set(order.traffic_detour_edges),
            committed_trajectory=plan.get("trajectory"),
            committed_nodes=plan.get("nodes"),
            spatial_route_nodes=order.spatial_route_nodes,
        )
        robot.has_executed_route = True
        robot.pending_route = None
        robot.target_lm = chunk_goal
        robot.updated_at = now

    def _update_route_preview(
        self,
        robot: FleetRobot,
        start_lm: str,
        final_goal_lm: str,
        blocked_edges: set[tuple[str, str]] | None = None,
        committed_trajectory: Any = None,
        committed_nodes: Any = None,
        spatial_route_nodes: Any = None,
    ) -> None:
        if start_lm not in self.landmarks or final_goal_lm not in self.landmarks:
            robot.route_preview = []
            robot.route_preview_dirty = True
            return
        committed_samples = [
            item
            for item in (committed_trajectory if isinstance(committed_trajectory, list) else [])
            if isinstance(item, dict)
        ]
        preview: list[dict[str, Any]] = [
            {
                "x": float(sample.get("x", 0.0) or 0.0),
                "y": float(sample.get("y", 0.0) or 0.0),
                "yaw": float(sample.get("yaw", 0.0) or 0.0),
                "phase": "committed",
            }
            for sample in committed_samples
        ]
        nodes = [
            str(node)
            for node in (committed_nodes if isinstance(committed_nodes, list) else [])
            if str(node) in self.landmarks
        ]
        continuation_start = nodes[-1] if nodes else start_lm
        if not preview:
            continuation_start = start_lm
        stable_nodes = [
            str(node)
            for node in (
                spatial_route_nodes
                if isinstance(spatial_route_nodes, list)
                else []
            )
            if str(node) in self.landmarks
        ]
        stable_suffix: list[str] = []
        if continuation_start in stable_nodes:
            stable_suffix = stable_nodes[stable_nodes.index(continuation_start):]
        if stable_suffix and stable_suffix[-1] != final_goal_lm:
            stable_suffix = []
        if continuation_start != final_goal_lm:
            try:
                route = (
                    self._planned_route_from_nodes(stable_suffix)
                    if len(stable_suffix) >= 2
                    else self.planner.route_planner.find_route(
                        continuation_start,
                        final_goal_lm,
                        blocked_edges=blocked_edges,
                    )
                )
                continuation = self.planner.route_planner.sample_route(
                    route,
                    sample_distance=0.50,
                )
            except (RuntimeError, ValueError):
                continuation = []
            for sample in continuation:
                if not isinstance(sample, dict):
                    continue
                point = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                    "phase": "forecast",
                }
                if preview and math.hypot(
                    point["x"] - float(preview[-1].get("x", 0.0) or 0.0),
                    point["y"] - float(preview[-1].get("y", 0.0) or 0.0),
                ) < 0.001:
                    continue
                preview.append(point)
        robot.route_preview = preview
        robot.route_preview_dirty = True

    def _pose_at_landmark(self, lm_name: str) -> dict[str, float] | None:
        landmark = self.landmarks.get(lm_name)
        if landmark is None:
            return None
        return {"x": landmark.x, "y": landmark.y, "yaw": 0.0}

    def _pose_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> dict[str, float] | None:
        if not trajectory:
            return None
        if len(trajectory) == 1 or elapsed <= float(trajectory[0].get("t", 0.0) or 0.0):
            return self._pose_from_sample(trajectory[0])
        last = trajectory[-1]
        if elapsed >= float(last.get("t", 0.0) or 0.0):
            return self._pose_from_sample(last)

        low = 0
        high = len(trajectory) - 1
        while low + 1 < high:
            middle = (low + high) // 2
            middle_time = float(trajectory[middle].get("t", 0.0) or 0.0)
            if middle_time < elapsed:
                low = middle
            else:
                high = middle
        index = low
        start = trajectory[index]
        goal = trajectory[index + 1]
        start_t = float(start.get("t", 0.0) or 0.0)
        goal_t = float(goal.get("t", 0.0) or 0.0)
        span = max(0.0001, goal_t - start_t)
        ratio = (elapsed - start_t) / span
        yaw = self._interpolate_angle(
            float(start.get("yaw", 0.0) or 0.0),
            float(goal.get("yaw", 0.0) or 0.0),
            ratio,
        )
        return {
            "x": float(start.get("x", 0.0) or 0.0)
            + ((float(goal.get("x", 0.0) or 0.0) - float(start.get("x", 0.0) or 0.0)) * ratio),
            "y": float(start.get("y", 0.0) or 0.0)
            + ((float(goal.get("y", 0.0) or 0.0) - float(start.get("y", 0.0) or 0.0)) * ratio),
            "yaw": yaw,
        }

    def _pose_from_sample(self, sample: dict[str, Any]) -> dict[str, float]:
        return {
            "x": float(sample.get("x", 0.0) or 0.0),
            "y": float(sample.get("y", 0.0) or 0.0),
            "yaw": float(sample.get("yaw", 0.0) or 0.0),
        }

    def _interpolate_angle(self, start: float, goal: float, ratio: float) -> float:
        delta = (goal - start + math.pi) % (2.0 * math.pi) - math.pi
        return start + (delta * ratio)

    def _clean_obstacle(self, item: dict[str, Any]) -> dict[str, float]:
        return {
            "x": float(item.get("x", 0.0) or 0.0),
            "y": float(item.get("y", 0.0) or 0.0),
            "radius": max(0.0, float(item.get("radius", 0.08) or 0.08)),
        }

    def _clean_area(self, item: dict[str, Any]) -> dict[str, float]:
        return {
            "x1": float(item.get("x1", 0.0) or 0.0),
            "y1": float(item.get("y1", 0.0) or 0.0),
            "x2": float(item.get("x2", 0.0) or 0.0),
            "y2": float(item.get("y2", 0.0) or 0.0),
        }


__all__ = [
    "FleetCollisionChecker",
    "FleetEvent",
    "FleetManagerCore",
    "FleetOrder",
    "FleetRobot",
]
