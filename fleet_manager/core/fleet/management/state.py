"""FleetManager runtime state, clocks and owned worker lifecycle."""

from __future__ import annotations

import math
from pathlib import Path
from threading import Lock
from time import time as _system_time
from typing import Any

from fleet_manager.core.geometry.collision import FleetCollisionChecker
from fleet_manager.core.transport.gateways import UnavailableRobotGateway
from fleet_manager.core.mapf.fleet_planner import FleetMapfPlanner
from fleet_manager.core.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.domain.models import FleetEvent, FleetOrder, FleetRobot
from fleet_manager.core.workers.planning import PlanningWorker
from fleet_manager.core.domain.settings import FleetSettings
from fleet_manager.core.route_core.models import GraphEdge, Landmark, MapMetadata
from fleet_manager.core.tasks.manager import FleetTaskManager
from fleet_manager.core.traffic.corridor_scheduler import (
    CentralCorridorScheduler,
    CorridorSchedule,
    CorridorSchedulerConfig,
)


def _manager_wall_time() -> float:
    """Read the compatibility clock exposed by ``core.manager``."""
    from fleet_manager.core import manager as manager_module

    return float(getattr(manager_module, "time", _system_time)())


class FleetManagerRuntimeStateMixin:
    """Own long-lived fleet state, locks, clocks and reset boundaries."""

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
        wall_now = self._initialize_core_services(
            landmarks,
            edges,
            params=params,
            map_dir=map_dir,
            map_metadata=map_metadata,
            remote_adapter=remote_adapter,
        )
        self._initialize_planning_runtime(wall_now)
        self._initialize_recovery_state()
        self._initialize_controlled_corridor_state()
        self._initialize_traffic_zone_state()
        self._initialize_traffic_metrics()

    def _initialize_core_services(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        *,
        params: dict[str, Any] | None,
        map_dir: Path | None,
        map_metadata: MapMetadata | None,
        remote_adapter: Any | None,
    ) -> float:
        """Create immutable inputs and the core planner/collision services."""

        self.landmarks = landmarks
        self.edges = edges
        self.params = params or {}
        self.settings = FleetSettings(self.params)
        wall_now = _manager_wall_time()
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
        return wall_now

    def _initialize_planning_runtime(self, wall_now: float) -> None:
        """Create locks and the single owned background planning worker."""

        self._route_revision_seq = int(wall_now * 1000)
        self._planner_lock = Lock()
        self._dispatch_job_lock = Lock()
        self._dispatch_job: dict[str, Any] | None = None
        self._planning_worker = PlanningWorker(name="fleet-mapf-worker")
        self._last_async_job_kind = ""

    def _initialize_recovery_state(self) -> None:
        """Create bounded state for retries, continuations and deadlocks."""

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

    def _initialize_controlled_corridor_state(self) -> None:
        """Build authored-corridor geometry, calendar and live indexes."""

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

    def _initialize_traffic_zone_state(self) -> None:
        """Create dynamic traffic-zone admission state."""

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

    def _initialize_traffic_metrics(self) -> None:
        """Create stable counters and the last exceptional transaction."""

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

    def close(self) -> None:
        """Discard and join the one owned background planning job."""

        with self._dispatch_job_lock:
            job = self._dispatch_job
            if job is not None:
                job["discard"] = True

        self._planning_worker.close()

        with self._dispatch_job_lock:
            if self._dispatch_job is job:
                self._dispatch_job = None

        if isinstance(job, dict):
            corridor_gates = job.get("corridor_gates")
            self._release_controlled_corridor_gate_pins(
                corridor_gates
                if isinstance(corridor_gates, dict)
                else None
            )

    def _build_controlled_corridor_region_bounds(
        self,
        graph: Any,
    ) -> dict[str, tuple[float, float, float, float]]:
        """Approximate authored rectangle bounds from annotated vertices."""
        padding = self.settings.fleet.number(
            "controlled_corridor_footprint_zone_padding_m",
            0.2,
            minimum=0.0,
        )
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
        def value(name: str, default: float) -> float:
            return self.settings.fleet.number(
                name,
                default,
                minimum=0.0,
            )

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
        return self.settings.fleet.integer(
            "terminal_order_history_limit",
            120,
            minimum=0,
            maximum=5000,
            default_if_falsy=True,
        )

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
            wall_now = _manager_wall_time()
            elapsed = max(0.0, wall_now - self._simulation_clock_wall_at)
            self._simulation_clock += elapsed * self._simulation_time_scale
            self._simulation_clock_wall_at = wall_now
            self._simulation_time_scale = requested
        return requested

    def _now(self) -> float:
        with self._simulation_clock_lock:
            wall_now = _manager_wall_time()
            elapsed = max(0.0, wall_now - self._simulation_clock_wall_at)
            self._simulation_clock += elapsed * self._simulation_time_scale
            self._simulation_clock_wall_at = wall_now
            return self._simulation_clock

    def _configured_simulation_time_scale(self) -> float:
        return self.settings.fleet.number(
            "simulation_time_scale",
            1.0,
            minimum=1.0,
            maximum=self._simulation_time_scale_limit(),
            default_if_falsy=True,
        )

    def _simulation_time_scale_limit(self) -> float:
        return self.settings.fleet.number(
            "simulation_time_scale_max",
            self.MAX_SIMULATION_TIME_SCALE,
            minimum=1.0,
            maximum=self.MAX_SIMULATION_TIME_SCALE,
            default_if_falsy=True,
        )



__all__ = ["FleetManagerRuntimeStateMixin"]
