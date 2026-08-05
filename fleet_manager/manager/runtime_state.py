"""FleetManager runtime state, clocks and owned worker lifecycle."""

from __future__ import annotations

import math
from functools import wraps
from typing import Any, Callable, TypeVar

from fleet_manager.manager.tasks.statuses import TERMINAL_ORDER_STATUSES
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.tasks.manager import FleetTaskManager
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorSchedulerConfig,
)

ResultT = TypeVar("ResultT")


def runtime_command(
    method: Callable[..., ResultT],
) -> Callable[..., ResultT]:
    """Route a public mutation through the attached runtime executor."""

    @wraps(method)
    def wrapped(self: Any, *args: Any, **kwargs: Any) -> ResultT:
        executor = getattr(self, "_runtime_command_executor", None)
        if executor is None:
            return method(self, *args, **kwargs)
        return executor(lambda: method(self, *args, **kwargs))

    return wrapped


def _state_property(container_name: str, field_name: str) -> property:
    """Route one legacy attribute to exactly one container field."""

    def read(instance: Any) -> Any:
        container = getattr(instance, container_name)
        return getattr(container, field_name)

    def write(instance: Any, value: Any) -> None:
        container = getattr(instance, container_name)
        setattr(container, field_name, value)

    return property(read, write)


class FleetManagerStateCompatibilityMixin:
    """Map legacy mixin attributes to the explicit state containers."""

    robots = _state_property("fleet_state", "robots")
    task_manager = _state_property("fleet_state", "task_manager")
    events = _state_property("fleet_state", "events")
    obstacles = _state_property("fleet_state", "obstacles")
    obstacle_areas = _state_property("fleet_state", "obstacle_areas")
    active_robot_modes = _state_property(
        "fleet_state",
        "active_robot_modes",
    )

    _dispatch_job = _state_property("planning_state", "active_job")
    _last_async_job_kind = _state_property(
        "planning_state",
        "last_async_job_kind",
    )
    _runtime_replans = _state_property("planning_state", "runtime_replans")
    _rolling_prefetch_retry_at = _state_property(
        "planning_state",
        "rolling_prefetch_retry_at",
    )
    _rolling_prefetch_eligible_since = _state_property(
        "planning_state",
        "rolling_prefetch_eligible_since",
    )
    _rolling_prefetch_last_attempt_at = _state_property(
        "planning_state",
        "rolling_prefetch_last_attempt_at",
    )
    _stationary_order_retry_state = _state_property(
        "planning_state",
        "stationary_order_retry_state",
    )
    _dispatch_conflict_dependencies = _state_property(
        "planning_state",
        "dispatch_conflict_dependencies",
    )
    _rolling_prefetch_failures = _state_property(
        "planning_state",
        "rolling_prefetch_failures",
    )
    _rolling_prefetch_blockers = _state_property(
        "planning_state",
        "rolling_prefetch_blockers",
    )

    _stationary_clearance_relocations = _state_property(
        "recovery_state",
        "stationary_clearance_relocations",
    )
    _rolling_vacancy_recovery_signature = _state_property(
        "recovery_state",
        "rolling_vacancy_signature",
    )
    _rolling_vacancy_recovery_blacklist = _state_property(
        "recovery_state",
        "rolling_vacancy_blacklist",
    )
    _commanded_sink_vacancy_signatures = _state_property(
        "recovery_state",
        "commanded_vacancy_signatures",
    )
    _commanded_sink_vacancy_blacklist = _state_property(
        "recovery_state",
        "commanded_vacancy_blacklist",
    )
    _coupled_replan_last_attempt = _state_property(
        "recovery_state",
        "coupled_replan_last_attempt",
    )
    _coupled_replan_failures = _state_property(
        "recovery_state",
        "coupled_replan_failures",
    )
    _active_wait_cycles = _state_property("recovery_state", "active_wait_cycles")
    _wait_cycle_last_arbitration = _state_property(
        "recovery_state",
        "wait_cycle_last_arbitration",
    )
    _wait_cycle_grant_signatures = _state_property(
        "recovery_state",
        "wait_cycle_grant_signatures",
    )
    _wait_cycle_recovery_attempts = _state_property(
        "recovery_state",
        "wait_cycle_recovery_attempts",
    )
    _controlled_corridor_recovery_latches = _state_property(
        "recovery_state",
        "corridor_recovery_latches",
    )

    _controlled_corridor_graph = _state_property(
        "traffic_state",
        "controlled_corridor_graph",
    )
    _controlled_corridor_region_bounds = _state_property(
        "traffic_state",
        "controlled_corridor_region_bounds",
    )
    _controlled_corridor_scheduler = _state_property(
        "traffic_state",
        "controlled_corridor_scheduler",
    )
    _controlled_corridor_schedule = _state_property(
        "traffic_state",
        "controlled_corridor_schedule",
    )
    _controlled_corridor_wait_since = _state_property(
        "traffic_state",
        "controlled_corridor_wait_since",
    )
    _controlled_corridor_leases = _state_property(
        "traffic_state",
        "controlled_corridor_leases",
    )
    _controlled_corridor_passages = _state_property(
        "traffic_state",
        "controlled_corridor_passages",
    )
    _controlled_corridor_prefetch_intents = _state_property(
        "traffic_state",
        "controlled_corridor_prefetch_intents",
    )
    _controlled_corridor_entry_cache = _state_property(
        "traffic_state",
        "controlled_corridor_entry_cache",
    )
    _controlled_corridor_approach_holds = _state_property(
        "traffic_state",
        "controlled_corridor_approach_holds",
    )
    _controlled_corridor_winners = _state_property(
        "traffic_state",
        "controlled_corridor_winners",
    )
    _controlled_corridor_occupancy = _state_property(
        "traffic_state",
        "controlled_corridor_occupancy",
    )
    _controlled_corridor_queues = _state_property(
        "traffic_state",
        "controlled_corridor_queues",
    )
    _controlled_corridor_blockers = _state_property(
        "traffic_state",
        "controlled_corridor_blockers",
    )
    _controlled_corridor_tick_now = _state_property(
        "traffic_state",
        "controlled_corridor_tick_now",
    )

    _traffic_zone_by_lm = _state_property("traffic_state", "traffic_zone_by_lm")
    _traffic_zone_wait_since = _state_property(
        "traffic_state",
        "traffic_zone_wait_since",
    )
    _traffic_zone_leases = _state_property(
        "traffic_state",
        "traffic_zone_leases",
    )
    _traffic_zone_phase = _state_property("traffic_state", "traffic_zone_phase")
    _traffic_zone_emergency_until = _state_property(
        "traffic_state",
        "traffic_zone_emergency_until",
    )
    _traffic_zone_winners = _state_property(
        "traffic_state",
        "traffic_zone_winners",
    )
    _traffic_zone_demand = _state_property("traffic_state", "traffic_zone_demand")
    _traffic_zone_occupancy = _state_property(
        "traffic_state",
        "traffic_zone_occupancy",
    )
    _traffic_zone_queues = _state_property("traffic_state", "traffic_zone_queues")
    _traffic_zone_tick_now = _state_property(
        "traffic_state",
        "traffic_zone_tick_now",
    )
    traffic_metrics = _state_property("traffic_state", "metrics")
    _last_runtime_safety_rollback = _state_property(
        "traffic_state",
        "last_runtime_safety_rollback",
    )


class FleetManagerRuntimeStateMixin(FleetManagerStateCompatibilityMixin):
    """Own long-lived fleet state, locks, clocks and reset boundaries."""

    @property
    def planning_revision(self) -> int:
        """Current version of data that can invalidate a plan."""

        return self.fleet_state.revision.value

    def _advance_planning_revision(self, reason: str) -> int:
        """Record one planning-relevant state transition."""

        revision = self.fleet_state.revision.advance(reason)
        if hasattr(self, "_planning_input_fingerprint"):
            self._planning_input_fingerprint = (
                self._planning_state_fingerprint()
            )
        return revision

    def _synchronize_planning_revision(self) -> int:
        """Detect planning-relevant mutations made by legacy mixins."""

        current = self._planning_state_fingerprint()
        previous = getattr(self, "_planning_input_fingerprint", current)
        if current == previous:
            return self.planning_revision
        self._planning_input_fingerprint = current
        return self.fleet_state.revision.advance(
            "planning input changed during runtime step"
        )

    def _planning_state_fingerprint(self) -> tuple[Any, ...]:
        """Compact deterministic identity of inputs that can stale a plan."""

        corridor_lease_owners = tuple(sorted(
            (
                str(region_id),
                str(lease[0]),
            )
            for region_id, lease in (
                self.traffic_state.controlled_corridor_leases.items()
            )
            if isinstance(lease, tuple) and lease
        ))
        zone_lease_owners = tuple(sorted(
            (
                str(zone_id),
                str(robot_name),
            )
            for zone_id, robot_name in (
                self.traffic_state.traffic_zone_leases.keys()
            )
        ))
        robot_values = tuple(
            self._robot_planning_fingerprint(robot)
            for robot in sorted(self.robots.values(), key=lambda item: item.name)
        )
        active_order_ids = {
            robot.active_order_id
            for robot in self.robots.values()
            if robot.active_order_id
        }
        order_values = tuple(
            (
                order.order_id,
                (
                    "ACTIVE"
                    if order.order_id in active_order_ids
                    and order.status
                    not in TERMINAL_ORDER_STATUSES | {"PAUSED"}
                    else order.status
                ),
                order.vehicle,
                order.assigned_robot,
                order.target_lm,
                tuple(order.targets),
                int(order.step_index),
                int(order.spatial_route_revision),
            )
            for order in sorted(
                self.task_manager.orders.values(),
                key=lambda item: item.order_id,
            )
        )
        schedule = self.traffic_state.controlled_corridor_schedule
        return (
            robot_values,
            order_values,
            tuple(sorted(str(item) for item in self.obstacles)),
            tuple(sorted(str(item) for item in self.obstacle_areas)),
            tuple(sorted(self.traffic_state.stationary_blockers.items())),
            # Lease expiry is a runtime heartbeat, not a new ownership
            # decision. Including the renewed timestamp here invalidated every
            # background result in a busy fleet. Creation, removal and owner
            # handoff still change these owner-only signatures.
            corridor_lease_owners,
            zone_lease_owners,
            tuple(getattr(schedule, "slots", ()) or ()),
        )

    @staticmethod
    def _robot_planning_fingerprint(robot: FleetRobot) -> tuple[Any, ...]:
        committed_motion = bool(
            not robot.is_remote()
            and robot.active_order_id
            and robot.trajectory
            and robot.status in {"MOVING", "WAITING", "PLANNING"}
        )
        location_state: tuple[Any, ...]
        if committed_motion:
            # Route-clock progress and the boundary WAITING transition are
            # deterministic consequences of the committed trajectory. The
            # continuation snapshot already contains that trajectory and its
            # route revision, so invalidating it here would make every rolling
            # prefetch stale at exactly the handoff it was created for.
            location_state = ("committed_route",)
        else:
            location_state = (
                robot.status,
                robot.current_lm,
                robot.target_lm,
            )
        return (
            robot.name,
            robot.mode,
            *location_state,
            robot.active_order_id,
            int(robot.route_revision),
            bool(robot.remote_online),
        )

    def set_runtime_command_executor(self, executor: Any | None) -> None:
        """Attach the synchronous command boundary owned by RuntimeLoop."""

        if executor is not None and not callable(executor):
            raise TypeError("runtime command executor must be callable")
        self._runtime_command_executor = executor

    @property
    def orders(self) -> dict[str, FleetOrder]:
        return self.task_manager.orders

    @orders.setter
    def orders(self, value: dict[str, FleetOrder]) -> None:
        if hasattr(self, "task_manager"):
            self.task_manager.replace_storage(value)
        else:  # Defensive compatibility for unusual subclass initializers.
            self.task_manager = FleetTaskManager(value)

    def close(self) -> None:
        """Discard and join the one owned background planning job."""

        with self._dispatch_job_lock:
            job = self._dispatch_job
            if job is not None:
                job.discard = True

        self._planning_worker.close()

        with self._dispatch_job_lock:
            if self._dispatch_job is job:
                self._dispatch_job = None

        if job is not None:
            corridor_gates = job.corridor_gates
            self._release_controlled_corridor_gate_pins(
                corridor_gates or None
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


    @runtime_command
    def set_active_robot_modes(self, modes: set[str] | list[str] | tuple[str, ...] | None) -> None:
        previous = self.active_robot_modes
        if modes is None:
            self.active_robot_modes = None
        else:
            clean_modes = {
                str(mode or "").strip().lower()
                for mode in modes
                if str(mode or "").strip()
            }
            self.active_robot_modes = clean_modes or None
        if self.active_robot_modes != previous:
            self._advance_planning_revision("active robot modes changed")

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

    @runtime_command
    def reset_planning_runtime_state(self) -> None:
        """Reset transient planner/arbitration state without racing a worker.

        Mark the current result stale and signal cooperative cancellation.
        The dispatcher still discards a solver that cannot stop immediately.
        """
        cancelled_job_id = ""
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                if self._dispatch_job.kind == "dispatch":
                    for entry in self._dispatch_job.entries:
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
                self._dispatch_job.discard = True
                cancelled_job_id = self._dispatch_job.job_id
        if cancelled_job_id:
            self._planning_worker.cancel_job(cancelled_job_id)
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
        self.recovery_state.quarantined_robots.clear()
        self.recovery_state.recovery_cooldowns.clear()
        self.traffic_state.temporal_reservations.clear()
        self.traffic_state.stationary_blockers.clear()
        self.planning_state.stale_candidates = 0
        self.planning_state.committed_candidates = 0
        self.planning_state.diagnostic_counts.clear()
        self._runtime_tick_route_clocks.clear()
        self.reset_traffic_flow_state()
        self._advance_planning_revision("planning runtime state reset")

    def simulation_time(self) -> float:
        """Return the accelerated clock used by simulated fleet runtime."""
        return self._now()

    def simulation_time_scale(self) -> float:
        with self._simulation_clock_lock:
            return self._simulation_time_scale

    @runtime_command
    def set_simulation_time_scale(self, value: Any) -> float:
        try:
            requested = float(value)
        except (TypeError, ValueError):
            requested = 1.0
        if not math.isfinite(requested):
            requested = 1.0
        maximum = self._simulation_time_scale_limit()
        requested = max(1.0, min(maximum, requested))
        previous = self.simulation_time_scale()
        with self._simulation_clock_lock:
            wall_now = self._wall_time()
            elapsed = max(0.0, wall_now - self._simulation_clock_wall_at)
            self._simulation_clock += elapsed * self._simulation_time_scale
            self._simulation_clock_wall_at = wall_now
            self._simulation_time_scale = requested
        if requested != previous:
            self._advance_planning_revision("simulation time scale changed")
        return requested

    def _now(self) -> float:
        with self._simulation_clock_lock:
            wall_now = self._wall_time()
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
