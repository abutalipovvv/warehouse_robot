"""Mutable Fleet Manager state owned by the runtime thread."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

from fleet_manager.core.fleet.domain.models import FleetEvent, FleetRobot
from fleet_manager.core.planning_models import (
    FrozenMapping,
    PlanningJob,
    PlanningSnapshot,
    ReservationSnapshot,
    RobotPlanningState,
    RoutePlanningState,
    TrafficResourceSnapshot,
)
from fleet_manager.core.tasks.manager import FleetTaskManager


@dataclass(slots=True)
class RevisionClock:
    """Monotonic version of state that affects planning."""

    value: int = 0
    last_reason: str = "initialized"

    def advance(self, reason: str) -> int:
        clean_reason = str(reason).strip()
        if not clean_reason:
            raise ValueError("revision reason must not be empty")
        self.value += 1
        self.last_reason = clean_reason
        return self.value


@dataclass(slots=True)
class FleetState:
    """Robot, task and operator-visible fleet state."""

    task_manager: FleetTaskManager = field(default_factory=FleetTaskManager)
    robots: dict[str, FleetRobot] = field(default_factory=dict)
    events: list[FleetEvent] = field(default_factory=list)
    obstacles: list[dict[str, float]] = field(default_factory=list)
    obstacle_areas: list[dict[str, float]] = field(default_factory=list)
    active_robot_modes: set[str] | None = None
    revision: RevisionClock = field(default_factory=RevisionClock)


@dataclass(slots=True)
class TrafficState:
    """Reservations, controlled corridors and traffic zones."""

    temporal_reservations: list[dict[str, Any]] = field(default_factory=list)
    stationary_blockers: dict[str, str] = field(default_factory=dict)

    controlled_corridor_graph: Any | None = None
    controlled_corridor_region_bounds: dict[
        str,
        tuple[float, float, float, float],
    ] = field(default_factory=dict)
    controlled_corridor_scheduler: Any | None = None
    controlled_corridor_schedule: Any | None = None
    controlled_corridor_wait_since: dict[
        tuple[str, str, int, str],
        float,
    ] = field(default_factory=dict)
    controlled_corridor_leases: dict[str, tuple[str, float]] = field(
        default_factory=dict
    )
    controlled_corridor_passages: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    controlled_corridor_prefetch_intents: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    controlled_corridor_entry_cache: dict[str, Any] = field(
        default_factory=dict
    )
    controlled_corridor_approach_holds: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    controlled_corridor_winners: dict[str, str] = field(default_factory=dict)
    controlled_corridor_occupancy: dict[str, list[str]] = field(
        default_factory=dict
    )
    controlled_corridor_queues: dict[str, list[str]] = field(
        default_factory=dict
    )
    controlled_corridor_blockers: dict[str, str] = field(default_factory=dict)
    controlled_corridor_tick_now: float = 0.0

    traffic_zone_by_lm: dict[str, Any] = field(default_factory=dict)
    traffic_zone_wait_since: dict[tuple[str, str], float] = field(
        default_factory=dict
    )
    traffic_zone_leases: dict[tuple[str, str], float] = field(
        default_factory=dict
    )
    traffic_zone_phase: dict[str, tuple[str, float]] = field(
        default_factory=dict
    )
    traffic_zone_emergency_until: dict[str, float] = field(
        default_factory=dict
    )
    traffic_zone_winners: dict[str, str] = field(default_factory=dict)
    traffic_zone_demand: dict[str, int] = field(default_factory=dict)
    traffic_zone_occupancy: dict[str, int] = field(default_factory=dict)
    traffic_zone_queues: dict[str, list[str]] = field(default_factory=dict)
    traffic_zone_tick_now: float = 0.0

    metrics: dict[str, int] = field(default_factory=dict)
    last_runtime_safety_rollback: dict[str, Any] | None = None


@dataclass(slots=True)
class PlanningState:
    """Planning jobs, replans and rolling-continuation latches."""

    active_job: dict[str, Any] | None = None
    last_async_job_kind: str = ""
    runtime_replans: dict[str, dict[str, Any]] = field(default_factory=dict)
    rolling_prefetch_retry_at: dict[str, float] = field(default_factory=dict)
    rolling_prefetch_eligible_since: dict[str, float] = field(
        default_factory=dict
    )
    rolling_prefetch_last_attempt_at: dict[str, float] = field(
        default_factory=dict
    )
    stationary_order_retry_state: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    dispatch_conflict_dependencies: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    rolling_prefetch_failures: dict[str, int] = field(default_factory=dict)
    rolling_prefetch_blockers: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    jobs: dict[str, PlanningJob] = field(default_factory=dict)
    stale_candidates: int = 0
    committed_candidates: int = 0
    submission_sequence: int = 0
    diagnostic_counts: dict[str, int] = field(default_factory=dict)

    def record_event(self, event_name: str) -> int:
        clean_name = str(event_name).strip()
        if not clean_name:
            raise ValueError("planning event name must not be empty")
        count = self.diagnostic_counts.get(clean_name, 0) + 1
        self.diagnostic_counts[clean_name] = count
        return count


@dataclass(slots=True)
class RecoveryState:
    """Deadlock information, retry history and recovery cooldowns."""

    stationary_clearance_relocations: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    rolling_vacancy_signature: tuple[tuple[str, str, int], ...] = ()
    rolling_vacancy_blacklist: set[Any] = field(default_factory=set)
    commanded_vacancy_signatures: dict[str, Any] = field(default_factory=dict)
    commanded_vacancy_blacklist: set[Any] = field(default_factory=set)
    coupled_replan_last_attempt: dict[tuple[str, ...], float] = field(
        default_factory=dict
    )
    coupled_replan_failures: dict[tuple[str, ...], int] = field(
        default_factory=dict
    )
    active_wait_cycles: dict[tuple[str, ...], float] = field(
        default_factory=dict
    )
    wait_cycle_last_arbitration: dict[tuple[str, ...], float] = field(
        default_factory=dict
    )
    wait_cycle_grant_signatures: dict[tuple[str, ...], Any] = field(
        default_factory=dict
    )
    wait_cycle_recovery_attempts: dict[Any, float] = field(
        default_factory=dict
    )
    corridor_recovery_latches: dict[Any, str] = field(default_factory=dict)
    quarantined_robots: set[str] = field(default_factory=set)
    recovery_cooldowns: dict[str, float] = field(default_factory=dict)


class PlanningSnapshotFactory:
    """Build immutable planning input from explicit state containers."""

    def __init__(
        self,
        fleet_state: FleetState,
        traffic_state: TrafficState,
    ) -> None:
        self.fleet_state = fleet_state
        self.traffic_state = traffic_state

    def create(
        self,
        *,
        created_at: float,
        requests: Iterable[Mapping[str, Any]],
        primary_payload: Mapping[str, Any],
        fallback_payload: Mapping[str, Any] | None = None,
        blockers: Iterable[str] = (),
        soft_blocked_lms: Iterable[str] = (),
        strict_stationary_avoidance: bool = True,
        reservation_offset: float = 0.0,
        held_snapshot_owners: Iterable[str] = (),
        release_owners: Iterable[str] = (),
        graph_revision: int | str | None = None,
        map_revision: int | str | None = None,
    ) -> PlanningSnapshot:
        robots = tuple(
            RobotPlanningState.from_robot(robot)
            for robot in sorted(
                self.fleet_state.robots.values(),
                key=lambda item: item.name,
            )
        )
        routes = tuple(
            RoutePlanningState.from_robot(robot)
            for robot in sorted(
                self.fleet_state.robots.values(),
                key=lambda item: item.name,
            )
            if robot.trajectory
        )
        return PlanningSnapshot(
            revision=self.fleet_state.revision.value,
            created_at=float(created_at),
            robots=robots,
            active_routes=routes,
            reservations=self._reservations(primary_payload),
            traffic_resources=self._traffic_resources(),
            blockers=tuple(sorted(str(item) for item in blockers)),
            graph_revision=graph_revision,
            map_revision=map_revision,
            requests=tuple(
                FrozenMapping.from_mapping(request)
                for request in requests
            ),
            primary_payload=FrozenMapping.from_mapping(primary_payload),
            fallback_payload=(
                FrozenMapping.from_mapping(fallback_payload)
                if fallback_payload is not None
                else None
            ),
            soft_blocked_lms=tuple(
                sorted(str(item) for item in soft_blocked_lms)
            ),
            strict_stationary_avoidance=bool(strict_stationary_avoidance),
            reservation_offset=float(reservation_offset),
            held_snapshot_owners=tuple(
                sorted(str(item) for item in held_snapshot_owners)
            ),
            release_owners=tuple(
                sorted(str(item) for item in release_owners)
            ),
        )

    @staticmethod
    def _reservations(
        payload: Mapping[str, Any],
    ) -> tuple[ReservationSnapshot, ...]:
        reservations: list[ReservationSnapshot] = []
        for item in payload.get("reserved_vertex_intervals", []):
            if not isinstance(item, Mapping):
                continue
            reservations.append(
                ReservationSnapshot(
                    kind="vertex",
                    owner=str(item.get("robot") or ""),
                    start=float(item.get("start", 0.0) or 0.0),
                    end=float(item.get("end", 0.0) or 0.0),
                    resource=(str(item.get("node") or ""),),
                )
            )
        for item in payload.get("reserved_edge_intervals", []):
            if not isinstance(item, Mapping):
                continue
            reservations.append(
                ReservationSnapshot(
                    kind="edge",
                    owner=str(item.get("robot") or ""),
                    start=float(item.get("start", 0.0) or 0.0),
                    end=float(item.get("end", 0.0) or 0.0),
                    resource=(
                        str(item.get("from") or ""),
                        str(item.get("to") or ""),
                    ),
                )
            )
        return tuple(reservations)

    def _traffic_resources(self) -> tuple[TrafficResourceSnapshot, ...]:
        resources: list[TrafficResourceSnapshot] = []
        for resource_id, lease in sorted(
            self.traffic_state.controlled_corridor_leases.items()
        ):
            owner, expires_at = lease
            resources.append(
                TrafficResourceSnapshot(
                    kind="controlled_corridor",
                    resource_id=str(resource_id),
                    owner=str(owner),
                    expires_at=float(expires_at),
                )
            )
        for (zone_id, owner), expires_at in sorted(
            self.traffic_state.traffic_zone_leases.items()
        ):
            resources.append(
                TrafficResourceSnapshot(
                    kind="traffic_zone",
                    resource_id=str(zone_id),
                    owner=str(owner),
                    expires_at=float(expires_at),
                )
            )
        return tuple(resources)
