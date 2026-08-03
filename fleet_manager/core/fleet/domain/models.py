"""Transport-independent state used by simulation and real fleets."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import time
from typing import Any


@dataclass
class FleetRobot:
    name: str
    current_lm: str
    mode: str = "simulated"
    target_lm: str = ""
    status: str = "IDLE"
    updated_at: float = field(default_factory=time)
    pose: dict[str, float] | None = None
    trajectory: list[dict[str, Any]] = field(default_factory=list)
    plan_nodes: list[str] = field(default_factory=list)
    route_started_at: float | None = None
    route_clock: float = 0.0
    last_tick_at: float | None = None
    last_reason: str = ""
    route_note: str = ""
    blocked_since: float | None = None
    last_replan_at: float | None = None
    trajectory_dirty: bool = False
    active_order_id: str = ""
    base_url: str = ""
    remote_id: str = ""
    remote_online: bool = True
    remote_error: str = ""
    remote_status: dict[str, Any] = field(default_factory=dict)
    remote_last_poll_at: float | None = None
    route_revision: int = 0
    route_chunk_index: int = 0
    route_chunk_goal_lm: str = ""
    route_final_lm: str = ""
    route_preview: list[dict[str, Any]] = field(default_factory=list)
    route_preview_dirty: bool = False
    has_executed_route: bool = False
    # First instant at which this robot exhausted a rolling chunk. Keep this
    # separate from ``updated_at``: runtime status synchronization refreshes
    # that timestamp on every physics tick and would make an old waiter look
    # perpetually new to the continuation scheduler.
    rolling_boundary_since: float | None = None
    pending_route: dict[str, Any] | None = None
    retreat_target_clock: float | None = None
    retreat_target_lm: str = ""
    retreat_blocked_edges: list[tuple[str, str]] = field(default_factory=list)
    # Preserve the physical dependency which caused a reverse evacuation.
    # ``wait_for_robot`` is intentionally cleared while the robot retreats,
    # but the subsequent same-goal replan still needs the exact parked body
    # that made the old approach unusable.
    retreat_blocker_signatures: list[tuple[str, str, int]] = field(
        default_factory=list
    )
    # A robot moved out of a controlled-corridor portal must not immediately
    # replan back into the queue while the authoritative passage owner is
    # still crossing that portal.  The hold is transferred to the
    # transactional replan state when the reverse motion completes.
    retreat_corridor_hold: dict[str, Any] | None = None
    traffic_priority_until: float = 0.0
    wait_for_robot: str = ""
    wait_resource: str = ""
    wait_release_at: float = 0.0
    traffic_stall_since: float | None = None
    collision_preflight_revision: int = -1
    collision_preflight_due_at: float = 0.0

    def to_dict(self, include_trajectory: bool = True) -> dict[str, Any]:
        # The committed rolling target is internal. Operator clients should
        # continue to see the final task destination.
        display_target_lm = self.route_final_lm or self.target_lm
        return {
            "name": self.name,
            "currentLm": self.current_lm,
            "mode": self.mode,
            "type": self.mode,
            "targetName": display_target_lm,
            "targetLm": display_target_lm,
            "status": self.status,
            "updatedAt": self.updated_at,
            "pose": self.pose,
            "trajectory": (
                self.trajectory
                if include_trajectory or self.trajectory_dirty
                else []
            ),
            "planNodes": (
                self.plan_nodes
                if include_trajectory or self.trajectory_dirty
                else []
            ),
            "routeClock": self.route_clock,
            "reason": self.last_reason,
            "routeNote": self.route_note,
            "blockedSince": self.blocked_since,
            "lastReplanAt": self.last_replan_at,
            "activeOrderId": self.active_order_id,
            "baseUrl": self.base_url,
            "remoteId": self.remote_id,
            "online": self.remote_online,
            "remoteError": self.remote_error,
            "remoteStatus": self.remote_status,
            "remoteLastPollAt": self.remote_last_poll_at,
            "routeRevision": self.route_revision,
            "routeChunkIndex": self.route_chunk_index,
            "routeChunkGoalLm": self.route_chunk_goal_lm,
            "routeFinalLm": self.route_final_lm,
            "routePreview": (
                self.route_preview
                if include_trajectory or self.route_preview_dirty
                else []
            ),
            "rollingBoundarySince": self.rolling_boundary_since,
            "trafficPriorityUntil": self.traffic_priority_until,
            "waitDependency": (
                {
                    "robot": self.wait_for_robot,
                    "resource": self.wait_resource,
                    "releaseAt": self.wait_release_at,
                }
                if self.wait_for_robot
                else None
            ),
            "trafficStallSince": self.traffic_stall_since,
        }

    def is_remote(self) -> bool:
        return self.mode in {
            "remote",
            "robot",
            "real",
            "grpc",
            "aivison_grpc",
            "real_grpc",
        }


@dataclass
class FleetOrder:
    order_id: str
    target_lm: str
    vehicle: str = ""
    priority: int = 0
    status: str = "QUEUED"
    created_at: float = field(default_factory=time)
    updated_at: float = field(default_factory=time)
    assigned_robot: str = ""
    start_lm: str = ""
    route_nodes: list[str] = field(default_factory=list)
    error: str = ""
    external_id: str = ""
    targets: list[str] = field(default_factory=list)
    step_index: int = 0
    speed: float = 0.0
    acceleration: float = 0.0
    rotate: bool = False
    turn_speed: float = 0.0
    stretch_motion_to_reservation_ticks: bool = True
    dispatch_failures: int = 0
    traffic_detour_edges: list[tuple[str, str]] = field(default_factory=list)
    traffic_detour_attempts: int = 0
    spatial_route_nodes: list[str] = field(default_factory=list)
    spatial_route_revision: int = 0
    traffic_blocked_since: float | None = None
    # Core-only maintenance moves (for example clearing a traffic lane) use
    # the ordinary task/MAPF lifecycle but are intentionally absent from the
    # operator order feed and benchmark counters.
    internal_kind: str = ""

    def to_dict(self) -> dict[str, Any]:
        targets = self.targets or ([self.target_lm] if self.target_lm else [])
        current_step = max(0, min(self.step_index, max(0, len(targets) - 1)))
        current_target = targets[current_step] if targets else self.target_lm
        return {
            "id": self.order_id,
            "orderId": self.order_id,
            "externalId": self.external_id,
            "vehicle": self.vehicle,
            "targetLm": current_target,
            "targets": targets,
            "currentStep": current_step,
            "totalSteps": len(targets),
            "steps": self._steps_payload(targets, current_step),
            "priority": self.priority,
            "status": self.status,
            "createdAt": self.created_at,
            "updatedAt": self.updated_at,
            "assignedRobot": self.assigned_robot,
            "startLm": self.start_lm,
            "routeNodes": self.route_nodes,
            "error": self.error,
            "speed": self.speed,
            "acceleration": self.acceleration,
            "rotate": self.rotate,
            "turnSpeed": self.turn_speed,
            "stretchMotionToReservationTicks": (
                self.stretch_motion_to_reservation_ticks
            ),
            "dispatchFailures": self.dispatch_failures,
            "trafficDetourAttempts": self.traffic_detour_attempts,
            "trafficDetourEdges": [
                f"{src}->{dst}" for src, dst in self.traffic_detour_edges
            ],
            "spatialRouteNodes": self.spatial_route_nodes,
            "spatialRouteRevision": self.spatial_route_revision,
            "trafficBlockedSince": self.traffic_blocked_since,
        }

    def _steps_payload(
        self,
        targets: list[str],
        current_step: int,
    ) -> list[dict[str, Any]]:
        steps = []
        for index, target_lm in enumerate(targets):
            if self.status == "CANCELED" and index >= current_step:
                status = "CANCELED"
            elif self.status == "FAILED" and index >= current_step:
                status = "FAILED"
            elif self.status == "COMPLETED" or index < current_step:
                status = "COMPLETED"
            elif index == current_step:
                status = self.status
            else:
                status = "QUEUED"
            steps.append({
                "index": index,
                "targetLm": target_lm,
                "status": status,
            })
        return steps


@dataclass
class FleetEvent:
    stamp: float
    level: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp": self.stamp,
            "level": self.level,
            "message": self.message,
        }


__all__ = ["FleetEvent", "FleetOrder", "FleetRobot"]
