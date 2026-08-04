"""Order models owned by Fleet Manager task orchestration."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import time
from typing import Any


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
    # Maintenance moves use the normal task lifecycle, but operator clients
    # and benchmark counters must not expose them as user orders.
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
                f"{source}->{target}"
                for source, target in self.traffic_detour_edges
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
