"""Fleet state snapshots, streaming payloads and world synchronization."""

from __future__ import annotations

from typing import Any

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot

from .runtime_state import runtime_command


class FleetManagerSnapshotMixin:
    """Serialize stable API snapshots without changing field order."""

    @runtime_command
    def state(self, include_trajectories: bool = True) -> dict[str, Any]:
        self._advance_runtime()
        return self._state_snapshot(include_trajectories=include_trajectories)

    @runtime_command
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

    @runtime_command
    def tick(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._advance_runtime()
        return self.stream_tick()

    @runtime_command
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

    @runtime_command
    def update_world(self, payload: dict[str, Any]) -> dict[str, Any]:
        obstacles = payload.get("obstacles", [])
        areas = payload.get("obstacleAreas", [])
        previous_obstacles = list(self.obstacles)
        previous_areas = list(self.obstacle_areas)
        previous_params = self.params
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
        world_changed = bool(
            self.obstacles != previous_obstacles
            or self.obstacle_areas != previous_areas
            or self.params is not previous_params
        )
        if world_changed:
            self._advance_planning_revision("world or planning params changed")
        if (
            len(self.obstacles) != len(previous_obstacles)
            or len(self.obstacle_areas) != len(previous_areas)
        ):
            self._event(
                "info",
                f"world synced: obstacles={counts[0]}, areas={counts[1]}",
            )
        return {"ok": True, "state": self.state()}

    def _configure_robot_gateway(self) -> None:
        """Let a transport-specific subclass rebuild its params-bound gateway."""


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
