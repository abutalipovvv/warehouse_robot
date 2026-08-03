"""Runtime motion snapshots, rollback and collision invariants."""

from __future__ import annotations

from dataclasses import dataclass
import math
from time import time
from typing import Any

from fleet_manager.core.domain.models import FleetRobot


@dataclass(slots=True)
class _UnsafeMotion:
    """Robot pairs whose proposed movement must be rolled back together."""

    robot_names: set[str]
    pairs: list[tuple[str, str]]
    details: list[dict[str, Any]]


class FleetMotionSafetyMixin:
    """Commit a runtime tick only when robot motion remains collision-safe."""

    def _runtime_safety_snapshot(self, robot: FleetRobot) -> dict[str, Any]:
        active_order = self.orders.get(robot.active_order_id) if robot.active_order_id else None
        return {
            "pose": dict(robot.pose) if robot.pose is not None else None,
            "current_lm": robot.current_lm,
            "target_lm": robot.target_lm,
            "status": robot.status,
            "trajectory": robot.trajectory,
            "plan_nodes": robot.plan_nodes,
            "route_started_at": robot.route_started_at,
            "route_clock": robot.route_clock,
            "last_reason": robot.last_reason,
            "route_note": robot.route_note,
            "blocked_since": robot.blocked_since,
            "last_replan_at": robot.last_replan_at,
            "route_revision": robot.route_revision,
            "route_chunk_index": robot.route_chunk_index,
            "route_chunk_goal_lm": robot.route_chunk_goal_lm,
            "route_final_lm": robot.route_final_lm,
            "route_preview": robot.route_preview,
            "route_preview_dirty": robot.route_preview_dirty,
            "has_executed_route": robot.has_executed_route,
            "pending_route": robot.pending_route,
            "retreat_target_clock": robot.retreat_target_clock,
            "retreat_target_lm": robot.retreat_target_lm,
            "retreat_blocked_edges": list(robot.retreat_blocked_edges),
            "retreat_blocker_signatures": list(
                robot.retreat_blocker_signatures
            ),
            "retreat_corridor_hold": (
                dict(robot.retreat_corridor_hold)
                if isinstance(robot.retreat_corridor_hold, dict)
                else None
            ),
            "traffic_priority_until": robot.traffic_priority_until,
            "wait_for_robot": robot.wait_for_robot,
            "wait_resource": robot.wait_resource,
            "wait_release_at": robot.wait_release_at,
            "traffic_stall_since": robot.traffic_stall_since,
            "active_order_id": robot.active_order_id,
            "active_order": (
                {
                    "status": active_order.status,
                    "updated_at": active_order.updated_at,
                    "assigned_robot": active_order.assigned_robot,
                    "start_lm": active_order.start_lm,
                    "route_nodes": list(active_order.route_nodes),
                    "error": active_order.error,
                    "target_lm": active_order.target_lm,
                    "step_index": active_order.step_index,
                    "spatial_route_nodes": list(active_order.spatial_route_nodes),
                    "spatial_route_revision": active_order.spatial_route_revision,
                    "traffic_blocked_since": active_order.traffic_blocked_since,
                    "traffic_detour_edges": list(active_order.traffic_detour_edges),
                    "traffic_detour_attempts": active_order.traffic_detour_attempts,
                }
                if active_order is not None
                else None
            ),
        }

    def _restore_runtime_safety_snapshot(
        self,
        robot: FleetRobot,
        snapshot: dict[str, Any],
        now: float,
    ) -> None:
        robot.pose = dict(snapshot["pose"]) if snapshot["pose"] is not None else None
        robot.current_lm = str(snapshot["current_lm"])
        robot.target_lm = str(snapshot["target_lm"])
        robot.status = str(snapshot["status"])
        robot.trajectory = snapshot["trajectory"]
        robot.plan_nodes = snapshot["plan_nodes"]
        robot.route_started_at = snapshot["route_started_at"]
        robot.route_clock = float(snapshot["route_clock"])
        robot.last_reason = str(snapshot["last_reason"])
        robot.route_note = str(snapshot["route_note"])
        robot.blocked_since = snapshot["blocked_since"]
        robot.last_replan_at = snapshot["last_replan_at"]
        robot.route_revision = int(snapshot["route_revision"])
        robot.route_chunk_index = int(snapshot["route_chunk_index"])
        robot.route_chunk_goal_lm = str(snapshot["route_chunk_goal_lm"])
        robot.route_final_lm = str(snapshot["route_final_lm"])
        robot.route_preview = snapshot["route_preview"]
        robot.route_preview_dirty = bool(snapshot["route_preview_dirty"])
        robot.has_executed_route = bool(snapshot.get("has_executed_route", False))
        robot.pending_route = snapshot.get("pending_route")
        robot.retreat_target_clock = snapshot.get("retreat_target_clock")
        robot.retreat_target_lm = str(snapshot.get("retreat_target_lm") or "")
        robot.retreat_blocked_edges = list(
            snapshot.get("retreat_blocked_edges", [])
        )
        robot.retreat_blocker_signatures = list(
            snapshot.get("retreat_blocker_signatures", [])
        )
        raw_corridor_hold = snapshot.get("retreat_corridor_hold")
        robot.retreat_corridor_hold = (
            dict(raw_corridor_hold)
            if isinstance(raw_corridor_hold, dict)
            else None
        )
        robot.traffic_priority_until = float(snapshot["traffic_priority_until"])
        robot.wait_for_robot = str(snapshot.get("wait_for_robot") or "")
        robot.wait_resource = str(snapshot.get("wait_resource") or "")
        robot.wait_release_at = float(snapshot.get("wait_release_at", 0.0) or 0.0)
        robot.traffic_stall_since = snapshot.get("traffic_stall_since")
        robot.active_order_id = str(snapshot["active_order_id"])
        order_snapshot = snapshot.get("active_order")
        order = self.orders.get(robot.active_order_id) if robot.active_order_id else None
        if order is not None and isinstance(order_snapshot, dict):
            order.status = str(order_snapshot["status"])
            order.updated_at = float(order_snapshot["updated_at"])
            order.assigned_robot = str(order_snapshot["assigned_robot"])
            order.start_lm = str(order_snapshot["start_lm"])
            order.route_nodes = list(order_snapshot["route_nodes"])
            order.error = str(order_snapshot["error"])
            order.target_lm = str(order_snapshot["target_lm"])
            order.step_index = int(order_snapshot["step_index"])
            order.spatial_route_nodes = list(order_snapshot.get("spatial_route_nodes", []))
            order.spatial_route_revision = int(
                order_snapshot.get("spatial_route_revision", 0) or 0
            )
            order.traffic_blocked_since = order_snapshot.get("traffic_blocked_since")
            order.traffic_detour_edges = list(
                order_snapshot.get("traffic_detour_edges", [])
            )
            order.traffic_detour_attempts = int(
                order_snapshot.get("traffic_detour_attempts", 0) or 0
            )
        robot.last_tick_at = now
        robot.trajectory_dirty = True
        robot.updated_at = now

    def _runtime_safety_telemetry_context(
        self,
        robot: FleetRobot,
        snapshot: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Return compact route evidence for one rollback endpoint."""
        source = snapshot if snapshot is not None else {}
        pose = source.get("pose") if snapshot is not None else robot.pose
        trajectory = (
            source.get("trajectory")
            if snapshot is not None
            else robot.trajectory
        )
        if not isinstance(trajectory, list):
            trajectory = []
        try:
            route_clock = float(
                source.get("route_clock", robot.route_clock)
                if snapshot is not None
                else robot.route_clock
            )
        except (TypeError, ValueError):
            route_clock = 0.0
        try:
            route_revision = int(
                source.get("route_revision", robot.route_revision)
                if snapshot is not None
                else robot.route_revision
            )
        except (TypeError, ValueError):
            route_revision = 0
        return {
            "pose": dict(pose) if isinstance(pose, dict) else None,
            "currentLm": str(
                source.get("current_lm", robot.current_lm)
                if snapshot is not None
                else robot.current_lm
            ),
            "status": str(
                source.get("status", robot.status)
                if snapshot is not None
                else robot.status
            ),
            "routeClock": route_clock,
            "routeRevision": route_revision,
            "edgeId": self._edge_id_at_trajectory(
                trajectory,
                route_clock,
            ),
        }

    def _enforce_runtime_safety_invariant(
        self,
        snapshots: dict[str, dict[str, Any]],
        now: float,
    ) -> None:
        unsafe = self._find_unsafe_runtime_motion(snapshots)
        if not unsafe.robot_names:
            return
        self._rollback_runtime_safety_transaction(
            unsafe,
            snapshots,
            now,
        )
        remaining_pairs = self._resolve_stationary_safety_pairs(
            unsafe.pairs,
            now,
        )
        self._resolve_runtime_safety_components(remaining_pairs, now)

    def _find_unsafe_runtime_motion(
        self,
        snapshots: dict[str, dict[str, Any]],
    ) -> _UnsafeMotion:
        """Detect endpoint and swept-body conflicts introduced this tick."""

        robots = [
            robot
            for robot in self._runtime_robots()
            if (
                not robot.is_remote()
                and robot.pose is not None
                and robot.name in snapshots
            )
        ]
        unsafe_names: set[str] = set()
        unsafe_pairs: list[tuple[str, str]] = []
        unsafe_pair_details: list[dict[str, Any]] = []
        for index, robot in enumerate(robots):
            for other in robots[index + 1:]:
                previous_robot_pose = snapshots[robot.name].get("pose")
                previous_other_pose = snapshots[other.name].get("pose")
                endpoint_overlap = self.collision.footprints_overlap(
                    robot.pose,
                    other.pose,
                )
                swept_overlap = self._swept_footprints_overlap(
                    previous_robot_pose,
                    robot.pose,
                    previous_other_pose,
                    other.pose,
                )
                if not endpoint_overlap and not swept_overlap:
                    continue
                if (
                    previous_robot_pose is not None
                    and previous_other_pose is not None
                    and self.collision.footprints_overlap(
                        previous_robot_pose,
                        previous_other_pose,
                    )
                ):
                    # The invariant cannot repair an overlap that existed
                    # before this tick; do not create an endless rollback loop.
                    continue
                unsafe_names.update((robot.name, other.name))
                unsafe_pairs.append((robot.name, other.name))
                unsafe_pair_details.append({
                    "robots": [robot.name, other.name],
                    "kind": (
                        "both"
                        if endpoint_overlap and swept_overlap
                        else "endpoint"
                        if endpoint_overlap
                        else "swept"
                    ),
                    "before": {
                        robot.name: self._runtime_safety_telemetry_context(
                            robot,
                            snapshots[robot.name],
                        ),
                        other.name: self._runtime_safety_telemetry_context(
                            other,
                            snapshots[other.name],
                        ),
                    },
                    "proposed": {
                        robot.name: self._runtime_safety_telemetry_context(
                            robot
                        ),
                        other.name: self._runtime_safety_telemetry_context(
                            other
                        ),
                    },
                })
        return _UnsafeMotion(
            robot_names=unsafe_names,
            pairs=unsafe_pairs,
            details=unsafe_pair_details,
        )

    def _rollback_runtime_safety_transaction(
        self,
        unsafe: _UnsafeMotion,
        snapshots: dict[str, dict[str, Any]],
        now: float,
    ) -> None:
        """Atomically restore all robots involved in one unsafe frame."""

        involved = [
            self.robots[name] for name in sorted(unsafe.robot_names)
        ]
        for robot in involved:
            self._restore_runtime_safety_snapshot(
                robot,
                snapshots[robot.name],
                now,
            )

        # One call is one atomic rollback transaction, even when several
        # disconnected pairs crossed their safety envelopes in the same
        # physics frame.  Every component is resolved below; counting the
        # frame once preserves the metric's historical meaning.
        rollback_sequence = (
            int(self.traffic_metrics["runtimeSafetyRollbacks"]) + 1
        )
        self.traffic_metrics["runtimeSafetyRollbacks"] = rollback_sequence
        self._last_runtime_safety_rollback = {
            "sequence": rollback_sequence,
            "stamp": time(),
            "simulationStamp": float(now),
            "pairCount": len(unsafe.details),
            "pairs": unsafe.details,
        }

    def _resolve_stationary_safety_pairs(
        self,
        unsafe_pairs: list[tuple[str, str]],
        now: float,
    ) -> list[tuple[str, str]]:
        """Evacuate motion-vs-stationary conflicts before assigning priority."""

        remaining_pairs = list(unsafe_pairs)
        attempted_stationary_pairs: set[tuple[str, str]] = set()
        while True:
            handled_mover = ""
            for pair in remaining_pairs:
                if pair in attempted_stationary_pairs:
                    continue
                attempted_stationary_pairs.add(pair)
                handled_mover = self._resolve_stationary_safety_pair(
                    pair,
                    remaining_pairs,
                    now,
                )
                if handled_mover:
                    break
            if not handled_mover:
                break
            remaining_pairs = [
                pair
                for pair in remaining_pairs
                if handled_mover not in pair
            ]
        return remaining_pairs

    def _resolve_stationary_safety_pair(
        self,
        pair: tuple[str, str],
        remaining_pairs: list[tuple[str, str]],
        now: float,
    ) -> str:
        first = self.robots[pair[0]]
        second = self.robots[pair[1]]
        if bool(first.trajectory) == bool(second.trajectory):
            return ""
        blocker = second if first.trajectory else first
        mover = first if first.trajectory else second
        if mover.retreat_target_clock is not None:
            return self._replace_blocked_safety_retreat(
                mover,
                blocker,
                remaining_pairs,
                now,
            )
        evacuated_name = self._start_deadlock_corridor_evacuation(
            [blocker, mover],
            blocker,
            now,
        )
        if not evacuated_name:
            return ""
        evacuated = self.robots.get(evacuated_name)
        if evacuated is not None and evacuated.status == "RETREATING":
            evacuated.traffic_priority_until = max(
                evacuated.traffic_priority_until,
                now + self._deadlock_priority_lease(),
            )
        related_pairs = [
            unsafe_pair
            for unsafe_pair in remaining_pairs
            if evacuated_name in unsafe_pair
        ]
        self._event(
            "error",
            "runtime safety invariant prevented footprint overlap: "
            f"{self._runtime_safety_pair_text(related_pairs)}; rolled back, "
            f"evacuating {evacuated_name}",
        )
        return evacuated_name

    def _replace_blocked_safety_retreat(
        self,
        mover: FleetRobot,
        blocker: FleetRobot,
        remaining_pairs: list[tuple[str, str]],
        now: float,
    ) -> str:
        """Replace an unsafe reverse step with a detour or bounded hold."""

        order = self._active_order_for_robot(mover)
        blocked_edges = list(mover.retreat_blocked_edges)
        if order is not None and blocked_edges:
            order.traffic_detour_edges = list(dict.fromkeys(blocked_edges))
        related_pairs = [
            pair for pair in remaining_pairs if mover.name in pair
        ]
        replan_handled, replan_started = (
            self._queue_background_replan_recovery_action(
                mover,
                now,
                "deadlock retreat blocked; alternate route required",
            )
        )
        if replan_handled:
            if order is not None and blocked_edges and replan_started:
                order.traffic_detour_attempts += 1
            self._clear_deadlock_retreat(mover)
            if replan_started:
                self.traffic_metrics["cycleReplans"] += 1
                self._event(
                    "error",
                    "runtime safety invariant prevented footprint overlap: "
                    f"{self._runtime_safety_pair_text(related_pairs)}; "
                    f"blocked retreat for {mover.name} replaced with "
                    "alternate route",
                )
            return mover.name

        lease_until = now + max(1.0, self._deadlock_priority_lease())
        mover.status = "WAITING"
        mover.last_reason = f"yield to {blocker.name}"
        mover.wait_for_robot = blocker.name
        mover.wait_resource = "blocked_retreat"
        mover.wait_release_at = lease_until
        mover.traffic_priority_until = 0.0
        mover.blocked_since = mover.blocked_since or now
        mover.traffic_stall_since = mover.traffic_stall_since or now
        mover.last_tick_at = now
        mover.updated_at = now
        blocker.traffic_priority_until = max(
            blocker.traffic_priority_until,
            lease_until,
        )
        self._update_active_order_from_robot(mover)
        self._event(
            "error",
            "runtime safety invariant prevented footprint overlap: "
            f"{self._runtime_safety_pair_text(related_pairs)}; blocked "
            f"retreat for {mover.name} held for {blocker.name}",
        )
        return mover.name

    def _resolve_runtime_safety_components(
        self,
        remaining_pairs: list[tuple[str, str]],
        now: float,
    ) -> None:
        """Assign one deterministic priority winner per conflict component."""

        adjacency: dict[str, set[str]] = {}
        for first_name, second_name in remaining_pairs:
            adjacency.setdefault(first_name, set()).add(second_name)
            adjacency.setdefault(second_name, set()).add(first_name)
        unseen = set(adjacency)
        while unseen:
            root = min(unseen)
            component_names: set[str] = set()
            stack = [root]
            while stack:
                name = stack.pop()
                if name in component_names:
                    continue
                component_names.add(name)
                stack.extend(adjacency.get(name, set()) - component_names)
            unseen.difference_update(component_names)
            component_pairs = [
                pair for pair in remaining_pairs
                if pair[0] in component_names and pair[1] in component_names
            ]
            component = [
                self.robots[name] for name in sorted(component_names)
            ]
            winner = min(
                component,
                key=self._runtime_safety_priority_key,
            )
            winner.traffic_priority_until = (
                now + self._deadlock_priority_lease()
            )
            winner.status = "MOVING" if winner.trajectory else "WAITING"
            winner.last_reason = "runtime safety rollback; priority granted"
            winner.blocked_since = now
            winner.traffic_stall_since = winner.traffic_stall_since or now
            for robot in component:
                if robot.name == winner.name:
                    continue
                robot.status = "WAITING"
                robot.last_reason = f"yield to {winner.name}"
                robot.blocked_since = now
                robot.traffic_stall_since = robot.traffic_stall_since or now
                robot.wait_for_robot = winner.name
                robot.wait_resource = "runtime_safety"
                robot.wait_release_at = winner.traffic_priority_until
                self._update_active_order_from_robot(robot)
            self._update_active_order_from_robot(winner)
            self._event(
                "error",
                "runtime safety invariant prevented footprint overlap: "
                f"{self._runtime_safety_pair_text(component_pairs)}; "
                "rolled back, priority "
                f"{winner.name}",
            )

    def _runtime_safety_priority_key(
        self,
        robot: FleetRobot,
    ) -> tuple[int, int, str]:
        order = self._active_order_for_robot(robot)
        return (
            int(robot.retreat_target_clock is not None),
            -int(order.priority if order is not None else 0),
            robot.name,
        )

    @staticmethod
    def _runtime_safety_pair_text(
        pairs: list[tuple[str, str]],
    ) -> str:
        return ", ".join(f"{first}/{second}" for first, second in pairs)

    def _swept_footprints_overlap(
        self,
        first_start: dict[str, float] | None,
        first_end: dict[str, float],
        second_start: dict[str, float] | None,
        second_end: dict[str, float],
    ) -> bool:
        if first_start is None or second_start is None:
            return False

        relative_x = float(first_start.get("x", 0.0) or 0.0) - float(second_start.get("x", 0.0) or 0.0)
        relative_y = float(first_start.get("y", 0.0) or 0.0) - float(second_start.get("y", 0.0) or 0.0)
        relative_dx = (
            float(first_end.get("x", 0.0) or 0.0) - float(first_start.get("x", 0.0) or 0.0)
            - float(second_end.get("x", 0.0) or 0.0) + float(second_start.get("x", 0.0) or 0.0)
        )
        relative_dy = (
            float(first_end.get("y", 0.0) or 0.0) - float(first_start.get("y", 0.0) or 0.0)
            - float(second_end.get("y", 0.0) or 0.0) + float(second_start.get("y", 0.0) or 0.0)
        )
        relative_speed_sq = (relative_dx * relative_dx) + (relative_dy * relative_dy)
        closest_ratio = 0.0
        if relative_speed_sq > 0.000000001:
            closest_ratio = max(
                0.0,
                min(1.0, -((relative_x * relative_dx) + (relative_y * relative_dy)) / relative_speed_sq),
            )
        closest_distance = math.hypot(
            relative_x + (relative_dx * closest_ratio),
            relative_y + (relative_dy * closest_ratio),
        )
        if closest_distance > self.collision.robot_broadphase_distance():
            return False

        first_travel = math.hypot(
            float(first_end.get("x", 0.0) or 0.0) - float(first_start.get("x", 0.0) or 0.0),
            float(first_end.get("y", 0.0) or 0.0) - float(first_start.get("y", 0.0) or 0.0),
        )
        second_travel = math.hypot(
            float(second_end.get("x", 0.0) or 0.0) - float(second_start.get("x", 0.0) or 0.0),
            float(second_end.get("y", 0.0) or 0.0) - float(second_start.get("y", 0.0) or 0.0),
        )
        first_turn = abs(math.atan2(
            math.sin(float(first_end.get("yaw", 0.0) or 0.0) - float(first_start.get("yaw", 0.0) or 0.0)),
            math.cos(float(first_end.get("yaw", 0.0) or 0.0) - float(first_start.get("yaw", 0.0) or 0.0)),
        ))
        second_turn = abs(math.atan2(
            math.sin(float(second_end.get("yaw", 0.0) or 0.0) - float(second_start.get("yaw", 0.0) or 0.0)),
            math.cos(float(second_end.get("yaw", 0.0) or 0.0) - float(second_start.get("yaw", 0.0) or 0.0)),
        ))
        linear_samples = int(math.ceil(max(first_travel, second_travel) / 0.025))
        angular_samples = int(math.ceil(max(first_turn, second_turn) / 0.05))
        samples = max(2, min(40, max(linear_samples, angular_samples)))
        for index in range(1, samples):
            ratio = index / samples
            first_pose = self._interpolate_pose(first_start, first_end, ratio)
            second_pose = self._interpolate_pose(second_start, second_end, ratio)
            if self.collision.footprints_overlap(first_pose, second_pose):
                return True
        return False
__all__ = ["FleetMotionSafetyMixin"]
