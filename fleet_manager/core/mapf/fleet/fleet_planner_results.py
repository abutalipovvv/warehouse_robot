"""Normalize backend plans into the stable Fleet MAPF response contract."""

from __future__ import annotations

from typing import Any

from .fleet_planner_requests import PreparedFleetRequest


class PlanningResultFormatter:
    def __init__(self, planner: Any) -> None:
        self.planner = planner

    def format(
        self,
        prepared: PreparedFleetRequest,
        result: Any,
        *,
        used_blocked_edges: set[tuple[str, str]],
        used_reserved_detour: bool,
        fallback_reason: str,
    ) -> dict[str, Any]:
        del used_reserved_detour
        owner = self.planner
        plans = self._formatted_plans(prepared, result)
        deadlock = self._is_deadlock_result(result)
        traffic_graph = owner._traffic_graph(prepared.speed)
        return {
            "ok": bool(result.plans) or not prepared.requests,
            "debug": self._debug_payload(
                prepared,
                result,
                used_blocked_edges=used_blocked_edges,
                fallback_reason=fallback_reason,
                deadlock=deadlock,
                traffic_graph=traffic_graph,
            ),
            "timeStepSec": owner.time_step_sec,
            "plans": plans,
        }

    def _formatted_plans(
        self,
        prepared: PreparedFleetRequest,
        result: Any,
    ) -> list[dict[str, Any]]:
        """Preserve request order while omitting unresolved robot plans."""
        plans: list[dict[str, Any]] = []
        if not result.plans:
            return plans
        for request in prepared.requests:
            plan = result.plans.get(request.robot_name)
            if plan is not None:
                plans.append(self._formatted_plan(prepared, request, plan))
        return plans

    def _formatted_plan(
        self,
        prepared: PreparedFleetRequest,
        request: Any,
        plan: Any,
    ) -> dict[str, Any]:
        """Translate one backend plan into the public trajectory contract."""
        owner = self.planner
        trajectory = owner._trajectory_for_nodes(
            plan.nodes,
            prepared.speed,
            plan.times,
            acceleration=prepared.acceleration,
            rotate_enabled=prepared.rotate_enabled,
            turn_speed=prepared.turn_speed,
            stretch_motion_to_reservation_ticks=prepared.stretch_motion,
            start_yaw=prepared.start_yaws.get(
                request.robot_name,
                request.start_yaw,
            ),
            yaws=plan.yaws,
            actions=plan.actions,
        )
        return {
            "robot": request.robot_name,
            "startLm": request.start_lm,
            "goalLm": request.goal_lm,
            "nodes": plan.nodes,
            "times": plan.times,
            "yaws": plan.yaws,
            "actions": plan.actions,
            "timedSegments": owner._timed_segments_for_nodes(
                plan.nodes,
                plan.times,
                plan.actions,
            ),
            "trajectory": trajectory,
            "arrivalTime": trajectory[-1]["t"] if trajectory else 0.0,
        }

    @staticmethod
    def _is_deadlock_result(result: Any) -> bool:
        return any(
            marker in str(result.debug.reason or "")
            for marker in (
                "priority_cycle",
                "priority_repair_limit",
            )
        )

    def _debug_payload(
        self,
        prepared: PreparedFleetRequest,
        result: Any,
        *,
        used_blocked_edges: set[tuple[str, str]],
        fallback_reason: str,
        deadlock: bool,
        traffic_graph: Any,
    ) -> dict[str, Any]:
        """Build the stable diagnostics schema independently from plans."""
        owner = self.planner
        return {
            "reason": result.debug.reason,
            "conflictsResolved": result.debug.conflicts_resolved,
            "highLevelNodes": result.debug.high_level_nodes,
            "expandedNodes": result.debug.expanded_nodes,
            "blockedLms": sorted(prepared.blocked_lms),
            "blockedEdges": [
                f"{src}->{dst}"
                for src, dst in sorted(used_blocked_edges)
            ],
            "hardBlockedEdges": [
                f"{src}->{dst}"
                for src, dst in sorted(prepared.blocked_edges)
            ],
            "reservedDetourEdges": [
                f"{src}->{dst}"
                for src, dst in sorted(prepared.reserved_interval_edges)
            ],
            "reservedVertices": len(
                prepared.reserved_vertex_constraints
            ),
            "reservedEdges": len(prepared.reserved_edge_constraints),
            "reservedVertexIntervals": len(
                prepared.reserved_vertex_intervals
            ),
            "reservedEdgeIntervals": len(
                prepared.reserved_edge_intervals
            ),
            "reservedDetourEnabled": prepared.reserved_detour_enabled,
            "reservedFallbackReason": fallback_reason,
            "waitCost": owner.wait_cost,
            "plannerBackend": owner._planner_backend_for_payload(
                prepared.payload
            ),
            "lowLevelMaxTime": prepared.low_level_max_time,
            "cbsFallbackAllowed": prepared.allow_cbs_fallback,
            "routeSpeed": prepared.speed,
            "routeAcceleration": prepared.acceleration,
            "rotateEnabled": prepared.rotate_enabled,
            "turnSpeed": prepared.turn_speed,
            "deadlock": deadlock,
            "reservationBlockerRobots": list(
                result.debug.blocking_robots
            ),
            "reservationBlockers": [
                {
                    "robot": robot_name,
                    "resource": resource,
                }
                for robot_name, resource
                in result.debug.blocking_reservations
            ],
            "controlledCorridors": len(
                traffic_graph.controlled_region_ids()
            ),
            "controlledCorridorsMode": owner.controlled_corridors_mode,
            "controlledCorridorAutoDetect": (
                owner.controlled_corridor_auto_detect
            ),
            "deadlockReason": (
                "cyclic traffic dependencies could not be safely ordered"
                if deadlock
                else ""
            ),
        }

    def timed_segments(
        self,
        nodes: list[str],
        times: list[int],
        actions: list[str] | None = None,
    ) -> list[dict[str, Any]]:
        segments: list[dict[str, Any]] = []
        for index in range(1, min(len(nodes), len(times))):
            start_tick = int(times[index - 1])
            end_tick = int(times[index])
            start_node = nodes[index - 1]
            end_node = nodes[index]
            action = (
                str(actions[index]).strip().lower()
                if actions is not None and index < len(actions)
                else ""
            )
            base = {
                "startTick": start_tick,
                "endTick": end_tick,
                "notBeforeSec": (
                    start_tick * self.planner.time_step_sec
                ),
                "plannedArrivalSec": (
                    end_tick * self.planner.time_step_sec
                ),
            }
            if start_node == end_node:
                segments.append(
                    {
                        **base,
                        "kind": (
                            "rotate"
                            if action == "rotate"
                            else "wait"
                        ),
                        "node": start_node,
                    }
                )
                continue

            edge = self.planner.edge_by_key.get(
                (start_node, end_node)
            )
            segments.append(
                {
                    **base,
                    "kind": "move",
                    "from": start_node,
                    "to": end_node,
                    "motionDirection": (
                        edge.motion_direction_label(
                            edge.motion_direction_code()
                        )
                        if edge is not None
                        else "not_specified"
                    ),
                }
            )
        return segments
