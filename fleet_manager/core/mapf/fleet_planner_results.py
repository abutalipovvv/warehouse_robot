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
        plans: list[dict[str, Any]] = []
        if result.plans:
            for request in prepared.requests:
                plan = result.plans.get(request.robot_name)
                if plan is None:
                    continue
                trajectory = owner._trajectory_for_nodes(
                    plan.nodes,
                    prepared.speed,
                    plan.times,
                    acceleration=prepared.acceleration,
                    rotate_enabled=prepared.rotate_enabled,
                    turn_speed=prepared.turn_speed,
                    stretch_motion_to_reservation_ticks=(
                        prepared.stretch_motion
                    ),
                    start_yaw=prepared.start_yaws.get(
                        request.robot_name,
                        request.start_yaw,
                    ),
                    yaws=plan.yaws,
                    actions=plan.actions,
                )
                plans.append(
                    {
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
                        "arrivalTime": (
                            trajectory[-1]["t"]
                            if trajectory
                            else 0.0
                        ),
                    }
                )

        deadlock = any(
            marker in str(result.debug.reason or "")
            for marker in (
                "priority_cycle",
                "priority_repair_limit",
            )
        )
        traffic_graph = owner._traffic_graph(prepared.speed)
        return {
            "ok": bool(result.plans) or not prepared.requests,
            "debug": {
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
                    for src, dst in sorted(
                        prepared.reserved_interval_edges
                    )
                ],
                "reservedVertices": len(
                    prepared.reserved_vertex_constraints
                ),
                "reservedEdges": len(
                    prepared.reserved_edge_constraints
                ),
                "reservedVertexIntervals": len(
                    prepared.reserved_vertex_intervals
                ),
                "reservedEdgeIntervals": len(
                    prepared.reserved_edge_intervals
                ),
                "reservedDetourEnabled": (
                    prepared.reserved_detour_enabled
                ),
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
                "controlledCorridorsMode": (
                    owner.controlled_corridors_mode
                ),
                "controlledCorridorAutoDetect": (
                    owner.controlled_corridor_auto_detect
                ),
                "deadlockReason": (
                    "cyclic traffic dependencies could not be safely ordered"
                    if deadlock
                    else ""
                ),
            },
            "timeStepSec": owner.time_step_sec,
            "plans": plans,
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

