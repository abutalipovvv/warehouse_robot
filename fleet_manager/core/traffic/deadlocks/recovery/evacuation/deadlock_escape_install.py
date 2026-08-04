"""Transactional installation of a graph escape retreat."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot


GraphEscapePlan = tuple[list[dict[str, Any]], float]


class GraphEscapeInstallMixin:
    """Install a graph-safe escape trajectory without bypassing planner rules."""

    def _install_graph_escape_retreat(
        self,
        robot: FleetRobot,
        escape_route: list[str],
        blocked_edges: list[tuple[str, str]],
        now: float,
    ) -> bool:
        """Install a short reverse-clock escape when history starts at a portal.

        Ordinary retreat reuses an already committed trajectory. A fresh
        lifelong-order chunk can start on the contested portal itself, however,
        so there is no earlier sample to return to. Plan a normal graph/motion-
        rule compliant path from the portal to an external holding pocket, then
        store its time-reversed representation. The existing retreat runtime can
        execute it by decreasing ``route_clock`` and, on arrival, transactionally
        replan the same active order to its original goal.
        """
        route_nodes = self._validated_graph_escape_route(robot, escape_route)
        if route_nodes is None:
            return False
        order = self._active_order_for_robot(robot)
        if order is None or order.status in {"COMPLETED", "CANCELED", "FAILED"}:
            return False
        request, payload = self._graph_escape_plan_request(
            robot,
            order,
            route_nodes,
            blocked_edges,
        )
        planned = self._plan_graph_escape_trajectory(robot, request, payload)
        if planned is None:
            return False
        forward, duration = planned
        retreat_trajectory = self._reverse_graph_escape_trajectory(
            forward,
            duration,
        )
        self._release_graph_escape_control_state(robot)
        self._commit_graph_escape_retreat(
            robot,
            order,
            route_nodes,
            blocked_edges,
            retreat_trajectory,
            duration,
            now,
        )
        return True

    def _validated_graph_escape_route(
        self,
        robot: FleetRobot,
        escape_route: list[str],
    ) -> list[str] | None:
        """Normalize an authored route and verify its live starting point."""
        route_nodes = [
            str(node)
            for node in escape_route
            if str(node) in self.landmarks
        ]
        safe_start_lm = self._safe_replan_start_lm(robot)
        if len(route_nodes) < 2 or route_nodes[0] != safe_start_lm:
            return None
        return route_nodes

    def _graph_escape_plan_request(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        route_nodes: list[str],
        blocked_edges: list[tuple[str, str]],
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Build the ordinary planner request used by portal recovery."""
        request: dict[str, Any] = {
            "name": robot.name,
            "startLm": route_nodes[0],
            "goalLm": route_nodes[-1],
            "routeNodes": route_nodes,
        }
        if robot.pose is not None:
            request["startPose"] = dict(robot.pose)
        payload = self._order_plan_payload(order, request) | {
            "robots": [request],
            "allowCbsFallback": False,
            "blocked_edges": [
                {"from": src, "to": dst}
                for src, dst in blocked_edges
            ],
        }
        return request, payload

    def _plan_graph_escape_trajectory(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        payload: dict[str, Any],
    ) -> GraphEscapePlan | None:
        """Plan and physically audit one outward portal trajectory."""
        # Never stall the physics thread behind an unrelated long CBS job. The
        # graph-stable cycle is retried on the next arbitration interval when
        # the shared planner is busy.
        if not self._planner_lock.acquire(blocking=False):
            return None
        try:
            result = self._plan_valid_requests_unlocked([request], payload)
            if not result.get("ok") or not result.get("plans"):
                # The escape selector already excludes controlled regions,
                # current bodies and static obstacles. Temporal reservations of
                # the cycle itself may still reject the only outward edge; for
                # this explicit right-of-way action, generate its kinematics
                # without those stale future reservations. Runtime swept-
                # footprint checks remain the final motion authority.
                result = self.planner.plan({
                    **payload,
                    "robots": [request],
                    "blocked_lms": [],
                    "reserved_vertex_constraints": [],
                    "reserved_edge_constraints": [],
                    "reserved_vertex_intervals": [],
                    "reserved_edge_intervals": [],
                })
        finally:
            self._planner_lock.release()

        plan = self._plan_for_robot(result, robot.name)
        if plan is None:
            return None
        forward = [
            dict(sample)
            for sample in plan.get("trajectory", [])
            if isinstance(sample, dict)
        ]
        if len(forward) < 2:
            return None
        duration = float(forward[-1].get("t", 0.0) or 0.0)
        if duration <= 0.000001:
            return None
        if self._trajectory_current_body_blocker(
            robot,
            forward,
            0.0,
            duration,
        ):
            # A spatial pocket is insufficient when the path to that pocket
            # crosses a robot that is currently stopped on an intermediate
            # LM.  Runtime collision checks would stop the escape safely, but
            # without this transaction-level audit it could never finish.
            return None
        return forward, duration

    @staticmethod
    def _reverse_graph_escape_trajectory(
        forward: list[dict[str, Any]],
        duration: float,
    ) -> list[dict[str, Any]]:
        """Convert forward kinematics to the reverse-clock retreat format."""
        retreat_trajectory: list[dict[str, Any]] = []
        for sample in reversed(forward):
            transformed = dict(sample)
            transformed["t"] = max(
                0.0,
                duration - float(sample.get("t", 0.0) or 0.0),
            )
            retreat_trajectory.append(transformed)
        retreat_trajectory.sort(
            key=lambda sample: float(sample.get("t", 0.0) or 0.0)
        )
        return retreat_trajectory

    def _release_graph_escape_control_state(self, robot: FleetRobot) -> None:
        """Release the stale replan and corridor ownership transaction."""
        # Supersede any failed same-goal transaction at this unchanged portal.
        self._runtime_replans.pop(robot.name, None)
        old_passage = self._controlled_corridor_passages.pop(robot.name, None)
        old_regions = {
            str(item)
            for item in (
                old_passage.get("regions", ())
                if isinstance(old_passage, dict)
                else ()
            )
            if str(item)
        }
        for region_id in old_regions:
            lease = self._controlled_corridor_leases.get(region_id)
            if lease and lease[0] == robot.name:
                self._controlled_corridor_leases.pop(region_id, None)
        self._controlled_corridor_winners.pop(robot.name, None)

    def _commit_graph_escape_retreat(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        route_nodes: list[str],
        blocked_edges: list[tuple[str, str]],
        retreat_trajectory: list[dict[str, Any]],
        duration: float,
        now: float,
    ) -> None:
        """Commit the validated reverse-clock route as one atomic transition."""
        order.status = "EXECUTING"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.traffic_detour_edges = list(dict.fromkeys(blocked_edges))
        order.route_nodes = list(route_nodes)
        robot.trajectory = retreat_trajectory
        robot.plan_nodes = list(reversed(route_nodes))
        robot.route_clock = duration
        robot.route_started_at = now
        robot.route_revision = self._next_route_revision()
        robot.pending_route = None
        robot.retreat_target_clock = 0.0
        robot.retreat_target_lm = route_nodes[-1]
        robot.retreat_blocked_edges = list(dict.fromkeys(blocked_edges))
        robot.status = "RETREATING"
        robot.last_reason = f"deadlock portal escape to {route_nodes[-1]}"
        robot.blocked_since = None
        robot.traffic_stall_since = None
        robot.last_tick_at = now
        robot.traffic_priority_until = now + self._deadlock_priority_lease()
        robot.collision_preflight_due_at = 0.0
        robot.trajectory_dirty = True
        robot.route_preview_dirty = True
        self._clear_wait_dependency(robot)
        robot.updated_at = now
