"""Publish order status, cancellation and completion transitions."""

from __future__ import annotations

from fleet_manager.manager.tasks.statuses import TERMINAL_ORDER_STATUSES
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot


class OrderLifecycleMixin:
    """Publish order status, cancellation and completion transitions."""

    def _set_order_status(
        self,
        order: FleetOrder,
        status: str,
        robot: FleetRobot | None = None,
        start_lm: str = "",
        error: str = "",
    ) -> None:
        order.status = status
        order.updated_at = self._now()
        order.error = error
        if status in {"EXECUTING", "COMPLETED"}:
            order.dispatch_failures = 0
            order.traffic_blocked_since = None
        if status not in {"QUEUED", "PLANNING"}:
            self._clear_stationary_order_retry_state(order.order_id)
        if robot is not None:
            order.assigned_robot = robot.name
            if not order.vehicle and status not in {"PLANNING", "QUEUED"}:
                order.vehicle = robot.name
        if start_lm:
            order.start_lm = start_lm

    def _set_order_error(self, order: FleetOrder, error: str) -> None:
        if order.error != error:
            self._event("warn", f"order pending: {order.order_id} {error}")
        order.status = "QUEUED"
        order.error = error
        order.dispatch_failures += 1
        error_value = str(error or "").lower()
        if "stationary_robot_blocks_route" not in error_value:
            self._clear_stationary_order_retry_state(order.order_id)
        traffic_failure = any(
            marker in error_value
            for marker in (
                "no_sipp_path",
                "reserved_",
                "traffic window",
                "deadlock",
                "continuous reservation conflict",
                "resource_conflict",
                "resource_constrained",
                "priority_cycle",
                "no_solution",
                "blocked edge",
            )
        )
        if traffic_failure:
            now = self._now()
            if order.traffic_blocked_since is None:
                order.traffic_blocked_since = now
            elif (
                order.internal_kind != "traffic_clearance"
                and now - order.traffic_blocked_since
                >= self._traffic_replan_after()
            ):
                self._queue_alternate_corridor_detour(
                    order,
                    order.start_lm,
                    self._active_order_target(order),
                )
                order.spatial_route_nodes = []
        if not order.vehicle:
            order.assigned_robot = ""
        order.updated_at = self._now()

    def _queue_alternate_corridor_detour(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        *,
        avoid_lm: str = "",
        replace_existing: bool = False,
    ) -> bool:
        """Exclude the next corridor only when the same goal stays reachable."""
        if order.internal_kind == "traffic_clearance":
            # The route is an authoritative outward evacuation selected by
            # _stationary_clearance_route().  A generic detour can cross the
            # corridor lease held by the robot this order is meant to free.
            return False
        if start_lm not in self.landmarks or final_goal_lm not in self.landmarks:
            return False
        # A detour is a one-chunk traffic decision, not permanent map editing.
        # Retry an existing exclusion after backoff instead of accumulating
        # enough exclusions to erode the graph of a long-running order.
        if order.traffic_detour_edges and not replace_existing:
            return False
        owner = str(order.vehicle or order.assigned_robot or "").strip()
        stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names={owner} if owner else set(),
        )
        stationary_edges = self._blocked_edges_for_lms(stationary_lms)
        if avoid_lm and (
            avoid_lm not in self.landmarks
            or avoid_lm in {start_lm, final_goal_lm}
        ):
            # A robot cannot route around its own source, and an occupied goal
            # needs ordinary wait/clearance rather than a misleading detour.
            return False
        avoid_edges = (
            self._blocked_edges_for_lms({avoid_lm})
            if avoid_lm
            else set()
        )
        route = None
        existing_nodes = (
            []
            if replace_existing
            else [
                str(node)
                for node in order.spatial_route_nodes
                if str(node) in self.landmarks
            ]
        )
        if start_lm in existing_nodes and existing_nodes[-1:] == [final_goal_lm]:
            suffix = existing_nodes[existing_nodes.index(start_lm):]
            if all(
                self.planner.route_planner.get_edge(src, dst) is not None
                for src, dst in zip(suffix, suffix[1:])
            ):
                route = self._planned_route_from_nodes(suffix)
        edge_penalties = (
            self._traffic_route_edge_penalties(order, start_lm, final_goal_lm)
            if self._congestion_routing_enabled()
            else None
        )
        if route is None:
            try:
                route = self.planner.route_planner.find_route(
                    start_lm,
                    final_goal_lm,
                    blocked_edges=stationary_edges,
                    edge_penalties=edge_penalties,
                )
            except ValueError:
                return False
        if len(route.nodes) < 2:
            return False
        if avoid_lm and avoid_lm not in route.nodes:
            # The whole-route selector has already avoided this stationary
            # body. During an explicit replacement, retiring the stale
            # one-chunk exclusion and cache is itself the required route
            # change; otherwise do not ban an unrelated first edge.
            if not replace_existing:
                return False
            order.traffic_detour_edges = []
            order.traffic_detour_attempts += 1
            order.spatial_route_nodes = []
            self._event(
                "warn",
                f"{order.vehicle or order.assigned_robot} stale traffic detour "
                f"retired; congestion route avoids {avoid_lm}",
            )
            return True

        src, dst = str(route.nodes[0]), str(route.nodes[1])
        candidate = (
            set(avoid_edges)
            if avoid_lm
            else {(src, dst), (dst, src)}
        )
        try:
            alternate = self.planner.route_planner.find_route(
                start_lm,
                final_goal_lm,
                blocked_edges=candidate | stationary_edges,
                edge_penalties=edge_penalties,
            )
        except ValueError:
            # A single-exit landmark must wait; banning its only corridor
            # would turn temporary congestion into a permanent no-path error.
            return False
        if alternate.nodes == route.nodes:
            return False

        order.traffic_detour_edges = sorted(candidate)
        order.traffic_detour_attempts += 1
        order.spatial_route_nodes = []
        self._event(
            "warn",
            f"{order.vehicle or order.assigned_robot} alternate corridor queued: "
            + (
                f"avoid occupied LM {avoid_lm}, keep goal {final_goal_lm}"
                if avoid_lm
                else f"avoid {src}<->{dst}, keep goal {final_goal_lm}"
            ),
        )
        return True

    def _order_stall_allows_detour(self, order: FleetOrder) -> bool:
        now = self._now()
        if order.traffic_blocked_since is None:
            order.traffic_blocked_since = now
            return False
        return now - order.traffic_blocked_since >= self._traffic_replan_after()

    def _cancel_order(self, order: FleetOrder, reason: str) -> None:
        for robot in self._runtime_robots():
            if robot.active_order_id == order.order_id:
                self._stop_robot(robot, cancel_active_order=False)
        self._set_order_status(order, "CANCELED", error=reason)
        self._event("warn", f"order canceled: {order.order_id}")

    def _pause_order(self, order: FleetOrder, reason: str) -> None:
        paused_robot: FleetRobot | None = None
        for robot in self._runtime_robots():
            if robot.active_order_id == order.order_id:
                paused_robot = robot
                break
        if paused_robot is None and order.assigned_robot:
            paused_robot = self.robots.get(order.assigned_robot)
        if paused_robot is None and order.vehicle:
            paused_robot = self.robots.get(order.vehicle)

        if paused_robot is not None:
            self._cancel_remote_route(paused_robot, reason)
            nearest_lm = self._nearest_lm_for_robot(paused_robot)
            if nearest_lm in self.landmarks:
                paused_robot.current_lm = nearest_lm
            paused_robot.target_lm = ""
            paused_robot.status = "PAUSED"
            paused_robot.trajectory = []
            paused_robot.plan_nodes = []
            paused_robot.trajectory_dirty = True
            paused_robot.route_started_at = None
            paused_robot.route_clock = 0.0
            paused_robot.last_tick_at = None
            paused_robot.blocked_since = None
            paused_robot.last_replan_at = None
            paused_robot.last_reason = reason
            paused_robot.route_note = ""
            paused_robot.active_order_id = ""
            self._clear_remote_route_metadata(paused_robot)
            paused_robot.updated_at = self._now()
            order.assigned_robot = paused_robot.name
            if not order.vehicle:
                order.vehicle = paused_robot.name
            order.start_lm = paused_robot.current_lm

        order.status = "PAUSED"
        order.error = reason
        order.updated_at = self._now()
        self._event("warn", f"order paused: {order.order_id}")

    def _cancel_active_order_for_robot(self, robot: FleetRobot, reason: str) -> None:
        if not robot.active_order_id:
            return
        self._cancel_remote_route(robot, reason)
        order = self.orders.get(robot.active_order_id)
        if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
            self._set_order_status(order, "CANCELED", error=reason)
            self._event("warn", f"order canceled: {order.order_id} {reason}")
        robot.active_order_id = ""

    def _cancel_orders_for_robot(self, robot_name: str, reason: str) -> None:
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.vehicle == robot_name or order.assigned_robot == robot_name:
                self._set_order_status(order, "CANCELED", error=reason)
                self._event("warn", f"order canceled: {order.order_id} {reason}")

    def _replace_orders_for_robot(self, robot_name: str, reason: str) -> None:
        robot = self.robots.get(robot_name)
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.vehicle == robot_name or order.assigned_robot == robot_name:
                self._set_order_status(order, "CANCELED", error=reason)
                self._event("warn", f"order canceled: {order.order_id} {reason}")

        if robot is None:
            return
        self._cancel_remote_route(robot, reason)
        nearest_lm = self._nearest_lm_for_robot(robot)
        if nearest_lm in self.landmarks:
            robot.current_lm = nearest_lm
        robot.target_lm = ""
        robot.status = "IDLE"
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.last_tick_at = None
        robot.blocked_since = None
        robot.last_replan_at = None
        robot.last_reason = reason
        robot.route_note = ""
        robot.active_order_id = ""
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()

    def _cancel_all_orders(self, reason: str) -> None:
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            self._set_order_status(order, "CANCELED", error=reason)
            self._event("warn", f"order canceled: {order.order_id} {reason}")

    def _active_order_target(self, order: FleetOrder) -> str:
        targets = order.targets or ([order.target_lm] if order.target_lm else [])
        if not targets:
            return order.target_lm
        order.step_index = max(0, min(order.step_index, len(targets) - 1))
        order.target_lm = targets[order.step_index]
        return order.target_lm

    def _advance_or_complete_order(self, order: FleetOrder, robot: FleetRobot, now: float) -> bool:
        # A reached step invalidates any parked-blocker signature collected
        # while trying to depart toward the previous target.
        self._clear_stationary_order_retry_state(order.order_id)
        targets = order.targets or ([order.target_lm] if order.target_lm else [])
        if order.step_index + 1 >= len(targets):
            order.status = "COMPLETED"
            order.error = ""
            order.updated_at = now
            order.assigned_robot = robot.name
            order.route_nodes = list(robot.plan_nodes)
            robot.active_order_id = ""
            return True

        previous_target = targets[order.step_index]
        order.step_index += 1
        order.target_lm = targets[order.step_index]
        order.status = "QUEUED"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = robot.current_lm
        order.route_nodes = []
        order.spatial_route_nodes = []
        order.spatial_route_revision = 0
        order.traffic_blocked_since = None
        robot.active_order_id = ""
        self._event(
            "info",
            f"order step completed: {order.order_id} {robot.name}@{previous_target}; next {order.target_lm}",
        )
        return False

    def _complete_active_order(self, robot: FleetRobot, now: float) -> None:
        if not robot.active_order_id:
            return
        order = self.orders.get(robot.active_order_id)
        if order is None:
            robot.active_order_id = ""
            return
        order.route_nodes = list(robot.plan_nodes)
        self._advance_or_complete_order(order, robot, now)
