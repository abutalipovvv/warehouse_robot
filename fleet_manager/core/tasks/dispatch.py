"""Order lifecycle, dispatch and rolling task execution."""

from __future__ import annotations

from heapq import heappop, heappush
import math
from threading import Thread
from time import time
from typing import Any

from fleet_manager.core.constants import (
    FLEET_CONTROL_OWNER_ID,
    ORDER_SEQUENCE_KEYS,
    ORDER_TARGET_KEYS,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.models import FleetOrder, FleetRobot


class FleetTaskDispatchMixin:
    """Shared task policy for simulation and gRPC fleet runtimes."""

    def _orders_list(self) -> list[dict[str, Any]]:
        return self.task_manager.ordered_payloads(
            enabled=self._order_enabled,
            limit=120,
        )

    def _build_orders(self, payload: dict[str, Any]) -> list[FleetOrder]:
        return [self._build_order(payload)]

    def _build_order(self, payload: dict[str, Any]) -> FleetOrder:
        order_id = str(
            payload.get("id")
            or payload.get("orderId")
            or payload.get("taskId")
            or payload.get("externalId")
            or ""
        ).strip()
        if not order_id:
            order_id = f"order-{int(self._now() * 1000)}"

        targets = self._target_lms_from_payload(payload)
        if not targets:
            raise ValueError("targetLm/goalLm/location is required")
        for target_lm in targets:
            if target_lm not in self.landmarks:
                raise ValueError(f"unknown target LM: {target_lm}")

        vehicle = str(
            payload.get("vehicle")
            or payload.get("robot")
            or payload.get("robotName")
            or payload.get("name")
            or ""
        ).strip()
        if vehicle and vehicle not in self.robots:
            raise ValueError(f"unknown robot: {vehicle}")

        try:
            priority = int(payload.get("priority", 0) or 0)
        except (TypeError, ValueError):
            priority = 0
        external_id = str(payload.get("externalId") or payload.get("taskId") or "").strip()
        speed = self._float_payload(payload, ("speed", "routeSpeed"), 0.0)
        acceleration = self._float_payload(payload, ("acceleration", "routeAcceleration", "route_acceleration"), 0.0)
        rotate = self._bool_payload(payload, ("rotate", "simulateRotation", "simulate_rotation"), False)
        turn_speed = self._float_payload(payload, ("turnSpeed", "turn_speed", "rotationSpeed", "rotation_speed"), 0.0)
        stretch_motion = self._bool_payload(
            payload,
            ("stretchMotionToReservationTicks", "stretch_motion_to_reservation_ticks"),
            True,
        )
        now = self._now()
        return FleetOrder(
            order_id=order_id,
            target_lm=targets[0],
            vehicle=vehicle,
            priority=priority,
            external_id=external_id,
            targets=targets,
            speed=speed,
            acceleration=acceleration,
            rotate=rotate,
            turn_speed=turn_speed,
            stretch_motion_to_reservation_ticks=stretch_motion,
            created_at=now,
            updated_at=now,
        )

    def _float_payload(self, payload: dict[str, Any], keys: tuple[str, ...], default: float = 0.0) -> float:
        for key in keys:
            if key not in payload:
                continue
            try:
                return float(payload.get(key) or default)
            except (TypeError, ValueError):
                return default
        return default

    def _bool_payload(self, payload: dict[str, Any], keys: tuple[str, ...], default: bool = False) -> bool:
        for key in keys:
            if key not in payload:
                continue
            value = payload.get(key)
            if isinstance(value, str):
                return value.strip().lower() in {"1", "true", "yes", "on"}
            return bool(value)
        return default

    def _target_lms_from_payload(self, payload: dict[str, Any]) -> list[str]:
        targets: list[str] = []
        for key in ORDER_SEQUENCE_KEYS:
            raw_sequence = payload.get(key)
            if not isinstance(raw_sequence, list):
                continue
            for item in raw_sequence:
                target_lm = self._target_lm_from_payload_item(item)
                if target_lm:
                    targets.append(target_lm)
            if targets:
                return targets

        target_lm = self._target_lm_from_payload_item(payload)
        return [target_lm] if target_lm else []

    def _target_lm_from_payload_item(self, item: Any) -> str:
        if isinstance(item, dict):
            for key in ORDER_TARGET_KEYS:
                target_lm = str(item.get(key) or "").strip()
                if target_lm:
                    return target_lm
            return ""
        return str(item or "").strip()

    def _dispatch_orders(
        self,
        force: bool = False,
        *,
        async_simulated: bool = False,
    ) -> int:
        dispatched = self._finish_async_simulated_dispatch() if async_simulated else 0
        now = self._now()
        queued_orders = [
            order for order in self.orders.values()
            if order.status == "QUEUED"
            and (
                force
                or not order.error
                or now - order.updated_at >= self._order_dispatch_retry_interval(order)
            )
        ]
        queued_orders.sort(
            key=lambda item: (
                bool(item.error),
                -int(item.priority or 0),
                item.updated_at if item.error else item.created_at,
                item.order_id,
            )
        )

        # Dynamic orders used to call MAPF independently for every idle robot.
        # Apart from being expensive, that made all other robots look parked
        # and produced a fleet full of wait-only plans.  Plan a small coupled
        # component at a time and cap synchronous work per runtime tick.
        handled: set[str] = set()
        ready = self._ready_simulated_order_entries(queued_orders)
        handled.update(
            order.order_id
            for order in queued_orders
            if order.status != "QUEUED"
            or str(order.error or "").startswith("manual graph reconnect blocked:")
        )
        stationary_release_names = self._stationary_release_robot_names()
        if stationary_release_names:
            # Release a parked terminal dependency before unrelated recovery
            # batches. The existing priority/FIFO order stays stable within
            # the release and ordinary groups.
            ready.sort(
                key=lambda entry: entry[1].name not in stationary_release_names
            )
        if async_simulated and not self._async_simulated_dispatch_active():
            prefetch_entries = self._ready_rolling_prefetch_entries()
            prefetch = prefetch_entries[0] if prefetch_entries else None
            prefetch_is_urgent = bool(
                prefetch is not None
                and float(prefetch[-1]) <= self._rolling_prefetch_urgent_lead()
            )
            prefetch_repeatedly_blocked = bool(
                prefetch_entries
                and any(
                    self._rolling_prefetch_failures.get(entry[1].name, 0) > 0
                    for entry in prefetch_entries
                )
            )
            recovery_turn_after_dispatch = bool(
                prefetch_repeatedly_blocked
                and ready
                and self._last_async_job_kind in {"dispatch", "coupled_replan"}
            )
            # Fill idle robots before spending the only planner slot on a
            # healthy, non-urgent continuation. A route inside its final
            # critical seconds still wins, unless its previous recovery
            # attempt failed; in that case the stationary departure must run
            # first because it may be the physical blocker.
            if prefetch is not None and (
                not ready
                or (prefetch_is_urgent and not prefetch_repeatedly_blocked)
                or recovery_turn_after_dispatch
            ):
                self._start_async_rolling_prefetch(prefetch_entries)
                return dispatched
        planning_budget = self._dispatch_plan_budget()
        batch_size = self._dispatch_batch_size()
        planning_calls = 0
        while ready and planning_calls < planning_budget:
            first = ready.pop(0)
            motion_key = self._order_motion_key(first[0])
            group = [first]
            group_limit = self._dispatch_recovery_group_limit(
                first[0],
                first[1],
                batch_size,
            )
            if first[1].name in stationary_release_names:
                # Several queued departures can physically box one another in
                # a dense Kiva junction. Planning the first blocker alone
                # cannot release that component; coordinate all currently
                # identified departures up to the bounded local MAPF cap.
                group_limit = min(
                    self.planner.local_cbs_max_robots,
                    max(batch_size, len(stationary_release_names)),
                )
            remaining: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]] = []
            for entry in ready:
                if (
                    len(group) < group_limit
                    and self._order_motion_key(entry[0]) == motion_key
                    and (
                        first[1].name not in stationary_release_names
                        or entry[1].name in stationary_release_names
                    )
                ):
                    group.append(entry)
                else:
                    remaining.append(entry)
            ready = remaining
            if first[1].name in stationary_release_names:
                for release_order, _, _, _ in group:
                    # Rebuild the cached suffix against the current joint
                    # starts, but keep an explicit recovery detour.  Clearing
                    # that detour here sent a corridor entrant straight back
                    # into the owner it was supposed to release.
                    release_order.spatial_route_nodes = []
            if async_simulated:
                if self._async_simulated_dispatch_active():
                    break
                self._start_async_simulated_dispatch(group)
                group_dispatched = 0
            else:
                group_dispatched, _ = self._dispatch_simulated_order_batch(group)
                dispatched += group_dispatched
            group_handled = {order.order_id for order, _, _, _ in group}
            handled.update(group_handled)
            planning_calls += 1
            if async_simulated:
                break

        # Keep support for remote, automatic and already-at-goal orders.  A
        # failed coupled group is deliberately not retried individually in the
        # same tick: doing so recreated both CPU starvation and wait-only plans.
        remaining_budget = max(0, planning_budget - planning_calls)
        for order in queued_orders:
            if order.order_id in handled:
                continue
            robot = self.robots.get(order.vehicle) if order.vehicle else None
            if async_simulated and robot is not None and not robot.is_remote():
                # Simulated MAPF is deliberately background-only on runtime
                # ticks.  Otherwise one leftover order here would put the
                # expensive planner back on the websocket/status path.
                continue
            if (
                robot is not None
                and not robot.is_remote()
                and self._robot_can_accept_order(robot, explicit=True)
                and self._safe_replan_start_lm(robot) != order.target_lm
            ):
                if remaining_budget <= 0:
                    continue
                remaining_budget -= 1
            if self._dispatch_order(order, force=force):
                dispatched += 1
        return dispatched

    def _stationary_release_robot_names(self) -> set[str]:
        """Find queued stationary robots that currently block active traffic."""
        release: set[str] = set()
        for waiter in self._runtime_robots():
            if waiter.status != "WAITING" or not waiter.trajectory:
                continue
            blocker_name = (
                waiter.wait_for_robot
                or self._robot_name_from_conflict_reason(waiter.last_reason)
            )
            blocker = self.robots.get(blocker_name)
            if (
                blocker is None
                or blocker.trajectory
                or blocker.status not in {"IDLE", "ARRIVED", "BLOCKED"}
            ):
                continue
            pending = self._active_order_for_robot(blocker)
            if pending is not None and pending.status in {"QUEUED", "PLANNING"}:
                release.add(blocker.name)
        # A coupled planner failure identifies the member whose route failed
        # validation. Do not let that one member keep poisoning every later
        # batch: release it first, then allow the unaffected queue heads to be
        # planned without it.
        for order in self.orders.values():
            if order.status != "QUEUED" or not order.error:
                continue
            blocker_name = self._planner_conflict_robot_name(order.error)
            blocker = self.robots.get(blocker_name)
            if (
                blocker is None
                or blocker.trajectory
                or blocker.status not in {"IDLE", "ARRIVED", "BLOCKED"}
            ):
                continue
            pending = self._active_order_for_robot(blocker)
            if pending is not None and pending.status in {"QUEUED", "PLANNING"}:
                release.add(blocker.name)
        return release

    def _planner_conflict_robot_name(self, reason: str) -> str:
        """Extract the member named by MAPF plan validation failures."""
        text = str(reason or "")
        markers = (
            "cbs_resource_conflict:",
            "resource_conflict:",
            "cbs_missing_plan:",
            "missing_plan:",
            "no_low_level_path:",
        )
        for marker in markers:
            marker_index = text.find(marker)
            if marker_index < 0:
                continue
            tail = text[marker_index + len(marker):]
            for robot_name in sorted(self.robots, key=len, reverse=True):
                if tail == robot_name or (
                    tail.startswith(robot_name)
                    and tail[len(robot_name):len(robot_name) + 1]
                    in {":", ";", ",", " ", ")"}
                ):
                    return robot_name
        return ""

    def _ready_simulated_order_entries(
        self,
        orders: list[FleetOrder],
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]]:
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]] = []
        used_robots: set[str] = set()
        for order in orders:
            if not order.vehicle or order.vehicle in used_robots:
                continue
            if not self._order_is_robot_queue_head(order):
                continue
            robot = self.robots.get(order.vehicle)
            if robot is None or robot.is_remote():
                continue
            if not self._robot_can_accept_order(robot, explicit=True):
                continue
            final_goal = self._active_order_target(order)
            start_lm = self._safe_replan_start_lm(robot)
            if not start_lm or start_lm not in self.landmarks:
                self._dispatch_manual_graph_reconnect(order, robot, final_goal)
                used_robots.add(robot.name)
                continue
            if start_lm == final_goal:
                now = self._now()
                robot.current_lm = final_goal
                robot.target_lm = ""
                robot.status = "ARRIVED"
                robot.active_order_id = ""
                robot.last_reason = "order already at target"
                robot.updated_at = now
                completed = self._advance_or_complete_order(order, robot, now)
                self._event(
                    "info",
                    (
                        f"order completed: {order.order_id} {robot.name}@{final_goal}"
                        if completed
                        else f"order step reached: {order.order_id} {robot.name}@{final_goal}"
                    ),
                )
                used_robots.add(robot.name)
                continue
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                # The rolling waypoint and stable spatial suffix are selected
                # after the dispatch batch is formed. Only then do we know
                # which stationary owners will receive an atomic joint plan
                # and may safely be released from persistent occupancy.
                "goalLm": final_goal,
            }
            if robot.pose is not None:
                request["startPose"] = dict(robot.pose)
            entries.append((order, robot, request, final_goal))
            used_robots.add(robot.name)
        return entries

    def _dispatch_manual_graph_reconnect(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        final_goal: str,
    ) -> bool:
        """Return a manually moved simulated robot to the traffic graph.

        MAPF starts at graph landmarks by design. A free-drive pose therefore
        needs a short collision-checked approach chunk before the queued graph
        route can be planned. Keeping it as part of the same order avoids both
        teleporting the robot and silently dropping the operator's goal.
        """
        if robot.is_remote() or robot.pose is None:
            self._set_order_error(order, "robot has no graph-safe start pose")
            return False
        reconnect_lm = self._nearest_lm_for_robot(robot)
        landmark = self.landmarks.get(reconnect_lm)
        if landmark is None:
            self._set_order_error(order, "no graph landmark near manual pose")
            return False

        start_pose = {
            "x": float(robot.pose.get("x", 0.0) or 0.0),
            "y": float(robot.pose.get("y", 0.0) or 0.0),
            "yaw": float(robot.pose.get("yaw", 0.0) or 0.0),
        }
        dx = float(landmark.x) - start_pose["x"]
        dy = float(landmark.y) - start_pose["y"]
        distance = math.hypot(dx, dy)
        if distance <= 0.000001:
            robot.current_lm = reconnect_lm
            return False

        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        speed = max(
            0.05,
            float(order.speed or navigation.get("route_speed", 0.35) or 0.35),
        )
        turn_speed = max(
            0.05,
            float(order.turn_speed or navigation.get("max_angular_speed", 0.9) or 0.9),
        )
        heading = math.atan2(dy, dx)
        yaw_delta = math.atan2(
            math.sin(heading - start_pose["yaw"]),
            math.cos(heading - start_pose["yaw"]),
        )
        sample_dt = max(0.03, min(0.08, self.collision.sample_time_step() / 2.0))
        rotate_duration = abs(yaw_delta) / turn_speed
        rotate_steps = max(1, int(math.ceil(rotate_duration / sample_dt))) if rotate_duration > 0.01 else 0
        move_duration = distance / speed
        move_steps = max(
            1,
            int(math.ceil(max(move_duration / sample_dt, distance / 0.04))),
        )
        trajectory: list[dict[str, Any]] = [
            {
                "t": 0.0,
                **start_pose,
                "edgeId": f"MANUAL->{reconnect_lm}",
                "motionDirection": "not_specified",
            }
        ]
        for index in range(1, rotate_steps + 1):
            ratio = index / rotate_steps
            trajectory.append(
                {
                    "t": rotate_duration * ratio,
                    "x": start_pose["x"],
                    "y": start_pose["y"],
                    "yaw": start_pose["yaw"] + yaw_delta * ratio,
                    "edgeId": f"MANUAL->ROTATE@{reconnect_lm}",
                    "motionDirection": "rotate",
                }
            )
        for index in range(1, move_steps + 1):
            ratio = index / move_steps
            sample: dict[str, Any] = {
                "t": rotate_duration + move_duration * ratio,
                "x": start_pose["x"] + dx * ratio,
                "y": start_pose["y"] + dy * ratio,
                "yaw": heading,
                "edgeId": f"MANUAL->{reconnect_lm}",
                "motionDirection": "forward",
            }
            if index == move_steps:
                sample["lm"] = reconnect_lm
            trajectory.append(sample)

        for sample in trajectory:
            reason = self.collision.blocked_reason(
                pose=sample,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
            if not reason:
                for other in self._runtime_robots():
                    if other.name == robot.name or other.pose is None:
                        continue
                    if self.collision.robot_footprints_conflict(sample, other.pose):
                        reason = f"robot footprint conflict with {other.name}"
                        break
            if reason:
                self._set_order_error(order, f"manual graph reconnect blocked: {reason}")
                return False

        now = self._now()
        robot.current_lm = reconnect_lm
        robot.target_lm = reconnect_lm
        robot.status = "MOVING"
        robot.trajectory = trajectory
        robot.trajectory_dirty = True
        robot.plan_nodes = [reconnect_lm]
        robot.route_started_at = now
        robot.route_clock = 0.0
        robot.last_tick_at = now
        robot.blocked_since = None
        robot.last_replan_at = None
        robot.last_reason = "returning manual pose to traffic graph"
        robot.route_note = "manual graph reconnect"
        robot.active_order_id = order.order_id
        robot.route_revision = self._next_route_revision()
        robot.route_chunk_index = 0
        robot.route_chunk_goal_lm = reconnect_lm
        robot.route_final_lm = final_goal
        robot.route_preview = [dict(sample) for sample in trajectory]
        robot.route_preview_dirty = True
        robot.pending_route = None
        robot.has_executed_route = True
        robot.updated_at = now

        order.status = "EXECUTING"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = reconnect_lm
        order.route_nodes = [reconnect_lm]
        self._event(
            "info",
            f"manual graph reconnect: {robot.name}->{reconnect_lm}; then {final_goal}",
        )
        return True

    def _order_is_robot_queue_head(self, order: FleetOrder) -> bool:
        """Keep generated lifelong orders FIFO for each robot.

        A later benchmark order is generated from the previous order's final
        LM. Dispatching it first (for example because it has higher priority)
        can make that otherwise distant goal only one edge from the robot's
        current pose. Priority still orders traffic between different robots.
        """
        if not order.order_id.startswith("dynamic-") or not order.vehicle:
            return True
        pending = [
            candidate
            for candidate in self.orders.values()
            if candidate.order_id.startswith("dynamic-")
            and candidate.vehicle == order.vehicle
            and candidate.status not in TERMINAL_ORDER_STATUSES
        ]
        if not pending:
            return True
        pending.sort(key=lambda candidate: (candidate.created_at, candidate.order_id))
        return pending[0] is order

    def _dispatch_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[int, set[str]]:
        if not entries:
            return 0, set()
        requests, payload = self._prepare_simulated_order_batch(entries)
        result = self._plan_valid_requests(requests, payload)
        return self._finish_simulated_order_batch(entries, result)

    def _prepare_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        released_owners = {robot.name for _, robot, _, _ in entries}
        protected_starts = {
            str(request.get("startLm") or "")
            for _, _, request, _ in entries
        }
        reserved_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        for order, robot, raw_request, final_goal in entries:
            request = dict(raw_request)
            start_lm = str(request["startLm"])
            planning_goal = self._rolling_planning_goal(
                start_lm,
                final_goal,
                order,
                release_robot_names=released_owners,
            )
            planning_goal = self._distinct_rolling_batch_goal(
                order,
                start_lm,
                final_goal,
                planning_goal,
                reserved_goals=reserved_goals,
                protected_starts=protected_starts,
                release_robot_names=released_owners,
            )
            reserved_goals.add(planning_goal)
            request["goalLm"] = planning_goal
            request.pop("routeNodes", None)
            self._attach_spatial_route_to_request(
                request,
                order,
                start_lm,
                planning_goal,
                final_goal,
                release_robot_names=released_owners,
            )
            raw_request.clear()
            raw_request.update(request)
            requests.append(request)
            self._set_order_status(
                order,
                "PLANNING",
                robot=robot,
                start_lm=start_lm,
            )
        first_order = entries[0][0]
        payload = self._order_plan_payload(first_order, requests[0]) | {"robots": requests}
        return requests, payload

    def _distinct_rolling_batch_goal(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        planning_goal_lm: str,
        *,
        reserved_goals: set[str],
        protected_starts: set[str],
        release_robot_names: set[str],
    ) -> str:
        """Keep converging rolling requests from sharing a terminal vertex."""
        goal_uses_other_start = (
            planning_goal_lm in protected_starts
            and planning_goal_lm != start_lm
        )
        if planning_goal_lm not in reserved_goals and not goal_uses_other_start:
            return planning_goal_lm
        try:
            route_nodes = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
                release_robot_names=release_robot_names,
            )
        except ValueError:
            return planning_goal_lm
        if planning_goal_lm not in route_nodes:
            return planning_goal_lm
        target_index = route_nodes.index(planning_goal_lm)
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        graph = self.planner._traffic_graph(
            self.planner._route_speed(route_payload),
        )
        minimum_distance = max(
            0.0,
            float(self.planner.min_robot_center_distance_m),
        )

        def usable(candidate: str) -> bool:
            if (
                candidate == start_lm
                or candidate in reserved_goals
                or candidate in protected_starts
            ):
                return False
            vertex = graph.vertices.get(candidate)
            if vertex is not None and not vertex.can_wait:
                return False
            landmark = self.landmarks.get(candidate)
            if landmark is None:
                return False
            return all(
                other not in self.landmarks
                or math.hypot(
                    landmark.x - self.landmarks[other].x,
                    landmark.y - self.landmarks[other].y,
                ) + 0.000001 >= minimum_distance
                for other in reserved_goals
            )

        # Prefer a slightly earlier holding vertex so the bounded request does
        # not grow. If that branch has no safe wait point, a later unique
        # vertex is still preferable to rejecting the entire coupled batch.
        for index in range(target_index - 1, 0, -1):
            candidate = str(route_nodes[index])
            if usable(candidate):
                return candidate
        for index in range(target_index + 1, len(route_nodes)):
            candidate = str(route_nodes[index])
            if usable(candidate):
                return candidate
        return planning_goal_lm

    def _finish_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
        result: dict[str, Any],
    ) -> tuple[int, set[str]]:
        handled = {order.order_id for order, _, _, _ in entries}
        final_goals = {
            robot.name: final_goal
            for _, robot, _, final_goal in entries
        }
        if not result.get("ok") or not result.get("plans"):
            reason = self._planner_failure_reason(result)
            conflict_robot = self._planner_conflict_robot_name(reason)
            isolated = conflict_robot and any(
                robot.name == conflict_robot
                for _, robot, _, _ in entries
            )
            for order, robot, _, _ in entries:
                if isolated and robot.name != conflict_robot:
                    # The shared request failed validation for one named
                    # member. Keep the other queue heads immediately eligible
                    # instead of copying the same error/failure backoff to the
                    # whole group.
                    order.status = "QUEUED"
                    order.error = ""
                    order.updated_at = self._now()
                    continue
                self._set_order_error(order, reason)
            return 0, handled

        result = self._rolling_result(result, final_goals)
        plans_by_robot = {
            str(plan.get("robot")): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        accepted: list[dict[str, Any]] = []
        accepted_entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, dict[str, Any]]] = []
        for order, robot, request, final_goal in entries:
            plan = plans_by_robot.get(robot.name)
            if plan is None:
                self._set_order_error(order, "planner did not return robot plan")
                continue
            if self._wait_only_rolling_plan(plan, final_goal):
                detour_queued = (
                    self._order_stall_allows_detour(order)
                    and self._queue_alternate_corridor_detour(
                        order,
                        str(request.get("startLm") or ""),
                        final_goal,
                    )
                )
                self._set_order_error(
                    order,
                    "traffic window has no progress; alternate corridor queued"
                    if detour_queued
                    else "traffic window has no progress; joint retry pending",
                )
                self._event(
                    "warn",
                    f"{robot.name} wait-only route rejected; order kept queued",
                )
                continue
            accepted.append(plan)
            accepted_entries.append((order, robot, request, final_goal, plan))

        if not accepted:
            return 0, handled
        accepted_result = {**result, "plans": accepted}
        now = self._now()
        self._apply_planner_result(accepted_result, now)
        for order, robot, request, _, plan in accepted_entries:
            order.route_nodes = [str(item) for item in plan.get("nodes", [])]
            robot.active_order_id = order.order_id
            self._apply_simulated_route_metadata(robot, order, plan, now)
            self._set_order_status(
                order,
                "EXECUTING",
                robot=robot,
                start_lm=str(request["startLm"]),
            )
            order.traffic_detour_edges = []
            self._event(
                "info",
                f"order dispatched: {order.order_id} {robot.name} "
                f"{request['startLm']}->{order.target_lm}",
            )
        return len(accepted_entries), handled

    def _async_simulated_dispatch_active(self) -> bool:
        with self._dispatch_job_lock:
            return self._dispatch_job is not None

    def _start_async_simulated_dispatch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> None:
        if not entries:
            return
        requests, payload = self._prepare_simulated_order_batch(entries)
        job: dict[str, Any] = {
            "kind": "dispatch",
            "entries": list(entries),
            "requests": requests,
            "payload": payload,
            "done": False,
            "result": None,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                for order, _, _, _ in entries:
                    if order.status == "PLANNING":
                        order.status = "QUEUED"
                return
            self._dispatch_job = job

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background planner failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-dispatch",
            daemon=True,
        ).start()

    def _rolling_prefetch_candidates(
        self,
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]:
        lead = self._rolling_prefetch_lead()
        now = self._now()
        candidates: list[tuple[float, str, FleetOrder, FleetRobot, dict[str, Any], str]] = []
        for robot in self._runtime_robots():
            if (
                robot.is_remote()
                or robot.pending_route is not None
                or robot.status not in {"MOVING", "WAITING"}
                or not robot.active_order_id
                or not robot.route_chunk_goal_lm
                or not robot.trajectory
            ):
                continue
            if now + 0.000001 < self._rolling_prefetch_retry_at.get(robot.name, 0.0):
                continue
            order = self.orders.get(robot.active_order_id)
            if order is None or order.status in TERMINAL_ORDER_STATUSES:
                continue
            final_goal = self._active_order_target(order)
            if not final_goal or robot.route_chunk_goal_lm == final_goal:
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            remaining = max(0.0, final_time - robot.route_clock)
            if remaining > lead:
                continue
            start_lm = robot.route_chunk_goal_lm
            planning_goal = self._rolling_planning_goal(start_lm, final_goal, order)
            handoff_pose = self._pose_at_trajectory(
                robot.trajectory,
                final_time,
            ) or self._pose_at_landmark(start_lm)
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
                # Preserve the exact arrival yaw. Resetting to the landmark's
                # synthetic yaw=0 made the body rotate instantaneously at the
                # rolling handoff and could sweep through a nearby robot.
                "startPose": handoff_pose,
            }
            self._attach_spatial_route_to_request(
                request,
                order,
                start_lm,
                planning_goal,
                final_goal,
            )
            candidates.append((remaining, robot.name, order, robot, request, final_goal))
        return [
            (order, robot, request, final_goal, remaining)
            for remaining, _, order, robot, request, final_goal in sorted(candidates)
        ]

    def _ready_rolling_prefetch_entry(
        self,
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float] | None:
        candidates = self._rolling_prefetch_candidates()
        return candidates[0] if candidates else None

    def _rolling_full_collapse_release_entries(
        self,
    ) -> (
        list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]
        | None
    ):
        """Release a completely stopped rolling cohort in vacancy order.

        Ordinary rolling retries remain authoritative while even one active
        simulated robot can still make progress. This path is intentionally
        armed only after every active non-terminal order has exhausted its
        chunk, has no pending/retreat motion, and has already failed a normal
        continuation attempt.
        """
        cohort: list[tuple[FleetOrder, FleetRobot]] = []
        for robot in self._runtime_robots():
            if robot.is_remote() or not robot.active_order_id:
                continue
            order = self.orders.get(robot.active_order_id)
            if order is None or order.status in TERMINAL_ORDER_STATUSES:
                continue
            cohort.append((order, robot))
        if len(cohort) < 2:
            return None
        if any(
            robot.status in {"MOVING", "RETREATING"}
            or robot.pending_route is not None
            or not self._robot_waits_at_rolling_boundary(robot)
            or self._rolling_prefetch_failures.get(robot.name, 0) < 1
            for _, robot in cohort
        ):
            return None

        signature = tuple(sorted(
            (
                robot.name,
                str(robot.route_chunk_goal_lm or ""),
                int(robot.route_revision),
            )
            for _, robot in cohort
        ))
        if signature != self._rolling_vacancy_recovery_signature:
            self._rolling_vacancy_recovery_signature = signature
            self._rolling_vacancy_recovery_blacklist.clear()

        starts = {
            robot.name: str(robot.route_chunk_goal_lm or "")
            for _, robot in cohort
        }
        unique_starts = (
            len(set(starts.values())) == len(starts)
            and all(starts.values())
        )
        dependencies: dict[str, set[str]] = {
            robot.name: set()
            for _, robot in cohort
        }
        by_name = {robot.name: (order, robot) for order, robot in cohort}
        dependencies_complete = unique_starts
        dynamic_blocked_edges = self._dynamic_blocked_edges()
        if dependencies_complete:
            for order, robot in cohort:
                route_info = self._rolling_collapse_route_prefix(
                    order,
                    starts[robot.name],
                    self._active_order_target(order),
                    dynamic_blocked_edges=dynamic_blocked_edges,
                )
                if route_info is None:
                    dependencies_complete = False
                    break
                route_prefix, graph = route_info
                prefix_resources = set()
                for node in route_prefix:
                    prefix_resources.update(graph.vertex_resources(node))
                for src, dst in zip(route_prefix, route_prefix[1:]):
                    lane = graph.lane_for(src, dst)
                    if lane is None:
                        dependencies_complete = False
                        break
                    prefix_resources.update(graph.lane_resources(lane))
                if not dependencies_complete:
                    break
                for other_name, other_start in starts.items():
                    if other_name == robot.name:
                        continue
                    occupancy_resources = set(
                        graph.vertex_resources(other_start)
                    )
                    if prefix_resources.intersection(occupancy_resources):
                        dependencies[robot.name].add(other_name)

        sinks = (
            [
                name
                for name, blocked_by in dependencies.items()
                if not blocked_by
            ]
            if dependencies_complete
            else []
        )
        if dependencies_complete and sinks:
            incoming = {
                name: sum(
                    name in blocked_by
                    for blocked_by in dependencies.values()
                )
                for name in sinks
            }
            sink_name = min(
                sinks,
                key=lambda name: (-incoming[name], name),
            )
            order, robot = by_name[sink_name]
            return [self._rolling_collapse_prefetch_entry(order, robot)]

        vacancy_entry = self._rolling_vacancy_escape_entry(
            cohort,
            signature,
        )
        return [vacancy_entry] if vacancy_entry is not None else []

    def _rolling_collapse_route_prefix(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        *,
        dynamic_blocked_edges: set[tuple[str, str]],
    ) -> tuple[list[str], Any] | None:
        """Recover a non-mutating, resource-sized boundary dependency route."""
        route_nodes = [
            str(node)
            for node in order.spatial_route_nodes
            if str(node) in self.landmarks
        ]
        if start_lm in route_nodes:
            route_nodes = route_nodes[route_nodes.index(start_lm):]
        cached_route_is_valid = bool(
            len(route_nodes) >= 2
            and route_nodes[0] == start_lm
            and route_nodes[-1] == final_goal_lm
            and all(
                dst in self.planner.graph.get(src, [])
                for src, dst in zip(route_nodes, route_nodes[1:])
            )
        )
        if not cached_route_is_valid:
            try:
                route_nodes = [
                    str(node)
                    for node in self.planner.route_planner.find_route(
                        start_lm,
                        final_goal_lm,
                        blocked_edges=(
                            set(order.traffic_detour_edges)
                            | dynamic_blocked_edges
                        ),
                    ).nodes
                ]
            except (RuntimeError, ValueError):
                return None
        if len(route_nodes) < 2:
            return None

        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        graph = self.planner._traffic_graph(
            self.planner._route_speed(route_payload),
        )
        selected_index = graph.extend_route_index_to_controlled_exit(
            route_nodes,
            1,
        )
        selected_index = self._rolling_safe_hold_index(
            route_nodes,
            selected_index,
            final_goal_lm,
            traffic_graph=graph,
        )
        selected_index = min(len(route_nodes) - 1, max(1, selected_index))
        route_prefix = route_nodes[:selected_index + 1]
        if len(route_prefix) < 2:
            return None
        return route_prefix, graph

    def _rolling_collapse_prefetch_entry(
        self,
        order: FleetOrder,
        robot: FleetRobot,
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]:
        final_goal = self._active_order_target(order)
        start_lm = str(robot.route_chunk_goal_lm or "")
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        handoff_pose = (
            self._pose_at_trajectory(robot.trajectory, final_time)
            or self._pose_at_landmark(start_lm)
        )
        request: dict[str, Any] = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": final_goal,
        }
        if handoff_pose is not None:
            request["startPose"] = handoff_pose
        return order, robot, request, final_goal, 0.0

    def _rolling_vacancy_escape_entry(
        self,
        cohort: list[tuple[FleetOrder, FleetRobot]],
        signature: tuple[tuple[str, str, int], ...],
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float] | None:
        """Find one fixed route from a dependency cycle to a free wait pocket."""
        occupied_lms = {
            self._nearest_lm_for_robot(robot)
            for robot in self._runtime_robots()
        }
        occupied_lms.discard("")
        occupied_lms.update(
            str(robot.route_chunk_goal_lm or "")
            for _, robot in cohort
            if robot.route_chunk_goal_lm
        )
        blocked_edges = self._dynamic_blocked_edges()
        candidates: list[
            tuple[
                float,
                str,
                str,
                FleetOrder,
                FleetRobot,
                list[str],
            ]
        ] = []
        for order, robot in sorted(cohort, key=lambda item: item[1].name):
            start_lm = str(robot.route_chunk_goal_lm or "")
            if start_lm not in self.landmarks:
                continue
            route_payload: dict[str, Any] = {}
            if order.speed > 0.0:
                route_payload["speed"] = order.speed
            if order.acceleration > 0.0:
                route_payload["acceleration"] = order.acceleration
            speed = self.planner._route_speed(route_payload)
            acceleration = self.planner._route_acceleration(route_payload)
            graph = self.planner._traffic_graph(speed)
            source_shared_resources = {
                resource
                for resource in graph.vertex_resources(start_lm)
                if resource.kind in {
                    "controlled_region",
                    "mutex_zone",
                    "clearance",
                }
            }
            other_cohort_occupancy = set()
            for _, other in cohort:
                if other.name == robot.name:
                    continue
                other_start = str(other.route_chunk_goal_lm or "")
                if other_start:
                    other_cohort_occupancy.update(
                        graph.vertex_resources(other_start)
                    )
            blocked_lms = occupied_lms - {start_lm}
            horizon = self._rolling_horizon()
            step_limit = self._rolling_horizon_steps()
            queue: list[
                tuple[float, int, str, tuple[str, ...]]
            ] = [(0.0, 0, start_lm, (start_lm,))]
            best_elapsed = {start_lm: 0.0}
            while queue:
                elapsed, edge_count, node, path_tuple = heappop(queue)
                if elapsed > best_elapsed.get(node, float("inf")) + 0.000001:
                    continue
                for neighbour in sorted(self.planner.graph.get(node, [])):
                    lane = graph.lane_for(node, neighbour)
                    if (
                        lane is None
                        or neighbour in blocked_lms
                        or (node, neighbour) in blocked_edges
                        or set(graph.lane_resources(lane)).intersection(
                            other_cohort_occupancy
                        )
                    ):
                        continue
                    next_edge_count = edge_count + 1
                    next_elapsed = elapsed + (
                        self.planner._edge_tick_cost(
                            node,
                            neighbour,
                            speed,
                            acceleration,
                        )
                        * max(0.001, self.planner.time_step_sec)
                    )
                    if (
                        step_limit > 0
                        and next_edge_count > max(1, step_limit)
                    ):
                        continue
                    if (
                        horizon > 0.0
                        and next_edge_count > 1
                        and next_elapsed > horizon + 0.000001
                    ):
                        continue
                    previous_best = best_elapsed.get(neighbour)
                    if (
                        previous_best is not None
                        and previous_best <= next_elapsed + 0.000001
                    ):
                        continue
                    best_elapsed[neighbour] = next_elapsed
                    next_path = (*path_tuple, neighbour)
                    vertex = graph.vertices.get(neighbour)
                    blacklist_key = (signature, robot.name, neighbour)
                    goal_resources = set(
                        graph.vertex_resources(neighbour)
                    )
                    if (
                        neighbour not in occupied_lms
                        and vertex is not None
                        and vertex.can_wait
                        and not source_shared_resources.intersection(
                            goal_resources
                        )
                        and blacklist_key
                        not in self._rolling_vacancy_recovery_blacklist
                    ):
                        candidates.append(
                            (
                                next_elapsed,
                                robot.name,
                                neighbour,
                                order,
                                robot,
                                list(next_path),
                            )
                        )
                        queue.clear()
                        break
                    heappush(
                        queue,
                        (
                            next_elapsed,
                            next_edge_count,
                            neighbour,
                            next_path,
                        ),
                    )

        if not candidates:
            return None
        _, _, pocket_lm, order, robot, route_nodes = min(
            candidates,
            key=lambda item: item[:3],
        )
        entry = self._rolling_collapse_prefetch_entry(order, robot)
        request = entry[2]
        request.update({
            "goalLm": pocket_lm,
            "routeNodes": route_nodes,
            "vacancyRecovery": True,
        })
        return entry

    def _ready_rolling_prefetch_entries(
        self,
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]:
        collapse_release = self._rolling_full_collapse_release_entries()
        if collapse_release is not None:
            return collapse_release
        candidates = self._rolling_prefetch_candidates()
        if not candidates:
            return []
        release_pressure = self._rolling_boundary_release_pressure()
        forced_release = None
        if release_pressure:
            # A terminal holder is the sink of the physical wait chain. Plan
            # it alone first: grouping it with unrelated stopped boundaries
            # can make one CBS/resource failure keep the whole aisle boxed.
            direct_releases = [
                entry
                for entry in candidates
                if float(entry[-1]) <= 0.000001
                and entry[1].name in release_pressure
            ]
            if direct_releases:
                direct_releases.sort(
                    key=lambda entry: (
                        -release_pressure[entry[1].name],
                        self._rolling_prefetch_failures.get(entry[1].name, 0),
                        entry[1].name,
                    )
                )
                forced_release = direct_releases[0]
                if (
                    self._rolling_prefetch_failures.get(
                        forced_release[1].name,
                        0,
                    )
                    == 0
                ):
                    return [forced_release]

        first = forced_release or candidates[0]
        if float(first[-1]) > 0.000001:
            # An ahead-of-time continuation has a different prediction offset
            # from every peer. Keep that inexpensive request independent.
            return [first]

        motion_key = self._order_motion_key(first[0])
        limit = self._rolling_prefetch_recovery_batch_size()
        # Robots already holding at a chunk boundary must be released
        # together. Planning them one-by-one makes every other holder look
        # like a permanent obstacle and causes planner starvation.
        endpoint_entries = [
            entry
            for entry in candidates
            if float(entry[-1]) <= 0.000001
            and self._order_motion_key(entry[0]) == motion_key
        ]
        # Rotate cheap pair attempts through the stopped fleet. Otherwise the
        # lexicographically first pair is retried after every planner timeout
        # while equally blocked neighbours never become movable participants.
        endpoint_entries.sort(
            key=lambda entry: (
                entry[1].name
                != (forced_release[1].name if forced_release is not None else ""),
                self._rolling_prefetch_failures.get(entry[1].name, 0),
                entry[1].name,
            )
        )
        first = forced_release or endpoint_entries[0]
        endpoint_entries = self._rolling_boundary_dependency_component(
            endpoint_entries,
            first,
        )
        seed_failures = self._rolling_prefetch_failures.get(
            first[1].name,
            0,
        )
        if forced_release is None and seed_failures <= 0:
            # First isolate a fresh stopped endpoint.  Coupling two unrelated
            # boundary holders makes one blocked route reject both, and under
            # a full fleet wave that quickly turns every otherwise movable
            # robot into one global retry batch.  Same-LM starts are a real
            # physical component and still need an immediate joint release.
            first_start = str(first[2].get("startLm") or "")
            same_start = [
                entry
                for entry in endpoint_entries
                if str(entry[2].get("startLm") or "") == first_start
            ]
            if len(same_start) <= 1:
                return [first]
            endpoint_entries = same_start
        failures = max(
            (
                self._rolling_prefetch_failures.get(entry[1].name, 0)
                for entry in endpoint_entries
            ),
            default=0,
        )
        # A fixed pair is enough for the usual head-on handoff, but it cannot
        # open a dense boundary where three or more stopped robots mutually
        # occupy the only usable exits. Escalate the local group immediately
        # after a failed attempt while retaining the configured cheap fast
        # path for healthy traffic.
        # Expand only through route dependencies.  Treating every exhausted
        # rolling chunk on the map as one component coupled unrelated aisles:
        # a conflict in one narrow passage then rejected otherwise movable
        # robots in every other passage.
        # A route-overlap component can span most of a dense warehouse even
        # though only its nearest members can release the seed.  Never hand a
        # fleet-wide component to one prioritized-SIPP retry: bounded local
        # waves rotate quickly and let successful endpoints disappear from
        # the next component.
        hard_limit = min(
            len(endpoint_entries),
            self.planner.local_cbs_max_robots,
        )
        limit = min(
            hard_limit,
            max(limit, limit * (2 ** min(4, failures))),
        )
        return endpoint_entries[:limit]

    def _rolling_boundary_dependency_component(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
        ],
        seed: tuple[FleetOrder, FleetRobot, dict[str, Any], str, float],
    ) -> list[
        tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
    ]:
        """Return the local stopped component reachable from ``seed``.

        A boundary holder is a dependency when its start LM lies on another
        holder's committed spatial suffix.  The relation is made undirected
        for recovery because either robot may have to move first.  Breadth
        first ordering lets the cheap 2/4-robot attempts include the nearest
        blockers before a genuinely connected component is expanded.
        """
        by_name = {entry[1].name: entry for entry in entries}
        seed_name = seed[1].name
        if seed_name not in by_name:
            return [seed]

        starts = {
            name: str(entry[2].get("startLm") or "")
            for name, entry in by_name.items()
        }
        route_nodes: dict[str, set[str]] = {}
        for name, entry in by_name.items():
            raw_nodes = entry[2].get("routeNodes", [])
            route_nodes[name] = {
                str(node)
                for node in (
                    raw_nodes if isinstance(raw_nodes, list) else []
                )
                if str(node)
            }
        adjacency = {name: set() for name in by_name}
        names = sorted(by_name)
        for index, name in enumerate(names):
            for other_name in names[index + 1:]:
                same_start = bool(
                    starts[name]
                    and starts[name] == starts[other_name]
                )
                route_dependency = bool(
                    starts[other_name] in route_nodes[name]
                    or starts[name] in route_nodes[other_name]
                )
                if not same_start and not route_dependency:
                    continue
                adjacency[name].add(other_name)
                adjacency[other_name].add(name)

        ordered_names: list[str] = []
        queued = [seed_name]
        seen = {seed_name}
        while queued:
            name = queued.pop(0)
            ordered_names.append(name)
            neighbours = sorted(
                adjacency[name] - seen,
                key=lambda neighbour: (
                    self._rolling_prefetch_failures.get(neighbour, 0),
                    neighbour,
                ),
            )
            seen.update(neighbours)
            queued.extend(neighbours)
        return [by_name[name] for name in ordered_names]

    def _rolling_boundary_release_pressure(self) -> dict[str, int]:
        """Count wait-chain robots trapped behind each exhausted chunk."""
        pressure: dict[str, int] = {}
        for waiter in self._runtime_robots():
            if (
                waiter.status != "WAITING"
                or not waiter.trajectory
                or not self._is_robot_conflict(waiter.last_reason)
            ):
                continue
            current = waiter
            visited = {waiter.name}
            depth = 0
            while True:
                blocker_name = (
                    current.wait_for_robot
                    or self._robot_name_from_conflict_reason(current.last_reason)
                )
                if not blocker_name or blocker_name in visited:
                    break
                visited.add(blocker_name)
                blocker = self.robots.get(blocker_name)
                if blocker is None:
                    break
                depth += 1
                if self._robot_waits_at_rolling_boundary(blocker):
                    pressure[blocker.name] = (
                        pressure.get(blocker.name, 0)
                        + max(1, depth)
                    )
                    break
                if (
                    blocker.status != "WAITING"
                    or not blocker.trajectory
                    or not self._is_robot_conflict(blocker.last_reason)
                ):
                    break
                current = blocker
        return pressure

    def _robot_waits_at_rolling_boundary(self, robot: FleetRobot) -> bool:
        if (
            robot.status != "WAITING"
            or not robot.trajectory
            or not robot.active_order_id
            or not robot.route_chunk_goal_lm
        ):
            return False
        order = self.orders.get(robot.active_order_id)
        if (
            order is None
            or order.status != "PLANNING"
            or self._active_order_target(order) == robot.route_chunk_goal_lm
        ):
            return False
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        return (
            robot.route_clock >= final_time - 0.000001
            or str(robot.last_reason or "") == "rolling continuation pending"
        )

    def _start_async_rolling_prefetch(
        self,
        entries: (
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
            | list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]
        ),
    ) -> None:
        if isinstance(entries, tuple):
            entries = [entries]
        if not entries:
            return
        vacancy_recovery = bool(
            len(entries) == 1
            and entries[0][2].get("vacancyRecovery")
        )
        released_owners = {entry[1].name for entry in entries}
        boundary_recovery = all(
            float(entry[-1]) <= 0.000001
            for entry in entries
        )
        protected_starts = {
            str(entry[2].get("startLm") or "")
            for entry in entries
        }
        reserved_goals: set[str] = set()
        prepared: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]] = []
        for order, robot, raw_request, final_goal, offset in entries:
            request = dict(raw_request)
            start_lm = str(request.get("startLm") or "")
            if vacancy_recovery:
                # This path was selected specifically to break a directed
                # dependency cycle. Keep it fixed; rebuilding the ordinary
                # final-order suffix recreates the same cycle.
                planning_goal = str(request.get("goalLm") or "")
            else:
                planning_goal = (
                    self._rolling_recovery_planning_goal(
                        start_lm,
                        final_goal,
                        order,
                        release_robot_names=released_owners,
                    )
                    if boundary_recovery
                    else self._rolling_planning_goal(
                        start_lm,
                        final_goal,
                        order,
                        release_robot_names=released_owners,
                    )
                )
                planning_goal = self._distinct_rolling_batch_goal(
                    order,
                    start_lm,
                    final_goal,
                    planning_goal,
                    reserved_goals=reserved_goals,
                    protected_starts=protected_starts,
                    release_robot_names=released_owners,
                )
            reserved_goals.add(planning_goal)
            request["goalLm"] = planning_goal
            if not vacancy_recovery:
                request.pop("routeNodes", None)
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    final_goal,
                    release_robot_names=released_owners,
                )
            prepared.append((order, robot, request, final_goal, offset))

        order, _, request, _, offset = prepared[0]
        requests = [entry[2] for entry in prepared]
        payload = self._order_plan_payload(order, request) | {
            "robots": requests,
            "reservationOffsetSec": offset,
            # Boundary holders are movable participants in this request.
            # Let hybrid SIPP use local CBS if priority ordering alone cannot
            # release their coupled starts.
            # A full boundary recovery is intentionally handled by the fast
            # prioritized SIPP pass. Exponential CBS remains useful for a
            # small local conflict, but must never monopolise the runtime
            # planner for a 10-20 robot traffic wave.
            "allowCbsFallback": (
                False
                if vacancy_recovery
                else 1 < len(prepared) <= min(
                    4,
                    self.planner.local_cbs_max_robots,
                )
            ),
        }
        job: dict[str, Any] = {
            "kind": "prefetch_batch" if len(prepared) > 1 else "prefetch",
            "entries": prepared,
            "route_revisions": {
                robot.name: robot.route_revision
                for _, robot, _, _, _ in prepared
            },
            "result": None,
            "done": False,
        }
        if vacancy_recovery:
            job["vacancy_recovery_signature"] = (
                self._rolling_vacancy_recovery_signature
            )
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return
            self._dispatch_job = job

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background prefetch failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-prefetch",
            daemon=True,
        ).start()

    def _rolling_recovery_planning_goal(
        self,
        start_lm: str,
        final_goal_lm: str,
        order: FleetOrder,
        *,
        release_robot_names: set[str],
    ) -> str:
        """Commit only the next graph-safe corridor exit for a stopped batch."""
        try:
            route_nodes = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
                release_robot_names=release_robot_names,
            )
        except ValueError:
            return final_goal_lm
        if len(route_nodes) < 2:
            return final_goal_lm
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        speed = self.planner._route_speed(route_payload)
        traffic_graph = self.planner._traffic_graph(speed)
        exit_index = traffic_graph.extend_route_index_to_controlled_exit(
            route_nodes,
            1,
        )
        exit_index = self._rolling_safe_hold_index(
            route_nodes,
            exit_index,
            final_goal_lm,
            traffic_graph=traffic_graph,
        )
        return str(route_nodes[min(len(route_nodes) - 1, exit_index)])

    def _finish_async_simulated_dispatch(self) -> int:
        with self._dispatch_job_lock:
            job = self._dispatch_job
            if job is None or not bool(job.get("done")):
                return 0
            self._dispatch_job = None
        self._last_async_job_kind = str(job.get("kind") or "dispatch")

        if job.get("kind") in {"prefetch", "prefetch_batch"}:
            return self._finish_async_rolling_prefetch(job)
        if job.get("kind") == "coupled_replan":
            return self._finish_async_coupled_replan(job)

        entries = [
            entry
            for entry in job.get("entries", [])
            if self._async_dispatch_entry_is_current(entry)
        ]
        if not entries:
            return 0
        result = job.get("result")
        if not isinstance(result, dict):
            result = {
                "ok": False,
                "plans": [],
                "debug": {"reason": "background planner returned no result"},
            }
        dispatched, _ = self._finish_simulated_order_batch(entries, result)
        return dispatched

    def _plan_for_robot(self, result: dict[str, Any], robot_name: str) -> dict[str, Any] | None:
        """Return one robot's plan from a shared MAPF planning result."""
        for plan in result.get("plans", []):
            if isinstance(plan, dict) and str(plan.get("robot") or "") == robot_name:
                return plan
        return None

    def _blacklist_failed_rolling_vacancy(
        self,
        job: dict[str, Any],
        robot: FleetRobot,
        request: dict[str, Any],
    ) -> None:
        """Remember a failed pocket only while the collapse is unchanged."""
        if not request.get("vacancyRecovery"):
            return
        raw_signature = job.get("vacancy_recovery_signature")
        if not isinstance(raw_signature, tuple) or not raw_signature:
            return
        signature = tuple(
            (str(name), str(start_lm), int(route_revision))
            for name, start_lm, route_revision in raw_signature
        )
        current_signature = tuple(sorted(
            (
                current.name,
                str(current.route_chunk_goal_lm or ""),
                int(current.route_revision),
            )
            for current in self._runtime_robots()
            if (
                not current.is_remote()
                and current.active_order_id
                and current.active_order_id in self.orders
                and self.orders[current.active_order_id].status
                not in TERMINAL_ORDER_STATUSES
            )
        ))
        if current_signature != signature:
            return
        pocket_lm = str(request.get("goalLm") or "")
        if not pocket_lm:
            return
        self._rolling_vacancy_recovery_blacklist.add(
            (signature, robot.name, pocket_lm)
        )

    def _finish_async_rolling_prefetch(self, job: dict[str, Any]) -> int:
        raw_entries = job.get("entries", [])
        if not isinstance(raw_entries, list):
            return 0
        route_revisions = job.get("route_revisions", {})
        entries = [
            entry
            for entry in raw_entries
            if isinstance(entry, tuple)
            and len(entry) == 5
            and self.orders.get(entry[0].order_id) is entry[0]
            and self.robots.get(entry[1].name) is entry[1]
            and entry[1].active_order_id == entry[0].order_id
            and entry[1].route_revision
            == int(route_revisions.get(entry[1].name, -1))
            and entry[1].route_chunk_goal_lm
            == str(entry[2].get("startLm") or "")
            and bool(entry[1].trajectory)
        ]
        if not entries:
            return 0
        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            reason = (
                self._planner_failure_reason(result)
                if isinstance(result, dict)
                else "rolling prefetch returned no result"
            )
            conflict_robot = self._planner_conflict_robot_name(reason)
            for order, robot, request, _, _ in entries:
                self._blacklist_failed_rolling_vacancy(
                    job,
                    robot,
                    request,
                )
                if not conflict_robot or robot.name == conflict_robot:
                    self._rolling_prefetch_failures[robot.name] = (
                        self._rolling_prefetch_failures.get(robot.name, 0) + 1
                    )
                self._defer_rolling_prefetch(
                    robot,
                    order,
                    # Keep failed boundary members eligible together. Staggered
                    # retries split the recovery wave back into independent
                    # requests and recreated the same stationary blockers.
                    retry_multiplier=(
                        2.0 if robot.name == conflict_robot else 1.0
                    ),
                )
            return 0
        result = self._rolling_result(
            result,
            {robot.name: final_goal for _, robot, _, final_goal, _ in entries},
        )
        for order, robot, request, final_goal, _ in entries:
            plan = self._plan_for_robot(result, robot.name)
            if plan is None or self._wait_only_rolling_plan(plan, final_goal):
                self._blacklist_failed_rolling_vacancy(
                    job,
                    robot,
                    request,
                )
                self._rolling_prefetch_failures[robot.name] = (
                    self._rolling_prefetch_failures.get(robot.name, 0) + 1
                )
                self._defer_rolling_prefetch(
                    robot,
                    order,
                    retry_multiplier=1.0,
                )
                continue
            self._rolling_prefetch_retry_at.pop(robot.name, None)
            self._rolling_prefetch_failures.pop(robot.name, None)
            if self._append_rolling_prefetch(robot, order, plan, final_goal):
                continue
            robot.pending_route = {
                "order_id": order.order_id,
                "start_lm": str(request.get("startLm") or ""),
                "final_goal": final_goal,
                "result": {**result, "plans": [plan]},
            }
        return 0

    def _defer_rolling_prefetch(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        *,
        retry_multiplier: float = 1.0,
    ) -> None:
        failures = max(
            1,
            int(self._rolling_prefetch_failures.get(robot.name, 0) or 0),
        )
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        self._rolling_prefetch_retry_at[robot.name] = (
            self._now()
            + (
                self._order_dispatch_retry_interval(order)
                * (2 ** min(3, failures - 1))
                * time_scale
                * max(1.0, retry_multiplier)
            )
        )

    def _append_rolling_prefetch(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
        final_goal: str,
    ) -> bool:
        """Atomically append a safe future chunk without touching execution.

        A rolling continuation is planned from the current chunk's terminal
        LM. Once it is ready, keeping it in a separate pending route forces a
        zero-based route-clock handoff at that LM. The browser then observes a
        short stop even when both chunks are the same straight graph line.

        Joining the two already time-parameterised trajectories preserves the
        current pose, status, route clock and physics tick. Runtime collision
        checking remains authoritative for every future sample.
        """
        current = [sample for sample in robot.trajectory if isinstance(sample, dict)]
        continuation = [sample for sample in plan.get("trajectory", []) if isinstance(sample, dict)]
        if len(current) < 2 or len(continuation) < 2:
            return False

        expected_start = robot.route_chunk_goal_lm
        plan_start = str(plan.get("startLm") or "").strip()
        if not expected_start or plan_start != expected_start:
            return False
        current_end = current[-1]
        continuation_start = continuation[0]
        position_gap = math.hypot(
            float(current_end.get("x", 0.0) or 0.0)
            - float(continuation_start.get("x", 0.0) or 0.0),
            float(current_end.get("y", 0.0) or 0.0)
            - float(continuation_start.get("y", 0.0) or 0.0),
        )
        if position_gap > self._runtime_replan_lm_tolerance():
            return False

        current_end_time = float(current_end.get("t", 0.0) or 0.0)
        continuation_start_time = float(continuation_start.get("t", 0.0) or 0.0)
        appended: list[dict[str, Any]] = [dict(sample) for sample in current]
        for sample in continuation[1:]:
            shifted = dict(sample)
            shifted["t"] = current_end_time + max(
                0.0,
                float(sample.get("t", continuation_start_time) or continuation_start_time)
                - continuation_start_time,
            )
            appended.append(shifted)
        if float(appended[-1].get("t", 0.0) or 0.0) <= current_end_time + 0.000001:
            return False

        current_nodes = [str(node) for node in robot.plan_nodes]
        continuation_nodes = [str(node) for node in plan.get("nodes", [])]
        if not current_nodes or not continuation_nodes or current_nodes[-1] != continuation_nodes[0]:
            return False
        combined_nodes = current_nodes + continuation_nodes[1:]
        chunk_goal = str(plan.get("goalLm") or continuation_nodes[-1]).strip()
        if not chunk_goal:
            return False

        robot.trajectory = appended
        robot.trajectory_dirty = True
        robot.plan_nodes = combined_nodes
        robot.target_lm = chunk_goal
        robot.route_chunk_goal_lm = chunk_goal
        robot.route_chunk_index = max(0, robot.route_chunk_index + 1)
        robot.route_final_lm = str(plan.get("finalGoalLm") or final_goal).strip()
        robot.route_revision = self._next_route_revision()
        robot.pending_route = None
        robot.has_executed_route = True
        robot.status = "MOVING"
        robot.last_reason = "rolling route continued"
        robot.blocked_since = None
        robot.updated_at = self._now()
        order.route_nodes = list(combined_nodes)
        order.status = "EXECUTING"
        order.error = ""
        order.updated_at = robot.updated_at
        self._update_route_preview(
            robot,
            robot.current_lm,
            robot.route_final_lm,
            blocked_edges=set(order.traffic_detour_edges),
            committed_trajectory=appended,
            committed_nodes=combined_nodes,
            spatial_route_nodes=order.spatial_route_nodes,
        )
        self._event(
            "info",
            f"route continuation committed without stop: {order.order_id} "
            f"{robot.name}->{chunk_goal}",
        )
        return True

    def _start_async_coupled_replan(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> bool:
        if self._last_async_job_kind not in {"prefetch", "prefetch_batch"}:
            prefetch_entries = self._ready_rolling_prefetch_entries()
            if (
                prefetch_entries
                and float(prefetch_entries[0][-1])
                <= self._rolling_prefetch_urgent_lead()
            ):
                # Wait-cycle arbitration runs before dispatch. Preserve an
                # earliest-deadline turn for an expiring rolling trajectory;
                # after that prefetch attempt, a failed cycle may use the
                # following turn so neither recovery class starves.
                return False
        if (
            self._last_async_job_kind != "dispatch"
            and self._queued_simulated_dispatch_waiting(now)
        ):
            # Runtime arbitration runs before the dispatcher on every tick.
            # Without an explicit turn boundary, a different wait-cycle key
            # could occupy the sole planner worker again immediately and
            # leave freshly ARRIVED robots queued indefinitely.
            return False
        cycle_key = tuple(sorted(robot.name for robot in robots))
        if len(cycle_key) < 2:
            return False
        if len(cycle_key) > self.planner.local_cbs_max_robots:
            if cycle_key not in self._coupled_replan_failures:
                self._coupled_replan_failures[cycle_key] = 1
                self.traffic_metrics["coupledReplansFailed"] += 1
                self._event(
                    "warn",
                    f"wait cycle has {len(cycle_key)} robots; local CBS cap is "
                    f"{self.planner.local_cbs_max_robots}",
                )
            return False
        # Re-running the same expensive CBS against unchanged poses cannot
        # produce a different answer. One failed attempt arms deterministic
        # corridor evacuation; another attempt is allowed only after actual
        # traffic progress clears this episode in _record_traffic_progress().
        if self._coupled_replan_failures.get(cycle_key, 0) > 0:
            return False
        last_attempt = self._coupled_replan_last_attempt.get(cycle_key, 0.0)
        if now - last_attempt < self._deadlock_coupled_replan_interval():
            return False

        requests: list[dict[str, Any]] = []
        entries: list[dict[str, Any]] = []
        ordered = [winner] + sorted(
            (robot for robot in robots if robot.name != winner.name),
            key=lambda robot: robot.name,
        )
        for robot in ordered:
            order = self._active_order_for_robot(robot)
            start_lm = self._safe_replan_start_lm(robot)
            final_goal = (
                self._active_order_target(order)
                if order is not None
                else robot.route_final_lm or robot.target_lm
            )
            if (
                order is None
                or not start_lm
                or start_lm not in self.landmarks
                or final_goal not in self.landmarks
            ):
                if cycle_key not in self._coupled_replan_failures:
                    self._coupled_replan_failures[cycle_key] = 1
                    self.traffic_metrics["coupledReplansFailed"] += 1
                    self._event(
                        "warn",
                        f"local CBS cannot start for wait cycle "
                        f"{', '.join(cycle_key)}: {robot.name} is between graph LMs; "
                        "corridor evacuation armed",
                    )
                return False
            planning_goal = self._rolling_planning_goal(start_lm, final_goal, order)
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
                "startPose": (
                    dict(robot.pose)
                    if robot.pose is not None
                    else self._pose_at_landmark(start_lm)
                ),
            }
            requests.append(request)
            entries.append(
                {
                    "robot": robot.name,
                    "order": order.order_id,
                    "start": start_lm,
                    "finalGoal": final_goal,
                    "routeRevision": robot.route_revision,
                }
            )

        first_order = self.orders[str(entries[0]["order"])]
        payload = self._order_plan_payload(first_order, requests[0]) | {
            "robots": requests,
            "plannerBackend": "cbs",
            "allowCbsFallback": True,
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
            "reservedEdgeDetourEnabled": False,
        }
        job: dict[str, Any] = {
            "kind": "coupled_replan",
            "cycle": cycle_key,
            "entries": entries,
            "requests": requests,
            "result": None,
            "done": False,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return False
            self._dispatch_job = job
        self._coupled_replan_last_attempt[cycle_key] = now
        self.traffic_metrics["coupledReplansStarted"] += 1

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background coupled CBS failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-coupled-cbs",
            daemon=True,
        ).start()
        self._event(
            "warn",
            f"local CBS started for wait cycle: {', '.join(cycle_key)}",
        )
        return True

    def _finish_async_coupled_replan(self, job: dict[str, Any]) -> int:
        cycle_key = tuple(str(name) for name in job.get("cycle", ()))
        entries = [entry for entry in job.get("entries", []) if isinstance(entry, dict)]
        current: list[tuple[FleetRobot, FleetOrder, dict[str, Any]]] = []
        for entry in entries:
            robot = self.robots.get(str(entry.get("robot") or ""))
            order = self.orders.get(str(entry.get("order") or ""))
            if (
                robot is None
                or order is None
                or robot.active_order_id != order.order_id
                or order.status in TERMINAL_ORDER_STATUSES
                or robot.route_revision != int(entry.get("routeRevision", -1))
                or self._safe_replan_start_lm(robot) != str(entry.get("start") or "")
            ):
                return 0
            current.append((robot, order, entry))

        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            self._record_coupled_replan_failure(cycle_key, result)
            return 0
        final_goals = {
            robot.name: str(entry.get("finalGoal") or "")
            for robot, _, entry in current
        }
        result = self._rolling_result(result, final_goals)
        plans = {
            str(plan.get("robot") or ""): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        if any(
            robot.name not in plans
            or self._wait_only_rolling_plan(plans[robot.name], final_goals[robot.name])
            for robot, _, _ in current
        ):
            self._record_coupled_replan_failure(cycle_key, result)
            return 0

        now = self._now()
        self._apply_planner_result(result, now)
        for robot, order, entry in current:
            plan = plans[robot.name]
            self._adopt_coupled_spatial_detour(
                order,
                plan,
                str(entry.get("finalGoal") or ""),
            )
            order.route_nodes = [str(node) for node in plan.get("nodes", [])]
            order.status = "EXECUTING"
            order.error = ""
            order.updated_at = now
            self._apply_simulated_route_metadata(robot, order, plan, now)
            robot.last_reason = "local coupled CBS committed"
            robot.last_replan_at = now
            robot.traffic_stall_since = None
            self._clear_wait_dependency(robot)
        self._coupled_replan_failures.pop(cycle_key, None)
        self._active_wait_cycles.pop(cycle_key, None)
        self.traffic_metrics["coupledReplansSucceeded"] += 1
        self.traffic_metrics["cycleReplans"] += 1
        self._event(
            "warn",
            f"local CBS committed for wait cycle: {', '.join(cycle_key)}",
        )
        return len(current)

    def _queued_simulated_dispatch_waiting(self, now: float) -> bool:
        """Return whether a fleet robot has an eligible queued route."""
        for order in self.orders.values():
            if (
                order.status != "QUEUED"
                or not order.vehicle
                or (
                    order.error
                    and now - order.updated_at
                    < self._order_dispatch_retry_interval(order)
                )
                or not self._order_is_robot_queue_head(order)
            ):
                continue
            robot = self.robots.get(order.vehicle)
            if (
                robot is not None
                and not robot.is_remote()
                and self._robot_can_accept_order(robot, explicit=True)
            ):
                return True
        return False

    def _record_coupled_replan_failure(
        self,
        cycle_key: tuple[str, ...],
        result: Any,
    ) -> None:
        self._coupled_replan_failures[cycle_key] = (
            self._coupled_replan_failures.get(cycle_key, 0) + 1
        )
        self.traffic_metrics["coupledReplansFailed"] += 1
        reason = self._planner_failure_reason(result) if isinstance(result, dict) else "no result"
        self._event(
            "warn",
            f"local CBS pending for wait cycle {', '.join(cycle_key)}: {reason}",
        )

    def _async_dispatch_entry_is_current(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str],
    ) -> bool:
        order, robot, request, _ = entry
        if self.orders.get(order.order_id) is not order or order.status != "PLANNING":
            return False
        if self.robots.get(robot.name) is not robot or robot.active_order_id:
            order.status = "QUEUED"
            return False
        start_lm = self._safe_replan_start_lm(robot)
        if start_lm != str(request.get("startLm") or ""):
            order.status = "QUEUED"
            order.error = "robot moved while background plan was running"
            order.updated_at = self._now()
            return False
        return True

    def _order_motion_key(
        self,
        order: FleetOrder,
    ) -> tuple[
        float,
        float,
        bool,
        float,
        bool,
        tuple[tuple[str, str], ...],
    ]:
        return (
            round(float(order.speed), 6),
            round(float(order.acceleration), 6),
            bool(order.rotate),
            round(float(order.turn_speed), 6),
            bool(order.stretch_motion_to_reservation_ticks),
            tuple(
                sorted(
                    (str(src), str(dst))
                    for src, dst in order.traffic_detour_edges
                )
            ),
        )

    def _dispatch_plan_budget(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 2
        try:
            return max(1, min(8, int(fleet.get("dispatch_plan_budget_per_tick", 2) or 2)))
        except (TypeError, ValueError):
            return 2

    def _dispatch_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 2
        try:
            return max(2, min(8, int(fleet.get("dispatch_joint_batch_size", 2) or 2)))
        except (TypeError, ValueError):
            return 2

    def _dispatch_rolling_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1
        try:
            return max(1, min(4, int(fleet.get("dispatch_rolling_batch_size", 1) or 1)))
        except (TypeError, ValueError):
            return 1

    def _rolling_prefetch_recovery_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = int(
                fleet.get(
                    "rolling_prefetch_recovery_batch_size",
                    self.planner.local_cbs_max_robots,
                )
                or self.planner.local_cbs_max_robots
            )
        except (TypeError, ValueError):
            configured = self.planner.local_cbs_max_robots
        return max(
            2,
            min(4, self.planner.local_cbs_max_robots, configured),
        )

    def _dispatch_recovery_group_limit(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        batch_size: int,
    ) -> int:
        """Keep normal rolling jobs cheap, but jointly release starved peers."""
        failures = max(0, int(order.dispatch_failures or 0))
        if failures >= 2:
            # Repeating a failed one-robot request against the same parked
            # neighbours cannot open a dense start area. Do not subsequently
            # shrink this recovery wave back to the ordinary rolling batch of
            # one: the group needs CBS/SIPP to choose a coordinated release.
            return min(
                self.planner.local_cbs_max_robots,
                max(batch_size, 2 + failures),
            )
        if robot.has_executed_route:
            # Healthy rolling continuations already see committed peers as
            # reservations and should remain small at 50-robot scale.
            return min(batch_size, self._dispatch_rolling_batch_size())
        return batch_size

    def _rolling_prefetch_lead(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 5.0
        try:
            configured = float(fleet.get("rolling_prefetch_lead_sec", 5.0) or 5.0)
        except (TypeError, ValueError):
            configured = 5.0
        # The planner runs in wall time while route clocks run in simulation
        # time. At 4x, a fixed 8 simulation-second lead leaves only two real
        # seconds and synchronises the fleet at the rolling boundary. Scale
        # the lead to preserve approximately the same planner deadline.
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        configured *= time_scale
        horizon = self._rolling_horizon()
        upper = max(0.5, horizon * 0.8) if horizon > 0.0 else 5.0
        return max(0.5, min(upper, configured))

    def _rolling_prefetch_urgent_lead(self) -> float:
        # Protect an executing route before starting another robot. The old
        # two-second cap was shorter than the bounded local-CBS budget, so a
        # busy dispatch queue could postpone a continuation until the robot
        # had already stopped at its horizon boundary.
        return max(1.0, min(8.0, self._rolling_prefetch_lead() * 0.4))

    def _order_dispatch_retry_interval(self, order: FleetOrder | None = None) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            base = max(0.1, float(fleet.get("order_dispatch_retry_sec", 0.5) or 0.5))
        except (TypeError, ValueError):
            base = 0.5
        try:
            maximum = max(base, float(fleet.get("order_dispatch_retry_max_sec", 4.0) or 4.0))
        except (TypeError, ValueError):
            maximum = 4.0
        failures = max(0, int(order.dispatch_failures if order is not None else 0))
        return min(maximum, base * (2 ** min(4, max(0, failures - 1))))

    def _dispatch_order(self, order: FleetOrder, force: bool = False) -> bool:
        if not self._order_is_robot_queue_head(order):
            return False
        order.target_lm = self._active_order_target(order)
        candidates = self._candidate_robots_for_order(order)
        if not candidates:
            self._set_order_error(order, "no available robot")
            return False

        failed_reason = ""
        for robot in candidates:
            start_lm = self._safe_replan_start_lm(robot)
            if not start_lm or start_lm not in self.landmarks:
                failed_reason = "robot is between graph landmarks"
                continue
            if start_lm == order.target_lm and not robot.trajectory:
                robot.current_lm = order.target_lm
                robot.target_lm = ""
                robot.status = "ARRIVED"
                robot.active_order_id = ""
                robot.last_reason = "order already at target"
                robot.updated_at = self._now()
                if self._advance_or_complete_order(order, robot, self._now()):
                    self._event("info", f"order completed: {order.order_id} {robot.name}@{order.target_lm}")
                    return True
                return self._dispatch_order(order, force=force)

            planning_goal = (
                order.target_lm
                if robot.is_remote()
                else self._rolling_planning_goal(start_lm, order.target_lm, order)
            )
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
            }
            if robot.pose is not None:
                request["startPose"] = dict(robot.pose)
            if not robot.is_remote():
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    order.target_lm,
                )

            self._set_order_status(order, "PLANNING", robot=robot, start_lm=start_lm)
            result = self._plan_valid_requests([request], self._order_plan_payload(order, request))
            if result.get("ok") and result.get("plans"):
                now = self._now()
                plan = self._plan_for_robot(result, robot.name)
                remote_route: dict[str, Any] | None = None
                if robot.is_remote():
                    if plan is None:
                        failed_reason = "planner did not return robot plan"
                        continue
                    try:
                        remote_route = self._execute_remote_plan(robot, order, plan, result)
                    except Exception as exc:
                        failed_reason = f"remote execute failed: {exc}"
                        robot.remote_error = str(exc)
                        control_conflict = self._is_remote_control_conflict(exc)
                        robot.remote_online = control_conflict
                        robot.status = "MANUAL" if control_conflict else "OFFLINE"
                        robot.last_reason = failed_reason
                        robot.updated_at = now
                        continue
                if not robot.is_remote():
                    result = self._rolling_result(
                        result,
                        {robot.name: order.target_lm},
                    )
                    plan = self._plan_for_robot(result, robot.name)
                    if plan is None:
                        failed_reason = "planner did not return rolling route chunk"
                        continue
                    if self._wait_only_rolling_plan(plan, order.target_lm):
                        detour_queued = (
                            self._order_stall_allows_detour(order)
                            and self._queue_alternate_corridor_detour(
                                order,
                                start_lm,
                                order.target_lm,
                            )
                        )
                        failed_reason = (
                            "traffic window has no progress; alternate corridor queued"
                            if detour_queued
                            else "traffic window has no progress; joint retry pending"
                        )
                        self._event(
                            "warn",
                            f"{robot.name} wait-only route rejected; order kept queued",
                        )
                        continue
                order.route_nodes = [
                    str(item)
                    for plan in result.get("plans", [])
                    if isinstance(plan, dict)
                    for item in plan.get("nodes", [])
                ]
                self._apply_planner_result(result, now, order_id=order.order_id)
                if remote_route is not None:
                    self._apply_remote_route_metadata(robot, remote_route, now)
                elif plan is not None:
                    self._apply_simulated_route_metadata(robot, order, plan, now)
                self._set_order_status(order, "EXECUTING", robot=robot, start_lm=start_lm)
                order.traffic_detour_edges = []
                self._event(
                    "info",
                    f"order dispatched: {order.order_id} {robot.name} {start_lm}->{order.target_lm}",
                )
                return True
            failed_reason = self._planner_failure_reason(result)
            if self._planner_deadlock_result(result):
                robot.status = "WAITING"
                robot.last_reason = failed_reason
                robot.blocked_since = self._now()
                robot.updated_at = self._now()
            if order.vehicle:
                order.status = "QUEUED"
                order.assigned_robot = robot.name
                order.start_lm = start_lm
                order.updated_at = self._now()
            else:
                order.status = "QUEUED"
                order.start_lm = start_lm
                order.updated_at = self._now()

        self._set_order_error(order, failed_reason or "dispatch pending")
        return False

    def _order_plan_payload(self, order: FleetOrder, request: dict[str, Any]) -> dict[str, Any]:
        robot = self.robots.get(str(request.get("name") or ""))
        rolling_continuation = bool(robot is not None and robot.has_executed_route)
        recovery_group = int(order.dispatch_failures or 0) >= 2
        traffic_detour = bool(order.traffic_detour_edges)
        payload: dict[str, Any] = {
            "robots": [request],
            # A 10 second rolling waypoint never needs the global 160 second
            # low-level search.  Bounding the background search prevents one
            # congested request from monopolising Python's GIL and freezing
            # the simulation clock.
            "lowLevelMaxTime": self._runtime_low_level_max_time(),
            # CBS coordinates a newly released group.  For a continuation the
            # other robots are fixed time reservations; CBS cannot move them,
            # so falling back from SIPP only burns several seconds.  Traffic
            # retries/deadlock priority leases handle that case on a fresh tick.
            # After repeated failures the dispatcher deliberately builds a
            # small recovery group, where CBS can coordinate the participants
            # instead of treating every neighbour as an immutable obstacle.
            "allowCbsFallback": not rolling_continuation or recovery_group,
            # A robot without an executable trajectory is a physical obstacle,
            # not a temporal reservation that may disappear after the rolling
            # horizon. Its LM is excluded on the first attempt and a failed
            # detour is kept queued instead of falling back through the body.
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
            "reservedEdgeDetourEnabled": traffic_detour,
        }
        if traffic_detour:
            payload["blocked_edges"] = [
                {"from": src, "to": dst}
                for src, dst in order.traffic_detour_edges
            ]
        if order.speed > 0.0:
            payload["speed"] = order.speed
        if order.acceleration > 0.0:
            payload["acceleration"] = order.acceleration
        payload["rotate"] = bool(order.rotate)
        payload["stretchMotionToReservationTicks"] = bool(order.stretch_motion_to_reservation_ticks)
        if order.turn_speed > 0.0:
            payload["turnSpeed"] = order.turn_speed
        return payload

    def _runtime_low_level_max_time(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured_max = max(8, int(fleet.get("cbs_low_level_max_time", 160) or 160))
        except (TypeError, ValueError):
            configured_max = 160
        # The request may spend one reservation horizon waiting and then one
        # rolling horizon moving.  A small guard absorbs rounding and safety
        # margins while keeping the state space proportional to the window.
        window_sec = self._rolling_horizon() + self._reservation_horizon()
        guard_sec = max(2.0, self._reservation_safety_time() * 4.0)
        ticks = math.ceil((window_sec + guard_sec) / self._reservation_time_step())
        corridor_ticks = self.planner.controlled_corridor_max_ticks()
        if corridor_ticks:
            reservation_ticks = math.ceil(
                self._reservation_horizon() / self._reservation_time_step()
            )
            guard_ticks = math.ceil(guard_sec / self._reservation_time_step())
            ticks = max(ticks, reservation_ticks + corridor_ticks + guard_ticks)
        return max(8, min(configured_max, int(ticks)))

    def _candidate_robots_for_order(self, order: FleetOrder) -> list[FleetRobot]:
        if order.vehicle:
            robot = self.robots.get(order.vehicle)
            if robot is None:
                return []
            return [robot] if self._robot_can_accept_order(robot, explicit=True) else []

        candidates = [
            robot for robot in self._runtime_robots()
            if self._robot_can_accept_order(robot, explicit=False)
        ]
        candidates.sort(
            key=lambda robot: (
                self._lm_distance(self._nearest_lm_for_robot(robot), order.target_lm),
                robot.name,
            )
        )
        return candidates

    def _robot_can_accept_order(self, robot: FleetRobot, explicit: bool = False) -> bool:
        if not self._robot_enabled(robot):
            return False
        if robot.is_remote():
            self._sync_remote_robot(robot, self._now(), force=False)
            if not robot.remote_online:
                return False
            owner_id, _ = self._remote_control_owner(robot)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return False
            if robot.status in {"LOCALIZING", "OFFLINE", "ERROR"}:
                return False
        if robot.active_order_id:
            return False
        if robot.target_lm or robot.trajectory:
            return False
        if robot.status == "STOPPED" and not explicit:
            return False
        if robot.status == "MANUAL" and not explicit:
            return False
        if robot.status in {"MOVING", "WAITING", "PLANNING", "BLOCKED"}:
            return False
        return True

    def _lm_distance(self, start_lm: str, goal_lm: str) -> float:
        start = self.landmarks.get(start_lm)
        goal = self.landmarks.get(goal_lm)
        if start is None or goal is None:
            return float("inf")
        return math.hypot(goal.x - start.x, goal.y - start.y)

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
            elif now - order.traffic_blocked_since >= self._traffic_replan_after():
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
    ) -> bool:
        """Exclude the next corridor only when the same goal stays reachable."""
        if start_lm not in self.landmarks or final_goal_lm not in self.landmarks:
            return False
        # A detour is a one-chunk traffic decision, not permanent map editing.
        # Retry an existing exclusion after backoff instead of accumulating
        # enough exclusions to erode the graph of a long-running order.
        if order.traffic_detour_edges:
            return False
        owner = str(order.vehicle or order.assigned_robot or "").strip()
        stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names={owner} if owner else set(),
        )
        stationary_edges = self._blocked_edges_for_lms(stationary_lms)
        route = None
        existing_nodes = [
            str(node)
            for node in order.spatial_route_nodes
            if str(node) in self.landmarks
        ]
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
            # body. Do not ban an unrelated first edge and destabilise it.
            return False

        src, dst = str(route.nodes[0]), str(route.nodes[1])
        if avoid_lm:
            for candidate_src, candidate_dst in zip(route.nodes, route.nodes[1:]):
                if avoid_lm in {str(candidate_src), str(candidate_dst)}:
                    src, dst = str(candidate_src), str(candidate_dst)
                    break
        candidate = {(src, dst), (dst, src)}
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
            f"avoid {src}<->{dst}, keep goal {final_goal_lm}",
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
