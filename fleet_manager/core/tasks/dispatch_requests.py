"""Build deterministic single and joint planner request batches."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

from fleet_manager.core.domain.constants import (
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.traffic.corridor_scheduler import (
    CorridorRequest,
    CorridorSlot,
)


DispatchEntry = tuple[FleetOrder, FleetRobot, dict[str, Any], str]
ManualReconnectTarget = tuple[str, Any]


@dataclass(slots=True)
class _PredispatchCoordination:
    """Graph and fairness state for one departure coordination pass."""

    stationary_entries: list[DispatchEntry]
    by_name: dict[str, DispatchEntry]
    state: dict[str, Any]
    protected_routes: dict[str, tuple[Any, ...]]
    protected_names: set[str]
    local_limit: int = 2
    all_release_names: set[str] = field(default_factory=set)
    routes: dict[str, list[str]] = field(default_factory=dict)
    prefix_edges: int = 4
    starts: dict[str, str] = field(default_factory=dict)
    adjacency: dict[str, set[str]] = field(default_factory=dict)
    components: list[tuple[str, ...]] = field(default_factory=list)
    selected_component: tuple[str, ...] = ()
    coordinated: set[str] = field(default_factory=set)
    attempts: dict[tuple[Any, ...], Any] = field(default_factory=dict)
    attempt_key: tuple[Any, ...] = ()
    external_stationary_lms: tuple[str, ...] = ()


class DispatchRequestBatchMixin:
    """Build deterministic single and joint planner request batches."""

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

    def _register_ready_dispatch_corridor_intents(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ],
        *,
        now: float,
    ) -> None:
        """Publish a fleet snapshot before spending a MAPF worker turn.

        Registering intents only for the first one or two dispatch candidates
        made the calendar change as the rest of a 50-robot wave slowly became
        visible.  Almost every already-computed route was then stale at commit.
        Congestion A* is cached per order and cheap compared with SIPP/CBS, so
        expose the complete ready wave first; the next scheduler tick produces
        one stable fleet-wide ordering.
        """
        if self._controlled_corridor_scheduler is None:
            return
        for order, robot, raw_request, final_goal in entries:
            existing = self._controlled_corridor_prefetch_intents.get(
                robot.name
            )
            if (
                isinstance(existing, dict)
                and self._controlled_corridor_intent_is_current(
                    robot,
                    order,
                    existing,
                )
            ):
                continue
            request = dict(raw_request)
            start_lm = str(request.get("startLm") or "")
            if (
                not start_lm
                or start_lm not in self.landmarks
                or final_goal not in self.landmarks
            ):
                continue
            try:
                planning_goal = self._rolling_planning_goal(
                    start_lm,
                    final_goal,
                    order,
                    release_robot_names={robot.name},
                )
                request["goalLm"] = planning_goal
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    final_goal,
                    release_robot_names={robot.name},
                )
            except ValueError:
                continue
            self._controlled_corridor_prefetch_intent(
                order,
                robot,
                request,
                prediction_offset=0.0,
                now=now,
            )

    def _corridor_dispatch_readiness_key(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str],
    ) -> tuple[int, float, str]:
        """Join deterministic order dispatch with the central slot calendar."""
        order, robot, raw_request, _ = entry
        intent = self._controlled_corridor_prefetch_intents.get(robot.name)
        if not isinstance(intent, dict):
            return 1, 0.0, order.order_id
        if not self._controlled_corridor_intent_is_current(
            robot,
            order,
            intent,
        ):
            return 1, 0.0, order.order_id
        corridor_request = intent.get("request")
        schedule = self._controlled_corridor_schedule
        slot = (
            schedule.slot_for(robot.name)
            if schedule is not None
            else None
        )
        if (
            isinstance(corridor_request, CorridorRequest)
            and isinstance(slot, CorridorSlot)
            and slot.regions == corridor_request.regions
            and slot.direction == corridor_request.direction
            and slot.staging_lm == corridor_request.staging_lm
            and slot.exit_lm == corridor_request.exit_lm
        ):
            return 0, float(slot.entry_time), order.order_id
        start_lm = str(raw_request.get("startLm") or "")
        if (
            isinstance(corridor_request, CorridorRequest)
            and corridor_request.staging_lm != start_lm
        ):
            return 1, float(corridor_request.earliest_entry), order.order_id
        return 2, (
            float(corridor_request.earliest_entry)
            if isinstance(corridor_request, CorridorRequest)
            else 0.0
        ), order.order_id

    def _coordinate_mutual_stationary_departures(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ],
    ) -> tuple[set[str], set[str]]:
        """Make physically coupled fresh departures spatially solvable."""
        context = self._prepare_predispatch_coordination(entries)
        if len(context.stationary_entries) < 2:
            return set(), context.protected_names
        self._discover_predispatch_routes(context)
        if not self._build_predispatch_components(context):
            return set(), context.protected_names
        if self._select_predispatch_component(context):
            return context.coordinated, context.protected_names
        return self._reroute_predispatch_component(context)

    def _prepare_predispatch_coordination(
        self,
        entries: list[DispatchEntry],
    ) -> _PredispatchCoordination:
        stationary_entries = [
            entry
            for entry in entries
            if (
                int(entry[0].dispatch_failures or 0) >= 2
                and entry[0].internal_kind != "traffic_clearance"
                and not entry[1].is_remote()
                and not entry[1].trajectory
                and not entry[1].active_order_id
                and entry[1].status in {"IDLE", "ARRIVED", "BLOCKED"}
                and str(entry[2].get("startLm") or "") in self.landmarks
                and entry[3] in self.landmarks
            )
        ]
        stationary_entries.sort(
            key=lambda entry: (
                -int(entry[0].dispatch_failures or 0),
                float(entry[0].updated_at or 0.0),
                entry[1].name,
            )
        )
        by_name = {entry[1].name: entry for entry in stationary_entries}

        # Keep a tiny, pruned scheduler state rather than using failure counts
        # as a cursor. Failure counts change after every rejected MAPF request
        # and previously made the same physical snapshot look new forever.
        state = getattr(self, "_predispatch_component_state", None)
        if not isinstance(state, dict):
            state = {}
            setattr(self, "_predispatch_component_state", state)
        state.setdefault("last_component", ())
        state.setdefault("component_seeds", {})
        state.setdefault("attempts", {})
        state.setdefault("discovery_cursor", "")
        state.setdefault("protected_routes", {})
        raw_protected = state.get("protected_routes")
        if not isinstance(raw_protected, dict):
            raw_protected = {}
        protected_routes = {
            name: fingerprint
            for name, fingerprint in raw_protected.items()
            if name in by_name
            and fingerprint == self._predispatch_protected_fingerprint(by_name[name])
        }
        state["protected_routes"] = protected_routes
        protected_names = set(protected_routes)
        return _PredispatchCoordination(
            stationary_entries=stationary_entries,
            by_name=by_name,
            state=state,
            protected_routes=protected_routes,
            protected_names=protected_names,
        )

    @staticmethod
    def _predispatch_protected_fingerprint(
        entry: DispatchEntry,
    ) -> tuple[Any, ...]:
        order, _, request, final_goal = entry
        return (
            order.order_id,
            str(request.get("startLm") or ""),
            final_goal,
            int(order.spatial_route_revision or 0),
            tuple(str(node) for node in order.spatial_route_nodes),
        )

    def _discover_predispatch_routes(
        self,
        context: _PredispatchCoordination,
    ) -> None:
        stationary_entries = context.stationary_entries
        state = context.state
        local_limit = max(2, int(self.planner.local_cbs_max_robots))
        all_release_names = {entry[1].name for entry in stationary_entries}
        routes: dict[str, list[str]] = {}

        missing_routes: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ] = []
        for order, robot, request, final_goal in stationary_entries:
            start_lm = str(request.get("startLm") or "")
            existing = [
                str(node)
                for node in order.spatial_route_nodes
                if str(node) in self.landmarks
            ]
            suffix: list[str] = []
            if start_lm in existing and existing[-1:] == [final_goal]:
                candidate = existing[existing.index(start_lm):]
                if all(
                    dst in self.planner.graph.get(src, [])
                    for src, dst in zip(candidate, candidate[1:])
                ):
                    suffix = candidate
            if suffix:
                routes[robot.name] = suffix
            else:
                missing_routes.append((order, robot, request, final_goal))

        # Route discovery is bounded too. Rotating only the missing routes is
        # enough to make candidates beyond the first local-CBS cap visible on
        # later turns without running whole-fleet A* synchronously.
        missing_by_name = {
            entry[1].name: entry
            for entry in missing_routes
        }
        missing_names = sorted(missing_by_name)
        if missing_names:
            cursor = str(state.get("discovery_cursor") or "")
            rotated = [name for name in missing_names if name > cursor]
            rotated.extend(name for name in missing_names if name <= cursor)
            for name in rotated[:local_limit]:
                order, robot, request, final_goal = missing_by_name[name]
                start_lm = str(request.get("startLm") or "")
                try:
                    routes[name] = self._ensure_order_spatial_route(
                        order,
                        start_lm,
                        final_goal,
                        release_robot_names=all_release_names,
                    )
                except ValueError:
                    routes[name] = []
            state["discovery_cursor"] = rotated[
                min(local_limit, len(rotated)) - 1
            ]
        context.local_limit = local_limit
        context.all_release_names = all_release_names
        context.routes = routes

    def _build_predispatch_components(
        self,
        context: _PredispatchCoordination,
    ) -> bool:
        by_name = context.by_name
        state = context.state
        routes = context.routes
        prefix_edges = 4
        starts = {
            name: str(entry[2].get("startLm") or "")
            for name, entry in by_name.items()
        }
        adjacency = {name: set() for name in routes if routes[name]}
        route_names = sorted(adjacency)
        for index, first_name in enumerate(route_names):
            for second_name in route_names[index + 1:]:
                # A MAPF request has one motion payload. Differing speed/turn
                # settings cannot truthfully be called an atomic component.
                if self._order_motion_key(by_name[first_name][0]) != (
                    self._order_motion_key(by_name[second_name][0])
                ):
                    continue
                same_start = bool(
                    starts[first_name]
                    and starts[first_name] == starts[second_name]
                )
                first_crosses_second = starts[second_name] in routes[
                    first_name
                ][1:prefix_edges + 1]
                second_crosses_first = starts[first_name] in routes[
                    second_name
                ][1:prefix_edges + 1]
                if not (
                    same_start
                    or first_crosses_second
                    or second_crosses_first
                ):
                    continue
                adjacency[first_name].add(second_name)
                adjacency[second_name].add(first_name)

        components: list[tuple[str, ...]] = []
        unseen = {name for name, peers in adjacency.items() if peers}
        while unseen:
            seed = min(unseen)
            queued = [seed]
            seen = {seed}
            unseen.remove(seed)
            while queued:
                name = queued.pop(0)
                neighbours = sorted(adjacency[name] - seen)
                seen.update(neighbours)
                unseen.difference_update(neighbours)
                queued.extend(neighbours)
            components.append(tuple(sorted(seen)))
        components.sort()
        if not components:
            state["last_component"] = ()
            state["component_seeds"] = {}
            state["attempts"] = {}
            return False
        context.prefix_edges = prefix_edges
        context.starts = starts
        context.adjacency = adjacency
        context.components = components
        return True

    def _select_predispatch_component(
        self,
        context: _PredispatchCoordination,
    ) -> bool:
        state = context.state
        local_limit = context.local_limit
        adjacency = context.adjacency
        components = context.components
        component_keys = set(components)
        component_seeds = state.get("component_seeds")
        if not isinstance(component_seeds, dict):
            component_seeds = {}
        component_seeds = {
            key: value
            for key, value in component_seeds.items()
            if key in component_keys
        }
        attempts = state.get("attempts")
        if not isinstance(attempts, dict):
            attempts = {}
        attempts = {
            key: value
            for key, value in attempts.items()
            if isinstance(key, tuple) and key and key[0] in component_keys
        }
        state["component_seeds"] = component_seeds
        state["attempts"] = attempts

        last_component = tuple(state.get("last_component") or ())
        selected_component = next(
            (component for component in components if component > last_component),
            components[0],
        )
        state["last_component"] = selected_component

        # Components larger than the configured local solver are traversed by
        # a rotating bounded BFS. This keeps the submitted subset connected
        # while eventually exposing every member of an oversized component.
        previous_seed = str(component_seeds.get(selected_component) or "")
        seed_name = next(
            (name for name in selected_component if name > previous_seed),
            selected_component[0],
        )
        component_seeds[selected_component] = seed_name
        selected_names: list[str] = []
        queued = [seed_name]
        seen = {seed_name}
        while queued and len(selected_names) < local_limit:
            name = queued.pop(0)
            selected_names.append(name)
            neighbours = sorted(adjacency[name] - seen)
            seen.update(neighbours)
            queued.extend(neighbours)
        coordinated = set(selected_names)
        context.selected_component = selected_component
        context.coordinated = coordinated
        attempt_key = (selected_component, tuple(sorted(coordinated)))
        external_stationary_lms = tuple(
            sorted(
                self._stationary_robot_blocked_lms(
                    exclude_robot_names=set(selected_component),
                )
            )
        )
        before_snapshot = self._predispatch_component_snapshot(context)
        before_snapshot = (*before_snapshot, ("blocked", external_stationary_lms))
        context.attempts = attempts
        context.attempt_key = attempt_key
        context.external_stationary_lms = external_stationary_lms
        if attempts.get(attempt_key) == before_snapshot:
            return True
        return False

    @staticmethod
    def _predispatch_component_snapshot(
        context: _PredispatchCoordination,
    ) -> tuple[Any, ...]:
        return tuple(
            (
                name,
                context.by_name[name][0].order_id,
                context.starts[name],
                context.by_name[name][3],
                int(context.by_name[name][1].route_revision or 0),
                int(
                    context.by_name[name][0].spatial_route_revision or 0
                ),
                tuple(context.routes.get(name, [])),
            )
            for name in context.selected_component
        )

    def _reroute_predispatch_component(
        self,
        context: _PredispatchCoordination,
    ) -> tuple[set[str], set[str]]:
        by_name = context.by_name
        protected_routes = context.protected_routes
        protected_names = context.protected_names
        prefix_edges = context.prefix_edges
        starts = context.starts
        routes = context.routes
        selected_component = context.selected_component
        coordinated = context.coordinated
        attempts = context.attempts
        attempt_key = context.attempt_key
        external_stationary_lms = context.external_stationary_lms
        def immediate_conflicts(name: str, route: list[str]) -> set[str]:
            start_lm = starts[name]
            prefix = route[1:prefix_edges + 1]
            return {
                peer_name
                for peer_name in selected_component
                if peer_name != name
                and (
                    starts[peer_name] == start_lm
                    or starts[peer_name] in prefix
                )
            }

        # Lower numerical task priority yields. At equal priority the older or
        # more-failed order keeps the direct route, so the newer/less-starved
        # member considers the bypass first.
        candidates = sorted(
            (by_name[name] for name in coordinated),
            key=lambda entry: (
                int(entry[0].priority or 0),
                int(entry[0].dispatch_failures or 0),
                -float(entry[0].created_at or 0.0),
                entry[1].name,
            ),
        )
        rerouted_names: set[str] = set()
        component_start_lms = {
            starts[name]
            for name in selected_component
            if starts[name]
        }
        for order, robot, request, final_goal in candidates:
            current_route = routes.get(robot.name, [])
            conflicts_before = immediate_conflicts(robot.name, current_route)
            if not conflicts_before:
                continue
            start_lm = str(request.get("startLm") or "")
            peer_start_lms = component_start_lms - {start_lm}
            blocked_edges = (
                self._dynamic_blocked_edges()
                | set(order.traffic_detour_edges)
                | self._blocked_edges_for_lms(set(external_stationary_lms))
                | self._blocked_edges_for_lms(peer_start_lms)
            )
            edge_penalties = (
                self._traffic_route_edge_penalties(
                    order,
                    start_lm,
                    final_goal,
                )
                if self._congestion_routing_enabled()
                else None
            )
            try:
                alternate = self.planner.route_planner.find_route(
                    start_lm,
                    final_goal,
                    blocked_edges=blocked_edges,
                    edge_penalties=edge_penalties,
                )
            except ValueError:
                continue
            alternate_nodes = [str(node) for node in alternate.nodes]
            conflicts_after = immediate_conflicts(
                robot.name,
                alternate_nodes,
            )
            if (
                len(alternate_nodes) < 2
                or alternate_nodes == current_route
                or len(conflicts_after) >= len(conflicts_before)
            ):
                continue

            # The first translation must open clearance from every peer body
            # it used to cross, not merely choose a graph-theoretic detour.
            start_vertex = self.landmarks.get(start_lm)
            next_vertex = self.landmarks.get(alternate_nodes[1])
            opens_clearance = True
            if start_vertex is not None and next_vertex is not None:
                for peer_name in conflicts_before:
                    peer_vertex = self.landmarks.get(starts[peer_name])
                    if peer_vertex is None:
                        continue
                    start_distance = math.hypot(
                        float(start_vertex.x) - float(peer_vertex.x),
                        float(start_vertex.y) - float(peer_vertex.y),
                    )
                    next_distance = math.hypot(
                        float(next_vertex.x) - float(peer_vertex.x),
                        float(next_vertex.y) - float(peer_vertex.y),
                    )
                    if next_distance + 0.000001 < start_distance:
                        opens_clearance = False
                        break
            if not opens_clearance:
                continue

            order.spatial_route_nodes = alternate_nodes
            order.spatial_route_revision = self._next_route_revision()
            order.traffic_detour_attempts += 1
            order.traffic_blocked_since = None
            routes[robot.name] = alternate_nodes
            rerouted_names.add(robot.name)
            protected_routes[robot.name] = self._predispatch_protected_fingerprint(
                by_name[robot.name]
            )
            self._event(
                "warn",
                f"pre-dispatch traffic release: {robot.name} routes around "
                f"stationary departures {', '.join(sorted(conflicts_before))}",
            )

        # Store the post-decision physical snapshot. Dispatch failure counters
        # and timestamps are deliberately absent, so an unchanged component
        # cannot consume A* and allocate route revisions on every retry.
        attempts[attempt_key] = (
            *self._predispatch_component_snapshot(context),
            ("blocked", external_stationary_lms),
        )
        return coordinated, protected_names | rerouted_names

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
        target = self._manual_reconnect_target(order, robot)
        if target is None:
            return False
        reconnect_lm, landmark = target
        trajectory = self._manual_reconnect_trajectory(
            order,
            robot,
            reconnect_lm,
            landmark,
        )
        if not trajectory:
            robot.current_lm = reconnect_lm
            return False
        reason = self._manual_reconnect_blocked_reason(robot, trajectory)
        if reason:
            self._set_order_error(
                order,
                f"manual graph reconnect blocked: {reason}",
            )
            return False
        self._commit_manual_graph_reconnect(
            order,
            robot,
            reconnect_lm,
            final_goal,
            trajectory,
        )
        return True

    def _manual_reconnect_target(
        self,
        order: FleetOrder,
        robot: FleetRobot,
    ) -> ManualReconnectTarget | None:
        """Resolve a simulated manual pose to its nearest graph landmark."""
        if robot.is_remote() or robot.pose is None:
            self._set_order_error(order, "robot has no graph-safe start pose")
            return None
        reconnect_lm = self._nearest_lm_for_robot(robot)
        landmark = self.landmarks.get(reconnect_lm)
        if landmark is None:
            self._set_order_error(order, "no graph landmark near manual pose")
            return None
        return reconnect_lm, landmark

    def _manual_reconnect_trajectory(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        reconnect_lm: str,
        landmark: Any,
    ) -> list[dict[str, Any]]:
        """Generate a sampled rotate-then-drive approach to the graph."""
        if robot.pose is None:
            return []
        start_pose = {
            "x": float(robot.pose.get("x", 0.0) or 0.0),
            "y": float(robot.pose.get("y", 0.0) or 0.0),
            "yaw": float(robot.pose.get("yaw", 0.0) or 0.0),
        }
        dx = float(landmark.x) - start_pose["x"]
        dy = float(landmark.y) - start_pose["y"]
        distance = math.hypot(dx, dy)
        if distance <= 0.000001:
            return []

        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        speed = max(
            0.05,
            float(order.speed or navigation.get("route_speed", 0.35) or 0.35),
        )
        turn_speed = max(
            0.05,
            float(
                order.turn_speed
                or navigation.get("max_angular_speed", 0.9)
                or 0.9
            ),
        )
        heading = math.atan2(dy, dx)
        yaw_delta = math.atan2(
            math.sin(heading - start_pose["yaw"]),
            math.cos(heading - start_pose["yaw"]),
        )
        sample_dt = max(0.03, min(0.08, self.collision.sample_time_step() / 2.0))
        rotate_duration = abs(yaw_delta) / turn_speed
        rotate_steps = (
            max(1, int(math.ceil(rotate_duration / sample_dt)))
            if rotate_duration > 0.01
            else 0
        )
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
        return trajectory

    def _manual_reconnect_blocked_reason(
        self,
        robot: FleetRobot,
        trajectory: list[dict[str, Any]],
    ) -> str:
        """Return the first static or live-footprint collision reason."""
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
                return str(reason)
        return ""

    def _commit_manual_graph_reconnect(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        reconnect_lm: str,
        final_goal: str,
        trajectory: list[dict[str, Any]],
    ) -> None:
        """Commit the validated approach chunk and retain the operator goal."""
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

    def _dispatch_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[int, set[str]]:
        if not entries:
            return 0, set()
        try:
            requests, payload = self._prepare_simulated_order_batch(entries)
        except ValueError as exc:
            for order, _, _, _ in entries:
                if order.status == "PLANNING":
                    order.status = "QUEUED"
                if order.internal_kind == "traffic_clearance":
                    self._set_order_error(
                        order,
                        f"traffic clearance route invalid: {exc}",
                    )
            return 0, {order.order_id for order, _, _, _ in entries}
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
            if order.internal_kind == "traffic_clearance":
                raise
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
        # not grow.  When the requested endpoint is another robot's current
        # stop line, extending *through* it can pull a second controlled
        # corridor into this request and turn one ordinary wait into a long
        # no-wait chain.  Keep the blocked endpoint authoritative in that
        # case: SIPP reports its owner and the next central recovery turn moves
        # the real dependency first.
        for index in range(target_index - 1, 0, -1):
            candidate = str(route_nodes[index])
            if usable(candidate):
                return candidate
        if goal_uses_other_start:
            return planning_goal_lm
        for index in range(target_index + 1, len(route_nodes)):
            candidate = str(route_nodes[index])
            if usable(candidate):
                return candidate
        return planning_goal_lm

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
            return 1
        try:
            return max(
                1,
                min(
                    8,
                    int(
                        fleet.get(
                            "dispatch_joint_batch_size",
                            1,
                        )
                        or 1
                    ),
                ),
            )
        except (TypeError, ValueError):
            return 1

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
        del order
        if robot.has_executed_route:
            # Healthy rolling continuations already see committed peers as
            # reservations and should remain small at 50-robot scale.
            return min(batch_size, self._dispatch_rolling_batch_size())
        return batch_size

    def _dispatch_request_signature(
        self,
        order: FleetOrder,
        robot: FleetRobot,
    ) -> tuple[Any, ...]:
        return (
            order.order_id,
            robot.name,
            self._safe_replan_start_lm(robot),
            self._active_order_target(order),
            int(order.spatial_route_revision or 0),
        )

    def _dispatch_blocker_signature(
        self,
        robot: FleetRobot,
    ) -> tuple[Any, ...]:
        active = self._active_order_for_robot(robot)
        active_id = (
            active.order_id
            if active is not None
            and active.status not in TERMINAL_ORDER_STATUSES
            else str(robot.active_order_id or "")
        )
        return (
            robot.name,
            self._traffic_lm_for_robot(robot),
            int(robot.route_revision),
            str(robot.route_chunk_goal_lm or ""),
            active_id,
            bool(robot.trajectory),
            str(robot.status or ""),
        )

    def _dispatch_conflict_dependency_ready(
        self,
        order: FleetOrder,
    ) -> bool:
        """Wake a failed fresh departure only after its real blocker changes."""
        state = self._dispatch_conflict_dependencies.get(order.order_id)
        if not isinstance(state, dict):
            return True
        owner = str(order.vehicle or order.assigned_robot or "")
        robot = self.robots.get(owner)
        blocker = self.robots.get(str(state.get("blocker") or ""))
        if (
            robot is None
            or blocker is None
            or self._dispatch_request_signature(order, robot)
            != state.get("requester")
            or self._dispatch_blocker_signature(blocker)
            != state.get("blocker_signature")
        ):
            self._dispatch_conflict_dependencies.pop(
                order.order_id,
                None,
            )
            return True
        if self._now() >= float(state.get("probe_at", float("inf"))):
            # A bounded safety probe covers a long graph edge on which the
            # nearest-LM signature may remain unchanged after the conflict
            # footprint has already cleared. It is deliberately much slower
            # than the old 0.5 s retry loop.
            self._dispatch_conflict_dependencies.pop(
                order.order_id,
                None,
            )
            return True
        return False

    def _prune_dispatch_conflict_dependencies(self) -> None:
        for order_id in list(self._dispatch_conflict_dependencies):
            order = self.orders.get(order_id)
            if (
                order is None
                or order.status in TERMINAL_ORDER_STATUSES
            ):
                self._dispatch_conflict_dependencies.pop(
                    order_id,
                    None,
                )

    def _record_dispatch_conflict_dependencies(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ],
        debug: dict[str, Any],
    ) -> dict[str, set[str]]:
        """Persist exact continuous requester→blocker evidence.

        The continuous validator already reports structured identities.  The
        old dispatch path discarded them, copied one failure to an arbitrary
        recovery batch, and progressively grew that batch to eight robots.
        That positive feedback was the main long-running degradation.
        """
        entries_by_robot = {
            robot.name: (order, robot)
            for order, robot, _, _ in entries
        }
        conflicts_by_requester: dict[str, set[str]] = {}
        raw_conflicts = debug.get("continuousUnresolvedConflicts", ())
        if not isinstance(raw_conflicts, (list, tuple)):
            return conflicts_by_requester
        now = self._now()
        for raw_conflict in raw_conflicts:
            if not isinstance(raw_conflict, dict):
                continue
            requester_name = str(raw_conflict.get("robot") or "").strip()
            blocker_name = str(raw_conflict.get("other") or "").strip()
            if (
                requester_name not in entries_by_robot
                or blocker_name not in self.robots
                or blocker_name == requester_name
            ):
                continue
            conflicts_by_requester.setdefault(
                requester_name,
                set(),
            ).add(blocker_name)
            order, requester = entries_by_robot[requester_name]
            blocker = self.robots[blocker_name]
            # Parked, uncommanded bodies already use the relocation/quarantine
            # lifecycle.  This registry is specifically for a blocker whose
            # committed motion can make the same request succeed later.
            blocker_order = self._active_order_for_robot(blocker)
            blocker_is_commanded = bool(
                blocker.trajectory
                or blocker.active_order_id
                or (
                    blocker_order is not None
                    and blocker_order.status
                    not in TERMINAL_ORDER_STATUSES
                )
            )
            if not blocker_is_commanded:
                continue
            try:
                conflict_time = max(
                    0.0,
                    float(raw_conflict.get("time", 0.0) or 0.0),
                )
            except (TypeError, ValueError):
                conflict_time = 0.0
            probe_delay = max(3.0, min(10.0, conflict_time + 1.0))
            self._dispatch_conflict_dependencies[order.order_id] = {
                "requester": self._dispatch_request_signature(
                    order,
                    requester,
                ),
                "blocker": blocker.name,
                "blocker_signature": self._dispatch_blocker_signature(
                    blocker,
                ),
                "resource": str(
                    raw_conflict.get("edge") or "unknown"
                ),
                "source": str(raw_conflict.get("source") or ""),
                "recorded_at": now,
                "probe_at": now + probe_delay,
            }
            requester.last_reason = (
                f"waiting for {blocker.name} before dispatch"
            )
            requester.updated_at = now
        return conflicts_by_requester

    def _order_plan_payload(self, order: FleetOrder, request: dict[str, Any]) -> dict[str, Any]:
        robot = self.robots.get(str(request.get("name") or ""))
        rolling_continuation = bool(robot is not None and robot.has_executed_route)
        recovery_group = int(order.dispatch_failures or 0) >= 2
        # Preparing the request transitions the order to PLANNING and clears
        # its visible error string. The occupancy record is the durable marker
        # that this is the same stationary retry.
        stationary_retry = (
            order.order_id in self._stationary_order_retry_state
        )
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
            "allowCbsFallback": (
                not rolling_continuation
                or recovery_group
                or stationary_retry
            ),
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


__all__ = ["DispatchRequestBatchMixin"]
