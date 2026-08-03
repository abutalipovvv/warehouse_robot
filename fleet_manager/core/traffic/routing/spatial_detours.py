"""Spatial route selection around congestion and stationary bodies."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
import math
from typing import Any

from fleet_manager.core.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, PlannedRoute


@dataclass(frozen=True, slots=True)
class _WaiterReleaseCheck:
    """Route that must remain possible after the blocker parks."""

    enabled: bool
    start_lm: str
    goal_lm: str
    blocked_edges: frozenset[tuple[str, str]]


@dataclass(frozen=True, slots=True)
class _ClearanceSearchContext:
    """Read-only inputs for the bounded clearance-pocket search."""

    start_lm: str
    max_hops: int
    traffic_graph: Any
    forbidden_lms: frozenset[str]
    blocked_edges: frozenset[tuple[str, str]]
    minimum_clearance: float
    origin: Any
    other_positions: tuple[tuple[float, float], ...]
    node_demand: dict[str, int]
    active_targets: frozenset[str]
    active_target_positions: tuple[tuple[float, float], ...]
    incoming_degree: dict[str, int]


class SpatialDetourMixin:
    """Build and cache graph-safe spatial detours."""

    def _planned_route_from_nodes(self, nodes: list[str]) -> PlannedRoute:
        edges: list[GraphEdge] = []
        for src, dst in zip(nodes, nodes[1:]):
            edge = self.planner.route_planner.get_edge(src, dst)
            if edge is None:
                raise ValueError(f"spatial route contains non-edge {src}->{dst}")
            edges.append(edge)
        return PlannedRoute(
            nodes=list(nodes),
            edges=edges,
            length=sum(float(edge.length) for edge in edges),
        )

    def _ensure_order_spatial_route(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        *,
        release_robot_names: set[str] | None = None,
    ) -> list[str]:
        owner = str(order.vehicle or order.assigned_robot or "").strip()
        released_owners = set(release_robot_names or set())
        if owner:
            released_owners.add(owner)
        stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names=released_owners,
        )
        existing = [
            str(node)
            for node in order.spatial_route_nodes
            if str(node) in self.landmarks
        ]
        if order.internal_kind == "traffic_clearance":
            # Hidden clearance moves are created with one explicit outward
            # route.  Unlike normal order routes this is not a congestion-A*
            # cache: replacing it can acquire the causal owner's controlled
            # corridor and turn a parked-body recovery into a lease cycle.
            # Temporary occupancy is handled by SIPP/admission waits.  If the
            # authored route itself became invalid, fail this bounded
            # maintenance attempt without mutating it so lifecycle cleanup can
            # cancel/requeue from a safe graph LM.
            if start_lm in existing and existing[-1:] == [final_goal_lm]:
                suffix = existing[existing.index(start_lm):]
                if len(suffix) >= 2 and all(
                    dst in self.planner.graph.get(src, [])
                    for src, dst in zip(suffix, suffix[1:])
                ):
                    return suffix
            raise ValueError("traffic clearance route is no longer valid")
        if start_lm in existing and existing and existing[-1] == final_goal_lm:
            suffix = existing[existing.index(start_lm):]
            if all(
                dst in self.planner.graph.get(src, [])
                for src, dst in zip(suffix, suffix[1:])
            ) and not stationary_lms.intersection(suffix):
                return suffix

        # Never leave an invalid cached route available to the next retry or
        # to the web preview. In particular, a robot that finished its order
        # may have parked on a previously valid suffix between rolling MAPF
        # windows. If the detour search below fails, an empty route correctly
        # means "wait for a free corridor", not "try the occupied suffix
        # again".
        if order.spatial_route_nodes:
            order.spatial_route_nodes = []
            order.spatial_route_revision = self._next_route_revision()

        blocked_edges = (
            set(order.traffic_detour_edges)
            | self._dynamic_blocked_edges()
            | self._blocked_edges_for_lms(stationary_lms)
        )
        edge_penalties = (
            self._traffic_route_edge_penalties(order, start_lm, final_goal_lm)
            if self._congestion_routing_enabled()
            else None
        )
        try:
            route = self.planner.route_planner.find_route(
                start_lm,
                final_goal_lm,
                blocked_edges=blocked_edges,
                edge_penalties=edge_penalties,
            )
        except ValueError:
            if not order.traffic_detour_edges:
                raise
            # A deadlock detour excludes one transiently congested edge for a
            # single rolling chunk.  Long-lived package orders can still be
            # waiting when the rest of their wave parks elsewhere; that new
            # occupancy may close the alternate arm while the original arm is
            # now completely free.  Keeping the old exclusion forever leaves
            # an ARRIVED robot with a QUEUED order and freezes the wave
            # barrier.  Re-evaluate once without only that transient ban while
            # retaining every static/dynamic/stationary safety constraint.
            retry_blocked_edges = (
                self._dynamic_blocked_edges()
                | self._blocked_edges_for_lms(stationary_lms)
            )
            route = self.planner.route_planner.find_route(
                start_lm,
                final_goal_lm,
                blocked_edges=retry_blocked_edges,
                edge_penalties=edge_penalties,
            )
            released_edges = list(order.traffic_detour_edges)
            order.traffic_detour_edges = []
            self._event(
                "info",
                f"{owner or order.order_id} released stale traffic detour "
                f"after occupancy changed: "
                + ", ".join(
                    f"{src}->{dst}" for src, dst in released_edges
                ),
            )
        order.spatial_route_nodes = [str(node) for node in route.nodes]
        order.spatial_route_revision = self._next_route_revision()
        order.traffic_blocked_since = None
        return list(order.spatial_route_nodes)

    def _stationary_robot_blocked_lms(
        self,
        *,
        exclude_robot_names: set[str] | None = None,
    ) -> set[str]:
        """Return graph LMs occupied by robots that have no motion timeline.

        Moving/waiting trajectories remain temporal SIPP reservations. An
        enabled IDLE/ARRIVED robot with a QUEUED/PLANNING assignment is a
        commanded departure, not permanent storage. Serialized dispatch must
        be allowed to route the rest of that departure wave; exact runtime
        footprint checks hold an early route until the neighbour really moves.
        STOPPED robots and robots without an assignment remain persistent
        physical obstacles. Coupled request owners are explicitly excluded by
        name so they may receive a coordinated departure plan in that request.
        """
        excluded = exclude_robot_names or set()
        blocked: set[str] = set()
        for robot in self._runtime_robots():
            if robot.name in excluded:
                continue
            exhausted_rolling_holder = self._robot_waits_at_rolling_boundary(
                robot
            )
            if (
                robot.status in {"MOVING", "WAITING", "RETREATING"}
                and robot.trajectory
                and not exhausted_rolling_holder
            ):
                continue
            pending_order = self._active_order_for_robot(robot)
            if (
                robot.status in {"IDLE", "ARRIVED"}
                and pending_order is not None
                and pending_order.status in {"QUEUED", "PLANNING"}
                and not self._stationary_order_is_quarantined(pending_order)
            ):
                continue
            lm_name = self._nearest_lm_for_robot(robot)
            if lm_name in self.landmarks:
                blocked.add(lm_name)
        return blocked

    def _blocked_edges_for_lms(
        self,
        blocked_lms: set[str],
    ) -> set[tuple[str, str]]:
        if not blocked_lms:
            return set()
        return {
            (str(src), str(dst))
            for src, neighbours in self.planner.graph.items()
            for dst in neighbours
            if src in blocked_lms or dst in blocked_lms
        }

    def _stationary_clearance_route(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        *,
        forbidden_lms: set[str] | None = None,
        extra_blocked_edges: set[tuple[str, str]] | None = None,
        avoid_controlled_regions: bool = False,
        start_lm_override: str | None = None,
        require_waiter_release: bool = False,
        require_unowned_controlled_regions: bool = False,
        prospectively_vacated_robot_names: set[str] | None = None,
    ) -> list[str]:
        """Choose a short graph-safe pocket for an inactive traffic blocker.

        This is deliberately a spatial selector only.  The returned route is
        later submitted as a normal internal task, so Rolling SIPP/CBS still
        owns timing, edge motion rules, reservations and collision safety.
        Candidate goals must be legal holding vertices outside controlled
        corridors and clear every robot's *current* body.  Active route demand
        is a soft ordering criterion so a useful pocket is not rejected on a
        busy map, but an unused branch wins whenever one exists.
        """
        requested_start_lm = str(start_lm_override or "").strip()
        prospectively_vacated = {
            str(name)
            for name in (prospectively_vacated_robot_names or set())
            if str(name) and str(name) != blocker.name
        }
        start_lm = (
            requested_start_lm
            if requested_start_lm in self.landmarks
            else self._traffic_lm_for_robot(blocker)
        )
        if start_lm not in self.landmarks:
            return []
        forbidden = {
            str(lm_name)
            for lm_name in (forbidden_lms or set())
            if str(lm_name) in self.landmarks
        }

        graph = self._controlled_corridor_graph
        if graph is None:
            graph = self.planner._traffic_graph(self.planner._route_speed({}))
        stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names={
                blocker.name,
                *prospectively_vacated,
            },
        )
        blocked_edges = (
            self._dynamic_blocked_edges()
            | self._blocked_edges_for_lms(stationary_lms)
            | {
                (str(src), str(dst))
                for src, dst in (extra_blocked_edges or set())
            }
        )
        release_check = self._waiter_release_check(
            waiter,
            blocker,
            require_waiter_release=require_waiter_release,
            prospectively_vacated=prospectively_vacated,
        )
        context = self._clearance_search_context(
            blocker,
            start_lm=start_lm,
            forbidden_lms=forbidden,
            blocked_edges=blocked_edges,
            traffic_graph=graph,
            prospectively_vacated=prospectively_vacated,
        )
        candidates = self._clearance_pocket_candidates(
            waiter,
            blocker,
            context=context,
            avoid_controlled_regions=avoid_controlled_regions,
            require_unowned_controlled_regions=(
                require_unowned_controlled_regions
            ),
        )
        for _, path in sorted(candidates, key=lambda item: item[0]):
            if not self._pocket_releases_waiter(
                str(path[-1]),
                release_check,
            ):
                continue
            if self._clearance_path_crosses_causal_waiter(
                waiter,
                blocker,
                path,
            ):
                continue
            return path
        return []

    def _waiter_release_check(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        *,
        require_waiter_release: bool,
        prospectively_vacated: set[str],
    ) -> _WaiterReleaseCheck:
        """Describe the route the blocker relocation is expected to release."""

        waiter_start_lm = self._safe_replan_start_lm(waiter)
        if waiter_start_lm not in self.landmarks:
            waiter_start_lm = self._traffic_lm_for_robot(waiter)
        waiter_order = self._active_order_for_robot(waiter)
        waiter_goal_lm = (
            self._active_order_target(waiter_order)
            if waiter_order is not None
            else str(waiter.route_final_lm or waiter.target_lm or "")
        )
        waiter_release_check_enabled = bool(
            require_waiter_release
            and waiter_start_lm in self.landmarks
            and waiter_goal_lm in self.landmarks
            and waiter_start_lm != waiter_goal_lm
        )
        waiter_stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names={
                blocker.name,
                waiter.name,
                *prospectively_vacated,
            },
        )
        # A body already parked on the requested goal is a later arrival
        # dependency, not evidence that moving this causal blocker is useless.
        # The waiter can queue outside that goal after the intervening route is
        # opened; treating the goal as a blocked graph cut rejected valid
        # branch pockets such as B->P.
        waiter_stationary_lms.discard(waiter_goal_lm)
        waiter_release_blocked_edges = (
            self._dynamic_blocked_edges()
            | self._blocked_edges_for_lms(waiter_stationary_lms)
        )
        return _WaiterReleaseCheck(
            enabled=waiter_release_check_enabled,
            start_lm=waiter_start_lm,
            goal_lm=waiter_goal_lm,
            blocked_edges=frozenset(waiter_release_blocked_edges),
        )

    def _clearance_search_context(
        self,
        blocker: FleetRobot,
        *,
        start_lm: str,
        forbidden_lms: set[str],
        blocked_edges: set[tuple[str, str]],
        traffic_graph: Any,
        prospectively_vacated: set[str],
    ) -> _ClearanceSearchContext:
        """Collect the immutable geometry and demand used by pocket search."""

        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            max_hops = int(
                fleet.get("parked_clearance_relocation_max_hops", 8) or 8
            )
        except (TypeError, ValueError):
            max_hops = 8
        max_hops = max(2, min(24, max_hops))
        minimum_clearance = max(
            float(self.planner.min_robot_center_distance_m),
            float(self.collision.robot_broadphase_distance()),
        )
        origin = self.landmarks[start_lm]
        other_positions: list[tuple[float, float]] = []
        for robot in self._runtime_robots():
            if (
                robot.name == blocker.name
                or robot.name in prospectively_vacated
            ):
                continue
            if robot.pose is not None:
                other_positions.append((
                    float(robot.pose.get("x", 0.0) or 0.0),
                    float(robot.pose.get("y", 0.0) or 0.0),
                ))
                continue
            lm_name = self._traffic_lm_for_robot(robot)
            landmark = self.landmarks.get(lm_name)
            if landmark is not None:
                other_positions.append((float(landmark.x), float(landmark.y)))

        node_demand: dict[str, int] = {}
        active_targets: set[str] = set()
        for robot in self._runtime_robots():
            if (
                robot.name == blocker.name
                or robot.name in prospectively_vacated
            ):
                continue
            order = self._active_order_for_robot(robot)
            if order is None:
                route_nodes = [str(node) for node in robot.plan_nodes]
            else:
                target = self._active_order_target(order)
                if target:
                    active_targets.add(target)
                route_nodes = [
                    str(node)
                    for node in (order.spatial_route_nodes or robot.plan_nodes)
                    if str(node) in self.landmarks
                ]
            current = self._traffic_lm_for_robot(robot)
            if current in route_nodes:
                route_nodes = route_nodes[route_nodes.index(current):]
            for node in dict.fromkeys(route_nodes[1:]):
                node_demand[node] = node_demand.get(node, 0) + 1
        active_target_positions = [
            (float(self.landmarks[name].x), float(self.landmarks[name].y))
            for name in active_targets
            if name in self.landmarks
        ]

        incoming_degree: dict[str, int] = {}
        for source, neighbours in self.planner.graph.items():
            for destination in neighbours:
                incoming_degree[destination] = incoming_degree.get(destination, 0) + 1
        return _ClearanceSearchContext(
            start_lm=start_lm,
            max_hops=max_hops,
            traffic_graph=traffic_graph,
            forbidden_lms=frozenset(forbidden_lms),
            blocked_edges=frozenset(blocked_edges),
            minimum_clearance=minimum_clearance,
            origin=origin,
            other_positions=tuple(other_positions),
            node_demand=node_demand,
            active_targets=frozenset(active_targets),
            active_target_positions=tuple(active_target_positions),
            incoming_degree=incoming_degree,
        )

    def _clearance_pocket_candidates(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        *,
        context: _ClearanceSearchContext,
        avoid_controlled_regions: bool,
        require_unowned_controlled_regions: bool,
    ) -> list[tuple[tuple[Any, ...], list[str]]]:
        """Run bounded Dijkstra search and rank every legal holding pocket."""

        candidates: list[tuple[tuple[Any, ...], list[str]]] = []
        pending: list[tuple[float, int, str, tuple[str, ...]]] = [
            (0.0, 0, context.start_lm, (context.start_lm,))
        ]
        best_distance: dict[str, float] = {context.start_lm: 0.0}
        visited = 0
        while pending and visited < 512:
            distance, hops, node, path = heappop(pending)
            if distance > best_distance.get(node, float("inf")) + 0.000001:
                continue
            visited += 1
            score = self._clearance_candidate_score(
                waiter,
                context=context,
                node=node,
                path=path,
                distance=distance,
                hops=hops,
            )
            if score is not None:
                candidates.append((score, list(path)))

            if hops >= context.max_hops:
                continue
            for neighbour in self.planner.graph.get(node, []):
                neighbour = str(neighbour)
                if (
                    neighbour in path
                    or (node, neighbour) in context.blocked_edges
                ):
                    continue
                step_distance = self._clearance_transition_distance(
                    blocker,
                    context=context,
                    source=node,
                    destination=neighbour,
                    avoid_controlled_regions=avoid_controlled_regions,
                    require_unowned_controlled_regions=(
                        require_unowned_controlled_regions
                    ),
                )
                if step_distance is None:
                    continue
                next_distance = distance + step_distance
                if next_distance + 0.000001 >= best_distance.get(
                    neighbour,
                    float("inf"),
                ):
                    continue
                best_distance[neighbour] = next_distance
                heappush(
                    pending,
                    (next_distance, hops + 1, neighbour, (*path, neighbour)),
                )
        return candidates

    def _clearance_candidate_score(
        self,
        waiter: FleetRobot,
        *,
        context: _ClearanceSearchContext,
        node: str,
        path: tuple[str, ...],
        distance: float,
        hops: int,
    ) -> tuple[Any, ...] | None:
        landmark = self.landmarks.get(node)
        vertex = context.traffic_graph.vertices.get(node)
        if (
            hops <= 0
            or landmark is None
            or vertex is None
            or not vertex.can_wait
            or vertex.controlled_region_ids
            or node in context.forbidden_lms
            or node in context.active_targets
            or not self._clearance_footprint_is_static_clear(landmark)
            or self._landmark_distance(landmark, context.origin) + 0.000001
            < context.minimum_clearance
            or any(
                math.hypot(
                    float(landmark.x) - x,
                    float(landmark.y) - y,
                ) + 0.000001 < context.minimum_clearance
                for x, y in context.other_positions
            )
            or any(
                math.hypot(
                    float(landmark.x) - x,
                    float(landmark.y) - y,
                ) + 0.000001 < context.minimum_clearance
                for x, y in context.active_target_positions
            )
        ):
            return None

        route_load = sum(
            context.node_demand.get(route_node, 0)
            for route_node in dict.fromkeys(path[1:])
        )
        target_load = context.node_demand.get(node, 0)
        waiter_lm = self.landmarks.get(self._traffic_lm_for_robot(waiter))
        waiter_distance = (
            self._landmark_distance(landmark, waiter_lm)
            if waiter_lm is not None
            else 0.0
        )
        degree = (
            len(self.planner.graph.get(node, []))
            + context.incoming_degree.get(node, 0)
        )
        return (
            target_load > 0,
            target_load,
            route_load,
            0 if vertex.is_parking else 2 if vertex.is_charger else 1,
            degree,
            distance,
            -waiter_distance,
            node,
        )

    def _clearance_footprint_is_static_clear(self, landmark: Any) -> bool:
        """Check a holding LM with representative arrival headings."""

        for sample_index in range(8):
            pose = {
                "x": float(landmark.x),
                "y": float(landmark.y),
                "yaw": (sample_index * math.pi) / 4.0,
            }
            if self.collision.blocked_reason(
                pose=pose,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            ):
                return False
        return True

    @staticmethod
    def _landmark_distance(first: Any, second: Any) -> float:
        return math.hypot(
            float(first.x) - float(second.x),
            float(first.y) - float(second.y),
        )

    def _clearance_transition_distance(
        self,
        blocker: FleetRobot,
        *,
        context: _ClearanceSearchContext,
        source: str,
        destination: str,
        avoid_controlled_regions: bool,
        require_unowned_controlled_regions: bool,
    ) -> float | None:
        graph = context.traffic_graph
        current_vertex = graph.vertices.get(source)
        next_vertex = graph.vertices.get(destination)
        traffic_lane = graph.lane_for(source, destination)
        current_regions = set(
            current_vertex.controlled_region_ids
            if current_vertex is not None
            else ()
        )
        next_regions = set(
            next_vertex.controlled_region_ids
            if next_vertex is not None
            else ()
        )
        if traffic_lane is not None:
            next_regions.update(traffic_lane.controlled_region_ids)
        newly_entered_regions = next_regions.difference(current_regions)
        if avoid_controlled_regions and newly_entered_regions:
            return None
        if (
            require_unowned_controlled_regions
            and newly_entered_regions
            and self._controlled_regions_owned_by_other(
                blocker,
                newly_entered_regions,
            )
        ):
            return None
        edge = self.planner.route_planner.get_edge(source, destination)
        if edge is None:
            return None
        return max(0.001, float(edge.length))

    def _controlled_regions_owned_by_other(
        self,
        blocker: FleetRobot,
        region_ids: set[str],
    ) -> bool:
        for region_id in region_ids:
            physical_owners = set(
                self._controlled_corridor_occupancy.get(region_id, [])
            )
            lease_owner = str(
                self._controlled_corridor_leases.get(
                    region_id,
                    ("", 0.0),
                )[0]
                or ""
            )
            if (
                physical_owners - {blocker.name}
                or lease_owner not in {"", blocker.name}
            ):
                return True
        return False

    def _pocket_releases_waiter(
        self,
        pocket_lm: str,
        check: _WaiterReleaseCheck,
    ) -> bool:
        if not check.enabled:
            return True
        if pocket_lm in {check.start_lm, check.goal_lm}:
            return False
        try:
            self.planner.route_planner.find_route(
                check.start_lm,
                check.goal_lm,
                blocked_edges=(
                    check.blocked_edges
                    | self._blocked_edges_for_lms({pocket_lm})
                ),
            )
        except ValueError:
            return False
        return True

    def _clearance_path_crosses_causal_waiter(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        route_nodes: list[str],
    ) -> bool:
        """Reject a maintenance path that depends on its held waiter moving."""
        state = self._runtime_replans.get(waiter.name)
        if not isinstance(state, dict):
            return False
        waiter_order = self._active_order_for_robot(waiter)
        if (
            waiter_order is None
            or str(state.get("order_id") or "") != waiter_order.order_id
        ):
            return False
        causal_names = {
            str(name)
            for key in ("clearance_blocker_names", "blocker_names")
            for name in state.get(key, ())
            if str(name)
        }
        causal_names.update(
            str(signature[0])
            for signature in state.get("causal_blocker_signatures", ())
            if isinstance(signature, (list, tuple)) and len(signature) == 3
        )
        if blocker.name not in causal_names:
            return False
        return bool(
            self._graph_escape_route_current_body_blocker(
                blocker,
                route_nodes,
                only_robot_names={waiter.name},
            )
        )

    def _traffic_route_edge_penalties(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
    ) -> dict[tuple[str, str], float]:
        """Estimate whole-route demand instead of reacting at the bottleneck.

        Every active robot contributes its remaining committed spatial route.
        Directed load, opposing flow and destination-node demand are soft A*
        costs, so a route always remains available but equal Manhattan paths
        are spread before their ten-second SIPP windows reach the centre.
        Once selected, ``spatial_route_nodes`` keeps the route stable until an
        explicit persistent-stall detour invalidates it.
        """
        owner = order.vehicle or order.assigned_robot
        edge_loads: dict[tuple[str, str], int] = {}
        node_loads: dict[str, int] = {}
        for robot in self._runtime_robots():
            if robot.name == owner:
                continue
            active_order = self._active_order_for_robot(robot)
            if active_order is None or active_order is order:
                continue
            route_nodes = [
                str(node)
                for node in active_order.spatial_route_nodes
                if str(node) in self.landmarks
            ]
            if len(route_nodes) < 2:
                continue
            current = self._traffic_lm_for_robot(robot)
            if current in route_nodes:
                route_nodes = route_nodes[route_nodes.index(current):]
            for node in dict.fromkeys(route_nodes[1:]):
                node_loads[node] = node_loads.get(node, 0) + 1
            for src, dst in zip(route_nodes, route_nodes[1:]):
                edge = (src, dst)
                edge_loads[edge] = edge_loads.get(edge, 0) + 1

        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        edge_weight = self._positive_float_param(
            fleet,
            "congestion_edge_load_penalty_m",
            0.55,
        )
        opposing_weight = self._positive_float_param(
            fleet,
            "congestion_opposing_load_penalty_m",
            1.20,
        )
        node_weight = self._positive_float_param(
            fleet,
            "congestion_node_load_penalty_m",
            0.15,
        )
        tie_break_weight = self._positive_float_param(
            fleet,
            "congestion_tie_break_penalty_m",
            0.002,
        )
        corridor_transition_weight = self._positive_float_param(
            fleet,
            "controlled_corridor_direct_transition_penalty_m",
            4.0,
        )
        corridor_graph = self._controlled_corridor_graph

        node_penalties: dict[str, float] = {}
        for robot in self._runtime_robots():
            if robot.name == owner or robot.pose is None:
                continue
            node = self._traffic_lm_for_robot(robot)
            if not node or node in {start_lm, final_goal_lm}:
                continue
            if robot.status in {"IDLE", "BLOCKED", "PAUSED", "OFFLINE"}:
                penalty = 6.0
            elif robot.status == "WAITING" and self._is_robot_conflict(robot.last_reason):
                penalty = 4.0
            elif robot.status in {"WAITING", "RETREATING"}:
                penalty = 2.0
            else:
                penalty = 0.5
            node_penalties[node] = max(node_penalties.get(node, 0.0), penalty)

        penalties: dict[tuple[str, str], float] = {}
        for edge in self.edges:
            edge_key = (edge.from_name, edge.to_name)
            reverse_key = (edge.to_name, edge.from_name)
            penalty = (
                edge_loads.get(edge_key, 0) * edge_weight
                + edge_loads.get(reverse_key, 0) * opposing_weight
                + node_loads.get(edge.to_name, 0) * node_weight
                + max(
                    node_penalties.get(edge.from_name, 0.0) * 0.35,
                    node_penalties.get(edge.to_name, 0.0),
                )
            )
            lane = (
                corridor_graph.lane_for(edge.from_name, edge.to_name)
                if corridor_graph is not None
                else None
            )
            if (
                lane is not None
                and len(lane.controlled_region_ids) > 1
            ):
                # Prefer the nearby safe holding pocket over a direct
                # no-wait transition between two authored corridor zones.
                # This is a soft cost: a map with no alternative remains
                # routable and is protected by atomic passage admission.
                penalty += corridor_transition_weight
            if owner and tie_break_weight > 0.0:
                stable_value = sum(
                    (index + 1) * ord(character)
                    for index, character in enumerate(
                        f"{owner}|{edge.from_name}|{edge.to_name}"
                    )
                )
                penalty += tie_break_weight * ((stable_value % 997) / 997.0)
            if penalty > 0.0:
                penalties[edge_key] = penalty
        return penalties

    def _congestion_routing_enabled(self) -> bool:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return True
        value = fleet.get("congestion_routing_enabled", True)
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(value)

    def _positive_float_param(
        self,
        values: dict[str, Any],
        key: str,
        default: float,
    ) -> float:
        try:
            value = values.get(key, default)
            if value is None:
                value = default
            return max(0.0, float(value))
        except (TypeError, ValueError):
            return max(0.0, float(default))


__all__ = ["SpatialDetourMixin"]
