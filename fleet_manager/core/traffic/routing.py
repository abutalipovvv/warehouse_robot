"""Spatial routing, congestion costs and traffic-zone admission."""

from __future__ import annotations

from dataclasses import replace
from heapq import heappop, heappush
import math
from typing import Any

from fleet_manager.core.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, PlannedRoute
from fleet_manager.core.traffic.corridor_scheduler import (
    CorridorDecisionStatus,
    CorridorOccupancy,
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSlot,
    CorridorSlotState,
)


class TrafficRoutingMixin:
    """Choose stable rolling routes around congestion and parked robots."""

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
        # Admission intentionally keeps ``current_lm`` on the source side of
        # an edge until the tagged destination sample is reached.  A recovery
        # transaction has a different requirement: MAPF validates startPose
        # against startLm, so a robot already inside the replan tolerance of
        # the next LM must start from that physically authoritative LM.  The
        # override is only supplied by bounded recovery paths; ordinary
        # occupancy/admission continues to use the source-side LM.
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

        def pocket_releases_waiter(pocket_lm: str) -> bool:
            if not waiter_release_check_enabled:
                return True
            if pocket_lm in {waiter_start_lm, waiter_goal_lm}:
                return False
            try:
                self.planner.route_planner.find_route(
                    waiter_start_lm,
                    waiter_goal_lm,
                    blocked_edges=(
                        waiter_release_blocked_edges
                        | self._blocked_edges_for_lms({pocket_lm})
                    ),
                )
            except ValueError:
                return False
            return True

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

        def footprint_is_static_clear(landmark: Any) -> bool:
            # A graph centre can still be too close to a rack/wall for the
            # replacement robot model.  The arrival yaw is chosen later by
            # MAPF, so validate the complete body at representative headings,
            # not merely the landmark point.
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

        candidates: list[tuple[tuple[Any, ...], list[str]]] = []
        pending: list[tuple[float, int, str, tuple[str, ...]]] = [
            (0.0, 0, start_lm, (start_lm,))
        ]
        best_distance: dict[str, float] = {start_lm: 0.0}
        visited = 0
        while pending and visited < 512:
            distance, hops, node, path = heappop(pending)
            if distance > best_distance.get(node, float("inf")) + 0.000001:
                continue
            visited += 1
            landmark = self.landmarks.get(node)
            vertex = graph.vertices.get(node)
            if (
                hops > 0
                and landmark is not None
                and vertex is not None
                and vertex.can_wait
                and not vertex.controlled_region_ids
                and node not in forbidden
                and node not in active_targets
                and footprint_is_static_clear(landmark)
                and math.hypot(
                    float(landmark.x) - float(origin.x),
                    float(landmark.y) - float(origin.y),
                ) + 0.000001 >= minimum_clearance
                and all(
                    math.hypot(
                        float(landmark.x) - x,
                        float(landmark.y) - y,
                    ) + 0.000001 >= minimum_clearance
                    for x, y in other_positions
                )
                and all(
                    math.hypot(
                        float(landmark.x) - x,
                        float(landmark.y) - y,
                    ) + 0.000001 >= minimum_clearance
                    for x, y in active_target_positions
                )
            ):
                route_load = sum(
                    node_demand.get(route_node, 0)
                    for route_node in dict.fromkeys(path[1:])
                )
                target_load = node_demand.get(node, 0)
                waiter_lm = self.landmarks.get(self._traffic_lm_for_robot(waiter))
                waiter_distance = (
                    math.hypot(
                        float(landmark.x) - float(waiter_lm.x),
                        float(landmark.y) - float(waiter_lm.y),
                    )
                    if waiter_lm is not None
                    else 0.0
                )
                degree = len(self.planner.graph.get(node, [])) + incoming_degree.get(node, 0)
                score = (
                    target_load > 0,
                    target_load,
                    route_load,
                    0 if vertex.is_parking else 2 if vertex.is_charger else 1,
                    degree,
                    distance,
                    -waiter_distance,
                    node,
                )
                candidates.append((score, list(path)))

            if hops >= max_hops:
                continue
            for neighbour in self.planner.graph.get(node, []):
                neighbour = str(neighbour)
                if neighbour in path or (node, neighbour) in blocked_edges:
                    continue
                neighbour_vertex = graph.vertices.get(neighbour)
                current_vertex = graph.vertices.get(node)
                traffic_lane = graph.lane_for(node, neighbour)
                current_regions = set(
                    current_vertex.controlled_region_ids
                    if current_vertex is not None
                    else ()
                )
                next_regions = set(
                    neighbour_vertex.controlled_region_ids
                    if neighbour_vertex is not None
                    else ()
                )
                if traffic_lane is not None:
                    next_regions.update(traffic_lane.controlled_region_ids)
                if (
                    avoid_controlled_regions
                    and next_regions.difference(current_regions)
                ):
                    # Emergency portal clearance must move farther into the
                    # external holding area, never enter a narrow resource
                    # merely because a pocket on its far side scores better.
                    # Check the lane as well as the destination: a one-edge
                    # authored corridor has two external stop-line vertices,
                    # so neither endpoint itself carries the region tag.
                    #
                    # A legacy body already *inside* a controlled vertex may
                    # still traverse resources it physically occupies
                    # outward, but it cannot acquire a different region on
                    # that escape. Once it reaches an external vertex this
                    # same guard prevents it from re-entering.
                    continue
                newly_entered_regions = next_regions.difference(
                    current_regions
                )
                if (
                    require_unowned_controlled_regions
                    and newly_entered_regions
                ):
                    blocked_by_authority = False
                    for region_id in newly_entered_regions:
                        physical_owners = set(
                            self._controlled_corridor_occupancy.get(
                                region_id,
                                [],
                            )
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
                            or lease_owner
                            not in {"", blocker.name}
                        ):
                            blocked_by_authority = True
                            break
                    if blocked_by_authority:
                        continue
                edge = self.planner.route_planner.get_edge(node, neighbour)
                if edge is None:
                    continue
                next_distance = distance + max(0.001, float(edge.length))
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

        if not candidates:
            return []
        for _, path in sorted(candidates, key=lambda item: item[0]):
            if not pocket_releases_waiter(str(path[-1])):
                # Moving a parked body from one point of the same unavoidable
                # lane to another cannot release the order. Repeating such
                # moves caused a long chain of successful maintenance orders
                # while the user order made zero progress.
                continue
            if self._clearance_path_crosses_causal_waiter(
                waiter,
                blocker,
                path,
            ):
                # This maintenance move exists solely to release ``waiter``.
                # If its path needs that transactionally held robot to move
                # first, dispatching it creates the lifecycle cycle
                # waiter -> clearance -> waiter. Try another safe pocket; when
                # none exists the waiter's own retained-route replan must run.
                continue
            return path
        return []

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

    def _controlled_corridor_param(self, key: str, default: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        return self._positive_float_param(fleet, key, default)

    def _controlled_corridor_pose_is_at_lm(
        self,
        pose: dict[str, Any] | None,
        lm_name: str,
    ) -> bool:
        if pose is None:
            return False
        landmark = self.landmarks.get(lm_name)
        if landmark is None:
            return False
        # Admission ownership is stricter than the normal replanning
        # tolerance. A robot 5-10 cm beyond the entry LM is already inside and
        # must keep the token until its complete footprint reaches the exit.
        return math.hypot(
            landmark.x - float(pose.get("x", 0.0) or 0.0),
            landmark.y - float(pose.get("y", 0.0) or 0.0),
        ) <= 0.03

    def _controlled_regions_for_robot(self, robot: FleetRobot) -> set[str]:
        graph = self._controlled_corridor_graph
        if graph is None:
            return set()
        regions: set[str] = set()
        current_lm = self._traffic_lm_for_robot(robot)
        vertex = graph.vertices.get(current_lm)
        if vertex is not None:
            regions.update(vertex.controlled_region_ids)

        if not robot.trajectory:
            return regions
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, robot.route_clock)
        )
        if edge is None:
            return regions
        segment_index = self._trajectory_segment_index(
            robot.trajectory,
            robot.route_clock,
            boundary_belongs_to_previous=True,
        )
        segment_start = float(
            robot.trajectory[segment_index].get("t", 0.0) or 0.0
        )
        segment_end = float(
            robot.trajectory[segment_index + 1].get("t", segment_start)
            or segment_start
        )
        src, dst = edge
        lane_lookup = getattr(graph, "lane_for", None)
        if not callable(lane_lookup):
            # Lightweight test/integration graph adapters may expose only
            # vertex annotations. The vertex region is still authoritative;
            # lane occupancy simply cannot add anything in that adapter.
            return regions
        lane = lane_lookup(src, dst)
        if lane is None:
            return regions
        src_vertex = graph.vertices.get(src)
        dst_vertex = graph.vertices.get(dst)
        for region_id in lane.controlled_region_ids:
            at_entry_boundary = (
                (
                    robot.route_clock <= segment_start + 0.000001
                    or self._controlled_corridor_pose_is_at_lm(
                        robot.pose,
                        src,
                    )
                )
                and (
                    src_vertex is None
                    or region_id not in src_vertex.controlled_region_ids
                )
            )
            at_exit_boundary = (
                (
                    robot.route_clock >= segment_end - 0.000001
                    or self._controlled_corridor_pose_is_at_lm(
                        robot.pose,
                        dst,
                    )
                )
                and (
                    dst_vertex is None
                    or region_id not in dst_vertex.controlled_region_ids
                )
            )
            if not at_entry_boundary and not at_exit_boundary:
                regions.add(region_id)
        return regions

    def _controlled_regions_intersecting_footprint(
        self,
        robot: FleetRobot,
    ) -> set[str]:
        """Return authored regions touched by the complete physical body."""
        bounds_by_region = getattr(
            self,
            "_controlled_corridor_region_bounds",
            {},
        )
        route_regions = self._controlled_regions_for_robot(robot)
        if robot.pose is None or not bounds_by_region:
            return route_regions
        corners = self.collision.footprint_corners(robot.pose)
        if not corners:
            return route_regions
        min_x = min(float(point["x"]) for point in corners)
        max_x = max(float(point["x"]) for point in corners)
        min_y = min(float(point["y"]) for point in corners)
        max_y = max(float(point["y"]) for point in corners)
        geometric_regions = {
            str(region_id)
            for region_id, bounds in bounds_by_region.items()
            if (
                max_x >= float(bounds[0])
                and min_x <= float(bounds[2])
                and max_y >= float(bounds[1])
                and min_y <= float(bounds[3])
            )
        }
        # Auto-detected corridors have no authored rectangle to intersect;
        # their ``A<=>B`` region spans the complete inferred lane chain.
        # Explicit edge-only regions can likewise lack annotated vertices.
        geometric_regions.update(
            region_id
            for region_id in route_regions
            if (
                region_id not in bounds_by_region
                or "<=>" in region_id
            )
        )
        return geometric_regions

    def _controlled_corridor_staging_lm(
        self,
        robot: FleetRobot,
        *,
        portal_lm: str,
        portal_clock: float,
    ) -> tuple[str, float]:
        """Return the closest upstream LM that leaves the portal clear.

        The graph endpoint immediately outside a controlled corridor is also
        the exit point for traffic travelling in the opposite direction.  A
        red light on that endpoint can therefore prevent the current owner
        from completing its turn out of the corridor.  Walk backwards along
        the committed trajectory and place the stop line at the first safe LM
        whose centre is outside the robot/robot broadphase around the portal.

        Maps with no earlier graph LM keep the portal as a compatibility
        fallback.  They remain capacity safe, but cannot provide the extra
        exit pocket without an upstream waiting point in the graph.
        """
        graph = self._controlled_corridor_graph
        portal = self.landmarks.get(portal_lm)
        if graph is None or portal is None:
            return portal_lm, portal_clock

        clearance = max(0.0, self.collision.robot_broadphase_distance())
        fallback = (portal_lm, portal_clock)
        seen: set[str] = set()
        for sample in reversed(robot.trajectory):
            sample_clock = float(sample.get("t", 0.0) or 0.0)
            if sample_clock > portal_clock + 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if not lm_name or lm_name in seen:
                continue
            seen.add(lm_name)
            vertex = graph.vertices.get(lm_name)
            landmark = self.landmarks.get(lm_name)
            edge = self._parse_edge_id(str(sample.get("edgeId") or ""))
            incoming_lane = graph.lane_for(*edge) if edge is not None else None
            crosses_controlled_resource = bool(
                (vertex is not None and vertex.controlled_region_ids)
                or (
                    incoming_lane is not None
                    and incoming_lane.controlled_region_ids
                )
            )
            if (
                vertex is None
                or landmark is None
                or not vertex.can_wait
                or vertex.controlled_region_ids
            ):
                if crosses_controlled_resource:
                    break
                continue
            fallback = (lm_name, sample_clock)
            if crosses_controlled_resource:
                # This is the graph-safe exit of a preceding narrow passage.
                # A stop line before that passage is not reachable from the
                # current approach leg and can already lie behind the robot.
                break
            distance = math.hypot(
                float(landmark.x) - float(portal.x),
                float(landmark.y) - float(portal.y),
            )
            if distance + 0.000001 >= clearance:
                return lm_name, sample_clock
        return fallback

    def _next_controlled_corridor_entry(
        self,
        robot: FleetRobot,
        *,
        lookahead_sec: float | None = None,
    ) -> dict[str, Any] | None:
        """Return the complete no-wait passage approaching ``robot``.

        Geometric corridor rectangles may touch through an edge whose two
        endpoints are both non-waitable.  A per-rectangle traffic light is
        unsafe there: after entering region A the robot can be denied region B
        with nowhere legal to stop.  Admission is therefore computed from the
        last external safe LM through the first following external safe LM and
        contains every controlled resource crossed in between.
        """
        graph = self._controlled_corridor_graph
        if graph is None or len(robot.trajectory) < 2:
            if self.robots.get(robot.name) is robot:
                self._controlled_corridor_entry_cache.pop(
                    robot.name,
                    None,
                )
            return None
        lookahead = (
            self._controlled_corridor_entry_lookahead()
            if lookahead_sec is None
            else max(0.0, float(lookahead_sec))
        )
        cache_name = (
            robot.name
            if self.robots.get(robot.name) is robot
            else ""
        )
        pose = robot.pose if isinstance(robot.pose, dict) else {}
        cache_key = (
            int(robot.route_revision),
            len(robot.trajectory),
            float(robot.route_clock),
            str(robot.current_lm or ""),
            float(pose.get("x", 0.0) or 0.0),
            float(pose.get("y", 0.0) or 0.0),
            float(pose.get("yaw", 0.0) or 0.0),
            float(lookahead),
        )
        if cache_name:
            cached = self._controlled_corridor_entry_cache.get(cache_name)
            if (
                cached is not None
                and cached[0] is robot.trajectory
                and cached[1] == cache_key
            ):
                return cached[2]

        inside = self._controlled_regions_for_robot(robot)
        first_index = max(
            0,
            self._trajectory_segment_index(
                robot.trajectory,
                robot.route_clock,
                boundary_belongs_to_previous=True,
            ) - 1,
        )
        entry: dict[str, Any] | None = None
        passage_regions: list[str] = []
        resource_windows: dict[str, dict[str, Any]] = {}
        for index in range(first_index, len(robot.trajectory) - 1):
            start = robot.trajectory[index]
            end = robot.trajectory[index + 1]
            start_time = float(start.get("t", 0.0) or 0.0)
            end_time = float(end.get("t", start_time) or start_time)
            if end_time + 0.000001 < robot.route_clock:
                continue
            eta = max(0.0, start_time - robot.route_clock)
            if entry is None and eta > lookahead + 0.000001:
                break
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            edge = self._parse_edge_id(edge_id)
            if edge is None:
                continue
            src, dst = edge
            lane = graph.lane_for(src, dst)
            if lane is None:
                continue
            # A rendered/commanded trajectory contains several interpolated
            # samples with the same ``src->dst`` edge id.  The edge's graph
            # destination is reached only by the sample explicitly tagged with
            # that LM; treating the first interpolation sample as arrival made
            # the central calendar release a corridor almost an edge too early.
            reaches_dst_lm = str(end.get("lm") or "").strip() == dst
            src_vertex = graph.vertices.get(src)
            dst_vertex = graph.vertices.get(dst)
            lane_regions = tuple(lane.controlled_region_ids)
            if (
                entry is None
                and inside
                and reaches_dst_lm
                and dst_vertex is not None
                and dst_vertex.can_wait
                and not dst_vertex.controlled_region_ids
            ):
                # The currently occupied passage ends at this graph-safe
                # vertex.  A later route segment may enter the same or another
                # controlled corridor again, but that is a separate admission
                # transaction.  Looking through this safe exit bundled the
                # future re-entry with the passage being cleared and could
                # make its owner wait for its own lease before it had even
                # reached the intervening turn.
                if cache_name:
                    self._controlled_corridor_entry_cache[cache_name] = (
                        robot.trajectory,
                        cache_key,
                        None,
                    )
                return None
            if entry is None:
                new_regions = [
                    region_id
                    for region_id in lane_regions
                    if region_id not in inside
                    and not (
                        self._controlled_corridor_pose_is_at_lm(
                            robot.pose,
                            dst,
                        )
                        and (
                            dst_vertex is None
                            or region_id
                            not in dst_vertex.controlled_region_ids
                        )
                    )
                ]
                if not new_regions:
                    continue
                source_is_safe = bool(
                    src_vertex is not None
                    and src_vertex.can_wait
                    and not src_vertex.controlled_region_ids
                )
                if not source_is_safe:
                    # The robot is already inside an older/legacy passage.
                    # It must keep moving to the next safe exit; creating a
                    # new red light here would stop it in the narrow space.
                    continue
                staging_lm, staging_clock = (
                    self._controlled_corridor_staging_lm(
                        robot,
                        portal_lm=src,
                        portal_clock=start_time,
                    )
                )
                at_staging = self._controlled_corridor_pose_is_at_lm(
                    robot.pose,
                    staging_lm,
                )
                entry = {
                    "region": new_regions[0],
                    "src": src,
                    "dst": dst,
                    "holding_lm": staging_lm,
                    "staging_clock": staging_clock,
                    "exit_lm": "",
                    "eta": eta,
                    "entry_clock": start_time,
                    "exit_clock": end_time,
                    "at_boundary": self._controlled_corridor_pose_is_at_lm(
                        robot.pose,
                        src,
                    ),
                    "at_staging": at_staging,
                    "passed_staging": bool(
                        robot.route_clock > staging_clock + 0.000001
                        and not at_staging
                    ),
                }

            local_direction = self._controlled_corridor_lane_direction(
                src,
                dst,
            )
            segment_regions = tuple(dict.fromkeys(
                (
                    *lane_regions,
                    *(
                        dst_vertex.controlled_region_ids
                        if dst_vertex is not None
                        else ()
                    ),
                )
            ))
            for region_id in segment_regions:
                window = resource_windows.get(region_id)
                if window is None:
                    resource_windows[region_id] = {
                        "entry_clock": start_time,
                        "exit_clock": end_time,
                        "direction": local_direction,
                        "directions": [local_direction],
                    }
                    continue
                window["entry_clock"] = min(
                    float(window["entry_clock"]),
                    start_time,
                )
                window["exit_clock"] = max(
                    float(window["exit_clock"]),
                    end_time,
                )
                directions = window.setdefault(
                    "directions",
                    [str(window["direction"])],
                )
                if not directions or directions[-1] != local_direction:
                    directions.append(local_direction)
                window["direction"] = (
                    str(directions[0])
                    if len(directions) == 1
                    else "flow:path:"
                    + ">".join(
                        str(direction).removeprefix("flow:")
                        for direction in directions
                    )
                )

            for region_id in lane_regions:
                if region_id not in passage_regions:
                    passage_regions.append(region_id)
            if dst_vertex is not None:
                for region_id in dst_vertex.controlled_region_ids:
                    if region_id not in passage_regions:
                        passage_regions.append(region_id)
                if (
                    reaches_dst_lm
                    and dst_vertex.can_wait
                    and not dst_vertex.controlled_region_ids
                ):
                    entry["exit_lm"] = dst
                    entry["exit_clock"] = end_time
                    entry["direction"] = (
                        f"{entry.get('src') or src}->{dst}"
                    )
                    break

        if entry is None or not passage_regions:
            if cache_name:
                self._controlled_corridor_entry_cache[cache_name] = (
                    robot.trajectory,
                    cache_key,
                    None,
                )
            return None
        entry["regions"] = tuple(passage_regions)
        entry["passage"] = "|".join(passage_regions)
        staging_clock = float(
            entry.get("staging_clock", entry.get("entry_clock", 0.0))
            or 0.0
        )
        passage_duration = max(
            self._runtime_motion_step(),
            float(entry.get("exit_clock", staging_clock) or staging_clock)
            - staging_clock,
        )
        exit_clock = float(
            entry.get("exit_clock", staging_clock) or staging_clock
        )
        passage_samples = [
            sample
            for sample in robot.trajectory
            if (
                isinstance(sample, dict)
                and staging_clock - 0.000001
                <= float(sample.get("t", 0.0) or 0.0)
                <= exit_clock + 0.000001
            )
        ]
        entry["no_wait_lms"] = tuple(dict.fromkeys(
            lm_name
            for sample in passage_samples
            if (
                float(sample.get("t", 0.0) or 0.0)
                > staging_clock + 0.000001
                and float(sample.get("t", 0.0) or 0.0)
                < exit_clock - 0.000001
                and (lm_name := str(sample.get("lm") or "").strip())
            )
        ))
        entry["has_wait_after_staging"] = any(
            (
                float(second.get("t", 0.0) or 0.0)
                > float(first.get("t", 0.0) or 0.0) + 0.000001
                and float(first.get("t", 0.0) or 0.0)
                > staging_clock + 0.000001
                and math.hypot(
                    float(second.get("x", 0.0) or 0.0)
                    - float(first.get("x", 0.0) or 0.0),
                    float(second.get("y", 0.0) or 0.0)
                    - float(first.get("y", 0.0) or 0.0),
                )
                <= 0.000001
                and abs(
                    math.atan2(
                        math.sin(
                            float(second.get("yaw", 0.0) or 0.0)
                            - float(first.get("yaw", 0.0) or 0.0)
                        ),
                        math.cos(
                            float(second.get("yaw", 0.0) or 0.0)
                            - float(first.get("yaw", 0.0) or 0.0)
                        ),
                    )
                )
                <= 0.000001
            )
            for first, second in zip(
                passage_samples,
                passage_samples[1:],
            )
        )
        entry["resource_windows"] = tuple(
            CorridorResourceWindow(
                region_id=region_id,
                entry_offset_sec=max(
                    0.0,
                    float(resource_windows[region_id]["entry_clock"])
                    - staging_clock,
                ),
                exit_offset_sec=min(
                    passage_duration,
                    max(
                        self._runtime_motion_step(),
                        float(resource_windows[region_id]["exit_clock"])
                        - staging_clock,
                    ),
                ),
                direction=str(resource_windows[region_id]["direction"]),
            )
            for region_id in passage_regions
            if region_id in resource_windows
        )
        if cache_name:
            self._controlled_corridor_entry_cache[cache_name] = (
                robot.trajectory,
                cache_key,
                entry,
            )
        return entry

    def _controlled_corridor_prefetch_intent(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        *,
        prediction_offset: float,
        now: float,
    ) -> dict[str, Any] | None:
        """Register the first authored-corridor passage before MAPF.

        A normal committed trajectory is already visible to the central
        calendar. A rolling continuation is not: without this intent SIPP can
        discover a downstream reservation only after entering a no-wait
        chain, reject the plan, and repeat forever. A nominal kinematic
        timeline is sufficient for admission; the finished MAPF trajectory is
        rechecked against the live slot before it is appended.
        """
        scheduler = self._controlled_corridor_scheduler
        if scheduler is None:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        route_nodes = [
            str(node)
            for node in request.get("routeNodes", ())
            if str(node) in self.landmarks
        ]
        if len(route_nodes) < 2:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        start_lm = str(request.get("startLm") or "")
        if route_nodes[0] != start_lm:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        if order.acceleration > 0.0:
            route_payload["acceleration"] = order.acceleration
        speed = self.planner._route_speed(route_payload)
        acceleration = self.planner._route_acceleration(route_payload)
        start_pose = request.get("startPose")
        start_yaw = (
            float(start_pose.get("yaw", 0.0) or 0.0)
            if isinstance(start_pose, dict)
            else 0.0
        )
        trajectory = self.planner._trajectory_for_nodes(
            route_nodes,
            speed,
            acceleration=acceleration,
            rotate_enabled=bool(order.rotate),
            turn_speed=(
                order.turn_speed
                if order.turn_speed > 0.0
                else self.planner._turn_speed({})
            ),
            stretch_motion_to_reservation_ticks=(
                order.stretch_motion_to_reservation_ticks
            ),
            start_yaw=start_yaw,
        )
        if len(trajectory) < 2:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        pose = (
            {
                "x": float(start_pose.get("x", 0.0) or 0.0),
                "y": float(start_pose.get("y", 0.0) or 0.0),
                "yaw": start_yaw,
            }
            if isinstance(start_pose, dict)
            else self._pose_at_landmark(start_lm)
        )
        synthetic = FleetRobot(
            name=robot.name,
            current_lm=start_lm,
            target_lm=str(request.get("goalLm") or route_nodes[-1]),
            status="MOVING",
            active_order_id=robot.active_order_id,
            pose=pose,
            trajectory=trajectory,
            route_clock=0.0,
            route_revision=int(robot.route_revision),
        )
        entry = self._next_controlled_corridor_entry(synthetic)
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            entry is None
            or not regions
            or not set(regions).issubset(scheduler.controlled_regions)
        ):
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        exit_lm = str(entry.get("exit_lm") or "")
        staging_lm = str(
            entry.get("holding_lm")
            or entry.get("src")
            or ""
        )
        if exit_lm not in self.landmarks or staging_lm not in self.landmarks:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        direction = self._controlled_corridor_flow_direction(entry)
        staging_clock = max(
            0.0,
            float(entry.get("staging_clock", 0.0) or 0.0),
        )
        exit_clock = max(
            staging_clock + self._runtime_motion_step(),
            float(
                entry.get("exit_clock", staging_clock)
                or staging_clock
            ),
        )
        intent_kind = (
            "rolling"
            if (
                robot.active_order_id == order.order_id
                and bool(robot.trajectory)
            )
            else "dispatch"
        )
        signature = (
            int(robot.route_revision),
            order.order_id,
            intent_kind,
            start_lm,
            tuple(route_nodes),
            regions,
            direction,
            staging_lm,
            exit_lm,
        )
        handoff_at = now + max(0.0, float(prediction_offset))
        earliest_entry = handoff_at + staging_clock
        existing = self._controlled_corridor_prefetch_intents.get(
            robot.name
        )
        if (
            isinstance(existing, dict)
            and existing.get("signature") == signature
        ):
            # Congestion routing may rebuild the order suffix while preserving
            # this exact rolling chunk and first corridor passage.  The intent
            # signature already proves that its executable route is unchanged;
            # refresh only the parent-route generation so the same object is
            # not declared stale a few milliseconds after worker start.
            existing["spatial_route_revision"] = int(
                order.spatial_route_revision or 0
            )
            # Do not invalidate the just-produced schedule for sub-tick ETA
            # drift.  The gate below compares the slot with the current
            # nominal staging time and refreshes only a genuinely missed
            # command.  Rewriting the epoch here made a stopped boundary
            # advance its ETA on every tick and therefore never become ready.
            return existing

        corridor_request = CorridorRequest(
            robot_id=robot.name,
            regions=regions,
            direction=direction,
            earliest_entry=earliest_entry,
            duration_sec=max(
                self._runtime_motion_step(),
                exit_clock - staging_clock,
            ),
            staging_lm=staging_lm,
            exit_lm=exit_lm,
            route_revision=int(robot.route_revision),
            priority=float(order.priority or 0),
            wait_age_sec=0.0,
            deadline=None,
            downstream_available=True,
            entered=False,
            past_commit_point=False,
            # This is only a future route proposal. It becomes immutable in
            # ``commit_slot`` after the exact SIPP trajectory has been
            # revalidated; wall-clock proximity alone must never create a
            # green command for an idle robot.
            requires_explicit_commit=True,
            resource_windows=tuple(
                window
                for window in entry.get("resource_windows", ())
                if isinstance(window, CorridorResourceWindow)
            ),
        )
        intent = {
            "signature": signature,
            "kind": intent_kind,
            "order_id": order.order_id,
            "start_lm": start_lm,
            "route_revision": int(robot.route_revision),
            "spatial_route_revision": int(
                order.spatial_route_revision or 0
            ),
            "trajectory_route_nodes": tuple(route_nodes),
            "request": corridor_request,
            "entry": dict(entry),
            "trajectory": trajectory,
            "start_pose": pose,
            "registered_at": now,
            "handoff_at": handoff_at,
            "last_schedule_epoch": None,
        }
        self._controlled_corridor_prefetch_intents[robot.name] = intent
        return intent

    def _controlled_corridor_intent_is_current(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        intent: dict[str, Any],
    ) -> bool:
        """Validate a future passage without relying on positional tuples."""
        signature = intent.get("signature")
        raw_request = intent.get("request")
        kind = str(intent.get("kind") or "")
        start_lm = str(intent.get("start_lm") or "")
        if (
            not isinstance(signature, tuple)
            or len(signature) != 9
            or not isinstance(raw_request, CorridorRequest)
            or kind not in {"dispatch", "rolling"}
            or self.orders.get(order.order_id) is not order
            or self.robots.get(robot.name) is not robot
            or order.status in TERMINAL_ORDER_STATUSES
            or str(intent.get("order_id") or "") != order.order_id
            or int(intent.get("route_revision", -1))
            != int(robot.route_revision)
            or int(intent.get("spatial_route_revision", -1))
            != int(order.spatial_route_revision or 0)
            or signature
            != (
                int(robot.route_revision),
                order.order_id,
                kind,
                start_lm,
                tuple(
                    str(node)
                    for node in intent.get("trajectory_route_nodes", ())
                ),
                raw_request.regions,
                raw_request.direction,
                raw_request.staging_lm,
                raw_request.exit_lm,
            )
        ):
            return False
        if kind == "rolling":
            return bool(
                robot.active_order_id == order.order_id
                and robot.trajectory
                and str(robot.route_chunk_goal_lm or "") == start_lm
            )
        owner = str(order.vehicle or order.assigned_robot or "")
        return bool(
            owner == robot.name
            and not robot.active_order_id
            and not robot.trajectory
            and order.status in {"QUEUED", "PLANNING"}
            and self._safe_replan_start_lm(robot) == start_lm
        )

    def _controlled_corridor_prefetch_gate(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        *,
        prediction_offset: float,
        now: float,
    ) -> dict[str, Any] | None:
        """Return a scheduled start delay, or an unready authored intent."""
        scheduler = self._controlled_corridor_scheduler
        if scheduler is None:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        schedule = self._controlled_corridor_schedule
        existing_intent = (
            self._controlled_corridor_prefetch_intents.get(robot.name)
        )
        existing_request = (
            existing_intent.get("request")
            if isinstance(existing_intent, dict)
            else None
        )
        current_slot = (
            schedule.slot_for(robot.name)
            if schedule is not None
            else None
        )
        slot_belongs_to_intent = bool(
            isinstance(existing_intent, dict)
            and isinstance(existing_request, CorridorRequest)
            and isinstance(current_slot, CorridorSlot)
            and existing_intent.get("last_schedule_epoch")
            == schedule.epoch
            and current_slot.regions == existing_request.regions
            and current_slot.direction == existing_request.direction
            and current_slot.staging_lm == existing_request.staging_lm
            and current_slot.exit_lm == existing_request.exit_lm
        )
        live_entry = self._next_controlled_corridor_entry(robot)
        live_regions = self._controlled_corridor_entry_regions(
            live_entry
        )
        if (
            live_regions
            and set(live_regions).issubset(
                scheduler.controlled_regions
            )
        ):
            # The calendar currently has one transaction id per physical
            # robot.  Its immediate committed/approaching passage must remain
            # authoritative; a later rolling chunk cannot overwrite that
            # slot.  Plan the continuation against normal SIPP reservations
            # now. Once the current passage exits, the appended trajectory
            # becomes the live central request and receives its own gate
            # before reaching the next stop line.
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        if (
            isinstance(current_slot, CorridorSlot)
            and not slot_belongs_to_intent
        ):
            # A previous passage still occupies this robot's single calendar
            # transaction.  Do not bypass central admission: register the new
            # intent and let the next physical snapshot retire the old slot.
            # Returning ``None`` here used to send fresh orders straight into
            # MAPF without a corridor command.
            intent = self._controlled_corridor_prefetch_intent(
                order,
                robot,
                request,
                prediction_offset=prediction_offset,
                now=now,
            )
            return (
                {"ready": False, "intent": intent}
                if intent is not None
                else None
            )
        intent = self._controlled_corridor_prefetch_intent(
            order,
            robot,
            request,
            prediction_offset=prediction_offset,
            now=now,
        )
        if intent is None:
            return None
        schedule = self._controlled_corridor_schedule
        corridor_request = intent.get("request")
        if (
            schedule is None
            or not isinstance(corridor_request, CorridorRequest)
            or intent.get("last_schedule_epoch") != schedule.epoch
        ):
            return {"ready": False, "intent": intent}
        slot = schedule.slot_for(robot.name)
        if slot is None:
            # The finite calendar may not yet contain this passage, but free
            # space before its stop line is not part of the controlled
            # corridor. Let the normal MAPF stack move the robot there instead
            # of keeping a whole dispatch wave parked at its spawn positions.
            approach_gate = self._corridor_approach_gate(
                robot,
                request,
                intent,
            )
            return (
                approach_gate
                if approach_gate is not None
                else {"ready": False, "intent": intent}
            )
        if (
            slot.regions != corridor_request.regions
            or slot.direction != corridor_request.direction
            or slot.staging_lm != corridor_request.staging_lm
            or slot.exit_lm != corridor_request.exit_lm
            or int(slot.route_revision) != int(robot.route_revision)
        ):
            return {"ready": False, "intent": intent}
        entry = intent.get("entry")
        staging_clock = (
            float(entry.get("staging_clock", 0.0) or 0.0)
            if isinstance(entry, dict)
            else 0.0
        )
        nominal_staging_at = (
            now
            + max(0.0, float(prediction_offset))
            + max(0.0, staging_clock)
        )
        timing_tolerance = max(
            self._runtime_motion_step(),
            float(getattr(self.planner, "time_step_sec", 0.2) or 0.2),
        )
        if float(slot.entry_time) < nominal_staging_at - timing_tolerance:
            # Missing a tentative corridor slot must not freeze a robot which
            # is still outside the controlled passage.  The old slot is no
            # longer a valid entry command, but the ordinary graph prefix up
            # to a safe external holding LM is independent of that command.
            #
            # Refreshing the ETA first used to create a moving-target loop:
            # every scheduler tick shifted the slot forward while the robot
            # remained at its rolling boundary, so it could never reach the
            # stop line from which the slot could actually be honoured.
            approach_gate = self._corridor_approach_gate(
                robot,
                request,
                intent,
            )
            if approach_gate is not None:
                return approach_gate
            # The robot missed a tentative command while its previous chunk
            # was held.  Refresh the intent ETA and let the next runtime tick
            # place it again; never silently enter on an expired green light.
            intent["request"] = replace(
                corridor_request,
                earliest_entry=nominal_staging_at,
            )
            intent["handoff_at"] = (
                now + max(0.0, float(prediction_offset))
            )
            intent["last_schedule_epoch"] = None
            return {"ready": False, "intent": intent}
        planning_start_at = (
            now + max(0.0, float(prediction_offset))
        )
        rolling_horizon = self._rolling_horizon()
        if (
            rolling_horizon > 0.0
            and float(slot.exit_time)
            > planning_start_at + rolling_horizon + timing_tolerance
        ):
            # A rolling command is atomic only through its committed endpoint.
            # Asking SIPP to honour a corridor slot whose safe exit lies beyond
            # that endpoint creates an apparently valid long plan which
            # ``_rolling_result`` must trim before the exit.  Commit validation
            # then rejects the prefix and the same robot is planned forever.
            #
            # Use the free approach capacity instead: commit an ordinary prefix
            # to the external stop line and request a fresh passage there.  A
            # robot already at the stop line must remain in the calendar; a
            # zero-length "approach" would only turn the same red light into a
            # busy MAPF loop.
            approach_gate = self._corridor_approach_gate(
                robot,
                request,
                intent,
            )
            if approach_gate is not None:
                return approach_gate
            if str(request.get("startLm") or "") != (
                corridor_request.staging_lm
            ):
                return {"ready": False, "intent": intent}
            # A no-wait passage is an indivisible command. Once the robot is
            # already at its external stop line, let this gated command extend
            # through the safe exit even when that is longer than the ordinary
            # rolling horizon. Otherwise a long but valid authored corridor can
            # never be entered.
        departure_not_before = max(
            0.0,
            float(slot.entry_time)
            - planning_start_at,
        )
        return {
            "ready": True,
            "intent": intent,
            "slot": slot,
            # The red light belongs to the external corridor stop line, not
            # necessarily to the beginning of this rolling chunk.  Holding a
            # robot at its route start wastes all free approach capacity and
            # was the main reason a distant entrant could freeze two nearby
            # robots.  SIPP now carries this absolute route-clock constraint
            # to the staging LM and lets the robot approach it normally.
            "departureNotBefore": {
                "node": corridor_request.staging_lm,
                "timeSec": departure_not_before,
            },
            # The backed-off stop line is the last legal waiting point. SIPP
            # may rotate while traversing the passage, but any traffic delay
            # after this LM must be moved back to the stop line.
            "noWaitNodes": list(
                entry.get("no_wait_lms", ())
                if isinstance(entry, dict)
                else ()
            ),
        }

    def _corridor_approach_gate(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        intent: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Release a corridor-free prefix while retaining the stable route."""
        if not self._prepare_corridor_approach_request(
            request,
            intent,
            robot=robot,
        ):
            return None
        corridor_request = intent.get("request")
        return {
            "ready": True,
            "approachOnly": True,
            "holdingLm": str(request.get("goalLm") or ""),
            "stagingLm": (
                corridor_request.staging_lm
                if isinstance(corridor_request, CorridorRequest)
                else str(request.get("goalLm") or "")
            ),
        }

    def _prepare_corridor_approach_request(
        self,
        request: dict[str, Any],
        intent: dict[str, Any],
        *,
        robot: FleetRobot | None = None,
    ) -> bool:
        """Trim one request to its assigned safe approach-queue LM.

        The stable order route is deliberately not changed.  Only this temporal
        chunk ends at one graph-safe holding point. The first robot waits at
        the corridor stop line, the next robot waits at the preceding free LM,
        and so on. This prevents many independently planned chunks from
        converging on the same red-light vertex and blocking the corridor exit.
        """
        corridor_request = intent.get("request")
        entry = intent.get("entry")
        if (
            not isinstance(corridor_request, CorridorRequest)
            or not isinstance(entry, dict)
        ):
            return False
        start_lm = str(request.get("startLm") or "")
        staging_lm = corridor_request.staging_lm
        if not start_lm:
            return False
        route_nodes = [
            str(node)
            for node in request.get("routeNodes", ())
            if str(node) in self.landmarks
        ]
        if len(route_nodes) < 2 or route_nodes[0] != start_lm:
            return False

        # The same LM can occur more than once on a constrained recovery route.
        # Select the occurrence belonging to the first scheduled passage: the
        # last staging occurrence no later than its entry portal.
        portal_lm = str(entry.get("src") or "")
        portal_indices = [
            index
            for index, node in enumerate(route_nodes)
            if node == portal_lm
        ]
        portal_index = portal_indices[0] if portal_indices else len(route_nodes)
        staging_indices = [
            index
            for index, node in enumerate(route_nodes[: portal_index + 1])
            if node == staging_lm
        ]
        if not staging_indices:
            return False
        staging_index = staging_indices[-1]
        holding_lm = staging_lm
        if robot is not None:
            holding_lm = self._controlled_corridor_approach_holding_lm(
                robot,
                route_nodes=route_nodes,
                staging_index=staging_index,
                staging_lm=staging_lm,
                intent=intent,
            )
        if not holding_lm or holding_lm == start_lm:
            return False
        holding_indices = [
            index
            for index, node in enumerate(route_nodes[: staging_index + 1])
            if node == holding_lm
        ]
        if not holding_indices:
            return False
        holding_index = holding_indices[-1]
        if holding_index <= 0:
            return False
        request["goalLm"] = holding_lm
        request["routeNodes"] = route_nodes[: holding_index + 1]
        request.pop("departureNotBefore", None)
        return True

    def _controlled_corridor_approach_holding_lm(
        self,
        robot: FleetRobot,
        *,
        route_nodes: list[str],
        staging_index: int,
        staging_lm: str,
        intent: dict[str, Any],
    ) -> str:
        """Reserve the closest unclaimed safe LM in one portal queue.

        This queue is intentionally graph-based and map-independent. Only
        corridors explicitly authored in the editor call it; ordinary open
        space continues to use congestion A* and Rolling SIPP unchanged.
        """
        for robot_name, assignment in list(
            self._controlled_corridor_approach_holds.items()
        ):
            owner = self.robots.get(robot_name)
            live_intent = self._controlled_corridor_prefetch_intents.get(
                robot_name
            )
            assignment_order_id = str(
                assignment.get("order_id") or ""
            )
            assignment_signature = assignment.get("intent_signature")
            intent_still_current = bool(
                isinstance(live_intent, dict)
                and live_intent.get("signature") == assignment_signature
                and str(live_intent.get("order_id") or "")
                == assignment_order_id
            )
            route_still_executing = bool(
                assignment_order_id
                and owner is not None
                and owner.active_order_id == assignment_order_id
            )
            if (
                owner is None
                or int(assignment.get("route_revision", -1))
                != int(owner.route_revision)
                or not (
                    intent_still_current
                    or route_still_executing
                )
            ):
                self._controlled_corridor_approach_holds.pop(
                    robot_name,
                    None,
                )

        reserved_lms = {
            str(assignment.get("lm") or "")
            for robot_name, assignment
            in self._controlled_corridor_approach_holds.items()
            if robot_name != robot.name
            and str(assignment.get("staging_lm") or "") == staging_lm
        }
        # A robot with an already committed live corridor route may wait at
        # the same external stop line even though it no longer owns an
        # approach-only assignment. Keep that cell for the leader until its
        # live passage advances.
        for robot_name, passage in self._controlled_corridor_passages.items():
            if robot_name == robot.name:
                continue
            if (
                str(passage.get("staging_lm") or "") == staging_lm
                and bool(passage.get("committed"))
                and not bool(passage.get("entered"))
            ):
                reserved_lms.add(staging_lm)

        graph = self._controlled_corridor_graph
        candidates: list[str] = []
        seen: set[str] = set()
        for node in reversed(route_nodes[: staging_index + 1]):
            if node in seen:
                continue
            seen.add(node)
            vertex = graph.vertices.get(node) if graph is not None else None
            if (
                vertex is not None
                and vertex.can_wait
                and not vertex.controlled_region_ids
            ):
                candidates.append(node)
        if not candidates:
            candidates = [staging_lm]

        holding_lm = next(
            (
                candidate
                for candidate in candidates
                if candidate not in reserved_lms
            ),
            str(route_nodes[0] if route_nodes else ""),
        )
        self._controlled_corridor_approach_holds[robot.name] = {
            "lm": holding_lm,
            "staging_lm": staging_lm,
            "route_revision": int(robot.route_revision),
            "order_id": str(
                intent.get("order_id")
                or robot.active_order_id
                or ""
            ),
            "intent_signature": intent.get("signature"),
            "assigned_at": self._now(),
        }
        return holding_lm

    def _controlled_corridor_prefetch_plan_is_current(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        plan: dict[str, Any],
        gate: dict[str, Any],
        *,
        now: float,
    ) -> tuple[bool, str]:
        """Revalidate one prefetched passage against the live calendar.

        MAPF runs outside the runtime tick.  While it is computing, a physical
        occupant may delay or replace the tentative corridor slot.  A route is
        appendable only when its actual first authored passage still fits the
        current immutable slot.  This is the commit-side half of the central
        traffic contract; a rejected result is simply rescheduled and is not
        counted as a planner failure.
        """
        if self._controlled_corridor_scheduler is None:
            return True, ""
        intent = gate.get("intent")
        signature = gate.get("signature")
        scheduled_slot = gate.get("slot")
        current_intent = self._controlled_corridor_prefetch_intents.get(
            robot.name
        )
        order = (
            self.orders.get(str(intent.get("order_id") or ""))
            if isinstance(intent, dict)
            else None
        )
        if (
            not isinstance(intent, dict)
            or not isinstance(signature, tuple)
            or not isinstance(scheduled_slot, CorridorSlot)
            or current_intent is not intent
            or intent.get("signature") != signature
            or order is None
            or not self._controlled_corridor_intent_is_current(
                robot,
                order,
                intent,
            )
        ):
            return False, "corridor intent changed while planning"

        schedule = self._controlled_corridor_schedule
        current_slot = (
            schedule.slot_for(robot.name)
            if schedule is not None
            else None
        )
        corridor_request = intent.get("request")
        if (
            not isinstance(current_slot, CorridorSlot)
            or not isinstance(corridor_request, CorridorRequest)
            or current_slot.regions != corridor_request.regions
            or current_slot.direction != corridor_request.direction
            or current_slot.staging_lm != corridor_request.staging_lm
            or current_slot.exit_lm != corridor_request.exit_lm
            or int(current_slot.route_revision) != int(robot.route_revision)
        ):
            return False, "corridor slot is no longer available"

        trajectory = [
            dict(sample)
            for sample in plan.get("trajectory", ())
            if isinstance(sample, dict)
        ]
        if len(trajectory) < 2:
            return False, "corridor plan has no executable trajectory"
        start_lm = str(request.get("startLm") or "")
        start_pose = request.get("startPose")
        pose = (
            {
                "x": float(start_pose.get("x", 0.0) or 0.0),
                "y": float(start_pose.get("y", 0.0) or 0.0),
                "yaw": float(start_pose.get("yaw", 0.0) or 0.0),
            }
            if isinstance(start_pose, dict)
            else self._pose_at_landmark(start_lm)
        )
        synthetic = FleetRobot(
            name=robot.name,
            current_lm=start_lm,
            target_lm=str(plan.get("goalLm") or ""),
            status="MOVING",
            active_order_id=robot.active_order_id,
            pose=pose,
            trajectory=trajectory,
            route_clock=0.0,
            route_revision=int(robot.route_revision),
        )
        entry = self._next_controlled_corridor_entry(
            synthetic,
            # A central gate can intentionally add a long wait at the stop
            # line. Commit validation must inspect the complete returned
            # command, not apply the discovery lookahead a second time and
            # mistake a delayed passage for a changed route.
            lookahead_sec=float("inf"),
        )
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            entry is None
            or regions != corridor_request.regions
            or self._controlled_corridor_flow_direction(entry)
            != corridor_request.direction
            or str(entry.get("holding_lm") or entry.get("src") or "")
            != corridor_request.staging_lm
            or str(entry.get("exit_lm") or "") != corridor_request.exit_lm
        ):
            return False, "MAPF result changed the scheduled corridor passage"
        if bool(entry.get("has_wait_after_staging")):
            return False, "MAPF result waits after corridor commit point"

        current_end = (
            float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            if robot.trajectory
            else float(robot.route_clock)
        )
        handoff_delay = max(
            0.0,
            current_end - float(robot.route_clock),
        )
        staging_clock = max(
            0.0,
            float(entry.get("staging_clock", 0.0) or 0.0),
        )
        actual_staging_at = now + handoff_delay + staging_clock
        plan_windows = tuple(
            window
            for window in entry.get("resource_windows", ())
            if isinstance(window, CorridorResourceWindow)
        )
        if {
            window.region_id for window in plan_windows
        } != set(current_slot.regions):
            return False, "corridor resource set changed while planning"
        exit_clock = max(
            staging_clock + self._runtime_motion_step(),
            float(
                entry.get("exit_clock", staging_clock)
                or staging_clock
            ),
        )
        gate["actual_slot"] = CorridorSlot(
            robot_id=robot.name,
            regions=regions,
            direction=corridor_request.direction,
            entry_time=actual_staging_at,
            exit_time=actual_staging_at + exit_clock - staging_clock,
            staging_lm=corridor_request.staging_lm,
            exit_lm=corridor_request.exit_lm,
            route_revision=int(robot.route_revision),
            state=CorridorSlotState.COMMITTED,
            resource_windows=plan_windows,
            past_commit_point=False,
            physically_observed=False,
        )
        return True, ""

    def _commit_controlled_corridor_prefetch_slot(
        self,
        robot: FleetRobot,
        gate: dict[str, Any],
    ) -> bool:
        """Atomically retain a validated slot across route-revision handoff."""
        scheduler = self._controlled_corridor_scheduler
        schedule = self._controlled_corridor_schedule
        intent = gate.get("intent")
        if (
            scheduler is None
            or schedule is None
            or not isinstance(intent, dict)
            or self._controlled_corridor_prefetch_intents.get(robot.name)
            is not intent
        ):
            return False
        expected_slot = gate.get("slot")
        if not isinstance(expected_slot, CorridorSlot):
            return False
        actual_slot = gate.get("actual_slot")
        if not isinstance(actual_slot, CorridorSlot):
            actual_slot = expected_slot
        committed = scheduler.commit_slot(
            robot.name,
            # Commit the exact proposal captured before MAPF started.  Passing
            # the current slot here would let a stale worker validate a
            # different calendar decision after its original tentative slot
            # was moved or displaced.
            expected=expected_slot,
            actual=actual_slot,
        )
        if committed is None:
            return False
        self._controlled_corridor_schedule = committed
        intent["last_schedule_epoch"] = committed.epoch
        return True

    def _pin_controlled_corridor_gates(
        self,
        gates: dict[str, dict[str, Any]],
    ) -> bool:
        """Lease every captured tentative slot to one in-flight MAPF job."""
        scheduler = self._controlled_corridor_scheduler
        if not gates:
            return True
        if scheduler is None:
            return False
        pinned: list[tuple[str, CorridorSlot]] = []
        for robot_name, gate in gates.items():
            slot = gate.get("slot") if isinstance(gate, dict) else None
            if not isinstance(slot, CorridorSlot) or not scheduler.pin_slot(
                robot_name,
                expected=slot,
            ):
                for pinned_name, pinned_slot in pinned:
                    scheduler.release_slot_pin(
                        pinned_name,
                        expected=pinned_slot,
                    )
                return False
            pinned.append((robot_name, slot))
        return True

    def _release_controlled_corridor_gate_pins(
        self,
        gates: dict[str, dict[str, Any]] | None,
    ) -> None:
        """End worker leases without releasing a newer replacement pin."""
        scheduler = self._controlled_corridor_scheduler
        if scheduler is None or not isinstance(gates, dict):
            return
        for robot_name, gate in gates.items():
            slot = gate.get("slot") if isinstance(gate, dict) else None
            if isinstance(slot, CorridorSlot):
                scheduler.release_slot_pin(
                    robot_name,
                    expected=slot,
                )

    def _handle_controlled_corridor_gate_rejection(
        self,
        robot_name: str,
        gate: dict[str, Any],
        reason: str,
    ) -> None:
        """Keep queue age across temporary calendar displacement."""
        intent = gate.get("intent")
        current = self._controlled_corridor_prefetch_intents.get(robot_name)
        if current is not intent or not isinstance(intent, dict):
            return
        if reason in {
            "corridor slot is no longer available",
            "corridor slot changed before command commit",
        }:
            # The route proposal is still valid; only its tentative calendar
            # position changed.  Preserve ``registered_at`` so repeated
            # displacement cannot starve this robot, and ask the next scheduler
            # snapshot to assign a fresh slot.  When SIPP reached the same
            # passage later than the nominal calendar proposal, keep that
            # validated ETA/resource template as the new lower bound. Without
            # this handoff the scheduler offered the same too-early slot and
            # the worker reproduced the same committed-slot conflict forever.
            corridor_request = intent.get("request")
            actual_slot = gate.get("actual_slot")
            if (
                isinstance(corridor_request, CorridorRequest)
                and isinstance(actual_slot, CorridorSlot)
                and actual_slot.robot_id == robot_name
                and actual_slot.regions == corridor_request.regions
                and actual_slot.direction == corridor_request.direction
                and actual_slot.staging_lm == corridor_request.staging_lm
                and actual_slot.exit_lm == corridor_request.exit_lm
            ):
                intent["request"] = replace(
                    corridor_request,
                    earliest_entry=max(
                        corridor_request.earliest_entry,
                        actual_slot.entry_time,
                    ),
                    duration_sec=max(
                        self._runtime_motion_step(),
                        actual_slot.duration_sec,
                    ),
                    resource_windows=actual_slot.resource_windows,
                )
            intent["last_schedule_epoch"] = None
            return
        self._controlled_corridor_prefetch_intents.pop(robot_name, None)

    @staticmethod
    def _controlled_corridor_entry_regions(
        entry: dict[str, Any] | None,
    ) -> tuple[str, ...]:
        if not isinstance(entry, dict):
            return ()
        raw = entry.get("regions")
        if isinstance(raw, (list, tuple)):
            regions = tuple(str(item) for item in raw if str(item))
            if regions:
                return tuple(dict.fromkeys(regions))
        region = str(entry.get("region") or "")
        return (region,) if region else ()

    def _controlled_corridor_has_grant(
        self,
        robot_name: str,
        regions: tuple[str, ...] | list[str],
    ) -> bool:
        """Return whether the central calendar admits this corridor passage."""
        required = {str(region) for region in regions if str(region)}
        if not required:
            return True
        if self._controlled_corridor_scheduler is None:
            return False

        robot = self.robots.get(robot_name)
        if (
            robot is not None
            and required.issubset(
                self._controlled_regions_for_robot(robot)
            )
        ):
            return True
        schedule = self._controlled_corridor_schedule
        slot = (
            schedule.slot_for(robot_name)
            if schedule is not None
            else None
        )
        if (
            slot is None
            or slot.state is not CorridorSlotState.COMMITTED
            or not required.issubset(slot.regions)
            or robot is None
            or int(slot.route_revision) != int(robot.route_revision)
        ):
            return False
        upcoming = self._next_controlled_corridor_entry(robot)
        if (
            upcoming is not None
            and self._controlled_corridor_entry_regions(upcoming)
            and slot.direction
            != self._controlled_corridor_flow_direction(upcoming)
        ):
            return False
        immediate_window = max(
            self._runtime_motion_step(),
            self._continuous_collision_step(),
        )
        return bool(
            slot.entry_time
            <= self._controlled_corridor_tick_now
            + immediate_window
            + 0.000001
        )

    def _controlled_corridor_entry_lookahead(self) -> float:
        return max(
            1.0,
            self._controlled_corridor_param(
                "controlled_corridor_schedule_horizon_sec",
                self._controlled_corridor_param(
                    "controlled_corridor_entry_lookahead_sec",
                    max(30.0, self._rolling_horizon()),
                ),
            ),
        )

    def _retained_route_is_superseded(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether a valid transaction has retired this route suffix.

        The trajectory remains attached while its replacement is planned so
        collision prediction and browser rendering never see an unsafe empty
        frame. It must not, however, renew an external corridor admission after
        recovery has explicitly declared that spatial suffix unusable.
        """
        state = self._runtime_replans.get(robot.name)
        if not isinstance(state, dict) or not bool(
            state.get("retained_route_superseded")
        ):
            return False
        order = self.orders.get(str(state.get("order_id") or ""))
        return bool(
            order is not None
            and robot.active_order_id == order.order_id
            and int(state.get("route_revision", -1))
            == int(robot.route_revision)
            and abs(
                float(state.get("route_clock", 0.0) or 0.0)
                - float(robot.route_clock)
            ) <= 0.000001
            and str(state.get("start_lm") or "")
            == self._safe_replan_start_lm(robot)
        )

    def _prepare_controlled_corridor_admissions(self, now: float) -> None:
        """Refresh corridor authority, or stay inert when no zones exist."""
        scheduler = self._controlled_corridor_scheduler
        if scheduler is not None:
            self._prepare_central_controlled_corridor_schedule(now)
            return

        # Corridor dispatch is opt-in through explicitly authored controlled
        # regions. With no scheduler there is no implicit lease/FIFO fallback:
        # ordinary rolling SIPP/CBS traffic control remains the sole authority.
        self._controlled_corridor_tick_now = now
        self._controlled_corridor_schedule = None
        self._controlled_corridor_wait_since.clear()
        self._controlled_corridor_leases.clear()
        self._controlled_corridor_passages.clear()
        self._controlled_corridor_winners.clear()
        self._controlled_corridor_occupancy.clear()
        self._controlled_corridor_queues.clear()
        self._controlled_corridor_blockers.clear()

    def _controlled_corridor_queue_predecessor(
        self,
        robot: FleetRobot,
        entry: dict[str, Any],
        regions: tuple[str, ...],
        direction: str,
    ) -> str:
        """Return a same-direction robot physically ahead at this portal."""
        dependency_name = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(robot.last_reason)
        )
        if not dependency_name or dependency_name == robot.name:
            return ""
        dependency = self.robots.get(dependency_name)
        if (
            dependency is None
            or not dependency.trajectory
            or dependency.pose is None
            or robot.pose is None
        ):
            return ""
        dependency_entry = self._next_controlled_corridor_entry(
            dependency
        )
        if dependency_entry is None:
            return ""
        dependency_regions = set(
            self._controlled_corridor_entry_regions(dependency_entry)
        )
        dependency_direction = self._controlled_corridor_flow_direction(
            dependency_entry
        )
        portal_lm = str(entry.get("src") or "")
        if (
            not set(regions).intersection(dependency_regions)
            or dependency_direction != direction
            or str(dependency_entry.get("src") or "") != portal_lm
        ):
            return ""
        portal = self.landmarks.get(portal_lm)
        if portal is None:
            return ""
        robot_distance = math.hypot(
            float(robot.pose.get("x", 0.0)) - float(portal.x),
            float(robot.pose.get("y", 0.0)) - float(portal.y),
        )
        dependency_distance = math.hypot(
            float(dependency.pose.get("x", 0.0)) - float(portal.x),
            float(dependency.pose.get("y", 0.0)) - float(portal.y),
        )
        return (
            dependency.name
            if dependency_distance + 0.001 < robot_distance
            else ""
        )

    def _controlled_corridor_flow_direction(
        self,
        entry: dict[str, Any],
    ) -> str:
        """Return a stable flow phase shared by equal travel directions."""
        resource_windows = entry.get("resource_windows")
        if isinstance(resource_windows, (list, tuple)):
            first_window = next(
                (
                    window
                    for window in resource_windows
                    if isinstance(window, CorridorResourceWindow)
                ),
                None,
            )
            if first_window is not None:
                return first_window.direction
        src_name = str(entry.get("src") or "")
        dst_name = str(entry.get("exit_lm") or "")
        fallback = str(
            entry.get("direction")
            or f"{src_name}->{dst_name}"
        )
        return self._controlled_corridor_lane_direction(
            src_name,
            dst_name,
            fallback=fallback,
        )

    def _controlled_corridor_lane_direction(
        self,
        src_name: str,
        dst_name: str,
        *,
        fallback: str = "",
    ) -> str:
        """Quantize one local corridor resource traversal direction."""
        src = self.landmarks.get(str(src_name or ""))
        dst = self.landmarks.get(str(dst_name or ""))
        if src is None or dst is None:
            return str(fallback or f"{src_name}->{dst_name}")
        dx = float(dst.x) - float(src.x)
        dy = float(dst.y) - float(src.y)
        if abs(dx) <= 0.000001 and abs(dy) <= 0.000001:
            return str(fallback or f"{src.name}->{dst.name}")
        if abs(dx) >= abs(dy) * 2.0:
            return "flow:east" if dx > 0.0 else "flow:west"
        if abs(dy) >= abs(dx) * 2.0:
            return "flow:south" if dy > 0.0 else "flow:north"
        horizontal = "east" if dx > 0.0 else "west"
        vertical = "south" if dy > 0.0 else "north"
        return f"flow:{vertical}-{horizontal}"

    def _central_corridor_manages_wait(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether the central calendar already orders this wait.

        The central calendar owns admission at an *external* stop line.  It
        also owns a physical queue outside that line when the named blocker
        has an earlier slot on the same resource.  Treating that expected
        queue as a generic deadlock made followers retreat and globally
        replan after only a few seconds, even though their leader was already
        clearing the corridor.  Unexpected conflicts, bodies after entry and
        blockers absent from the calendar still go to the local wait-for
        resolver.
        """
        scheduler = self._controlled_corridor_scheduler
        schedule = self._controlled_corridor_schedule
        if scheduler is None or schedule is None or not robot.trajectory:
            return False
        reason = str(robot.last_reason or "")
        admission_wait = bool(
            reason.startswith("corridor admission wait at ")
            or reason.startswith(
                "corridor admission timeout: corridor admission wait at "
            )
        )
        if not admission_wait and not self._is_robot_conflict(reason):
            return False
        route_regions = (
            self._controlled_regions_for_robot(robot)
            & set(scheduler.controlled_regions)
        )
        if route_regions:
            # Admission must never suppress recovery for a body which already
            # crossed into a controlled resource.  A stale red-light message
            # is cleared by the next motion tick; until then the physical
            # wait-for graph is the safer authority.
            return False
        entry = self._next_controlled_corridor_entry(robot)
        if entry is None:
            return False
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            not regions
            or not set(regions).issubset(scheduler.controlled_regions)
        ):
            return False
        staging_clock = float(
            entry.get("staging_clock", entry.get("entry_clock", 0.0))
            or 0.0
        )
        if (
            float(robot.route_clock)
            + self._runtime_motion_step()
            < staging_clock
        ):
            return False
        decision = schedule.decisions.get(robot.name)
        blocker_name = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(reason)
        )
        deferred_queue_wait = bool(
            not admission_wait
            and decision is not None
            and decision.status is CorridorDecisionStatus.DEFERRED
            and self._controlled_corridor_queue_predecessor(
                robot,
                entry,
                regions,
                self._controlled_corridor_flow_direction(entry),
            )
            == blocker_name
        )
        if (
            (admission_wait or deferred_queue_wait)
            and decision is not None
            and decision.status
            in {
                CorridorDecisionStatus.GRANTED,
                CorridorDecisionStatus.DEFERRED,
            }
        ):
            blocker = self.robots.get(blocker_name)
            blocker_dependency = (
                (
                    str(blocker.wait_for_robot or "").strip()
                    or self._robot_name_from_conflict_reason(
                        blocker.last_reason
                    )
                )
                if blocker is not None
                else ""
            )
            if (
                blocker is not None
                and blocker.status == "WAITING"
                and blocker_dependency == robot.name
            ):
                # A red-light waiter and its named owner now wait for each
                # other.  That reciprocal dependency is a real cycle even
                # while both bodies are still outside the authored region;
                # hiding one edge as an "expected queue" turns it into an
                # acyclic chain and can grant the red-light waiter forever.
                # Expose both edges to deterministic local arbitration.
                return False
            if decision.slot is None:
                # A deferred request is still an intentional calendar
                # decision (predecessor, downstream box or horizon), not a
                # planner failure. It must remain at the authored stop line
                # without spawning a global detour every few seconds.
                return True
        slot = schedule.slot_for(robot.name)
        if not (
            slot is not None
            and int(slot.route_revision) == int(robot.route_revision)
            and set(regions).issubset(slot.regions)
            and slot.direction
            == self._controlled_corridor_flow_direction(entry)
        ):
            return False
        if admission_wait:
            return True

        blocker = self.robots.get(blocker_name)
        blocker_slot = (
            schedule.slot_for(blocker_name)
            if blocker is not None
            else None
        )
        if (
            blocker is None
            or blocker_slot is None
            or int(blocker_slot.route_revision)
            != int(blocker.route_revision)
        ):
            return False
        robot_windows = {
            window.region_id: (
                slot.entry_time + window.entry_offset_sec,
                slot.entry_time + window.exit_offset_sec,
                window.direction,
            )
            for window in slot.resource_windows
        }
        blocker_windows = {
            window.region_id: (
                blocker_slot.entry_time + window.entry_offset_sec,
                blocker_slot.entry_time + window.exit_offset_sec,
                window.direction,
            )
            for window in blocker_slot.resource_windows
        }
        for region_id in set(regions).intersection(blocker_windows):
            robot_entry, robot_exit, robot_direction = robot_windows[
                region_id
            ]
            blocker_entry, blocker_exit, blocker_direction = blocker_windows[
                region_id
            ]
            if robot_direction == blocker_direction:
                if (
                    blocker_entry <= robot_entry + 0.000001
                    and blocker_exit <= robot_exit + 0.000001
                ):
                    return True
            elif blocker_exit <= robot_entry + 0.000001:
                return True
        return False

    def _central_corridor_owner_is_clearing(
        self,
        robot: FleetRobot,
    ) -> bool:
        """Return whether this body must reach the external exit first.

        A committed owner that has entered an atomic passage may not obey a
        far-horizon collision forecast *beyond* the passage exit by stopping
        inside the narrow resource.  Runtime still checks every immediate
        20–50 ms motion step and the global swept-footprint invariant; this
        flag only suppresses the distant preflight freeze until the complete
        body clears the authored corridor.
        """
        passage = self._controlled_corridor_passages.get(robot.name)
        if not isinstance(passage, dict):
            return False
        try:
            passage_revision = int(
                passage.get("route_revision", -1)
            )
        except (TypeError, ValueError):
            return False
        if passage_revision != int(robot.route_revision):
            return False
        return bool(
            passage.get("entered")
            or passage.get("past_commit_point")
        )

    def _controlled_corridor_live_occupancy(
        self,
        robot: FleetRobot,
        *,
        physical_regions: set[str],
        previous_slot: CorridorSlot | None,
        entry: dict[str, Any],
        previous_passage: dict[str, Any],
        now: float,
    ) -> CorridorOccupancy:
        """Project one physical owner from immutable trajectory-clock data.

        Calendar slots are outputs of admission control. Their duration and
        offsets may move when a robot is delayed, so feeding those values back
        into trajectory time creates an accumulating positive feedback loop.
        ``route_resource_windows`` is the immutable template captured when the
        passage was first discovered. Every runtime tick rebases that template
        from the robot's current route clock.
        """
        motion_step = max(
            self._runtime_motion_step(),
            self._controlled_corridor_param(
                "controlled_corridor_occupancy_recheck_sec",
                0.1,
            ),
        )
        current_lm = self._traffic_lm_for_robot(robot)
        candidate_regions = tuple(dict.fromkeys(
            (
                *(
                    previous_slot.regions
                    if previous_slot is not None
                    else tuple(
                        str(item)
                        for item in previous_passage.get("regions", ())
                        if str(item)
                    )
                ),
                *sorted(physical_regions),
            )
        ))
        exit_lm = str(
            (previous_slot.exit_lm if previous_slot is not None else "")
            or entry.get("exit_lm")
            or previous_passage.get("exit_lm")
            or current_lm
        )
        staging_lm = str(
            (previous_slot.staging_lm if previous_slot is not None else "")
            or entry.get("holding_lm")
            or previous_passage.get("staging_lm")
            or current_lm
        )
        direction = str(
            (previous_slot.direction if previous_slot is not None else "")
            or (
                self._controlled_corridor_flow_direction(entry)
                if entry
                else ""
            )
            or previous_passage.get("direction")
            or f"occupied:{robot.name}"
        )
        route_staging_clock = float(
            entry.get(
                "staging_clock",
                previous_passage.get(
                    "staging_clock",
                    robot.route_clock,
                ),
            )
            or 0.0
        )
        route_templates = tuple(
            window
            for window in (
                entry.get("resource_windows", ())
                or previous_passage.get("route_resource_windows", ())
                or (
                    previous_slot.resource_windows
                    if (
                        previous_slot is not None
                        and not previous_slot.physically_observed
                    )
                    else ()
                )
            )
            if isinstance(window, CorridorResourceWindow)
        )
        templates_by_region = {
            window.region_id: window
            for window in route_templates
        }
        route_clock = float(robot.route_clock)
        fallback_remaining = max(
            motion_step,
            self._controlled_corridor_physical_exit_time(
                robot,
                exit_lm,
                now,
            )
            - now,
        )
        live_windows: list[CorridorResourceWindow] = []
        for region_id in candidate_regions:
            template = templates_by_region.get(region_id)
            if template is None:
                if (
                    region_id not in physical_regions
                    and previous_slot is None
                ):
                    continue
                entry_offset = 0.0
                exit_offset = fallback_remaining
                local_direction = direction
            else:
                route_entry = (
                    route_staging_clock + template.entry_offset_sec
                )
                route_exit = (
                    route_staging_clock + template.exit_offset_sec
                )
                if (
                    region_id not in physical_regions
                    and route_exit <= route_clock + 0.000001
                ):
                    # This resource is behind the complete footprint. Do not
                    # repaint a released corridor red while the owner advances
                    # through a later rectangle in the same atomic passage.
                    continue
                entry_offset = (
                    0.0
                    if region_id in physical_regions
                    else max(0.0, route_entry - route_clock)
                )
                exit_offset = max(
                    entry_offset + motion_step,
                    motion_step,
                    route_exit - route_clock,
                )
                local_direction = template.direction
            live_windows.append(
                CorridorResourceWindow(
                    region_id=region_id,
                    entry_offset_sec=entry_offset,
                    exit_offset_sec=exit_offset,
                    direction=local_direction,
                )
            )

        # ``physical_regions`` is non-empty for callers, but a malformed
        # legacy passage may have no template. Preserve physical truth with a
        # bounded recheck window instead of returning an invalid empty claim.
        covered = {window.region_id for window in live_windows}
        for region_id in sorted(physical_regions - covered):
            live_windows.append(
                CorridorResourceWindow(
                    region_id=region_id,
                    entry_offset_sec=0.0,
                    exit_offset_sec=fallback_remaining,
                    direction=direction,
                )
            )
        occupancy_windows = tuple(live_windows)
        regions = tuple(window.region_id for window in occupancy_windows)
        expected_exit = now + max(
            window.exit_offset_sec
            for window in occupancy_windows
        )
        return CorridorOccupancy(
            robot_id=robot.name,
            regions=regions,
            direction=direction,
            entered_at=now,
            expected_exit_time=expected_exit,
            exit_lm=exit_lm,
            route_revision=int(robot.route_revision),
            staging_lm=staging_lm,
            resource_windows=occupancy_windows,
        )

    def _prepare_central_controlled_corridor_schedule(
        self,
        now: float,
    ) -> None:
        """Build one fleet-wide calendar for explicitly controlled passages."""
        scheduler = self._controlled_corridor_scheduler
        graph = self._controlled_corridor_graph
        if scheduler is None or graph is None:
            return

        self._controlled_corridor_tick_now = now
        old_schedule = scheduler.current_schedule
        physical_by_robot: dict[str, set[str]] = {}
        occupancy_by_region: dict[str, set[str]] = {}
        for robot in self._runtime_robots():
            route_regions = (
                self._controlled_regions_for_robot(robot)
                & set(scheduler.controlled_regions)
            )
            footprint_regions = (
                self._controlled_regions_intersecting_footprint(robot)
                & set(scheduler.controlled_regions)
            )
            previous_slot = (
                old_schedule.slot_for(robot.name)
                if old_schedule is not None
                else None
            )
            regions = set(route_regions)
            if route_regions:
                # Once the centre is on a controlled graph resource, protect
                # every authored rectangle touched by the complete body.
                regions.update(footprint_regions)
            elif (
                previous_slot is not None
                and previous_slot.past_commit_point
            ):
                # After the centre reaches the external exit LM, retain only
                # the physical tail which still intersects the just-completed
                # passage.  Do not turn an unrelated neighbouring rectangle
                # into a new occupancy claim.
                regions.update(
                    footprint_regions.intersection(previous_slot.regions)
                )
            # A legal external holding LM is deliberately close to the
            # corridor mouth.  Its rectangular footprint may overlap the
            # editor rectangle by a few centimetres, but that is not proof of
            # entry: treating it as occupancy creates an ``owner <self>``
            # slot with no exit and freezes the red light forever.  The graph
            # crossing (or an already-entered previous slot) is the commit
            # authority.
            if not regions:
                continue
            physical_by_robot[robot.name] = regions
            for region_id in regions:
                occupancy_by_region.setdefault(region_id, set()).add(
                    robot.name
                )
        self._controlled_corridor_occupancy = {
            region_id: sorted(robot_names)
            for region_id, robot_names in occupancy_by_region.items()
        }

        requests: list[CorridorRequest] = []
        entries_by_robot: dict[str, dict[str, Any]] = {}
        scheduled_intent_names: set[str] = set()
        active_wait_keys: set[tuple[str, str, int, str]] = set()
        downstream_blockers: dict[str, str] = {}
        starvation = max(
            1.0,
            self._controlled_corridor_param(
                "controlled_corridor_starvation_sec",
                8.0,
            ),
        )
        for robot in self._runtime_robots():
            if (
                robot.status not in {"MOVING", "WAITING"}
                or not robot.trajectory
                or (
                    robot.name not in physical_by_robot
                    and self._retained_route_is_superseded(robot)
                )
            ):
                continue
            entry = self._next_controlled_corridor_entry(robot)
            if entry is None:
                continue
            regions = self._controlled_corridor_entry_regions(entry)
            if (
                not regions
                or not set(regions).issubset(
                    scheduler.controlled_regions
                )
            ):
                continue
            exit_lm = str(entry.get("exit_lm") or "")
            if exit_lm not in self.landmarks:
                continue
            direction = self._controlled_corridor_flow_direction(entry)
            passage_id = str(entry.get("passage") or "|".join(regions))
            wait_key = (
                passage_id,
                direction,
                int(robot.route_revision),
                robot.name,
            )
            active_wait_keys.add(wait_key)
            wait_since = self._controlled_corridor_wait_since.setdefault(
                wait_key,
                now,
            )
            entry_clock = float(
                entry.get("entry_clock", robot.route_clock)
                or robot.route_clock
            )
            raw_staging_clock = entry.get("staging_clock")
            staging_clock = min(
                entry_clock,
                float(
                    entry_clock
                    if raw_staging_clock is None
                    else raw_staging_clock
                ),
            )
            exit_clock = max(
                entry_clock + self._runtime_motion_step(),
                float(entry.get("exit_clock", entry_clock) or entry_clock),
            )
            earliest_entry = now + max(
                0.0,
                staging_clock - float(robot.route_clock),
            )
            blocker = self._controlled_corridor_downstream_blocker(
                robot,
                exit_lm,
                exit_clock,
            )
            if blocker:
                downstream_blockers[robot.name] = blocker
            order = self._active_order_for_robot(robot)
            predecessor_robot_id = (
                self._controlled_corridor_queue_predecessor(
                    robot,
                    entry,
                    regions,
                    direction,
                )
            )
            request = CorridorRequest(
                robot_id=robot.name,
                regions=regions,
                direction=direction,
                earliest_entry=earliest_entry,
                duration_sec=max(
                    self._runtime_motion_step(),
                    exit_clock - staging_clock,
                ),
                staging_lm=str(
                    entry.get("holding_lm")
                    or entry.get("src")
                    or ""
                ),
                exit_lm=exit_lm,
                route_revision=int(robot.route_revision),
                priority=float(order.priority if order is not None else 0),
                wait_age_sec=max(0.0, now - wait_since),
                deadline=wait_since + starvation,
                downstream_available=not bool(blocker),
                predecessor_robot_id=predecessor_robot_id or None,
                entered=bool(
                    physical_by_robot.get(robot.name, set())
                    .intersection(regions)
                ),
                past_commit_point=bool(
                    physical_by_robot.get(robot.name, set())
                    .intersection(regions)
                    or (
                        bool(entry.get("passed_staging"))
                        and not bool(
                            entry.get("has_wait_after_staging")
                        )
                    )
                ),
                resource_windows=tuple(
                    window
                    for window in entry.get("resource_windows", ())
                    if isinstance(window, CorridorResourceWindow)
                ),
            )
            requests.append(request)
            entries_by_robot[robot.name] = entry

        # Future rolling chunks are not present in ``robot.trajectory`` yet.
        # Admit their first authored passage from the registered nominal
        # timeline so SIPP receives a red-light time at the external staging
        # LM instead of discovering the conflict inside a no-wait chain.
        for robot_name, intent in list(
            self._controlled_corridor_prefetch_intents.items()
        ):
            robot = self.robots.get(robot_name)
            order = (
                self.orders.get(str(intent.get("order_id") or ""))
                if isinstance(intent, dict)
                else None
            )
            raw_request = (
                intent.get("request")
                if isinstance(intent, dict)
                else None
            )
            signature = (
                intent.get("signature")
                if isinstance(intent, dict)
                else None
            )
            if (
                robot is None
                or order is None
                or not isinstance(intent, dict)
                or not self._controlled_corridor_intent_is_current(
                    robot,
                    order,
                    intent,
                )
                or not isinstance(raw_request, CorridorRequest)
                or not isinstance(signature, tuple)
            ):
                self._controlled_corridor_prefetch_intents.pop(
                    robot_name,
                    None,
                )
                continue
            previous_slot = (
                old_schedule.slot_for(robot_name)
                if old_schedule is not None
                else None
            )
            if (
                robot_name in entries_by_robot
                or robot_name in physical_by_robot
                or (
                    previous_slot is not None
                    and previous_slot.state
                    is CorridorSlotState.COMMITTED
                    and previous_slot.exit_time > now
                )
            ):
                # One robot can own only its immediate passage. A future
                # continuation waits until the current passage has exited.
                intent["last_schedule_epoch"] = None
                continue
            entry = intent.get("entry")
            if not isinstance(entry, dict):
                self._controlled_corridor_prefetch_intents.pop(
                    robot_name,
                    None,
                )
                continue
            handoff_at = float(
                intent.get("handoff_at", now)
                or now
            )
            synthetic = FleetRobot(
                name=robot.name,
                current_lm=str(intent.get("start_lm") or ""),
                target_lm=raw_request.exit_lm,
                status="MOVING",
                active_order_id=order.order_id,
                pose=(
                    dict(intent["start_pose"])
                    if isinstance(intent.get("start_pose"), dict)
                    else self._pose_at_landmark(
                        str(intent.get("start_lm") or "")
                    )
                ),
                trajectory=[
                    dict(sample)
                    for sample in intent.get("trajectory", ())
                    if isinstance(sample, dict)
                ],
                route_clock=-max(0.0, handoff_at - now),
                route_revision=int(robot.route_revision),
            )
            blocker = self._controlled_corridor_downstream_blocker(
                synthetic,
                raw_request.exit_lm,
                float(
                    entry.get(
                        "exit_clock",
                        raw_request.duration_sec,
                    )
                    or raw_request.duration_sec
                ),
            )
            if blocker:
                downstream_blockers[robot_name] = blocker
            wait_since = float(
                intent.get("registered_at", now)
                or now
            )
            request = replace(
                raw_request,
                wait_age_sec=max(0.0, now - wait_since),
                deadline=wait_since + starvation,
                downstream_available=not bool(blocker),
            )
            intent["request"] = request
            passage_id = str(
                entry.get("passage")
                or "|".join(request.regions)
            )
            active_wait_keys.add(
                (
                    passage_id,
                    request.direction,
                    int(request.route_revision),
                    robot_name,
                )
            )
            requests.append(request)
            entries_by_robot[robot_name] = entry
            scheduled_intent_names.add(robot_name)

        for key in list(self._controlled_corridor_wait_since):
            if key not in active_wait_keys:
                self._controlled_corridor_wait_since.pop(key, None)

        # A physical owner can be close enough to its safe exit that
        # ``_next_controlled_corridor_entry`` correctly reports no *future*
        # passage. It still needs the exit pocket protected until its complete
        # body leaves. Recheck the retained physical passage here so local
        # arbitration knows that the external body must clear first.
        for robot_name in physical_by_robot:
            if robot_name in downstream_blockers:
                continue
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            previous_slot = (
                old_schedule.slot_for(robot_name)
                if old_schedule is not None
                else None
            )
            previous_passage = self._controlled_corridor_passages.get(
                robot_name,
                {},
            )
            exit_lm = str(
                (previous_slot.exit_lm if previous_slot is not None else "")
                or previous_passage.get("exit_lm")
                or ""
            )
            if exit_lm not in self.landmarks:
                continue
            exit_clock = float(robot.route_clock)
            for sample in robot.trajectory:
                sample_clock = float(sample.get("t", 0.0) or 0.0)
                if sample_clock + 0.000001 < robot.route_clock:
                    continue
                if str(sample.get("lm") or "") != exit_lm:
                    continue
                exit_clock = sample_clock
                break
            blocker = self._controlled_corridor_downstream_blocker(
                robot,
                exit_lm,
                exit_clock,
            )
            if blocker:
                downstream_blockers[robot_name] = blocker

        occupancies: list[CorridorOccupancy] = []
        for robot_name, physical_regions in physical_by_robot.items():
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            previous_slot = (
                old_schedule.slot_for(robot_name)
                if old_schedule is not None
                else None
            )
            entry = entries_by_robot.get(robot_name, {})
            old_passage = self._controlled_corridor_passages.get(
                robot_name,
                {},
            )
            occupancies.append(
                self._controlled_corridor_live_occupancy(
                    robot,
                    physical_regions=physical_regions,
                    previous_slot=previous_slot,
                    entry=entry,
                    previous_passage=old_passage,
                    now=now,
                )
            )

        schedule = scheduler.update(
            requests,
            now=now,
            occupancies=occupancies,
        )
        self._controlled_corridor_schedule = schedule
        for robot_name in scheduled_intent_names:
            intent = self._controlled_corridor_prefetch_intents.get(
                robot_name
            )
            if isinstance(intent, dict):
                intent["last_schedule_epoch"] = schedule.epoch
        self._controlled_corridor_blockers = downstream_blockers
        previous_committed = {
            slot.robot_id
            for slot in (old_schedule.slots if old_schedule is not None else ())
            if slot.state is CorridorSlotState.COMMITTED
        }
        newly_committed = {
            slot.robot_id
            for slot in schedule.slots
            if slot.state is CorridorSlotState.COMMITTED
        } - previous_committed
        self.traffic_metrics["corridorAdmissionsGranted"] += len(
            newly_committed
        )

        passages: dict[str, dict[str, Any]] = {}
        leases: dict[str, tuple[str, float]] = {}
        winners: dict[str, str] = {}
        queues: dict[str, list[tuple[float, str]]] = {}
        immediate_window = max(
            self._runtime_motion_step(),
            self._continuous_collision_step(),
        )
        for slot in schedule.slots:
            entry = entries_by_robot.get(slot.robot_id, {})
            previous_passage = self._controlled_corridor_passages.get(
                slot.robot_id,
                {},
            )
            entered = slot.robot_id in physical_by_robot
            entry_route_windows = tuple(
                window
                for window in entry.get("resource_windows", ())
                if isinstance(window, CorridorResourceWindow)
            )
            route_resource_windows = (
                entry_route_windows
                or tuple(
                    window
                    for window in previous_passage.get(
                        "route_resource_windows",
                        (),
                    )
                    if isinstance(window, CorridorResourceWindow)
                )
                or (
                    slot.resource_windows
                    if not slot.physically_observed
                    else ()
                )
            )
            staging_clock = float(
                entry.get(
                    "staging_clock",
                    previous_passage.get(
                        "staging_clock",
                        0.0,
                    ),
                )
                or 0.0
            )
            resource_intervals = tuple(
                {
                    "region": window.region_id,
                    "direction": window.direction,
                    "entry_time": (
                        slot.entry_time + window.entry_offset_sec
                    ),
                    "exit_time": (
                        slot.entry_time + window.exit_offset_sec
                    ),
                }
                for window in slot.resource_windows
            )
            active = bool(
                entered
                or (
                    slot.state is CorridorSlotState.COMMITTED
                    and slot.entry_time <= now + immediate_window
                )
            )
            passages[slot.robot_id] = {
                "regions": slot.regions,
                "entry_lm": str(entry.get("src") or ""),
                "staging_lm": slot.staging_lm,
                "staging_clock": staging_clock,
                # Immutable trajectory-clock template. Calendar windows are
                # regenerated from it; they must never become input to the
                # next route-time projection.
                "route_resource_windows": route_resource_windows,
                "exit_lm": slot.exit_lm,
                "direction": slot.direction,
                "lease_until": slot.exit_time,
                "entry_time": slot.entry_time,
                "exit_time": slot.exit_time,
                "resource_intervals": resource_intervals,
                "entered": entered,
                "committed": (
                    slot.state is CorridorSlotState.COMMITTED
                ),
                "tentative": (
                    slot.state is CorridorSlotState.TENTATIVE
                ),
                "past_commit_point": slot.past_commit_point,
                "route_revision": slot.route_revision,
                "schedule_epoch": schedule.epoch,
            }
            if active:
                winners[slot.robot_id] = slot.regions[0]
            for interval in resource_intervals:
                region_id = str(interval["region"])
                interval_entry = float(interval["entry_time"])
                interval_exit = float(interval["exit_time"])
                resource_active = bool(
                    slot.state is CorridorSlotState.COMMITTED
                    and active
                    and interval_entry <= now + immediate_window
                    and interval_exit > now - 0.000001
                )
                if resource_active:
                    leases.setdefault(
                        region_id,
                        (slot.robot_id, interval_exit),
                    )
                else:
                    queues.setdefault(region_id, []).append(
                        (interval_entry, slot.robot_id)
                    )

        for request in requests:
            if schedule.slot_for(request.robot_id) is not None:
                continue
            for region_id in request.regions:
                queues.setdefault(region_id, []).append(
                    (float("inf"), request.robot_id)
                )
        self._controlled_corridor_passages = passages
        self._controlled_corridor_leases = leases
        self._controlled_corridor_winners = winners
        self._controlled_corridor_queues = {
            region_id: [
                robot_name
                for _, robot_name in sorted(
                    members,
                    key=lambda item: (item[0], item[1]),
                )
            ]
            for region_id, members in queues.items()
        }

    def _controlled_corridor_downstream_blocker(
        self,
        robot: FleetRobot,
        exit_lm: str,
        exit_clock: float,
    ) -> str:
        """Return a body not proven to clear the exit before our arrival."""
        candidate_pose = self._pose_at_trajectory(
            robot.trajectory,
            exit_clock,
        )
        if candidate_pose is None:
            exit_landmark = self.landmarks.get(exit_lm)
            if exit_landmark is None:
                return ""
            candidate_pose = {
                "x": float(exit_landmark.x),
                "y": float(exit_landmark.y),
                "yaw": float(robot.pose.get("yaw", 0.0))
                if robot.pose is not None
                else 0.0,
            }
        for other in self._runtime_robots():
            if other.name == robot.name or other.pose is None:
                continue
            current_conflict = self.collision.robot_footprints_conflict(
                candidate_pose,
                other.pose,
            )
            prediction_offset = max(
                0.0,
                float(exit_clock) - float(robot.route_clock),
            )
            predicted_pose = self._predicted_robot_pose(
                other,
                prediction_offset,
            )
            predicted_conflict = bool(
                predicted_pose is not None
                and self.collision.robot_footprints_conflict(
                    candidate_pose,
                    predicted_pose,
                )
            )
            predicted_at_terminal = False
            if other.trajectory:
                final_clock = float(
                    other.trajectory[-1].get("t", 0.0) or 0.0
                )
                predicted_at_terminal = (
                    float(other.route_clock) + prediction_offset
                    >= final_clock - self._runtime_motion_step()
                )
            if not current_conflict:
                # Future moving/moving crossings remain SIPP's responsibility.
                # A trajectory which *ends* in the exit pocket is different:
                # there is no committed departure after its arrival, so
                # admitting this corridor would knowingly block the box.
                # Protect that terminal body even while it is still moving
                # toward the exit. Ordinary through-traffic remains temporal
                # SIPP traffic and is not promoted to a hard block.
                if predicted_conflict and predicted_at_terminal:
                    return other.name
                continue
            if predicted_conflict or (
                other.status != "MOVING"
                or not other.trajectory
            ):
                return other.name
        return ""

    def _controlled_corridor_physical_exit_time(
        self,
        robot: FleetRobot,
        exit_lm: str,
        now: float,
    ) -> float:
        """Project an entered owner's safe exit from its live route clock.

        Reusing an overdue pre-entry slot as physical occupancy collapsed its
        exit to ``now + occupancy_recheck`` on every tick.  The first future
        trajectory sample at the atomic bundle's external exit is a safer and
        much more stable estimate: while a robot moves, simulation time and
        route time advance together; while it waits, the estimate moves forward
        and the corridor remains protected.
        """
        remaining = self._runtime_motion_step()
        route_clock = float(robot.route_clock)
        for sample in robot.trajectory:
            if not isinstance(sample, dict):
                continue
            sample_clock = float(sample.get("t", 0.0) or 0.0)
            if sample_clock + 0.000001 < route_clock:
                continue
            if str(sample.get("lm") or "") != exit_lm:
                continue
            remaining = max(remaining, sample_clock - route_clock)
            break
        return float(now) + remaining

    def _controlled_corridor_admission_reason(
        self,
        robot: FleetRobot,
        check_clock: float,
    ) -> str:
        graph = self._controlled_corridor_graph
        if (
            graph is None
            or not robot.trajectory
            or robot.status == "RETREATING"
        ):
            return ""
        inside = self._controlled_regions_for_robot(robot)
        upcoming = self._next_controlled_corridor_entry(robot)
        if upcoming is not None:
            region_id = str(upcoming["region"])
            regions = self._controlled_corridor_entry_regions(upcoming)
            entry_lm = str(upcoming["src"])
            holding_lm = str(upcoming.get("holding_lm") or "")
            stop_lm = holding_lm or entry_lm
            at_stop_line = self._controlled_corridor_pose_is_at_lm(
                robot.pose,
                stop_lm,
            )
            # A physics substep may straddle a graph sample without ever
            # placing the pose inside the old 3 cm stop-line tolerance.  Gate
            # the crossing interval itself, not just the sampled position.
            staging_clock = float(
                upcoming.get(
                    "staging_clock",
                    upcoming.get("entry_clock", robot.route_clock),
                )
                or 0.0
            )
            immediate_window = max(
                self._runtime_motion_step(),
                self._continuous_collision_step(),
            )
            crossing_boundary = bool(
                check_clock >= staging_clock - 0.000001
                and robot.route_clock <= staging_clock + 0.000001
                and check_clock - robot.route_clock
                <= immediate_window + 0.000001
            )
            staging_motion_complete = bool(
                float(robot.route_clock)
                >= staging_clock - 0.000001
            )
            passed_staging = bool(upcoming.get("passed_staging"))
            if (
                region_id not in inside
                and (
                    (at_stop_line and staging_motion_complete)
                    or crossing_boundary
                    or passed_staging
                )
                and not self._controlled_corridor_has_grant(
                    robot.name,
                    regions,
                )
            ):
                return self._controlled_corridor_wait_reason(
                    robot,
                    stop_lm,
                    region_id,
                )
            # A complete atomic passage was identified, so its route-clock
            # stop line is authoritative. The lane-level compatibility
            # fallback below would otherwise see the same edge while the
            # robot is merely rotating at its holding LM and stop that turn.
            return ""
        if (
            check_clock - robot.route_clock
            > max(
                self._runtime_motion_step(),
                self._continuous_collision_step(),
            )
            + 0.000001
        ):
            return ""
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, check_clock)
        )
        if edge is None:
            return ""
        src, dst = edge
        lane = graph.lane_for(src, dst)
        if lane is None or not lane.controlled_region_ids:
            return ""
        src_vertex = graph.vertices.get(src)
        lane_regions = tuple(lane.controlled_region_ids)
        for region_id in lane_regions:
            if region_id in inside:
                continue
            if src_vertex is not None and (
                not src_vertex.can_wait
                or src_vertex.controlled_region_ids
            ):
                # A legacy robot already inside a no-wait passage must clear
                # it; new entrants are protected by the atomic bundle above.
                # This also covers auto-detected transfer junctions which are
                # deliberately non-waitable but do not belong to either
                # adjacent corridor resource themselves.
                continue
            if self._controlled_corridor_has_grant(
                robot.name,
                lane_regions,
            ):
                continue
            return self._controlled_corridor_wait_reason(
                robot,
                src,
                region_id,
            )
        return ""

    def _transfer_controlled_corridor_lease(
        self,
        winner: FleetRobot,
        participants: list[FleetRobot],
        now: float,
    ) -> bool:
        """Confirm a central-calendar owner during deadlock arbitration."""
        del participants, now
        if self._controlled_corridor_scheduler is None:
            return False

        # Generic wait-cycle arbitration may confirm the calendar owner, but
        # must never repaint corridor authority independently.
        upcoming = self._next_controlled_corridor_entry(winner)
        regions = list(
            self._controlled_corridor_entry_regions(upcoming)
        )
        for current_region in self._controlled_regions_for_robot(winner):
            if current_region not in regions:
                regions.append(current_region)
        return bool(
            regions
            and self._controlled_corridor_has_grant(
                winner.name,
                regions,
            )
        )

    def _controlled_corridor_wait_reason(
        self,
        robot: FleetRobot,
        stop_lm: str,
        region_id: str,
    ) -> str:
        owners = self._controlled_corridor_occupancy.get(region_id, [])
        lease = self._controlled_corridor_leases.get(region_id, ("", 0.0))
        owner = (
            owners[0]
            if owners
            else lease[0]
            or self._controlled_corridor_blockers.get(robot.name, "")
        )
        suffix = f"; owner {owner}" if owner else ""
        reason = (
            f"corridor admission wait at {stop_lm} for {region_id}{suffix}"
        )
        if robot.last_reason != reason:
            self.traffic_metrics["corridorAdmissionWaits"] += 1
        return reason

    def _traffic_zone_control_enabled(self) -> bool:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return True
        value = fleet.get("traffic_zone_control_enabled", True)
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(value)

    def _traffic_zone_param(self, key: str, default: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        return self._positive_float_param(fleet, key, default)

    def _edge_has_explicit_corridor_authority(self, src: str, dst: str) -> bool:
        """Avoid two independent admission gates on one physical edge.

        Dynamic traffic zones regulate fleet-wide demand. A Traffic Editor
        controlled corridor is the more precise local authority for its tagged
        edges, so the coarse zone light must not choose a different winner on
        the same transition.
        """
        graph = self._controlled_corridor_graph
        if graph is None:
            return False
        lane = graph.lane_for(src, dst)
        return bool(lane is not None and lane.controlled_region_ids)

    def _build_traffic_zone_index(self) -> dict[str, str]:
        if not self._traffic_zone_control_enabled() or not self.landmarks:
            return {}
        zone_size = self._traffic_zone_param("traffic_zone_size_m", 6.0)
        if zone_size <= 0.0:
            return {}
        origin_x = min(float(landmark.x) for landmark in self.landmarks.values())
        origin_y = min(float(landmark.y) for landmark in self.landmarks.values())
        zones: dict[str, str] = {}
        for name, landmark in self.landmarks.items():
            column = int(math.floor((float(landmark.x) - origin_x) / zone_size))
            row = int(math.floor((float(landmark.y) - origin_y) / zone_size))
            zones[name] = f"flow:{column}:{row}"
        return zones

    def _traffic_zone_route_demand(self) -> dict[str, int]:
        owners_by_zone: dict[str, set[str]] = {}
        for robot in self._runtime_robots():
            order = self._active_order_for_robot(robot)
            if order is None or len(order.spatial_route_nodes) < 2:
                continue
            route_nodes = [
                str(node)
                for node in order.spatial_route_nodes
                if str(node) in self._traffic_zone_by_lm
            ]
            current = self._traffic_lm_for_robot(robot)
            if current in route_nodes:
                route_nodes = route_nodes[route_nodes.index(current):]
            for zone_id in {
                self._traffic_zone_by_lm[node]
                for node in route_nodes
                if node in self._traffic_zone_by_lm
            }:
                owners_by_zone.setdefault(zone_id, set()).add(robot.name)
        return {
            zone_id: len(owners)
            for zone_id, owners in owners_by_zone.items()
        }

    def _next_traffic_zone_transition(
        self,
        robot: FleetRobot,
    ) -> tuple[str, str, str, str, float] | None:
        if not robot.trajectory:
            return None
        lookahead = self._traffic_zone_param(
            "traffic_zone_entry_lookahead_sec",
            3.0,
        )
        first_index = max(
            0,
            self._trajectory_segment_index(
                robot.trajectory,
                robot.route_clock,
                boundary_belongs_to_previous=True,
            ) - 1,
        )
        for index in range(first_index, len(robot.trajectory) - 1):
            start = robot.trajectory[index]
            end = robot.trajectory[index + 1]
            start_time = float(start.get("t", 0.0) or 0.0)
            end_time = float(end.get("t", start_time) or start_time)
            if end_time + 0.000001 < robot.route_clock:
                continue
            if start_time - robot.route_clock > lookahead + 0.000001:
                break
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            parsed = self._parse_edge_id(edge_id)
            if parsed is None:
                continue
            src, dst = parsed
            if self._edge_has_explicit_corridor_authority(src, dst):
                continue
            src_zone = self._traffic_zone_by_lm.get(src, "")
            dst_zone = self._traffic_zone_by_lm.get(dst, "")
            if not src_zone or not dst_zone or src_zone == dst_zone:
                continue
            first = self.landmarks.get(src)
            second = self.landmarks.get(dst)
            if first is None or second is None:
                continue
            dx = float(second.x) - float(first.x)
            dy = float(second.y) - float(first.y)
            if abs(dx) >= abs(dy):
                phase = "E" if dx >= 0.0 else "W"
            else:
                phase = "S" if dy >= 0.0 else "N"
            return src, dst, dst_zone, phase, max(0.0, start_time - robot.route_clock)
        return None

    def _prepare_traffic_zone_admissions(self, now: float) -> None:
        self._traffic_zone_tick_now = now
        self._traffic_zone_winners = {}
        self._traffic_zone_queues = {}
        if not self._traffic_zone_by_lm or not self._traffic_zone_control_enabled():
            self._traffic_zone_demand = {}
            self._traffic_zone_occupancy = {}
            return

        demand = self._traffic_zone_route_demand()
        self._traffic_zone_demand = demand
        threshold = max(
            1,
            int(self._traffic_zone_param("traffic_zone_demand_threshold", 6.0)),
        )
        hot_zones = {
            zone_id
            for zone_id, value in demand.items()
            if value >= threshold
        }
        occupancy_owners: dict[str, set[str]] = {}
        for robot in self._runtime_robots():
            current_lm = self._traffic_lm_for_robot(robot)
            zone_id = self._traffic_zone_by_lm.get(current_lm, "")
            if zone_id:
                occupancy_owners.setdefault(zone_id, set()).add(robot.name)
        self._traffic_zone_occupancy = {
            zone_id: len(owners)
            for zone_id, owners in occupancy_owners.items()
        }

        for key, expiry in list(self._traffic_zone_leases.items()):
            zone_id, robot_name = key
            robot = self.robots.get(robot_name)
            current_zone = (
                self._traffic_zone_by_lm.get(self._traffic_lm_for_robot(robot), "")
                if robot is not None
                else ""
            )
            if expiry <= now or robot is None or current_zone == zone_id:
                self._traffic_zone_leases.pop(key, None)

        candidates_by_zone: dict[str, list[dict[str, Any]]] = {}
        candidate_keys: set[tuple[str, str]] = set()
        for robot in self._runtime_robots():
            if robot.status not in {"MOVING", "WAITING"} or robot.is_remote():
                continue
            if robot.traffic_priority_until > now:
                continue
            transition = self._next_traffic_zone_transition(robot)
            if transition is None:
                continue
            src, dst, target_zone, phase, eta = transition
            source_zone = self._traffic_zone_by_lm.get(src, "")
            if target_zone not in hot_zones:
                continue
            # Admit only when moving up the demand gradient. Equal/downhill
            # transitions drain congestion freely, so neighbouring zone gates
            # cannot form a circular wait around the busy region.
            if demand.get(source_zone, 0) >= demand.get(target_zone, 0):
                continue
            key = (target_zone, robot.name)
            candidate_keys.add(key)
            wait_since = self._traffic_zone_wait_since.setdefault(key, now)
            order = self._active_order_for_robot(robot)
            candidates_by_zone.setdefault(target_zone, []).append({
                "robot": robot,
                "src": src,
                "dst": dst,
                "phase": phase,
                "eta": eta,
                "wait_since": wait_since,
                "priority": int(order.priority if order is not None else 0),
            })

        for key in list(self._traffic_zone_wait_since):
            if key not in candidate_keys:
                self._traffic_zone_wait_since.pop(key, None)

        capacity = max(
            1,
            int(self._traffic_zone_param("traffic_zone_capacity", 3.0)),
        )
        batch_size = max(
            1,
            int(self._traffic_zone_param("traffic_zone_batch_size", 3.0)),
        )
        phase_duration = max(
            0.5,
            self._traffic_zone_param("traffic_zone_phase_sec", 3.0),
        )
        lease_duration = max(
            0.5,
            self._traffic_zone_param("traffic_zone_admission_lease_sec", 4.0),
        )
        starvation = max(
            1.0,
            self._traffic_zone_param("traffic_zone_starvation_sec", 8.0),
        )

        for zone_id, candidates in candidates_by_zone.items():
            occupied = set(occupancy_owners.get(zone_id, set()))
            leased = {
                robot_name
                for (lease_zone, robot_name), expiry in self._traffic_zone_leases.items()
                if lease_zone == zone_id and expiry > now
            }
            for robot_name in leased:
                self._traffic_zone_winners[robot_name] = zone_id
            slots = max(0, capacity - len(occupied | leased))
            if slots <= 0:
                available = sorted(
                    (
                        item for item in candidates
                        if item["robot"].name not in leased
                    ),
                    key=lambda item: (
                        item["wait_since"],
                        -item["priority"],
                        item["eta"],
                        item["robot"].name,
                    ),
                )
                starved = [
                    item for item in available
                    if now - float(item["wait_since"]) >= starvation
                ]
                emergency_until = self._traffic_zone_emergency_until.get(
                    zone_id,
                    0.0,
                )
                selected_name = ""
                if starved and emergency_until <= now:
                    # Capacity is a throughput guard, not a collision model. If
                    # robots already inside keep occupancy above the nominal
                    # cap, release one oldest entrant per phase. Exact SIPP and
                    # runtime footprints still veto unsafe physical motion.
                    selected = starved[0]
                    selected_name = selected["robot"].name
                    self._traffic_zone_leases[(zone_id, selected_name)] = (
                        now + lease_duration
                    )
                    self._traffic_zone_winners[selected_name] = zone_id
                    self._traffic_zone_phase[zone_id] = (
                        selected["phase"],
                        now + phase_duration,
                    )
                    self._traffic_zone_emergency_until[zone_id] = (
                        now + phase_duration
                    )
                    self.traffic_metrics["zoneAdmissionsGranted"] += 1
                self._traffic_zone_queues[zone_id] = [
                    item["robot"].name
                    for item in available
                    if item["robot"].name != selected_name
                ]
                continue

            candidates.sort(key=lambda item: (
                item["wait_since"],
                -item["priority"],
                item["eta"],
                item["robot"].name,
            ))
            available = [
                item for item in candidates
                if item["robot"].name not in leased
            ]
            if not available:
                continue
            starved = [
                item for item in available
                if now - float(item["wait_since"]) >= starvation
            ]
            active_phase, phase_until = self._traffic_zone_phase.get(
                zone_id,
                ("", 0.0),
            )
            if starved:
                selected_phase = starved[0]["phase"]
                self._traffic_zone_phase[zone_id] = (
                    selected_phase,
                    now + phase_duration,
                )
            elif active_phase and phase_until > now and any(
                item["phase"] == active_phase for item in available
            ):
                selected_phase = active_phase
            else:
                selected_phase = available[0]["phase"]
                self._traffic_zone_phase[zone_id] = (
                    selected_phase,
                    now + phase_duration,
                )
            compatible = [
                item for item in available
                if item["phase"] == selected_phase
            ]
            selected = compatible[:min(slots, batch_size)]
            for item in selected:
                robot_name = item["robot"].name
                key = (zone_id, robot_name)
                if key not in self._traffic_zone_leases:
                    self.traffic_metrics["zoneAdmissionsGranted"] += 1
                self._traffic_zone_leases[key] = now + lease_duration
                self._traffic_zone_winners[robot_name] = zone_id
            selected_names = {item["robot"].name for item in selected}
            self._traffic_zone_queues[zone_id] = [
                item["robot"].name
                for item in available
                if item["robot"].name not in selected_names
            ]

    def _traffic_zone_admission_reason(
        self,
        robot: FleetRobot,
        check_clock: float,
    ) -> str:
        if (
            not self._traffic_zone_by_lm
            or not self._traffic_zone_control_enabled()
            or robot.status == "RETREATING"
            or robot.traffic_priority_until > self._traffic_zone_tick_now
        ):
            return ""
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, check_clock)
        )
        if edge is None:
            return ""
        src, dst = edge
        if self._edge_has_explicit_corridor_authority(src, dst):
            return ""
        source_zone = self._traffic_zone_by_lm.get(src, "")
        target_zone = self._traffic_zone_by_lm.get(dst, "")
        if not source_zone or not target_zone or source_zone == target_zone:
            return ""
        threshold = max(
            1,
            int(self._traffic_zone_param("traffic_zone_demand_threshold", 6.0)),
        )
        if self._traffic_zone_demand.get(target_zone, 0) < threshold:
            return ""
        if self._traffic_zone_demand.get(source_zone, 0) >= self._traffic_zone_demand.get(
            target_zone,
            0,
        ):
            return ""
        lease = self._traffic_zone_leases.get((target_zone, robot.name), 0.0)
        if lease > self._traffic_zone_tick_now:
            return ""
        # Hold on the graph vertex outside the zone, never midway along the
        # entering edge merely because far-lookahead noticed the closed gate.
        if robot.pose is None or not self._pose_is_at_lm(robot.pose, src):
            return ""
        reason = f"traffic admission wait at {src} for {target_zone}"
        if robot.last_reason != reason:
            self.traffic_metrics["zoneAdmissionWaits"] += 1
        return reason

    def _traffic_flow_payload(self) -> dict[str, Any]:
        zones = []
        corridor_schedule = self._controlled_corridor_schedule
        threshold = max(
            1,
            int(self._traffic_zone_param("traffic_zone_demand_threshold", 6.0)),
        )
        for zone_id in sorted(
            set(self._traffic_zone_demand)
            | set(self._traffic_zone_occupancy)
            | set(self._traffic_zone_queues)
        ):
            demand = int(self._traffic_zone_demand.get(zone_id, 0))
            queue = list(self._traffic_zone_queues.get(zone_id, []))
            if demand < threshold and not queue:
                continue
            phase, phase_until = self._traffic_zone_phase.get(zone_id, ("", 0.0))
            zones.append({
                "id": zone_id,
                "demand": demand,
                "occupancy": int(self._traffic_zone_occupancy.get(zone_id, 0)),
                "queue": queue,
                "phase": phase,
                "phaseUntil": phase_until,
            })
        return {
            "enabled": bool(
                self._traffic_zone_by_lm
                or self._controlled_corridor_graph is not None
            ),
            "controlledCorridorsEnabled": bool(
                self._controlled_corridor_graph is not None
            ),
            "zones": zones,
            "controlledCorridors": [
                {
                    "id": region_id,
                    "occupancy": list(
                        self._controlled_corridor_occupancy.get(region_id, [])
                    ),
                    "queue": list(
                        self._controlled_corridor_queues.get(region_id, [])
                    ),
                    "winner": next(
                        (
                            robot_name
                            for robot_name, winner_region in
                            self._controlled_corridor_winners.items()
                            if winner_region == region_id
                        ),
                        str(
                            self._controlled_corridor_leases.get(
                                region_id,
                                ("", 0.0),
                            )[0]
                            or ""
                        ),
                    ),
                }
                for region_id in sorted(
                    set(self._controlled_corridor_occupancy)
                    | set(self._controlled_corridor_queues)
                    | set(self._controlled_corridor_leases)
                )
            ],
            "controlledCorridorSchedule": {
                "epoch": (
                    corridor_schedule.epoch
                    if corridor_schedule is not None
                    else 0
                ),
                "generatedAt": (
                    corridor_schedule.generated_at
                    if corridor_schedule is not None
                    else 0.0
                ),
                "horizonEnd": (
                    corridor_schedule.horizon_end
                    if corridor_schedule is not None
                    else 0.0
                ),
                "slots": [
                    {
                        "robot": slot.robot_id,
                        "regions": list(slot.regions),
                        "direction": slot.direction,
                        "entryTime": slot.entry_time,
                        "exitTime": slot.exit_time,
                        "stagingLm": slot.staging_lm,
                        "exitLm": slot.exit_lm,
                        "state": slot.state.value,
                        "pastCommitPoint": slot.past_commit_point,
                        "physicallyObserved": slot.physically_observed,
                        "routeRevision": slot.route_revision,
                        "resourceWindows": [
                            {
                                "region": window.region_id,
                                "direction": window.direction,
                                "entryTime": (
                                    slot.entry_time
                                    + window.entry_offset_sec
                                ),
                                "exitTime": (
                                    slot.entry_time
                                    + window.exit_offset_sec
                                ),
                            }
                            for window in slot.resource_windows
                        ],
                    }
                    for slot in (
                        corridor_schedule.slots
                        if corridor_schedule is not None
                        else ()
                    )
                ],
            },
        }

    def _attach_spatial_route_to_request(
        self,
        request: dict[str, Any],
        order: FleetOrder,
        start_lm: str,
        planning_goal_lm: str,
        final_goal_lm: str,
        *,
        release_robot_names: set[str] | None = None,
    ) -> None:
        try:
            suffix = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
                release_robot_names=release_robot_names,
            )
        except ValueError:
            if order.internal_kind == "traffic_clearance":
                # A maintenance move has exactly one graph-safe evacuation
                # route.  Missing that suffix is a failed maintenance attempt,
                # never permission to let the planner choose another path.
                raise
            return
        if planning_goal_lm not in suffix:
            if order.internal_kind == "traffic_clearance":
                raise ValueError(
                    "traffic clearance planning goal is outside its fixed route"
                )
            return
        goal_index = suffix.index(planning_goal_lm)
        route_nodes = suffix[:goal_index + 1]
        if len(route_nodes) >= 2:
            request["routeNodes"] = route_nodes
        elif order.internal_kind == "traffic_clearance":
            raise ValueError("traffic clearance request has no executable edge")

    def _adopt_coupled_spatial_detour(
        self,
        order: FleetOrder,
        plan: dict[str, Any],
        final_goal_lm: str,
    ) -> None:
        if order.internal_kind == "traffic_clearance":
            # Coupled temporal planning may schedule waits on the explicit
            # route, but it must never promote a locally discovered path into
            # a new spatial clearance route.
            return
        plan_nodes: list[str] = []
        for value in plan.get("nodes", []):
            node = str(value)
            if node in self.landmarks and (not plan_nodes or plan_nodes[-1] != node):
                plan_nodes.append(node)
        if len(plan_nodes) < 2:
            return
        chunk_goal = plan_nodes[-1]
        old_nodes = [str(node) for node in order.spatial_route_nodes]
        owner = str(order.vehicle or order.assigned_robot or "").strip()
        stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names={owner} if owner else set(),
        )
        old_suffix = (
            old_nodes[old_nodes.index(chunk_goal) + 1:]
            if chunk_goal in old_nodes
            else []
        )
        if old_suffix and not stationary_lms.intersection(old_suffix):
            suffix = old_suffix
        elif chunk_goal == final_goal_lm:
            suffix = []
        else:
            try:
                suffix = self.planner.route_planner.find_route(
                    chunk_goal,
                    final_goal_lm,
                    blocked_edges=(
                        self._dynamic_blocked_edges()
                        | self._blocked_edges_for_lms(stationary_lms)
                    ),
                    edge_penalties=(
                        self._traffic_route_edge_penalties(
                            order,
                            chunk_goal,
                            final_goal_lm,
                        )
                        if self._congestion_routing_enabled()
                        else None
                    ),
                ).nodes[1:]
            except ValueError:
                suffix = []
        adopted = plan_nodes + [node for node in suffix if node != plan_nodes[-1]]
        if not adopted or adopted[-1] != final_goal_lm:
            return
        order.spatial_route_nodes = adopted
        order.spatial_route_revision = self._next_route_revision()

    def _rolling_planning_goal(
        self,
        start_lm: str,
        final_goal_lm: str,
        order: FleetOrder,
        *,
        release_robot_names: set[str] | None = None,
    ) -> str:
        """Choose the committed waypoint before running time-aware MAPF.

        Planning the complete lifelong order first defeats a rolling horizon:
        sufficiently distant goals exceed the low-level time bound before a
        usable prefix exists.  The spatial route is only used to select a graph
        waypoint; MAPF still owns all timing, waiting and conflict decisions.
        """
        horizon = self._rolling_horizon()
        step_limit = self._rolling_horizon_steps()
        if horizon <= 0.0 and step_limit <= 0:
            return final_goal_lm
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
            return final_goal_lm
        if len(route_nodes) < 2:
            return final_goal_lm
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        if order.acceleration > 0.0:
            route_payload["acceleration"] = order.acceleration
        speed = self.planner._route_speed(route_payload)
        acceleration = self.planner._route_acceleration(route_payload)
        traffic_graph = self.planner._traffic_graph(speed)
        elapsed = 0.0
        selected_index = 1
        for index in range(1, len(route_nodes)):
            if step_limit > 0 and index > step_limit:
                break
            elapsed += (
                self.planner._edge_tick_cost(
                    route_nodes[index - 1],
                    route_nodes[index],
                    speed,
                    acceleration,
                )
                * max(0.001, self.planner.time_step_sec)
            )
            # Always choose at least the next graph LM.  A single long edge is
            # indivisible and must be committed as one safe graph segment.
            if index == 1 or horizon <= 0.0 or elapsed <= horizon + 0.000001:
                selected_index = index
                continue
            break
        selected_index = traffic_graph.extend_route_index_to_controlled_exit(
            route_nodes,
            selected_index,
        )
        selected_index = self._rolling_safe_hold_index(
            route_nodes,
            selected_index,
            final_goal_lm,
            traffic_graph=traffic_graph,
        )
        return str(route_nodes[selected_index])

    def _rolling_safe_hold_index(
        self,
        route_nodes: list[str],
        selected_index: int,
        final_goal_lm: str,
        *,
        traffic_graph: Any | None = None,
    ) -> int:
        """Keep a failed rolling handoff out of a transit intersection.

        Auto-corridor maps use degree-three/four LMs as boundaries between
        independently controlled regions. Those LMs are transfer boxes, not
        parking pockets. If a background continuation misses its deadline,
        ending the committed chunk there blocks every incident corridor.

        Stop far enough *inside the inbound controlled region* instead. The
        robot retains that region's occupancy token, while the junction and
        all unrelated exits remain usable. A ready prefetch is still appended
        before this point, so healthy traffic never observes the safe hold.
        """
        if not route_nodes:
            return 0
        selected_index = max(0, min(int(selected_index), len(route_nodes) - 1))
        selected_lm = str(route_nodes[selected_index])
        if selected_lm == final_goal_lm:
            return selected_index

        if traffic_graph is None:
            traffic_graph = self.planner._traffic_graph(
                self.planner._route_speed({}),
            )
        selected_vertex = traffic_graph.vertices.get(selected_lm)
        incoming_lane = (
            traffic_graph.lane_for(
                str(route_nodes[selected_index - 1]),
                selected_lm,
            )
            if selected_index > 0
            else None
        )
        just_exited_controlled_corridor = bool(
            incoming_lane is not None
            and incoming_lane.controlled_region_ids
            and (
                selected_vertex is None
                or not selected_vertex.controlled_region_ids
            )
        )
        outgoing_lane = (
            traffic_graph.lane_for(
                selected_lm,
                str(route_nodes[selected_index + 1]),
            )
            if selected_index + 1 < len(route_nodes)
            else None
        )
        approaching_controlled_corridor = bool(
            outgoing_lane is not None
            and outgoing_lane.controlled_region_ids
            and (
                selected_vertex is None
                or not selected_vertex.controlled_region_ids
            )
        )
        if approaching_controlled_corridor and selected_index > 1:
            # The graph LM immediately before a narrow region is its entry
            # portal, but it is also the physical exit pocket for the
            # opposite phase. A rolling chunk ending on that portal can wait
            # there indefinitely while the admitted owner is unable to leave.
            # Keep the chunk one complete robot clearance upstream. The next
            # rolling continuation will approach the real traffic light only
            # after it has a central slot.
            clearance = max(
                0.60,
                self.collision.robot_broadphase_distance(),
            )
            distance = 0.0
            for cursor in range(selected_index - 1, 0, -1):
                src = str(route_nodes[cursor])
                dst = str(route_nodes[cursor + 1])
                lane = traffic_graph.lane_for(src, dst)
                if lane is not None and lane.controlled_region_ids:
                    # There is no open-space queue pocket on this approach.
                    # Do not move the endpoint back into another corridor.
                    break
                edge = self.planner.edge_by_key.get((src, dst))
                if edge is not None:
                    distance += max(0.0, float(edge.length))
                else:
                    first = self.landmarks.get(src)
                    second = self.landmarks.get(dst)
                    if first is not None and second is not None:
                        distance += math.hypot(
                            first.x - second.x,
                            first.y - second.y,
                        )
                vertex = traffic_graph.vertices.get(src)
                if (
                    distance + 0.000001 >= clearance
                    and vertex is not None
                    and vertex.can_wait
                    and not vertex.controlled_region_ids
                ):
                    return cursor
        if (
            just_exited_controlled_corridor
            and selected_index + 1 < len(route_nodes)
        ):
            # The first waitable LM outside a narrow region is also its exit
            # portal. It is safe as a traffic-light stop for an entrant on the
            # opposite side, but unsafe as an exhausted rolling endpoint: a
            # stopped body there seals the exit and makes the next owner and
            # the follower wait for each other. Commit a short open-space tail
            # until the complete body is clear of the portal.
            clearance = max(
                0.60,
                float(self.planner.min_robot_center_distance_m),
            )
            distance = 0.0
            for cursor in range(selected_index + 1, len(route_nodes)):
                src = str(route_nodes[cursor - 1])
                dst = str(route_nodes[cursor])
                lane = traffic_graph.lane_for(src, dst)
                if lane is not None and lane.controlled_region_ids:
                    # Never use this clearance rule to enter a second authored
                    # corridor without a separate central slot.
                    break
                edge = self.planner.edge_by_key.get((src, dst))
                if edge is not None:
                    distance += max(0.0, float(edge.length))
                else:
                    first = self.landmarks.get(src)
                    second = self.landmarks.get(dst)
                    if first is not None and second is not None:
                        distance += math.hypot(
                            first.x - second.x,
                            first.y - second.y,
                        )
                vertex = traffic_graph.vertices.get(dst)
                if (
                    distance + 0.000001 >= clearance
                    and vertex is not None
                    and vertex.can_wait
                    and not vertex.controlled_region_ids
                ):
                    return cursor

        if selected_index > 0 and (
            selected_vertex is None
            or selected_vertex.can_wait
        ):
            return selected_index

        clearance = max(
            0.60,
            float(self.planner.min_robot_center_distance_m),
        )
        distance = 0.0
        for cursor in range(selected_index - 1, 0, -1):
            src = str(route_nodes[cursor])
            dst = str(route_nodes[cursor + 1])
            edge = self.planner.edge_by_key.get((src, dst))
            if edge is not None:
                distance += max(0.0, float(edge.length))
            else:
                first = self.landmarks.get(src)
                second = self.landmarks.get(dst)
                if first is not None and second is not None:
                    distance += math.hypot(first.x - second.x, first.y - second.y)
            vertex = traffic_graph.vertices.get(src)
            if (
                distance + 0.000001 >= clearance
                and cursor > 0
                and src != str(route_nodes[0])
                and vertex is not None
                and vertex.can_wait
            ):
                return cursor

        # The no-wait transfer box can be the immediate successor of the
        # current stop line. Backtracking would then make a zero-length
        # continuation, so cross the junction and target the next legal stop
        # line instead.
        for cursor in range(selected_index + 1, len(route_nodes)):
            node = str(route_nodes[cursor])
            if node == final_goal_lm:
                return cursor
            vertex = traffic_graph.vertices.get(node)
            if vertex is not None and vertex.can_wait:
                return cursor
        return selected_index

    def _wait_only_rolling_plan(
        self,
        plan: dict[str, Any],
        final_goal_lm: str,
    ) -> bool:
        nodes = [str(node) for node in plan.get("nodes", [])]
        if not nodes:
            return True
        start_lm = str(plan.get("startLm") or nodes[0])
        if final_goal_lm == start_lm:
            return False
        return all(node == start_lm for node in nodes)

    def _rolling_result(
        self,
        result: dict[str, Any],
        final_goals: dict[str, str],
        *,
        corridor_gates: dict[str, dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Commit only complete graph nodes inside the configured rolling horizon."""
        horizon = self._rolling_horizon()
        step_limit = self._rolling_horizon_steps()
        if horizon <= 0.0 and step_limit <= 0:
            return result
        time_step = max(0.001, float(result.get("timeStepSec", 1.0) or 1.0))
        for plan in result.get("plans", []):
            if not isinstance(plan, dict):
                continue
            robot_name = str(plan.get("robot") or "")
            final_goal = str(final_goals.get(robot_name) or plan.get("goalLm") or "").strip()
            nodes = [str(node) for node in plan.get("nodes", [])]
            times = [int(value) for value in plan.get("times", [])]
            if len(nodes) < 2 or len(times) != len(nodes):
                plan["finalGoalLm"] = final_goal
                continue

            final_index = len(nodes) - 1
            chunk_index = final_index
            if horizon > 0.0:
                allowed_tick = horizon / time_step
                progress_indices = [
                    index
                    for index in range(1, len(nodes))
                    if nodes[index] != nodes[0]
                ]
                eligible = [
                    index
                    for index in progress_indices
                    if float(times[index] - times[0]) <= allowed_tick + 0.000001
                ]
                # A turn can consume the whole short window. Commit the first
                # complete graph move as the minimum useful rolling chunk;
                # never publish a rotate-only plan as "no progress".
                chunk_index = (
                    max(eligible)
                    if eligible
                    else min(progress_indices, default=1)
                )
            if step_limit > 0:
                chunk_index = min(chunk_index, max(1, step_limit))
            chunk_index = min(final_index, max(1, chunk_index))
            corridor_bounds = self._corridor_plan_bounds(
                nodes,
                (
                    corridor_gates.get(robot_name)
                    if isinstance(corridor_gates, dict)
                    else None
                ),
            )
            if corridor_bounds is not None:
                staging_index, exit_index = corridor_bounds
                if chunk_index >= staging_index:
                    # Passing the stop line without also committing the safe
                    # external exit is forbidden. This atomic extension is
                    # intentionally allowed to exceed the ordinary rolling
                    # horizon; the central calendar and SIPP gate already own
                    # the complete no-wait transaction.
                    chunk_index = max(chunk_index, exit_index)
                else:
                    # SIPP may insert an unexpected upstream wait after the
                    # nominal slot calculation. The prefix is still useful and
                    # safe, but it has not consumed the corridor command.
                    plan["corridorPassageDeferred"] = True
            route_speed = float(
                result.get("debug", {}).get("routeSpeed", 0.0)
                if isinstance(result.get("debug"), dict)
                else 0.0
            )
            if route_speed <= 0.0:
                route_speed = self.planner._route_speed({})
            traffic_graph = self.planner._traffic_graph(route_speed)
            chunk_index = traffic_graph.extend_route_index_to_controlled_exit(
                nodes,
                chunk_index,
            )
            # Temporal reservations can consume part of the horizon after the
            # request's graph-safe waypoint was selected. The result trimmer
            # may therefore stop earlier than that waypoint; apply the same
            # stop-line rule here or it can recreate a no-wait junction chunk
            # even though _rolling_planning_goal chose a safe endpoint.
            chunk_index = self._rolling_safe_hold_index(
                nodes,
                chunk_index,
                final_goal,
                traffic_graph=traffic_graph,
            )

            chunk_goal = nodes[chunk_index]
            arrival_time = max(0.0, float(times[chunk_index] - times[0]) * time_step)
            trajectory = [
                sample for sample in plan.get("trajectory", [])
                if isinstance(sample, dict)
            ]
            trajectory_end = self._trajectory_chunk_end_index(
                trajectory,
                chunk_goal,
                arrival_time,
            )
            if trajectory_end is None and trajectory:
                # A planner result from an older/in-flight worker can lack an
                # LM marker at the newly selected rolling boundary.  Do not
                # publish a shorter node list with the uncut full trajectory:
                # make its real graph terminal the chunk endpoint instead.
                trajectory_goal = str(
                    trajectory[-1].get("lm") or ""
                ).strip()
                terminal_indices = [
                    index
                    for index, node in enumerate(nodes)
                    if node == trajectory_goal
                ]
                if terminal_indices:
                    chunk_index = terminal_indices[-1]
                    chunk_goal = trajectory_goal
                    trajectory_end = len(trajectory) - 1
                    arrival_time = float(
                        trajectory[-1].get("t", arrival_time)
                        or arrival_time
                    )
            if trajectory_end is not None:
                plan["trajectory"] = trajectory[:trajectory_end + 1]
                if plan["trajectory"]:
                    arrival_time = float(plan["trajectory"][-1].get("t", arrival_time) or arrival_time)
            plan["nodes"] = nodes[:chunk_index + 1]
            plan["times"] = times[:chunk_index + 1]
            for key in ("yaws", "actions"):
                values = plan.get(key)
                if isinstance(values, list) and len(values) >= chunk_index + 1:
                    plan[key] = values[:chunk_index + 1]
            timed_segments = plan.get("timedSegments")
            if isinstance(timed_segments, list):
                plan["timedSegments"] = timed_segments[:chunk_index]
            plan["goalLm"] = chunk_goal
            plan["finalGoalLm"] = final_goal
            plan["arrivalTime"] = arrival_time
            plan["rollingChunk"] = bool(
                chunk_goal != final_goal or chunk_index < final_index
            )
        return result

    @staticmethod
    def _corridor_plan_bounds(
        nodes: list[str],
        gate: dict[str, Any] | None,
    ) -> tuple[int, int] | None:
        """Locate the exact staged passage represented by a central gate."""
        if not isinstance(gate, dict):
            return None
        intent = gate.get("intent")
        if not isinstance(intent, dict):
            return None
        corridor_request = intent.get("request")
        entry = intent.get("entry")
        if (
            not isinstance(corridor_request, CorridorRequest)
            or not isinstance(entry, dict)
        ):
            return None
        src = str(entry.get("src") or "")
        dst = str(entry.get("dst") or "")
        transition_index = next(
            (
                index
                for index in range(1, len(nodes))
                if nodes[index - 1] == src and nodes[index] == dst
            ),
            None,
        )
        if transition_index is None:
            return None
        staging_indices = [
            index
            for index, node in enumerate(nodes[:transition_index])
            if node == corridor_request.staging_lm
        ]
        if not staging_indices:
            return None
        exit_index = next(
            (
                index
                for index in range(transition_index, len(nodes))
                if nodes[index] == corridor_request.exit_lm
            ),
            None,
        )
        if exit_index is None:
            return None
        return staging_indices[-1], exit_index

    def _trajectory_chunk_end_index(
        self,
        trajectory: list[dict[str, Any]],
        chunk_goal: str,
        arrival_time: float,
    ) -> int | None:
        if not trajectory:
            return None
        candidates = [
            index
            for index, sample in enumerate(trajectory)
            if str(sample.get("lm") or "").strip() == chunk_goal
        ]
        if candidates:
            # Discrete SIPP node times and the continuous trajectory can
            # differ slightly after acceleration/rotation interpolation.
            # Selecting the last sample before ``arrival_time`` used to cut
            # the trajectory on the following edge while still publishing
            # ``chunk_goal`` as its endpoint.  The robot then physically
            # stopped at one LM but rolling continuation waited forever for
            # another.  A rolling boundary is a graph resource, so the sample
            # carrying that exact LM is authoritative; time is only used to
            # disambiguate repeated visits to the same node.
            return min(
                candidates,
                key=lambda index: (
                    abs(
                        float(
                            trajectory[index].get("t", 0.0)
                            or 0.0
                        )
                        - arrival_time
                    ),
                    index,
                ),
            )
        # Never manufacture a graph endpoint from an arbitrary mid-edge
        # sample.  Callers retain the complete trajectory and the metadata
        # guard normalises its real terminal LM.
        return None
