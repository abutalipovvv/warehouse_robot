"""Spatial routing, congestion costs and traffic-zone admission."""

from __future__ import annotations

import math
from time import time
from typing import Any

from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, PlannedRoute


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
        route = self.planner.route_planner.find_route(
            start_lm,
            final_goal_lm,
            blocked_edges=blocked_edges,
            edge_penalties=edge_penalties,
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

        Moving/waiting trajectories remain temporal SIPP reservations. A
        QUEUED/PLANNING order alone is not proof that an IDLE/ARRIVED robot is
        about to move: dispatch may be delayed or fail, so until an executable
        trajectory is committed its current LM remains a persistent physical
        obstacle. The only bootstrap exception is a newly spawned robot whose
        first dispatch has never failed; this prevents a freshly created fleet
        from treating every initial spawn as an immutable wall. Once a robot
        has executed any route it never receives that exception again. Coupled
        request owners are explicitly excluded by name so they may receive a
        coordinated departure plan in that same request.
        """
        excluded = exclude_robot_names or set()
        blocked: set[str] = set()
        for robot in self._runtime_robots():
            if robot.name in excluded:
                continue
            if robot.status in {"MOVING", "WAITING", "RETREATING"} and robot.trajectory:
                continue
            pending_order = self._active_order_for_robot(robot)
            if (
                robot.status in {"IDLE", "ARRIVED"}
                and not robot.has_executed_route
                and pending_order is not None
                and pending_order.status in {"QUEUED", "PLANNING"}
                and int(pending_order.dispatch_failures or 0) == 0
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
        for index in range(len(robot.trajectory) - 1):
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
            "enabled": bool(self._traffic_zone_by_lm),
            "zones": zones,
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
            return
        if planning_goal_lm not in suffix:
            return
        goal_index = suffix.index(planning_goal_lm)
        route_nodes = suffix[:goal_index + 1]
        if len(route_nodes) >= 2:
            request["routeNodes"] = route_nodes

    def _adopt_coupled_spatial_detour(
        self,
        order: FleetOrder,
        plan: dict[str, Any],
        final_goal_lm: str,
    ) -> None:
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
            return final_goal_lm
        if len(route_nodes) < 2:
            return final_goal_lm
        if order.dispatch_failures > 0:
            # Recovery must make useful progress instead of rejecting an
            # entire horizon because its second/third edge is temporarily
            # unavailable. The same committed spatial suffix remains intact;
            # only the temporal chunk is shortened to the next graph LM.
            return str(route_nodes[1])

        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        if order.acceleration > 0.0:
            route_payload["acceleration"] = order.acceleration
        speed = self.planner._route_speed(route_payload)
        acceleration = self.planner._route_acceleration(route_payload)
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
        return str(route_nodes[selected_index])

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
            plan["rollingChunk"] = chunk_index < final_index
        return result

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
            and float(sample.get("t", 0.0) or 0.0) >= arrival_time - 0.001
        ]
        if candidates:
            return candidates[0]
        before = [
            index
            for index, sample in enumerate(trajectory)
            if float(sample.get("t", 0.0) or 0.0) <= arrival_time + 0.001
        ]
        return before[-1] if before else 0
