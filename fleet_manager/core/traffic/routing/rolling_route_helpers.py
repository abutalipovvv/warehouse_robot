"""Rolling route result, safe-hold and chunk-boundary helpers."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.fleet.domain.models import FleetOrder
from fleet_manager.core.traffic.corridors.scheduling.corridor_scheduler import CorridorRequest


class RollingRouteMixin:
    """Trim rolling plans only at graph-safe and corridor-atomic boundaries."""

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
            hold_index = self._rolling_hold_before_controlled_corridor(
                route_nodes,
                selected_index,
                traffic_graph,
            )
            if hold_index is not None:
                return hold_index
        if (
            just_exited_controlled_corridor
            and selected_index + 1 < len(route_nodes)
        ):
            hold_index = self._rolling_hold_after_controlled_corridor(
                route_nodes,
                selected_index,
                traffic_graph,
            )
            if hold_index is not None:
                return hold_index

        if selected_index > 0 and (
            selected_vertex is None
            or selected_vertex.can_wait
        ):
            return selected_index

        upstream_hold = self._rolling_hold_before_transfer_box(
            route_nodes,
            selected_index,
            traffic_graph,
        )
        if upstream_hold is not None:
            return upstream_hold

        # If backtracking would make a zero-length continuation, cross the
        # transfer box and target the next legal stop line instead.
        return self._rolling_hold_after_transfer_box(
            route_nodes,
            selected_index,
            final_goal_lm,
            traffic_graph,
        )

    def _rolling_hold_before_controlled_corridor(
        self,
        route_nodes: list[str],
        selected_index: int,
        traffic_graph: Any,
    ) -> int | None:
        """Keep one complete robot clearance before an entry portal."""
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
                return None
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
        return None

    def _rolling_hold_after_controlled_corridor(
        self,
        route_nodes: list[str],
        selected_index: int,
        traffic_graph: Any,
    ) -> int | None:
        """Move a rolling endpoint far enough beyond an exit portal."""
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
                return None
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
        return None

    def _rolling_hold_before_transfer_box(
        self,
        route_nodes: list[str],
        selected_index: int,
        traffic_graph: Any,
    ) -> int | None:
        """Find a waitable upstream LM before a non-waitable junction."""
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
                    distance += math.hypot(
                        first.x - second.x,
                        first.y - second.y,
                    )
            vertex = traffic_graph.vertices.get(src)
            if (
                distance + 0.000001 >= clearance
                and cursor > 0
                and src != str(route_nodes[0])
                and vertex is not None
                and vertex.can_wait
            ):
                return cursor
        return None

    @staticmethod
    def _rolling_hold_after_transfer_box(
        route_nodes: list[str],
        selected_index: int,
        final_goal_lm: str,
        traffic_graph: Any,
    ) -> int:
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
            final_goal = str(
                final_goals.get(robot_name) or plan.get("goalLm") or ""
            ).strip()
            nodes = [str(node) for node in plan.get("nodes", [])]
            times = [int(value) for value in plan.get("times", [])]
            if len(nodes) < 2 or len(times) != len(nodes):
                plan["finalGoalLm"] = final_goal
                continue

            final_index = len(nodes) - 1
            chunk_index = self._rolling_horizon_chunk_index(
                nodes,
                times,
                horizon=horizon,
                time_step=time_step,
                step_limit=step_limit,
            )
            gate = (
                corridor_gates.get(robot_name)
                if isinstance(corridor_gates, dict)
                else None
            )
            chunk_index = self._rolling_corridor_chunk_index(
                plan,
                nodes,
                chunk_index,
                gate,
            )
            traffic_graph = self._rolling_result_traffic_graph(result)
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
            (
                chunk_index,
                chunk_goal,
                arrival_time,
                trajectory,
                trajectory_end,
            ) = self._rolling_trajectory_boundary(
                plan,
                nodes,
                times,
                chunk_index,
                time_step,
            )
            self._commit_rolling_plan_chunk(
                plan,
                nodes,
                times,
                chunk_index=chunk_index,
                chunk_goal=chunk_goal,
                final_goal=final_goal,
                final_index=final_index,
                arrival_time=arrival_time,
                trajectory=trajectory,
                trajectory_end=trajectory_end,
            )
        return result

    @staticmethod
    def _rolling_horizon_chunk_index(
        nodes: list[str],
        times: list[int],
        *,
        horizon: float,
        time_step: float,
        step_limit: int,
    ) -> int:
        """Choose the last complete graph move inside the rolling window."""
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
            # A turn can consume the whole short window. The first complete
            # graph move is the minimum useful chunk.
            chunk_index = (
                max(eligible)
                if eligible
                else min(progress_indices, default=1)
            )
        if step_limit > 0:
            chunk_index = min(chunk_index, max(1, step_limit))
        return min(final_index, max(1, chunk_index))

    def _rolling_corridor_chunk_index(
        self,
        plan: dict[str, Any],
        nodes: list[str],
        chunk_index: int,
        gate: dict[str, Any] | None,
    ) -> int:
        """Keep a gated no-wait corridor passage atomic."""
        corridor_bounds = self._corridor_plan_bounds(nodes, gate)
        if corridor_bounds is None:
            return chunk_index
        staging_index, exit_index = corridor_bounds
        if chunk_index >= staging_index:
            # Once the stop line is passed, commit through the external exit.
            return max(chunk_index, exit_index)
        # An upstream temporal wait means the corridor command is deferred.
        plan["corridorPassageDeferred"] = True
        return chunk_index

    def _rolling_result_traffic_graph(self, result: dict[str, Any]) -> Any:
        """Build the resource graph matching this planner result's speed."""
        debug = result.get("debug")
        route_speed = float(
            debug.get("routeSpeed", 0.0)
            if isinstance(debug, dict)
            else 0.0
        )
        if route_speed <= 0.0:
            route_speed = self.planner._route_speed({})
        return self.planner._traffic_graph(route_speed)

    def _rolling_trajectory_boundary(
        self,
        plan: dict[str, Any],
        nodes: list[str],
        times: list[int],
        chunk_index: int,
        time_step: float,
    ) -> tuple[int, str, float, list[dict[str, Any]], int | None]:
        """Match a graph boundary to its physical trajectory sample."""
        chunk_goal = nodes[chunk_index]
        arrival_time = max(
            0.0,
            float(times[chunk_index] - times[0]) * time_step,
        )
        trajectory = [
            sample
            for sample in plan.get("trajectory", [])
            if isinstance(sample, dict)
        ]
        trajectory_end = self._trajectory_chunk_end_index(
            trajectory,
            chunk_goal,
            arrival_time,
        )
        if trajectory_end is not None or not trajectory:
            return (
                chunk_index,
                chunk_goal,
                arrival_time,
                trajectory,
                trajectory_end,
            )

        # An older in-flight result can lack an LM marker at the newly chosen
        # boundary. Use its real graph terminal instead of publishing the full
        # trajectory with a shorter node list.
        trajectory_goal = str(trajectory[-1].get("lm") or "").strip()
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
                trajectory[-1].get("t", arrival_time) or arrival_time
            )
        return (
            chunk_index,
            chunk_goal,
            arrival_time,
            trajectory,
            trajectory_end,
        )

    @staticmethod
    def _commit_rolling_plan_chunk(
        plan: dict[str, Any],
        nodes: list[str],
        times: list[int],
        *,
        chunk_index: int,
        chunk_goal: str,
        final_goal: str,
        final_index: int,
        arrival_time: float,
        trajectory: list[dict[str, Any]],
        trajectory_end: int | None,
    ) -> None:
        """Trim all correlated plan arrays to one consistent boundary."""
        if trajectory_end is not None:
            plan["trajectory"] = trajectory[:trajectory_end + 1]
            if plan["trajectory"]:
                arrival_time = float(
                    plan["trajectory"][-1].get("t", arrival_time)
                    or arrival_time
                )
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
