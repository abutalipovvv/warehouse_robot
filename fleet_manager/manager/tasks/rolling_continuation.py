"""Select, defer and commit rolling continuation candidates."""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import Any

from fleet_manager.manager.tasks.statuses import TERMINAL_ORDER_STATUSES
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.state import FleetState, PlanningState


class RollingContinuationService:
    """Calculate continuation priorities from explicit planning state."""

    def __init__(
        self,
        fleet_state: FleetState,
        planning_state: PlanningState,
        retry_interval: Callable[[FleetOrder], float],
        clock: Callable[[], float],
    ) -> None:
        self._fleet = fleet_state
        self._state = planning_state
        self._retry_interval = retry_interval
        self._clock = clock

    def mark_eligible(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        remaining: float,
        now: float | None = None,
    ) -> None:
        current_time = self._clock() if now is None else float(now)
        self._state.rolling_prefetch_eligible_since.setdefault(
            robot.name,
            current_time,
        )
        if remaining <= 0.000001 and robot.rolling_boundary_since is None:
            robot.rolling_boundary_since = self.boundary_wait_since(
                order,
                robot,
                current_time,
            )

    def retry_is_due(
        self,
        robot_name: str,
        *,
        now: float | None = None,
        required_blocker: bool = False,
    ) -> bool:
        if required_blocker:
            return True
        current_time = self._clock() if now is None else float(now)
        retry_at = self._state.rolling_prefetch_retry_at.get(robot_name, 0.0)
        return current_time + 0.000001 >= retry_at

    def defer_invalid_clearance(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        error: Exception,
        *,
        now: float | None = None,
    ) -> None:
        current_time = self._clock() if now is None else float(now)
        reason = f"traffic clearance route invalid: {error}"
        order.error = reason
        order.updated_at = current_time
        if robot.status == "WAITING":
            robot.last_reason = reason
        robot.last_tick_at = current_time
        robot.updated_at = current_time
        self._state.rolling_prefetch_retry_at[robot.name] = (
            current_time + self._retry_interval(order)
        )

    def defer_prefetch(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        *,
        boundary_waiting: bool,
        boundary_retry_interval: float,
        time_scale: float,
        retry_multiplier: float = 1.0,
        now: float | None = None,
    ) -> None:
        current_time = self._clock() if now is None else float(now)
        if boundary_waiting:
            retry_at = current_time + max(0.0, boundary_retry_interval)
        else:
            failures = max(
                1,
                int(
                    self._state.rolling_prefetch_failures.get(
                        robot.name,
                        0,
                    )
                    or 0
                ),
            )
            retry_at = current_time + (
                self._retry_interval(order)
                * (2 ** min(3, failures - 1))
                * max(1.0, float(time_scale))
                * max(1.0, retry_multiplier)
            )
        self._state.rolling_prefetch_retry_at[robot.name] = retry_at

    def boundary_wait_since(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        now: float,
    ) -> float:
        if robot.rolling_boundary_since is not None:
            return min(now, float(robot.rolling_boundary_since))
        eligible = self._state.rolling_prefetch_eligible_since.get(robot.name)
        timestamps = [
            float(value)
            for value in (eligible, order.updated_at, robot.updated_at)
            if value is not None and float(value) > 0.0
        ]
        return min([now, *timestamps]) if timestamps else now

    def boundary_priority(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        now: float,
    ) -> tuple[float, float, str]:
        waiting_since = self.boundary_wait_since(order, robot, now)
        last_attempt = self._state.rolling_prefetch_last_attempt_at.get(
            robot.name
        )
        if last_attempt is None:
            failures = self._state.rolling_prefetch_failures.get(robot.name, 0)
            last_attempt = waiting_since + (
                min(8, max(0, int(failures))) * self._retry_interval(order)
            )
        service_anchor = max(waiting_since, float(last_attempt))
        return service_anchor, waiting_since, robot.name

    def candidate_priority(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        remaining: float,
        now: float,
    ) -> tuple[float, float, float, str]:
        if remaining <= 0.000001:
            service_anchor, waiting_since, name = self.boundary_priority(
                order,
                robot,
                now,
            )
            return (
                service_anchor + self._retry_interval(order),
                1.0,
                waiting_since,
                name,
            )
        return (
            now + remaining,
            0.0,
            self._state.rolling_prefetch_eligible_since.get(robot.name, now),
            robot.name,
        )


class RollingContinuationMixin:
    """Select, defer and commit rolling continuation candidates."""

    def _rolling_prefetch_candidates(
        self,
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]:
        lead = self._rolling_prefetch_lead()
        now = self._now()
        required_stopped_blockers: set[str] = set()
        for requester_name in list(self._rolling_prefetch_blockers):
            for blocker_name in self._valid_rolling_prefetch_blockers(
                requester_name
            ):
                blocker = self.robots.get(blocker_name)
                if (
                    blocker is not None
                    and self._robot_waits_at_rolling_boundary(blocker)
                ):
                    required_stopped_blockers.add(blocker_name)
        candidates: list[
            tuple[
                tuple[float, float, float, str],
                FleetOrder,
                FleetRobot,
                dict[str, Any],
                str,
                float,
            ]
        ] = []
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
            self._rolling_continuation_service.mark_eligible(
                order,
                robot,
                remaining,
                now,
            )
            if not self._rolling_continuation_service.retry_is_due(
                robot.name,
                now=now,
                required_blocker=robot.name in required_stopped_blockers,
            ):
                continue
            start_lm = robot.route_chunk_goal_lm
            try:
                planning_goal = self._rolling_planning_goal(
                    start_lm,
                    final_goal,
                    order,
                )
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
            except ValueError as exc:
                if order.internal_kind != "traffic_clearance":
                    raise
                self._defer_invalid_clearance_route(order, robot, now, exc)
                continue
            priority = self._rolling_prefetch_candidate_priority(
                order,
                robot,
                remaining,
                now,
            )
            candidates.append(
                (priority, order, robot, request, final_goal, remaining)
            )
        return [
            (order, robot, request, final_goal, remaining)
            for _, order, robot, request, final_goal, remaining in sorted(
                candidates,
                key=lambda item: item[0],
            )
        ]

    def _defer_invalid_clearance_route(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        now: float,
        error: Exception,
    ) -> None:
        """Hold a broken maintenance route for bounded lifecycle cleanup."""
        self._rolling_continuation_service.defer_invalid_clearance(
            order,
            robot,
            error,
            now=now,
        )

    def _rolling_boundary_wait_since(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        now: float,
    ) -> float:
        """Return a stable age for a stopped continuation holder."""
        return self._rolling_continuation_service.boundary_wait_since(
            order,
            robot,
            now,
        )

    def _rolling_boundary_priority(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str, float],
        now: float,
    ) -> tuple[float, float, str]:
        """Least-recently-served ordering with oldest-waiter fallback."""
        order, robot, _, _, _ = entry
        return self._rolling_continuation_service.boundary_priority(
            order,
            robot,
            now,
        )

    def _rolling_prefetch_candidate_priority(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        remaining: float,
        now: float,
    ) -> tuple[float, float, float, str]:
        """Earliest-deadline scheduling across motion and stopped holders.

        A moving route's hard deadline is its chunk end. A boundary holder's
        service deadline is one short retry quantum after its least-recent
        attempt. This prevents both failure modes: an endless urgent stream
        cannot starve stopped robots, while a just-serviced blocked holder
        cannot repeatedly preempt a route that is about to expire.
        """
        return self._rolling_continuation_service.candidate_priority(
            order,
            robot,
            remaining,
            now,
        )

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
        cohort = self._stopped_rolling_collapse_cohort()
        if cohort is None:
            return None
        signature = self._rolling_collapse_signature(cohort)
        dependencies, by_name, dependencies_complete = (
            self._rolling_collapse_dependencies(cohort)
        )
        sink_name = self._rolling_collapse_sink(
            dependencies,
            dependencies_complete,
        )
        if sink_name:
            order, robot = by_name[sink_name]
            return [self._rolling_collapse_prefetch_entry(order, robot)]

        vacancy_entry = self._rolling_vacancy_escape_entry(cohort, signature)
        # A missing pocket is not terminal. The ordinary fair endpoint queue
        # may retry after the physical traffic state changes.
        return [vacancy_entry] if vacancy_entry is not None else None

    def _stopped_rolling_collapse_cohort(
        self,
    ) -> list[tuple[FleetOrder, FleetRobot]] | None:
        """Return the active cohort only when every member is fully stopped."""
        cohort: list[tuple[FleetOrder, FleetRobot]] = []
        for robot in self._runtime_robots():
            if robot.is_remote() or not robot.active_order_id:
                continue
            order = self.orders.get(robot.active_order_id)
            if order is None or order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.internal_kind == "traffic_clearance":
                # Maintenance routes are immutable bounded escapes already.
                return None
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
        return cohort

    def _rolling_collapse_signature(
        self,
        cohort: list[tuple[FleetOrder, FleetRobot]],
    ) -> tuple[tuple[str, str, int], ...]:
        """Reset failed pockets when the physical collapse episode changes."""
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
        return signature

    def _rolling_collapse_dependencies(
        self,
        cohort: list[tuple[FleetOrder, FleetRobot]],
    ) -> tuple[
        dict[str, set[str]],
        dict[str, tuple[FleetOrder, FleetRobot]],
        bool,
    ]:
        """Build the resource dependency graph for stopped route prefixes."""
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
            for requester_name in dependencies:
                dependencies[requester_name].update(
                    blocker_name
                    for blocker_name in self._valid_rolling_prefetch_blockers(
                        requester_name
                    )
                    if blocker_name in dependencies
                    and blocker_name != requester_name
                )
        return dependencies, by_name, dependencies_complete

    @staticmethod
    def _rolling_collapse_sink(
        dependencies: dict[str, set[str]],
        dependencies_complete: bool,
    ) -> str:
        """Choose the unblocked body that releases the most peers."""
        if not dependencies_complete:
            return ""
        sinks = [
            name
            for name, blocked_by in dependencies.items()
            if not blocked_by
        ]
        if not sinks:
            return ""
        incoming = {
            name: sum(
                name in blocked_by
                for blocked_by in dependencies.values()
            )
            for name in sinks
        }
        return min(
            sinks,
            key=lambda name: (-incoming[name], name),
        )

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
        now = self._now()
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
                        *self._rolling_boundary_priority(entry, now),
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
                *self._rolling_boundary_priority(entry, now),
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
        holder's committed spatial suffix, or when the preceding SIPP/exact
        footprint validation named it as the actual blocker.  The relation is
        made undirected for recovery because either robot may have to move
        first.  Breadth first ordering lets the cheap 2/4-robot attempts
        include the nearest blockers before a genuinely connected component
        is expanded.
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
        for name in names:
            for blocker_name in self._valid_rolling_prefetch_blockers(name):
                if blocker_name not in by_name or blocker_name == name:
                    continue
                adjacency[name].add(blocker_name)
                adjacency[blocker_name].add(name)

        ordered_names: list[str] = []
        queued = [seed_name]
        seen = {seed_name}
        while queued:
            name = queued.pop(0)
            ordered_names.append(name)
            neighbours = sorted(
                adjacency[name] - seen,
                key=lambda neighbour: self._rolling_boundary_priority(
                    by_name[neighbour],
                    self._now(),
                ),
            )
            seen.update(neighbours)
            queued.extend(neighbours)
        return [by_name[name] for name in ordered_names]

    def _valid_rolling_prefetch_blockers(
        self,
        robot_name: str,
    ) -> set[str]:
        """Return unchanged blocker evidence for one rolling continuation."""
        evidence = self._rolling_prefetch_blockers.get(robot_name)
        robot = self.robots.get(robot_name)
        if not isinstance(evidence, dict) or robot is None:
            self._rolling_prefetch_blockers.pop(robot_name, None)
            return set()
        requester_signature = (
            int(robot.route_revision),
            str(robot.route_chunk_goal_lm or ""),
            str(robot.active_order_id or ""),
        )
        raw_requester_signature = evidence.get("requester")
        if (
            not isinstance(raw_requester_signature, tuple)
            or requester_signature != raw_requester_signature
        ):
            self._rolling_prefetch_blockers.pop(robot_name, None)
            return set()

        blockers = evidence.get("blockers")
        if not isinstance(blockers, dict):
            self._rolling_prefetch_blockers.pop(robot_name, None)
            return set()
        valid: set[str] = set()
        for blocker_name, raw_signature in list(blockers.items()):
            blocker = self.robots.get(str(blocker_name))
            if (
                blocker is None
                or blocker.name == robot_name
                or not isinstance(raw_signature, tuple)
                or (
                    int(blocker.route_revision),
                    str(blocker.route_chunk_goal_lm or ""),
                    str(blocker.active_order_id or ""),
                )
                != raw_signature
            ):
                blockers.pop(blocker_name, None)
                continue
            valid.add(blocker.name)
        if not blockers:
            self._rolling_prefetch_blockers.pop(robot_name, None)
        return valid

    def _record_rolling_prefetch_blockers(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
        ],
        debug: dict[str, Any],
        *,
        conflict_robot: str = "",
    ) -> None:
        """Persist exact planner dependencies until either route advances."""
        entry_names = {entry[1].name for entry in entries}
        blockers_by_requester: dict[str, set[str]] = {
            name: set()
            for name in entry_names
        }
        raw_conflicts = debug.get("continuousUnresolvedConflicts", ())
        if isinstance(raw_conflicts, (list, tuple)):
            for raw_conflict in raw_conflicts:
                if not isinstance(raw_conflict, dict):
                    continue
                requester = str(raw_conflict.get("robot") or "").strip()
                blocker = str(raw_conflict.get("other") or "").strip()
                if requester in entry_names and blocker:
                    blockers_by_requester[requester].add(blocker)
                elif blocker in entry_names and requester:
                    # Batch collision reports are directional for scheduling,
                    # but both moving participants form one recovery component.
                    blockers_by_requester[blocker].add(requester)

        reservation_blockers = {
            str(name).strip()
            for name in debug.get("reservationBlockerRobots", ())
            if str(name).strip()
        } if isinstance(
            debug.get("reservationBlockerRobots", ()),
            (list, tuple, set),
        ) else set()
        reservation_owner = (
            conflict_robot
            if conflict_robot in entry_names
            else (
                next(iter(entry_names))
                if len(entry_names) == 1
                else ""
            )
        )
        if reservation_owner:
            blockers_by_requester[reservation_owner].update(
                reservation_blockers
            )
            fallback = str(
                debug.get("continuousConflictRobot") or ""
            ).strip()
            if fallback:
                blockers_by_requester[reservation_owner].add(fallback)

        for _, robot, _, _, _ in entries:
            # One planning result is the newest authoritative evidence for
            # this unchanged request. Do not retain a blocker from an older
            # failure when the latest diagnostic names nobody.
            self._rolling_prefetch_blockers.pop(robot.name, None)
            blocker_signatures: dict[str, tuple[int, str, str]] = {}
            for blocker_name in blockers_by_requester.get(robot.name, set()):
                blocker = self.robots.get(blocker_name)
                if blocker is None or blocker.name == robot.name:
                    continue
                blocker_signatures[blocker.name] = (
                    int(blocker.route_revision),
                    str(blocker.route_chunk_goal_lm or ""),
                    str(blocker.active_order_id or ""),
                )
            if not blocker_signatures:
                continue
            self._rolling_prefetch_blockers[robot.name] = {
                "requester": (
                    int(robot.route_revision),
                    str(robot.route_chunk_goal_lm or ""),
                    str(robot.active_order_id or ""),
                ),
                "blockers": blocker_signatures,
            }

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

    def _rolling_recovery_planning_goal(
        self,
        start_lm: str,
        final_goal_lm: str,
        order: FleetOrder,
        *,
        release_robot_names: set[str],
    ) -> str:
        """Release a stopped boundary with one useful rolling chunk.

        A controlled corridor must be committed through its next safe exit.
        Outside an explicitly controlled corridor, however, returning the
        first neighbouring LM creates a three/four-second chunk that becomes
        an urgent prefetch again immediately. A few recovered robots can then
        monopolise the single planner worker and leave the rest of the fleet
        at ``rolling continuation pending``. Ordinary graph space therefore
        uses the normal horizon-sized rolling goal.
        """
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
        speed = self.planner._route_speed(route_payload)
        traffic_graph = self.planner._traffic_graph(speed)
        first_lane = traffic_graph.lane_for(route_nodes[0], route_nodes[1])
        start_vertex = traffic_graph.vertices.get(route_nodes[0])
        if not (
            first_lane is not None
            and first_lane.controlled_region_ids
        ) and not (
            start_vertex is not None
            and start_vertex.controlled_region_ids
        ):
            return self._rolling_planning_goal(
                start_lm,
                final_goal_lm,
                order,
                release_robot_names=release_robot_names,
            )
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

    def _defer_rolling_prefetch(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        *,
        retry_multiplier: float = 1.0,
    ) -> None:
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        self._rolling_continuation_service.defer_prefetch(
            order,
            robot,
            boundary_waiting=self._robot_waits_at_rolling_boundary(robot),
            boundary_retry_interval=self._rolling_boundary_retry_interval(
                order
            ),
            time_scale=time_scale,
            retry_multiplier=retry_multiplier,
            now=self._now(),
        )

    def _rolling_boundary_retry_interval(
        self,
        order: FleetOrder | None = None,
    ) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = max(
                0.1,
                float(fleet.get("rolling_boundary_retry_sec", 0.5) or 0.5),
            )
        except (TypeError, ValueError):
            configured = 0.5
        try:
            maximum = max(
                0.1,
                float(
                    fleet.get("order_dispatch_retry_max_sec", 4.0)
                    or 4.0
                ),
            )
        except (TypeError, ValueError):
            maximum = 4.0
        return min(configured, maximum)

    def _compact_rolling_trajectory_history(
        self,
        robot: FleetRobot,
    ) -> list[dict[str, Any]]:
        """Drop executed samples while retaining a graph-safe retreat tail.

        Rolling chunks used to append onto the complete order trajectory.
        Reservations, websocket serialization and Babylon route updates then
        became progressively more expensive throughout a long order. Keep
        the current/future timeline plus the previous distinct LM required by
        deadlock retreat. Keep the route clock/timestamps monotonic across the
        append so browser interpolation does not see an artificial route reset.
        """
        trajectory = [
            sample
            for sample in robot.trajectory
            if isinstance(sample, dict)
        ]
        if len(trajectory) < 3 or robot.route_clock <= 0.000001:
            return trajectory

        active_index = self._trajectory_segment_index(
            trajectory,
            robot.route_clock,
            boundary_belongs_to_previous=True,
        )
        keep_index = 0
        distinct_lms: list[str] = []
        for index in range(min(active_index, len(trajectory) - 1), -1, -1):
            sample = trajectory[index]
            if float(sample.get("t", 0.0) or 0.0) > robot.route_clock + 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name not in self.landmarks:
                continue
            if not distinct_lms or distinct_lms[-1] != lm_name:
                distinct_lms.append(lm_name)
            if len(distinct_lms) >= 2:
                keep_index = index
                break

        if keep_index <= 0:
            return trajectory
        compacted = [dict(sample) for sample in trajectory[keep_index:]]
        robot.trajectory = compacted
        compacted_nodes: list[str] = []
        for sample in compacted:
            lm_name = str(sample.get("lm") or "").strip()
            if (
                lm_name in self.landmarks
                and (not compacted_nodes or compacted_nodes[-1] != lm_name)
            ):
                compacted_nodes.append(lm_name)
        if len(compacted_nodes) >= 2:
            robot.plan_nodes = compacted_nodes
        robot.trajectory_dirty = True
        return compacted

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
        current = self._compact_rolling_trajectory_history(robot)
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
        self._clear_rolling_prefetch_state(robot.name)
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
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        return max(
            1.0,
            min(8.0 * time_scale, self._rolling_prefetch_lead() * 0.4),
        )
