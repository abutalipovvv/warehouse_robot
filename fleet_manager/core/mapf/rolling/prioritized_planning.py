"""Priority ordering, blocker analysis and rolling SIPP orchestration."""

from __future__ import annotations

import re
from time import monotonic
from typing import Callable

from ..cbs.cbs_models import (
    LmRobotPlan,
    LmRobotRequest,
    NodeName,
    PlannerDebug,
    PlannerResult,
)
from ..common.reservations import ReservationTable, ResourceId
from .rolling_models import (
    RollingPlanningMetrics,
    StaticReservations,
)
from .rolling_reservations import (
    PathReservationWriter,
    ReservationTableFactory,
)
from ..sipp.sipp import SippPlanner
from ..sipp.sipp_models import SippRobotRequest, TimedPath
from ..graph.traffic_graph_models import TrafficGraph


class PriorityOrderManager:
    """Track deterministic robot-order repairs and prevent cycles."""

    def __init__(
        self,
        requests: list[LmRobotRequest],
    ) -> None:
        self.ordered_requests = list(requests)
        self.seen_orders = {self.order_key(self.ordered_requests)}
        self.max_repairs = max(
            2,
            min(32, len(requests) * 2),
        )
        self.repairs = 0

    @property
    def order_count(self) -> int:
        return len(self.seen_orders)

    @staticmethod
    def order_key(
        requests: list[LmRobotRequest],
    ) -> tuple[str, ...]:
        return tuple(request.robot_name for request in requests)

    def proposed_order(
        self,
        robot_name: str,
        blockers: set[str],
    ) -> list[LmRobotRequest]:
        requests = self.ordered_requests
        if not blockers:
            return list(requests)

        indices = {
            request.robot_name: index
            for index, request in enumerate(requests)
        }
        robot_index = indices.get(robot_name)
        blocker_indices = [
            indices[name]
            for name in blockers
            if name in indices
        ]
        if robot_index is None or not blocker_indices:
            return list(requests)

        insert_at = min(blocker_indices)
        if insert_at >= robot_index:
            return list(requests)

        promoted = list(requests)
        request = promoted.pop(robot_index)
        promoted.insert(insert_at, request)
        return promoted

    def try_apply(
        self,
        proposed: list[LmRobotRequest],
        blockers: set[str],
    ) -> bool:
        key = self.order_key(proposed)
        if (
            not blockers
            or self.repairs >= self.max_repairs
            or key in self.seen_orders
        ):
            return False

        self.seen_orders.add(key)
        self.ordered_requests = proposed
        self.repairs += 1
        return True

    def has_seen(
        self,
        proposed: list[LmRobotRequest],
    ) -> bool:
        return self.order_key(proposed) in self.seen_orders


class StagingWaitRepairPolicy:
    """Move unavoidable no-wait corridor delay back to the start LM."""

    _CANNOT_WAIT_PATTERN = re.compile(
        r"cannot_wait:[^@]+@(\d+)-(\d+)"
    )

    def __init__(
        self,
        graph: TrafficGraph,
        *,
        low_level_max_time: int,
        max_attempts: int = 16,
    ) -> None:
        self.graph = graph
        self.low_level_max_time = low_level_max_time
        self.max_attempts = max_attempts

    def next_departure(
        self,
        request: LmRobotRequest,
        failure: str,
        current_departure: int,
        attempts: int,
    ) -> int | None:
        wait_match = self._CANNOT_WAIT_PATTERN.search(failure)
        start_vertex = self.graph.vertices.get(request.start_lm)
        if (
            wait_match is None
            or start_vertex is None
            or not start_vertex.can_wait
            or attempts >= self.max_attempts
        ):
            return None

        wait_start = int(wait_match.group(1))
        wait_end = int(wait_match.group(2))
        next_departure = (
            current_departure
            + max(1, wait_end - wait_start)
        )
        if next_departure >= self.low_level_max_time:
            return None
        return next_departure


class BlockerResolver:
    """Resolve planner failure text to exact reservation owners."""

    _RESOURCE_KIND_PRIORITY = {
        "vertex": 0,
        "lane": 1,
        "controlled_region": 2,
        "lane_group": 3,
        "mutex_zone": 4,
        "clearance": 5,
        "rotation_clearance": 6,
    }

    def __init__(
        self,
        graph: TrafficGraph,
        *,
        low_level_max_time: int,
    ) -> None:
        self.graph = graph
        self.low_level_max_time = low_level_max_time

    def exact_owners(
        self,
        failure: str,
        reservations: ReservationTable,
        *,
        authorized_controlled_regions: tuple[str, ...] = (),
    ) -> set[str]:
        resources: tuple[ResourceId, ...] = ()
        start = 0
        end = self.low_level_max_time + 1

        edge_match = re.search(
            r"reserved_edge:([^@]+)->([^@]+)@(\d+)-(\d+)",
            failure,
        )
        if edge_match:
            source = edge_match.group(1)
            target = edge_match.group(2)
            resources = self._lane_resources(source, target)
            start = int(edge_match.group(3))
            end = max(
                start + 1,
                int(edge_match.group(4)) + 1,
            )
        else:
            vertex_match = re.search(
                r"reserved_lm:([^@]+)@(\d+)",
                failure,
            )
            if vertex_match:
                resources = self.graph.vertex_resources(
                    vertex_match.group(1)
                )
                start = int(vertex_match.group(2))
                end = start + 1
            else:
                resources, start, end = (
                    self._constraint_resources(failure)
                )

        authorized_regions = frozenset(
            authorized_controlled_regions
        )
        owners: set[str] = set()
        for resource in resources:
            if (
                resource.kind == "controlled_region"
                and resource.name in authorized_regions
            ):
                continue
            for interval in reservations.conflicts(
                resource,
                start,
                end,
            ):
                if interval.robot_name:
                    owners.add(interval.robot_name)
        return owners

    def reservation_pairs(
        self,
        owners: set[str],
        sipp: SippPlanner,
    ) -> tuple[tuple[str, str], ...]:
        """Attach every external owner to one deterministic resource."""

        pairs: list[tuple[str, str]] = []
        for owner in sorted(owners):
            resources = sipp.blocking_resources_by_robot.get(
                owner,
                set(),
            )
            if not resources:
                continue
            resource = min(
                resources,
                key=lambda item: (
                    self._RESOURCE_KIND_PRIORITY.get(
                        item.kind,
                        99,
                    ),
                    item.kind,
                    item.name,
                ),
            )
            pairs.append((owner, str(resource)))
        return tuple(pairs)

    def _constraint_resources(
        self,
        failure: str,
    ) -> tuple[tuple[ResourceId, ...], int, int]:
        resource_vertex_match = re.search(
            r"^(?:resource_constrained|wait_resource_constrained|"
            r"rotation_resource_constrained):([^@]+)"
            r"@(\d+)(?:-(\d+))?",
            failure,
        )
        if resource_vertex_match:
            node = resource_vertex_match.group(1)
            start = int(resource_vertex_match.group(2))
            end = max(
                start + 1,
                int(
                    resource_vertex_match.group(3)
                    or start
                )
                + 1,
            )
            resources = (
                self.graph.rotation_resources(node)
                if failure.startswith(
                    "rotation_resource_constrained:"
                )
                else self.graph.vertex_resources(node)
            )
            return resources, start, end

        resource_edge_match = re.search(
            r"edge_resource_constrained:([^@]+)->([^@]+)"
            r"@(\d+)-(\d+)",
            failure,
        )
        if resource_edge_match:
            source = resource_edge_match.group(1)
            target = resource_edge_match.group(2)
            start = int(resource_edge_match.group(3))
            end = max(
                start + 1,
                int(resource_edge_match.group(4)) + 1,
            )
            return (
                self._lane_resources(source, target),
                start,
                end,
            )

        return (), 0, self.low_level_max_time + 1

    def _lane_resources(
        self,
        source: NodeName,
        target: NodeName,
    ) -> tuple[ResourceId, ...]:
        lane = self.graph.lane_for(source, target)
        if lane is None:
            return (
                ResourceId("lane", f"{source}->{target}"),
            )
        return self.graph.lane_resources(lane)


class PrioritizedSippCoordinator:
    """Compose SIPP, reservations and priority repair into fleet plans."""

    def __init__(
        self,
        graph: TrafficGraph,
        table_factory: ReservationTableFactory,
        path_writer: PathReservationWriter,
        blocker_resolver: BlockerResolver,
        *,
        heuristic_fn: Callable[[NodeName, NodeName], float] | None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None,
        heading_fn: Callable[[NodeName, NodeName], float] | None,
        heading_options_fn: (
            Callable[[NodeName, NodeName], tuple[float, ...]] | None
        ),
        turn_cost_fn: Callable[[float, float], int] | None,
        low_level_max_time: int,
        wait_cost: int,
        max_planning_time_sec: float,
    ) -> None:
        self.graph = graph
        self.table_factory = table_factory
        self.path_writer = path_writer
        self.blocker_resolver = blocker_resolver
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.heading_fn = heading_fn
        self.heading_options_fn = heading_options_fn
        self.turn_cost_fn = turn_cost_fn
        self.low_level_max_time = low_level_max_time
        self.wait_cost = wait_cost
        self.max_planning_time_sec = max_planning_time_sec
        self.staging_repair = StagingWaitRepairPolicy(
            graph,
            low_level_max_time=low_level_max_time,
        )

    def plan(
        self,
        robot_requests: list[LmRobotRequest],
        *,
        blocked_nodes: set[NodeName],
        blocked_edges: set[tuple[NodeName, NodeName]],
        static: StaticReservations,
        should_cancel: Callable[[], bool] | None = None,
    ) -> PlannerResult:
        debug = PlannerDebug(reason="rolling_sipp:init")
        planning_deadline = (
            monotonic() + self.max_planning_time_sec
        )
        if not robot_requests:
            debug.reason = "rolling_sipp:empty_requests"
            return PlannerResult(plans={}, debug=debug)

        request_error = self._request_error(
            robot_requests,
            blocked_nodes,
        )
        if request_error:
            debug.reason = request_error
            return PlannerResult(plans={}, debug=debug)

        sipp = self._sipp_planner()
        metrics = RollingPlanningMetrics()
        priority = PriorityOrderManager(robot_requests)

        while True:
            if should_cancel is not None and should_cancel():
                raise InterruptedError("planning cancelled")
            if monotonic() >= planning_deadline:
                return self._timeout_result(
                    debug,
                    metrics,
                    priority,
                )

            reservations = self.table_factory.create(
                robot_requests,
                static,
            )
            plans: dict[str, LmRobotPlan] = {}
            retry_with_new_order = False

            for request in priority.ordered_requests:
                if should_cancel is not None and should_cancel():
                    raise InterruptedError("planning cancelled")
                if monotonic() >= planning_deadline:
                    return self._timeout_result(
                        debug,
                        metrics,
                        priority,
                    )

                path = self._plan_one_robot(
                    sipp,
                    request,
                    reservations,
                    blocked_nodes,
                    blocked_edges,
                    planning_deadline,
                    metrics,
                    should_cancel,
                )
                if path is None:
                    result, retry_with_new_order = (
                        self._handle_failure(
                            debug,
                            metrics,
                            priority,
                            request,
                            sipp,
                            reservations,
                            plans,
                        )
                    )
                    if result is not None:
                        return result
                    break

                self.path_writer.reserve(reservations, path)
                plans[request.robot_name] = LmRobotPlan(
                    robot_name=request.robot_name,
                    start_lm=request.start_lm,
                    goal_lm=request.goal_lm,
                    nodes=path.nodes,
                    times=path.times,
                    yaws=path.yaws,
                    actions=path.actions,
                )

            if retry_with_new_order:
                continue
            break

        debug.reason = "rolling_sipp:success"
        if priority.repairs:
            debug.reason = (
                f"{debug.reason}:priority_repairs="
                f"{priority.repairs}"
            )
        if metrics.staging_wait_repairs:
            debug.reason = (
                f"{debug.reason}:staging_wait_repairs="
                f"{metrics.staging_wait_repairs}"
            )
        debug.conflicts_resolved = (
            priority.repairs
            + metrics.staging_wait_repairs
        )
        debug.expanded_nodes = metrics.expanded_nodes
        debug.high_level_nodes = priority.order_count
        return PlannerResult(plans=plans, debug=debug)

    def _request_error(
        self,
        requests: list[LmRobotRequest],
        blocked_nodes: set[NodeName],
    ) -> str:
        seen_goals: dict[NodeName, str] = {}
        for request in requests:
            if request.start_lm not in self.graph.vertices:
                return (
                    f"rolling_sipp:unknown_start:"
                    f"{request.robot_name}:{request.start_lm}"
                )
            if request.goal_lm not in self.graph.vertices:
                return (
                    f"rolling_sipp:unknown_goal:"
                    f"{request.robot_name}:{request.goal_lm}"
                )
            if request.start_lm in blocked_nodes:
                return (
                    f"rolling_sipp:start_blocked:"
                    f"{request.robot_name}"
                )
            if request.goal_lm in blocked_nodes:
                return (
                    f"rolling_sipp:goal_blocked:"
                    f"{request.robot_name}"
                )
            if request.goal_lm in seen_goals:
                return (
                    f"rolling_sipp:shared_goal_not_supported:"
                    f"{seen_goals[request.goal_lm]},"
                    f"{request.robot_name}@{request.goal_lm}"
                )
            seen_goals[request.goal_lm] = request.robot_name
        return ""

    def _sipp_planner(self) -> SippPlanner:
        return SippPlanner(
            self.graph,
            heuristic_fn=self.heuristic_fn,
            move_cost_fn=self.move_cost_fn,
            heading_fn=self.heading_fn,
            heading_options_fn=self.heading_options_fn,
            turn_cost_fn=self.turn_cost_fn,
            low_level_max_time=self.low_level_max_time,
            wait_cost=self.wait_cost,
        )

    def _plan_one_robot(
        self,
        sipp: SippPlanner,
        request: LmRobotRequest,
        reservations: ReservationTable,
        blocked_nodes: set[NodeName],
        blocked_edges: set[tuple[NodeName, NodeName]],
        planning_deadline: float,
        metrics: RollingPlanningMetrics,
        should_cancel: Callable[[], bool] | None,
    ) -> TimedPath | None:
        initial_departure = max(
            0,
            int(request.start_not_before_tick),
        )
        staging_attempts = 0

        while True:
            if should_cancel is not None and should_cancel():
                raise InterruptedError("planning cancelled")
            path = sipp.plan(
                SippRobotRequest(
                    robot_name=request.robot_name,
                    start_lm=request.start_lm,
                    goal_lm=request.goal_lm,
                    start_yaw=request.start_yaw,
                    route_nodes=request.route_nodes,
                    initial_departure_not_before=(
                        initial_departure
                    ),
                    node_departure_not_before=(
                        request.node_departure_not_before
                    ),
                    authorized_controlled_regions=(
                        request.authorized_controlled_regions
                    ),
                    no_wait_nodes=request.no_wait_nodes,
                ),
                reservations,
                blocked_nodes=blocked_nodes,
                blocked_edges=blocked_edges,
                planning_deadline=planning_deadline,
                should_cancel=should_cancel,
            )
            metrics.expanded_nodes += sipp.expanded_nodes
            if path is not None:
                return path

            next_departure = self.staging_repair.next_departure(
                request,
                sipp.last_failure,
                initial_departure,
                staging_attempts,
            )
            if next_departure is None:
                return None

            initial_departure = next_departure
            staging_attempts += 1
            metrics.staging_wait_repairs += 1

    def _handle_failure(
        self,
        debug: PlannerDebug,
        metrics: RollingPlanningMetrics,
        priority: PriorityOrderManager,
        request: LmRobotRequest,
        sipp: SippPlanner,
        reservations: ReservationTable,
        plans: dict[str, LmRobotPlan],
    ) -> tuple[PlannerResult | None, bool]:
        failure = (
            sipp.last_failure
            or f"rolling_sipp:no_path:{request.robot_name}"
        )
        if failure.startswith("planning_timeout:"):
            return (
                self._timeout_result(
                    debug,
                    metrics,
                    priority,
                ),
                False,
            )

        exact_blockers = self.blocker_resolver.exact_owners(
            failure,
            reservations,
            authorized_controlled_regions=(
                request.authorized_controlled_regions
            ),
        )
        all_blockers = (
            exact_blockers
            or set(sipp.blocking_robot_names)
        ) - {request.robot_name}
        blockers = all_blockers & set(plans)
        if not blockers:
            blockers = (
                sipp.blocking_robot_names
                & set(plans)
            )

        proposed = priority.proposed_order(
            request.robot_name,
            blockers,
        )
        if priority.try_apply(proposed, blockers):
            return None, True

        debug.reason = failure
        if blockers and priority.has_seen(proposed):
            debug.reason = f"{failure}:priority_cycle"
        elif (
            blockers
            and priority.repairs >= priority.max_repairs
        ):
            debug.reason = (
                f"{failure}:priority_repair_limit"
            )
        elif not blockers:
            debug.reason = (
                f"no_low_level_path:{request.robot_name}:"
                f"{failure}"
            )

        debug.blocking_robots = tuple(sorted(all_blockers))
        debug.blocking_reservations = (
            self.blocker_resolver.reservation_pairs(
                all_blockers,
                sipp,
            )
        )
        debug.conflicts_resolved = priority.repairs
        debug.expanded_nodes = metrics.expanded_nodes
        debug.high_level_nodes = priority.order_count
        return PlannerResult(plans={}, debug=debug), False

    @staticmethod
    def _timeout_result(
        debug: PlannerDebug,
        metrics: RollingPlanningMetrics,
        priority: PriorityOrderManager,
    ) -> PlannerResult:
        debug.reason = "rolling_sipp:planning_timeout"
        debug.conflicts_resolved = priority.repairs
        debug.expanded_nodes = metrics.expanded_nodes
        debug.high_level_nodes = priority.order_count
        return PlannerResult(plans={}, debug=debug)
