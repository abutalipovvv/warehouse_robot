"""Reservation-table construction, path writing and plan validation."""

from __future__ import annotations

from ..cbs.cbs_models import LmRobotPlan, LmRobotRequest, NodeName
from ..common.reservations import (
    ReservationInterval,
    ReservationTable,
    ResourceId,
)
from .rolling_models import StaticReservations
from ..sipp.sipp_models import TimedPath, TimedState
from ..graph.traffic_graph_models import TrafficGraph


class ResourceReservationWriter:
    """Write graph resources to a :class:`ReservationTable`."""

    def __init__(self, graph: TrafficGraph) -> None:
        self.graph = graph

    def reserve_vertex(
        self,
        reservations: ReservationTable,
        node: NodeName,
        start: int,
        end: int,
        robot_name: str,
        reason: str,
        *,
        committed: bool = True,
    ) -> None:
        for resource in self.graph.vertex_resources(node):
            reservations.reserve(
                ReservationInterval(
                    resource=resource,
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason=reason,
                    committed=committed,
                )
            )

    def reserve_rotation(
        self,
        reservations: ReservationTable,
        node: NodeName,
        start: int,
        end: int,
        robot_name: str,
        *,
        committed: bool = True,
    ) -> None:
        for resource in self.graph.rotation_resources(node):
            reservations.reserve(
                ReservationInterval(
                    resource=resource,
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason="rotate",
                    committed=committed,
                )
            )

    def reserve_lane(
        self,
        reservations: ReservationTable,
        source: NodeName,
        target: NodeName,
        start: int,
        end: int,
        robot_name: str,
        reason: str,
        *,
        committed: bool = True,
    ) -> None:
        lane = self.graph.lane_for(source, target)
        if lane is None:
            reservations.reserve(
                ReservationInterval(
                    resource=ResourceId(
                        "lane",
                        f"{source}->{target}",
                    ),
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason=reason,
                    committed=committed,
                )
            )
            return

        for resource in self.graph.lane_resources(lane):
            reservations.reserve(
                ReservationInterval(
                    resource=resource,
                    robot_name=robot_name,
                    start=start,
                    end=end,
                    reason=reason,
                    committed=committed,
                )
            )


class ReservationTableFactory:
    """Build the same initial table for every priority-order attempt."""

    def __init__(
        self,
        graph: TrafficGraph,
        resource_writer: ResourceReservationWriter,
    ) -> None:
        self.graph = graph
        self.resource_writer = resource_writer

    def create(
        self,
        requests: list[LmRobotRequest],
        static: StaticReservations,
    ) -> ReservationTable:
        reservations = ReservationTable(
            self.graph.reservation_capacities()
        )
        self._write_static(reservations, static)
        self._write_initial_positions(reservations, requests)
        return reservations

    def _write_static(
        self,
        reservations: ReservationTable,
        static: StaticReservations,
    ) -> None:
        for time_tick, node in static.vertex_constraints:
            self.resource_writer.reserve_vertex(
                reservations,
                node,
                time_tick,
                time_tick + 1,
                "reserved",
                "constraint",
            )
        for time_tick, source, target in static.edge_constraints:
            self.resource_writer.reserve_lane(
                reservations,
                source,
                target,
                time_tick,
                time_tick + 1,
                "reserved",
                "constraint",
            )
        for start, end, node, owner in static.vertex_intervals:
            self.resource_writer.reserve_vertex(
                reservations,
                node,
                start,
                end + 1,
                owner or "reserved",
                "reserved",
            )
        for (
            start,
            end,
            source,
            target,
            owner,
        ) in static.edge_intervals:
            self.resource_writer.reserve_lane(
                reservations,
                source,
                target,
                start,
                end + 1,
                owner or "reserved",
                "reserved",
            )

    def _write_initial_positions(
        self,
        reservations: ReservationTable,
        requests: list[LmRobotRequest],
    ) -> None:
        for request in requests:
            self.resource_writer.reserve_vertex(
                reservations,
                request.start_lm,
                0,
                1,
                request.robot_name,
                "initial_position",
            )


class PathReservationWriter:
    """Reserve visits, waits, rotations, moves and final occupancy."""

    def __init__(
        self,
        resource_writer: ResourceReservationWriter,
        *,
        low_level_max_time: int,
    ) -> None:
        self.resource_writer = resource_writer
        self.low_level_max_time = low_level_max_time

    def reserve(
        self,
        reservations: ReservationTable,
        path: TimedPath,
    ) -> None:
        states = list(path.states)
        for state in states:
            self.resource_writer.reserve_vertex(
                reservations,
                state.node,
                state.time,
                state.time + 1,
                path.robot_name,
                "visit",
                committed=False,
            )

        for start, end in zip(states, states[1:]):
            if start.node == end.node:
                if end.action == "rotate":
                    self.resource_writer.reserve_rotation(
                        reservations,
                        start.node,
                        start.time,
                        end.time + 1,
                        path.robot_name,
                        committed=False,
                    )
                else:
                    self.resource_writer.reserve_vertex(
                        reservations,
                        start.node,
                        start.time,
                        end.time + 1,
                        path.robot_name,
                        "wait",
                        committed=False,
                    )
                continue

            self.resource_writer.reserve_lane(
                reservations,
                start.node,
                end.node,
                start.time,
                end.time,
                path.robot_name,
                "move",
                committed=False,
            )

        final = states[-1]
        self.resource_writer.reserve_vertex(
            reservations,
            final.node,
            final.time,
            self.low_level_max_time + 1,
            path.robot_name,
            "goal",
            committed=False,
        )


class RollingPlanValidator:
    """Validate complete plans against the same reservation rules."""

    def __init__(
        self,
        graph: TrafficGraph,
        table_factory: ReservationTableFactory,
        path_writer: PathReservationWriter,
        *,
        low_level_max_time: int,
    ) -> None:
        self.graph = graph
        self.table_factory = table_factory
        self.path_writer = path_writer
        self.low_level_max_time = low_level_max_time

    def validate(
        self,
        robot_requests: list[LmRobotRequest],
        plans: dict[str, LmRobotPlan],
        static: StaticReservations,
    ) -> str:
        reservations = self.table_factory.create(
            robot_requests,
            static,
        )
        for request in robot_requests:
            plan = plans.get(request.robot_name)
            if plan is None:
                return f"missing_plan:{request.robot_name}"

            path = self._timed_path(plan)
            if not self._path_is_free(
                reservations,
                path,
                authorized_controlled_regions=(
                    request.authorized_controlled_regions
                ),
            ):
                return f"resource_conflict:{request.robot_name}"
            self.path_writer.reserve(reservations, path)
        return ""

    @staticmethod
    def _timed_path(plan: LmRobotPlan) -> TimedPath:
        return TimedPath(
            robot_name=plan.robot_name,
            start_lm=plan.start_lm,
            goal_lm=plan.goal_lm,
            states=tuple(
                TimedState(
                    int(time_tick),
                    node,
                    (
                        float(plan.yaws[index])
                        if index < len(plan.yaws)
                        else 0.0
                    ),
                    (
                        str(plan.actions[index])
                        if index < len(plan.actions)
                        else "wait"
                    ),
                )
                for index, (time_tick, node) in enumerate(
                    zip(plan.times, plan.nodes)
                )
            ),
        )

    def _path_is_free(
        self,
        reservations: ReservationTable,
        path: TimedPath,
        *,
        authorized_controlled_regions: tuple[str, ...] = (),
    ) -> bool:
        states = list(path.states)
        if not states:
            return False

        authorized_regions = frozenset(
            authorized_controlled_regions
        )

        def usable(
            resources: tuple[ResourceId, ...],
        ) -> tuple[ResourceId, ...]:
            if not authorized_regions:
                return resources
            return tuple(
                resource
                for resource in resources
                if not (
                    resource.kind == "controlled_region"
                    and resource.name in authorized_regions
                )
            )

        for state in states:
            if not reservations.resources_are_free(
                usable(
                    self.graph.vertex_resources(state.node)
                ),
                state.time,
                state.time + 1,
                ignore_robot_name=path.robot_name,
            ):
                return False

        for start, end in zip(states, states[1:]):
            if start.node == end.node:
                resources = (
                    usable(
                        self.graph.rotation_resources(start.node)
                    )
                    if end.action == "rotate"
                    else usable(
                        self.graph.vertex_resources(start.node)
                    )
                )
                interval_end = end.time + 1
            else:
                lane = self.graph.lane_for(
                    start.node,
                    end.node,
                )
                resources = (
                    usable(self.graph.lane_resources(lane))
                    if lane is not None
                    else (
                        ResourceId(
                            "lane",
                            f"{start.node}->{end.node}",
                        ),
                    )
                )
                interval_end = end.time

            if not reservations.resources_are_free(
                resources,
                start.time,
                interval_end,
                ignore_robot_name=path.robot_name,
            ):
                return False

        final = states[-1]
        return reservations.resources_are_free(
            usable(self.graph.vertex_resources(final.node)),
            final.time,
            self.low_level_max_time + 1,
            ignore_robot_name=path.robot_name,
        )
