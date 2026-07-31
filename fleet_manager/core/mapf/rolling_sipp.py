"""Public facade for prioritized rolling SIPP planning."""

from __future__ import annotations

from typing import Callable

from .lm_cbs import (
    LmRobotPlan,
    LmRobotRequest,
    NodeName,
    PlannerResult,
)
from .prioritized_planning import (
    BlockerResolver,
    PrioritizedSippCoordinator,
)
from .reservations import ReservationTable
from .rolling_models import (
    EdgeConstraintInput,
    EdgeIntervalInput,
    StaticReservations,
    VertexConstraintInput,
    VertexIntervalInput,
)
from .rolling_reservations import (
    PathReservationWriter,
    ReservationTableFactory,
    ResourceReservationWriter,
    RollingPlanValidator,
)
from .traffic_graph import TrafficGraph


class RollingSippPlanner:
    """Compose reservation, validation and priority-planning services."""

    def __init__(
        self,
        graph: TrafficGraph,
        *,
        heuristic_fn: Callable[[NodeName, NodeName], float] | None = None,
        move_cost_fn: Callable[[NodeName, NodeName], int] | None = None,
        heading_fn: Callable[[NodeName, NodeName], float] | None = None,
        heading_options_fn: (
            Callable[[NodeName, NodeName], tuple[float, ...]] | None
        ) = None,
        turn_cost_fn: Callable[[float, float], int] | None = None,
        low_level_max_time: int = 160,
        wait_cost: int = 6,
        max_planning_time_sec: float = 5.0,
    ) -> None:
        self.graph = graph
        self.heuristic_fn = heuristic_fn
        self.move_cost_fn = move_cost_fn
        self.heading_fn = heading_fn
        self.heading_options_fn = heading_options_fn
        self.turn_cost_fn = turn_cost_fn
        self.low_level_max_time = max(
            1,
            int(low_level_max_time),
        )
        self.wait_cost = max(1, int(wait_cost))
        self.max_planning_time_sec = max(
            0.0,
            float(max_planning_time_sec),
        )

        resource_writer = ResourceReservationWriter(graph)
        table_factory = ReservationTableFactory(
            graph,
            resource_writer,
        )
        path_writer = PathReservationWriter(
            resource_writer,
            low_level_max_time=self.low_level_max_time,
        )
        blocker_resolver = BlockerResolver(
            graph,
            low_level_max_time=self.low_level_max_time,
        )

        self._validator = RollingPlanValidator(
            graph,
            table_factory,
            path_writer,
            low_level_max_time=self.low_level_max_time,
        )
        self._blocker_resolver = blocker_resolver
        self._coordinator = PrioritizedSippCoordinator(
            graph,
            table_factory,
            path_writer,
            blocker_resolver,
            heuristic_fn=heuristic_fn,
            move_cost_fn=move_cost_fn,
            heading_fn=heading_fn,
            heading_options_fn=heading_options_fn,
            turn_cost_fn=turn_cost_fn,
            low_level_max_time=self.low_level_max_time,
            wait_cost=self.wait_cost,
            max_planning_time_sec=self.max_planning_time_sec,
        )

    def validate_plans(
        self,
        robot_requests: list[LmRobotRequest],
        plans: dict[str, LmRobotPlan],
        *,
        reserved_vertex_constraints: list[
            VertexConstraintInput
        ] | None = None,
        reserved_edge_constraints: list[
            EdgeConstraintInput
        ] | None = None,
        reserved_vertex_intervals: list[
            VertexIntervalInput
        ] | None = None,
        reserved_edge_intervals: list[
            EdgeIntervalInput
        ] | None = None,
    ) -> str:
        return self._validator.validate(
            robot_requests,
            plans,
            self._static_reservations(
                reserved_vertex_constraints,
                reserved_edge_constraints,
                reserved_vertex_intervals,
                reserved_edge_intervals,
            ),
        )

    def plan_for_robots(
        self,
        robot_requests: list[LmRobotRequest],
        *,
        blocked_nodes: list[NodeName] | None = None,
        blocked_edges: set[
            tuple[NodeName, NodeName]
        ] | None = None,
        reserved_vertex_constraints: list[
            VertexConstraintInput
        ] | None = None,
        reserved_edge_constraints: list[
            EdgeConstraintInput
        ] | None = None,
        reserved_vertex_intervals: list[
            VertexIntervalInput
        ] | None = None,
        reserved_edge_intervals: list[
            EdgeIntervalInput
        ] | None = None,
    ) -> PlannerResult:
        return self._coordinator.plan(
            robot_requests,
            blocked_nodes=set(blocked_nodes or []),
            blocked_edges=set(blocked_edges or set()),
            static=self._static_reservations(
                reserved_vertex_constraints,
                reserved_edge_constraints,
                reserved_vertex_intervals,
                reserved_edge_intervals,
            ),
        )

    def _blocking_plan_owners(
        self,
        failure: str,
        reservations: ReservationTable,
        *,
        authorized_controlled_regions: tuple[str, ...] = (),
    ) -> set[str]:
        """Compatibility proxy retained for one diagnostic caller."""

        return self._blocker_resolver.exact_owners(
            failure,
            reservations,
            authorized_controlled_regions=(
                authorized_controlled_regions
            ),
        )

    @staticmethod
    def _static_reservations(
        vertex_constraints: list[
            VertexConstraintInput
        ] | None,
        edge_constraints: list[
            EdgeConstraintInput
        ] | None,
        vertex_intervals: list[
            VertexIntervalInput
        ] | None,
        edge_intervals: list[
            EdgeIntervalInput
        ] | None,
    ) -> StaticReservations:
        return StaticReservations(
            vertex_constraints=tuple(
                vertex_constraints or ()
            ),
            edge_constraints=tuple(
                edge_constraints or ()
            ),
            vertex_intervals=tuple(
                vertex_intervals or ()
            ),
            edge_intervals=tuple(edge_intervals or ()),
        )
