from __future__ import annotations

from fleet_manager.core.mapf.cbs.cbs_models import (
    LmRobotPlan,
    LmRobotRequest,
)
from fleet_manager.core.mapf.rolling.prioritized_planning import (
    BlockerResolver,
    PriorityOrderManager,
    StagingWaitRepairPolicy,
)
from fleet_manager.core.mapf.common.reservations import (
    ReservationInterval,
    ReservationTable,
    ResourceId,
)
from fleet_manager.core.mapf.rolling.rolling_models import (
    StaticReservations,
)
from fleet_manager.core.mapf.rolling.rolling_reservations import (
    PathReservationWriter,
    ReservationTableFactory,
    ResourceReservationWriter,
    RollingPlanValidator,
)
from fleet_manager.core.mapf.sipp.sipp_models import TimedPath, TimedState
from fleet_manager.core.mapf.graph.traffic_graph_models import TrafficGraph
from fleet_manager.core.mapping.maps.models import (
    GraphEdge,
    Landmark,
    WorldPoint,
)


def _graph(
    *edge_names: tuple[str, str],
) -> TrafficGraph:
    node_names = sorted(
        {
            node
            for edge_name in edge_names
            for node in edge_name
        }
    )
    landmarks = {
        name: Landmark(
            name,
            float(index),
            0.0,
        )
        for index, name in enumerate(node_names)
    }
    edges = [
        GraphEdge(
            from_name=source,
            to_name=target,
            length=1.0,
            kind="line",
            edge_type="line",
            world_points=(
                WorldPoint(
                    landmarks[source].x,
                    landmarks[source].y,
                ),
                WorldPoint(
                    landmarks[target].x,
                    landmarks[target].y,
                ),
            ),
        )
        for source, target in edge_names
    ]
    return TrafficGraph.from_route_core(
        landmarks,
        edges,
        default_speed_mps=1.0,
    )


def _reservation_components(
    graph: TrafficGraph,
    *,
    low_level_max_time: int = 5,
) -> tuple[
    ResourceReservationWriter,
    ReservationTableFactory,
    PathReservationWriter,
]:
    resources = ResourceReservationWriter(graph)
    factory = ReservationTableFactory(graph, resources)
    paths = PathReservationWriter(
        resources,
        low_level_max_time=low_level_max_time,
    )
    return resources, factory, paths


def test_table_factory_writes_static_and_initial_reservations() -> None:
    graph = _graph(("A", "B"))
    _, factory, _ = _reservation_components(graph)
    request = LmRobotRequest("robot", "A", "B")
    static = StaticReservations(
        vertex_constraints=((2, "B"),),
        edge_intervals=((3, 4, "A", "B", "owner"),),
    )

    table = factory.create([request], static)

    initial = table.intervals_for_resource(
        ResourceId("vertex", "A")
    )
    assert any(
        (
            interval.robot_name,
            interval.start,
            interval.end,
            interval.reason,
        )
        == ("robot", 0, 1, "initial_position")
        for interval in initial
    )
    blocked_vertex = table.intervals_for_resource(
        ResourceId("vertex", "B")
    )
    assert any(
        (
            interval.robot_name,
            interval.start,
            interval.end,
            interval.reason,
        )
        == ("reserved", 2, 3, "constraint")
        for interval in blocked_vertex
    )
    lane = graph.lane_for("A", "B")
    assert lane is not None
    for resource in graph.lane_resources(lane):
        assert any(
            (
                interval.robot_name,
                interval.start,
                interval.end,
            )
            == ("owner", 3, 5)
            for interval in table.intervals_for_resource(
                resource
            )
        )


def test_path_writer_reserves_wait_move_and_final_occupancy() -> None:
    graph = _graph(("A", "B"))
    _, _, writer = _reservation_components(
        graph,
        low_level_max_time=5,
    )
    table = ReservationTable(graph.reservation_capacities())
    path = TimedPath(
        robot_name="robot",
        start_lm="A",
        goal_lm="B",
        states=(
            TimedState(0, "A", action="start"),
            TimedState(2, "A", action="wait"),
            TimedState(3, "B", action="move"),
        ),
    )

    writer.reserve(table, path)

    source_intervals = table.intervals_for_resource(
        ResourceId("vertex", "A")
    )
    assert any(
        (
            interval.start,
            interval.end,
            interval.reason,
            interval.committed,
        )
        == (0, 3, "wait", False)
        for interval in source_intervals
    )
    target_intervals = table.intervals_for_resource(
        ResourceId("vertex", "B")
    )
    assert any(
        (
            interval.start,
            interval.end,
            interval.reason,
            interval.committed,
        )
        == (3, 6, "goal", False)
        for interval in target_intervals
    )
    lane = graph.lane_for("A", "B")
    assert lane is not None
    for resource in graph.lane_resources(lane):
        assert any(
            (
                interval.start,
                interval.end,
                interval.reason,
                interval.committed,
            )
            == (2, 3, "move", False)
            for interval in table.intervals_for_resource(
                resource
            )
        )


def test_plan_validator_reports_missing_and_conflicting_plans() -> None:
    graph = _graph(("A", "B"))
    _, factory, paths = _reservation_components(graph)
    validator = RollingPlanValidator(
        graph,
        factory,
        paths,
        low_level_max_time=5,
    )
    request = LmRobotRequest("robot", "A", "B")
    plan = LmRobotPlan(
        robot_name="robot",
        start_lm="A",
        goal_lm="B",
        nodes=["A", "B"],
        times=[0, 1],
        yaws=[0.0, 0.0],
        actions=["start", "move"],
    )

    assert validator.validate([request], {}, StaticReservations()) == (
        "missing_plan:robot"
    )
    assert validator.validate(
        [request],
        {"robot": plan},
        StaticReservations(),
    ) == ""
    assert validator.validate(
        [request],
        {"robot": plan},
        StaticReservations(vertex_constraints=((1, "B"),)),
    ) == "resource_conflict:robot"


def test_priority_order_manager_promotes_and_rejects_a_cycle() -> None:
    first = LmRobotRequest("first", "A", "B")
    second = LmRobotRequest("second", "C", "D")
    orders = PriorityOrderManager([first, second])

    promoted = orders.proposed_order(
        "second",
        {"first"},
    )

    assert [request.robot_name for request in promoted] == [
        "second",
        "first",
    ]
    assert orders.try_apply(promoted, {"first"})
    assert orders.repairs == 1
    assert orders.order_count == 2

    cycle = orders.proposed_order("first", {"second"})
    assert [request.robot_name for request in cycle] == [
        "first",
        "second",
    ]
    assert orders.has_seen(cycle)
    assert not orders.try_apply(cycle, {"second"})


def test_staging_wait_policy_moves_delay_to_a_safe_start() -> None:
    graph = _graph(("A", "B"))
    policy = StagingWaitRepairPolicy(
        graph,
        low_level_max_time=12,
    )
    request = LmRobotRequest("robot", "A", "B")

    assert policy.next_departure(
        request,
        "cannot_wait:B@3-7",
        current_departure=2,
        attempts=0,
    ) == 6
    assert policy.next_departure(
        request,
        "reserved_edge:A->B@0-3",
        current_departure=2,
        attempts=0,
    ) is None
    assert policy.next_departure(
        request,
        "cannot_wait:B@3-7",
        current_departure=2,
        attempts=16,
    ) is None


def test_blocker_resolver_respects_controlled_region_authority() -> None:
    region = "corridor:test"
    landmarks = {
        "A": Landmark("A", 0.0, 0.0),
        "B": Landmark("B", 1.0, 0.0),
    }
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="line",
        world_points=(
            WorldPoint(0.0, 0.0),
            WorldPoint(1.0, 0.0),
        ),
        properties={"controlled_region": region},
    )
    graph = TrafficGraph.from_route_core(
        landmarks,
        [edge],
        default_speed_mps=1.0,
    )
    table = ReservationTable()
    table.reserve(
        ReservationInterval(
            ResourceId("controlled_region", region),
            "authorized-owner",
            0,
            3,
        )
    )
    table.reserve(
        ReservationInterval(
            ResourceId("lane", "A->B"),
            "real-blocker",
            0,
            3,
        )
    )
    resolver = BlockerResolver(
        graph,
        low_level_max_time=5,
    )

    owners = resolver.exact_owners(
        "reserved_edge:A->B@0-2",
        table,
        authorized_controlled_regions=(region,),
    )

    assert owners == {"real-blocker"}
