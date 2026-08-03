"""Models shared by low-level and high-level CBS planning."""

from __future__ import annotations

from dataclasses import dataclass, field


NodeName = str


@dataclass(frozen=True)
class State:
    time: int
    node: NodeName
    yaw: float = 0.0

    def is_equal_except_time(self, other: State) -> bool:
        return self.node == other.node


@dataclass
class Conflict:
    VERTEX = 1
    EDGE = 2
    RESOURCE = 3

    time: int = -1
    end_time: int = -1
    type: int = -1
    agent_1: str = ""
    agent_2: str = ""
    node_1: NodeName | None = None
    node_2: NodeName | None = None
    agent_1_from: NodeName | None = None
    agent_1_to: NodeName | None = None
    agent_1_time: int = -1
    agent_2_from: NodeName | None = None
    agent_2_to: NodeName | None = None
    agent_2_time: int = -1
    agent_1_resource_kind: str = ""
    agent_2_resource_kind: str = ""
    agent_1_resource_start: int = -1
    agent_1_resource_end: int = -1
    agent_2_resource_start: int = -1
    agent_2_resource_end: int = -1
    agent_1_resource_entry: NodeName | None = None
    agent_2_resource_entry: NodeName | None = None
    resource: object | None = None


@dataclass(frozen=True)
class VertexConstraint:
    time: int
    node: NodeName


@dataclass(frozen=True)
class EdgeConstraint:
    time: int
    from_node: NodeName
    to_node: NodeName


@dataclass(frozen=True)
class VertexIntervalConstraint:
    start_time: int
    end_time: int
    node: NodeName
    owner: str = ""


@dataclass(frozen=True)
class EdgeIntervalConstraint:
    start_time: int
    end_time: int
    from_node: NodeName
    to_node: NodeName
    owner: str = ""


@dataclass(frozen=True)
class ResourceIntervalConstraint:
    start_time: int
    end_time: int
    resource: object


@dataclass(frozen=True)
class PathVertexInterval:
    agent: str
    start_time: int
    end_time: int
    node: NodeName


@dataclass(frozen=True)
class PathEdgeInterval:
    agent: str
    start_time: int
    end_time: int
    from_node: NodeName
    to_node: NodeName


@dataclass(frozen=True)
class PathResourceInterval:
    agent: str
    start_time: int
    end_time: int
    resource: object
    kind: str
    node: NodeName | None = None
    from_node: NodeName | None = None
    to_node: NodeName | None = None


@dataclass
class Constraints:
    vertex_constraints: set[VertexConstraint] = field(
        default_factory=set
    )
    edge_constraints: set[EdgeConstraint] = field(
        default_factory=set
    )
    vertex_interval_constraints: set[
        VertexIntervalConstraint
    ] = field(default_factory=set)
    edge_interval_constraints: set[
        EdgeIntervalConstraint
    ] = field(default_factory=set)
    resource_interval_constraints: set[
        ResourceIntervalConstraint
    ] = field(default_factory=set)

    def add_constraint(self, other: Constraints) -> None:
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints
        self.vertex_interval_constraints |= (
            other.vertex_interval_constraints
        )
        self.edge_interval_constraints |= (
            other.edge_interval_constraints
        )
        self.resource_interval_constraints |= (
            other.resource_interval_constraints
        )


@dataclass(frozen=True)
class LmRobotRequest:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    start_yaw: float = 0.0
    route_nodes: tuple[NodeName, ...] = ()
    start_not_before_tick: int = 0
    node_departure_not_before: tuple[
        tuple[NodeName, int],
        ...,
    ] = ()
    authorized_controlled_regions: tuple[str, ...] = ()
    no_wait_nodes: tuple[NodeName, ...] = ()


@dataclass
class LmRobotPlan:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    nodes: list[NodeName]
    times: list[int] = field(default_factory=list)
    yaws: list[float] = field(default_factory=list)
    actions: list[str] = field(default_factory=list)


@dataclass
class PlannerDebug:
    reason: str = "unknown"
    conflicts_resolved: int = 0
    high_level_nodes: int = 0
    expanded_nodes: int = 0
    blocking_robots: tuple[str, ...] = ()
    blocking_reservations: tuple[tuple[str, str], ...] = ()


@dataclass
class PlannerResult:
    plans: dict[str, LmRobotPlan]
    debug: PlannerDebug


@dataclass
class HighLevelNode:
    solution: dict[str, list[State]] = field(
        default_factory=dict
    )
    constraint_dict: dict[str, Constraints] = field(
        default_factory=dict
    )
    cost: int = 0

    def __hash__(self) -> int:
        frozen_solution = tuple(
            (
                name,
                tuple(
                    (state.time, state.node, state.yaw)
                    for state in path
                ),
            )
            for name, path in sorted(self.solution.items())
        )
        frozen_constraints = tuple(
            (
                name,
                tuple(
                    sorted(
                        (item.time, item.node)
                        for item
                        in constraints.vertex_constraints
                    )
                ),
                tuple(
                    sorted(
                        (
                            item.time,
                            item.from_node,
                            item.to_node,
                        )
                        for item
                        in constraints.edge_constraints
                    )
                ),
                tuple(
                    sorted(
                        (
                            item.start_time,
                            item.end_time,
                            item.node,
                        )
                        for item
                        in constraints.vertex_interval_constraints
                    )
                ),
                tuple(
                    sorted(
                        (
                            item.start_time,
                            item.end_time,
                            item.from_node,
                            item.to_node,
                        )
                        for item
                        in constraints.edge_interval_constraints
                    )
                ),
                tuple(
                    sorted(
                        (
                            item.start_time,
                            item.end_time,
                            str(item.resource),
                        )
                        for item
                        in constraints.resource_interval_constraints
                    )
                ),
            )
            for name, constraints
            in sorted(self.constraint_dict.items())
        )
        return hash((frozen_solution, frozen_constraints))

    def __lt__(self, other: HighLevelNode) -> bool:
        return self.cost < other.cost


__all__ = [
    "Conflict",
    "Constraints",
    "EdgeConstraint",
    "EdgeIntervalConstraint",
    "HighLevelNode",
    "LmRobotPlan",
    "LmRobotRequest",
    "NodeName",
    "PathEdgeInterval",
    "PathResourceInterval",
    "PathVertexInterval",
    "PlannerDebug",
    "PlannerResult",
    "ResourceIntervalConstraint",
    "State",
    "VertexConstraint",
    "VertexIntervalConstraint",
]
