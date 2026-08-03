"""Immutable request, state and path models for SIPP planning."""

from __future__ import annotations

from dataclasses import dataclass


NodeName = str


@dataclass(frozen=True, slots=True)
class SippRobotRequest:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    start_yaw: float = 0.0
    route_nodes: tuple[NodeName, ...] = ()
    initial_departure_not_before: int = 0
    node_departure_not_before: tuple[tuple[NodeName, int], ...] = ()
    authorized_controlled_regions: tuple[str, ...] = ()
    no_wait_nodes: tuple[NodeName, ...] = ()


@dataclass(frozen=True, slots=True)
class TimedState:
    time: int
    node: NodeName
    yaw: float = 0.0
    action: str = "wait"


@dataclass(frozen=True, slots=True)
class SippState:
    time: int
    node: NodeName
    interval_start: int
    interval_end: int
    yaw: float = 0.0

    @property
    def key(self) -> tuple[NodeName, int, int, int]:
        """Identity used for SIPP dominance.

        Arrival time is deliberately excluded: within one safe interval and
        heading, an earlier arrival dominates a later arrival.
        """

        return (
            self.node,
            self.interval_start,
            self.interval_end,
            int(round(self.yaw * 1_000_000)),
        )


@dataclass(frozen=True, slots=True)
class TimedPath:
    robot_name: str
    start_lm: NodeName
    goal_lm: NodeName
    states: tuple[TimedState, ...]

    @property
    def nodes(self) -> list[NodeName]:
        return [state.node for state in self.states]

    @property
    def times(self) -> list[int]:
        return [state.time for state in self.states]

    @property
    def yaws(self) -> list[float]:
        return [state.yaw for state in self.states]

    @property
    def actions(self) -> list[str]:
        return [state.action for state in self.states]
