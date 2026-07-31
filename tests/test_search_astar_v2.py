from __future__ import annotations

from dataclasses import dataclass, FrozenInstanceError
import math
from typing import Iterable

import pytest

from fleet_manager.search import AStarSolver, SearchProblem, SearchResult


@dataclass
class GraphProblem:
    start_state: str
    goals: set[str]
    graph: dict[str, tuple[tuple[str, float], ...]]
    estimates: dict[str, float]

    def is_goal(self, state: str) -> bool:
        return state in self.goals

    def neighbors(self, state: str) -> Iterable[tuple[str, float]]:
        return self.graph.get(state, ())

    def heuristic(self, state: str) -> float:
        return self.estimates.get(state, 0.0)


def test_astar_finds_the_lowest_cost_path() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={
            "start": (("slow", 2), ("fast", 1)),
            "slow": (("goal", 5),),
            "fast": (("middle", 1),),
            "middle": (("goal", 1),),
        },
        estimates={
            "start": 3,
            "slow": 5,
            "fast": 2,
            "middle": 1,
            "goal": 0,
        },
    )

    result = AStarSolver[str]().solve(problem)

    assert result.found
    assert result.path == ("start", "fast", "middle", "goal")
    assert result.cost == pytest.approx(3)
    assert result.expanded_count == 4
    assert result.failure_reason is None


def test_unreachable_goal_is_an_explicit_result() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={"start": (("dead_end", 1),)},
        estimates={},
    )

    result = AStarSolver[str]().solve(problem)

    assert not result.found
    assert result.path == ()
    assert result.cost == math.inf
    assert result.expanded_count == 2
    assert result.failure_reason == "unreachable"


def test_start_equal_to_goal_returns_a_single_state_path() -> None:
    problem = GraphProblem(
        start_state="same",
        goals={"same"},
        graph={},
        estimates={"same": 0},
    )

    result = AStarSolver[str]().solve(problem)

    assert result == SearchResult.success(
        ("same",),
        cost=0,
        expanded_count=1,
    )


def test_equal_priorities_follow_neighbor_order_deterministically() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"left_goal", "right_goal"},
        graph={
            "start": (("left", 1), ("right", 1)),
            "left": (("left_goal", 1),),
            "right": (("right_goal", 1),),
        },
        estimates={
            "start": 2,
            "left": 1,
            "right": 1,
            "left_goal": 0,
            "right_goal": 0,
        },
    )

    paths = {
        AStarSolver[str]().solve(problem).path
        for _ in range(20)
    }

    assert paths == {("start", "left", "left_goal")}


def test_a_state_is_reopened_when_a_better_path_is_found() -> None:
    # "a" is expanded first at cost 3.  Going through "b" reaches it later
    # at cost 2; supporting that improvement is essential for inconsistent
    # (but still admissible) heuristics.
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={
            "start": (("a", 3), ("b", 1)),
            "a": (("goal", 1),),
            "b": (("a", 1),),
        },
        estimates={
            "start": 3,
            "a": 0,
            "b": 2,
            "goal": 0,
        },
    )

    result = AStarSolver[str]().solve(problem)

    assert result.path == ("start", "b", "a", "goal")
    assert result.cost == pytest.approx(3)
    assert result.expanded_count == 5


def test_stale_more_expensive_queue_entries_are_ignored() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={
            "start": (("node", 10), ("shortcut", 1)),
            "shortcut": (("node", 1),),
            "node": (("goal", 1),),
        },
        estimates={},
    )

    result = AStarSolver[str]().solve(problem)

    assert result.path == ("start", "shortcut", "node", "goal")
    assert result.cost == pytest.approx(3)
    assert result.expanded_count == 4


def test_optional_problem_key_controls_state_dominance() -> None:
    @dataclass(frozen=True)
    class VersionedState:
        name: str
        arrival: int

    start = VersionedState("start", 0)
    branch = VersionedState("branch", 1)
    late = VersionedState("shared", 4)
    early = VersionedState("shared", 2)

    class KeyedProblem:
        start_state = start

        def key(self, state: VersionedState) -> str:
            return state.name

        def is_goal(self, state: VersionedState) -> bool:
            return False

        def neighbors(
            self,
            state: VersionedState,
        ) -> Iterable[tuple[VersionedState, float]]:
            if state == start:
                return ((late, 4), (branch, 1))
            if state == branch:
                return ((early, 1),)
            return ()

        def heuristic(self, state: VersionedState) -> float:
            return 0.0

    result = AStarSolver[VersionedState]().solve(KeyedProblem())

    assert not result.found
    # The late and early objects share one search key.  Only the cheaper,
    # earlier version is expanded.
    assert result.expanded_count == 3


def test_problem_key_keeps_winning_full_state_in_the_result_path() -> None:
    @dataclass(frozen=True)
    class TimedNode:
        node: str
        time: int

    start = TimedNode("start", 0)
    detour = TimedNode("detour", 1)
    late_shared = TimedNode("shared", 8)
    early_shared = TimedNode("shared", 2)
    goal = TimedNode("goal", 3)

    class KeyedProblem:
        start_state = start

        def key(self, state: TimedNode) -> str:
            return state.node

        def is_goal(self, state: TimedNode) -> bool:
            return state.node == "goal"

        def neighbors(
            self,
            state: TimedNode,
        ) -> Iterable[tuple[TimedNode, float]]:
            if state == start:
                return ((late_shared, 8), (detour, 1))
            if state == detour:
                return ((early_shared, 1),)
            if state.node == "shared":
                return ((goal, 1),)
            return ()

        def heuristic(self, state: TimedNode) -> float:
            return 0.0

    result = AStarSolver[TimedNode]().solve(KeyedProblem())

    assert result.path == (start, detour, early_shared, goal)
    assert result.cost == pytest.approx(3)


def test_optional_dominance_can_prefer_an_earlier_temporal_state() -> None:
    @dataclass(frozen=True)
    class TemporalState:
        node: str
        arrival: int

    start = TemporalState("start", 0)
    early = TemporalState("shared", 2)
    branch = TemporalState("branch", 1)
    late_but_cheaper = TemporalState("shared", 8)
    goal = TemporalState("goal", 3)

    class TemporalProblem:
        start_state = start

        def key(self, state: TemporalState) -> str:
            return state.node

        def dominance(
            self,
            state: TemporalState,
            path_cost: float,
        ) -> float:
            return float(state.arrival)

        def is_goal(self, state: TemporalState) -> bool:
            return state.node == "goal"

        def neighbors(
            self,
            state: TemporalState,
        ) -> Iterable[tuple[TemporalState, float]]:
            if state == start:
                return ((early, 5), (branch, 1))
            if state == early:
                return ((goal, 1),)
            if state == branch:
                return ((late_but_cheaper, 2),)
            return ()

        def heuristic(self, state: TemporalState) -> float:
            return 5.0 if state == branch else 0.0

    result = AStarSolver[TemporalState]().solve(TemporalProblem())

    assert result.path == (start, early, goal)
    assert result.cost == pytest.approx(6)


def test_cancellation_callback_returns_the_requested_failure_reason() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={"start": (("goal", 1),)},
        estimates={},
    )
    callback_calls = 0

    def deadline_reached() -> bool:
        nonlocal callback_calls
        callback_calls += 1
        return callback_calls >= 2

    result = AStarSolver[str]().solve(
        problem,
        should_cancel=deadline_reached,
        cancellation_reason="planning_timeout:robot",
    )

    assert not result.found
    assert result.failure_reason == "planning_timeout:robot"
    assert result.expanded_count == 1


def test_immediate_cancellation_does_not_expand_the_start() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"start"},
        graph={},
        estimates={},
    )

    result = AStarSolver[str]().solve(
        problem,
        should_cancel=lambda: True,
        cancellation_reason="deadline",
    )

    assert result.failure_reason == "deadline"
    assert result.expanded_count == 0


def test_cancellation_reason_must_be_non_empty() -> None:
    problem = GraphProblem(
        start_state="start",
        goals=set(),
        graph={},
        estimates={},
    )

    with pytest.raises(ValueError, match="non-empty"):
        AStarSolver[str]().solve(
            problem,
            should_cancel=lambda: False,
            cancellation_reason="",
        )


def test_problem_key_must_return_a_hashable_value() -> None:
    class InvalidKeyProblem(GraphProblem):
        def key(self, state: str) -> list[str]:
            return [state]

    problem = InvalidKeyProblem(
        start_state="start",
        goals=set(),
        graph={},
        estimates={},
    )

    with pytest.raises(TypeError, match="start_state key must be hashable"):
        AStarSolver[str]().solve(problem)


def test_expansion_limit_returns_a_failure_result() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={
            "start": (("middle", 1),),
            "middle": (("goal", 1),),
        },
        estimates={},
    )

    result = AStarSolver[str](max_expansions=1).solve(problem)

    assert not result.found
    assert result.failure_reason == "expansion_limit"
    assert result.expanded_count == 1


@pytest.mark.parametrize("limit", [0, -1])
def test_expansion_limit_must_be_positive(limit: int) -> None:
    with pytest.raises(ValueError, match="at least 1"):
        AStarSolver[str](max_expansions=limit)


@pytest.mark.parametrize("limit", [True, 1.5])
def test_expansion_limit_must_be_an_integer(limit: object) -> None:
    with pytest.raises(TypeError, match="int or None"):
        AStarSolver[str](max_expansions=limit)  # type: ignore[arg-type]


@pytest.mark.parametrize("cost", [-1, math.inf, math.nan])
def test_astar_rejects_invalid_step_costs(cost: float) -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={"start": (("goal", cost),)},
        estimates={},
    )

    with pytest.raises(ValueError, match="step cost"):
        AStarSolver[str]().solve(problem)


def test_astar_rejects_non_numeric_step_cost() -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={"start": (("goal", "one"),)},  # type: ignore[dict-item]
        estimates={},
    )

    with pytest.raises(TypeError, match="step cost"):
        AStarSolver[str]().solve(problem)


@pytest.mark.parametrize("estimate", [-1, math.inf, math.nan])
def test_astar_rejects_invalid_heuristics(estimate: float) -> None:
    problem = GraphProblem(
        start_state="start",
        goals={"goal"},
        graph={},
        estimates={"start": estimate},
    )

    with pytest.raises(ValueError, match="heuristic"):
        AStarSolver[str]().solve(problem)


def test_astar_rejects_unhashable_start_state() -> None:
    class UnhashableProblem:
        start_state = ["start"]

        def is_goal(self, state: object) -> bool:
            return False

        def neighbors(self, state: object) -> tuple[()]:
            return ()

        def heuristic(self, state: object) -> float:
            return 0.0

    with pytest.raises(TypeError, match="start_state must be hashable"):
        AStarSolver().solve(UnhashableProblem())  # type: ignore[arg-type]


def test_astar_rejects_unhashable_neighbor_state() -> None:
    class UnhashableNeighborProblem:
        start_state = "start"

        def is_goal(self, state: object) -> bool:
            return False

        def neighbors(self, state: object) -> Iterable[tuple[object, float]]:
            return [(["bad"], 1)]

        def heuristic(self, state: object) -> float:
            return 0.0

    with pytest.raises(TypeError, match="neighbor state must be hashable"):
        AStarSolver().solve(UnhashableNeighborProblem())  # type: ignore[arg-type]


def test_astar_rejects_malformed_neighbor_records() -> None:
    class MalformedProblem:
        start_state = "start"

        def is_goal(self, state: str) -> bool:
            return False

        def neighbors(self, state: str) -> Iterable[tuple[str, float]]:
            return [("missing-cost",)]  # type: ignore[list-item]

        def heuristic(self, state: str) -> float:
            return 0.0

    with pytest.raises(TypeError, match="pairs"):
        AStarSolver[str]().solve(MalformedProblem())


def test_search_problem_is_runtime_checkable() -> None:
    problem = GraphProblem(
        start_state="start",
        goals=set(),
        graph={},
        estimates={},
    )

    assert isinstance(problem, SearchProblem)


def test_search_result_is_immutable_and_validates_invariants() -> None:
    result = SearchResult.success(("start", "goal"), cost=1, expanded_count=2)

    with pytest.raises(FrozenInstanceError):
        result.cost = 2  # type: ignore[misc]
    with pytest.raises(ValueError, match="contain a path"):
        SearchResult(
            found=True,
            path=(),
            cost=0,
            expanded_count=0,
        )
    with pytest.raises(ValueError, match="cannot contain a path"):
        SearchResult(
            found=False,
            path=("unexpected",),
            cost=math.inf,
            expanded_count=0,
            failure_reason="failed",
        )
    with pytest.raises(ValueError, match="positive infinity"):
        SearchResult(
            found=False,
            path=(),
            cost=0,
            expanded_count=0,
            failure_reason="failed",
        )
    with pytest.raises(ValueError, match="failure reason"):
        SearchResult(
            found=False,
            path=(),
            cost=math.inf,
            expanded_count=0,
        )
    with pytest.raises(ValueError, match="must not be negative"):
        SearchResult.success(("start",), cost=0, expanded_count=-1)
    with pytest.raises(ValueError, match="finite and non-negative"):
        SearchResult.success(("start",), cost=math.nan, expanded_count=0)
    with pytest.raises(TypeError, match="real number"):
        SearchResult.success(
            ("start",),
            cost="free",  # type: ignore[arg-type]
            expanded_count=0,
        )
    with pytest.raises(TypeError, match="path must be a tuple"):
        SearchResult(
            found=True,
            path=["start"],  # type: ignore[arg-type]
            cost=0,
            expanded_count=0,
        )
    with pytest.raises(TypeError, match="expanded_count must be an int"):
        SearchResult.success(("start",), cost=0, expanded_count=True)
