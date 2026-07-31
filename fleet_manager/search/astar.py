"""A small, deterministic A* implementation."""

from __future__ import annotations

from collections.abc import Callable, Hashable, Iterator
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
import math
from numbers import Real
from typing import Generic, TypeVar

from .problem import SearchProblem
from .result import SearchResult


StateT = TypeVar("StateT", bound=Hashable)


@dataclass(frozen=True, slots=True)
class _SearchCallbacks(Generic[StateT]):
    """Optional problem policies validated before the search starts."""

    key: Callable[[StateT], Hashable] | None
    dominance: Callable[[StateT, float], Real] | None
    should_cancel: Callable[[], bool] | None
    cancellation_reason: str


@dataclass(slots=True)
class _SearchFrontier(Generic[StateT]):
    """Mutable A* state shared by the expansion stages."""

    open_states: list[tuple[float, int, float, StateT, Hashable]]
    sequence: Iterator[int]
    best_cost: dict[Hashable, float]
    parent: dict[Hashable, Hashable]
    state_by_key: dict[Hashable, StateT]
    expanded_at_dominance: dict[Hashable, float]
    expanded_count: int = 0


class AStarSolver(Generic[StateT]):
    """Find a least-cost path through a :class:`SearchProblem`.

    Equal-priority states are expanded in the order in which ``neighbors``
    yields them.  A state is reopened when a cheaper path reaches it, so
    admissible but inconsistent heuristics remain supported.
    """

    def __init__(self, *, max_expansions: int | None = None) -> None:
        if max_expansions is not None:
            if (
                isinstance(max_expansions, bool)
                or not isinstance(max_expansions, int)
            ):
                raise TypeError("max_expansions must be an int or None")
            if max_expansions < 1:
                raise ValueError("max_expansions must be at least 1")
        self.max_expansions = max_expansions

    def solve(
        self,
        problem: SearchProblem[StateT],
        *,
        should_cancel: Callable[[], bool] | None = None,
        cancellation_reason: str = "cancelled",
    ) -> SearchResult[StateT]:
        """Solve ``problem`` or return an explicit failure.

        ``should_cancel`` is checked before each state expansion and once
        before reporting an unreachable goal.  A deadline can therefore be
        represented without coupling this generic solver to a clock.
        """
        callbacks = self._validated_search_callbacks(
            problem,
            should_cancel=should_cancel,
            cancellation_reason=cancellation_reason,
        )
        frontier = self._initial_search_frontier(problem, callbacks)
        return self._run_search(problem, callbacks, frontier)

    @staticmethod
    def _validated_search_callbacks(
        problem: SearchProblem[StateT],
        *,
        should_cancel: Callable[[], bool] | None,
        cancellation_reason: str,
    ) -> _SearchCallbacks[StateT]:
        """Validate optional callbacks without changing legacy check order."""
        if should_cancel is not None:
            if not callable(should_cancel):
                raise TypeError("should_cancel must be callable or None")
            if not isinstance(cancellation_reason, str) or not cancellation_reason:
                raise ValueError(
                    "cancellation_reason must be a non-empty string"
                )

        problem_key = getattr(problem, "key", None)
        if problem_key is not None and not callable(problem_key):
            raise TypeError("problem.key must be callable")
        problem_dominance = getattr(problem, "dominance", None)
        if (
            problem_dominance is not None
            and not callable(problem_dominance)
        ):
            raise TypeError("problem.dominance must be callable")
        return _SearchCallbacks(
            key=problem_key,
            dominance=problem_dominance,
            should_cancel=should_cancel,
            cancellation_reason=cancellation_reason,
        )

    def _initial_search_frontier(
        self,
        problem: SearchProblem[StateT],
        callbacks: _SearchCallbacks[StateT],
    ) -> _SearchFrontier[StateT]:
        """Validate the start and seed the deterministic heap."""
        start = problem.start_state
        start_key = (
            callbacks.key(start)
            if callbacks.key is not None
            else start
        )
        self._validate_hashable(
            start_key,
            name=(
                "start_state key"
                if callbacks.key is not None
                else "start_state"
            ),
        )
        start_heuristic = self._validate_score(
            problem.heuristic(start),
            name="heuristic",
        )

        sequence = count()
        open_states: list[
            tuple[float, int, float, StateT, Hashable]
        ] = []
        heappush(
            open_states,
            (
                start_heuristic,
                next(sequence),
                0.0,
                start,
                start_key,
            ),
        )
        return _SearchFrontier(
            open_states=open_states,
            sequence=sequence,
            best_cost={start_key: 0.0},
            parent={},
            state_by_key={start_key: start},
            expanded_at_dominance={},
        )

    def _run_search(
        self,
        problem: SearchProblem[StateT],
        callbacks: _SearchCallbacks[StateT],
        frontier: _SearchFrontier[StateT],
    ) -> SearchResult[StateT]:
        """Expand the frontier until a goal or terminal condition is reached."""
        open_states = frontier.open_states
        best_cost = frontier.best_cost
        parent = frontier.parent
        state_by_key = frontier.state_by_key
        expanded_at_dominance = frontier.expanded_at_dominance
        problem_dominance = callbacks.dominance
        should_cancel = callbacks.should_cancel
        cancellation_reason = callbacks.cancellation_reason
        validate_score = self._validate_score
        expand_neighbors = self._expand_search_neighbors
        expanded_count = frontier.expanded_count
        while open_states:
            if should_cancel is not None and should_cancel():
                frontier.expanded_count = expanded_count
                return SearchResult.failure(
                    reason=cancellation_reason,
                    expanded_count=expanded_count,
                )

            _, _, current_cost, current, current_key = heappop(open_states)
            current_best_cost = best_cost.get(
                current_key,
                math.inf,
            )
            current_dominance = validate_score(
                (
                    problem_dominance(current, current_cost)
                    if problem_dominance is not None
                    else current_cost
                ),
                name="dominance",
            )
            if current_dominance > current_best_cost:
                continue

            previous_dominance = expanded_at_dominance.get(
                current_key
            )
            if (
                previous_dominance is not None
                and current_dominance >= previous_dominance
            ):
                continue

            if (
                self.max_expansions is not None
                and expanded_count >= self.max_expansions
            ):
                frontier.expanded_count = expanded_count
                return SearchResult.failure(
                    reason="expansion_limit",
                    expanded_count=expanded_count,
                )

            expanded_at_dominance[current_key] = current_dominance
            expanded_count += 1

            if problem.is_goal(current):
                frontier.expanded_count = expanded_count
                return SearchResult.success(
                    self._reconstruct_path(
                        parent,
                        state_by_key,
                        current_key,
                    ),
                    cost=current_best_cost,
                    expanded_count=expanded_count,
                )

            expand_neighbors(
                problem,
                callbacks,
                frontier,
                current=current,
                current_key=current_key,
                current_best_cost=current_best_cost,
            )

        if should_cancel is not None and should_cancel():
            frontier.expanded_count = expanded_count
            return SearchResult.failure(
                reason=cancellation_reason,
                expanded_count=expanded_count,
            )
        frontier.expanded_count = expanded_count
        return SearchResult.failure(
            reason="unreachable",
            expanded_count=expanded_count,
        )

    def _expand_search_neighbors(
        self,
        problem: SearchProblem[StateT],
        callbacks: _SearchCallbacks[StateT],
        frontier: _SearchFrontier[StateT],
        *,
        current: StateT,
        current_key: Hashable,
        current_best_cost: float,
    ) -> None:
        """Relax one state's outgoing transitions into the shared frontier."""
        problem_key = callbacks.key
        problem_dominance = callbacks.dominance
        expanded_at_dominance = frontier.expanded_at_dominance
        best_cost = frontier.best_cost
        validate_score = self._validate_score
        for transition in problem.neighbors(current):
            try:
                neighbor, raw_step_cost = transition
            except (TypeError, ValueError) as error:
                raise TypeError(
                    "neighbors must yield (state, step_cost) pairs"
                ) from error

            neighbor_key = (
                problem_key(neighbor)
                if problem_key is not None
                else neighbor
            )
            step_cost = validate_score(raw_step_cost, name="step cost")
            candidate_cost = current_best_cost + step_cost
            neighbor_dominance = validate_score(
                (
                    problem_dominance(neighbor, candidate_cost)
                    if problem_dominance is not None
                    else candidate_cost
                ),
                name="dominance",
            )
            try:
                previous_dominance = expanded_at_dominance.get(neighbor_key)
                known_cost = best_cost.get(neighbor_key, math.inf)
            except (TypeError, ValueError) as error:
                key_name = (
                    "neighbor state key"
                    if problem_key is not None
                    else "neighbor state"
                )
                raise TypeError(f"{key_name} must be hashable") from error
            if (
                previous_dominance is not None
                and previous_dominance <= neighbor_dominance
            ):
                continue
            if candidate_cost >= known_cost:
                continue

            heuristic = validate_score(
                problem.heuristic(neighbor),
                name="heuristic",
            )
            best_cost[neighbor_key] = candidate_cost
            frontier.parent[neighbor_key] = current_key
            frontier.state_by_key[neighbor_key] = neighbor
            heappush(
                frontier.open_states,
                (
                    candidate_cost + heuristic,
                    next(frontier.sequence),
                    candidate_cost,
                    neighbor,
                    neighbor_key,
                ),
            )

    @staticmethod
    def _validate_hashable(value: object, *, name: str) -> None:
        try:
            hash(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be hashable") from error

    @staticmethod
    def _validate_score(value: Real, *, name: str) -> float:
        value_type = type(value)
        if value_type is float:
            result = value
        elif value_type is int:
            result = float(value)
        elif isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"{name} must be a real number")
        else:
            result = float(value)
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        if result < 0.0:
            raise ValueError(f"{name} must not be negative")
        return result

    @staticmethod
    def _reconstruct_path(
        parent: dict[Hashable, Hashable],
        state_by_key: dict[Hashable, StateT],
        goal_key: Hashable,
    ) -> tuple[StateT, ...]:
        reversed_keys = [goal_key]
        while reversed_keys[-1] in parent:
            reversed_keys.append(parent[reversed_keys[-1]])
        return tuple(
            state_by_key[key]
            for key in reversed(reversed_keys)
        )
