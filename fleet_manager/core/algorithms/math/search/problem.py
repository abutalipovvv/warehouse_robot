"""The minimal problem contract required by graph-search algorithms."""

from __future__ import annotations

from collections.abc import Hashable, Iterable
from typing import Protocol, TypeVar, runtime_checkable


StateT = TypeVar("StateT", bound=Hashable)


@runtime_checkable
class SearchProblem(Protocol[StateT]):
    """A state space consumed by :class:`AStarSolver`.

    ``neighbors`` yields ``(next_state, step_cost)`` pairs.  Costs and
    heuristics must be finite and non-negative.

    A problem may additionally define ``key(state)``.  The solver then uses
    that hashable value for cost dominance and parent tracking while retaining
    the full state objects in the returned path.  Without it, the state itself
    is the key.

    Temporal searches may also define ``dominance(state, path_cost)``.  Its
    finite, non-negative result decides whether a version of the same key was
    already expanded in a better form.  The default is ``path_cost``.

    A problem may define ``tie_breaker(state) -> str`` to preserve a
    domain-specific deterministic order for equal A* priorities.
    """

    @property
    def start_state(self) -> StateT:
        ...

    def is_goal(self, state: StateT) -> bool:
        ...

    def neighbors(self, state: StateT) -> Iterable[tuple[StateT, float]]:
        ...

    def heuristic(self, state: StateT) -> float:
        ...
