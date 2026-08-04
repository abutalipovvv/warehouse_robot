"""Immutable result returned by graph-search algorithms."""

from __future__ import annotations

from collections.abc import Hashable, Iterable
from dataclasses import dataclass
import math
from numbers import Real
from typing import Generic, TypeVar


StateT = TypeVar("StateT", bound=Hashable)


@dataclass(frozen=True, slots=True)
class SearchResult(Generic[StateT]):
    """A successful path or an explicit search failure."""

    found: bool
    path: tuple[StateT, ...]
    cost: float
    expanded_count: int
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.found, bool):
            raise TypeError("found must be a bool")
        if not isinstance(self.path, tuple):
            raise TypeError("path must be a tuple")
        if (
            isinstance(self.expanded_count, bool)
            or not isinstance(self.expanded_count, int)
        ):
            raise TypeError("expanded_count must be an int")
        if self.expanded_count < 0:
            raise ValueError("expanded_count must not be negative")
        if isinstance(self.cost, bool) or not isinstance(self.cost, Real):
            raise TypeError("cost must be a real number")
        cost = float(self.cost)
        object.__setattr__(self, "cost", cost)

        if self.found:
            if not self.path:
                raise ValueError("a successful result must contain a path")
            if not math.isfinite(cost) or cost < 0.0:
                raise ValueError(
                    "a successful result cost must be finite and non-negative"
                )
            if self.failure_reason is not None:
                raise ValueError(
                    "a successful result cannot have a failure reason"
                )
            return

        if self.path:
            raise ValueError("a failed result cannot contain a path")
        if cost != math.inf:
            raise ValueError("a failed result cost must be positive infinity")
        if not isinstance(self.failure_reason, str) or not self.failure_reason:
            raise ValueError("a failed result must have a failure reason")

    @classmethod
    def success(
        cls,
        path: Iterable[StateT],
        *,
        cost: float,
        expanded_count: int,
    ) -> SearchResult[StateT]:
        return cls(
            found=True,
            path=tuple(path),
            cost=cost,
            expanded_count=expanded_count,
        )

    @classmethod
    def failure(
        cls,
        *,
        reason: str,
        expanded_count: int,
    ) -> SearchResult[StateT]:
        return cls(
            found=False,
            path=(),
            cost=math.inf,
            expanded_count=expanded_count,
            failure_reason=reason,
        )
