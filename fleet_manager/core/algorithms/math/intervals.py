"""Half-open time interval mathematics.

All intervals use the same ``[start, end)`` convention.  Consequently two
intervals that only share an endpoint do not overlap.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Iterable


def _finite_number(value: Real, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, order=True, slots=True)
class TimeInterval:
    """An immutable half-open interval ``[start, end)``."""

    start: float
    end: float

    def __post_init__(self) -> None:
        start = _finite_number(self.start, name="start")
        end = _finite_number(self.end, name="end")
        if end < start:
            raise ValueError("end must be greater than or equal to start")
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)

    @property
    def duration(self) -> float:
        return self.end - self.start

    @property
    def is_empty(self) -> bool:
        return self.start == self.end

    def contains(self, moment: Real) -> bool:
        value = _finite_number(moment, name="moment")
        return self.start <= value < self.end

    def __contains__(self, moment: object) -> bool:
        if isinstance(moment, bool) or not isinstance(moment, Real):
            return False
        value = float(moment)
        return math.isfinite(value) and self.start <= value < self.end

    def overlaps(self, other: TimeInterval) -> bool:
        if self.is_empty or other.is_empty:
            return False
        return self.start < other.end and other.start < self.end

    def touches(self, other: TimeInterval) -> bool:
        if self.is_empty or other.is_empty:
            return False
        return self.end == other.start or other.end == self.start

    def intersection(self, other: TimeInterval) -> TimeInterval | None:
        start = max(self.start, other.start)
        end = min(self.end, other.end)
        if start >= end:
            return None
        return TimeInterval(start, end)

    def merged_with(self, other: TimeInterval) -> TimeInterval:
        """Merge overlapping or directly adjacent intervals.

        A gap is treated as a caller error because returning one interval would
        incorrectly mark that gap as occupied.
        """

        if self.end < other.start or other.end < self.start:
            raise ValueError("cannot merge intervals separated by a gap")
        return TimeInterval(min(self.start, other.start), max(self.end, other.end))

    def subtract(self, other: TimeInterval) -> tuple[TimeInterval, ...]:
        """Remove ``other`` and return the remaining zero, one or two pieces."""

        overlap = self.intersection(other)
        if overlap is None:
            return () if self.is_empty else (self,)

        pieces: list[TimeInterval] = []
        if self.start < overlap.start:
            pieces.append(TimeInterval(self.start, overlap.start))
        if overlap.end < self.end:
            pieces.append(TimeInterval(overlap.end, self.end))
        return tuple(pieces)

    def shifted(self, offset: Real) -> TimeInterval:
        amount = _finite_number(offset, name="offset")
        return TimeInterval(self.start + amount, self.end + amount)


def merge_intervals(
    intervals: Iterable[TimeInterval],
) -> tuple[TimeInterval, ...]:
    """Return sorted, non-empty intervals with overlaps and adjacency merged."""

    ordered = sorted(interval for interval in intervals if not interval.is_empty)
    if not ordered:
        return ()

    merged: list[TimeInterval] = [ordered[0]]
    for interval in ordered[1:]:
        previous = merged[-1]
        if interval.start <= previous.end:
            merged[-1] = previous.merged_with(interval)
        else:
            merged.append(interval)
    return tuple(merged)


def subtract_intervals(
    interval: TimeInterval,
    excluded: Iterable[TimeInterval],
) -> tuple[TimeInterval, ...]:
    """Subtract several intervals from one interval.

    This is the basic operation used to derive safe intervals from occupied
    reservation windows.
    """

    remaining: tuple[TimeInterval, ...] = (
        () if interval.is_empty else (interval,)
    )
    for blocked in merge_intervals(excluded):
        next_remaining: list[TimeInterval] = []
        for candidate in remaining:
            next_remaining.extend(candidate.subtract(blocked))
        remaining = tuple(next_remaining)
        if not remaining:
            break
    return remaining


def closed_intervals_overlap(
    start_a: Real,
    end_a: Real,
    start_b: Real,
    end_b: Real,
) -> bool:
    """Return whether two closed intervals share at least one point."""

    return start_a <= end_b and start_b <= end_a
