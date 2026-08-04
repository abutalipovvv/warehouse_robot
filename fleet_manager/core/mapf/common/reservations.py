from __future__ import annotations

from bisect import bisect_right
from collections import defaultdict
from dataclasses import dataclass, field
from itertools import groupby
from typing import Iterable, Mapping

from fleet_manager.core.algorithms.math.intervals import TimeInterval, merge_intervals


@dataclass(frozen=True, order=True, slots=True)
class ResourceId:
    """Identity of one capacity-constrained traffic resource."""

    kind: str
    name: str

    def __str__(self) -> str:
        return f"{self.kind}:{self.name}"


@dataclass(frozen=True, slots=True)
class ReservationInterval:
    """A robot's half-open occupation window for one resource."""

    resource: ResourceId
    robot_name: str
    start: int
    end: int
    reason: str = "move"
    committed: bool = True

    @property
    def window(self) -> TimeInterval:
        return TimeInterval(self.start, self.end)

    def overlaps(self, start: int, end: int) -> bool:
        return self.start < end and start < self.end


@dataclass(frozen=True, slots=True)
class SafeInterval:
    """A half-open tick range in which a resource remains available."""

    start: int
    end: int

    @property
    def window(self) -> TimeInterval:
        return TimeInterval(self.start, self.end)

    def contains(self, time_tick: int) -> bool:
        return self.start <= time_tick < self.end


@dataclass(slots=True)
class _ResourceCalendar:
    """Sorted reservations and capacity rules for a single resource."""

    capacity: int = 1
    intervals: list[ReservationInterval] = field(default_factory=list)

    def copy(self) -> "_ResourceCalendar":
        return _ResourceCalendar(
            capacity=self.capacity,
            intervals=list(self.intervals),
        )

    def set_capacity(self, capacity: int) -> None:
        self.capacity = max(1, int(capacity))

    def add(self, interval: ReservationInterval) -> None:
        sort_key = (
            interval.start,
            interval.end,
            interval.robot_name,
        )
        index = bisect_right(
            self.intervals,
            sort_key,
            key=lambda item: (
                item.start,
                item.end,
                item.robot_name,
            ),
        )
        self.intervals.insert(index, interval)

    def conflicts(
        self,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[ReservationInterval]:
        conflicts: list[ReservationInterval] = []
        for interval in self.intervals:
            if interval.start >= end:
                break
            if interval.end <= start:
                continue
            if ignore_robot_name and interval.robot_name == ignore_robot_name:
                continue
            if interval.overlaps(start, end):
                conflicts.append(interval)
        return conflicts

    def blocked_ranges(
        self,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[tuple[int, int]]:
        """Return periods where distinct active robots fill the capacity."""

        capacity = self.capacity
        events: list[tuple[int, str, int]] = []
        for interval in self.intervals:
            if interval.start >= end:
                break
            if interval.end <= start:
                continue
            if ignore_robot_name and interval.robot_name == ignore_robot_name:
                continue
            events.append(
                (max(start, interval.start), interval.robot_name, 1)
            )
            events.append(
                (min(end, interval.end), interval.robot_name, -1)
            )
        if not events:
            return []

        blocked: list[tuple[int, int]] = []
        active_reservations: dict[str, int] = {}
        cursor = start

        for time_tick, grouped_events in groupby(
            sorted(events),
            key=lambda item: item[0],
        ):
            if cursor < time_tick and len(active_reservations) >= capacity:
                blocked.append((cursor, time_tick))

            changes: dict[str, int] = defaultdict(int)
            for _, robot_name, change in grouped_events:
                changes[robot_name] += change
            for robot_name, change in changes.items():
                count = active_reservations.get(robot_name, 0) + change
                if count > 0:
                    active_reservations[robot_name] = count
                else:
                    active_reservations.pop(robot_name, None)
            cursor = time_tick

        if cursor < end and len(active_reservations) >= capacity:
            blocked.append((cursor, end))
        return blocked

    def remove_by_identity(self, identities: set[int]) -> None:
        self.intervals = [
            interval
            for interval in self.intervals
            if id(interval) not in identities
        ]


class ReservationTable:
    """Capacity-aware reservation calendars indexed by traffic resource."""

    def __init__(
        self,
        capacities: Mapping[ResourceId, int] | None = None,
    ) -> None:
        self._calendars: dict[ResourceId, _ResourceCalendar] = {
            resource: _ResourceCalendar(max(1, int(capacity)))
            for resource, capacity in (capacities or {}).items()
        }
        self._by_robot: dict[str, list[ReservationInterval]] = defaultdict(list)

    def copy(self) -> "ReservationTable":
        clone = ReservationTable()
        clone._calendars = {
            resource: calendar.copy()
            for resource, calendar in self._calendars.items()
        }
        clone._by_robot = defaultdict(
            list,
            {
                robot_name: list(intervals)
                for robot_name, intervals in self._by_robot.items()
            },
        )
        return clone

    def set_capacity(self, resource: ResourceId, capacity: int) -> None:
        self._calendar(resource).set_capacity(capacity)

    def reserve(self, interval: ReservationInterval) -> None:
        start = max(0, int(interval.start))
        end = max(start + 1, int(interval.end))
        normalized = ReservationInterval(
            resource=interval.resource,
            robot_name=interval.robot_name,
            start=start,
            end=end,
            reason=interval.reason,
            committed=interval.committed,
        )
        self._calendar(normalized.resource).add(normalized)
        self._by_robot[normalized.robot_name].append(normalized)

    def reserve_many(self, intervals: Iterable[ReservationInterval]) -> None:
        for interval in intervals:
            self.reserve(interval)

    def conflicts(
        self,
        resource: ResourceId,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[ReservationInterval]:
        query_start, query_end = self._normalized_bounds(start, end)
        calendar = self._calendars.get(resource)
        if calendar is None:
            return []
        return calendar.conflicts(
            query_start,
            query_end,
            ignore_robot_name=ignore_robot_name,
        )

    def is_free(
        self,
        resource: ResourceId,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> bool:
        query_start, query_end = self._normalized_bounds(start, end)
        calendar = self._calendars.get(resource)
        return (
            calendar is None
            or not calendar.blocked_ranges(
                query_start,
                query_end,
                ignore_robot_name=ignore_robot_name,
            )
        )

    def resources_are_free(
        self,
        resources: Iterable[ResourceId],
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> bool:
        return all(
            self.is_free(
                resource,
                start,
                end,
                ignore_robot_name=ignore_robot_name,
            )
            for resource in resources
        )

    def safe_intervals_for_resources(
        self,
        resources: Iterable[ResourceId],
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> tuple[SafeInterval, ...]:
        window_start = max(0, int(start))
        window_end = max(window_start, int(end))
        if window_start >= window_end:
            return ()

        blocked = self._merged_blocked_ranges(
            resources,
            window_start,
            window_end,
            ignore_robot_name=ignore_robot_name,
        )
        if not blocked:
            return (SafeInterval(window_start, window_end),)

        safe_intervals: list[SafeInterval] = []
        cursor = window_start
        for blocked_start, blocked_end in blocked:
            if cursor < blocked_start:
                safe_intervals.append(SafeInterval(cursor, blocked_start))
            cursor = max(cursor, blocked_end)
        if cursor < window_end:
            safe_intervals.append(SafeInterval(cursor, window_end))
        return tuple(safe_intervals)

    def release_robot_uncommitted(self, robot_name: str) -> None:
        intervals = self._by_robot.get(robot_name, [])
        if not intervals:
            return

        uncommitted_ids = {
            id(interval)
            for interval in intervals
            if not interval.committed
        }
        if not uncommitted_ids:
            return

        for resource, calendar in list(self._calendars.items()):
            calendar.remove_by_identity(uncommitted_ids)
            if not calendar.intervals and calendar.capacity == 1:
                self._calendars.pop(resource, None)
        self._by_robot[robot_name] = [
            interval
            for interval in intervals
            if interval.committed
        ]

    def intervals_for_resource(
        self,
        resource: ResourceId,
    ) -> tuple[ReservationInterval, ...]:
        calendar = self._calendars.get(resource)
        return tuple(calendar.intervals) if calendar is not None else ()

    def _calendar(self, resource: ResourceId) -> _ResourceCalendar:
        return self._calendars.setdefault(resource, _ResourceCalendar())

    def _blocked_windows(
        self,
        resources: Iterable[ResourceId],
        query: TimeInterval,
        *,
        ignore_robot_name: str = "",
    ) -> tuple[TimeInterval, ...]:
        return merge_intervals(
            TimeInterval(start, end)
            for start, end in self._merged_blocked_ranges(
                resources,
                int(query.start),
                int(query.end),
                ignore_robot_name=ignore_robot_name,
            )
        )

    def _merged_blocked_ranges(
        self,
        resources: Iterable[ResourceId],
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[tuple[int, int]]:
        blocked: list[tuple[int, int]] = []
        for resource in resources:
            calendar = self._calendars.get(resource)
            if calendar is None:
                continue
            blocked.extend(
                calendar.blocked_ranges(
                    start,
                    end,
                    ignore_robot_name=ignore_robot_name,
                )
            )
        if not blocked:
            return []

        blocked.sort()
        merged = [blocked[0]]
        for blocked_start, blocked_end in blocked[1:]:
            previous_start, previous_end = merged[-1]
            if blocked_start > previous_end:
                merged.append((blocked_start, blocked_end))
            else:
                merged[-1] = (
                    previous_start,
                    max(previous_end, blocked_end),
                )
        return merged

    @staticmethod
    def _normalized_bounds(start: int, end: int) -> tuple[int, int]:
        normalized_start = max(0, int(start))
        normalized_end = max(normalized_start + 1, int(end))
        return normalized_start, normalized_end

    @classmethod
    def _normalized_query(cls, start: int, end: int) -> TimeInterval:
        normalized_start, normalized_end = cls._normalized_bounds(start, end)
        return TimeInterval(normalized_start, normalized_end)

    # Compatibility helpers retained while older planners migrate to the
    # TimeInterval API.
    def _merged_blocked_intervals(
        self,
        resources: Iterable[ResourceId],
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[tuple[int, int]]:
        query_start, query_end = self._normalized_bounds(start, end)
        return self._merged_blocked_ranges(
            resources,
            query_start,
            query_end,
            ignore_robot_name=ignore_robot_name,
        )

    def _blocked_intervals_for_resource(
        self,
        resource: ResourceId,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[tuple[int, int]]:
        query_start, query_end = self._normalized_bounds(start, end)
        calendar = self._calendars.get(resource)
        if calendar is None:
            return []
        return calendar.blocked_ranges(
            query_start,
            query_end,
            ignore_robot_name=ignore_robot_name,
        )
