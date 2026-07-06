from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from itertools import groupby
from typing import Iterable, Mapping


@dataclass(frozen=True, order=True, slots=True)
class ResourceId:
    kind: str
    name: str

    def __str__(self) -> str:
        return f"{self.kind}:{self.name}"


@dataclass(frozen=True, slots=True)
class ReservationInterval:
    resource: ResourceId
    robot_name: str
    start: int
    end: int
    reason: str = "move"
    committed: bool = True

    def overlaps(self, start: int, end: int) -> bool:
        return self.start < end and start < self.end


@dataclass(frozen=True, slots=True)
class SafeInterval:
    start: int
    end: int

    def contains(self, time_tick: int) -> bool:
        return self.start <= time_tick < self.end


class ReservationTable:
    def __init__(
        self,
        capacities: Mapping[ResourceId, int] | None = None,
    ) -> None:
        self._by_resource: dict[ResourceId, list[ReservationInterval]] = defaultdict(list)
        self._by_robot: dict[str, list[ReservationInterval]] = defaultdict(list)
        self._capacities: dict[ResourceId, int] = {
            resource: max(1, int(capacity))
            for resource, capacity in (capacities or {}).items()
        }

    def copy(self) -> "ReservationTable":
        clone = ReservationTable(self._capacities)
        clone._by_resource = defaultdict(list, {key: list(value) for key, value in self._by_resource.items()})
        clone._by_robot = defaultdict(list, {key: list(value) for key, value in self._by_robot.items()})
        return clone

    def set_capacity(self, resource: ResourceId, capacity: int) -> None:
        self._capacities[resource] = max(1, int(capacity))

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
        resource_intervals = self._by_resource[normalized.resource]
        resource_intervals.append(normalized)
        resource_intervals.sort(key=lambda item: (item.start, item.end, item.robot_name))
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
        start = max(0, int(start))
        end = max(start + 1, int(end))
        conflicts: list[ReservationInterval] = []
        for interval in self._by_resource.get(resource, []):
            if interval.start >= end:
                break
            if interval.end <= start:
                continue
            if ignore_robot_name and interval.robot_name == ignore_robot_name:
                continue
            if interval.overlaps(start, end):
                conflicts.append(interval)
        return conflicts

    def is_free(
        self,
        resource: ResourceId,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> bool:
        capacity = self._capacities.get(resource, 1)
        conflicts = self.conflicts(
            resource,
            start,
            end,
            ignore_robot_name=ignore_robot_name,
        )
        occupied_by = {interval.robot_name for interval in conflicts}
        return len(occupied_by) < capacity

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
        start = max(0, int(start))
        end = max(start, int(end))
        if start >= end:
            return ()

        blocked_intervals = self._merged_blocked_intervals(
            resources,
            start,
            end,
            ignore_robot_name=ignore_robot_name,
        )
        if not blocked_intervals:
            return (SafeInterval(start, end),)

        intervals: list[SafeInterval] = []
        cursor = start
        for blocked_start, blocked_end in blocked_intervals:
            if cursor < blocked_start:
                intervals.append(SafeInterval(cursor, blocked_start))
            cursor = max(cursor, blocked_end)
        if cursor < end:
            intervals.append(SafeInterval(cursor, end))
        return tuple(intervals)

    def _merged_blocked_intervals(
        self,
        resources: Iterable[ResourceId],
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[tuple[int, int]]:
        intervals: list[tuple[int, int]] = []
        for resource in resources:
            intervals.extend(
                self._blocked_intervals_for_resource(
                    resource,
                    start,
                    end,
                    ignore_robot_name=ignore_robot_name,
                )
            )
        if not intervals:
            return []

        intervals.sort()
        merged: list[tuple[int, int]] = []
        for blocked_start, blocked_end in intervals:
            if blocked_start >= blocked_end:
                continue
            if not merged or blocked_start > merged[-1][1]:
                merged.append((blocked_start, blocked_end))
            else:
                previous_start, previous_end = merged[-1]
                merged[-1] = (previous_start, max(previous_end, blocked_end))
        return merged

    def _blocked_intervals_for_resource(
        self,
        resource: ResourceId,
        start: int,
        end: int,
        *,
        ignore_robot_name: str = "",
    ) -> list[tuple[int, int]]:
        capacity = self._capacities.get(resource, 1)
        resource_intervals = self._by_resource.get(resource, [])
        if not resource_intervals:
            return []

        events: list[tuple[int, int]] = []
        for interval in resource_intervals:
            if interval.start >= end:
                break
            if interval.end <= start:
                continue
            if ignore_robot_name and interval.robot_name == ignore_robot_name:
                continue
            events.append((max(start, interval.start), 1))
            events.append((min(end, interval.end), -1))
        if not events:
            return []

        blocked: list[tuple[int, int]] = []
        active = 0
        cursor = start
        for time_tick, grouped in groupby(sorted(events), key=lambda item: item[0]):
            if cursor < time_tick and active >= capacity:
                blocked.append((cursor, time_tick))
            active += sum(delta for _, delta in grouped)
            cursor = time_tick
        if cursor < end and active >= capacity:
            blocked.append((cursor, end))
        return blocked

    def release_robot_uncommitted(self, robot_name: str) -> None:
        intervals = self._by_robot.get(robot_name, [])
        if not intervals:
            return
        keep = [interval for interval in intervals if interval.committed]
        remove = {id(interval) for interval in intervals if not interval.committed}
        for resource, resource_intervals in list(self._by_resource.items()):
            next_intervals = [
                interval for interval in resource_intervals
                if id(interval) not in remove
            ]
            if next_intervals:
                self._by_resource[resource] = next_intervals
            else:
                self._by_resource.pop(resource, None)
        self._by_robot[robot_name] = keep

    def intervals_for_resource(self, resource: ResourceId) -> tuple[ReservationInterval, ...]:
        return tuple(self._by_resource.get(resource, ()))
