from __future__ import annotations

from random import Random

from fleet_manager.core.mapf.reservations import (
    ReservationInterval,
    ReservationTable,
    ResourceId,
)


def _brute_force_safe_ticks(
    intervals: list[ReservationInterval],
    *,
    capacity: int,
    start: int,
    end: int,
    ignore_robot_name: str = "",
) -> list[bool]:
    result: list[bool] = []
    for time_tick in range(start, end):
        active_robots = {
            interval.robot_name
            for interval in intervals
            if interval.robot_name != ignore_robot_name
            and interval.start <= time_tick < interval.end
        }
        result.append(len(active_robots) < capacity)
    return result


def _safe_interval_ticks(
    table: ReservationTable,
    resource: ResourceId,
    *,
    start: int,
    end: int,
    ignore_robot_name: str = "",
) -> list[bool]:
    result = [False] * (end - start)
    for interval in table.safe_intervals_for_resources(
        (resource,),
        start,
        end,
        ignore_robot_name=ignore_robot_name,
    ):
        for time_tick in range(interval.start, interval.end):
            result[time_tick - start] = True
    return result


def test_random_capacity_calendars_match_tick_reference() -> None:
    random = Random(20260731)

    for capacity in range(1, 5):
        resource = ResourceId("lane_group", f"capacity-{capacity}")
        table = ReservationTable({resource: capacity})
        intervals: list[ReservationInterval] = []
        for index in range(80):
            start = random.randrange(0, 50)
            interval = ReservationInterval(
                resource=resource,
                robot_name=f"robot-{random.randrange(8)}",
                start=start,
                end=start + random.randrange(1, 12),
                reason=f"sample-{index}",
            )
            intervals.append(interval)
            table.reserve(interval)

        for ignored_robot in ("", "robot-0", "robot-5"):
            assert _safe_interval_ticks(
                table,
                resource,
                start=0,
                end=64,
                ignore_robot_name=ignored_robot,
            ) == _brute_force_safe_ticks(
                intervals,
                capacity=capacity,
                start=0,
                end=64,
                ignore_robot_name=ignored_robot,
            )


def test_safe_windows_intersect_blocking_periods_from_all_resources() -> None:
    first = ResourceId("vertex", "A")
    second = ResourceId("clearance", "A<->B")
    table = ReservationTable()
    table.reserve(ReservationInterval(first, "r1", 2, 5))
    table.reserve(ReservationInterval(second, "r2", 7, 9))

    safe = table.safe_intervals_for_resources((first, second), 0, 10)

    assert [(item.start, item.end) for item in safe] == [
        (0, 2),
        (5, 7),
        (9, 10),
    ]


def test_copy_is_independent_from_original_table() -> None:
    resource = ResourceId("vertex", "A")
    original = ReservationTable()
    original.reserve(ReservationInterval(resource, "committed", 0, 2))

    copied = original.copy()
    copied.reserve(ReservationInterval(resource, "new", 3, 5))

    assert [item.robot_name for item in original.intervals_for_resource(resource)] == [
        "committed",
    ]
    assert [item.robot_name for item in copied.intervals_for_resource(resource)] == [
        "committed",
        "new",
    ]


def test_release_removes_only_uncommitted_intervals_for_selected_robot() -> None:
    resource = ResourceId("lane", "A->B")
    table = ReservationTable()
    table.reserve(
        ReservationInterval(
            resource,
            "selected",
            0,
            3,
            committed=True,
        )
    )
    table.reserve(
        ReservationInterval(
            resource,
            "selected",
            3,
            6,
            committed=False,
        )
    )
    table.reserve(
        ReservationInterval(
            resource,
            "other",
            3,
            6,
            committed=False,
        )
    )

    table.release_robot_uncommitted("selected")

    assert [
        (item.robot_name, item.start, item.end, item.committed)
        for item in table.intervals_for_resource(resource)
    ] == [
        ("selected", 0, 3, True),
        ("other", 3, 6, False),
    ]


def test_reserve_normalizes_negative_and_empty_windows() -> None:
    resource = ResourceId("vertex", "A")
    table = ReservationTable()

    table.reserve(ReservationInterval(resource, "negative", -4, -2))
    table.reserve(ReservationInterval(resource, "empty", 5, 5))

    assert [
        (item.start, item.end)
        for item in table.intervals_for_resource(resource)
    ] == [(0, 1), (5, 6)]
