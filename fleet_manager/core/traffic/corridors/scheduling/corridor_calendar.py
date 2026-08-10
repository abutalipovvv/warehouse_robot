"""Interval placement for the controlled-corridor calendar."""

from __future__ import annotations

from typing import Iterable

from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorRequest,
    CorridorSchedulerConfig,
    CorridorSlot,
)


def _request_base_entry(request: CorridorRequest, now: float) -> float:
    entry = max(now, request.earliest_entry)
    if request.downstream_ready_at is not None:
        entry = max(entry, request.downstream_ready_at - request.duration_sec)
    return entry


def _earliest_placement(
    request: CorridorRequest,
    *,
    slots: Iterable[CorridorSlot],
    now: float,
    horizon_end: float,
    config: CorridorSchedulerConfig,
) -> tuple[float, float, int] | None:
    request_regions = frozenset(request.regions)
    relevant = sorted(
        (
            slot
            for slot in slots
            if not request_regions.isdisjoint(slot.regions)
        ),
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    candidate_entry = _request_base_entry(request, now)

    # Moving a conflicting passage forward can only resolve that conflict by
    # placing its shared resource window after the existing one.  Repeat
    # because the shift may reach another future window on a different
    # corridor resource.  The monotonic loop is deterministic and bounded by
    # the finite calendar; unlike a graph search it cannot oscillate.
    max_iterations = max(
        4,
        (len(relevant) + 1) * (len(request.regions) + 1),
    )
    for _ in range(max_iterations):
        if candidate_entry > horizon_end + 1e-9:
            return None
        required_entry = candidate_entry
        for slot in relevant:
            predecessor = (
                request.predecessor_robot_id == slot.robot_id
            )
            if (
                not predecessor
                and _request_slot_windows_fit(
                    request,
                    candidate_entry,
                    slot,
                    config,
                )
            ):
                continue
            required_entry = max(
                required_entry,
                _request_entry_after_slot(
                    request,
                    slot,
                    config,
                ),
            )
        if required_entry <= candidate_entry + 1e-9:
            previous, following = _calendar_neighbours(
                candidate_entry,
                relevant,
            )
            return (
                candidate_entry,
                candidate_entry + request.duration_sec,
                _phase_switches(
                    previous,
                    following,
                    request.direction,
                ),
            )
        candidate_entry = required_entry
    return None


def _placement_at(
    request: CorridorRequest,
    *,
    entry_time: float,
    slots: Iterable[CorridorSlot],
    now: float,
    horizon_end: float,
    config: CorridorSchedulerConfig,
) -> tuple[float, float, int] | None:
    if (
        entry_time + 1e-9 < _request_base_entry(request, now)
        or entry_time > horizon_end + 1e-9
    ):
        return None
    request_regions = frozenset(request.regions)
    relevant = sorted(
        (
            slot
            for slot in slots
            if not request_regions.isdisjoint(slot.regions)
        ),
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    if any(
        not _request_slot_windows_fit(
            request,
            entry_time,
            slot,
            config,
        )
        for slot in relevant
    ):
        return None
    if request.predecessor_robot_id:
        predecessor = next(
            (
                slot
                for slot in relevant
                if slot.robot_id == request.predecessor_robot_id
            ),
            None,
        )
        if (
            predecessor is None
            or entry_time + 1e-9
            < _request_entry_after_slot(
                request,
                predecessor,
                config,
            )
        ):
            return None
    previous, following = _calendar_neighbours(
        entry_time,
        relevant,
    )
    return (
        entry_time,
        entry_time + request.duration_sec,
        _phase_switches(previous, following, request.direction),
    )


def _request_slot_windows_fit(
    request: CorridorRequest,
    entry_time: float,
    slot: CorridorSlot,
    config: CorridorSchedulerConfig,
) -> bool:
    guard = _direction_change_guard(config)
    for request_window in request.resource_windows:
        for slot_window in slot.resource_windows:
            if request_window.region_id != slot_window.region_id:
                continue
            first_entry = entry_time + request_window.entry_offset_sec
            first_exit = entry_time + request_window.exit_offset_sec
            second_entry = slot.entry_time + slot_window.entry_offset_sec
            second_exit = slot.entry_time + slot_window.exit_offset_sec
            if request_window.direction == slot_window.direction:
                first_is_after = (
                    first_entry
                    >= second_entry + config.headway_sec - 1e-9
                    and first_exit
                    >= second_exit + config.headway_sec - 1e-9
                )
                first_is_before = (
                    first_entry + config.headway_sec
                    <= second_entry + 1e-9
                    and first_exit + config.headway_sec
                    <= second_exit + 1e-9
                )
                if not first_is_after and not first_is_before:
                    return False
            elif not (
                first_exit + guard <= second_entry + 1e-9
                or first_entry >= second_exit + guard - 1e-9
            ):
                return False
            break
    return True


def _request_entry_after_slot(
    request: CorridorRequest,
    slot: CorridorSlot,
    config: CorridorSchedulerConfig,
) -> float:
    required = float("-inf")
    guard = _direction_change_guard(config)
    for request_window in request.resource_windows:
        slot_window = next(
            (
                window
                for window in slot.resource_windows
                if window.region_id == request_window.region_id
            ),
            None,
        )
        if slot_window is None:
            continue
        slot_entry = slot.entry_time + slot_window.entry_offset_sec
        slot_exit = slot.entry_time + slot_window.exit_offset_sec
        if request_window.direction == slot_window.direction:
            required = max(
                required,
                slot_entry
                + config.headway_sec
                - request_window.entry_offset_sec,
                slot_exit
                + config.headway_sec
                - request_window.exit_offset_sec,
            )
        else:
            required = max(
                required,
                slot_exit
                + guard
                - request_window.entry_offset_sec,
            )
    return required


def _calendar_neighbours(
    entry_time: float,
    slots: Iterable[CorridorSlot],
) -> tuple[CorridorSlot | None, CorridorSlot | None]:
    previous: CorridorSlot | None = None
    following: CorridorSlot | None = None
    for slot in slots:
        if slot.entry_time <= entry_time + 1e-9:
            previous = slot
        elif following is None:
            following = slot
    return previous, following


def _slot_interval_fits(
    candidate: CorridorSlot,
    slots: Iterable[CorridorSlot],
    config: CorridorSchedulerConfig,
) -> bool:
    candidate_regions = frozenset(candidate.regions)
    for other in slots:
        if candidate_regions.isdisjoint(other.regions):
            continue
        if not _slot_windows_fit(candidate, other, config):
            return False
    return True


def _slot_windows_fit(
    first: CorridorSlot,
    second: CorridorSlot,
    config: CorridorSchedulerConfig,
) -> bool:
    """Compare immutable slot windows without building temporary mappings."""
    guard = _direction_change_guard(config)
    for first_window in first.resource_windows:
        for second_window in second.resource_windows:
            if first_window.region_id != second_window.region_id:
                continue
            first_entry = first.entry_time + first_window.entry_offset_sec
            first_exit = first.entry_time + first_window.exit_offset_sec
            second_entry = second.entry_time + second_window.entry_offset_sec
            second_exit = second.entry_time + second_window.exit_offset_sec
            if first_window.direction == second_window.direction:
                first_is_after = (
                    first_entry
                    >= second_entry + config.headway_sec - 1e-9
                    and first_exit
                    >= second_exit + config.headway_sec - 1e-9
                )
                first_is_before = (
                    first_entry + config.headway_sec
                    <= second_entry + 1e-9
                    and first_exit + config.headway_sec
                    <= second_exit + 1e-9
                )
                if not first_is_after and not first_is_before:
                    return False
            elif not (
                first_exit + guard <= second_entry + 1e-9
                or first_entry >= second_exit + guard - 1e-9
            ):
                return False
            break
    return True


def _direction_change_guard(config: CorridorSchedulerConfig) -> float:
    return max(config.headway_sec, config.direction_change_sec)


def _phase_switches(
    previous: CorridorSlot | None,
    following: CorridorSlot | None,
    direction: str,
) -> int:
    switches = 0
    if previous is not None and previous.direction != direction:
        switches += 1
    if following is not None and following.direction != direction:
        switches += 1
    return switches


class CorridorCalendar:
    """Place requests against immutable resource-window reservations."""

    def __init__(self, config: CorridorSchedulerConfig) -> None:
        self.config = config

    @staticmethod
    def request_base_entry(
        request: CorridorRequest,
        now: float,
    ) -> float:
        return _request_base_entry(request, now)

    def earliest_placement(
        self,
        request: CorridorRequest,
        *,
        slots: Iterable[CorridorSlot],
        now: float,
        horizon_end: float,
    ) -> tuple[float, float, int] | None:
        return _earliest_placement(
            request,
            slots=slots,
            now=now,
            horizon_end=horizon_end,
            config=self.config,
        )

    def placement_at(
        self,
        request: CorridorRequest,
        *,
        entry_time: float,
        slots: Iterable[CorridorSlot],
        now: float,
        horizon_end: float,
    ) -> tuple[float, float, int] | None:
        return _placement_at(
            request,
            entry_time=entry_time,
            slots=slots,
            now=now,
            horizon_end=horizon_end,
            config=self.config,
        )

    def slot_interval_fits(
        self,
        candidate: CorridorSlot,
        slots: Iterable[CorridorSlot],
    ) -> bool:
        return _slot_interval_fits(candidate, slots, self.config)


slot_interval_fits = _slot_interval_fits
request_entry_after_slot = _request_entry_after_slot
