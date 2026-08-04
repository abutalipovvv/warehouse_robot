"""Public facade for controlled-corridor scheduling."""

from __future__ import annotations

from dataclasses import replace
from typing import Collection, Iterable

from fleet_manager.core.traffic.corridors.scheduling.corridor_calendar import slot_interval_fits
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorDecision,
    CorridorDecisionStatus,
    CorridorOccupancy,
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSchedule,
    CorridorSchedulerConfig,
    CorridorSlot,
    CorridorSlotState,
    RouteRevision,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_planner import (
    CorridorScheduleBuilder,
)


def build_corridor_schedule(
    requests: Iterable[CorridorRequest],
    *,
    controlled_regions: Collection[str],
    now: float,
    config: CorridorSchedulerConfig | None = None,
    previous: CorridorSchedule | None = None,
    occupancies: Iterable[CorridorOccupancy] | None = None,
    pinned_slots: Iterable[CorridorSlot] | None = None,
) -> CorridorSchedule:
    """Build one deterministic rolling controlled-corridor calendar."""

    return CorridorScheduleBuilder(config).build(
        requests,
        controlled_regions=controlled_regions,
        now=now,
        previous=previous,
        occupancies=occupancies,
        pinned_slots=pinned_slots,
    )


class CentralCorridorScheduler:
    """Stateful facade that retains only the last immutable calendar."""

    def __init__(
        self,
        controlled_regions: Collection[str],
        *,
        config: CorridorSchedulerConfig | None = None,
    ) -> None:
        self._controlled_regions = frozenset(
            region.strip() for region in controlled_regions if region.strip()
        )
        self._config = config or CorridorSchedulerConfig()
        self._schedule: CorridorSchedule | None = None
        self._pinned_slots: dict[str, CorridorSlot] = {}

    @property
    def controlled_regions(self) -> frozenset[str]:
        return self._controlled_regions

    @property
    def current_schedule(self) -> CorridorSchedule | None:
        return self._schedule

    def update(
        self,
        requests: Iterable[CorridorRequest],
        *,
        now: float,
        occupancies: Iterable[CorridorOccupancy] | None = None,
    ) -> CorridorSchedule:
        self._schedule = build_corridor_schedule(
            requests,
            controlled_regions=self._controlled_regions,
            now=now,
            config=self._config,
            previous=self._schedule,
            occupancies=occupancies,
            pinned_slots=self._pinned_slots.values(),
        )
        for robot_id, pinned in list(self._pinned_slots.items()):
            current = self._schedule.slot_for(robot_id)
            if (
                current != pinned
                or current.state is not CorridorSlotState.TENTATIVE
            ):
                # Physical occupancy and already committed commands outrank an
                # in-flight proposal.  Once either invalidates the exact slot,
                # the worker may finish but its captured commit must fail.
                self._pinned_slots.pop(robot_id, None)
        return self._schedule

    def pin_slot(
        self,
        robot_id: str,
        *,
        expected: CorridorSlot,
    ) -> bool:
        """Hold one exact tentative proposal while its MAPF worker runs."""
        schedule = self._schedule
        name = str(robot_id)
        current = schedule.slot_for(name) if schedule is not None else None
        if (
            current != expected
            or expected.robot_id != name
            or expected.state is not CorridorSlotState.TENTATIVE
        ):
            return False
        existing = self._pinned_slots.get(name)
        if existing is not None and existing != expected:
            return False
        self._pinned_slots[name] = expected
        return True

    def release_slot_pin(
        self,
        robot_id: str,
        *,
        expected: CorridorSlot | None = None,
    ) -> None:
        """Release only the worker lease that captured ``expected``."""
        name = str(robot_id)
        current = self._pinned_slots.get(name)
        if current is None:
            return
        if expected is not None and current != expected:
            return
        self._pinned_slots.pop(name, None)

    def commit_slot(
        self,
        robot_id: str,
        *,
        expected: CorridorSlot,
        actual: CorridorSlot | None = None,
    ) -> CorridorSchedule | None:
        """Promote an unchanged tentative slot to an issued command.

        Planning a continuation is a two-phase transaction: first the rolling
        calendar offers a tentative slot, then MAPF proves that its actual
        resource windows fit.  At that point the command must survive the
        route-revision handoff exactly like an imminent/entered passage.
        ``expected`` prevents a stale worker result from committing a slot
        which the runtime scheduler has already moved.
        """
        schedule = self._schedule
        if schedule is None:
            return None
        current = schedule.slot_for(str(robot_id))
        if current is None or current != expected:
            return None
        candidate = actual or current
        if (
            candidate.robot_id != current.robot_id
            or candidate.regions != current.regions
            or candidate.direction != current.direction
            or candidate.staging_lm != current.staging_lm
            or candidate.exit_lm != current.exit_lm
            or tuple(
                (window.region_id, window.direction)
                for window in candidate.resource_windows
            )
            != tuple(
                (window.region_id, window.direction)
                for window in current.resource_windows
            )
            or candidate.entry_time
            < schedule.generated_at - self._config.occupancy_recheck_sec
            or candidate.entry_time > schedule.horizon_end + 1e-9
        ):
            return None
        candidate = replace(
            candidate,
            state=CorridorSlotState.COMMITTED,
            physically_observed=False,
        )
        other_slots = tuple(
            slot
            for slot in schedule.slots
            if slot.robot_id != robot_id
        )
        committed_others = tuple(
            slot
            for slot in other_slots
            if slot.state is CorridorSlotState.COMMITTED
        )
        if not slot_interval_fits(
            candidate,
            committed_others,
            self._config,
        ):
            return None
        if (
            current.state is CorridorSlotState.COMMITTED
            and candidate == current
        ):
            return schedule

        committed = candidate
        # A tentative slot is a calendar proposal, not an issued command. Exact
        # SIPP timing may legitimately arrive later than its nominal proposal
        # because of an ordinary edge/turn reservation. Keep physical and
        # already committed passages immutable, displace only overlapping
        # tentative proposals, and let the next update place them again around
        # this validated command.
        retained_others = tuple(
            slot
            for slot in other_slots
            if (
                slot.state is CorridorSlotState.COMMITTED
                or slot_interval_fits(
                    committed,
                    (slot,),
                    self._config,
                )
            )
        )
        displaced_robot_ids = {
            slot.robot_id
            for slot in other_slots
            if slot not in retained_others
        }
        slots = tuple(
            sorted(
                (
                    committed,
                    *retained_others,
                ),
                key=lambda slot: (slot.entry_time, slot.robot_id),
            )
        )
        decisions = dict(schedule.decisions)
        for displaced_robot_id in displaced_robot_ids:
            decisions[displaced_robot_id] = CorridorDecision(
                robot_id=displaced_robot_id,
                status=CorridorDecisionStatus.DEFERRED,
                reason="tentative slot displaced by validated command",
            )
        decisions[str(robot_id)] = CorridorDecision(
            robot_id=str(robot_id),
            status=CorridorDecisionStatus.GRANTED,
            reason="committed after MAPF resource validation",
            slot=committed,
        )
        self._schedule = CorridorSchedule(
            epoch=schedule.epoch + 1,
            generated_at=schedule.generated_at,
            horizon_end=schedule.horizon_end,
            slots=slots,
            decisions=decisions,
            changed=True,
        )
        self._pinned_slots.pop(str(robot_id), None)
        for displaced_robot_id in displaced_robot_ids:
            self._pinned_slots.pop(displaced_robot_id, None)
        return self._schedule

    def reset(self) -> None:
        self._schedule = None
        self._pinned_slots.clear()

    def transaction_state(
        self,
    ) -> tuple[CorridorSchedule | None, dict[str, CorridorSlot]]:
        """Return the small mutable facade state used by plan rollback."""

        return self._schedule, dict(self._pinned_slots)

    def restore_transaction_state(
        self,
        state: tuple[CorridorSchedule | None, dict[str, CorridorSlot]],
    ) -> None:
        """Restore a checkpoint after a failed runtime plan commit."""

        schedule, pinned_slots = state
        self._schedule = schedule
        self._pinned_slots.clear()
        self._pinned_slots.update(pinned_slots)
