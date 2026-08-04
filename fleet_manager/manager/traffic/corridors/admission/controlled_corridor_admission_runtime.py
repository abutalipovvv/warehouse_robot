"""Refresh the central calendar and publish runtime authority."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorOccupancy,
    CorridorResourceWindow,
    CorridorSlotState,
)

from .controlled_corridor_admission_models import (
    _CentralCorridorBuild,
    _CentralCorridorPublication,
)


class ControlledCorridorRuntimePublicationMixin:
    """Refresh the central calendar and publish runtime authority."""

    def _prepare_controlled_corridor_admissions(self, now: float) -> None:
        """Refresh corridor authority, or stay inert when no zones exist."""
        scheduler = self._controlled_corridor_scheduler
        if scheduler is not None:
            self._prepare_central_controlled_corridor_schedule(now)
            return

        # Corridor dispatch is opt-in through explicitly authored controlled
        # regions. With no scheduler there is no implicit lease/FIFO fallback:
        # ordinary rolling SIPP/CBS traffic control remains the sole authority.
        self._controlled_corridor_tick_now = now
        self._controlled_corridor_schedule = None
        self._controlled_corridor_wait_since.clear()
        self._controlled_corridor_leases.clear()
        self._controlled_corridor_passages.clear()
        self._controlled_corridor_winners.clear()
        self._controlled_corridor_occupancy.clear()
        self._controlled_corridor_queues.clear()
        self._controlled_corridor_blockers.clear()

    def _prepare_central_controlled_corridor_schedule(
        self,
        now: float,
    ) -> None:
        scheduler = self._controlled_corridor_scheduler
        graph = self._controlled_corridor_graph
        if scheduler is None or graph is None:
            return
        self._controlled_corridor_tick_now = now
        context = _CentralCorridorBuild(
            now=now,
            scheduler=scheduler,
            old_schedule=scheduler.current_schedule,
        )
        self._capture_controlled_corridor_occupancy(context)
        self._collect_controlled_corridor_requests(context)
        self._collect_controlled_corridor_prefetch_requests(context)
        self._maintain_controlled_corridor_waits(context)
        schedule = self._update_controlled_corridor_calendar(context)
        self._publish_controlled_corridor_runtime(context, schedule)

    def _update_controlled_corridor_calendar(
        self,
        context: _CentralCorridorBuild,
    ) -> Any:
        now = context.now
        scheduler = context.scheduler
        old_schedule = context.old_schedule
        physical_by_robot = context.physical_by_robot
        requests = context.requests
        entries_by_robot = context.entries_by_robot
        scheduled_intent_names = context.scheduled_intent_names
        downstream_blockers = context.downstream_blockers
        occupancies: list[CorridorOccupancy] = []
        for robot_name, physical_regions in physical_by_robot.items():
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            previous_slot = (
                old_schedule.slot_for(robot_name)
                if old_schedule is not None
                else None
            )
            entry = entries_by_robot.get(robot_name, {})
            old_passage = self._controlled_corridor_passages.get(
                robot_name,
                {},
            )
            occupancies.append(
                self._controlled_corridor_live_occupancy(
                    robot,
                    physical_regions=physical_regions,
                    previous_slot=previous_slot,
                    entry=entry,
                    previous_passage=old_passage,
                    now=now,
                )
            )

        schedule = scheduler.update(
            requests,
            now=now,
            occupancies=occupancies,
        )
        self._controlled_corridor_schedule = schedule
        for robot_name in scheduled_intent_names:
            intent = self._controlled_corridor_prefetch_intents.get(
                robot_name
            )
            if isinstance(intent, dict):
                intent["last_schedule_epoch"] = schedule.epoch
        self._controlled_corridor_blockers = downstream_blockers
        previous_committed = {
            slot.robot_id
            for slot in (old_schedule.slots if old_schedule is not None else ())
            if slot.state is CorridorSlotState.COMMITTED
        }
        newly_committed = {
            slot.robot_id
            for slot in schedule.slots
            if slot.state is CorridorSlotState.COMMITTED
        } - previous_committed
        self.traffic_metrics["corridorAdmissionsGranted"] += len(
            newly_committed
        )
        return schedule

    def _publish_controlled_corridor_runtime(
        self,
        context: _CentralCorridorBuild,
        schedule: Any,
    ) -> None:
        publication = _CentralCorridorPublication(
            immediate_window=max(
                self._runtime_motion_step(),
                self._continuous_collision_step(),
            )
        )
        for slot in schedule.slots:
            self._publish_controlled_corridor_slot(
                context,
                schedule,
                slot,
                publication,
            )
        for request in context.requests:
            if schedule.slot_for(request.robot_id) is not None:
                continue
            for region_id in request.regions:
                publication.queues.setdefault(region_id, []).append(
                    (float("inf"), request.robot_id)
                )
        self._controlled_corridor_passages = publication.passages
        self._controlled_corridor_leases = publication.leases
        self._controlled_corridor_winners = publication.winners
        self._controlled_corridor_queues = {
            region_id: [
                robot_name
                for _, robot_name in sorted(
                    members,
                    key=lambda item: (item[0], item[1]),
                )
            ]
            for region_id, members in publication.queues.items()
        }

    def _publish_controlled_corridor_slot(
        self,
        context: _CentralCorridorBuild,
        schedule: Any,
        slot: Any,
        publication: _CentralCorridorPublication,
    ) -> None:
        """Project one calendar slot into passage, lease and queue state."""
        entry = context.entries_by_robot.get(slot.robot_id, {})
        previous_passage = self._controlled_corridor_passages.get(
            slot.robot_id,
            {},
        )
        entered = slot.robot_id in context.physical_by_robot
        entry_route_windows = tuple(
            window
            for window in entry.get("resource_windows", ())
            if isinstance(window, CorridorResourceWindow)
        )
        route_resource_windows = (
            entry_route_windows
            or tuple(
                window
                for window in previous_passage.get(
                    "route_resource_windows",
                    (),
                )
                if isinstance(window, CorridorResourceWindow)
            )
            or (
                slot.resource_windows
                if not slot.physically_observed
                else ()
            )
        )
        staging_clock = float(
            entry.get(
                "staging_clock",
                previous_passage.get("staging_clock", 0.0),
            )
            or 0.0
        )
        resource_intervals = tuple(
            {
                "region": window.region_id,
                "direction": window.direction,
                "entry_time": (
                    slot.entry_time + window.entry_offset_sec
                ),
                "exit_time": (
                    slot.entry_time + window.exit_offset_sec
                ),
            }
            for window in slot.resource_windows
        )
        active = bool(
            entered
            or (
                slot.state is CorridorSlotState.COMMITTED
                and slot.entry_time
                <= context.now + publication.immediate_window
            )
        )
        publication.passages[slot.robot_id] = {
            "regions": slot.regions,
            "entry_lm": str(entry.get("src") or ""),
            "staging_lm": slot.staging_lm,
            "staging_clock": staging_clock,
            # Immutable route-clock template for future calendar projection.
            "route_resource_windows": route_resource_windows,
            "exit_lm": slot.exit_lm,
            "direction": slot.direction,
            "lease_until": slot.exit_time,
            "entry_time": slot.entry_time,
            "exit_time": slot.exit_time,
            "resource_intervals": resource_intervals,
            "entered": entered,
            "committed": slot.state is CorridorSlotState.COMMITTED,
            "tentative": slot.state is CorridorSlotState.TENTATIVE,
            "past_commit_point": slot.past_commit_point,
            "route_revision": slot.route_revision,
            "schedule_epoch": schedule.epoch,
        }
        if active:
            publication.winners[slot.robot_id] = slot.regions[0]
        for interval in resource_intervals:
            region_id = str(interval["region"])
            interval_entry = float(interval["entry_time"])
            interval_exit = float(interval["exit_time"])
            resource_active = bool(
                slot.state is CorridorSlotState.COMMITTED
                and active
                and interval_entry
                <= context.now + publication.immediate_window
                and interval_exit > context.now - 0.000001
            )
            if resource_active:
                publication.leases.setdefault(
                    region_id,
                    (slot.robot_id, interval_exit),
                )
            else:
                publication.queues.setdefault(
                    region_id,
                    [],
                ).append((interval_entry, slot.robot_id))
