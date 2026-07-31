"""Collect physical occupancy and live or prefetched requests."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.traffic.corridor_scheduler import (
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSlotState,
)

from .controlled_corridor_admission_models import _CentralCorridorBuild


class ControlledCorridorRequestCollectionMixin:
    """Collect physical occupancy and live or prefetched requests."""

    def _capture_controlled_corridor_occupancy(
        self,
        context: _CentralCorridorBuild,
    ) -> None:
        for robot in self._runtime_robots():
            regions = self._controlled_corridor_physical_regions(
                context,
                robot,
            )
            if not regions:
                continue
            context.physical_by_robot[robot.name] = regions
            for region_id in regions:
                context.occupancy_by_region.setdefault(
                    region_id,
                    set(),
                ).add(robot.name)
        self._controlled_corridor_occupancy = {
            region_id: sorted(robot_names)
            for region_id, robot_names
            in context.occupancy_by_region.items()
        }

    def _controlled_corridor_physical_regions(
        self,
        context: _CentralCorridorBuild,
        robot: FleetRobot,
    ) -> set[str]:
        """Project route and footprint evidence into authored resources."""
        controlled_regions = set(
            context.scheduler.controlled_regions
        )
        route_regions = (
            self._controlled_regions_for_robot(robot)
            & controlled_regions
        )
        footprint_regions = (
            self._controlled_regions_intersecting_footprint(robot)
            & controlled_regions
        )
        previous_slot = (
            context.old_schedule.slot_for(robot.name)
            if context.old_schedule is not None
            else None
        )
        regions = set(route_regions)
        if route_regions:
            # Once the centre is controlled, protect every touched rectangle.
            regions.update(footprint_regions)
        elif (
            previous_slot is not None
            and previous_slot.past_commit_point
        ):
            # At the external exit retain only the physical tail of the
            # completed passage, not an unrelated neighbouring rectangle.
            regions.update(
                footprint_regions.intersection(previous_slot.regions)
            )
        # Footprint overlap at an external holding LM is not proof of entry;
        # graph crossing or an already-entered slot remains authoritative.
        return regions

    def _collect_controlled_corridor_requests(
        self,
        context: _CentralCorridorBuild,
    ) -> None:
        starvation = max(
            1.0,
            self._controlled_corridor_param(
                "controlled_corridor_starvation_sec",
                8.0,
            ),
        )
        for robot in self._runtime_robots():
            self._collect_controlled_corridor_robot_request(
                context,
                robot,
                starvation=starvation,
            )
        context.starvation = starvation

    def _collect_controlled_corridor_robot_request(
        self,
        context: _CentralCorridorBuild,
        robot: FleetRobot,
        *,
        starvation: float,
    ) -> None:
        """Append one executable robot's immediate passage request."""
        if (
            robot.status not in {"MOVING", "WAITING"}
            or not robot.trajectory
            or (
                robot.name not in context.physical_by_robot
                and self._retained_route_is_superseded(robot)
            )
        ):
            return
        entry = self._next_controlled_corridor_entry(robot)
        if entry is None:
            return
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            not regions
            or not set(regions).issubset(
                context.scheduler.controlled_regions
            )
        ):
            return
        exit_lm = str(entry.get("exit_lm") or "")
        if exit_lm not in self.landmarks:
            return
        direction = self._controlled_corridor_flow_direction(entry)
        wait_key = (
            str(entry.get("passage") or "|".join(regions)),
            direction,
            int(robot.route_revision),
            robot.name,
        )
        context.active_wait_keys.add(wait_key)
        wait_since = self._controlled_corridor_wait_since.setdefault(
            wait_key,
            context.now,
        )
        entry_clock = float(
            entry.get("entry_clock", robot.route_clock)
            or robot.route_clock
        )
        raw_staging_clock = entry.get("staging_clock")
        staging_clock = min(
            entry_clock,
            float(
                entry_clock
                if raw_staging_clock is None
                else raw_staging_clock
            ),
        )
        exit_clock = max(
            entry_clock + self._runtime_motion_step(),
            float(entry.get("exit_clock", entry_clock) or entry_clock),
        )
        blocker = self._controlled_corridor_downstream_blocker(
            robot,
            exit_lm,
            exit_clock,
        )
        if blocker:
            context.downstream_blockers[robot.name] = blocker
        order = self._active_order_for_robot(robot)
        predecessor_robot_id = (
            self._controlled_corridor_queue_predecessor(
                robot,
                entry,
                regions,
                direction,
            )
        )
        physical_regions = context.physical_by_robot.get(
            robot.name,
            set(),
        )
        request = CorridorRequest(
            robot_id=robot.name,
            regions=regions,
            direction=direction,
            earliest_entry=context.now
            + max(
                0.0,
                staging_clock - float(robot.route_clock),
            ),
            duration_sec=max(
                self._runtime_motion_step(),
                exit_clock - staging_clock,
            ),
            staging_lm=str(
                entry.get("holding_lm")
                or entry.get("src")
                or ""
            ),
            exit_lm=exit_lm,
            route_revision=int(robot.route_revision),
            priority=float(order.priority if order is not None else 0),
            wait_age_sec=max(0.0, context.now - wait_since),
            deadline=wait_since + starvation,
            downstream_available=not bool(blocker),
            predecessor_robot_id=predecessor_robot_id or None,
            entered=bool(physical_regions.intersection(regions)),
            past_commit_point=bool(
                physical_regions.intersection(regions)
                or (
                    bool(entry.get("passed_staging"))
                    and not bool(entry.get("has_wait_after_staging"))
                )
            ),
            resource_windows=tuple(
                window
                for window in entry.get("resource_windows", ())
                if isinstance(window, CorridorResourceWindow)
            ),
        )
        context.requests.append(request)
        context.entries_by_robot[robot.name] = entry

    def _collect_controlled_corridor_prefetch_requests(
        self,
        context: _CentralCorridorBuild,
    ) -> None:
        # Future rolling chunks are not present in ``robot.trajectory`` yet.
        # Admit their first authored passage from the registered nominal
        # timeline so SIPP receives a red-light time at the external staging
        # LM instead of discovering the conflict inside a no-wait chain.
        for robot_name, intent in list(
            self._controlled_corridor_prefetch_intents.items()
        ):
            self._collect_controlled_corridor_prefetch_request(
                context,
                robot_name,
                intent,
            )

    def _collect_controlled_corridor_prefetch_request(
        self,
        context: _CentralCorridorBuild,
        robot_name: str,
        intent: Any,
    ) -> None:
        """Append one current future-passage proposal to the calendar."""
        robot = self.robots.get(robot_name)
        order = (
            self.orders.get(str(intent.get("order_id") or ""))
            if isinstance(intent, dict)
            else None
        )
        raw_request = (
            intent.get("request")
            if isinstance(intent, dict)
            else None
        )
        signature = (
            intent.get("signature")
            if isinstance(intent, dict)
            else None
        )
        if (
            robot is None
            or order is None
            or not isinstance(intent, dict)
            or not self._controlled_corridor_intent_is_current(
                robot,
                order,
                intent,
            )
            or not isinstance(raw_request, CorridorRequest)
            or not isinstance(signature, tuple)
        ):
            self._controlled_corridor_prefetch_intents.pop(
                robot_name,
                None,
            )
            return
        previous_slot = (
            context.old_schedule.slot_for(robot_name)
            if context.old_schedule is not None
            else None
        )
        if (
            robot_name in context.entries_by_robot
            or robot_name in context.physical_by_robot
            or (
                previous_slot is not None
                and previous_slot.state
                is CorridorSlotState.COMMITTED
                and previous_slot.exit_time > context.now
            )
        ):
            # One robot can own only its immediate passage.
            intent["last_schedule_epoch"] = None
            return
        entry = intent.get("entry")
        if not isinstance(entry, dict):
            self._controlled_corridor_prefetch_intents.pop(
                robot_name,
                None,
            )
            return
        handoff_at = float(
            intent.get("handoff_at", context.now)
            or context.now
        )
        synthetic = FleetRobot(
            name=robot.name,
            current_lm=str(intent.get("start_lm") or ""),
            target_lm=raw_request.exit_lm,
            status="MOVING",
            active_order_id=order.order_id,
            pose=(
                dict(intent["start_pose"])
                if isinstance(intent.get("start_pose"), dict)
                else self._pose_at_landmark(
                    str(intent.get("start_lm") or "")
                )
            ),
            trajectory=[
                dict(sample)
                for sample in intent.get("trajectory", ())
                if isinstance(sample, dict)
            ],
            route_clock=-max(0.0, handoff_at - context.now),
            route_revision=int(robot.route_revision),
        )
        blocker = self._controlled_corridor_downstream_blocker(
            synthetic,
            raw_request.exit_lm,
            float(
                entry.get(
                    "exit_clock",
                    raw_request.duration_sec,
                )
                or raw_request.duration_sec
            ),
        )
        if blocker:
            context.downstream_blockers[robot_name] = blocker
        wait_since = float(
            intent.get("registered_at", context.now)
            or context.now
        )
        request = replace(
            raw_request,
            wait_age_sec=max(0.0, context.now - wait_since),
            deadline=wait_since + context.starvation,
            downstream_available=not bool(blocker),
        )
        intent["request"] = request
        context.active_wait_keys.add(
            (
                str(
                    entry.get("passage")
                    or "|".join(request.regions)
                ),
                request.direction,
                int(request.route_revision),
                robot_name,
            )
        )
        context.requests.append(request)
        context.entries_by_robot[robot_name] = entry
        context.scheduled_intent_names.add(robot_name)

    def _maintain_controlled_corridor_waits(
        self,
        context: _CentralCorridorBuild,
    ) -> None:
        now = context.now
        old_schedule = context.old_schedule
        physical_by_robot = context.physical_by_robot
        active_wait_keys = context.active_wait_keys
        downstream_blockers = context.downstream_blockers
        for key in list(self._controlled_corridor_wait_since):
            if key not in active_wait_keys:
                self._controlled_corridor_wait_since.pop(key, None)

        # A physical owner can be close enough to its safe exit that
        # ``_next_controlled_corridor_entry`` correctly reports no *future*
        # passage. It still needs the exit pocket protected until its complete
        # body leaves. Recheck the retained physical passage here so local
        # arbitration knows that the external body must clear first.
        for robot_name in physical_by_robot:
            if robot_name in downstream_blockers:
                continue
            robot = self.robots.get(robot_name)
            if robot is None:
                continue
            previous_slot = (
                old_schedule.slot_for(robot_name)
                if old_schedule is not None
                else None
            )
            previous_passage = self._controlled_corridor_passages.get(
                robot_name,
                {},
            )
            exit_lm = str(
                (previous_slot.exit_lm if previous_slot is not None else "")
                or previous_passage.get("exit_lm")
                or ""
            )
            if exit_lm not in self.landmarks:
                continue
            exit_clock = float(robot.route_clock)
            for sample in robot.trajectory:
                sample_clock = float(sample.get("t", 0.0) or 0.0)
                if sample_clock + 0.000001 < robot.route_clock:
                    continue
                if str(sample.get("lm") or "") != exit_lm:
                    continue
                exit_clock = sample_clock
                break
            blocker = self._controlled_corridor_downstream_blocker(
                robot,
                exit_lm,
                exit_clock,
            )
            if blocker:
                downstream_blockers[robot_name] = blocker

__all__ = ["ControlledCorridorRequestCollectionMixin"]
