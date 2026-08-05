"""Revalidate, pin, commit and reject prefetched slots."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSlot,
    CorridorSlotState,
)
from fleet_manager.manager.coordination.corridors.prefetch.controlled_corridor_prefetch_models import (
    _CorridorPlannedPassage,
    _CorridorValidationContext,
)


class ControlledCorridorPrefetchValidationMixin:
    """Revalidate, pin, commit and reject prefetched slots."""

    def _controlled_corridor_prefetch_plan_is_current(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        plan: dict[str, Any],
        gate: dict[str, Any],
        *,
        now: float,
    ) -> tuple[bool, str]:
        """Revalidate one prefetched passage against the live calendar.

        MAPF runs outside the runtime tick.  While it is computing, a physical
        occupant may delay or replace the tentative corridor slot.  A route is
        appendable only when its actual first authored passage still fits the
        current immutable slot.  This is the commit-side half of the central
        traffic contract; a rejected result is simply rescheduled and is not
        counted as a planner failure.
        """
        if self._controlled_corridor_scheduler is None:
            return True, ""
        context, reason = self._controlled_corridor_validation_context(
            robot,
            gate,
        )
        if context is None:
            return False, reason
        passage, reason = self._controlled_corridor_planned_passage(
            robot,
            request,
            plan,
            context,
            now=now,
        )
        if passage is None:
            return False, reason
        gate["actual_slot"] = CorridorSlot(
            robot_id=robot.name,
            regions=passage.regions,
            direction=context.request.direction,
            entry_time=passage.actual_staging_at,
            exit_time=(
                passage.actual_staging_at
                + passage.exit_clock
                - passage.staging_clock
            ),
            staging_lm=context.request.staging_lm,
            exit_lm=context.request.exit_lm,
            route_revision=int(robot.route_revision),
            state=CorridorSlotState.COMMITTED,
            resource_windows=passage.resource_windows,
            past_commit_point=False,
            physically_observed=False,
        )
        return True, ""

    def _controlled_corridor_validation_context(
        self,
        robot: FleetRobot,
        gate: dict[str, Any],
    ) -> tuple[_CorridorValidationContext | None, str]:
        """Capture one still-current intent and its latest calendar slot."""
        intent = gate.get("intent")
        signature = gate.get("signature")
        scheduled_slot = gate.get("slot")
        current_intent = self._controlled_corridor_prefetch_intents.get(
            robot.name
        )
        order = (
            self.orders.get(str(intent.get("order_id") or ""))
            if isinstance(intent, dict)
            else None
        )
        if (
            not isinstance(intent, dict)
            or not isinstance(signature, tuple)
            or not isinstance(scheduled_slot, CorridorSlot)
            or current_intent is not intent
            or intent.get("signature") != signature
            or order is None
            or not self._controlled_corridor_intent_is_current(
                robot,
                order,
                intent,
            )
        ):
            return None, "corridor intent changed while planning"

        schedule = self._controlled_corridor_schedule
        current_slot = (
            schedule.slot_for(robot.name)
            if schedule is not None
            else None
        )
        corridor_request = intent.get("request")
        if (
            not isinstance(current_slot, CorridorSlot)
            or not isinstance(corridor_request, CorridorRequest)
            or current_slot.regions != corridor_request.regions
            or current_slot.direction != corridor_request.direction
            or current_slot.staging_lm != corridor_request.staging_lm
            or current_slot.exit_lm != corridor_request.exit_lm
            or int(current_slot.route_revision) != int(robot.route_revision)
        ):
            return None, "corridor slot is no longer available"
        return (
            _CorridorValidationContext(
                intent=intent,
                request=corridor_request,
                slot=current_slot,
            ),
            "",
        )

    def _controlled_corridor_planned_passage(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        plan: dict[str, Any],
        context: _CorridorValidationContext,
        *,
        now: float,
    ) -> tuple[_CorridorPlannedPassage | None, str]:
        """Reconstruct and validate the first authored passage in a MAPF plan."""
        trajectory = [
            dict(sample)
            for sample in plan.get("trajectory", ())
            if isinstance(sample, dict)
        ]
        if len(trajectory) < 2:
            return None, "corridor plan has no executable trajectory"
        start_lm = str(request.get("startLm") or "")
        start_pose = request.get("startPose")
        pose = (
            {
                "x": float(start_pose.get("x", 0.0) or 0.0),
                "y": float(start_pose.get("y", 0.0) or 0.0),
                "yaw": float(start_pose.get("yaw", 0.0) or 0.0),
            }
            if isinstance(start_pose, dict)
            else self._pose_at_landmark(start_lm)
        )
        synthetic = FleetRobot(
            name=robot.name,
            current_lm=start_lm,
            target_lm=str(plan.get("goalLm") or ""),
            status="MOVING",
            active_order_id=robot.active_order_id,
            pose=pose,
            trajectory=trajectory,
            route_clock=0.0,
            route_revision=int(robot.route_revision),
        )
        entry = self._next_controlled_corridor_entry(
            synthetic,
            # A central gate can intentionally add a long wait at the stop
            # line. Commit validation must inspect the complete returned
            # command, not apply the discovery lookahead a second time and
            # mistake a delayed passage for a changed route.
            lookahead_sec=float("inf"),
        )
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            entry is None
            or regions != context.request.regions
            or self._controlled_corridor_flow_direction(entry)
            != context.request.direction
            or str(entry.get("holding_lm") or entry.get("src") or "")
            != context.request.staging_lm
            or str(entry.get("exit_lm") or "") != context.request.exit_lm
        ):
            return None, "MAPF result changed the scheduled corridor passage"
        if bool(entry.get("has_wait_after_staging")):
            return None, "MAPF result waits after corridor commit point"

        current_end = (
            float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            if robot.trajectory
            else float(robot.route_clock)
        )
        handoff_delay = max(
            0.0,
            current_end - float(robot.route_clock),
        )
        staging_clock = max(
            0.0,
            float(entry.get("staging_clock", 0.0) or 0.0),
        )
        actual_staging_at = now + handoff_delay + staging_clock
        plan_windows = tuple(
            window
            for window in entry.get("resource_windows", ())
            if isinstance(window, CorridorResourceWindow)
        )
        if {
            window.region_id for window in plan_windows
        } != set(context.slot.regions):
            return None, "corridor resource set changed while planning"
        exit_clock = max(
            staging_clock + self._runtime_motion_step(),
            float(
                entry.get("exit_clock", staging_clock)
                or staging_clock
            ),
        )
        return (
            _CorridorPlannedPassage(
                entry=entry,
                regions=regions,
                resource_windows=plan_windows,
                actual_staging_at=actual_staging_at,
                staging_clock=staging_clock,
                exit_clock=exit_clock,
            ),
            "",
        )

    def _commit_controlled_corridor_prefetch_slot(
        self,
        robot: FleetRobot,
        gate: dict[str, Any],
    ) -> bool:
        """Atomically retain a validated slot across route-revision handoff."""
        scheduler = self._controlled_corridor_scheduler
        schedule = self._controlled_corridor_schedule
        intent = gate.get("intent")
        if (
            scheduler is None
            or schedule is None
            or not isinstance(intent, dict)
            or self._controlled_corridor_prefetch_intents.get(robot.name)
            is not intent
        ):
            return False
        expected_slot = gate.get("slot")
        if not isinstance(expected_slot, CorridorSlot):
            return False
        actual_slot = gate.get("actual_slot")
        if not isinstance(actual_slot, CorridorSlot):
            actual_slot = expected_slot
        committed = scheduler.commit_slot(
            robot.name,
            # Commit the exact proposal captured before MAPF started.  Passing
            # the current slot here would let a stale worker validate a
            # different calendar decision after its original tentative slot
            # was moved or displaced.
            expected=expected_slot,
            actual=actual_slot,
        )
        if committed is None:
            return False
        self._controlled_corridor_schedule = committed
        intent["last_schedule_epoch"] = committed.epoch
        return True

    def _pin_controlled_corridor_gates(
        self,
        gates: dict[str, dict[str, Any]],
    ) -> bool:
        """Lease every captured tentative slot to one in-flight MAPF job."""
        scheduler = self._controlled_corridor_scheduler
        if not gates:
            return True
        if scheduler is None:
            return False
        pinned: list[tuple[str, CorridorSlot]] = []
        for robot_name, gate in gates.items():
            slot = gate.get("slot") if isinstance(gate, dict) else None
            if not isinstance(slot, CorridorSlot) or not scheduler.pin_slot(
                robot_name,
                expected=slot,
            ):
                for pinned_name, pinned_slot in pinned:
                    scheduler.release_slot_pin(
                        pinned_name,
                        expected=pinned_slot,
                    )
                return False
            pinned.append((robot_name, slot))
        return True

    def _release_controlled_corridor_gate_pins(
        self,
        gates: dict[str, dict[str, Any]] | None,
    ) -> None:
        """End worker leases without releasing a newer replacement pin."""
        scheduler = self._controlled_corridor_scheduler
        if scheduler is None or not isinstance(gates, dict):
            return
        for robot_name, gate in gates.items():
            slot = gate.get("slot") if isinstance(gate, dict) else None
            if isinstance(slot, CorridorSlot):
                scheduler.release_slot_pin(
                    robot_name,
                    expected=slot,
                )

    def _handle_controlled_corridor_gate_rejection(
        self,
        robot_name: str,
        gate: dict[str, Any],
        reason: str,
    ) -> None:
        """Keep queue age across temporary calendar displacement."""
        intent = gate.get("intent")
        current = self._controlled_corridor_prefetch_intents.get(robot_name)
        if current is not intent or not isinstance(intent, dict):
            return
        if reason in {
            "corridor slot is no longer available",
            "corridor slot changed before command commit",
        }:
            # The route proposal is still valid; only its tentative calendar
            # position changed.  Preserve ``registered_at`` so repeated
            # displacement cannot starve this robot, and ask the next scheduler
            # snapshot to assign a fresh slot.  When SIPP reached the same
            # passage later than the nominal calendar proposal, keep that
            # validated ETA/resource template as the new lower bound. Without
            # this handoff the scheduler offered the same too-early slot and
            # the worker reproduced the same committed-slot conflict forever.
            corridor_request = intent.get("request")
            actual_slot = gate.get("actual_slot")
            if (
                isinstance(corridor_request, CorridorRequest)
                and isinstance(actual_slot, CorridorSlot)
                and actual_slot.robot_id == robot_name
                and actual_slot.regions == corridor_request.regions
                and actual_slot.direction == corridor_request.direction
                and actual_slot.staging_lm == corridor_request.staging_lm
                and actual_slot.exit_lm == corridor_request.exit_lm
            ):
                intent["request"] = replace(
                    corridor_request,
                    earliest_entry=max(
                        corridor_request.earliest_entry,
                        actual_slot.entry_time,
                    ),
                    duration_sec=max(
                        self._runtime_motion_step(),
                        actual_slot.duration_sec,
                    ),
                    resource_windows=actual_slot.resource_windows,
                )
            intent["last_schedule_epoch"] = None
            return
        self._controlled_corridor_prefetch_intents.pop(robot_name, None)
