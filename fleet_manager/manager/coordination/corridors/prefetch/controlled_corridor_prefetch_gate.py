"""Resolve scheduled gates and safe corridor approaches."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorRequest,
    CorridorSlot,
)


class ControlledCorridorPrefetchGateMixin:
    """Resolve scheduled gates and safe corridor approaches."""

    def _controlled_corridor_prefetch_gate(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        *,
        prediction_offset: float,
        now: float,
    ) -> dict[str, Any] | None:
        """Return a scheduled start delay, or an unready authored intent."""
        scheduler = self._controlled_corridor_scheduler
        if scheduler is None:
            self._controlled_corridor_prefetch_intents.pop(robot.name, None)
            return None
        schedule = self._controlled_corridor_schedule
        existing_intent = (
            self._controlled_corridor_prefetch_intents.get(robot.name)
        )
        current_slot = (
            schedule.slot_for(robot.name)
            if schedule is not None
            else None
        )
        slot_belongs_to_intent = (
            self._controlled_corridor_slot_belongs_to_intent(
                schedule,
                current_slot,
                existing_intent,
            )
        )
        if self._controlled_corridor_live_passage_blocks_prefetch(
            robot,
            scheduler,
        ):
            # The calendar currently has one transaction id per physical
            # robot.  Its immediate committed/approaching passage must remain
            # authoritative; a later rolling chunk cannot overwrite that
            # slot.  Plan the continuation against normal SIPP reservations
            # now. Once the current passage exits, the appended trajectory
            # becomes the live central request and receives its own gate
            # before reaching the next stop line.
            self._controlled_corridor_prefetch_intents.pop(robot.name, None)
            return None
        if (
            isinstance(current_slot, CorridorSlot)
            and not slot_belongs_to_intent
        ):
            # A previous passage still occupies this robot's single calendar
            # transaction.  Do not bypass central admission: register the new
            # intent and let the next physical snapshot retire the old slot.
            # Returning ``None`` here used to send fresh orders straight into
            # MAPF without a corridor command.
            intent = self._controlled_corridor_prefetch_intent(
                order,
                robot,
                request,
                prediction_offset=prediction_offset,
                now=now,
            )
            return (
                {"ready": False, "intent": intent}
                if intent is not None
                else None
            )
        intent = self._controlled_corridor_prefetch_intent(
            order,
            robot,
            request,
            prediction_offset=prediction_offset,
            now=now,
        )
        if intent is None:
            return None
        return self._controlled_corridor_scheduled_gate(
            robot,
            request,
            intent,
            prediction_offset=prediction_offset,
            now=now,
        )

    @staticmethod
    def _controlled_corridor_slot_belongs_to_intent(
        schedule: Any,
        current_slot: Any,
        intent: Any,
    ) -> bool:
        """Return whether a live calendar slot came from this exact intent."""
        corridor_request = (
            intent.get("request")
            if isinstance(intent, dict)
            else None
        )
        return bool(
            isinstance(corridor_request, CorridorRequest)
            and isinstance(current_slot, CorridorSlot)
            and intent.get("last_schedule_epoch") == schedule.epoch
            and current_slot.regions == corridor_request.regions
            and current_slot.direction == corridor_request.direction
            and current_slot.staging_lm == corridor_request.staging_lm
            and current_slot.exit_lm == corridor_request.exit_lm
        )

    def _controlled_corridor_live_passage_blocks_prefetch(
        self,
        robot: FleetRobot,
        scheduler: Any,
    ) -> bool:
        """Keep a physical passage authoritative over a later rolling chunk."""
        live_entry = self._next_controlled_corridor_entry(robot)
        live_regions = self._controlled_corridor_entry_regions(live_entry)
        return bool(
            live_regions
            and set(live_regions).issubset(scheduler.controlled_regions)
        )

    def _controlled_corridor_scheduled_gate(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        intent: dict[str, Any],
        *,
        prediction_offset: float,
        now: float,
    ) -> dict[str, Any]:
        """Resolve one authored intent against the latest schedule snapshot."""
        schedule = self._controlled_corridor_schedule
        corridor_request = intent.get("request")
        if (
            schedule is None
            or not isinstance(corridor_request, CorridorRequest)
            or intent.get("last_schedule_epoch") != schedule.epoch
        ):
            return {"ready": False, "intent": intent}
        slot = schedule.slot_for(robot.name)
        if slot is None:
            # The finite calendar may not yet contain this passage, but free
            # space before its stop line is not part of the controlled
            # corridor. Let the normal MAPF stack move the robot there instead
            # of keeping a whole dispatch wave parked at its spawn positions.
            approach_gate = self._corridor_approach_gate(
                robot,
                request,
                intent,
            )
            return (
                approach_gate
                if approach_gate is not None
                else {"ready": False, "intent": intent}
            )
        if (
            slot.regions != corridor_request.regions
            or slot.direction != corridor_request.direction
            or slot.staging_lm != corridor_request.staging_lm
            or slot.exit_lm != corridor_request.exit_lm
            or int(slot.route_revision) != int(robot.route_revision)
        ):
            return {"ready": False, "intent": intent}
        entry = intent.get("entry")
        staging_clock = (
            float(entry.get("staging_clock", 0.0) or 0.0)
            if isinstance(entry, dict)
            else 0.0
        )
        nominal_staging_at = (
            now
            + max(0.0, float(prediction_offset))
            + max(0.0, staging_clock)
        )
        timing_tolerance = max(
            self._runtime_motion_step(),
            float(getattr(self.planner, "time_step_sec", 0.2) or 0.2),
        )
        if float(slot.entry_time) < nominal_staging_at - timing_tolerance:
            return self._controlled_corridor_missed_slot_gate(
                robot,
                request,
                intent,
                corridor_request,
                nominal_staging_at=nominal_staging_at,
                prediction_offset=prediction_offset,
                now=now,
            )
        planning_start_at = (
            now + max(0.0, float(prediction_offset))
        )
        may_enter, horizon_gate = self._controlled_corridor_horizon_gate(
            robot,
            request,
            intent,
            corridor_request,
            slot,
            planning_start_at=planning_start_at,
            timing_tolerance=timing_tolerance,
        )
        if not may_enter:
            return horizon_gate
        departure_not_before = max(
            0.0,
            float(slot.entry_time)
            - planning_start_at,
        )
        return {
            "ready": True,
            "intent": intent,
            "slot": slot,
            # The red light belongs to the external corridor stop line, not
            # necessarily to the beginning of this rolling chunk.  Holding a
            # robot at its route start wastes all free approach capacity and
            # was the main reason a distant entrant could freeze two nearby
            # robots.  SIPP now carries this absolute route-clock constraint
            # to the staging LM and lets the robot approach it normally.
            "departureNotBefore": {
                "node": corridor_request.staging_lm,
                "timeSec": departure_not_before,
            },
            # The backed-off stop line is the last legal waiting point. SIPP
            # may rotate while traversing the passage, but any traffic delay
            # after this LM must be moved back to the stop line.
            "noWaitNodes": list(
                entry.get("no_wait_lms", ())
                if isinstance(entry, dict)
                else ()
            ),
        }

    def _controlled_corridor_missed_slot_gate(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        intent: dict[str, Any],
        corridor_request: CorridorRequest,
        *,
        nominal_staging_at: float,
        prediction_offset: float,
        now: float,
    ) -> dict[str, Any]:
        """Release a safe approach or invalidate one expired green light."""
        approach_gate = self._corridor_approach_gate(robot, request, intent)
        if approach_gate is not None:
            return approach_gate
        intent["request"] = replace(
            corridor_request,
            earliest_entry=nominal_staging_at,
        )
        intent["handoff_at"] = now + max(0.0, float(prediction_offset))
        intent["last_schedule_epoch"] = None
        return {"ready": False, "intent": intent}

    def _controlled_corridor_horizon_gate(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        intent: dict[str, Any],
        corridor_request: CorridorRequest,
        slot: CorridorSlot,
        *,
        planning_start_at: float,
        timing_tolerance: float,
    ) -> tuple[bool, dict[str, Any] | None]:
        """Resolve slots extending beyond the ordinary rolling horizon."""
        rolling_horizon = self._rolling_horizon()
        if (
            rolling_horizon <= 0.0
            or float(slot.exit_time)
            <= planning_start_at + rolling_horizon + timing_tolerance
        ):
            return True, None
        approach_gate = self._corridor_approach_gate(robot, request, intent)
        if approach_gate is not None:
            return False, approach_gate
        if str(request.get("startLm") or "") != corridor_request.staging_lm:
            return False, {"ready": False, "intent": intent}
        # A no-wait passage is indivisible once the robot reaches its external
        # stop line, even when it is longer than the ordinary rolling horizon.
        return True, None

    def _corridor_approach_gate(
        self,
        robot: FleetRobot,
        request: dict[str, Any],
        intent: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Release a corridor-free prefix while retaining the stable route."""
        if not self._prepare_corridor_approach_request(
            request,
            intent,
            robot=robot,
        ):
            return None
        corridor_request = intent.get("request")
        return {
            "ready": True,
            "approachOnly": True,
            "holdingLm": str(request.get("goalLm") or ""),
            "stagingLm": (
                corridor_request.staging_lm
                if isinstance(corridor_request, CorridorRequest)
                else str(request.get("goalLm") or "")
            ),
        }

    def _prepare_corridor_approach_request(
        self,
        request: dict[str, Any],
        intent: dict[str, Any],
        *,
        robot: FleetRobot | None = None,
    ) -> bool:
        """Trim one request to its assigned safe approach-queue LM.

        The stable order route is deliberately not changed.  Only this temporal
        chunk ends at one graph-safe holding point. The first robot waits at
        the corridor stop line, the next robot waits at the preceding free LM,
        and so on. This prevents many independently planned chunks from
        converging on the same red-light vertex and blocking the corridor exit.
        """
        corridor_request = intent.get("request")
        entry = intent.get("entry")
        if (
            not isinstance(corridor_request, CorridorRequest)
            or not isinstance(entry, dict)
        ):
            return False
        start_lm = str(request.get("startLm") or "")
        staging_lm = corridor_request.staging_lm
        if not start_lm:
            return False
        route_nodes = [
            str(node)
            for node in request.get("routeNodes", ())
            if str(node) in self.landmarks
        ]
        if len(route_nodes) < 2 or route_nodes[0] != start_lm:
            return False

        # The same LM can occur more than once on a constrained recovery route.
        # Select the occurrence belonging to the first scheduled passage: the
        # last staging occurrence no later than its entry portal.
        portal_lm = str(entry.get("src") or "")
        portal_indices = [
            index
            for index, node in enumerate(route_nodes)
            if node == portal_lm
        ]
        portal_index = portal_indices[0] if portal_indices else len(route_nodes)
        staging_indices = [
            index
            for index, node in enumerate(route_nodes[: portal_index + 1])
            if node == staging_lm
        ]
        if not staging_indices:
            return False
        staging_index = staging_indices[-1]
        holding_lm = staging_lm
        if robot is not None:
            holding_lm = self._controlled_corridor_approach_holding_lm(
                robot,
                route_nodes=route_nodes,
                staging_index=staging_index,
                staging_lm=staging_lm,
                intent=intent,
            )
        if not holding_lm or holding_lm == start_lm:
            return False
        holding_indices = [
            index
            for index, node in enumerate(route_nodes[: staging_index + 1])
            if node == holding_lm
        ]
        if not holding_indices:
            return False
        holding_index = holding_indices[-1]
        if holding_index <= 0:
            return False
        request["goalLm"] = holding_lm
        request["routeNodes"] = route_nodes[: holding_index + 1]
        request.pop("departureNotBefore", None)
        return True

    def _controlled_corridor_approach_holding_lm(
        self,
        robot: FleetRobot,
        *,
        route_nodes: list[str],
        staging_index: int,
        staging_lm: str,
        intent: dict[str, Any],
    ) -> str:
        """Reserve the closest unclaimed safe LM in one portal queue.

        This queue is intentionally graph-based and map-independent. Only
        corridors explicitly authored in the editor call it; ordinary open
        space continues to use congestion A* and Rolling SIPP unchanged.
        """
        for robot_name, assignment in list(
            self._controlled_corridor_approach_holds.items()
        ):
            owner = self.robots.get(robot_name)
            live_intent = self._controlled_corridor_prefetch_intents.get(
                robot_name
            )
            assignment_order_id = str(
                assignment.get("order_id") or ""
            )
            assignment_signature = assignment.get("intent_signature")
            intent_still_current = bool(
                isinstance(live_intent, dict)
                and live_intent.get("signature") == assignment_signature
                and str(live_intent.get("order_id") or "")
                == assignment_order_id
            )
            route_still_executing = bool(
                assignment_order_id
                and owner is not None
                and owner.active_order_id == assignment_order_id
            )
            if (
                owner is None
                or int(assignment.get("route_revision", -1))
                != int(owner.route_revision)
                or not (
                    intent_still_current
                    or route_still_executing
                )
            ):
                self._controlled_corridor_approach_holds.pop(
                    robot_name,
                    None,
                )

        reserved_lms = {
            str(assignment.get("lm") or "")
            for robot_name, assignment
            in self._controlled_corridor_approach_holds.items()
            if robot_name != robot.name
            and str(assignment.get("staging_lm") or "") == staging_lm
        }
        # A robot with an already committed live corridor route may wait at
        # the same external stop line even though it no longer owns an
        # approach-only assignment. Keep that cell for the leader until its
        # live passage advances.
        for robot_name, passage in self._controlled_corridor_passages.items():
            if robot_name == robot.name:
                continue
            if (
                str(passage.get("staging_lm") or "") == staging_lm
                and bool(passage.get("committed"))
                and not bool(passage.get("entered"))
            ):
                reserved_lms.add(staging_lm)

        graph = self._controlled_corridor_graph
        candidates: list[str] = []
        seen: set[str] = set()
        for node in reversed(route_nodes[: staging_index + 1]):
            if node in seen:
                continue
            seen.add(node)
            vertex = graph.vertices.get(node) if graph is not None else None
            if (
                vertex is not None
                and vertex.can_wait
                and not vertex.controlled_region_ids
            ):
                candidates.append(node)
        if not candidates:
            candidates = [staging_lm]

        holding_lm = next(
            (
                candidate
                for candidate in candidates
                if candidate not in reserved_lms
            ),
            str(route_nodes[0] if route_nodes else ""),
        )
        self._controlled_corridor_approach_holds[robot.name] = {
            "lm": holding_lm,
            "staging_lm": staging_lm,
            "route_revision": int(robot.route_revision),
            "order_id": str(
                intent.get("order_id")
                or robot.active_order_id
                or ""
            ),
            "intent_signature": intent.get("signature"),
            "assigned_at": self._now(),
        }
        return holding_lm
