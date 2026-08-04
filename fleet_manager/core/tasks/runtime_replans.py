"""Track runtime and coupled replan state transitions."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from fleet_manager.core.fleet.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot


@dataclass(slots=True)
class _RuntimeReplanFailure:
    """Evidence and mutations for one failed replacement transaction."""

    order: FleetOrder
    robot: FleetRobot
    state: dict[str, Any]
    reason: str
    now: float
    failures: int
    failure_text: str
    identical_failures: int
    original_reason: str
    stationary_debug: dict[str, Any]
    dynamic_conflict_signature: tuple[Any, ...]
    reservation_conflict_signature: tuple[Any, ...]
    causal_blocker_names: set[str] = field(default_factory=set)
    blocker_names: set[str] = field(default_factory=set)


class RuntimeReplanMixin:
    """Track runtime and coupled replan state transitions."""

    def _runtime_replan_state_is_current(
        self,
        robot: FleetRobot,
        state: dict[str, Any],
        *,
        allowed_stages: set[str] | None = None,
    ) -> bool:
        """Return whether a saved replacement still describes this robot."""
        order = self.orders.get(str(state.get("order_id") or ""))
        stage = str(state.get("stage") or "queued")
        return bool(
            order is not None
            and order.status not in TERMINAL_ORDER_STATUSES
            and not robot.is_remote()
            and robot.active_order_id == order.order_id
            and int(state.get("route_revision", -1))
            == int(robot.route_revision)
            and abs(
                float(state.get("route_clock", 0.0) or 0.0)
                - float(robot.route_clock)
            )
            <= 0.000001
            and str(state.get("start_lm") or "")
            == self._safe_replan_start_lm(robot)
            and (
                allowed_stages is None
                or stage in allowed_stages
            )
        )

    def _ready_runtime_replan_entry(
        self,
        now: float | None = None,
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any]] | None:
        """Return the oldest valid transactional runtime detour."""
        current_time = self._now() if now is None else float(now)
        ready: list[tuple[float, str, FleetOrder, FleetRobot, dict[str, Any]]] = []
        for robot_name, state in list(self._runtime_replans.items()):
            if not isinstance(state, dict):
                self._runtime_replans.pop(robot_name, None)
                continue
            robot = self.robots.get(robot_name)
            order = self.orders.get(str(state.get("order_id") or ""))
            start_lm = str(state.get("start_lm") or "")
            if (
                robot is None
                or order is None
                or not self._runtime_replan_state_is_current(robot, state)
            ):
                self._runtime_replans.pop(robot_name, None)
                continue
            corridor_graph = self._controlled_corridor_graph
            corridor_vertex = (
                corridor_graph.vertices.get(start_lm)
                if corridor_graph is not None
                else None
            )
            fixed_escape_route = self._runtime_replan_fixed_escape_route(
                state,
                start_lm,
            )
            escape_vertex = (
                corridor_graph.vertices.get(fixed_escape_route[-1])
                if corridor_graph is not None and fixed_escape_route
                else None
            )
            fixed_escape_leaves_corridor = bool(
                fixed_escape_route
                and escape_vertex is not None
                and escape_vertex.can_wait
                and not escape_vertex.controlled_region_ids
            )
            if (
                corridor_vertex is not None
                and corridor_vertex.controlled_region_ids
                and not fixed_escape_leaves_corridor
            ):
                # A map/corridor update may make a previously external LM an
                # internal no-wait point. Keep an ordinary committed route and
                # let corridor recovery move it to a legal boundary first. A
                # validated fixed escape is that recovery: it starts inside
                # the resource specifically to finish at an external wait LM.
                self._runtime_replans.pop(robot_name, None)
                continue
            stage = str(state.get("stage") or "queued")
            if stage == "planning":
                continue
            if stage == "deadlock_escalated":
                if not self._runtime_replan_escalation_blocker_changed(
                    state
                ):
                    # This transaction is intentionally visible to the
                    # wait-for graph. Re-running the identical planner request
                    # would only consume the shared worker while CBS/retreat
                    # is resolving the coupled component.
                    continue
                state["stage"] = "queued"
                state["retry_at"] = current_time
                state["failures"] = 0
                state.pop("escalated_at", None)
                state.pop("escalated_blocker", None)
                state.pop("escalated_blocker_order_id", None)
                state.pop("escalated_resource", None)
                state.pop("escalation_blocker_signature", None)
                state.pop("escalation_signature_kind", None)
                self._clear_wait_dependency(robot)
                robot.last_reason = (
                    "replanning after traffic blocker progressed"
                )
            corridor_clearance_hold = state.get("corridor_clearance_hold")
            if isinstance(corridor_clearance_hold, dict):
                if self._corridor_clearance_hold_active(
                    corridor_clearance_hold,
                    robot.name,
                ):
                    # This robot is the deliberately evacuated tail of a
                    # portal queue.  Its original task remains assigned, but
                    # the same-goal route may only be recomputed after the
                    # passage owner has physically cleared the local mouth.
                    continue
                state.pop("corridor_clearance_hold", None)
            clearance_blockers = tuple(
                str(name)
                for name in state.get("clearance_blocker_names", ())
                if str(name) in self.robots and str(name) != robot.name
            )
            if (
                clearance_blockers
                and self._active_stationary_clearance_for(clearance_blockers)
            ):
                # Keep the evacuated robot at its safe graph LM until the
                # causal parked body has actually completed its hidden move.
                # Planning against the still-stationary body only burns the
                # single worker and used to launch the known-bad old route.
                continue
            if clearance_blockers:
                state.pop("clearance_blocker_names", None)
            if current_time + 0.000001 < float(
                state.get("retry_at", 0.0) or 0.0
            ):
                continue
            ready.append(
                (
                    float(
                        state.get("last_attempt_at")
                        or state.get("queued_at", current_time)
                        or current_time
                    ),
                    robot.name,
                    order,
                    robot,
                    state,
                )
            )
        if not ready:
            return None
        _, _, order, robot, state = min(ready, key=lambda item: item[:2])
        return order, robot, state

    def _runtime_replan_fixed_escape_route(
        self,
        state: dict[str, Any],
        start_lm: str,
    ) -> list[str]:
        route = [
            str(node)
            for node in state.get("escape_route_nodes", ())
            if str(node) in self.landmarks
        ]
        if (
            len(route) < 2
            or route[0] != start_lm
            or str(state.get("escape_goal") or "") != route[-1]
            or any(
                dst not in self.planner.graph.get(src, [])
                for src, dst in zip(route, route[1:])
            )
        ):
            return []
        return route

    def _defer_runtime_replan(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        state: dict[str, Any],
        reason: str,
        *,
        debug: dict[str, Any] | None = None,
    ) -> None:
        """Keep the old route and retry the replacement with bounded backoff."""
        current = self._runtime_replans.get(robot.name)
        if current is not state:
            return

        failure = self._begin_runtime_replan_failure(
            order,
            robot,
            state,
            reason,
            debug=debug,
        )
        self._resolve_runtime_replan_blockers(failure)
        if self._recover_runtime_replan_stationary_blockers(failure):
            return
        self._escalate_repeated_runtime_replan(failure)
        self._publish_deferred_runtime_replan(failure)

    def _begin_runtime_replan_failure(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        state: dict[str, Any],
        reason: str,
        *,
        debug: dict[str, Any] | None,
    ) -> _RuntimeReplanFailure:
        now = self._now()
        failures = int(state.get("failures", 0) or 0) + 1
        failure_text = str(reason or "")
        identical_failures = (
            int(state.get("identical_failure_count", 0) or 0) + 1
            if str(state.get("last_failure_reason") or "") == failure_text
            else 1
        )
        state["failures"] = failures
        state["identical_failure_count"] = identical_failures
        state["stage"] = "retry"
        state["last_failure_reason"] = failure_text
        state["last_failure_at"] = now
        order.dispatch_failures += 1
        state["retry_at"] = now + self._order_dispatch_retry_interval(order)
        original_reason = str(state.get("reason") or "")
        stationary_debug = dict(debug) if isinstance(debug, dict) else {}
        dynamic_conflict_signature = (
            self._runtime_replan_dynamic_conflict_signature(
                robot,
                stationary_debug,
            )
        )
        if dynamic_conflict_signature:
            previous_signature = state.get("dynamic_conflict_signature")
            repeated_conflicts = (
                int(state.get("dynamic_conflict_count", 0) or 0) + 1
                if previous_signature == dynamic_conflict_signature
                else 1
            )
            state["dynamic_conflict_signature"] = dynamic_conflict_signature
            state["dynamic_conflict_count"] = repeated_conflicts
        else:
            state.pop("dynamic_conflict_signature", None)
            state.pop("dynamic_conflict_count", None)
        reservation_conflict_signature = (
            self._runtime_replan_reservation_conflict_signature(
                robot,
                reason,
                stationary_debug,
            )
        )
        if reservation_conflict_signature:
            previous_signature = state.get(
                "reservation_conflict_signature"
            )
            repeated_conflicts = (
                int(state.get("reservation_conflict_count", 0) or 0) + 1
                if previous_signature == reservation_conflict_signature
                else 1
            )
            state["reservation_conflict_signature"] = (
                reservation_conflict_signature
            )
            state["reservation_conflict_count"] = repeated_conflicts
        else:
            state.pop("reservation_conflict_signature", None)
            state.pop("reservation_conflict_count", None)
        return _RuntimeReplanFailure(
            order=order,
            robot=robot,
            state=state,
            reason=reason,
            now=now,
            failures=failures,
            failure_text=failure_text,
            identical_failures=identical_failures,
            original_reason=original_reason,
            stationary_debug=stationary_debug,
            dynamic_conflict_signature=dynamic_conflict_signature,
            reservation_conflict_signature=reservation_conflict_signature,
        )

    def _resolve_runtime_replan_blockers(
        self,
        failure: _RuntimeReplanFailure,
    ) -> None:
        robot = failure.robot
        state = failure.state
        reason = failure.reason
        original_reason = failure.original_reason
        stationary_debug = failure.stationary_debug
        raw_causal_signatures = state.get("causal_blocker_signatures", ())
        had_causal_signatures = bool(raw_causal_signatures)
        causal_blocker_names: set[str] = set()
        for signature in raw_causal_signatures:
            if not isinstance(signature, (list, tuple)) or len(signature) != 3:
                continue
            blocker_name = str(signature[0])
            captured_lm = str(signature[1])
            try:
                captured_revision = int(signature[2])
            except (TypeError, ValueError):
                continue
            blocker = self.robots.get(blocker_name)
            if (
                blocker is None
                or blocker.name == robot.name
                or blocker.status not in {"IDLE", "ARRIVED"}
                or blocker.trajectory
                or blocker.active_order_id
                or blocker.target_lm
                or int(blocker.route_revision) != captured_revision
                or self._traffic_lm_for_robot(blocker) != captured_lm
            ):
                continue
            causal_blocker_names.add(blocker_name)
        if had_causal_signatures and not causal_blocker_names:
            # The captured body moved or became commanded while this planner
            # attempt was running. Its old identity is no longer proof of the
            # current failure; use the fresh result below.
            state.pop("causal_blocker_signatures", None)
            state.pop("blocker_names", None)
        raw_blockers = stationary_debug.get("stationaryBlockerRobots", [])
        blocker_names = set(causal_blocker_names)
        if not blocker_names:
            blocker_names = {
                str(name)
                for name in (
                    raw_blockers if isinstance(raw_blockers, list) else []
                )
                if str(name) in self.robots and str(name) != robot.name
            }
        if (
            not blocker_names
            and bool(
                stationary_debug.get("stationaryRobotWait")
                or stationary_debug.get("stationaryTurnEnvelopeBlock")
            )
        ):
            unresolved_name = str(
                stationary_debug.get("continuousConflictRobot") or ""
            )
            if unresolved_name in self.robots and unresolved_name != robot.name:
                blocker_names.add(unresolved_name)
        if not blocker_names:
            blocker_names.update({
                str(name)
                for name in state.get("blocker_names", ())
                if str(name) in self.robots and str(name) != robot.name
            })
        if not blocker_names:
            blocker_name = (
                self._robot_name_from_conflict_reason(original_reason)
                or self._robot_name_from_conflict_reason(reason)
            )
            if blocker_name in self.robots and blocker_name != robot.name:
                blocker_names.add(blocker_name)
        failure.causal_blocker_names = causal_blocker_names
        failure.blocker_names = blocker_names

    def _recover_runtime_replan_stationary_blockers(
        self,
        failure: _RuntimeReplanFailure,
    ) -> bool:
        order = failure.order
        robot = failure.robot
        state = failure.state
        reason = failure.reason
        now = failure.now
        stationary_debug = failure.stationary_debug
        causal_blocker_names = failure.causal_blocker_names
        blocker_names = failure.blocker_names
        if blocker_names:
            state["blocker_names"] = tuple(sorted(blocker_names))
            # Planner validation has more precise blocker identity than the
            # original deadlock/evacuation reason stored in the transaction.
            # Feed that identity into the existing signature-based recovery:
            # it waits for a bounded number of identical failures and queues
            # at most one hidden clearance move for an inactive owner.
            stationary_debug["stationaryBlockerRobots"] = sorted(blocker_names)
            if causal_blocker_names or not stationary_debug.get("softBlockedLms"):
                stationary_debug["softBlockedLms"] = sorted({
                    lm_name
                    for blocker_name in blocker_names
                    for lm_name in [
                        self._nearest_lm_for_robot(self.robots[blocker_name])
                    ]
                    if lm_name in self.landmarks
                })
            before = set(self._stationary_clearance_relocations)
            self._record_stationary_order_failure(order, stationary_debug)
            clearance_blocker_names = set(blocker_names)
            clearance_blocker_names.update(
                str(name)
                for name in state.get("clearance_blocker_names", ())
                if str(name) in self.robots and str(name) != robot.name
            )
            clearance_active = self._active_stationary_clearance_for(
                clearance_blocker_names
            )
            if not clearance_active:
                alternate_blocker = (
                    self._queue_alternative_stationary_cut_clearance(
                        order,
                        robot,
                        cause=(
                            f"alternate route cut blocks "
                            f"{order.order_id}"
                        ),
                    )
                )
                if alternate_blocker:
                    clearance_blocker_names = {alternate_blocker}
                    clearance_active = True
            if clearance_active:
                state["clearance_blocker_names"] = tuple(
                    sorted(clearance_blocker_names)
                )
            if (
                set(self._stationary_clearance_relocations) != before
                or clearance_active
            ):
                # The hidden departure gets the next planner slot. Once it has
                # a committed trajectory, this retained-route transaction may
                # safely re-evaluate the same destination.
                state["retry_at"] = max(
                    float(state["retry_at"]),
                    now + min(2.0, self._order_dispatch_retry_interval(order)),
                )
            elif self._stage_stationary_waiter_escape(
                order,
                robot,
                state,
                blocker_names,
                now,
            ):
                # The graph escape atomically replaces this retained-route
                # transaction and sets RETREATING. Continuing below would
                # overwrite the new executable motion back to WAITING.
                return True
            if causal_blocker_names:
                # A completed evacuation proves that the retained approach is
                # unsafe while this exact body remains there. Keep the robot
                # on the safe LM and retry transactionally; allowing one old
                # route-clock step would discard the transaction and recreate
                # the same approach/retreat loop forever.
                state["stage"] = "queued"
        elif bool(
            stationary_debug.get("stationaryRobotWait")
            or stationary_debug.get("stationaryTurnEnvelopeBlock")
            or "stationary_robot_blocks_route" in str(reason).lower()
        ):
            # Runtime snapshots intentionally do not persist short-lived
            # blocker identities. After a restart the planner can still prove
            # that stationary occupancy disconnects the route, but its bounded
            # timeout may expose only the resource class, not the robot name.
            # Rebuild the exact dependency from the current graph cut instead
            # of retrying the same anonymous request forever.
            before = set(self._stationary_clearance_relocations)
            self._record_stationary_order_failure(order, stationary_debug)
            alternate_blocker = (
                self._queue_alternative_stationary_cut_clearance(
                    order,
                    robot,
                    cause=f"anonymous stationary route cut blocks {order.order_id}",
                )
            )
            if alternate_blocker:
                state["clearance_blocker_names"] = (alternate_blocker,)
                state["retry_at"] = max(
                    float(state["retry_at"]),
                    now + min(
                        2.0,
                        self._order_dispatch_retry_interval(order),
                    ),
                )
            elif set(self._stationary_clearance_relocations) != before:
                state["retry_at"] = max(
                    float(state["retry_at"]),
                    now + min(
                        2.0,
                        self._order_dispatch_retry_interval(order),
                    ),
                )
        return False

    def _escalate_repeated_runtime_replan(
        self,
        failure: _RuntimeReplanFailure,
    ) -> None:
        order = failure.order
        robot = failure.robot
        state = failure.state
        now = failure.now
        identical_failures = failure.identical_failures
        dynamic_conflict_signature = failure.dynamic_conflict_signature
        reservation_conflict_signature = failure.reservation_conflict_signature
        if (
            bool(state.get("retained_route_superseded"))
            and dynamic_conflict_signature
            and int(state.get("dynamic_conflict_count", 0) or 0) >= 2
            and state.get("detour_replacement_attempted_signature")
            != dynamic_conflict_signature
        ):
            state["detour_replacement_attempted_signature"] = (
                dynamic_conflict_signature
            )
            blocker_name = str(dynamic_conflict_signature[0])
            blocker = self.robots.get(blocker_name)
            avoid_lm = (
                self._traffic_lm_for_robot(blocker)
                if blocker is not None
                else ""
            )
            if self._queue_alternate_corridor_detour(
                order,
                str(state.get("start_lm") or ""),
                self._active_order_target(order),
                avoid_lm=avoid_lm,
                replace_existing=True,
            ):
                # The old one-chunk exclusion and cached A* suffix produced the
                # same continuous conflict more than once. Replace them as one
                # transaction and immediately let the worker try the genuinely
                # different route. Never accumulate exclusions across retries.
                state["detour_replaced_signature"] = (
                    dynamic_conflict_signature
                )
                state["stage"] = "queued"
                state["retry_at"] = min(
                    float(state.get("retry_at", now) or now),
                    now + 0.5,
                )
        if (
            not dynamic_conflict_signature
            and reservation_conflict_signature
            and int(state.get("reservation_conflict_count", 0) or 0) >= 2
        ):
            # The temporal planner named the owner of the resource which the
            # replacement needs.  Repeating the solo request cannot succeed:
            # that owner's old committed future remains a hard reservation.
            # Expose the real dependency to the normal wait-graph so the same
            # local component is replanned atomically.
            self._escalate_runtime_replan_dependency(
                robot,
                state,
                reservation_conflict_signature,
                now,
                signature_kind="reservation",
            )
        if (
            not bool(state.get("retained_route_superseded"))
            and dynamic_conflict_signature
            and int(state.get("dynamic_conflict_count", 0) or 0) >= 2
            and str(state.get("stage") or "") != "deadlock_escalated"
        ):
            # An ordinary replan keeps its old route executable, but two
            # failures against the exact same moving footprint prove that a
            # third solo request cannot improve the shared geometry. Expose
            # the dependency to the wait graph/local CBS instead of allowing
            # the oldest retry to monopolise the planner indefinitely.
            self._escalate_runtime_replan_dependency(
                robot,
                state,
                dynamic_conflict_signature,
                now,
                signature_kind="dynamic",
            )
        if (
            bool(state.get("retained_route_superseded"))
            and dynamic_conflict_signature
            and int(state.get("dynamic_conflict_count", 0) or 0) >= 3
            and state.get("detour_replacement_attempted_signature")
            == dynamic_conflict_signature
        ):
            # The old suffix is forbidden and one genuinely different A*
            # attempt still met the same live body. Stop burning the planner
            # at 10 Hz and expose this exact dependency to the existing
            # wait-graph -> local CBS -> graph-safe evacuation pipeline.
            self._escalate_runtime_replan_dependency(
                robot,
                state,
                dynamic_conflict_signature,
                now,
                signature_kind="dynamic",
            )
        wait_dependency_signature = (
            self._runtime_replan_wait_dependency_signature(
                robot,
                state,
            )
        )
        if (
            not dynamic_conflict_signature
            and not reservation_conflict_signature
            and wait_dependency_signature
            and identical_failures >= 2
            and str(state.get("stage") or "") != "deadlock_escalated"
        ):
            # Some bounded SIPP failures contain no owner metadata even
            # though runtime collision arbitration captured the exact peer
            # before queueing this transaction. Preserve that stable graph
            # dependency and park the identical retry until the peer changes
            # LM/route/order.
            self._escalate_runtime_replan_dependency(
                robot,
                state,
                wait_dependency_signature,
                now,
                signature_kind="wait_dependency",
            )

    def _publish_deferred_runtime_replan(
        self,
        failure: _RuntimeReplanFailure,
    ) -> None:
        order = failure.order
        robot = failure.robot
        state = failure.state
        reason = failure.reason
        now = failure.now
        failures = failure.failures
        escalated = str(state.get("stage") or "") == "deadlock_escalated"
        order.status = "WAITING_TRAFFIC" if escalated else "WAITING_OBSTACLE"
        order.error = (
            f"deadlock replan awaiting safe evacuation: {reason}"
            if escalated
            else f"runtime replan retry: {reason}"
        )
        order.updated_at = now
        robot.status = "WAITING"
        robot.last_reason = (
            f"occupied by {state.get('escalated_blocker')}"
            if escalated and state.get("escalated_blocker")
            else str(state.get("reason") or reason)
        )
        robot.last_replan_at = now
        robot.last_tick_at = now
        robot.collision_preflight_due_at = 0.0
        robot.updated_at = now
        self._event(
            "warn",
            f"{robot.name} runtime replan deferred; committed route retained: {reason}",
        )
        sink_name = str(state.get("queued_departure_sink") or "")
        if sink_name and failures >= 2:
            raw_signature = state.get("queued_departure_signature")
            signature = (
                tuple(raw_signature)
                if isinstance(raw_signature, tuple)
                else ()
            )
            pocket_lm = str(state.get("escape_goal") or "")
            if signature and pocket_lm:
                self._commanded_sink_vacancy_blacklist.add(
                    (sink_name, signature, robot.name, pocket_lm)
                )
            # A fixed escape that failed twice against unchanged traffic cannot
            # improve by monopolising the planner forever.  Keep the old route
            # and assignment intact, restore the physical dependency, and let
            # the next scheduler tick choose another waiter/pocket.
            self._runtime_replans.pop(robot.name, None)
            self._clear_stationary_order_retry_state(order.order_id)
            order.status = "WAITING_TRAFFIC"
            order.error = (
                f"queued departure vacancy retry: {reason}"
            )
            robot.wait_for_robot = sink_name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = now + self._order_dispatch_retry_interval(order)
            robot.last_reason = f"occupied by {sink_name}"
            self._event(
                "warn",
                f"{robot.name} vacancy {pocket_lm or 'route'} rejected twice; "
                f"rotating queued-departure recovery",
            )

    def _runtime_replan_failure_resource(self, reason: str) -> str:
        """Return the stable graph resource named by a low-level failure."""
        value = str(reason or "")
        markers = (
            "rotation_resource_constrained:",
            "rotation_vertex_reserved:",
            "wait_resource_constrained:",
            "edge_resource_constrained:",
            "resource_constrained:",
            "reserved_edge_interval:",
            "reserved_lm_interval:",
            "reserved_edge:",
            "reserved_lm:",
        )
        for marker in markers:
            if marker not in value:
                continue
            resource = value.rsplit(marker, 1)[-1].split("@", 1)[0].strip()
            if resource:
                return resource
        return ""

    def _runtime_replan_reservation_conflict_signature(
        self,
        robot: FleetRobot,
        reason: str,
        debug: dict[str, Any],
    ) -> tuple[Any, ...]:
        """Fingerprint an exact external owner reported by Rolling SIPP.

        Unlike continuous collision validation, a resource owner can move a
        few centimetres while still reserving the same corridor.  Route-clock
        is therefore deliberately absent; graph LM, route revision and active
        order change only after meaningful traffic progress.
        """
        reported_pairs: list[tuple[str, str]] = []
        raw_pairs = debug.get("reservationBlockers", ())
        if isinstance(raw_pairs, (list, tuple)):
            for item in raw_pairs:
                if not isinstance(item, dict):
                    continue
                owner_name = str(item.get("robot") or "").strip()
                owner_resource = str(
                    item.get("resource") or ""
                ).strip()
                if owner_name and owner_resource:
                    reported_pairs.append(
                        (owner_name, owner_resource)
                    )
        if not reported_pairs:
            raw_names = debug.get("reservationBlockerRobots", ())
            resource = self._runtime_replan_failure_resource(reason)
            if (
                not resource
                or not isinstance(raw_names, (list, tuple, set))
            ):
                return ()
            reported_pairs = [
                (str(raw_name), resource)
                for raw_name in raw_names
                if str(raw_name)
            ]

        candidates: list[tuple[FleetRobot, str]] = []
        for raw_name, resource in reported_pairs:
            blocker = self.robots.get(str(raw_name))
            if (
                blocker is None
                or blocker.name == robot.name
                or blocker.status not in {
                    "MOVING",
                    "WAITING",
                    "RETREATING",
                }
                or not blocker.trajectory
            ):
                continue
            candidates.append((blocker, resource))
        if not candidates:
            return ()

        def blocker_key(
            item: tuple[FleetRobot, str],
        ) -> tuple[float, str, str]:
            blocker, resource = item
            graph_resource = resource.split(":", 1)[-1]
            resource_lms = [
                node
                for node in graph_resource.split("->")
                if node in self.landmarks
            ]
            blocker_lm = self._traffic_lm_for_robot(blocker)
            distance = min(
                (
                    self._lm_distance(blocker_lm, resource_lm)
                    for resource_lm in resource_lms
                ),
                default=0.0,
            )
            return distance, blocker.name, resource

        blocker, resource = min(candidates, key=blocker_key)
        return (
            blocker.name,
            resource,
            self._traffic_lm_for_robot(blocker),
            int(blocker.route_revision),
            str(blocker.active_order_id or ""),
        )

    def _runtime_replan_wait_dependency_signature(
        self,
        robot: FleetRobot,
        state: dict[str, Any],
    ) -> tuple[Any, ...]:
        """Return an unchanged blocker captured by runtime arbitration."""

        raw = state.get("wait_dependency_signature")
        if not isinstance(raw, (list, tuple)) or len(raw) != 5:
            return ()
        blocker_name = str(raw[0])
        blocker = self.robots.get(blocker_name)
        if (
            blocker is None
            or blocker.name == robot.name
            or blocker.status not in {
                "MOVING",
                "WAITING",
                "RETREATING",
            }
            or not blocker.trajectory
        ):
            return ()
        current = (
            blocker.name,
            str(raw[1]),
            self._traffic_lm_for_robot(blocker),
            int(blocker.route_revision),
            str(blocker.active_order_id or ""),
        )
        return current if current == tuple(raw) else ()

    def _escalate_runtime_replan_dependency(
        self,
        robot: FleetRobot,
        state: dict[str, Any],
        signature: tuple[Any, ...],
        now: float,
        *,
        signature_kind: str,
    ) -> None:
        """Expose one proven live blocker to wait-graph/CBS arbitration."""
        blocker_name = str(signature[0])
        blocker = self.robots.get(blocker_name)
        state["stage"] = "deadlock_escalated"
        state["escalated_at"] = now
        state["escalated_blocker"] = blocker_name
        state["escalated_blocker_order_id"] = str(
            blocker.active_order_id if blocker is not None else ""
        )
        state["escalated_resource"] = str(signature[1])
        state["escalation_blocker_signature"] = signature
        state["escalation_signature_kind"] = str(signature_kind)
        robot.wait_for_robot = blocker_name
        robot.wait_resource = str(signature[1])
        robot.wait_release_at = 0.0
        episode_started = float(state.get("queued_at", now) or now)
        robot.blocked_since = robot.blocked_since or episode_started
        robot.traffic_stall_since = (
            robot.traffic_stall_since or episode_started
        )

    def _runtime_replan_dynamic_conflict_signature(
        self,
        robot: FleetRobot,
        debug: dict[str, Any],
    ) -> tuple[Any, ...]:
        """Fingerprint one unchanged live robot conflict across planner retries."""
        blocker_name = str(debug.get("continuousConflictRobot") or "").strip()
        conflict_edge = str(debug.get("continuousConflictEdge") or "").strip()
        blocker = self.robots.get(blocker_name)
        if (
            blocker is None
            or blocker.name == robot.name
            or blocker.status not in {"MOVING", "WAITING"}
            or not blocker.trajectory
            or not conflict_edge
        ):
            return ()
        return (
            blocker.name,
            conflict_edge,
            self._traffic_lm_for_robot(blocker),
            int(blocker.route_revision),
            round(float(blocker.route_clock), 3),
        )

    def _runtime_replan_escalation_blocker_changed(
        self,
        state: dict[str, Any],
    ) -> bool:
        """Return whether an escalated live dependency has made real progress."""
        raw_signature = state.get("escalation_blocker_signature")
        if not isinstance(raw_signature, (list, tuple)) or len(raw_signature) < 5:
            return True
        blocker_name = str(raw_signature[0])
        blocker = self.robots.get(blocker_name)
        if (
            blocker is None
            or blocker.status not in {"MOVING", "WAITING", "RETREATING"}
            or not blocker.trajectory
        ):
            return True
        if str(state.get("escalated_blocker_order_id") or "") != str(
            blocker.active_order_id or ""
        ):
            return True
        if str(state.get("escalation_signature_kind") or "") in {
            "reservation",
            "wait_dependency",
        }:
            current_signature = (
                blocker.name,
                str(raw_signature[1]),
                self._traffic_lm_for_robot(blocker),
                int(blocker.route_revision),
                str(blocker.active_order_id or ""),
            )
            return current_signature != tuple(raw_signature[:5])
        current_signature = (
            blocker.name,
            str(raw_signature[1]),
            self._traffic_lm_for_robot(blocker),
            int(blocker.route_revision),
            round(float(blocker.route_clock), 3),
        )
        return current_signature != tuple(raw_signature[:5])

    def _coupled_replan_failure_count_for_members(
        self,
        robot_names: tuple[str, ...] | list[str] | set[str],
    ) -> int:
        """Return failures recorded for this cycle or an expanded component."""
        members = {str(name) for name in robot_names if str(name)}
        if not members:
            return 0
        return max(
            (
                int(count)
                for key, count in self._coupled_replan_failures.items()
                if members.issubset(set(key))
            ),
            default=0,
        )

    def _coupled_replan_latest_attempt_for_members(
        self,
        robot_names: tuple[str, ...] | list[str] | set[str],
    ) -> float:
        """Return the newest attempt for this cycle or its expanded group."""
        members = {str(name) for name in robot_names if str(name)}
        if not members:
            return 0.0
        return max(
            (
                float(attempted_at)
                for key, attempted_at
                in self._coupled_replan_last_attempt.items()
                if members.issubset(set(key))
            ),
            default=0.0,
        )

    def _clear_coupled_replan_attempts_for_members(
        self,
        robot_names: tuple[str, ...] | list[str] | set[str],
        *,
        include_subsets: bool = False,
    ) -> None:
        """Forget exact and expanded attempts after real component progress."""
        members = {str(name) for name in robot_names if str(name)}
        if not members:
            return
        for state in (
            self._coupled_replan_last_attempt,
            self._coupled_replan_failures,
        ):
            for key in list(state):
                key_members = set(key)
                if (
                    members.issubset(key_members)
                    or (
                        include_subsets
                        and key_members.issubset(members)
                    )
                ):
                    state.pop(key, None)

    def _expand_coupled_replan_component(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
    ) -> list[FleetRobot]:
        """Join safe held neighbours from the same proven dependency graph."""
        by_name = {robot.name: robot for robot in robots}
        if not by_name or winner.name not in by_name:
            return robots
        winner_order = self._active_order_for_robot(winner)
        if winner_order is None:
            return robots
        motion_key = self._order_motion_key(winner_order)
        limit = max(2, int(self.planner.local_cbs_max_robots))

        def eligible(name: str) -> FleetRobot | None:
            candidate = self.robots.get(name)
            order = (
                self._active_order_for_robot(candidate)
                if candidate is not None
                else None
            )
            if (
                candidate is None
                or order is None
                or candidate.is_remote()
                or candidate.status != "WAITING"
                or order.internal_kind == "traffic_clearance"
                or self._safe_replan_start_lm(candidate)
                not in self.landmarks
                or self._order_motion_key(order) != motion_key
            ):
                return None
            return candidate

        changed = True
        while changed and len(by_name) <= limit:
            changed = False
            selected = set(by_name)
            related_names: set[str] = set()
            for cycle_key in self._active_wait_cycles:
                if selected.intersection(cycle_key):
                    related_names.update(cycle_key)
            for candidate in self._runtime_robots():
                state = self._runtime_replans.get(candidate.name)
                if (
                    not isinstance(state, dict)
                    or not self._runtime_replan_state_is_current(
                        candidate,
                        state,
                        allowed_stages={
                            "queued",
                            "planning",
                            "retry",
                            "deadlock_escalated",
                        },
                    )
                ):
                    continue
                dependencies = {
                    str(name)
                    for name in state.get("blocker_names", ())
                    if str(name)
                }
                escalated = str(state.get("escalated_blocker") or "")
                if escalated:
                    dependencies.add(escalated)
                for signature_key in (
                    "dynamic_conflict_signature",
                    "reservation_conflict_signature",
                ):
                    signature = state.get(signature_key)
                    if isinstance(signature, (list, tuple)) and signature:
                        dependencies.add(str(signature[0]))
                if (
                    candidate.name in selected
                    or dependencies.intersection(selected)
                ):
                    related_names.add(candidate.name)
                    related_names.update(dependencies)

            for name in sorted(related_names - selected):
                candidate = eligible(name)
                if candidate is None:
                    continue
                if len(by_name) >= limit:
                    return robots
                by_name[name] = candidate
                changed = True
        return [
            *robots,
            *(
                by_name[name]
                for name in sorted(set(by_name) - {robot.name for robot in robots})
            ),
        ]
