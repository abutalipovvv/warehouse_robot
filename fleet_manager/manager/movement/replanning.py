"""Runtime route-replanning transaction lifecycle."""

from __future__ import annotations

from fleet_manager.manager.tasks.statuses import TERMINAL_ORDER_STATUSES
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot


class FleetRuntimeReplanMixin:
    """Queue, hold, replace and retire background replanning transactions."""

    def _schedule_runtime_replan(self, robot: FleetRobot, now: float, reason: str) -> bool:
        active_order = self._active_order_for_robot(robot)
        if (
            active_order is not None
            and active_order.internal_kind == "traffic_clearance"
        ):
            # A clearance route is selected as a short, graph-safe evacuation
            # on the external side of the causal corridor.  Replacing it with
            # congestion A* after a traffic timeout can send the parked body
            # through the owner's still-valid corridor lease and recreate the
            # exact dependency this maintenance task exists to remove.  Keep
            # the explicit route and let ordinary temporal admission wait;
            # the bounded clearance lifecycle may cancel/requeue it later.
            robot.last_replan_at = now
            # This branch is observed on every physics tick after the normal
            # blocked-replan deadline.  Keep the first observation as the
            # lifecycle age; resetting it here makes a permanently blocked
            # clearance look freshly stalled forever.
            robot.traffic_stall_since = robot.traffic_stall_since or now
            return False
        if self._reason_requires_spatial_replan(reason):
            start_lm = self._safe_replan_start_lm(robot)
            if active_order is not None and start_lm:
                avoid_lm = ""
                if self._is_parked_robot_conflict(reason):
                    blocker = self.robots.get(
                        self._robot_name_from_conflict_reason(reason)
                    )
                    if blocker is not None:
                        avoid_lm = self._traffic_lm_for_robot(blocker)
                self._queue_alternate_corridor_detour(
                    active_order,
                    start_lm,
                    self._active_order_target(active_order),
                    avoid_lm=avoid_lm,
                )
        if self._queue_active_order_for_background_replan(robot, now, reason):
            return True
        if robot.active_order_id and not robot.is_remote():
            # An active simulated robot inside a controlled corridor retains
            # its executable trajectory so arbitration can grant an exit or
            # retreat. Never fall through to synchronous MAPF here: the
            # runtime thread would repeatedly acquire the sole planner lock,
            # overwrite the useful corridor wait dependency on failure, and
            # starve every rolling continuation.
            robot.last_replan_at = now
            return False
        # Manual/ad-hoc routes have no order that can be returned to the
        # dispatcher, so retain the synchronous compatibility path for those
        # uncommon requests only.
        return self._maybe_replan_robot(robot, now, reason)

    def _queue_active_order_for_background_replan(
        self,
        robot: FleetRobot,
        now: float,
        reason: str,
        *,
        supersede_retained_route: bool = False,
    ) -> bool:
        if robot.is_remote() or not robot.active_order_id:
            return False
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            return False
        if order.internal_kind == "traffic_clearance":
            # See _schedule_runtime_replan(): this order's explicit spatial
            # route is the recovery invariant, not a cache which a generic
            # runtime transaction may invalidate.
            return False
        start_lm = self._safe_replan_start_lm(robot)
        if not start_lm:
            return False
        corridor_graph = self._controlled_corridor_graph
        corridor_vertex = (
            corridor_graph.vertices.get(start_lm)
            if corridor_graph is not None
            else None
        )
        if (
            corridor_vertex is not None
            and corridor_vertex.controlled_region_ids
        ):
            # Never clear the only executable timeline while the physical
            # body is inside a capacity-one passage. Emergency callers used
            # to bypass this guard, leaving an IDLE/QUEUED robot permanently
            # parked on a no-wait LM. Recovery must first reach an external
            # safe holding LM, then it may enqueue the spatial detour.
            return False

        retained_route_superseded = bool(
            supersede_retained_route
            or self._replan_supersedes_retained_route(reason)
        )
        requires_spatial_replan = bool(
            supersede_retained_route
            or self._reason_requires_spatial_replan(reason)
        )
        existing = self._runtime_replans.get(robot.name)
        if isinstance(existing, dict):
            if self._same_runtime_replan_transaction(
                existing,
                robot,
                order,
                start_lm,
            ):
                return self._refresh_runtime_replan_transaction(
                    existing,
                    robot,
                    order,
                    now,
                    reason,
                    retained_route_superseded=(
                        retained_route_superseded
                    ),
                    requires_spatial_replan=requires_spatial_replan,
                )
            self._runtime_replans.pop(robot.name, None)

        if (
            retained_route_superseded
            and not self._superseded_runtime_replan_slot_available(robot.name)
        ):
            return False

        (
            blocker_names,
            causal_blocker_signatures,
            wait_dependency_signature,
        ) = self._runtime_replan_blocker_evidence(robot, reason)
        generation = (
            int(existing.get("generation", 0) or 0) + 1
            if isinstance(existing, dict)
            else 1
        )
        self._install_runtime_replan_transaction(
            robot,
            order,
            start_lm=start_lm,
            now=now,
            reason=reason,
            generation=generation,
            blocker_names=blocker_names,
            causal_blocker_signatures=causal_blocker_signatures,
            wait_dependency_signature=wait_dependency_signature,
            retained_route_superseded=retained_route_superseded,
            requires_spatial_replan=requires_spatial_replan,
        )
        return True

    def _same_runtime_replan_transaction(
        self,
        existing: dict[str, object],
        robot: FleetRobot,
        order: FleetOrder,
        start_lm: str,
    ) -> bool:
        return self._replanning_service.is_same_transaction(
            existing,
            robot,
            order,
            start_lm,
        )

    def _refresh_runtime_replan_transaction(
        self,
        existing: dict[str, object],
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
        reason: str,
        *,
        retained_route_superseded: bool,
        requires_spatial_replan: bool,
    ) -> bool:
        """Idempotently retain or safely promote an existing transaction."""

        if (
            not retained_route_superseded
            or bool(existing.get("retained_route_superseded"))
        ):
            return True
        if not self._superseded_runtime_replan_slot_available(robot.name):
            return False

        existing["retained_route_superseded"] = True
        existing["reason"] = str(
            reason
            or "deadlock corridor evacuated; alternate route required"
        )
        existing["generation"] = (
            int(existing.get("generation", 0) or 0) + 1
        )
        existing["stage"] = "queued"
        existing["retry_at"] = float(now)
        existing["promoted_at"] = float(now)
        if requires_spatial_replan:
            order.spatial_route_nodes = []
            order.traffic_blocked_since = now
        order.status = "PLANNING"
        order.error = f"runtime replan pending: {existing['reason']}"
        order.updated_at = now
        robot.status = "WAITING"
        robot.last_reason = (
            "replanning route while holding: "
            f"{existing['reason']}"
        )
        robot.last_tick_at = now
        robot.updated_at = now
        return True

    def _runtime_replan_blocker_evidence(
        self,
        robot: FleetRobot,
        reason: str,
    ) -> tuple[
        tuple[str, ...],
        tuple[tuple[str, str, int], ...],
        tuple[object, ...],
    ]:
        captured_blocker = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(reason)
        )
        blocker_names = (
            (captured_blocker,)
            if (
                captured_blocker in self.robots
                and captured_blocker != robot.name
            )
            else ()
        )
        causal_blocker_signatures = tuple(
            (
                name,
                self._traffic_lm_for_robot(self.robots[name]),
                int(self.robots[name].route_revision),
            )
            for name in blocker_names
        )
        wait_dependency_signature: tuple[object, ...] = (
            (
                captured_blocker,
                str(
                    robot.wait_resource
                    or self._edge_id_at_trajectory(
                        robot.trajectory,
                        robot.route_clock,
                    )
                    or "traffic"
                ),
                self._traffic_lm_for_robot(
                    self.robots[captured_blocker]
                ),
                int(self.robots[captured_blocker].route_revision),
                str(
                    self.robots[captured_blocker].active_order_id or ""
                ),
            )
            if blocker_names
            else ()
        )
        return (
            blocker_names,
            causal_blocker_signatures,
            wait_dependency_signature,
        )

    def _install_runtime_replan_transaction(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        *,
        start_lm: str,
        now: float,
        reason: str,
        generation: int,
        blocker_names: tuple[str, ...],
        causal_blocker_signatures: tuple[tuple[str, str, int], ...],
        wait_dependency_signature: tuple[object, ...],
        retained_route_superseded: bool,
        requires_spatial_replan: bool,
    ) -> None:
        self._replanning_service.install_transaction(
            robot,
            order,
            start_lm=start_lm,
            now=now,
            reason=reason,
            generation=generation,
            blocker_names=blocker_names,
            causal_blocker_signatures=causal_blocker_signatures,
            wait_dependency_signature=wait_dependency_signature,
            retained_route_superseded=retained_route_superseded,
            requires_spatial_replan=requires_spatial_replan,
        )
        self._clear_wait_dependency(robot)
        self._event(
            "warn",
            f"{robot.name} transactional background replan queued: {reason}",
        )

    def _superseded_runtime_replan_limit(self) -> int:
        """Bound routes which are invalidated before a replacement exists.

        A superseded transaction deliberately turns its retained trajectory
        into a stationary safety snapshot.  Admitting an unbounded number of
        those transactions behind the single atomic commit stream creates a
        positive feedback loop: every queued snapshot is another full-horizon
        obstacle for the next planner request.  Keep a tiny bounded recovery
        window; ordinary transactional replans continue to execute their
        retained routes and are not counted here.
        """

        return self.settings.fleet.integer(
            "max_superseded_runtime_replans",
            2,
            minimum=1,
            maximum=8,
            default_if_falsy=True,
        )

    def _superseded_runtime_replan_slot_available(
        self,
        robot_name: str,
    ) -> bool:
        """Return whether another unsafe-route replacement may be admitted."""

        active = 0
        for owner_name, state in self._runtime_replans.items():
            if owner_name == robot_name or not isinstance(state, dict):
                continue
            if not bool(state.get("retained_route_superseded")):
                continue
            owner = self.robots.get(owner_name)
            if owner is None:
                continue
            validator = getattr(
                self,
                "_runtime_replan_state_is_current",
                None,
            )
            if callable(validator) and not validator(
                owner,
                state,
                allowed_stages={
                    "queued",
                    "planning",
                    "retry",
                    "deadlock_escalated",
                },
            ):
                continue
            active += 1
            if active >= self._superseded_runtime_replan_limit():
                return False
        return True

    def _replan_supersedes_retained_route(self, reason: str) -> bool:
        """Return whether retrying the old spatial suffix is knowingly unsafe."""
        value = str(reason or "").strip().lower()
        return bool(
            "corridor evacuated" in value
            and "alternate route required" in value
        )

    def _queue_background_replan_recovery_action(
        self,
        robot: FleetRobot,
        now: float,
        reason: str,
        *,
        supersede_retained_route: bool = False,
    ) -> tuple[bool, bool]:
        """Return ``(handled, started)`` for a deadlock replan request.

        The ordinary queue method is intentionally idempotent: it returns
        ``True`` both when it creates a transaction and when the identical
        transaction is already queued/planning/retrying.  Deadlock call sites
        need the distinction because only the former is a new recovery action
        that may increment ``cycleReplans`` or detour-attempt counters.

        The no-state fallback preserves lightweight tests/adapters which
        replace the queue method with a successful stub.
        """
        before = self._runtime_replans.get(robot.name)
        before_generation = (
            int(before.get("generation", 0) or 0)
            if isinstance(before, dict)
            else -1
        )
        handled = self._queue_active_order_for_background_replan(
            robot,
            now,
            reason,
            supersede_retained_route=supersede_retained_route,
        )
        if not handled:
            return False, False
        after = self._runtime_replans.get(robot.name)
        after_generation = (
            int(after.get("generation", 0) or 0)
            if isinstance(after, dict)
            else -1
        )
        started = bool(
            before is not after
            or before_generation != after_generation
            or (before is None and after is None)
        )
        return True, started

    def _runtime_replan_holds_robot(self, robot: FleetRobot) -> bool:
        state = self._runtime_replans.get(robot.name)
        if not isinstance(state, dict):
            return False
        order = self.orders.get(str(state.get("order_id") or ""))
        if (
            order is None
            or order.status in TERMINAL_ORDER_STATUSES
            or robot.active_order_id != order.order_id
            or int(state.get("route_revision", -1)) != int(robot.route_revision)
            or abs(
                float(state.get("route_clock", 0.0) or 0.0)
                - float(robot.route_clock)
            ) > 0.000001
            or self._safe_replan_start_lm(robot)
            != str(state.get("start_lm") or "")
        ):
            self._runtime_replans.pop(robot.name, None)
            return False
        corridor_hold = state.get("corridor_clearance_hold")
        if isinstance(corridor_hold, dict):
            if self._corridor_clearance_hold_active(
                corridor_hold,
                robot.name,
            ):
                # A failed planner attempt normally lets the old route retry.
                # A portal evacuation is different: retrying that route would
                # immediately refill the pocket. Hold retry state as well as
                # queued/planning state until the owner clears or explicitly
                # depends on this robot moving farther away.
                return True
            state.pop("corridor_clearance_hold", None)
        stage = str(state.get("stage") or "")
        if stage == "deadlock_escalated":
            # The old suffix is a collision/reservation snapshot only.
            # Ordinary priority grants must not start it; the existing
            # wait-graph may release this hold only by installing an explicit
            # reverse retreat/graph escape (which changes route identity).
            return not (
                robot.status == "RETREATING"
                and robot.retreat_target_clock is not None
            )
        if bool(state.get("retained_route_superseded")):
            # Unlike an ordinary transient temporal failure, progress along
            # this route would immediately invalidate the transaction and
            # reacquire the corridor which recovery deliberately evacuated.
            # Hold through retry backoff until an alternate path commits.
            return stage in {"queued", "planning", "retry"}
        if state.get("clearance_blocker_names"):
            # This robot was intentionally moved/held to open the only route
            # for a stationary blocker.  Retrying its old suffix before that
            # maintenance move completes would close the pocket again.
            return stage in {"queued", "planning", "retry"}
        # Merely entering the planner queue is not a motion command.  There is
        # one shared worker today, so freezing every queued transaction turns
        # a short planner backlog into a fleet-wide stop.  Until this specific
        # request reaches the worker, keep checking the committed trajectory:
        # a transient blocker may clear without any spatial replan at all.
        # The planning stage still holds the exact snapshot required for an
        # atomic result commit.
        return stage == "planning"

    def _discard_runtime_replan_after_progress(self, robot: FleetRobot) -> None:
        state = self._runtime_replans.get(robot.name)
        if not isinstance(state, dict):
            return
        stage = str(state.get("stage") or "")
        if (
            stage not in {"queued", "retry"}
            or bool(state.get("retained_route_superseded"))
            or bool(state.get("clearance_blocker_names"))
        ):
            return
        if abs(
            float(state.get("route_clock", 0.0) or 0.0)
            - float(robot.route_clock)
        ) <= 0.000001:
            return
        self._runtime_replans.pop(robot.name, None)
        order = self.orders.get(robot.active_order_id)
        if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
            order.status = "EXECUTING"
            order.error = ""
            order.updated_at = self._now()
        self._event(
            "info",
            f"{robot.name} retained route cleared before replan retry",
        )

    def _reason_requires_spatial_replan(self, reason: str) -> bool:
        value = str(reason or "").lower()
        return bool(
            self._is_parked_robot_conflict(reason)
            or "alternate route required" in value
            or "traffic admission timeout" in value
            or "corridor admission timeout" in value
            or "traffic wait timeout" in value
            or "obstacle" in value
            or "blocked edge" in value
        )

    def _maybe_replan_robot(self, robot: FleetRobot, now: float, reason: str) -> bool:
        if not robot.target_lm:
            return False
        interval = self._replan_interval()
        if robot.last_replan_at is not None and now - robot.last_replan_at < interval:
            return False

        start_lm = self._safe_replan_start_lm(robot)
        if not start_lm or start_lm not in self.landmarks:
            already_deferred = robot.status == "WAITING" and robot.last_reason == reason
            robot.last_replan_at = now
            robot.status = "WAITING" if robot.trajectory else "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = now
            if robot.trajectory and not already_deferred:
                self._event(
                    "info",
                    f"{robot.name} replan deferred until the next LM (holding current edge)",
                )
            return False

        robot.last_replan_at = now
        request = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": robot.target_lm,
            "startPose": dict(robot.pose) if robot.pose else self._pose_at_landmark(start_lm),
        }
        no_reverse_edges = self._no_reverse_edges(robot, start_lm)
        result = self._plan_valid_requests(
            [request],
            {
                "robots": [request],
                "blocked_edges": [
                    {"from": src, "to": dst}
                    for src, dst in sorted(no_reverse_edges)
                ],
            },
        )
        if result.get("ok") and result.get("plans"):
            plan_note = self._plan_note(result)
            if (
                plan_note == "FALLBACK_WAIT"
                and robot.trajectory
                and self._is_parked_robot_conflict(reason)
            ):
                robot.status = "WAITING"
                robot.last_reason = reason
                robot.updated_at = now
                self._event("warn", f"{robot.name} replan pending: no detour around parked robot")
                return False
            self._apply_planner_result(result, now)
            robot.route_note = f"REPLAN: {plan_note}"
            robot.last_reason = robot.route_note
            self._event("info", f"{robot.name} replanned after block: {reason}")
            return True

        robot.status = "WAITING" if robot.trajectory else "BLOCKED"
        if self._is_parked_robot_conflict(reason):
            robot.last_reason = reason
        else:
            robot.last_reason = result.get("debug", {}).get("reason", reason)
        robot.updated_at = now
        self._event("warn", f"{robot.name} replan pending: {robot.last_reason}")
        return False

    def _maybe_replan_remote_robot_order(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
        reason: str,
    ) -> bool:
        if not robot.is_remote() or not robot.base_url:
            return False
        target_lm = self._active_order_target(order)
        if not target_lm:
            return False
        interval = self._replan_interval()
        if robot.last_replan_at is not None and now - robot.last_replan_at < interval:
            return False

        start_lm = self._safe_replan_start_lm(robot)
        if not start_lm or start_lm not in self.landmarks:
            robot.last_replan_at = now
            robot.status = "WAITING" if robot.trajectory else "BLOCKED"
            robot.last_reason = reason
            robot.updated_at = now
            return False

        robot.last_replan_at = now
        request = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": target_lm,
            "startPose": dict(robot.pose) if robot.pose else self._pose_at_landmark(start_lm),
        }
        result = self._plan_valid_requests([request], {"robots": [request]})
        if not result.get("ok") or not result.get("plans"):
            robot.status = "WAITING" if robot.trajectory else "BLOCKED"
            debug = result.get("debug", {})
            if isinstance(debug, dict):
                robot.last_reason = str(debug.get("reason") or reason)
            else:
                robot.last_reason = reason
            robot.updated_at = now
            self._event("warn", f"{robot.name} remote replan pending: {robot.last_reason}")
            return False

        plan = self._plan_for_robot(result, robot.name)
        if plan is None:
            robot.status = "WAITING"
            robot.last_reason = "remote replan did not return robot plan"
            robot.updated_at = now
            return False

        try:
            remote_route = self._execute_remote_plan(robot, order, plan, result)
        except Exception as exc:
            robot.remote_error = str(exc)
            robot.remote_online = False
            robot.status = "OFFLINE"
            robot.last_reason = f"remote replan execute failed: {exc}"
            robot.updated_at = now
            return False

        order.route_nodes = [str(item) for item in plan.get("nodes", []) if str(item)]
        self._apply_planner_result(result, now, order_id=order.order_id)
        self._apply_remote_route_metadata(robot, remote_route, now)
        self._set_order_status(order, "EXECUTING", robot=robot, start_lm=start_lm)
        robot.route_note = f"REPLAN: {self._plan_note(result)}"
        robot.last_reason = robot.route_note
        self._event("info", f"{robot.name} remote route revision {robot.route_revision}: {reason}")
        return True

    def _no_reverse_edges(self, robot: FleetRobot, start_lm: str) -> set[tuple[str, str]]:
        if not robot.plan_nodes or start_lm not in robot.plan_nodes:
            return set()
        index = robot.plan_nodes.index(start_lm)
        if index <= 0:
            return set()
        previous = robot.plan_nodes[index - 1]
        if previous not in self.landmarks:
            return set()
        return {(start_lm, previous)}
