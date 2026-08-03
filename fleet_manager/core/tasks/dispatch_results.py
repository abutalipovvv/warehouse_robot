"""Validate planner results and publish commit or failure transitions."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.domain.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.domain.models import FleetOrder, FleetRobot


RollingPrefetchResultEntry = tuple[
    FleetOrder,
    FleetRobot,
    dict[str, Any],
    str,
    float,
]
SimulatedDispatchEntry = tuple[
    FleetOrder,
    FleetRobot,
    dict[str, Any],
    str,
]
AcceptedSimulatedDispatchEntry = tuple[
    FleetOrder,
    FleetRobot,
    dict[str, Any],
    str,
    dict[str, Any],
]


class DispatchResultMixin:
    """Validate planner results and publish commit or failure transitions."""

    def _finish_simulated_order_batch(
        self,
        entries: list[SimulatedDispatchEntry],
        result: dict[str, Any],
        *,
        corridor_gates: dict[str, dict[str, Any]] | None = None,
    ) -> tuple[int, set[str]]:
        handled = {order.order_id for order, _, _, _ in entries}
        final_goals = {
            robot.name: final_goal
            for _, robot, _, final_goal in entries
        }
        if not result.get("ok") or not result.get("plans"):
            self._finish_failed_simulated_order_batch(entries, result)
            return 0, handled

        gate_by_robot = (
            corridor_gates
            if isinstance(corridor_gates, dict)
            else {}
        )
        result = self._rolling_result(
            result,
            final_goals,
            corridor_gates=gate_by_robot,
        )
        accepted_entries = self._accepted_simulated_dispatch_entries(
            entries,
            result,
            gate_by_robot,
            finish_now=self._now(),
        )
        if not accepted_entries:
            return 0, handled
        dispatched = self._commit_simulated_dispatch_entries(
            accepted_entries,
            result,
        )
        return dispatched, handled

    def _finish_failed_simulated_order_batch(
        self,
        entries: list[SimulatedDispatchEntry],
        result: dict[str, Any],
    ) -> None:
        reason = self._planner_failure_reason(result)
        debug = result.get("debug")
        if not isinstance(debug, dict):
            debug = {}
        conflicts_by_requester = self._record_dispatch_conflict_dependencies(
            entries,
            debug,
        )
        stationary_failure = bool(
            debug.get("stationaryRobotWait")
            or "stationary_robot_blocks_route" in reason
        )
        conflict_robot = self._planner_conflict_robot_name(reason)
        isolated = not conflicts_by_requester and conflict_robot and any(
            robot.name == conflict_robot
            for _, robot, _, _ in entries
        )
        for order, robot, _, _ in entries:
            affected = (
                robot.name in conflicts_by_requester
                if conflicts_by_requester
                else not isolated or robot.name == conflict_robot
            )
            if not affected:
                # One named member failed validation. Other queue heads should
                # remain immediately eligible for their next dispatch turn.
                order.status = "QUEUED"
                order.error = ""
                order.updated_at = self._now()
                self._clear_stationary_order_retry_state(order.order_id)
                continue
            self._set_order_error(order, reason)
            if (
                stationary_failure
                and self._stationary_failure_applies_to_robot(
                    debug,
                    robot.name,
                )
            ):
                self._record_stationary_order_failure(order, debug)

    def _accepted_simulated_dispatch_entries(
        self,
        entries: list[SimulatedDispatchEntry],
        result: dict[str, Any],
        corridor_gates: dict[str, dict[str, Any]],
        *,
        finish_now: float,
    ) -> list[AcceptedSimulatedDispatchEntry]:
        plans_by_robot = {
            str(plan.get("robot")): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        accepted: list[AcceptedSimulatedDispatchEntry] = []
        for order, robot, request, final_goal in entries:
            plan = plans_by_robot.get(robot.name)
            if plan is None:
                self._set_order_error(
                    order,
                    "planner did not return robot plan",
                )
                continue
            if not self._plan_follows_requested_clearance_route(
                order,
                request,
                plan,
            ):
                self._set_order_error(
                    order,
                    "traffic clearance planner changed its fixed route",
                )
                continue
            if not self._simulated_dispatch_corridor_gate_allows_commit(
                order,
                robot,
                request,
                plan,
                corridor_gates.get(robot.name),
                finish_now=finish_now,
            ):
                continue
            if self._wait_only_rolling_plan(plan, final_goal):
                self._reject_wait_only_simulated_dispatch(
                    order,
                    robot,
                    request,
                    final_goal,
                )
                continue
            accepted.append((order, robot, request, final_goal, plan))
        return accepted

    def _simulated_dispatch_corridor_gate_allows_commit(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        plan: dict[str, Any],
        corridor_gate: dict[str, Any] | None,
        *,
        finish_now: float,
    ) -> bool:
        if not isinstance(corridor_gate, dict):
            return True
        if bool(plan.get("corridorPassageDeferred")):
            intent = corridor_gate.get("intent")
            if (
                self._controlled_corridor_prefetch_intents.get(robot.name)
                is intent
            ):
                self._controlled_corridor_prefetch_intents.pop(
                    robot.name,
                    None,
                )
            return True

        gate_current, gate_reason = (
            self._controlled_corridor_prefetch_plan_is_current(
                robot,
                request,
                plan,
                corridor_gate,
                now=finish_now,
            )
        )
        if (
            gate_current
            and not self._commit_controlled_corridor_prefetch_slot(
                robot,
                corridor_gate,
            )
        ):
            gate_current = False
            gate_reason = "corridor slot changed before command commit"
        if gate_current:
            return True

        self._handle_controlled_corridor_gate_rejection(
            robot.name,
            corridor_gate,
            gate_reason,
        )
        order.status = "QUEUED"
        order.error = ""
        order.updated_at = finish_now
        self._event(
            "info",
            f"{robot.name} initial corridor route rescheduled: "
            f"{gate_reason}",
        )
        return False

    def _reject_wait_only_simulated_dispatch(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        final_goal: str,
    ) -> None:
        detour_queued = (
            self._order_stall_allows_detour(order)
            and self._queue_alternate_corridor_detour(
                order,
                str(request.get("startLm") or ""),
                final_goal,
            )
        )
        self._set_order_error(
            order,
            "traffic window has no progress; alternate corridor queued"
            if detour_queued
            else "traffic window has no progress; joint retry pending",
        )
        self._event(
            "warn",
            f"{robot.name} wait-only route rejected; order kept queued",
        )

    def _commit_simulated_dispatch_entries(
        self,
        entries: list[AcceptedSimulatedDispatchEntry],
        result: dict[str, Any],
    ) -> int:
        accepted_result = {
            **result,
            "plans": [entry[4] for entry in entries],
        }
        now = self._now()
        self._apply_planner_result(accepted_result, now)
        for order, robot, request, _, plan in entries:
            self._dispatch_conflict_dependencies.pop(
                order.order_id,
                None,
            )
            order.route_nodes = [
                str(item)
                for item in plan.get("nodes", [])
            ]
            robot.active_order_id = order.order_id
            self._apply_simulated_route_metadata(
                robot,
                order,
                plan,
                now,
            )
            self._set_order_status(
                order,
                "EXECUTING",
                robot=robot,
                start_lm=str(request["startLm"]),
            )
            order.traffic_detour_edges = []
            self._event(
                "info",
                f"order dispatched: {order.order_id} {robot.name} "
                f"{request['startLm']}->{order.target_lm}",
            )
        return len(entries)

    def _plan_follows_requested_clearance_route(
        self,
        order: FleetOrder,
        request: dict[str, Any],
        plan: dict[str, Any],
    ) -> bool:
        """Prove that a maintenance plan kept its immutable graph suffix."""
        if order.internal_kind != "traffic_clearance":
            return True

        def compact(values: Any) -> list[str]:
            nodes: list[str] = []
            for value in values if isinstance(values, (list, tuple)) else ():
                node = str(value)
                if node in self.landmarks and (not nodes or nodes[-1] != node):
                    nodes.append(node)
            return nodes

        requested = compact(request.get("routeNodes"))
        actual = compact(plan.get("nodes"))
        return (
            len(requested) >= 2
            and len(actual) >= 2
            and actual == requested[:len(actual)]
        )

    def _finish_async_runtime_replan(self, job: dict[str, Any]) -> int:
        robot_name = str(job.get("robot_name") or "")
        robot = self.robots.get(robot_name)
        order = self.orders.get(str(job.get("order_id") or ""))
        state = self._runtime_replans.get(robot_name)
        if (
            robot is None
            or order is None
            or not isinstance(state, dict)
            or order.status in TERMINAL_ORDER_STATUSES
            or order.internal_kind == "traffic_clearance"
            or robot.active_order_id != order.order_id
            or int(state.get("generation", -1))
            != int(job.get("generation", -2))
            or int(robot.route_revision) != int(job.get("route_revision", -1))
            or abs(float(robot.route_clock) - float(job.get("route_clock", 0.0)))
            > 0.000001
            or self._safe_replan_start_lm(robot)
            != str(job.get("start_lm") or "")
        ):
            if isinstance(state, dict) and str(state.get("stage") or "") == "planning":
                self._runtime_replans.pop(robot_name, None)
            return 0

        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            reason = (
                self._planner_failure_reason(result)
                if isinstance(result, dict)
                else "runtime replan returned no result"
            )
            debug = result.get("debug", {}) if isinstance(result, dict) else {}
            self._defer_runtime_replan(
                order,
                robot,
                state,
                reason,
                debug=debug if isinstance(debug, dict) else {},
            )
            return 0

        final_goal = str(job.get("final_goal") or self._active_order_target(order))
        result = self._rolling_result(result, {robot.name: final_goal})
        plan = self._plan_for_robot(result, robot.name)
        if plan is None or self._wait_only_rolling_plan(plan, final_goal):
            self._defer_runtime_replan(
                order,
                robot,
                state,
                "runtime replan made no route progress",
                debug=(
                    result.get("debug", {})
                    if isinstance(result.get("debug"), dict)
                    else {}
                ),
            )
            return 0

        # This method runs on the runtime thread between physics ticks.  All
        # identity/revision checks above happen immediately before this commit,
        # so clients observe either the complete old route or the complete new
        # route—never a cleared intermediate state.
        now = self._now()
        accepted_result = {**result, "plans": [plan]}
        order.route_nodes = [str(item) for item in plan.get("nodes", [])]
        self._apply_planner_result(
            accepted_result,
            now,
            order_id=order.order_id,
        )
        self._apply_simulated_route_metadata(robot, order, plan, now)
        self._set_order_status(
            order,
            "EXECUTING",
            robot=robot,
            start_lm=str(job.get("start_lm") or ""),
        )
        order.traffic_detour_edges = []
        robot.last_reason = "runtime route replan committed"
        robot.route_note = f"REPLAN: {self._plan_note(result)}"
        robot.updated_at = now
        self._event(
            "info",
            f"transactional runtime replan committed: {order.order_id} "
            f"{robot.name}@{job.get('start_lm')}->{final_goal}",
        )
        return 1

    def _finish_async_simulated_dispatch(self) -> int:
        with self._dispatch_job_lock:
            job = self._dispatch_job
            if job is None or not bool(job.get("done")):
                return 0
        # The result is already published, so joining only reaps the final
        # instructions of this finite worker thread. Keep the job slot occupied
        # until then so another caller cannot publish a replacement into the
        # tiny completion/finalizer window.
        if not self._planning_worker.join():
            return 0
        with self._dispatch_job_lock:
            if self._dispatch_job is not job:
                return 0
            self._dispatch_job = None
        corridor_gates = job.get("corridor_gates")
        gate_pins = (
            corridor_gates
            if isinstance(corridor_gates, dict)
            else None
        )
        if bool(job.get("discard")):
            self._release_controlled_corridor_gate_pins(gate_pins)
            self._last_async_job_kind = ""
            return 0
        self._last_async_job_kind = str(job.get("kind") or "dispatch")

        if job.get("kind") in {
            "prefetch",
            "prefetch_batch",
        }:
            try:
                return self._finish_async_rolling_prefetch(job)
            finally:
                self._release_controlled_corridor_gate_pins(gate_pins)
        if job.get("kind") == "runtime_replan":
            try:
                return self._finish_async_runtime_replan(job)
            finally:
                self._release_controlled_corridor_gate_pins(gate_pins)
        if job.get("kind") == "coupled_replan":
            try:
                return self._finish_async_coupled_replan(job)
            finally:
                self._release_controlled_corridor_gate_pins(gate_pins)

        entries = [
            entry
            for entry in job.get("entries", [])
            if self._async_dispatch_entry_is_current(entry)
        ]
        if not entries:
            self._release_controlled_corridor_gate_pins(gate_pins)
            return 0
        result = job.get("result")
        if not isinstance(result, dict):
            result = {
                "ok": False,
                "plans": [],
                "debug": {"reason": "background planner returned no result"},
            }
        try:
            dispatched, _ = self._finish_simulated_order_batch(
                entries,
                result,
                corridor_gates=gate_pins,
            )
            return dispatched
        finally:
            self._release_controlled_corridor_gate_pins(gate_pins)

    def _plan_for_robot(self, result: dict[str, Any], robot_name: str) -> dict[str, Any] | None:
        """Return one robot's plan from a shared MAPF planning result."""
        for plan in result.get("plans", []):
            if isinstance(plan, dict) and str(plan.get("robot") or "") == robot_name:
                return plan
        return None

    def _blacklist_failed_rolling_vacancy(
        self,
        job: dict[str, Any],
        robot: FleetRobot,
        request: dict[str, Any],
    ) -> None:
        """Remember a failed pocket only while the collapse is unchanged."""
        if not request.get("vacancyRecovery"):
            return
        raw_signature = job.get("vacancy_recovery_signature")
        if not isinstance(raw_signature, tuple) or not raw_signature:
            return
        signature = tuple(
            (str(name), str(start_lm), int(route_revision))
            for name, start_lm, route_revision in raw_signature
        )
        current_signature = tuple(sorted(
            (
                current.name,
                str(current.route_chunk_goal_lm or ""),
                int(current.route_revision),
            )
            for current in self._runtime_robots()
            if (
                not current.is_remote()
                and current.active_order_id
                and current.active_order_id in self.orders
                and self.orders[current.active_order_id].status
                not in TERMINAL_ORDER_STATUSES
            )
        ))
        if current_signature != signature:
            return
        pocket_lm = str(request.get("goalLm") or "")
        if not pocket_lm:
            return
        self._rolling_vacancy_recovery_blacklist.add(
            (signature, robot.name, pocket_lm)
        )

    def _finish_async_rolling_prefetch(self, job: dict[str, Any]) -> int:
        entries = self._current_rolling_prefetch_entries(job)
        if not entries:
            return 0
        result = job.get("result")
        if (
            not isinstance(result, dict)
            or not result.get("ok")
            or not result.get("plans")
        ):
            self._finish_failed_rolling_prefetch(job, entries, result)
            return 0
        corridor_gates = job.get("corridor_gates", {})
        if not isinstance(corridor_gates, dict):
            corridor_gates = {}
        result = self._rolling_result(
            result,
            {robot.name: final_goal for _, robot, _, final_goal, _ in entries},
            corridor_gates=corridor_gates,
        )
        finish_now = self._now()
        for entry in entries:
            self._commit_rolling_prefetch_entry(
                job,
                entry,
                result,
                corridor_gates=corridor_gates,
                finish_now=finish_now,
            )
        return 0

    def _current_rolling_prefetch_entries(
        self,
        job: dict[str, Any],
    ) -> list[RollingPrefetchResultEntry]:
        raw_entries = job.get("entries", [])
        if not isinstance(raw_entries, list):
            return []
        route_revisions = job.get("route_revisions", {})
        return [
            entry
            for entry in raw_entries
            if isinstance(entry, tuple)
            and len(entry) == 5
            and self.orders.get(entry[0].order_id) is entry[0]
            and self.robots.get(entry[1].name) is entry[1]
            and entry[1].active_order_id == entry[0].order_id
            and entry[1].route_revision
            == int(route_revisions.get(entry[1].name, -1))
            and entry[1].route_chunk_goal_lm
            == str(entry[2].get("startLm") or "")
            and bool(entry[1].trajectory)
        ]

    def _finish_failed_rolling_prefetch(
        self,
        job: dict[str, Any],
        entries: list[RollingPrefetchResultEntry],
        result: object,
    ) -> None:
        reason = (
            self._planner_failure_reason(result)
            if isinstance(result, dict)
            else "rolling prefetch returned no result"
        )
        debug = (
            result.get("debug", {})
            if (
                isinstance(result, dict)
                and isinstance(result.get("debug"), dict)
            )
            else {}
        )
        conflict_robot = self._planner_conflict_robot_name(reason)
        entry_names = {entry[1].name for entry in entries}
        isolated_member_failure = conflict_robot in entry_names
        self._record_rolling_prefetch_blockers(
            entries,
            debug,
            conflict_robot=conflict_robot,
        )
        for order, robot, request, final_goal, _ in entries:
            self._blacklist_failed_rolling_vacancy(job, robot, request)
            if self._stationary_failure_applies_to_robot(
                debug,
                robot.name,
            ):
                self._record_stationary_order_failure(order, debug)
                turn_lm = self._stationary_turn_conflict_lm(
                    debug,
                    robot.name,
                )
                start_lm = str(request.get("startLm") or "")
                if turn_lm:
                    self._queue_alternate_corridor_detour(
                        order,
                        start_lm,
                        final_goal,
                        avoid_lm=turn_lm,
                        replace_existing=True,
                    )
            if (
                not isolated_member_failure
                or robot.name == conflict_robot
            ):
                self._rolling_prefetch_failures[robot.name] = (
                    self._rolling_prefetch_failures.get(robot.name, 0) + 1
                )
            self._defer_rolling_prefetch(
                robot,
                order,
                retry_multiplier=(
                    2.0
                    if (
                        isolated_member_failure
                        and robot.name == conflict_robot
                    )
                    else 1.0
                ),
            )

    def _commit_rolling_prefetch_entry(
        self,
        job: dict[str, Any],
        entry: RollingPrefetchResultEntry,
        result: dict[str, Any],
        *,
        corridor_gates: dict[str, dict[str, Any]],
        finish_now: float,
    ) -> None:
        order, robot, request, final_goal, _ = entry
        plan = self._plan_for_robot(result, robot.name)
        if (
            plan is not None
            and not self._plan_follows_requested_clearance_route(
                order,
                request,
                plan,
            )
        ):
            self._defer_invalid_clearance_route(
                order,
                robot,
                self._now(),
                ValueError("planner changed the fixed route"),
            )
            return
        corridor_gate = corridor_gates.get(robot.name)
        if (
            plan is not None
            and isinstance(corridor_gate, dict)
            and bool(plan.get("corridorPassageDeferred"))
        ):
            intent = corridor_gate.get("intent")
            if (
                self._controlled_corridor_prefetch_intents.get(robot.name)
                is intent
            ):
                self._controlled_corridor_prefetch_intents.pop(
                    robot.name,
                    None,
                )
            corridor_gate = None
        if (
            plan is not None
            and isinstance(corridor_gate, dict)
            and not self._rolling_prefetch_gate_is_current(
                order,
                robot,
                request,
                plan,
                corridor_gate,
                finish_now,
            )
        ):
            return
        if plan is None or self._wait_only_rolling_plan(plan, final_goal):
            self._defer_failed_rolling_plan(
                job,
                entry,
                result,
            )
            return
        self._rolling_prefetch_retry_at.pop(robot.name, None)
        self._rolling_prefetch_failures.pop(robot.name, None)
        if self._append_rolling_prefetch(robot, order, plan, final_goal):
            return
        robot.pending_route = {
            "order_id": order.order_id,
            "start_lm": str(request.get("startLm") or ""),
            "final_goal": final_goal,
            "result": {**result, "plans": [plan]},
        }

    def _rolling_prefetch_gate_is_current(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        plan: dict[str, Any],
        corridor_gate: dict[str, Any],
        finish_now: float,
    ) -> bool:
        gate_current, gate_reason = (
            self._controlled_corridor_prefetch_plan_is_current(
                robot,
                request,
                plan,
                corridor_gate,
                now=finish_now,
            )
        )
        if (
            gate_current
            and not self._commit_controlled_corridor_prefetch_slot(
                robot,
                corridor_gate,
            )
        ):
            gate_current = False
            gate_reason = "corridor slot changed before command commit"
        if gate_current:
            return True
        self._handle_controlled_corridor_gate_rejection(
            robot.name,
            corridor_gate,
            gate_reason,
        )
        self._rolling_prefetch_retry_at[robot.name] = (
            finish_now
            + max(
                self._runtime_motion_step(),
                self._rolling_boundary_retry_interval(order),
            )
        )
        if self._robot_waits_at_rolling_boundary(robot):
            robot.last_reason = "waiting for refreshed corridor slot"
            robot.updated_at = finish_now
        self._event(
            "info",
            f"{robot.name} corridor continuation rescheduled: "
            f"{gate_reason}",
        )
        return False

    def _defer_failed_rolling_plan(
        self,
        job: dict[str, Any],
        entry: RollingPrefetchResultEntry,
        result: dict[str, Any],
    ) -> None:
        order, robot, request, final_goal, _ = entry
        self._record_rolling_prefetch_blockers(
            [(order, robot, request, final_goal, 0.0)],
            (
                result.get("debug", {})
                if isinstance(result.get("debug"), dict)
                else {}
            ),
            conflict_robot=robot.name,
        )
        self._blacklist_failed_rolling_vacancy(job, robot, request)
        self._rolling_prefetch_failures[robot.name] = (
            self._rolling_prefetch_failures.get(robot.name, 0) + 1
        )
        self._defer_rolling_prefetch(
            robot,
            order,
            retry_multiplier=1.0,
        )

    @staticmethod
    def _stationary_failure_applies_to_robot(
        debug: dict[str, Any],
        robot_name: str,
    ) -> bool:
        if not (
            debug.get("stationaryRobotWait")
            or debug.get("stationaryTurnEnvelopeBlock")
        ):
            return False
        conflicts = debug.get("continuousUnresolvedConflicts")
        if not isinstance(conflicts, list):
            return True
        named = {
            str(conflict.get("robot") or "")
            for conflict in conflicts
            if isinstance(conflict, dict)
            and str(conflict.get("robot") or "")
        }
        return not named or robot_name in named

    def _stationary_turn_conflict_lm(
        self,
        debug: dict[str, Any],
        robot_name: str,
    ) -> str:
        conflicts = debug.get("continuousUnresolvedConflicts")
        if not isinstance(conflicts, list):
            conflicts = []
        edge_ids = [
            str(conflict.get("edge") or "")
            for conflict in conflicts
            if isinstance(conflict, dict)
            and str(conflict.get("robot") or "") in {"", robot_name}
        ]
        fallback_edge = str(debug.get("continuousConflictEdge") or "")
        if fallback_edge:
            edge_ids.append(fallback_edge)
        prefix = "WAIT@ROTATE:"
        for edge_id in edge_ids:
            if not edge_id.startswith(prefix):
                continue
            lm_name = edge_id[len(prefix):].split("->", 1)[0].strip()
            if lm_name in self.landmarks:
                return lm_name
        return ""

    def _finish_async_coupled_replan(self, job: dict[str, Any]) -> int:
        cycle_key = tuple(str(name) for name in job.get("cycle", ()))
        entries = [entry for entry in job.get("entries", []) if isinstance(entry, dict)]
        current: list[tuple[FleetRobot, FleetOrder, dict[str, Any]]] = []
        for entry in entries:
            robot = self.robots.get(str(entry.get("robot") or ""))
            order = self.orders.get(str(entry.get("order") or ""))
            if (
                robot is None
                or order is None
                or robot.active_order_id != order.order_id
                or order.status in TERMINAL_ORDER_STATUSES
                or robot.route_revision != int(entry.get("routeRevision", -1))
                or not math.isclose(
                    float(robot.route_clock),
                    float(entry.get("routeClock", -1.0)),
                    rel_tol=0.0,
                    abs_tol=1e-6,
                )
                or self._safe_replan_start_lm(robot) != str(entry.get("start") or "")
            ):
                return 0
            current.append((robot, order, entry))

        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            self._record_coupled_replan_failure(cycle_key, result)
            return 0
        final_goals = {
            robot.name: str(entry.get("finalGoal") or "")
            for robot, _, entry in current
        }
        result = self._rolling_result(result, final_goals)
        plans = {
            str(plan.get("robot") or ""): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        if any(
            robot.name not in plans
            or self._wait_only_rolling_plan(plans[robot.name], final_goals[robot.name])
            for robot, _, _ in current
        ):
            self._record_coupled_replan_failure(cycle_key, result)
            return 0

        now = self._now()
        self._apply_planner_result(result, now)
        for robot, order, entry in current:
            plan = plans[robot.name]
            self._adopt_coupled_spatial_detour(
                order,
                plan,
                str(entry.get("finalGoal") or ""),
            )
            order.route_nodes = [str(node) for node in plan.get("nodes", [])]
            order.status = "EXECUTING"
            order.error = ""
            order.updated_at = now
            self._apply_simulated_route_metadata(robot, order, plan, now)
            robot.last_reason = "local coupled CBS committed"
            robot.last_replan_at = now
            robot.traffic_stall_since = None
            self._clear_wait_dependency(robot)
        self._clear_coupled_replan_attempts_for_members(
            cycle_key,
            include_subsets=True,
        )
        component_names = set(cycle_key)
        for state in (
            self._active_wait_cycles,
            self._wait_cycle_last_arbitration,
            self._wait_cycle_grant_signatures,
        ):
            for waiting_key in list(state):
                if set(waiting_key).issubset(component_names):
                    state.pop(waiting_key, None)
        self.traffic_metrics["coupledReplansSucceeded"] += 1
        self.traffic_metrics["cycleReplans"] += 1
        self._event(
            "warn",
            f"local CBS committed for wait cycle: {', '.join(cycle_key)}",
        )
        return len(current)

    def _record_coupled_replan_failure(
        self,
        cycle_key: tuple[str, ...],
        result: Any,
    ) -> None:
        self._coupled_replan_failures[cycle_key] = (
            self._coupled_replan_failures.get(cycle_key, 0) + 1
        )
        self.traffic_metrics["coupledReplansFailed"] += 1
        reason = self._planner_failure_reason(result) if isinstance(result, dict) else "no result"
        self._event(
            "warn",
            f"local CBS pending for wait cycle {', '.join(cycle_key)}: {reason}",
        )


__all__ = ["DispatchResultMixin"]
