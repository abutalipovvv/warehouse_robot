"""Own asynchronous planner job submission and start transitions."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from time import monotonic
from typing import Any

from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot
from fleet_manager.core.planning_models import (
    PlanCandidate,
    PlanningJob,
    PlanningJobStatus,
    PlanningPriority,
    PlanningReason,
)
from fleet_manager.core.planning_models import FrozenMapping
from fleet_manager.core.traffic.corridors.scheduling.corridor_scheduler import CorridorSlot


RollingPrefetchEntry = tuple[
    FleetOrder,
    FleetRobot,
    dict[str, Any],
    str,
    float,
]


@dataclass(slots=True)
class _RollingPrefetchBatch:
    """Prepared requests and corridor grants for one worker submission."""

    vacancy_recovery: bool
    prepared: list[RollingPrefetchEntry]
    corridor_gates: dict[str, dict[str, Any]] = field(default_factory=dict)
    requests: list[dict[str, Any]] = field(default_factory=list)
    payload: dict[str, Any] = field(default_factory=dict)
    job: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class _CoupledReplanMember:
    """Validated robot and order participating in one local CBS request."""

    robot: FleetRobot
    order: FleetOrder
    start_lm: str
    final_goal: str


@dataclass(slots=True)
class _CoupledReplanContext:
    """Mutable data passed through coupled-replan preparation stages."""

    robots: list[FleetRobot]
    winner: FleetRobot
    now: float
    cycle_key: tuple[str, ...]
    released_names: set[str] = field(default_factory=set)
    protected_starts: set[str] = field(default_factory=set)
    reserved_goals: set[str] = field(default_factory=set)
    requests: list[dict[str, Any]] = field(default_factory=list)
    entries: list[dict[str, Any]] = field(default_factory=list)
    payload: dict[str, Any] = field(default_factory=dict)
    job: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class _LegacyPlanningResult:
    """Compatibility hook output published back to the runtime owner."""

    job_token: int
    result: FrozenMapping


class AsyncPlanningJobMixin:
    """Own asynchronous planner job submission and start transitions."""

    def _async_simulated_dispatch_active(self) -> bool:
        with self._dispatch_job_lock:
            return self._dispatch_job is not None

    def _submit_async_planning_job(
        self,
        job: dict[str, Any],
        requests: list[dict[str, Any]],
        payload: dict[str, Any],
        *,
        failure_reason: str,
        thread_name: str,
    ) -> bool:
        """Submit immutable solver input and publish only to the runtime."""

        if (
            "_plan_valid_requests" in self.__dict__
            or not hasattr(self, "_planning_snapshot_factory")
        ):
            return self._submit_legacy_planning_hook(
                job,
                requests,
                payload,
                failure_reason=failure_reason,
                thread_name=thread_name,
            )

        try:
            planning_job = self._build_planning_job(
                job,
                requests,
                payload,
            )
        except Exception as exc:
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    self._dispatch_job = None
            self.planning_state.record_event("planning_job_failed")
            self._event(
                "error",
                f"planning_job_failed preparation: {type(exc).__name__}: {exc}",
            )
            return False

        if self._planning_worker.submit_job(
            planning_job,
            self._planning_solver_service.solve,
        ):
            return True

        self.planning_state.jobs.pop(planning_job.job_id, None)
        with self._dispatch_job_lock:
            if self._dispatch_job is job:
                self._dispatch_job = None
        return False

    def _build_planning_job(
        self,
        live_job: dict[str, Any],
        requests: list[dict[str, Any]],
        payload: dict[str, Any],
    ) -> PlanningJob:
        snapshot = self._planning_snapshot_for(requests, payload)
        self.planning_state.submission_sequence += 1
        sequence = self.planning_state.submission_sequence
        kind = str(live_job.get("kind") or "dispatch")
        reason = PlanningReason.from_job_kind(kind)
        robot_ids = tuple(
            sorted(
                str(request.get("name") or "")
                for request in requests
                if str(request.get("name") or "")
            )
        )
        job_id = f"{kind}-{sequence}"
        submitted_at = monotonic()
        deadline_seconds = self._planning_deadline_seconds(payload)
        planning_job = PlanningJob(
            job_id=job_id,
            reason=reason,
            priority=PlanningPriority.for_reason(reason),
            snapshot=snapshot,
            submitted_at=submitted_at,
            deadline=(
                submitted_at + deadline_seconds
                if deadline_seconds is not None
                else None
            ),
            coalescing_key=f"{kind}:{','.join(robot_ids)}",
            robot_ids=robot_ids,
            conflict_component_ids=(
                tuple(str(item) for item in live_job.get("cycle", ()))
                if kind == "coupled_replan"
                else ()
            ),
        )
        live_job["job_id"] = job_id
        live_job["expected_revision"] = snapshot.revision
        live_job["planning_job"] = planning_job
        self.planning_state.jobs[job_id] = planning_job
        self.planning_state.record_event("planning_job_submitted")
        self._event(
            "info",
            f"planning_job_submitted job_id={job_id} reason={reason.value} "
            f"priority={int(planning_job.priority)} "
            f"expected_revision={snapshot.revision} "
            f"robots={','.join(robot_ids)}",
        )
        if reason is PlanningReason.SAFETY_REPLAN:
            self.planning_state.record_event("safety_replan_requested")
            self._event(
                "warn",
                f"safety_replan_requested job_id={job_id} "
                f"robots={','.join(robot_ids)}",
            )
        elif reason is PlanningReason.DEADLOCK_RECOVERY:
            self.planning_state.record_event("deadlock_recovery_requested")
            self._event(
                "warn",
                f"deadlock_recovery_requested job_id={job_id} "
                f"robots={','.join(robot_ids)}",
            )
        return planning_job

    @staticmethod
    def _planning_deadline_seconds(
        payload: dict[str, Any],
    ) -> float | None:
        raw_value = payload.get("planningDeadlineSec")
        if raw_value is None:
            return None
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            return None
        return value if value > 0.0 else None

    def _collect_completed_planning_candidates(self) -> None:
        """Move worker output into live planning state on the runtime thread."""

        for update in self._planning_worker.take_job_events():
            planning_job = self.planning_state.jobs.get(update.job_id)
            if not isinstance(planning_job, PlanningJob):
                continue
            if update.status is PlanningJobStatus.RUNNING:
                if planning_job.status is PlanningJobStatus.QUEUED:
                    planning_job.transition(PlanningJobStatus.RUNNING)
                planning_job.started_at = update.occurred_at
                self.planning_state.record_event("planning_job_started")
                self._event(
                    "info",
                    f"planning_job_started job_id={planning_job.job_id} "
                    f"reason={planning_job.reason.value} "
                    f"priority={int(planning_job.priority)} "
                    f"expected_revision={planning_job.snapshot.revision} "
                    f"queue_wait_sec={max(0.0, update.occurred_at - planning_job.submitted_at):.6f}",
                )
                continue
            if (
                planning_job.status is PlanningJobStatus.QUEUED
                and update.status is PlanningJobStatus.CANCELLED
            ):
                planning_job.transition(PlanningJobStatus.CANCELLED)
            elif planning_job.status is PlanningJobStatus.QUEUED:
                planning_job.transition(PlanningJobStatus.RUNNING)
            if planning_job.status is PlanningJobStatus.RUNNING:
                planning_job.transition(update.status)
            planning_job.finished_at = update.occurred_at
            if update.status is PlanningJobStatus.CANCELLED:
                event_name = (
                    "planning_deadline_exceeded"
                    if "deadline" in update.message
                    else "planning_job_cancelled"
                )
                self.planning_state.record_event(event_name)
                self._event(
                    "warn",
                    f"{event_name} job_id={planning_job.job_id} "
                    f"reason={planning_job.reason.value} "
                    f"expected_revision={planning_job.snapshot.revision} "
                    f"detail={update.message}",
                )
            elif update.status is PlanningJobStatus.FAILED:
                self.planning_state.record_event("planning_job_failed")
                self._event(
                    "error",
                    f"planning_job_failed job_id={planning_job.job_id} "
                    f"reason={planning_job.reason.value} "
                    f"detail={update.message}",
                )

        for candidate in self._planning_worker.take_completed_results():
            if isinstance(candidate, _LegacyPlanningResult):
                with self._dispatch_job_lock:
                    live_job = self._dispatch_job
                    if (
                        live_job is not None
                        and id(live_job) == candidate.job_token
                    ):
                        live_job["result"] = candidate.result.to_dict()
                        live_job["done"] = True
                continue
            if not isinstance(candidate, PlanCandidate):
                continue
            with self._dispatch_job_lock:
                live_job = self._dispatch_job
                if (
                    live_job is None
                    or str(live_job.get("job_id") or "")
                    != candidate.job_id
                ):
                    continue
                live_job["candidate"] = candidate
                live_job["done"] = True

    def _submit_legacy_planning_hook(
        self,
        job: dict[str, Any],
        requests: list[dict[str, Any]],
        payload: dict[str, Any],
        *,
        failure_reason: str,
        thread_name: str,
    ) -> bool:
        """Preserve explicit test/extension hooks during the migration."""

        job_token = id(job)
        frozen_requests = deepcopy(requests)
        frozen_payload = deepcopy(payload)

        def plan_and_publish() -> None:
            try:
                result = self._plan_valid_requests(
                    frozen_requests,
                    frozen_payload,
                )
            except Exception as exc:  # pragma: no cover - worker safety net
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"{failure_reason}: {exc}"},
                }
            publish = getattr(
                self._planning_worker,
                "publish_result",
                None,
            )
            if callable(publish):
                publish(
                    _LegacyPlanningResult(
                        job_token=job_token,
                        result=FrozenMapping.from_mapping(result),
                    )
                )

        if self._planning_worker.submit(
            plan_and_publish,
            thread_name=thread_name,
        ):
            return True

        with self._dispatch_job_lock:
            if self._dispatch_job is job:
                self._dispatch_job = None
        return False

    def _start_async_simulated_dispatch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> bool:
        if not entries:
            return False
        try:
            requests, payload = self._prepare_simulated_order_batch(entries)
        except ValueError as exc:
            # In particular, never submit a traffic-clearance request after its
            # fixed evacuation suffix became invalid.  The bounded maintenance
            # lifecycle will cancel/requeue it; free A* must not invent a route
            # through the corridor owner this task is meant to release.
            for order, _, _, _ in entries:
                if order.status == "PLANNING":
                    order.status = "QUEUED"
                if order.internal_kind == "traffic_clearance":
                    self._set_order_error(
                        order,
                        f"traffic clearance route invalid: {exc}",
                    )
            return False
        corridor_gates: dict[str, dict[str, Any]] = {}
        runnable_entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ] = []
        runnable_requests: list[dict[str, Any]] = []
        gate_now = self._now()
        for entry, request in zip(entries, requests):
            order, robot, _, _ = entry
            gate = self._controlled_corridor_prefetch_gate(
                order,
                robot,
                request,
                prediction_offset=0.0,
                now=gate_now,
            )
            if gate is None:
                runnable_entries.append(entry)
                runnable_requests.append(request)
                continue
            if bool(gate.get("approachOnly")):
                # This temporal chunk deliberately ends at the external
                # corridor stop line. Keep the entry-side request in sync with
                # the worker request because result validation reads both.
                raw_request = entry[2]
                raw_request.clear()
                raw_request.update(request)
                runnable_entries.append(entry)
                runnable_requests.append(request)
                continue
            intent = gate.get("intent")
            slot = gate.get("slot")
            signature = (
                intent.get("signature")
                if isinstance(intent, dict)
                else None
            )
            departure_gate = gate.get("departureNotBefore")
            if not (
                bool(gate.get("ready"))
                and isinstance(intent, dict)
                and isinstance(slot, CorridorSlot)
                and isinstance(signature, tuple)
                and isinstance(departure_gate, dict)
            ):
                if order.status == "PLANNING":
                    order.status = "QUEUED"
                order.error = ""
                order.updated_at = gate_now
                continue
            request["departureNotBefore"] = [dict(departure_gate)]
            request["authorizedControlledRegions"] = list(slot.regions)
            request["noWaitNodes"] = list(gate.get("noWaitNodes", ()))
            runnable_entries.append(entry)
            runnable_requests.append(request)
            corridor_gates[robot.name] = {
                "intent": intent,
                "signature": signature,
                "slot": slot,
            }
        if not runnable_entries:
            return False
        entries = runnable_entries
        requests = runnable_requests
        payload["robots"] = requests
        job: dict[str, Any] = {
            "kind": "dispatch",
            "entries": list(entries),
            "requests": requests,
            "payload": payload,
            "done": False,
            "result": None,
        }
        if corridor_gates:
            job["corridor_gates"] = corridor_gates
            if not self._pin_controlled_corridor_gates(corridor_gates):
                for order, _, _, _ in entries:
                    if order.status == "PLANNING":
                        order.status = "QUEUED"
                return False
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                self._release_controlled_corridor_gate_pins(
                    corridor_gates,
                )
                for order, _, _, _ in entries:
                    if order.status == "PLANNING":
                        order.status = "QUEUED"
                return False
            self._dispatch_job = job

        if not self._submit_async_planning_job(
            job,
            requests,
            payload,
            failure_reason="background planner failed",
            thread_name="fleet-mapf-dispatch",
        ):
            self._release_controlled_corridor_gate_pins(
                corridor_gates,
            )
            for order, _, _, _ in entries:
                if order.status == "PLANNING":
                    order.status = "QUEUED"
            return False
        return True

    def _start_async_runtime_replan(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any]],
    ) -> bool:
        """Plan a replacement while leaving the executing route untouched."""
        order, robot, state = entry
        if order.internal_kind == "traffic_clearance":
            # Defense in depth: all normal producers already reject this task,
            # but a stale/injected transaction must not bypass its immutable
            # evacuation route.
            self._runtime_replans.pop(robot.name, None)
            return False
        start_lm = str(state.get("start_lm") or "")
        final_goal = self._active_order_target(order)
        if not final_goal or start_lm not in self.landmarks:
            self._defer_runtime_replan(
                order,
                robot,
                state,
                "runtime replan has no valid graph target",
            )
            return False
        try:
            escape_route = self._runtime_replan_fixed_escape_route(
                state,
                start_lm,
            )
            fixed_escape = bool(escape_route)
            planning_goal = (
                escape_route[-1]
                if fixed_escape
                else self._rolling_planning_goal(
                    start_lm,
                    final_goal,
                    order,
                    release_robot_names={robot.name},
                )
            )
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
            }
            if robot.pose is not None:
                request["startPose"] = dict(robot.pose)
            if fixed_escape:
                request["routeNodes"] = list(escape_route)
            else:
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    final_goal,
                    release_robot_names={robot.name},
                )
            payload = self._order_plan_payload(order, request) | {
                "robots": [request],
            }
            if fixed_escape:
                payload["blocked_lms"] = [
                    str(lm_name)
                    for lm_name in state.get("escape_blocked_lms", ())
                    if str(lm_name) in self.landmarks
                ]
        except (RuntimeError, ValueError) as exc:
            self._defer_runtime_replan(
                order,
                robot,
                state,
                f"runtime replan preparation failed: {exc}",
            )
            return False

        job: dict[str, Any] = {
            "kind": "runtime_replan",
            "order_id": order.order_id,
            "robot_name": robot.name,
            "generation": int(state.get("generation", 0) or 0),
            "route_revision": int(robot.route_revision),
            "route_clock": float(robot.route_clock),
            "start_lm": start_lm,
            "final_goal": final_goal,
            "escape_goal": planning_goal if fixed_escape else "",
            "request": request,
            "result": None,
            "done": False,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return False
            state["stage"] = "planning"
            state["last_attempt_at"] = self._now()
            self._dispatch_job = job
        order.status = "PLANNING"
        order.error = f"runtime replan pending: {state.get('reason', '')}"
        order.updated_at = self._now()

        if not self._submit_async_planning_job(
            job,
            [request],
            payload,
            failure_reason="background runtime replan failed",
            thread_name="fleet-mapf-runtime-replan",
        ):
            self._defer_runtime_replan(
                order,
                robot,
                state,
                "runtime replan planner worker unavailable",
            )
            return False
        return True

    def _start_async_rolling_prefetch(
        self,
        entries: (
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
            | list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]
        ),
    ) -> bool:
        if isinstance(entries, tuple):
            entries = [entries]
        if not entries:
            return False

        batch = self._prepare_rolling_prefetch_batch(entries)
        if batch is None:
            return False
        if not self._admit_rolling_prefetch_batch(batch):
            return False
        if not self._build_rolling_prefetch_job(batch):
            return False
        return self._submit_rolling_prefetch_job(batch)

    def _prepare_rolling_prefetch_batch(
        self,
        entries: list[RollingPrefetchEntry],
    ) -> _RollingPrefetchBatch | None:
        vacancy_recovery = bool(
            len(entries) == 1
            and entries[0][2].get("vacancyRecovery")
        )
        released_owners = {entry[1].name for entry in entries}
        boundary_recovery = all(
            float(entry[-1]) <= 0.000001
            for entry in entries
        )
        # A rolling chunk must not terminate on an unchanged boundary holder.
        # SIPP will correctly reject that occupied goal, but choosing it again
        # on every retry wastes the only planner turn and can make a healthy
        # corridor wave collapse at the next horizon boundary.  Participants
        # in this exact recovery wave remain releasable and are handled below.
        protected_starts = {
            str(entry[2].get("startLm") or "")
            for entry in entries
        }
        protected_starts.update(
            str(other.route_chunk_goal_lm or "")
            for other in self._runtime_robots()
            if (
                other.name not in released_owners
                and self._robot_waits_at_rolling_boundary(other)
                and other.route_chunk_goal_lm
            )
        )
        reserved_goals: set[str] = set()
        prepared: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]] = []
        for order, robot, raw_request, final_goal, offset in entries:
            request = dict(raw_request)
            start_lm = str(request.get("startLm") or "")
            try:
                if vacancy_recovery:
                    # This path was selected specifically to break a directed
                    # dependency cycle. Keep it fixed; rebuilding the ordinary
                    # final-order suffix recreates the same cycle.
                    planning_goal = str(request.get("goalLm") or "")
                else:
                    planning_goal = (
                        self._rolling_recovery_planning_goal(
                            start_lm,
                            final_goal,
                            order,
                            release_robot_names=released_owners,
                        )
                        if boundary_recovery
                        else self._rolling_planning_goal(
                            start_lm,
                            final_goal,
                            order,
                            release_robot_names=released_owners,
                        )
                    )
                    planning_goal = self._distinct_rolling_batch_goal(
                        order,
                        start_lm,
                        final_goal,
                        planning_goal,
                        reserved_goals=reserved_goals,
                        protected_starts=protected_starts,
                        release_robot_names=released_owners,
                    )
                request["goalLm"] = planning_goal
                if not vacancy_recovery:
                    request.pop("routeNodes", None)
                    self._attach_spatial_route_to_request(
                        request,
                        order,
                        start_lm,
                        planning_goal,
                        final_goal,
                        release_robot_names=released_owners,
                    )
            except ValueError as exc:
                if order.internal_kind != "traffic_clearance":
                    raise
                self._defer_invalid_clearance_route(
                    order,
                    robot,
                    self._now(),
                    exc,
                )
                continue
            reserved_goals.add(planning_goal)
            prepared.append((order, robot, request, final_goal, offset))

        if not prepared:
            return False
        return _RollingPrefetchBatch(
            vacancy_recovery=vacancy_recovery,
            prepared=prepared,
        )

    def _admit_rolling_prefetch_batch(
        self,
        batch: _RollingPrefetchBatch,
    ) -> bool:
        prepared = batch.prepared
        corridor_gates: dict[str, dict[str, Any]] = {}
        runnable: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
        ] = []
        corridor_gate_pending: list[FleetRobot] = []
        gate_now = self._now()
        # Register every authored-corridor intent before deciding whether this
        # batch is ready.  The runtime-thread calendar sees all contenders on
        # the next tick and assigns one deterministic global order.  Ordinary
        # open-space requests return ``None`` and keep the original fast path.
        for order, robot, request, final_goal, prediction_offset in prepared:
            gate = self._controlled_corridor_prefetch_gate(
                order,
                robot,
                request,
                prediction_offset=prediction_offset,
                now=gate_now,
            )
            if gate is None:
                runnable.append(
                    (order, robot, request, final_goal, prediction_offset)
                )
                continue
            if bool(gate.get("approachOnly")):
                # The assigned passage is beyond this rolling horizon. The
                # request was reduced to a normal, safe approach chunk and
                # therefore needs neither a corridor gate nor commit hook.
                runnable.append(
                    (order, robot, request, final_goal, prediction_offset)
                )
                continue
            intent = gate.get("intent")
            if not isinstance(intent, dict):
                self._queue_controlled_corridor_exit_clearance(robot)
                corridor_gate_pending.append(robot)
                continue
            if not bool(gate.get("ready")):
                self._queue_controlled_corridor_exit_clearance(robot)
                corridor_gate_pending.append(robot)
                continue
            slot = gate.get("slot")
            signature = intent.get("signature")
            if (
                not isinstance(slot, CorridorSlot)
                or not isinstance(signature, tuple)
            ):
                corridor_gate_pending.append(robot)
                continue
            departure_gate = gate.get("departureNotBefore")
            if not isinstance(departure_gate, dict):
                corridor_gate_pending.append(robot)
                continue
            request["departureNotBefore"] = [dict(departure_gate)]
            request["authorizedControlledRegions"] = list(slot.regions)
            request["noWaitNodes"] = list(gate.get("noWaitNodes", ()))
            runnable.append(
                (order, robot, request, final_goal, prediction_offset)
            )
            corridor_gates[robot.name] = {
                "intent": intent,
                "signature": signature,
                "slot": slot,
            }
        if corridor_gate_pending:
            # No MAPF failure occurred.  Waiting one runtime tick for the
            # central calendar is cheaper and more reliable than repeatedly
            # asking SIPP to rediscover the red light inside a no-wait chain.
            #
            # Record this as a scheduler service turn (not a failure).  Without
            # it, the same earliest-deadline robot whose *current* passage
            # still owns a slot is selected on every tick, preventing the
            # dispatcher from registering intents for the rest of the fleet.
            gate_retry = max(
                1.0,
                min(2.0, self._rolling_prefetch_lead() * 0.1),
            )
            for robot in corridor_gate_pending:
                self._rolling_prefetch_last_attempt_at[robot.name] = gate_now
                self._rolling_prefetch_retry_at[robot.name] = max(
                    self._rolling_prefetch_retry_at.get(
                        robot.name,
                        0.0,
                    ),
                    gate_now + gate_retry,
                )
        prepared = runnable
        if not prepared:
            return False
        batch.prepared = prepared
        batch.corridor_gates = corridor_gates
        return True

    def _build_rolling_prefetch_job(
        self,
        batch: _RollingPrefetchBatch,
    ) -> bool:
        vacancy_recovery = batch.vacancy_recovery
        prepared = batch.prepared
        corridor_gates = batch.corridor_gates
        order, _, request, _, offset = prepared[0]
        requests = [entry[2] for entry in prepared]
        payload = self._order_plan_payload(order, request) | {
            "robots": requests,
            "reservationOffsetSec": offset,
            # Boundary holders are movable participants in this request.
            # Let hybrid SIPP use local CBS if priority ordering alone cannot
            # release their coupled starts.
            # A full boundary recovery is intentionally handled by the fast
            # prioritized SIPP pass. Exponential CBS remains useful for a
            # small local conflict, but must never monopolise the runtime
            # planner for a 10-20 robot traffic wave.
            "allowCbsFallback": (
                False
                if vacancy_recovery
                else 1 < len(prepared) <= min(
                    4,
                    self.planner.local_cbs_max_robots,
                )
            ),
        }
        job: dict[str, Any] = {
            "kind": "prefetch_batch" if len(prepared) > 1 else "prefetch",
            "entries": prepared,
            "route_revisions": {
                robot.name: robot.route_revision
                for _, robot, _, _, _ in prepared
            },
            "result": None,
            "done": False,
        }
        if corridor_gates:
            job["corridor_gates"] = corridor_gates
            if not self._pin_controlled_corridor_gates(corridor_gates):
                return False
        if vacancy_recovery:
            job["vacancy_recovery_signature"] = (
                self._rolling_vacancy_recovery_signature
            )
        batch.requests = requests
        batch.payload = payload
        batch.job = job
        return True

    def _submit_rolling_prefetch_job(
        self,
        batch: _RollingPrefetchBatch,
    ) -> bool:
        prepared = batch.prepared
        corridor_gates = batch.corridor_gates
        requests = batch.requests
        payload = batch.payload
        job = batch.job
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                self._release_controlled_corridor_gate_pins(
                    corridor_gates,
                )
                return False
            self._dispatch_job = job
        attempt_now = self._now()
        for _, robot, _, _, offset in prepared:
            self._rolling_prefetch_last_attempt_at[robot.name] = attempt_now
            if float(offset) <= 0.000001:
                robot.rolling_boundary_since = (
                    robot.rolling_boundary_since or attempt_now
                )

        if not self._submit_async_planning_job(
            job,
            requests,
            payload,
            failure_reason="background prefetch failed",
            thread_name="fleet-mapf-prefetch",
        ):
            self._release_controlled_corridor_gate_pins(
                corridor_gates,
            )
            return False
        return True

    def _start_async_coupled_replan(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> bool:
        context = self._coupled_replan_context(robots, winner, now)
        if not self._coupled_replan_is_eligible(context):
            return False
        if not self._build_coupled_replan_requests(context):
            return False
        self._build_coupled_replan_job(context)
        return self._submit_coupled_replan_job(context)

    def _coupled_replan_context(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> _CoupledReplanContext:
        """Expand a wait component and capture its stable identity."""
        expanded = self._expand_coupled_replan_component(robots, winner)
        return _CoupledReplanContext(
            robots=expanded,
            winner=winner,
            now=now,
            cycle_key=tuple(sorted(robot.name for robot in expanded)),
        )

    def _coupled_replan_is_eligible(
        self,
        context: _CoupledReplanContext,
    ) -> bool:
        """Apply route-safety, fairness, size and retry guards in order."""
        for robot in context.robots:
            order = self._active_order_for_robot(robot)
            if order is not None and order.internal_kind == "traffic_clearance":
                return False
        if self._coupled_replan_yields_planner_turn(context.now):
            return False

        cycle_key = context.cycle_key
        if len(cycle_key) < 2:
            return False
        if len(cycle_key) > self.planner.local_cbs_max_robots:
            self._record_coupled_replan_start_failure(
                cycle_key,
                event_message=(
                    f"wait cycle has {len(cycle_key)} robots; local CBS cap is "
                    f"{self.planner.local_cbs_max_robots}"
                ),
            )
            return False
        # An unchanged component cannot produce a different local-CBS result.
        if self._coupled_replan_failures.get(cycle_key, 0) > 0:
            return False
        last_attempt = self._coupled_replan_last_attempt.get(cycle_key, 0.0)
        return (
            context.now - last_attempt
            >= self._deadlock_coupled_replan_interval()
        )

    def _coupled_replan_yields_planner_turn(self, now: float) -> bool:
        """Preserve fair turns for urgent prefetch and queued dispatch."""
        if self._last_async_job_kind not in {
            "prefetch",
            "prefetch_batch",
        }:
            prefetch_entries = self._ready_rolling_prefetch_entries()
            if (
                prefetch_entries
                and float(prefetch_entries[0][-1])
                <= self._rolling_prefetch_urgent_lead()
            ):
                return True
        return bool(
            self._last_async_job_kind != "dispatch"
            and self._queued_simulated_dispatch_waiting(now)
        )

    def _record_coupled_replan_start_failure(
        self,
        cycle_key: tuple[str, ...],
        *,
        event_message: str = "",
    ) -> None:
        """Arm deterministic evacuation once for an unplannable component."""
        if cycle_key in self._coupled_replan_failures:
            return
        self._coupled_replan_failures[cycle_key] = 1
        self.traffic_metrics["coupledReplansFailed"] += 1
        if event_message:
            self._event("warn", event_message)

    def _build_coupled_replan_requests(
        self,
        context: _CoupledReplanContext,
    ) -> bool:
        """Validate members and build requests in winner-first order."""
        context.released_names = set(context.cycle_key)
        context.protected_starts = {
            start_lm
            for robot in context.robots
            if (start_lm := self._safe_replan_start_lm(robot))
            in self.landmarks
        }
        ordered = [context.winner] + sorted(
            (
                robot
                for robot in context.robots
                if robot.name != context.winner.name
            ),
            key=lambda robot: robot.name,
        )
        for robot in ordered:
            member = self._validated_coupled_replan_member(
                context,
                robot,
            )
            if member is None:
                return False
            if not self._append_coupled_replan_request(context, member):
                return False
        return True

    def _validated_coupled_replan_member(
        self,
        context: _CoupledReplanContext,
        robot: FleetRobot,
    ) -> _CoupledReplanMember | None:
        """Resolve the graph endpoints required for one component member."""
        order = self._active_order_for_robot(robot)
        start_lm = self._safe_replan_start_lm(robot)
        final_goal = (
            self._active_order_target(order)
            if order is not None
            else robot.route_final_lm or robot.target_lm
        )
        if (
            order is not None
            and start_lm
            and start_lm in self.landmarks
            and final_goal in self.landmarks
        ):
            return _CoupledReplanMember(
                robot=robot,
                order=order,
                start_lm=start_lm,
                final_goal=final_goal,
            )
        self._record_coupled_replan_start_failure(
            context.cycle_key,
            event_message=(
                f"local CBS cannot start for wait cycle "
                f"{', '.join(context.cycle_key)}: "
                f"{robot.name} is between graph LMs; "
                "corridor evacuation armed"
            ),
        )
        return None

    def _append_coupled_replan_request(
        self,
        context: _CoupledReplanContext,
        member: _CoupledReplanMember,
    ) -> bool:
        """Build one routed request and its asynchronous commit snapshot."""
        planning_goal = self._rolling_planning_goal(
            member.start_lm,
            member.final_goal,
            member.order,
            release_robot_names=context.released_names,
        )
        planning_goal = self._distinct_rolling_batch_goal(
            member.order,
            member.start_lm,
            member.final_goal,
            planning_goal,
            reserved_goals=context.reserved_goals,
            protected_starts=context.protected_starts,
            release_robot_names=context.released_names,
        )
        robot = member.robot
        request: dict[str, Any] = {
            "name": robot.name,
            "startLm": member.start_lm,
            "goalLm": planning_goal,
            "startPose": (
                dict(robot.pose)
                if robot.pose is not None
                else self._pose_at_landmark(member.start_lm)
            ),
        }
        try:
            self._attach_spatial_route_to_request(
                request,
                member.order,
                member.start_lm,
                planning_goal,
                member.final_goal,
                release_robot_names=context.released_names,
            )
        except ValueError:
            self._record_coupled_replan_start_failure(context.cycle_key)
            return False
        context.reserved_goals.add(planning_goal)
        context.requests.append(request)
        context.entries.append(
            {
                "robot": robot.name,
                "order": member.order.order_id,
                "start": member.start_lm,
                "finalGoal": member.final_goal,
                "routeRevision": robot.route_revision,
                "routeClock": float(robot.route_clock),
            }
        )
        return True

    def _build_coupled_replan_job(
        self,
        context: _CoupledReplanContext,
    ) -> None:
        """Create the hybrid planner payload and immutable job snapshot."""
        first_order = self.orders[str(context.entries[0]["order"])]
        context.payload = self._order_plan_payload(
            first_order,
            context.requests[0],
        ) | {
            "robots": context.requests,
            "plannerBackend": "hybrid",
            "allowCbsFallback": True,
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
            "reservedEdgeDetourEnabled": False,
        }
        context.payload.pop("blocked_edges", None)
        context.job = {
            "kind": "coupled_replan",
            "cycle": context.cycle_key,
            "entries": context.entries,
            "requests": context.requests,
            "result": None,
            "done": False,
        }

    def _submit_coupled_replan_job(
        self,
        context: _CoupledReplanContext,
    ) -> bool:
        """Claim the worker slot, submit local CBS and publish start metrics."""
        job = context.job
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return False
            self._dispatch_job = job
        if not self._submit_async_planning_job(
            job,
            context.requests,
            context.payload,
            failure_reason="background coupled CBS failed",
            thread_name="fleet-mapf-coupled-cbs",
        ):
            return False
        self._coupled_replan_last_attempt[context.cycle_key] = context.now
        self.traffic_metrics["coupledReplansStarted"] += 1
        self._event(
            "warn",
            f"local CBS started for wait cycle: "
            f"{', '.join(context.cycle_key)}",
        )
        return True
