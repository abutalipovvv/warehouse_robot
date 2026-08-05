"""Parse, queue and admit fleet orders."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any

from fleet_manager.manager.tasks.statuses import (
    FLEET_CONTROL_OWNER_ID,
    ORDER_SEQUENCE_KEYS,
    ORDER_TARGET_KEYS,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.state import FleetState


class AdmissionStatus(str, Enum):
    """Outcome of choosing a robot for a validated order."""

    ACCEPTED = "accepted"
    REJECTED = "rejected"
    DEFERRED = "deferred"


@dataclass(frozen=True, slots=True)
class OrderAdmissionResult:
    """Explicit admission decision used before dispatch planning."""

    status: AdmissionStatus
    robot_id: str | None
    reason: str = ""


class OrderAdmissionService:
    """Validate payloads and build orders from explicit dependencies."""

    def __init__(
        self,
        fleet_state: FleetState,
        landmarks: Mapping[str, Any],
        clock: Callable[[], float],
        *,
        robot_enabled: Callable[[FleetRobot], bool] | None = None,
        refresh_remote: Callable[[FleetRobot], None] | None = None,
        remote_owner: Callable[[FleetRobot], str] | None = None,
        robot_landmark: Callable[[FleetRobot], str] | None = None,
    ) -> None:
        self._fleet_state = fleet_state
        self._landmarks = landmarks
        self._clock = clock
        self._robot_enabled = robot_enabled or (lambda _robot: True)
        self._refresh_remote = refresh_remote or (lambda _robot: None)
        self._remote_owner = remote_owner or (lambda _robot: "")
        self._robot_landmark = robot_landmark or (
            lambda robot: robot.current_lm
        )

    def build(self, payload: Mapping[str, Any]) -> FleetOrder:
        order_id = str(
            payload.get("id")
            or payload.get("orderId")
            or payload.get("taskId")
            or payload.get("externalId")
            or ""
        ).strip()
        now = float(self._clock())
        if not order_id:
            order_id = f"order-{int(now * 1000)}"

        targets = self.target_landmarks(payload)
        if not targets:
            raise ValueError("targetLm/goalLm/location is required")
        for target_lm in targets:
            if target_lm not in self._landmarks:
                raise ValueError(f"unknown target LM: {target_lm}")

        vehicle = str(
            payload.get("vehicle")
            or payload.get("robot")
            or payload.get("robotName")
            or payload.get("name")
            or ""
        ).strip()
        if vehicle and vehicle not in self._fleet_state.robots:
            raise ValueError(f"unknown robot: {vehicle}")

        try:
            priority = int(payload.get("priority", 0) or 0)
        except (TypeError, ValueError):
            priority = 0

        return FleetOrder(
            order_id=order_id,
            target_lm=targets[0],
            vehicle=vehicle,
            priority=priority,
            external_id=str(
                payload.get("externalId") or payload.get("taskId") or ""
            ).strip(),
            targets=targets,
            speed=self.float_value(payload, ("speed", "routeSpeed")),
            acceleration=self.float_value(
                payload,
                (
                    "acceleration",
                    "routeAcceleration",
                    "route_acceleration",
                ),
            ),
            rotate=self.bool_value(
                payload,
                ("rotate", "simulateRotation", "simulate_rotation"),
            ),
            turn_speed=self.float_value(
                payload,
                (
                    "turnSpeed",
                    "turn_speed",
                    "rotationSpeed",
                    "rotation_speed",
                ),
            ),
            stretch_motion_to_reservation_ticks=self.bool_value(
                payload,
                (
                    "stretchMotionToReservationTicks",
                    "stretch_motion_to_reservation_ticks",
                ),
                default=True,
            ),
            created_at=now,
            updated_at=now,
        )

    @classmethod
    def target_landmarks(cls, payload: Mapping[str, Any]) -> list[str]:
        targets: list[str] = []
        for key in ORDER_SEQUENCE_KEYS:
            raw_sequence = payload.get(key)
            if not isinstance(raw_sequence, list):
                continue
            for item in raw_sequence:
                target_lm = cls.target_landmark(item)
                if target_lm:
                    targets.append(target_lm)
            if targets:
                return targets

        target_lm = cls.target_landmark(payload)
        return [target_lm] if target_lm else []

    @staticmethod
    def target_landmark(item: Any) -> str:
        if isinstance(item, Mapping):
            for key in ORDER_TARGET_KEYS:
                target_lm = str(item.get(key) or "").strip()
                if target_lm:
                    return target_lm
            return ""
        return str(item or "").strip()

    @staticmethod
    def float_value(
        payload: Mapping[str, Any],
        keys: tuple[str, ...],
        default: float = 0.0,
    ) -> float:
        for key in keys:
            if key not in payload:
                continue
            try:
                return float(payload.get(key) or default)
            except (TypeError, ValueError):
                return default
        return default

    @staticmethod
    def bool_value(
        payload: Mapping[str, Any],
        keys: tuple[str, ...],
        default: bool = False,
    ) -> bool:
        for key in keys:
            if key not in payload:
                continue
            value = payload.get(key)
            if isinstance(value, str):
                return value.strip().lower() in {"1", "true", "yes", "on"}
            return bool(value)
        return default

    def admission_result(self, order: FleetOrder) -> OrderAdmissionResult:
        """Choose the best available robot without starting a planner."""

        candidates = self.candidate_robots(order)
        if not candidates:
            return OrderAdmissionResult(
                status=AdmissionStatus.DEFERRED,
                robot_id=None,
                reason="no available robot",
            )
        return OrderAdmissionResult(
            status=AdmissionStatus.ACCEPTED,
            robot_id=candidates[0].name,
        )

    def candidate_robots(self, order: FleetOrder) -> list[FleetRobot]:
        """Return deterministic candidates for one validated order."""

        if order.vehicle:
            robot = self._fleet_state.robots.get(order.vehicle)
            if robot is None:
                return []
            if self.can_accept(robot, explicit=True):
                return [robot]
            return []

        candidates = [
            robot
            for robot in self._fleet_state.robots.values()
            if self.can_accept(robot, explicit=False)
        ]
        candidates.sort(
            key=lambda robot: (
                self.landmark_distance(
                    self._robot_landmark(robot),
                    order.target_lm,
                ),
                robot.name,
            )
        )
        return candidates

    def can_accept(self, robot: FleetRobot, *, explicit: bool) -> bool:
        """Check order ownership and remote-control availability."""

        if not self._robot_enabled(robot):
            return False
        if robot.is_remote():
            self._refresh_remote(robot)
            if not robot.remote_online:
                return False
            owner_id = self._remote_owner(robot)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return False
            if robot.status in {"LOCALIZING", "OFFLINE", "ERROR"}:
                return False
        if robot.active_order_id or robot.target_lm or robot.trajectory:
            return False
        if robot.status in {"MOVING", "WAITING", "PLANNING", "BLOCKED"}:
            return False
        if robot.status in {"STOPPED", "MANUAL"} and not explicit:
            return False
        return True

    def landmark_distance(self, start_lm: str, goal_lm: str) -> float:
        """Return Euclidean landmark distance for stable candidate ordering."""

        start = self._landmarks.get(start_lm)
        goal = self._landmarks.get(goal_lm)
        if start is None or goal is None:
            return float("inf")
        return math.hypot(goal.x - start.x, goal.y - start.y)


DispatchEntry = tuple[FleetOrder, FleetRobot, dict[str, Any], str]
RollingPrefetchEntry = tuple[
    FleetOrder,
    FleetRobot,
    dict[str, Any],
    str,
    float,
]


@dataclass(slots=True)
class _DispatchCycle:
    """Mutable state for one ordered dispatch scheduler turn."""

    force: bool
    async_simulated: bool
    dispatched: int
    now: float
    clearance_departure_ready: bool
    queued_dispatch_waiting: bool
    early_prefetch_entries: list[RollingPrefetchEntry] | None
    prefetch_turn_after_dispatch: bool
    recovery_yields_dispatch_turn: bool
    live_wait_chain_sinks: set[str] = field(default_factory=set)
    stationary_release_names: set[str] = field(default_factory=set)
    queued_orders: list[FleetOrder] = field(default_factory=list)
    handled: set[str] = field(default_factory=set)
    ready: list[DispatchEntry] = field(default_factory=list)
    predeparture_release_names: set[str] = field(default_factory=set)
    predeparture_protected_names: set[str] = field(default_factory=set)
    coordinated_release_names: set[str] = field(default_factory=set)
    planning_budget: int = 0


@dataclass(frozen=True, slots=True)
class _PreparedDispatchPlan:
    """A planner result that is safe to commit to one robot and order."""

    result: dict[str, Any]
    plan: dict[str, Any]
    remote_route: dict[str, Any] | None
    committed_at: float


class OrderAdmissionMixin:
    """Parse, queue and admit fleet orders."""

    def _orders_list(self) -> list[dict[str, Any]]:
        return self.task_manager.ordered_payloads(
            enabled=self._order_enabled,
            limit=120,
        )

    def _build_orders(self, payload: dict[str, Any]) -> list[FleetOrder]:
        return [self._build_order(payload)]

    def _build_order(self, payload: dict[str, Any]) -> FleetOrder:
        return self._order_admission_service.build(payload)

    def _target_lms_from_payload(self, payload: dict[str, Any]) -> list[str]:
        return OrderAdmissionService.target_landmarks(payload)

    def _dispatch_orders(
        self,
        force: bool = False,
        *,
        async_simulated: bool = False,
    ) -> int:
        cycle = self._prepare_dispatch_cycle(
            force=force,
            async_simulated=async_simulated,
        )
        completed = self._start_dispatch_runtime_replan(cycle)
        if completed is not None:
            return completed

        self._collect_ready_dispatch_entries(cycle)
        completed = self._start_dispatch_prefetch(cycle)
        if completed is not None:
            return completed

        planning_calls = self._dispatch_ready_entry_batches(cycle)
        self._dispatch_remaining_orders(cycle, planning_calls)
        return cycle.dispatched

    def _prepare_dispatch_cycle(
        self,
        *,
        force: bool,
        async_simulated: bool,
    ) -> _DispatchCycle:
        dispatched = self._finish_async_simulated_dispatch() if async_simulated else 0
        now = self._now()
        self._prune_stationary_order_retry_state()
        self._prune_dispatch_conflict_dependencies()
        self._prune_stationary_clearance_relocations(now)
        clearance_departure_ready = any(
            order.internal_kind == "traffic_clearance"
            and self._queued_simulated_order_dispatch_ready(order, now)
            for order in self.orders.values()
        )
        queued_dispatch_waiting = (
            async_simulated
            and self._queued_simulated_dispatch_waiting(now)
        )
        early_prefetch_entries: (
            list[
                tuple[
                    FleetOrder,
                    FleetRobot,
                    dict[str, Any],
                    str,
                    float,
                ]
            ]
            | None
        ) = None
        if (
            async_simulated
            and not self._async_simulated_dispatch_active()
        ):
            early_prefetch_entries = (
                self._ready_rolling_prefetch_entries()
            )
        prefetch_turn_after_dispatch = bool(
            early_prefetch_entries
            and self._last_async_job_kind == "dispatch"
            and (
                float(early_prefetch_entries[0][-1])
                <= self._rolling_prefetch_urgent_lead()
                or any(
                    self._rolling_prefetch_failures.get(
                        entry[1].name,
                        0,
                    )
                    > 0
                    for entry in early_prefetch_entries
                )
            )
        )
        recovery_yields_dispatch_turn = bool(
            queued_dispatch_waiting
            and self._last_async_job_kind == "runtime_replan"
        )
        return _DispatchCycle(
            force=force,
            async_simulated=async_simulated,
            dispatched=dispatched,
            now=now,
            clearance_departure_ready=clearance_departure_ready,
            queued_dispatch_waiting=queued_dispatch_waiting,
            early_prefetch_entries=early_prefetch_entries,
            prefetch_turn_after_dispatch=prefetch_turn_after_dispatch,
            recovery_yields_dispatch_turn=recovery_yields_dispatch_turn,
        )

    def _start_dispatch_runtime_replan(
        self,
        cycle: _DispatchCycle,
    ) -> int | None:
        async_simulated = cycle.async_simulated
        clearance_departure_ready = cycle.clearance_departure_ready
        recovery_yields_dispatch_turn = cycle.recovery_yields_dispatch_turn
        prefetch_turn_after_dispatch = cycle.prefetch_turn_after_dispatch
        dispatched = cycle.dispatched
        now = cycle.now
        if (
            async_simulated
            and not clearance_departure_ready
            and not recovery_yields_dispatch_turn
            and not prefetch_turn_after_dispatch
            and not self._async_simulated_dispatch_active()
        ):
            # A queued departure can be the stationary sink of several live
            # routes.  Planning that departure alone is insufficient when an
            # upstream robot is already holding inside its turn envelope: the
            # planner sees the upstream body as a stationary reservation and
            # keeps returning the same impossible route.  Open one graph-safe
            # vacancy first, retaining the upstream robot's active order and
            # committed trajectory until the replacement is ready.
            self._queue_commanded_sink_vacancy_replan(now)
            runtime_replan = self._ready_runtime_replan_entry(now)
            if runtime_replan is not None and self._start_async_runtime_replan(
                runtime_replan,
            ):
                # A robot holding an already occupied graph resource is more
                # urgent than a fresh departure.  It still uses the same one
                # planner slot, so runtime motion and HTTP rendering remain
                # independent from MAPF CPU work.
                return dispatched
        return None

    def _collect_ready_dispatch_entries(
        self,
        cycle: _DispatchCycle,
    ) -> None:
        force = cycle.force
        now = cycle.now
        # Compute this before quarantine filtering. A commanded stationary
        # robot at the sink of a live wait-for chain is precisely the robot
        # that must be released; excluding it because its earlier solo plans
        # failed makes the quarantine self-perpetuating.
        live_wait_chain_sinks = self._live_stationary_wait_chain_sink_names()
        stationary_release_names = self._stationary_release_robot_names()
        queued_orders = [
            order for order in self.orders.values()
            if order.status == "QUEUED"
            and (
                str(order.vehicle or order.assigned_robot or "")
                in live_wait_chain_sinks
                or self._stationary_order_retry_ready(
                    order,
                    force=force,
                )
            )
            and (
                force
                or not order.error
                or now - order.updated_at >= self._order_dispatch_retry_interval(order)
            )
            and (
                force
                or self._dispatch_conflict_dependency_ready(order)
            )
        ]
        queued_orders.sort(
            key=lambda item: (
                bool(item.error),
                -int(item.priority or 0),
                item.updated_at if item.error else item.created_at,
                item.order_id,
            )
        )

        # Dynamic orders used to call MAPF independently for every idle robot.
        # Apart from being expensive, that made all other robots look parked
        # and produced a fleet full of wait-only plans.  Plan a small coupled
        # component at a time and cap synchronous work per runtime tick.
        handled: set[str] = set()
        ready = self._ready_simulated_order_entries(queued_orders)
        self._register_ready_dispatch_corridor_intents(
            ready,
            now=now,
        )
        handled.update(
            order.order_id
            for order in queued_orders
            if order.status != "QUEUED"
            or str(order.error or "").startswith("manual graph reconnect blocked:")
        )
        # Two route-less departures can be mutually stationary before a
        # runtime wait-for graph exists at all: A's first suffix crosses B's
        # start and B's first suffix crosses A's start. After bounded normal
        # attempts, re-running their unchanged spatial routes through
        # SIPP/CBS only repeats the same priority cycle. Give the
        # lower-priority member a stable spatial bypass first, then submit the
        # complete pair as one temporal release wave. This remains congestion
        # A* + Rolling SIPP/CBS; the bootstrap merely supplies them with a
        # solvable set of spatial routes.
        (
            predeparture_release_names,
            predeparture_protected_names,
        ) = (
            self._coordinate_mutual_stationary_departures(ready)
        )
        # A pre-departure component is already the exact connected component
        # selected for this scheduler turn.  Unioning it with every unrelated
        # stationary-release owner recreates the fleet-wide batches this
        # bootstrap is meant to avoid. Ordinary stationary releases retain
        # their existing grouping when no pre-departure component is active.
        coordinated_release_names = (
            predeparture_release_names or stationary_release_names
        )
        if coordinated_release_names:
            # Release a parked terminal dependency before unrelated recovery
            # batches. The existing priority/FIFO order stays stable within
            # the release and ordinary groups.
            ready.sort(
                key=lambda entry: entry[1].name not in coordinated_release_names
        )
        if self._controlled_corridor_scheduler is not None:
            # The calendar is fleet-wide, while the ordinary order queue is
            # creation-time ordered. Always service a robot whose green slot
            # already exists before repeatedly probing an earlier red-light
            # order. Without this join between the two queues, slots were
            # assigned to later robots but the dispatcher retried the same
            # unscheduled first three orders forever.
            ready.sort(
                key=lambda entry: (
                    entry[1].name not in coordinated_release_names
                    if coordinated_release_names
                    else False,
                    *self._corridor_dispatch_readiness_key(entry),
                )
            )
        cycle.live_wait_chain_sinks = live_wait_chain_sinks
        cycle.stationary_release_names = stationary_release_names
        cycle.queued_orders = queued_orders
        cycle.handled = handled
        cycle.ready = ready
        cycle.predeparture_release_names = predeparture_release_names
        cycle.predeparture_protected_names = predeparture_protected_names
        cycle.coordinated_release_names = coordinated_release_names

    def _start_dispatch_prefetch(
        self,
        cycle: _DispatchCycle,
    ) -> int | None:
        async_simulated = cycle.async_simulated
        dispatched = cycle.dispatched
        early_prefetch_entries = cycle.early_prefetch_entries
        ready = cycle.ready
        if async_simulated and not self._async_simulated_dispatch_active():
            prefetch_entries = (
                early_prefetch_entries
                if early_prefetch_entries is not None
                else self._ready_rolling_prefetch_entries()
            )
            prefetch = prefetch_entries[0] if prefetch_entries else None
            prefetch_is_urgent = bool(
                prefetch is not None
                and float(prefetch[-1]) <= self._rolling_prefetch_urgent_lead()
            )
            prefetch_repeatedly_blocked = bool(
                prefetch_entries
                and any(
                    self._rolling_prefetch_failures.get(entry[1].name, 0) > 0
                    for entry in prefetch_entries
                )
            )
            dispatch_turn_after_prefetch = bool(
                ready
                and self._last_async_job_kind in {
                    "prefetch",
                    "prefetch_batch",
                }
            )
            dispatch_turn_after_recovery = bool(
                ready
                and self._last_async_job_kind in {
                    "runtime_replan",
                    "coupled_replan",
                }
            )
            recovery_turn_after_dispatch = bool(
                prefetch_repeatedly_blocked
                and ready
                and self._last_async_job_kind in {"dispatch", "coupled_replan"}
            )
            # Fill idle robots before spending the only planner slot on a
            # healthy, non-urgent continuation. A route inside its final
            # critical seconds wins the first turn. Once that prefetch
            # completes, one ready departure receives the next planner turn
            # before another continuation may run. This bounded alternation
            # prevents a stream of healthy rolling chunks from leaving
            # commanded stationary robots route-less indefinitely.
            #
            # A failed continuation already uses the same alternating rule:
            # the stationary departure may be its physical blocker, then the
            # recovery prefetch receives the following turn.
            if prefetch is not None and (
                not ready
                or (
                    not dispatch_turn_after_prefetch
                    and not dispatch_turn_after_recovery
                    and prefetch_is_urgent
                    and not prefetch_repeatedly_blocked
                )
                or recovery_turn_after_dispatch
            ):
                if self._start_async_rolling_prefetch(
                    prefetch_entries
                ):
                    return dispatched
        return None

    def _dispatch_ready_entry_batches(
        self,
        cycle: _DispatchCycle,
    ) -> int:
        async_simulated = cycle.async_simulated
        dispatched = cycle.dispatched
        handled = cycle.handled
        ready = cycle.ready
        stationary_release_names = cycle.stationary_release_names
        coordinated_release_names = cycle.coordinated_release_names
        predeparture_protected_names = cycle.predeparture_protected_names
        planning_budget = self._dispatch_plan_budget()
        batch_size = self._dispatch_batch_size()
        planning_calls = 0
        while ready and planning_calls < planning_budget:
            first = ready.pop(0)
            motion_key = self._order_motion_key(first[0])
            group = [first]
            group_limit = self._dispatch_recovery_group_limit(
                first[0],
                first[1],
                batch_size,
            )
            if first[1].name in coordinated_release_names:
                # Several queued departures can physically box one another in
                # a dense Kiva junction. Planning the first blocker alone
                # cannot release that component; coordinate all currently
                # identified departures up to the bounded local MAPF cap.
                group_limit = min(
                    self.planner.local_cbs_max_robots,
                    max(batch_size, len(coordinated_release_names)),
                )
            remaining: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]] = []
            for entry in ready:
                if (
                    len(group) < group_limit
                    and self._order_motion_key(entry[0]) == motion_key
                    and (
                        first[1].name not in coordinated_release_names
                        or entry[1].name in coordinated_release_names
                    )
                ):
                    group.append(entry)
                else:
                    remaining.append(entry)
            ready = remaining
            if first[1].name in stationary_release_names:
                for release_order, _, _, _ in group:
                    # Rebuild the cached suffix against the current joint
                    # starts, but keep an explicit recovery detour.  Clearing
                    # that detour here sent a corridor entrant straight back
                    # into the owner it was supposed to release.
                    # A traffic-clearance task is different: its route was
                    # selected specifically to leave the blocked suffix and
                    # end in a safe unused pocket. Preserve that authoritative
                    # path instead of letting congestion A* pick the waiter's
                    # corridor again on a cyclic graph.
                    release_owner = str(
                        release_order.vehicle
                        or release_order.assigned_robot
                        or ""
                    )
                    if (
                        release_owner in stationary_release_names
                        and release_owner not in predeparture_protected_names
                        and release_order.internal_kind != "traffic_clearance"
                    ):
                        release_order.spatial_route_nodes = []
            if async_simulated:
                if self._async_simulated_dispatch_active():
                    break
                job_started = self._start_async_simulated_dispatch(group)
                group_dispatched = 0
            else:
                group_dispatched, _ = self._dispatch_simulated_order_batch(group)
                dispatched += group_dispatched
            group_handled = {order.order_id for order, _, _, _ in group}
            handled.update(group_handled)
            planning_calls += 1
            if async_simulated and job_started:
                break
        cycle.dispatched = dispatched
        cycle.handled = handled
        cycle.ready = ready
        cycle.planning_budget = planning_budget
        return planning_calls

    def _dispatch_remaining_orders(
        self,
        cycle: _DispatchCycle,
        planning_calls: int,
    ) -> None:
        force = cycle.force
        async_simulated = cycle.async_simulated
        dispatched = cycle.dispatched
        handled = cycle.handled
        queued_orders = cycle.queued_orders
        planning_budget = cycle.planning_budget

        # Keep support for remote, automatic and already-at-goal orders.  A
        # failed coupled group is deliberately not retried individually in the
        # same tick: doing so recreated both CPU starvation and wait-only plans.
        remaining_budget = max(0, planning_budget - planning_calls)
        for order in queued_orders:
            if order.order_id in handled:
                continue
            robot = self.robots.get(order.vehicle) if order.vehicle else None
            if async_simulated and robot is not None and not robot.is_remote():
                # Simulated MAPF is deliberately background-only on runtime
                # ticks.  Otherwise one leftover order here would put the
                # expensive planner back on the websocket/status path.
                continue
            if (
                robot is not None
                and not robot.is_remote()
                and self._robot_can_accept_order(robot, explicit=True)
                and self._safe_replan_start_lm(robot) != order.target_lm
            ):
                if remaining_budget <= 0:
                    continue
                remaining_budget -= 1
            if self._dispatch_order(order, force=force):
                dispatched += 1
        cycle.dispatched = dispatched

    def _order_is_robot_queue_head(self, order: FleetOrder) -> bool:
        """Keep generated lifelong orders FIFO for each robot.

        A later benchmark order is generated from the previous order's final
        LM. Dispatching it first (for example because it has higher priority)
        can make that otherwise distant goal only one edge from the robot's
        current pose. Priority still orders traffic between different robots.
        """
        if not order.order_id.startswith("dynamic-") or not order.vehicle:
            return True
        pending = [
            candidate
            for candidate in self.orders.values()
            if candidate.order_id.startswith("dynamic-")
            and candidate.vehicle == order.vehicle
            and candidate.status not in TERMINAL_ORDER_STATUSES
        ]
        if not pending:
            return True
        pending.sort(key=lambda candidate: (candidate.created_at, candidate.order_id))
        return pending[0] is order

    def _queued_simulated_order_dispatch_ready(
        self,
        order: FleetOrder,
        now: float,
        *,
        live_wait_chain_sinks: set[str] | None = None,
    ) -> bool:
        """Mirror the real queued-order admission checks for planner fairness."""
        owner_name = str(order.vehicle or order.assigned_robot or "")
        if (
            order.status != "QUEUED"
            or not owner_name
            or not self._order_is_robot_queue_head(order)
        ):
            return False
        sinks = (
            self._live_stationary_wait_chain_sink_names()
            if live_wait_chain_sinks is None
            else live_wait_chain_sinks
        )
        if (
            owner_name not in sinks
            and not self._stationary_order_retry_ready(order)
        ):
            return False
        if (
            order.error
            and now - order.updated_at
            < self._order_dispatch_retry_interval(order)
        ):
            return False
        if not self._dispatch_conflict_dependency_ready(order):
            return False
        robot = self.robots.get(owner_name)
        return bool(
            robot is not None
            and not robot.is_remote()
            and self._robot_can_accept_order(robot, explicit=True)
        )

    def _queued_simulated_dispatch_waiting(self, now: float) -> bool:
        """Return whether a fleet robot has an actually eligible queued route."""
        live_wait_chain_sinks = self._live_stationary_wait_chain_sink_names()
        return any(
            self._queued_simulated_order_dispatch_ready(
                order,
                now,
                live_wait_chain_sinks=live_wait_chain_sinks,
            )
            for order in self.orders.values()
        )

    def _async_dispatch_entry_is_current(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str],
    ) -> bool:
        order, robot, request, _ = entry
        if self.orders.get(order.order_id) is not order or order.status != "PLANNING":
            return False
        if self.robots.get(robot.name) is not robot or robot.active_order_id:
            order.status = "QUEUED"
            return False
        start_lm = self._safe_replan_start_lm(robot)
        if start_lm != str(request.get("startLm") or ""):
            order.status = "QUEUED"
            order.error = "robot moved while background plan was running"
            order.updated_at = self._now()
            return False
        return True

    def _order_dispatch_retry_interval(self, order: FleetOrder | None = None) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            base = max(0.1, float(fleet.get("order_dispatch_retry_sec", 0.5) or 0.5))
        except (TypeError, ValueError):
            base = 0.5
        try:
            maximum = max(base, float(fleet.get("order_dispatch_retry_max_sec", 4.0) or 4.0))
        except (TypeError, ValueError):
            maximum = 4.0
        failures = max(0, int(order.dispatch_failures if order is not None else 0))
        return min(maximum, base * (2 ** min(4, max(0, failures - 1))))

    def _dispatch_order(self, order: FleetOrder, force: bool = False) -> bool:
        if not self._order_is_robot_queue_head(order):
            return False
        order.target_lm = self._active_order_target(order)
        candidates = self._candidate_robots_for_order(order)
        if not candidates:
            self._set_order_error(order, "no available robot")
            return False

        failed_reason = ""
        stationary_failure_debug: dict[str, Any] | None = None
        for robot in candidates:
            stationary_failure_debug = None
            start_lm = self._safe_replan_start_lm(robot)
            if not start_lm or start_lm not in self.landmarks:
                failed_reason = "robot is between graph landmarks"
                continue
            if start_lm == order.target_lm and not robot.trajectory:
                return self._complete_order_at_current_target(
                    order,
                    robot,
                    force=force,
                )

            try:
                request = self._dispatch_request_for_robot(
                    order,
                    robot,
                    start_lm,
                )
            except ValueError as exc:
                if order.internal_kind != "traffic_clearance":
                    raise
                failed_reason = f"traffic clearance route invalid: {exc}"
                continue

            self._set_order_status(order, "PLANNING", robot=robot, start_lm=start_lm)
            result = self._plan_valid_requests([request], self._order_plan_payload(order, request))
            if result.get("ok") and result.get("plans"):
                prepared, failed_reason = self._prepare_dispatch_plan(
                    order,
                    robot,
                    start_lm,
                    request,
                    result,
                )
                if prepared is None:
                    continue
                self._commit_dispatch_plan(
                    order,
                    robot,
                    start_lm,
                    prepared,
                )
                return True
            failed_reason, stationary_failure_debug = (
                self._record_failed_dispatch_attempt(
                    order,
                    robot,
                    start_lm,
                    result,
                )
            )

        self._set_order_error(order, failed_reason or "dispatch pending")
        if stationary_failure_debug is not None:
            self._record_stationary_order_failure(
                order,
                stationary_failure_debug,
            )
        return False

    def _complete_order_at_current_target(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        *,
        force: bool,
    ) -> bool:
        robot.current_lm = order.target_lm
        robot.target_lm = ""
        robot.status = "ARRIVED"
        robot.active_order_id = ""
        robot.last_reason = "order already at target"
        robot.updated_at = self._now()
        if self._advance_or_complete_order(order, robot, self._now()):
            self._event(
                "info",
                f"order completed: {order.order_id} "
                f"{robot.name}@{order.target_lm}",
            )
            return True
        return self._dispatch_order(order, force=force)

    def _dispatch_request_for_robot(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        start_lm: str,
    ) -> dict[str, Any]:
        planning_goal = (
            order.target_lm
            if robot.is_remote()
            else self._rolling_planning_goal(
                start_lm,
                order.target_lm,
                order,
            )
        )
        request: dict[str, Any] = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": planning_goal,
        }
        if robot.pose is not None:
            request["startPose"] = dict(robot.pose)
        if not robot.is_remote() or order.internal_kind == "traffic_clearance":
            self._attach_spatial_route_to_request(
                request,
                order,
                start_lm,
                planning_goal,
                order.target_lm,
            )
        return request

    def _prepare_dispatch_plan(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        start_lm: str,
        request: dict[str, Any],
        result: dict[str, Any],
    ) -> tuple[_PreparedDispatchPlan | None, str]:
        committed_at = self._now()
        plan = self._plan_for_robot(result, robot.name)
        if (
            plan is not None
            and not self._plan_follows_requested_clearance_route(
                order,
                request,
                plan,
            )
        ):
            return (
                None,
                "traffic clearance planner changed its fixed route",
            )

        remote_route: dict[str, Any] | None = None
        if robot.is_remote():
            if plan is None:
                return None, "planner did not return robot plan"
            try:
                remote_route = self._execute_remote_plan(
                    robot,
                    order,
                    plan,
                    result,
                )
            except Exception as exc:
                reason = f"remote execute failed: {exc}"
                robot.remote_error = str(exc)
                control_conflict = self._is_remote_control_conflict(exc)
                robot.remote_online = control_conflict
                robot.status = "MANUAL" if control_conflict else "OFFLINE"
                robot.last_reason = reason
                robot.updated_at = committed_at
                return None, reason
        else:
            result = self._rolling_result(
                result,
                {robot.name: order.target_lm},
            )
            plan = self._plan_for_robot(result, robot.name)
            if plan is None:
                return None, "planner did not return rolling route chunk"
            if not self._plan_follows_requested_clearance_route(
                order,
                request,
                plan,
            ):
                return (
                    None,
                    "traffic clearance rolling result changed its fixed route",
                )
            if self._wait_only_rolling_plan(plan, order.target_lm):
                detour_queued = (
                    self._order_stall_allows_detour(order)
                    and self._queue_alternate_corridor_detour(
                        order,
                        start_lm,
                        order.target_lm,
                    )
                )
                reason = (
                    "traffic window has no progress; alternate corridor queued"
                    if detour_queued
                    else "traffic window has no progress; joint retry pending"
                )
                self._event(
                    "warn",
                    f"{robot.name} wait-only route rejected; order kept queued",
                )
                return None, reason

        return (
            _PreparedDispatchPlan(
                result=result,
                plan=plan,
                remote_route=remote_route,
                committed_at=committed_at,
            ),
            "",
        )

    def _commit_dispatch_plan(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        start_lm: str,
        prepared: _PreparedDispatchPlan,
    ) -> None:
        order.route_nodes = [
            str(item)
            for plan in prepared.result.get("plans", [])
            if isinstance(plan, dict)
            for item in plan.get("nodes", [])
        ]
        self._apply_planner_result(
            prepared.result,
            prepared.committed_at,
            order_id=order.order_id,
        )
        if prepared.remote_route is not None:
            self._apply_remote_route_metadata(
                robot,
                prepared.remote_route,
                prepared.committed_at,
            )
        else:
            self._apply_simulated_route_metadata(
                robot,
                order,
                prepared.plan,
                prepared.committed_at,
            )
        self._set_order_status(
            order,
            "EXECUTING",
            robot=robot,
            start_lm=start_lm,
        )
        order.traffic_detour_edges = []
        self._event(
            "info",
            f"order dispatched: {order.order_id} {robot.name} "
            f"{start_lm}->{order.target_lm}",
        )

    def _record_failed_dispatch_attempt(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        start_lm: str,
        result: dict[str, Any],
    ) -> tuple[str, dict[str, Any] | None]:
        reason = self._planner_failure_reason(result)
        debug = result.get("debug")
        if not isinstance(debug, dict):
            debug = {}
        stationary_failure_debug = (
            debug
            if (
                debug.get("stationaryRobotWait")
                or "stationary_robot_blocks_route" in reason
            )
            else None
        )
        if self._planner_deadlock_result(result):
            robot.status = "WAITING"
            robot.last_reason = reason
            robot.blocked_since = self._now()
            robot.updated_at = self._now()
        order.status = "QUEUED"
        if order.vehicle:
            order.assigned_robot = robot.name
        order.start_lm = start_lm
        order.updated_at = self._now()
        return reason, stationary_failure_debug

    def _candidate_robots_for_order(self, order: FleetOrder) -> list[FleetRobot]:
        return self._order_admission_service.candidate_robots(order)

    def _robot_can_accept_order(self, robot: FleetRobot, explicit: bool = False) -> bool:
        return self._order_admission_service.can_accept(
            robot,
            explicit=explicit,
        )

    def _lm_distance(self, start_lm: str, goal_lm: str) -> float:
        return self._order_admission_service.landmark_distance(
            start_lm,
            goal_lm,
        )
