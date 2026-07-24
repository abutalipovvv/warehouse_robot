"""Order lifecycle, dispatch and rolling task execution."""

from __future__ import annotations

from heapq import heappop, heappush
import math
from threading import Thread
from typing import Any

from fleet_manager.core.constants import (
    FLEET_CONTROL_OWNER_ID,
    ORDER_SEQUENCE_KEYS,
    ORDER_TARGET_KEYS,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.models import FleetOrder, FleetRobot
from fleet_manager.core.traffic.corridor_scheduler import (
    CorridorRequest,
    CorridorSlot,
)


class FleetTaskDispatchMixin:
    """Shared task policy for simulation and gRPC fleet runtimes."""

    def _orders_list(self) -> list[dict[str, Any]]:
        return self.task_manager.ordered_payloads(
            enabled=self._order_enabled,
            limit=120,
        )

    def _build_orders(self, payload: dict[str, Any]) -> list[FleetOrder]:
        return [self._build_order(payload)]

    def _build_order(self, payload: dict[str, Any]) -> FleetOrder:
        order_id = str(
            payload.get("id")
            or payload.get("orderId")
            or payload.get("taskId")
            or payload.get("externalId")
            or ""
        ).strip()
        if not order_id:
            order_id = f"order-{int(self._now() * 1000)}"

        targets = self._target_lms_from_payload(payload)
        if not targets:
            raise ValueError("targetLm/goalLm/location is required")
        for target_lm in targets:
            if target_lm not in self.landmarks:
                raise ValueError(f"unknown target LM: {target_lm}")

        vehicle = str(
            payload.get("vehicle")
            or payload.get("robot")
            or payload.get("robotName")
            or payload.get("name")
            or ""
        ).strip()
        if vehicle and vehicle not in self.robots:
            raise ValueError(f"unknown robot: {vehicle}")

        try:
            priority = int(payload.get("priority", 0) or 0)
        except (TypeError, ValueError):
            priority = 0
        external_id = str(payload.get("externalId") or payload.get("taskId") or "").strip()
        speed = self._float_payload(payload, ("speed", "routeSpeed"), 0.0)
        acceleration = self._float_payload(payload, ("acceleration", "routeAcceleration", "route_acceleration"), 0.0)
        rotate = self._bool_payload(payload, ("rotate", "simulateRotation", "simulate_rotation"), False)
        turn_speed = self._float_payload(payload, ("turnSpeed", "turn_speed", "rotationSpeed", "rotation_speed"), 0.0)
        stretch_motion = self._bool_payload(
            payload,
            ("stretchMotionToReservationTicks", "stretch_motion_to_reservation_ticks"),
            True,
        )
        now = self._now()
        return FleetOrder(
            order_id=order_id,
            target_lm=targets[0],
            vehicle=vehicle,
            priority=priority,
            external_id=external_id,
            targets=targets,
            speed=speed,
            acceleration=acceleration,
            rotate=rotate,
            turn_speed=turn_speed,
            stretch_motion_to_reservation_ticks=stretch_motion,
            created_at=now,
            updated_at=now,
        )

    def _float_payload(self, payload: dict[str, Any], keys: tuple[str, ...], default: float = 0.0) -> float:
        for key in keys:
            if key not in payload:
                continue
            try:
                return float(payload.get(key) or default)
            except (TypeError, ValueError):
                return default
        return default

    def _bool_payload(self, payload: dict[str, Any], keys: tuple[str, ...], default: bool = False) -> bool:
        for key in keys:
            if key not in payload:
                continue
            value = payload.get(key)
            if isinstance(value, str):
                return value.strip().lower() in {"1", "true", "yes", "on"}
            return bool(value)
        return default

    def _target_lms_from_payload(self, payload: dict[str, Any]) -> list[str]:
        targets: list[str] = []
        for key in ORDER_SEQUENCE_KEYS:
            raw_sequence = payload.get(key)
            if not isinstance(raw_sequence, list):
                continue
            for item in raw_sequence:
                target_lm = self._target_lm_from_payload_item(item)
                if target_lm:
                    targets.append(target_lm)
            if targets:
                return targets

        target_lm = self._target_lm_from_payload_item(payload)
        return [target_lm] if target_lm else []

    def _target_lm_from_payload_item(self, item: Any) -> str:
        if isinstance(item, dict):
            for key in ORDER_TARGET_KEYS:
                target_lm = str(item.get(key) or "").strip()
                if target_lm:
                    return target_lm
            return ""
        return str(item or "").strip()

    def _dispatch_orders(
        self,
        force: bool = False,
        *,
        async_simulated: bool = False,
    ) -> int:
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
        return dispatched

    def _stationary_release_robot_names(self) -> set[str]:
        """Find queued stationary robots that currently block active traffic."""
        release = self._live_stationary_wait_chain_sink_names()
        # A coupled planner failure identifies the member whose route failed
        # validation. Do not let that one member keep poisoning every later
        # batch: release it first, then allow the unaffected queue heads to be
        # planned without it. This set affects ordering/grouping only; unlike
        # a live wait-chain sink it must still pass retry quarantine.
        for order in self.orders.values():
            if order.internal_kind or order.status != "QUEUED" or not order.error:
                continue
            blocker_name = self._planner_conflict_robot_name(order.error)
            blocker = self.robots.get(blocker_name)
            if (
                blocker is None
                or blocker.trajectory
                or blocker.status not in {"IDLE", "ARRIVED", "BLOCKED"}
            ):
                continue
            pending = self._active_order_for_robot(blocker)
            if pending is not None and pending.status in {"QUEUED", "PLANNING"}:
                release.add(blocker.name)
        # Quarantine is a CPU backoff, not a terminal scheduling state. Once
        # its bounded cooldown expires, make a commanded stationary component
        # a coordinated release group. `_stationary_order_retry_ready` remains
        # the authority that admits it into the actual dispatch list.
        for order in self.orders.values():
            if (
                order.internal_kind
                or order.status != "QUEUED"
                or order.order_id not in self._stationary_order_retry_state
                or not self._stationary_order_retry_ready(order)
            ):
                continue
            owner_name = str(order.vehicle or order.assigned_robot or "")
            owner = self.robots.get(owner_name)
            if (
                owner is not None
                and not owner.trajectory
                and owner.status in {"IDLE", "ARRIVED", "BLOCKED"}
            ):
                release.add(owner.name)
        return release

    def _live_stationary_wait_chain_sink_names(self) -> set[str]:
        """Return commanded parked bodies directly holding live traffic.

        These are the only stationary retry owners allowed to bypass an
        unchanged quarantine immediately. A robot merely named by its own old
        planner error has no evidence that another body is waiting for it and
        must retain the normal bounded retry cadence.
        """
        release: set[str] = set()
        for waiter in self._runtime_robots():
            if waiter.status != "WAITING" or not waiter.trajectory:
                continue
            waiter_order = self._active_order_for_robot(waiter)
            if (
                waiter_order is not None
                and waiter_order.internal_kind
            ):
                continue
            blocker_name = (
                waiter.wait_for_robot
                or self._robot_name_from_conflict_reason(waiter.last_reason)
            )
            blocker = self.robots.get(blocker_name)
            if (
                blocker is None
                or blocker.trajectory
                or blocker.status not in {"IDLE", "ARRIVED", "BLOCKED"}
            ):
                continue
            pending = self._active_order_for_robot(blocker)
            if pending is not None and pending.status in {"QUEUED", "PLANNING"}:
                release.add(blocker.name)
        return release

    def _live_waiters_for_stationary_sink(
        self,
        sink: FleetRobot,
    ) -> list[FleetRobot]:
        """Return active robots physically holding for ``sink`` right now."""
        waiters: list[FleetRobot] = []
        for robot in self._runtime_robots():
            if (
                robot.name == sink.name
                or robot.status != "WAITING"
                or not robot.trajectory
                or not robot.active_order_id
            ):
                continue
            blocker_name = (
                robot.wait_for_robot
                or self._robot_name_from_conflict_reason(robot.last_reason)
            )
            if blocker_name != sink.name:
                continue
            order = self._active_order_for_robot(robot)
            if (
                order is None
                or order.internal_kind
                or order.status in TERMINAL_ORDER_STATUSES
            ):
                continue
            waiters.append(robot)
        return sorted(waiters, key=lambda robot: robot.name)

    def _queue_commanded_sink_vacancy_replan(self, now: float) -> bool:
        """Open a safe pocket when a queued departure is boxed by its waiters.

        The queued robot remains a physical obstacle.  One immediate upstream
        waiter receives a short, fixed graph route *away* from that body through
        the ordinary transactional runtime-replan path.  Its original order
        goal is retained and resumes after the pocket chunk completes.
        """
        if self._async_simulated_dispatch_active():
            return False

        candidates: list[
            tuple[
                tuple[float, int, str, str],
                FleetRobot,
                FleetOrder,
                FleetRobot,
                list[str],
                tuple[tuple[str, str, int], ...],
            ]
        ] = []
        live_episode_sinks: set[str] = {
            str(state.get("queued_departure_sink") or "")
            for state in self._runtime_replans.values()
            if isinstance(state, dict)
            and str(state.get("queued_departure_sink") or "") in self.robots
        }
        for sink_order in self.orders.values():
            if (
                sink_order.internal_kind
                or sink_order.status != "QUEUED"
                or int(sink_order.dispatch_failures or 0) < 2
            ):
                continue
            sink_name = str(
                sink_order.vehicle or sink_order.assigned_robot or ""
            )
            sink = self.robots.get(sink_name)
            if (
                sink is None
                or sink.is_remote()
                or sink.trajectory
                or sink.status not in {"IDLE", "ARRIVED", "BLOCKED"}
                or not self._robot_departure_pending(sink)
            ):
                continue
            waiters = self._live_waiters_for_stationary_sink(sink)
            if not waiters:
                continue
            live_episode_sinks.add(sink.name)
            sink_lm = self._traffic_lm_for_robot(sink)
            if sink_lm not in self.landmarks:
                continue
            signature = tuple(sorted(
                (
                    robot.name,
                    self._traffic_lm_for_robot(robot),
                    int(robot.route_revision),
                )
                for robot in [sink, *waiters]
            ))
            if (
                signature
                != self._commanded_sink_vacancy_signatures.get(sink.name)
            ):
                self._commanded_sink_vacancy_signatures[sink.name] = signature
                self._commanded_sink_vacancy_blacklist = {
                    item
                    for item in self._commanded_sink_vacancy_blacklist
                    if item[0] != sink.name
                }

            route_nodes = [
                str(node)
                for node in sink_order.spatial_route_nodes
                if str(node) in self.landmarks
            ]
            next_lm = ""
            if sink_lm in route_nodes:
                suffix = route_nodes[route_nodes.index(sink_lm):]
                if len(suffix) > 1:
                    next_lm = suffix[1]
            next_landmark = self.landmarks.get(next_lm)
            sink_edges = self._blocked_edges_for_lms({sink_lm})
            for waiter in waiters:
                if waiter.name in self._runtime_replans:
                    continue
                waiter_order = self._active_order_for_robot(waiter)
                start_lm = self._safe_replan_start_lm(waiter)
                if (
                    waiter_order is None
                    or start_lm not in self.landmarks
                    or waiter.is_remote()
                ):
                    continue
                forbidden = {
                    pocket
                    for known_sink, known_signature, owner, pocket
                    in self._commanded_sink_vacancy_blacklist
                    if known_sink == sink.name
                    and known_signature == signature
                    and owner == waiter.name
                }
                # A graph node can be distinct from the queued sink while its
                # approach still enters that robot's physical footprint.  In
                # the live Kiva case the selected "escape" began by moving an
                # upstream waiter one LM *towards* the parked departure. SIPP
                # could commit the graph route, but runtime collision safety
                # stopped it before the first sample; the same transaction was
                # then rebuilt at 10 Hz. Block every known-bad first edge and
                # audit the complete route against the causal sink body.
                blocked_escape_edges = set(sink_edges)
                for neighbour in self.planner.graph.get(start_lm, []):
                    neighbour = str(neighbour)
                    if neighbour not in self.landmarks:
                        continue
                    if self._graph_escape_route_current_body_blocker(
                        waiter,
                        [start_lm, neighbour],
                        only_robot_names={sink.name},
                    ):
                        blocked_escape_edges.add((start_lm, neighbour))

                escape: list[str] = []
                # A full-route audit may reject an intermediate sweep even
                # when its first edge is safe. Try a bounded number of other
                # pockets in this scheduler turn and persist rejected goals
                # for the unchanged physical episode.
                for _ in range(4):
                    candidate = self._stationary_clearance_route(
                        sink,
                        waiter,
                        forbidden_lms=forbidden,
                        extra_blocked_edges=blocked_escape_edges,
                        start_lm_override=start_lm,
                    )
                    if len(candidate) < 2:
                        break
                    body_blocker = self._graph_escape_route_current_body_blocker(
                        waiter,
                        candidate,
                        only_robot_names={sink.name},
                    )
                    if not body_blocker:
                        escape = candidate
                        break
                    rejected_pocket = str(candidate[-1])
                    self._commanded_sink_vacancy_blacklist.add(
                        (
                            sink.name,
                            signature,
                            waiter.name,
                            rejected_pocket,
                        )
                    )
                    if rejected_pocket in forbidden:
                        break
                    forbidden.add(rejected_pocket)
                if len(escape) < 2:
                    continue
                waiter_landmark = self.landmarks.get(start_lm)
                exit_distance = (
                    math.hypot(
                        float(waiter_landmark.x) - float(next_landmark.x),
                        float(waiter_landmark.y) - float(next_landmark.y),
                    )
                    if waiter_landmark is not None and next_landmark is not None
                    else float("inf")
                )
                candidates.append((
                    (
                        exit_distance,
                        len(escape),
                        waiter.name,
                        str(escape[-1]),
                    ),
                    sink,
                    sink_order,
                    waiter,
                    escape,
                    signature,
                ))

        stale_episode_sinks = (
            set(self._commanded_sink_vacancy_signatures)
            - live_episode_sinks
        )
        for sink_name in stale_episode_sinks:
            self._commanded_sink_vacancy_signatures.pop(sink_name, None)
        if stale_episode_sinks:
            self._commanded_sink_vacancy_blacklist = {
                item
                for item in self._commanded_sink_vacancy_blacklist
                if item[0] not in stale_episode_sinks
            }

        if not candidates:
            return False
        _, sink, _, waiter, escape, signature = min(
            candidates,
            key=lambda item: item[0],
        )
        order = self._active_order_for_robot(waiter)
        if order is None:
            return False
        start_lm = str(escape[0])
        existing = self._runtime_replans.get(waiter.name)
        generation = (
            int(existing.get("generation", 0) or 0) + 1
            if isinstance(existing, dict)
            else 1
        )
        reason = f"vacancy release for queued departure {sink.name}"
        self._runtime_replans[waiter.name] = {
            "order_id": order.order_id,
            "start_lm": start_lm,
            "route_revision": int(waiter.route_revision),
            "route_clock": float(waiter.route_clock),
            "reason": reason,
            "blocker_names": (sink.name,),
            "queued_at": float(now),
            "retry_at": float(now),
            "failures": 0,
            "generation": generation,
            "stage": "queued",
            "escape_route_nodes": list(escape),
            "escape_goal": str(escape[-1]),
            "escape_blocked_lms": (self._traffic_lm_for_robot(sink),),
            "queued_departure_sink": sink.name,
            "queued_departure_signature": signature,
        }
        order.status = "PLANNING"
        order.error = f"runtime replan pending: {reason}"
        order.updated_at = now
        waiter.status = "WAITING"
        waiter.last_reason = f"replanning route while holding: {reason}"
        waiter.last_replan_at = now
        waiter.last_tick_at = now
        waiter.traffic_priority_until = 0.0
        waiter.updated_at = now
        self._clear_wait_dependency(waiter)
        self._event(
            "warn",
            f"{waiter.name} opening vacancy for queued departure {sink.name}: "
            f"{'->'.join(escape)}",
        )
        return True

    def _stationary_retry_failure_limit(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = int(
                fleet.get("stationary_retry_quarantine_failures", 2) or 2
            )
        except (TypeError, ValueError):
            configured = 2
        return max(2, min(8, configured))

    def _stationary_blocker_signature(
        self,
        blocked_lms: tuple[str, ...],
    ) -> tuple[tuple[Any, ...], ...]:
        blocked = set(blocked_lms)
        signature: list[tuple[Any, ...]] = []
        for robot in self._runtime_robots():
            lm_name = self._nearest_lm_for_robot(robot)
            if lm_name not in blocked:
                continue
            pending = self._active_order_for_robot(robot)
            signature.append(
                (
                    robot.name,
                    lm_name,
                    robot.status,
                    int(robot.route_revision),
                    bool(robot.trajectory),
                    str(pending.order_id if pending is not None else ""),
                    str(pending.status if pending is not None else ""),
                )
            )
        return tuple(sorted(signature))

    def _record_stationary_order_failure(
        self,
        order: FleetOrder,
        debug: dict[str, Any],
    ) -> None:
        if order.internal_kind == "traffic_clearance":
            # A hidden maintenance move is already the recovery action for a
            # normal fleet order.  Let ordinary traffic arbitration move it,
            # but never let its own planning failure relocate another parked
            # robot: that creates unbounded clearance chains and, in a narrow
            # corridor, two maintenance orders that wait on each other.
            return
        owner = str(order.vehicle or order.assigned_robot or "")
        raw_blockers = debug.get("stationaryBlockerRobots", [])
        explicit_blocker_names = {
            str(name)
            for name in (raw_blockers if isinstance(raw_blockers, list) else [])
            if str(name) in self.robots and str(name) != owner
        }
        unresolved_name = str(debug.get("continuousConflictRobot") or "")
        if unresolved_name in self.robots and unresolved_name != owner:
            explicit_blocker_names.add(unresolved_name)
        waiter = self.robots.get(owner)
        if waiter is not None:
            runtime_blocker = (
                waiter.wait_for_robot
                or self._robot_name_from_conflict_reason(waiter.last_reason)
            )
            if runtime_blocker in self.robots and runtime_blocker != owner:
                explicit_blocker_names.add(runtime_blocker)
        # ``softBlockedLms`` is a global set of stationary occupancy used by
        # congestion A*. It is not proof that every robot on one of those LMs
        # caused this request to fail. Falling back from missing identity to
        # the complete signature moved arbitrary idle robots on the other side
        # of the map and, after cooldown, moved them straight back again.
        blocker_names = tuple(sorted(explicit_blocker_names))
        if blocker_names:
            # A proven blocker identity is the stable debounce boundary.
            # ``softBlockedLms`` contains every parked robot in the warehouse;
            # unrelated task completions used to reset this count forever.
            blocked_lms = tuple(sorted({
                lm_name
                for blocker_name in blocker_names
                for lm_name in [
                    self._traffic_lm_for_robot(self.robots[blocker_name])
                ]
                if lm_name in self.landmarks
            }))
        else:
            raw_lms = debug.get("softBlockedLms", [])
            blocked_lms = tuple(sorted({
                str(lm_name)
                for lm_name in (
                    raw_lms if isinstance(raw_lms, list) else []
                )
                if str(lm_name) in self.landmarks
            }))
            if not blocked_lms:
                blocked_lms = tuple(sorted(
                    self._stationary_robot_blocked_lms(
                        exclude_robot_names={owner} if owner else set(),
                    )
                ))
        signature = self._stationary_retry_blocker_signature(
            blocked_lms,
            blocker_names,
        )
        previous = self._stationary_order_retry_state.get(order.order_id, {})
        same_failure = bool(
            previous.get("blocked_lms") == blocked_lms
            and previous.get("signature") == signature
            and previous.get("blocker_names") == blocker_names
        )
        failure_count = (
            int(previous.get("failure_count", 0) or 0) + 1
            if same_failure
            else 1
        )
        retry_state: dict[str, Any] = {
            "blocked_lms": blocked_lms,
            "blocker_names": blocker_names,
            "signature": signature,
            "failure_count": failure_count,
        }
        if same_failure:
            # A graph escape intentionally replaces the runtime-replan
            # transaction (and therefore its short-lived state).  Keep the
            # bounded recovery episode with the user order so the same
            # unchanged pair cannot oscillate between already visited
            # holding pockets.
            for key in (
                "cut_search_signature",
                "cut_candidate_names",
                "waiter_escape_attempts",
                "waiter_escape_lms",
                "waiter_escape_in_flight",
                "waiter_escape_target_lm",
            ):
                if key in previous:
                    retry_state[key] = previous[key]
        self._stationary_order_retry_state[order.order_id] = retry_state
        if failure_count < self._stationary_clearance_failure_limit():
            return
        if waiter is None:
            return
        for blocker_name in blocker_names:
            blocker = self.robots.get(blocker_name)
            if blocker is not None and self._queue_stationary_clearance_relocation(
                waiter,
                blocker,
                cause=f"initial route blocked for {order.order_id}",
            ):
                break

    def _stationary_waiter_escape_attempt_limit(self) -> int:
        """Bound active-waiter pocket changes for one unchanged graph cut."""
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            value = int(
                fleet.get("parked_clearance_waiter_escape_attempts", 3) or 3
            )
        except (TypeError, ValueError):
            value = 3
        return max(1, min(8, value))

    def _inactive_stationary_clearance_candidate(
        self,
        robot: FleetRobot | None,
        *,
        exclude_name: str,
    ) -> bool:
        """Return whether core policy may move this exact parked body."""
        if (
            robot is None
            or robot.name == exclude_name
            or robot.status not in {"IDLE", "ARRIVED"}
            or robot.trajectory
            or robot.active_order_id
            or robot.target_lm
            or not robot.remote_online
        ):
            return False
        pending = self._active_order_for_robot(robot)
        if pending is not None and pending.status not in TERMINAL_ORDER_STATUSES:
            return False
        if robot.is_remote():
            owner_id, _ = self._remote_control_owner(robot)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return False
        return self._traffic_lm_for_robot(robot) in self.landmarks

    def _stationary_cut_search_signature(
        self,
        order: FleetOrder,
        waiter: FleetRobot,
        start_lm: str,
        goal_lm: str,
        candidates: list[FleetRobot],
    ) -> tuple[Any, ...]:
        return (
            order.order_id,
            waiter.name,
            start_lm,
            goal_lm,
            tuple(
                (
                    robot.name,
                    self._traffic_lm_for_robot(robot),
                    robot.status,
                    int(robot.route_revision),
                    str(robot.active_order_id or ""),
                    str(robot.target_lm or ""),
                )
                for robot in candidates
            ),
        )

    def _queue_alternative_stationary_cut_clearance(
        self,
        order: FleetOrder,
        waiter: FleetRobot,
        *,
        cause: str,
    ) -> str:
        """Relocate one proven stationary vertex cut on an alternate route.

        Planner diagnostics normally identify the body on the shortest
        rejected suffix.  A second parked robot may close the free bypass,
        however, and be much easier to move.  Releasing one candidate LM at a
        time proves that the candidate itself restores start->goal
        connectivity; the normal clearance selector then proves that its
        physical move is safe and genuinely releases the waiter.
        """
        retry_state = self._stationary_order_retry_state.get(order.order_id)
        if (
            not isinstance(retry_state, dict)
            or int(retry_state.get("failure_count", 0) or 0)
            < self._stationary_clearance_failure_limit()
        ):
            return ""
        start_lm = self._safe_replan_start_lm(waiter)
        goal_lm = self._active_order_target(order)
        if (
            start_lm not in self.landmarks
            or goal_lm not in self.landmarks
            or start_lm == goal_lm
        ):
            return ""

        candidates = sorted(
            (
                robot
                for robot in self._runtime_robots()
                if self._inactive_stationary_clearance_candidate(
                    robot,
                    exclude_name=waiter.name,
                )
            ),
            key=lambda robot: (
                self._lm_distance(
                    start_lm,
                    self._traffic_lm_for_robot(robot),
                )
                + self._lm_distance(
                    self._traffic_lm_for_robot(robot),
                    goal_lm,
                ),
                robot.name,
            ),
        )
        if not candidates:
            return ""

        # The proof is deliberately bounded for very large fleets. Nearest
        # cut bodies are checked first, while a changed fleet signature makes
        # the next episode eligible for a fresh scan.
        candidates = candidates[:64]
        search_signature = self._stationary_cut_search_signature(
            order,
            waiter,
            start_lm,
            goal_lm,
            candidates,
        )
        cached_signature = retry_state.get("cut_search_signature")
        if cached_signature == search_signature:
            candidate_names = tuple(
                str(name)
                for name in retry_state.get("cut_candidate_names", ())
                if str(name) in self.robots
            )
        else:
            stationary_lms = self._stationary_robot_blocked_lms(
                exclude_robot_names={waiter.name},
            )
            names_by_lm: dict[str, list[str]] = {}
            for candidate in candidates:
                candidate_lm = self._traffic_lm_for_robot(candidate)
                names_by_lm.setdefault(candidate_lm, []).append(candidate.name)
            dynamic_edges = self._dynamic_blocked_edges()
            proven: list[tuple[float, str]] = []
            for candidate in candidates:
                candidate_lm = self._traffic_lm_for_robot(candidate)
                # Removing one of multiple bodies on the same LM does not
                # release that graph resource.
                if len(names_by_lm.get(candidate_lm, ())) != 1:
                    continue
                blocked_lms = set(stationary_lms)
                blocked_lms.discard(candidate_lm)
                try:
                    route = self.planner.route_planner.find_route(
                        start_lm,
                        goal_lm,
                        blocked_edges=(
                            dynamic_edges
                            | self._blocked_edges_for_lms(blocked_lms)
                        ),
                    )
                except ValueError:
                    continue
                if candidate_lm not in route.nodes:
                    continue
                proven.append((float(route.length), candidate.name))
            proven.sort(key=lambda item: (item[0], item[1]))
            candidate_names = tuple(name for _, name in proven)
            retry_state["cut_search_signature"] = search_signature
            retry_state["cut_candidate_names"] = candidate_names

        for candidate_name in candidate_names:
            blocker = self.robots.get(candidate_name)
            if not self._inactive_stationary_clearance_candidate(
                blocker,
                exclude_name=waiter.name,
            ):
                continue
            replan_state = self._runtime_replans.get(waiter.name)
            previous_blocker_names = (
                tuple(replan_state.get("blocker_names", ()))
                if isinstance(replan_state, dict)
                else ()
            )
            if isinstance(replan_state, dict):
                # The graph-cut proof is also the missing causal identity
                # after a runtime restore. Mark the candidate during physical
                # route validation so a maintenance path through the held
                # waiter's body is rejected just as strictly as a
                # planner-reported blocker.
                replan_state["blocker_names"] = tuple(sorted({
                    *(
                        str(name)
                        for name in previous_blocker_names
                        if str(name)
                    ),
                    candidate_name,
                }))
            queued = self._queue_stationary_clearance_relocation(
                waiter,
                blocker,
                cause=cause,
            )
            if not queued and isinstance(replan_state, dict):
                if previous_blocker_names:
                    replan_state["blocker_names"] = previous_blocker_names
                else:
                    replan_state.pop("blocker_names", None)
            if queued:
                self._event(
                    "warn",
                    f"{waiter.name} alternate stationary cut released by "
                    f"{candidate_name}",
                )
                return candidate_name
        return ""

    def _stage_stationary_waiter_escape(
        self,
        order: FleetOrder,
        waiter: FleetRobot,
        replan_state: dict[str, Any],
        blocker_names: set[str],
        now: float,
    ) -> bool:
        """Move the held active robot once when it blocks blocker evacuation."""
        retry_state = self._stationary_order_retry_state.get(order.order_id)
        if (
            waiter.is_remote()
            or order.internal_kind
            or not bool(replan_state.get("retained_route_superseded"))
            or not isinstance(retry_state, dict)
            or int(retry_state.get("failure_count", 0) or 0)
            < self._stationary_clearance_failure_limit()
        ):
            return False
        attempts = int(retry_state.get("waiter_escape_attempts", 0) or 0)
        if attempts >= self._stationary_waiter_escape_attempt_limit():
            return False

        exact_blockers: list[FleetRobot] = []
        for blocker_name in sorted(blocker_names):
            blocker = self.robots.get(blocker_name)
            if self._inactive_stationary_clearance_candidate(
                blocker,
                exclude_name=waiter.name,
            ):
                exact_blockers.append(blocker)
        if not exact_blockers:
            return False

        start_lm = self._safe_replan_start_lm(waiter)
        if start_lm not in self.landmarks:
            return False
        forbidden_lms = {
            str(lm_name)
            for lm_name in retry_state.get("waiter_escape_lms", ())
            if str(lm_name) in self.landmarks
        }
        forbidden_lms.update(
            str(target)
            for target in (order.targets or [order.target_lm])
            if str(target) in self.landmarks
        )
        blocker_lms = {
            self._traffic_lm_for_robot(blocker)
            for blocker in exact_blockers
        }
        selector_blocker = exact_blockers[0]
        escape_route = self._stationary_clearance_route(
            selector_blocker,
            waiter,
            forbidden_lms=forbidden_lms,
            extra_blocked_edges=self._blocked_edges_for_lms(blocker_lms),
            avoid_controlled_regions=True,
            start_lm_override=start_lm,
        )
        if len(escape_route) < 2:
            escape_route = self._stationary_clearance_route(
                selector_blocker,
                waiter,
                forbidden_lms=forbidden_lms,
                extra_blocked_edges=self._blocked_edges_for_lms(blocker_lms),
                avoid_controlled_regions=False,
                require_unowned_controlled_regions=True,
                start_lm_override=start_lm,
            )
        if len(escape_route) < 2:
            return False

        escape_edges = set(zip(escape_route, escape_route[1:]))
        retained_blocked_edges = [
            (str(source), str(target))
            for source, target in order.traffic_detour_edges
            if (str(source), str(target)) not in escape_edges
        ]
        if not self._install_graph_escape_retreat(
            waiter,
            escape_route,
            retained_blocked_edges,
            now,
        ):
            return False

        signatures = tuple(
            (
                blocker.name,
                self._traffic_lm_for_robot(blocker),
                int(blocker.route_revision),
            )
            for blocker in exact_blockers
        )
        waiter.retreat_blocker_signatures = list(signatures)
        visited_lms = list(retry_state.get("waiter_escape_lms", ()))
        if escape_route[-1] not in visited_lms:
            visited_lms.append(str(escape_route[-1]))
        retry_state["waiter_escape_attempts"] = attempts + 1
        retry_state["waiter_escape_lms"] = tuple(visited_lms)
        retry_state["waiter_escape_in_flight"] = signatures
        retry_state["waiter_escape_target_lm"] = str(escape_route[-1])
        self._event(
            "warn",
            f"{waiter.name} staged traffic release to {escape_route[-1]} "
            f"before clearing {', '.join(item[0] for item in signatures)}",
        )
        return True

    def _stationary_retry_blocker_signature(
        self,
        blocked_lms: tuple[str, ...],
        blocker_names: tuple[str, ...],
    ) -> tuple[tuple[Any, ...], ...]:
        """Snapshot exact causal bodies, falling back to anonymous occupancy."""
        if not blocker_names:
            return self._stationary_blocker_signature(blocked_lms)
        signature: list[tuple[Any, ...]] = []
        for blocker_name in blocker_names:
            blocker = self.robots.get(blocker_name)
            if blocker is None:
                continue
            signature.append((
                blocker.name,
                self._traffic_lm_for_robot(blocker),
                blocker.status,
                int(blocker.route_revision),
                bool(blocker.trajectory),
                str(blocker.active_order_id or ""),
                str(blocker.target_lm or ""),
            ))
        return tuple(sorted(signature))

    def _stationary_clearance_enabled(self) -> bool:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return True
        value = fleet.get("parked_clearance_relocation_enabled", True)
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(value)

    def _stationary_clearance_failure_limit(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            value = int(
                fleet.get("parked_clearance_relocation_failures", 2) or 2
            )
        except (TypeError, ValueError):
            value = 2
        return max(1, min(8, value))

    def _stationary_clearance_cooldown(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            value = float(
                fleet.get("parked_clearance_relocation_cooldown_sec", 12.0)
                or 12.0
            )
        except (TypeError, ValueError):
            value = 12.0
        return max(2.0, min(120.0, value))

    def _stationary_clearance_timeout(self) -> float:
        """Bound a queued/holding maintenance lease without stopping mid-edge."""
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            value = float(
                fleet.get("parked_clearance_relocation_timeout_sec", 120.0)
                or 120.0
            )
        except (TypeError, ValueError):
            value = 120.0
        return max(20.0, min(600.0, value))

    def _prune_stationary_clearance_relocations(
        self,
        now: float | None = None,
    ) -> None:
        """Bound hidden maintenance-order history and expire stale moves."""
        current_time = self._now() if now is None else float(now)
        for blocker_name, state in list(
            self._stationary_clearance_relocations.items()
        ):
            order_id = str(state.get("order_id") or "")
            order = self.orders.get(order_id) if order_id else None
            if (
                order is not None
                and order.internal_kind == "traffic_clearance"
                and order.status in {"QUEUED", "PLANNING"}
            ):
                circular_waiter = self._circular_clearance_waiter(
                    blocker_name,
                    state,
                    order,
                )
                if circular_waiter is not None:
                    canceled = self._cancel_stationary_clearance_order(
                        blocker_name,
                        state,
                        order,
                        current_time,
                        reason=(
                            "causal waiter must depart before maintenance route"
                        ),
                    )
                    if canceled:
                        self._release_waiter_after_circular_clearance(
                            circular_waiter,
                            blocker_name,
                            current_time,
                        )
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                timed_out = bool(
                    current_time
                    - float(state.get("queued_at", current_time) or current_time)
                    >= self._stationary_clearance_timeout()
                )
                if (
                    self._stationary_clearance_cause_is_live(state)
                    and not timed_out
                ):
                    continue

                # The user-visible order which justified this maintenance
                # move has completed, disappeared, or changed target.  A
                # queued move can be canceled immediately.  An executing move
                # may finish its already committed path, unless it is holding
                # on a graph LM; stopping a moving robot half-way along an edge
                # would turn lifecycle cleanup into a new physical obstacle.
                state["orphaned"] = True
                state.setdefault("orphaned_at", current_time)
                state.setdefault(
                    "orphaned_reason",
                    (
                        "maintenance move exceeded its bounded lifetime"
                        if timed_out
                        else "causal waiter order is no longer active"
                    ),
                )
                blocker = self.robots.get(blocker_name)
                safe_lm = (
                    self._stationary_clearance_safe_hold_lm(blocker)
                    if blocker is not None
                    else ""
                )
                safe_to_cancel = bool(
                    blocker is None
                    or (
                        safe_lm
                        and blocker.status
                        in {"IDLE", "ARRIVED", "WAITING", "BLOCKED", "PLANNING"}
                    )
                )
                if not safe_to_cancel:
                    if blocker is not None and not blocker.is_remote():
                        self._arm_stationary_clearance_safe_stop(
                            blocker,
                            state,
                            order,
                            current_time,
                        )
                    continue
                canceled = self._cancel_stationary_clearance_order(
                    blocker_name,
                    state,
                    order,
                    current_time,
                    reason=str(state["orphaned_reason"]),
                )
                if not canceled:
                    # The remote robot may still be executing this route.
                    # Keep both the maintenance order and all route metadata
                    # authoritative until its transport acknowledges cancel.
                    continue
            if order is not None and order.internal_kind == "traffic_clearance":
                self.orders.pop(order_id, None)
            if order_id:
                state["order_id"] = ""
                state["cooldown_until"] = current_time + float(
                    state.get("cooldown_sec", self._stationary_clearance_cooldown())
                    or self._stationary_clearance_cooldown()
                )
            # Keep the compact per-robot history after cooldown. It remembers
            # the pockets already used for the same unchanged waiter episode,
            # preventing B->P followed by P->B oscillation. The dictionary is
            # bounded by fleet size and is cleared when a robot is removed or
            # the manager is reset.
        referenced = {
            str(state.get("order_id") or "")
            for state in self._stationary_clearance_relocations.values()
            if str(state.get("order_id") or "")
        }
        for order_id, order in list(self.orders.items()):
            if (
                order.internal_kind == "traffic_clearance"
                and order.status in TERMINAL_ORDER_STATUSES
                and order_id not in referenced
            ):
                self.orders.pop(order_id, None)

    def _stationary_clearance_lm_is_safe_hold(self, lm_name: str) -> bool:
        """Return whether expiry may stop a maintenance move at this LM."""
        if lm_name not in self.landmarks:
            return False
        graph = self._controlled_corridor_graph
        if graph is None:
            graph = self.planner._traffic_graph(self.planner._route_speed({}))
        vertex = graph.vertices.get(lm_name)
        return bool(
            vertex is not None
            and vertex.can_wait
            and not vertex.controlled_region_ids
        )

    def _stationary_clearance_safe_hold_lm(
        self,
        robot: FleetRobot,
    ) -> str:
        lm_name = self._safe_replan_start_lm(robot)
        return (
            lm_name
            if self._stationary_clearance_lm_is_safe_hold(lm_name)
            else ""
        )

    def _arm_stationary_clearance_safe_stop(
        self,
        robot: FleetRobot,
        state: dict[str, Any],
        order: FleetOrder,
        now: float,
    ) -> bool:
        """Shorten an expired simulated move to its next graph-safe LM.

        Clearing a trajectory while its body is between vertices creates an
        unreserved obstacle (and used to snap its pose back to a landmark).
        Keep the already collision-checked prefix instead.  Normal motion
        reaches its next waitable, external LM and completes the hidden order
        there; no new spatial route is introduced.
        """
        if (
            robot.is_remote()
            or robot.active_order_id != order.order_id
            or not robot.trajectory
        ):
            return False
        if str(state.get("safe_stop_lm") or ""):
            return True

        boundary_index = -1
        boundary_lm = ""
        boundary_clock = 0.0
        for index, sample in enumerate(robot.trajectory):
            sample_clock = float(sample.get("t", 0.0) or 0.0)
            if sample_clock <= robot.route_clock + 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if not self._stationary_clearance_lm_is_safe_hold(lm_name):
                continue
            boundary_index = index
            boundary_lm = lm_name
            boundary_clock = sample_clock
            break
        if boundary_index < 0:
            return False

        trajectory = [
            dict(sample)
            for sample in robot.trajectory[: boundary_index + 1]
        ]
        route_nodes: list[str] = []
        for sample in trajectory:
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name in self.landmarks and (
                not route_nodes or route_nodes[-1] != lm_name
            ):
                route_nodes.append(lm_name)
        if not route_nodes or route_nodes[-1] != boundary_lm:
            route_nodes.append(boundary_lm)

        robot.trajectory = trajectory
        robot.plan_nodes = route_nodes
        robot.target_lm = boundary_lm
        robot.route_chunk_goal_lm = boundary_lm
        robot.route_final_lm = boundary_lm
        robot.pending_route = None
        robot.route_revision = self._next_route_revision()
        robot.trajectory_dirty = True
        robot.route_preview_dirty = True
        robot.collision_preflight_due_at = 0.0
        order.target_lm = boundary_lm
        order.targets = [boundary_lm]
        order.step_index = 0
        order.route_nodes = list(route_nodes)
        # Keep the authored evacuation route immutable.  The executable
        # trajectory may end at an earlier safe prefix on expiry, but no later
        # planner may interpret lifecycle truncation as a new spatial route.
        order.updated_at = now
        state["safe_stop_lm"] = boundary_lm
        state["safe_stop_clock"] = boundary_clock
        state["safe_stop_armed_at"] = now
        self._clear_rolling_prefetch_state(robot.name)
        self._event(
            "warn",
            f"{robot.name} expired traffic clearance will stop safely at "
            f"{boundary_lm}",
        )
        return True

    def _circular_clearance_waiter(
        self,
        blocker_name: str,
        relocation: dict[str, Any],
        order: FleetOrder,
    ) -> FleetRobot | None:
        """Return the held waiter physically crossed by its own clearance."""
        signature = relocation.get("waiter_signature", ())
        if not isinstance(signature, (list, tuple)) or len(signature) != 4:
            return None
        waiter_name = str(signature[0] or "")
        waiter_order_id = str(signature[1] or "")
        waiter = self.robots.get(waiter_name)
        if waiter is None or waiter.name == blocker_name:
            return None
        replan = self._runtime_replans.get(waiter.name)
        if (
            not isinstance(replan, dict)
            or str(replan.get("order_id") or "") != waiter_order_id
            or waiter.active_order_id != waiter_order_id
        ):
            return None
        blocker = self.robots.get(blocker_name)
        route_nodes = [
            str(node)
            for node in (order.spatial_route_nodes or order.route_nodes)
            if str(node) in self.landmarks
        ]
        if (
            blocker is None
            or len(route_nodes) < 2
            or not self._clearance_path_crosses_causal_waiter(
                waiter,
                blocker,
                route_nodes,
            )
        ):
            return None
        return waiter

    def _release_waiter_after_circular_clearance(
        self,
        waiter: FleetRobot,
        blocker_name: str,
        now: float,
    ) -> None:
        """Make the causal waiter the next transaction after a bad clearance."""
        state = self._runtime_replans.get(waiter.name)
        if not isinstance(state, dict):
            return
        hold = state.get("corridor_clearance_hold")
        if isinstance(hold, dict) and str(hold.get("owner") or "") == blocker_name:
            state.pop("corridor_clearance_hold", None)
        for key in ("clearance_blocker_names", "blocker_names"):
            remaining = tuple(
                str(name)
                for name in state.get(key, ())
                if str(name) and str(name) != blocker_name
            )
            if remaining:
                state[key] = remaining
            else:
                state.pop(key, None)
        remaining_signatures = tuple(
            signature
            for signature in state.get("causal_blocker_signatures", ())
            if not (
                isinstance(signature, (list, tuple))
                and len(signature) == 3
                and str(signature[0]) == blocker_name
            )
        )
        if remaining_signatures:
            state["causal_blocker_signatures"] = remaining_signatures
        else:
            state.pop("causal_blocker_signatures", None)
        state["stage"] = "queued"
        state["retry_at"] = min(
            float(state.get("retry_at", now) or now),
            float(now),
        )
        waiter.status = "WAITING"
        waiter.last_reason = (
            f"replanning route after circular clearance {blocker_name}"
        )
        waiter.last_tick_at = now
        waiter.updated_at = now
        self._event(
            "warn",
            f"{waiter.name} released before circular traffic clearance "
            f"for {blocker_name}",
        )

    def _stationary_clearance_cause_is_live(
        self,
        state: dict[str, Any],
    ) -> bool:
        """Return whether the exact normal order still needs this move."""
        signature = state.get("waiter_signature", ())
        if not isinstance(signature, (list, tuple)) or len(signature) != 4:
            return False
        waiter_name = str(signature[0] or "")
        waiter_order_id = str(signature[1] or "")
        captured_lm = str(signature[2] or "")
        captured_target = str(signature[3] or "")
        waiter = self.robots.get(waiter_name)
        waiter_order = self.orders.get(waiter_order_id)
        if (
            waiter is None
            or waiter_order is None
            or waiter_order.internal_kind
            or waiter_order.status in TERMINAL_ORDER_STATUSES
            or waiter_name
            not in {
                str(waiter_order.vehicle or ""),
                str(waiter_order.assigned_robot or ""),
            }
            or self._active_order_target(waiter_order) != captured_target
            or self._traffic_lm_for_robot(waiter) != captured_lm
        ):
            return False
        if waiter.active_order_id and waiter.active_order_id != waiter_order_id:
            return False
        return self._active_order_for_robot(waiter) is waiter_order

    def _cancel_stationary_clearance_order(
        self,
        blocker_name: str,
        state: dict[str, Any],
        order: FleetOrder,
        now: float,
        *,
        reason: str,
    ) -> bool:
        """Cancel an orphaned maintenance move at a graph-safe hold point."""
        reason = f"traffic clearance canceled: {reason}"
        blocker = self.robots.get(blocker_name)
        if blocker is not None and blocker.active_order_id == order.order_id:
            # The core owns the lifecycle decision; the transport still owns
            # the physical route.  A gRPC robot must receive the cancellation
            # before its local metadata is cleared, otherwise it keeps driving
            # an order the dispatcher has already retired.
            if not self._cancel_remote_route(blocker, reason):
                return False

        order.status = "CANCELED"
        order.error = reason
        order.updated_at = now
        self._clear_stationary_order_retry_state(order.order_id)

        if blocker is not None and blocker.active_order_id == order.order_id:
            safe_lm = self._safe_replan_start_lm(blocker)
            if safe_lm in self.landmarks:
                blocker.current_lm = safe_lm
            blocker.target_lm = ""
            blocker.status = "IDLE"
            blocker.trajectory = []
            blocker.plan_nodes = []
            blocker.trajectory_dirty = True
            blocker.route_started_at = None
            blocker.route_clock = 0.0
            blocker.last_tick_at = None
            blocker.blocked_since = None
            blocker.traffic_stall_since = None
            blocker.last_replan_at = None
            blocker.last_reason = reason
            blocker.route_note = ""
            blocker.active_order_id = ""
            blocker.pending_route = None
            blocker.route_preview = []
            blocker.route_preview_dirty = True
            blocker.updated_at = now
            self._runtime_replans.pop(blocker.name, None)
            self._clear_wait_dependency(blocker)
            self._clear_remote_route_metadata(blocker)
        self._event("info", f"{blocker_name} {reason}")
        return True

    def _queue_stationary_clearance_relocation(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        *,
        cause: str,
    ) -> bool:
        """Move a genuinely inactive parked body through the normal MAPF path."""
        if (
            not self._stationary_clearance_enabled()
            or waiter.name == blocker.name
            or blocker.status not in {"IDLE", "ARRIVED"}
            or blocker.trajectory
            or blocker.active_order_id
            or blocker.target_lm
            or not blocker.remote_online
        ):
            return False
        pending = self._active_order_for_robot(blocker)
        if pending is not None and pending.status not in TERMINAL_ORDER_STATUSES:
            return False
        if blocker.is_remote():
            owner_id, _ = self._remote_control_owner(blocker)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return False
        waiter_order = self._active_order_for_robot(waiter)
        if (
            waiter_order is None
            or waiter_order.status in TERMINAL_ORDER_STATUSES
            or waiter_order.internal_kind
        ):
            return False

        waiter_signature = (
            waiter.name,
            waiter_order.order_id,
            self._traffic_lm_for_robot(waiter),
            self._active_order_target(waiter_order),
        )

        now = self._now()
        current_state = self._stationary_clearance_relocations.get(blocker.name)
        forbidden_lms: set[str] = set()
        if isinstance(current_state, dict):
            current_order = self.orders.get(
                str(current_state.get("order_id") or "")
            )
            if (
                current_order is not None
                and current_order.status not in TERMINAL_ORDER_STATUSES
            ):
                return False
            if now < float(current_state.get("cooldown_until", 0.0) or 0.0):
                return False
            if tuple(current_state.get("waiter_signature", ())) == waiter_signature:
                forbidden_lms.update(
                    str(lm_name)
                    for lm_name in current_state.get("visited_lms", ())
                    if str(lm_name) in self.landmarks
                )

        route_nodes = self._stationary_clearance_route(
            waiter,
            blocker,
            forbidden_lms=forbidden_lms,
            # A maintenance move exists only to open the waiter's route. It
            # must vacate the portal on the external side, not request the
            # very corridor token currently retained by that waiter/owner.
            # Crossing another controlled corridor would merely replace a
            # physical blockage with an admission cycle.
            avoid_controlled_regions=True,
            require_waiter_release=True,
        )
        if len(route_nodes) < 2:
            # Some layouts have no holding pocket on the blocker's current
            # side: the only genuine release is across an empty authored
            # corridor. Permit that bounded move only when no other robot owns
            # or occupies the required controlled resource. Rolling
            # SIPP/admission still schedules the actual traversal.
            route_nodes = self._stationary_clearance_route(
                waiter,
                blocker,
                forbidden_lms=forbidden_lms,
                avoid_controlled_regions=False,
                require_waiter_release=True,
                require_unowned_controlled_regions=True,
            )
        if len(route_nodes) < 2:
            return False
        start_lm = str(route_nodes[0])
        target_lm = str(route_nodes[-1])
        order_id = f"traffic-clearance-{blocker.name}-{int(now * 1000)}"
        suffix = 1
        while order_id in self.orders:
            suffix += 1
            order_id = (
                f"traffic-clearance-{blocker.name}-{int(now * 1000)}-{suffix}"
            )

        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        relocation = FleetOrder(
            order_id=order_id,
            target_lm=target_lm,
            vehicle=blocker.name,
            priority=max(10_000, int(waiter_order.priority or 0) + 1),
            created_at=now,
            updated_at=now,
            targets=[target_lm],
            speed=float(waiter_order.speed or 0.0),
            acceleration=float(waiter_order.acceleration or 0.0),
            rotate=(
                bool(waiter_order.rotate)
                if waiter_order is not None
                else bool(navigation.get("simulate_rotation", False))
            ),
            turn_speed=float(waiter_order.turn_speed or 0.0),
            stretch_motion_to_reservation_ticks=True,
            spatial_route_nodes=list(route_nodes),
            spatial_route_revision=self._next_route_revision(),
            internal_kind="traffic_clearance",
        )
        self.orders[order_id] = relocation
        visited_lms = set(forbidden_lms)
        visited_lms.update({start_lm, target_lm})
        self._stationary_clearance_relocations[blocker.name] = {
            "order_id": order_id,
            "waiter": waiter.name,
            "waiter_signature": waiter_signature,
            "origin_lm": start_lm,
            "target_lm": target_lm,
            "visited_lms": tuple(sorted(visited_lms)),
            "queued_at": now,
            "cooldown_until": 0.0,
            "cooldown_sec": self._stationary_clearance_cooldown(),
        }
        self._event(
            "warn",
            f"traffic clearance move queued: {blocker.name} "
            f"{start_lm}->{target_lm}; releases {waiter.name} ({cause})",
        )
        return True

    def _active_stationary_clearance_for(
        self,
        blocker_names: set[str] | tuple[str, ...] | list[str],
    ) -> bool:
        """Return whether an exact blocker already has a live clearance task."""
        for blocker_name in blocker_names:
            state = self._stationary_clearance_relocations.get(
                str(blocker_name)
            )
            if not isinstance(state, dict):
                continue
            order = self.orders.get(str(state.get("order_id") or ""))
            if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
                return True
        return False

    def _stationary_retry_is_commanded_component(
        self,
        order: FleetOrder,
        state: dict[str, Any],
    ) -> bool:
        """Allow periodic recovery only when every physical blocker can move."""
        owner_name = str(order.vehicle or order.assigned_robot or "")
        owner = self.robots.get(owner_name)
        blocker_names = tuple(state.get("blocker_names", ()))
        if (
            owner is None
            or not blocker_names
            or not self._robot_departure_pending(owner)
        ):
            return False
        for blocker_name in blocker_names:
            blocker = self.robots.get(str(blocker_name))
            if (
                blocker is None
                or blocker.trajectory
                or blocker.status not in {"IDLE", "ARRIVED", "BLOCKED"}
                or not self._robot_departure_pending(blocker)
            ):
                return False
        return True

    def _stationary_order_retry_ready(
        self,
        order: FleetOrder,
        *,
        force: bool = False,
    ) -> bool:
        state = self._stationary_order_retry_state.get(order.order_id)
        if not state:
            return True
        if force:
            return True
        if int(state.get("failure_count", 0) or 0) < (
            self._stationary_retry_failure_limit()
        ):
            return True
        blocked_lms = tuple(state.get("blocked_lms", ()))
        blocker_names = tuple(state.get("blocker_names", ()))
        if self._stationary_retry_blocker_signature(
            blocked_lms,
            blocker_names,
        ) != state.get("signature"):
            self._stationary_order_retry_state.pop(order.order_id, None)
            return True
        # An unrelated parked body has no action that could make a retry
        # succeed, so retain quarantine until its occupancy signature changes.
        # Only a component whose owner and blockers all have commanded
        # departures gets a bounded joint-recovery cadence.
        if not self._stationary_retry_is_commanded_component(order, state):
            return False
        return (
            self._now() - float(order.updated_at or 0.0)
            >= self._stationary_recovery_retry_interval()
        )

    def _stationary_recovery_retry_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = float(
                fleet.get("stationary_recovery_retry_sec", 4.0) or 4.0
            )
        except (TypeError, ValueError):
            configured = 4.0
        return max(1.0, min(30.0, configured))

    def _stationary_order_is_quarantined(
        self,
        order: FleetOrder,
    ) -> bool:
        state = self._stationary_order_retry_state.get(order.order_id)
        if (
            not state
            or int(state.get("failure_count", 0) or 0)
            < self._stationary_retry_failure_limit()
        ):
            return False
        blocked_lms = tuple(state.get("blocked_lms", ()))
        blocker_names = tuple(state.get("blocker_names", ()))
        return self._stationary_retry_blocker_signature(
            blocked_lms,
            blocker_names,
        ) == state.get("signature")

    def _clear_stationary_order_retry_state(self, order_id: str) -> None:
        self._stationary_order_retry_state.pop(str(order_id or ""), None)

    def _prune_stationary_order_retry_state(self) -> None:
        for order_id in list(self._stationary_order_retry_state):
            if order_id not in self.orders:
                self._stationary_order_retry_state.pop(order_id, None)

    def _planner_conflict_robot_name(self, reason: str) -> str:
        """Extract the member named by MAPF plan validation failures."""
        text = str(reason or "")
        markers = (
            "cbs_resource_conflict:",
            "resource_conflict:",
            "cbs_missing_plan:",
            "missing_plan:",
            "no_low_level_path:",
        )
        for marker in markers:
            marker_index = text.find(marker)
            if marker_index < 0:
                continue
            tail = text[marker_index + len(marker):]
            for robot_name in sorted(self.robots, key=len, reverse=True):
                if tail == robot_name or (
                    tail.startswith(robot_name)
                    and tail[len(robot_name):len(robot_name) + 1]
                    in {":", ";", ",", " ", ")"}
                ):
                    return robot_name
        return ""

    def _ready_simulated_order_entries(
        self,
        orders: list[FleetOrder],
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]]:
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]] = []
        used_robots: set[str] = set()
        for order in orders:
            if not order.vehicle or order.vehicle in used_robots:
                continue
            if not self._order_is_robot_queue_head(order):
                continue
            robot = self.robots.get(order.vehicle)
            if robot is None or robot.is_remote():
                continue
            if not self._robot_can_accept_order(robot, explicit=True):
                continue
            final_goal = self._active_order_target(order)
            start_lm = self._safe_replan_start_lm(robot)
            if not start_lm or start_lm not in self.landmarks:
                self._dispatch_manual_graph_reconnect(order, robot, final_goal)
                used_robots.add(robot.name)
                continue
            if start_lm == final_goal:
                now = self._now()
                robot.current_lm = final_goal
                robot.target_lm = ""
                robot.status = "ARRIVED"
                robot.active_order_id = ""
                robot.last_reason = "order already at target"
                robot.updated_at = now
                completed = self._advance_or_complete_order(order, robot, now)
                self._event(
                    "info",
                    (
                        f"order completed: {order.order_id} {robot.name}@{final_goal}"
                        if completed
                        else f"order step reached: {order.order_id} {robot.name}@{final_goal}"
                    ),
                )
                used_robots.add(robot.name)
                continue
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                # The rolling waypoint and stable spatial suffix are selected
                # after the dispatch batch is formed. Only then do we know
                # which stationary owners will receive an atomic joint plan
                # and may safely be released from persistent occupancy.
                "goalLm": final_goal,
            }
            if robot.pose is not None:
                request["startPose"] = dict(robot.pose)
            entries.append((order, robot, request, final_goal))
            used_robots.add(robot.name)
        return entries

    def _register_ready_dispatch_corridor_intents(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ],
        *,
        now: float,
    ) -> None:
        """Publish a fleet snapshot before spending a MAPF worker turn.

        Registering intents only for the first one or two dispatch candidates
        made the calendar change as the rest of a 50-robot wave slowly became
        visible.  Almost every already-computed route was then stale at commit.
        Congestion A* is cached per order and cheap compared with SIPP/CBS, so
        expose the complete ready wave first; the next scheduler tick produces
        one stable fleet-wide ordering.
        """
        if self._controlled_corridor_scheduler is None:
            return
        for order, robot, raw_request, final_goal in entries:
            existing = self._controlled_corridor_prefetch_intents.get(
                robot.name
            )
            if (
                isinstance(existing, dict)
                and self._controlled_corridor_intent_is_current(
                    robot,
                    order,
                    existing,
                )
            ):
                continue
            request = dict(raw_request)
            start_lm = str(request.get("startLm") or "")
            if (
                not start_lm
                or start_lm not in self.landmarks
                or final_goal not in self.landmarks
            ):
                continue
            try:
                planning_goal = self._rolling_planning_goal(
                    start_lm,
                    final_goal,
                    order,
                    release_robot_names={robot.name},
                )
                request["goalLm"] = planning_goal
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    final_goal,
                    release_robot_names={robot.name},
                )
            except ValueError:
                continue
            self._controlled_corridor_prefetch_intent(
                order,
                robot,
                request,
                prediction_offset=0.0,
                now=now,
            )

    def _corridor_dispatch_readiness_key(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str],
    ) -> tuple[int, float, str]:
        """Join deterministic order dispatch with the central slot calendar."""
        order, robot, raw_request, _ = entry
        intent = self._controlled_corridor_prefetch_intents.get(robot.name)
        if not isinstance(intent, dict):
            return 1, 0.0, order.order_id
        if not self._controlled_corridor_intent_is_current(
            robot,
            order,
            intent,
        ):
            return 1, 0.0, order.order_id
        corridor_request = intent.get("request")
        schedule = self._controlled_corridor_schedule
        slot = (
            schedule.slot_for(robot.name)
            if schedule is not None
            else None
        )
        if (
            isinstance(corridor_request, CorridorRequest)
            and isinstance(slot, CorridorSlot)
            and slot.regions == corridor_request.regions
            and slot.direction == corridor_request.direction
            and slot.staging_lm == corridor_request.staging_lm
            and slot.exit_lm == corridor_request.exit_lm
        ):
            return 0, float(slot.entry_time), order.order_id
        start_lm = str(raw_request.get("startLm") or "")
        if (
            isinstance(corridor_request, CorridorRequest)
            and corridor_request.staging_lm != start_lm
        ):
            return 1, float(corridor_request.earliest_entry), order.order_id
        return 2, (
            float(corridor_request.earliest_entry)
            if isinstance(corridor_request, CorridorRequest)
            else 0.0
        ), order.order_id

    def _coordinate_mutual_stationary_departures(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[set[str], set[str]]:
        """Make physically coupled fresh departures spatially solvable.

        This is the pre-dispatch equivalent of a runtime wait-cycle.  There is
        no executable trajectory yet, so the ordinary wait-for resolver has
        nothing to arbitrate.  Detect only the immediate route prefix: starts
        crossed much later will already have been vacated and are ordinary
        temporal SIPP resources, not a bootstrap deadlock.

        After bounded normal attempts, one actual conflict-connected component
        is selected fairly. Its crossing members take stable A* routes around
        *all* peer starts, then Rolling SIPP/CBS still owns their exact timing.
        The second returned set identifies still-valid coordinator route
        fingerprints; stationary-release preparation must not erase those
        committed bypasses on this or a later retry.
        """
        stationary_entries = [
            entry
            for entry in entries
            if (
                int(entry[0].dispatch_failures or 0) >= 2
                and entry[0].internal_kind != "traffic_clearance"
                and not entry[1].is_remote()
                and not entry[1].trajectory
                and not entry[1].active_order_id
                and entry[1].status in {"IDLE", "ARRIVED", "BLOCKED"}
                and str(entry[2].get("startLm") or "") in self.landmarks
                and entry[3] in self.landmarks
            )
        ]
        stationary_entries.sort(
            key=lambda entry: (
                -int(entry[0].dispatch_failures or 0),
                float(entry[0].updated_at or 0.0),
                entry[1].name,
            )
        )
        by_name = {entry[1].name: entry for entry in stationary_entries}

        # Keep a tiny, pruned scheduler state rather than using failure counts
        # as a cursor. Failure counts change after every rejected MAPF request
        # and previously made the same physical snapshot look new forever.
        state = getattr(self, "_predispatch_component_state", None)
        if not isinstance(state, dict):
            state = {}
            setattr(self, "_predispatch_component_state", state)
        state.setdefault("last_component", ())
        state.setdefault("component_seeds", {})
        state.setdefault("attempts", {})
        state.setdefault("discovery_cursor", "")
        state.setdefault("protected_routes", {})

        def protected_fingerprint(
            entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str],
        ) -> tuple[Any, ...]:
            order, _, request, final_goal = entry
            return (
                order.order_id,
                str(request.get("startLm") or ""),
                final_goal,
                int(order.spatial_route_revision or 0),
                tuple(str(node) for node in order.spatial_route_nodes),
            )

        raw_protected = state.get("protected_routes")
        if not isinstance(raw_protected, dict):
            raw_protected = {}
        protected_routes = {
            name: fingerprint
            for name, fingerprint in raw_protected.items()
            if name in by_name
            and fingerprint == protected_fingerprint(by_name[name])
        }
        state["protected_routes"] = protected_routes
        protected_names = set(protected_routes)
        if len(stationary_entries) < 2:
            return set(), protected_names

        local_limit = max(2, int(self.planner.local_cbs_max_robots))
        all_release_names = {entry[1].name for entry in stationary_entries}
        routes: dict[str, list[str]] = {}

        missing_routes: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ] = []
        for order, robot, request, final_goal in stationary_entries:
            start_lm = str(request.get("startLm") or "")
            existing = [
                str(node)
                for node in order.spatial_route_nodes
                if str(node) in self.landmarks
            ]
            suffix: list[str] = []
            if start_lm in existing and existing[-1:] == [final_goal]:
                candidate = existing[existing.index(start_lm):]
                if all(
                    dst in self.planner.graph.get(src, [])
                    for src, dst in zip(candidate, candidate[1:])
                ):
                    suffix = candidate
            if suffix:
                routes[robot.name] = suffix
            else:
                missing_routes.append((order, robot, request, final_goal))

        # Route discovery is bounded too. Rotating only the missing routes is
        # enough to make candidates beyond the first local-CBS cap visible on
        # later turns without running whole-fleet A* synchronously.
        missing_by_name = {
            entry[1].name: entry
            for entry in missing_routes
        }
        missing_names = sorted(missing_by_name)
        if missing_names:
            cursor = str(state.get("discovery_cursor") or "")
            rotated = [name for name in missing_names if name > cursor]
            rotated.extend(name for name in missing_names if name <= cursor)
            for name in rotated[:local_limit]:
                order, robot, request, final_goal = missing_by_name[name]
                start_lm = str(request.get("startLm") or "")
                try:
                    routes[name] = self._ensure_order_spatial_route(
                        order,
                        start_lm,
                        final_goal,
                        release_robot_names=all_release_names,
                    )
                except ValueError:
                    routes[name] = []
            state["discovery_cursor"] = rotated[
                min(local_limit, len(rotated)) - 1
            ]

        # A small prefix avoids rewriting routes merely because they happen to
        # visit a now-occupied start on the far side of the warehouse. Four
        # edges cover the adjacency/turn envelope that can prevent departure.
        prefix_edges = 4
        starts = {
            name: str(entry[2].get("startLm") or "")
            for name, entry in by_name.items()
        }
        adjacency = {name: set() for name in routes if routes[name]}
        route_names = sorted(adjacency)
        for index, first_name in enumerate(route_names):
            for second_name in route_names[index + 1:]:
                # A MAPF request has one motion payload. Differing speed/turn
                # settings cannot truthfully be called an atomic component.
                if self._order_motion_key(by_name[first_name][0]) != (
                    self._order_motion_key(by_name[second_name][0])
                ):
                    continue
                same_start = bool(
                    starts[first_name]
                    and starts[first_name] == starts[second_name]
                )
                first_crosses_second = starts[second_name] in routes[
                    first_name
                ][1:prefix_edges + 1]
                second_crosses_first = starts[first_name] in routes[
                    second_name
                ][1:prefix_edges + 1]
                if not (
                    same_start
                    or first_crosses_second
                    or second_crosses_first
                ):
                    continue
                adjacency[first_name].add(second_name)
                adjacency[second_name].add(first_name)

        components: list[tuple[str, ...]] = []
        unseen = {name for name, peers in adjacency.items() if peers}
        while unseen:
            seed = min(unseen)
            queued = [seed]
            seen = {seed}
            unseen.remove(seed)
            while queued:
                name = queued.pop(0)
                neighbours = sorted(adjacency[name] - seen)
                seen.update(neighbours)
                unseen.difference_update(neighbours)
                queued.extend(neighbours)
            components.append(tuple(sorted(seen)))
        components.sort()
        if not components:
            state["last_component"] = ()
            state["component_seeds"] = {}
            state["attempts"] = {}
            return set(), protected_names

        component_keys = set(components)
        component_seeds = state.get("component_seeds")
        if not isinstance(component_seeds, dict):
            component_seeds = {}
        component_seeds = {
            key: value
            for key, value in component_seeds.items()
            if key in component_keys
        }
        attempts = state.get("attempts")
        if not isinstance(attempts, dict):
            attempts = {}
        attempts = {
            key: value
            for key, value in attempts.items()
            if isinstance(key, tuple) and key and key[0] in component_keys
        }
        state["component_seeds"] = component_seeds
        state["attempts"] = attempts

        last_component = tuple(state.get("last_component") or ())
        selected_component = next(
            (component for component in components if component > last_component),
            components[0],
        )
        state["last_component"] = selected_component

        # Components larger than the configured local solver are traversed by
        # a rotating bounded BFS. This keeps the submitted subset connected
        # while eventually exposing every member of an oversized component.
        previous_seed = str(component_seeds.get(selected_component) or "")
        seed_name = next(
            (name for name in selected_component if name > previous_seed),
            selected_component[0],
        )
        component_seeds[selected_component] = seed_name
        selected_names: list[str] = []
        queued = [seed_name]
        seen = {seed_name}
        while queued and len(selected_names) < local_limit:
            name = queued.pop(0)
            selected_names.append(name)
            neighbours = sorted(adjacency[name] - seen)
            seen.update(neighbours)
            queued.extend(neighbours)
        coordinated = set(selected_names)

        def component_snapshot() -> tuple[Any, ...]:
            return tuple(
                (
                    name,
                    by_name[name][0].order_id,
                    starts[name],
                    by_name[name][3],
                    int(by_name[name][1].route_revision or 0),
                    int(by_name[name][0].spatial_route_revision or 0),
                    tuple(routes.get(name, [])),
                )
                for name in selected_component
            )

        attempt_key = (selected_component, tuple(sorted(coordinated)))
        external_stationary_lms = tuple(
            sorted(
                self._stationary_robot_blocked_lms(
                    exclude_robot_names=set(selected_component),
                )
            )
        )
        before_snapshot = component_snapshot()
        before_snapshot = (*before_snapshot, ("blocked", external_stationary_lms))
        if attempts.get(attempt_key) == before_snapshot:
            return coordinated, protected_names

        def immediate_conflicts(name: str, route: list[str]) -> set[str]:
            start_lm = starts[name]
            prefix = route[1:prefix_edges + 1]
            return {
                peer_name
                for peer_name in selected_component
                if peer_name != name
                and (
                    starts[peer_name] == start_lm
                    or starts[peer_name] in prefix
                )
            }

        # Lower numerical task priority yields. At equal priority the older or
        # more-failed order keeps the direct route, so the newer/less-starved
        # member considers the bypass first.
        candidates = sorted(
            (by_name[name] for name in coordinated),
            key=lambda entry: (
                int(entry[0].priority or 0),
                int(entry[0].dispatch_failures or 0),
                -float(entry[0].created_at or 0.0),
                entry[1].name,
            ),
        )
        rerouted_names: set[str] = set()
        component_start_lms = {
            starts[name]
            for name in selected_component
            if starts[name]
        }
        for order, robot, request, final_goal in candidates:
            current_route = routes.get(robot.name, [])
            conflicts_before = immediate_conflicts(robot.name, current_route)
            if not conflicts_before:
                continue
            start_lm = str(request.get("startLm") or "")
            peer_start_lms = component_start_lms - {start_lm}
            blocked_edges = (
                self._dynamic_blocked_edges()
                | set(order.traffic_detour_edges)
                | self._blocked_edges_for_lms(set(external_stationary_lms))
                | self._blocked_edges_for_lms(peer_start_lms)
            )
            edge_penalties = (
                self._traffic_route_edge_penalties(
                    order,
                    start_lm,
                    final_goal,
                )
                if self._congestion_routing_enabled()
                else None
            )
            try:
                alternate = self.planner.route_planner.find_route(
                    start_lm,
                    final_goal,
                    blocked_edges=blocked_edges,
                    edge_penalties=edge_penalties,
                )
            except ValueError:
                continue
            alternate_nodes = [str(node) for node in alternate.nodes]
            conflicts_after = immediate_conflicts(
                robot.name,
                alternate_nodes,
            )
            if (
                len(alternate_nodes) < 2
                or alternate_nodes == current_route
                or len(conflicts_after) >= len(conflicts_before)
            ):
                continue

            # The first translation must open clearance from every peer body
            # it used to cross, not merely choose a graph-theoretic detour.
            start_vertex = self.landmarks.get(start_lm)
            next_vertex = self.landmarks.get(alternate_nodes[1])
            opens_clearance = True
            if start_vertex is not None and next_vertex is not None:
                for peer_name in conflicts_before:
                    peer_vertex = self.landmarks.get(starts[peer_name])
                    if peer_vertex is None:
                        continue
                    start_distance = math.hypot(
                        float(start_vertex.x) - float(peer_vertex.x),
                        float(start_vertex.y) - float(peer_vertex.y),
                    )
                    next_distance = math.hypot(
                        float(next_vertex.x) - float(peer_vertex.x),
                        float(next_vertex.y) - float(peer_vertex.y),
                    )
                    if next_distance + 0.000001 < start_distance:
                        opens_clearance = False
                        break
            if not opens_clearance:
                continue

            order.spatial_route_nodes = alternate_nodes
            order.spatial_route_revision = self._next_route_revision()
            order.traffic_detour_attempts += 1
            order.traffic_blocked_since = None
            routes[robot.name] = alternate_nodes
            rerouted_names.add(robot.name)
            protected_routes[robot.name] = protected_fingerprint(
                by_name[robot.name]
            )
            self._event(
                "warn",
                f"pre-dispatch traffic release: {robot.name} routes around "
                f"stationary departures {', '.join(sorted(conflicts_before))}",
            )

        # Store the post-decision physical snapshot. Dispatch failure counters
        # and timestamps are deliberately absent, so an unchanged component
        # cannot consume A* and allocate route revisions on every retry.
        attempts[attempt_key] = (
            *component_snapshot(),
            ("blocked", external_stationary_lms),
        )
        return coordinated, protected_names | rerouted_names

    def _dispatch_manual_graph_reconnect(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        final_goal: str,
    ) -> bool:
        """Return a manually moved simulated robot to the traffic graph.

        MAPF starts at graph landmarks by design. A free-drive pose therefore
        needs a short collision-checked approach chunk before the queued graph
        route can be planned. Keeping it as part of the same order avoids both
        teleporting the robot and silently dropping the operator's goal.
        """
        if robot.is_remote() or robot.pose is None:
            self._set_order_error(order, "robot has no graph-safe start pose")
            return False
        reconnect_lm = self._nearest_lm_for_robot(robot)
        landmark = self.landmarks.get(reconnect_lm)
        if landmark is None:
            self._set_order_error(order, "no graph landmark near manual pose")
            return False

        start_pose = {
            "x": float(robot.pose.get("x", 0.0) or 0.0),
            "y": float(robot.pose.get("y", 0.0) or 0.0),
            "yaw": float(robot.pose.get("yaw", 0.0) or 0.0),
        }
        dx = float(landmark.x) - start_pose["x"]
        dy = float(landmark.y) - start_pose["y"]
        distance = math.hypot(dx, dy)
        if distance <= 0.000001:
            robot.current_lm = reconnect_lm
            return False

        navigation = self.params.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = {}
        speed = max(
            0.05,
            float(order.speed or navigation.get("route_speed", 0.35) or 0.35),
        )
        turn_speed = max(
            0.05,
            float(order.turn_speed or navigation.get("max_angular_speed", 0.9) or 0.9),
        )
        heading = math.atan2(dy, dx)
        yaw_delta = math.atan2(
            math.sin(heading - start_pose["yaw"]),
            math.cos(heading - start_pose["yaw"]),
        )
        sample_dt = max(0.03, min(0.08, self.collision.sample_time_step() / 2.0))
        rotate_duration = abs(yaw_delta) / turn_speed
        rotate_steps = max(1, int(math.ceil(rotate_duration / sample_dt))) if rotate_duration > 0.01 else 0
        move_duration = distance / speed
        move_steps = max(
            1,
            int(math.ceil(max(move_duration / sample_dt, distance / 0.04))),
        )
        trajectory: list[dict[str, Any]] = [
            {
                "t": 0.0,
                **start_pose,
                "edgeId": f"MANUAL->{reconnect_lm}",
                "motionDirection": "not_specified",
            }
        ]
        for index in range(1, rotate_steps + 1):
            ratio = index / rotate_steps
            trajectory.append(
                {
                    "t": rotate_duration * ratio,
                    "x": start_pose["x"],
                    "y": start_pose["y"],
                    "yaw": start_pose["yaw"] + yaw_delta * ratio,
                    "edgeId": f"MANUAL->ROTATE@{reconnect_lm}",
                    "motionDirection": "rotate",
                }
            )
        for index in range(1, move_steps + 1):
            ratio = index / move_steps
            sample: dict[str, Any] = {
                "t": rotate_duration + move_duration * ratio,
                "x": start_pose["x"] + dx * ratio,
                "y": start_pose["y"] + dy * ratio,
                "yaw": heading,
                "edgeId": f"MANUAL->{reconnect_lm}",
                "motionDirection": "forward",
            }
            if index == move_steps:
                sample["lm"] = reconnect_lm
            trajectory.append(sample)

        for sample in trajectory:
            reason = self.collision.blocked_reason(
                pose=sample,
                obstacles=self.obstacles,
                obstacle_areas=self.obstacle_areas,
            )
            if not reason:
                for other in self._runtime_robots():
                    if other.name == robot.name or other.pose is None:
                        continue
                    if self.collision.robot_footprints_conflict(sample, other.pose):
                        reason = f"robot footprint conflict with {other.name}"
                        break
            if reason:
                self._set_order_error(order, f"manual graph reconnect blocked: {reason}")
                return False

        now = self._now()
        robot.current_lm = reconnect_lm
        robot.target_lm = reconnect_lm
        robot.status = "MOVING"
        robot.trajectory = trajectory
        robot.trajectory_dirty = True
        robot.plan_nodes = [reconnect_lm]
        robot.route_started_at = now
        robot.route_clock = 0.0
        robot.last_tick_at = now
        robot.blocked_since = None
        robot.last_replan_at = None
        robot.last_reason = "returning manual pose to traffic graph"
        robot.route_note = "manual graph reconnect"
        robot.active_order_id = order.order_id
        robot.route_revision = self._next_route_revision()
        robot.route_chunk_index = 0
        robot.route_chunk_goal_lm = reconnect_lm
        robot.route_final_lm = final_goal
        robot.route_preview = [dict(sample) for sample in trajectory]
        robot.route_preview_dirty = True
        robot.pending_route = None
        robot.has_executed_route = True
        robot.updated_at = now

        order.status = "EXECUTING"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = reconnect_lm
        order.route_nodes = [reconnect_lm]
        self._event(
            "info",
            f"manual graph reconnect: {robot.name}->{reconnect_lm}; then {final_goal}",
        )
        return True

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

    def _dispatch_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[int, set[str]]:
        if not entries:
            return 0, set()
        try:
            requests, payload = self._prepare_simulated_order_batch(entries)
        except ValueError as exc:
            for order, _, _, _ in entries:
                if order.status == "PLANNING":
                    order.status = "QUEUED"
                if order.internal_kind == "traffic_clearance":
                    self._set_order_error(
                        order,
                        f"traffic clearance route invalid: {exc}",
                    )
            return 0, {order.order_id for order, _, _, _ in entries}
        result = self._plan_valid_requests(requests, payload)
        return self._finish_simulated_order_batch(entries, result)

    def _prepare_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        released_owners = {robot.name for _, robot, _, _ in entries}
        protected_starts = {
            str(request.get("startLm") or "")
            for _, _, request, _ in entries
        }
        reserved_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        for order, robot, raw_request, final_goal in entries:
            request = dict(raw_request)
            start_lm = str(request["startLm"])
            planning_goal = self._rolling_planning_goal(
                start_lm,
                final_goal,
                order,
                release_robot_names=released_owners,
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
            reserved_goals.add(planning_goal)
            request["goalLm"] = planning_goal
            request.pop("routeNodes", None)
            self._attach_spatial_route_to_request(
                request,
                order,
                start_lm,
                planning_goal,
                final_goal,
                release_robot_names=released_owners,
            )
            raw_request.clear()
            raw_request.update(request)
            requests.append(request)
            self._set_order_status(
                order,
                "PLANNING",
                robot=robot,
                start_lm=start_lm,
            )
        first_order = entries[0][0]
        payload = self._order_plan_payload(first_order, requests[0]) | {"robots": requests}
        return requests, payload

    def _distinct_rolling_batch_goal(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        planning_goal_lm: str,
        *,
        reserved_goals: set[str],
        protected_starts: set[str],
        release_robot_names: set[str],
    ) -> str:
        """Keep converging rolling requests from sharing a terminal vertex."""
        goal_uses_other_start = (
            planning_goal_lm in protected_starts
            and planning_goal_lm != start_lm
        )
        if planning_goal_lm not in reserved_goals and not goal_uses_other_start:
            return planning_goal_lm
        try:
            route_nodes = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
                release_robot_names=release_robot_names,
            )
        except ValueError:
            if order.internal_kind == "traffic_clearance":
                raise
            return planning_goal_lm
        if planning_goal_lm not in route_nodes:
            return planning_goal_lm
        target_index = route_nodes.index(planning_goal_lm)
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        graph = self.planner._traffic_graph(
            self.planner._route_speed(route_payload),
        )
        minimum_distance = max(
            0.0,
            float(self.planner.min_robot_center_distance_m),
        )

        def usable(candidate: str) -> bool:
            if (
                candidate == start_lm
                or candidate in reserved_goals
                or candidate in protected_starts
            ):
                return False
            vertex = graph.vertices.get(candidate)
            if vertex is not None and not vertex.can_wait:
                return False
            landmark = self.landmarks.get(candidate)
            if landmark is None:
                return False
            return all(
                other not in self.landmarks
                or math.hypot(
                    landmark.x - self.landmarks[other].x,
                    landmark.y - self.landmarks[other].y,
                ) + 0.000001 >= minimum_distance
                for other in reserved_goals
            )

        # Prefer a slightly earlier holding vertex so the bounded request does
        # not grow.  When the requested endpoint is another robot's current
        # stop line, extending *through* it can pull a second controlled
        # corridor into this request and turn one ordinary wait into a long
        # no-wait chain.  Keep the blocked endpoint authoritative in that
        # case: SIPP reports its owner and the next central recovery turn moves
        # the real dependency first.
        for index in range(target_index - 1, 0, -1):
            candidate = str(route_nodes[index])
            if usable(candidate):
                return candidate
        if goal_uses_other_start:
            return planning_goal_lm
        for index in range(target_index + 1, len(route_nodes)):
            candidate = str(route_nodes[index])
            if usable(candidate):
                return candidate
        return planning_goal_lm

    def _finish_simulated_order_batch(
        self,
        entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str]],
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
            reason = self._planner_failure_reason(result)
            debug = result.get("debug")
            if not isinstance(debug, dict):
                debug = {}
            conflicts_by_requester = (
                self._record_dispatch_conflict_dependencies(
                    entries,
                    debug,
                )
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
                conflict_is_structured = bool(conflicts_by_requester)
                affected = (
                    robot.name in conflicts_by_requester
                    if conflict_is_structured
                    else not isolated or robot.name == conflict_robot
                )
                if not affected:
                    # The shared request failed validation for one named
                    # member. Keep the other queue heads immediately eligible
                    # instead of copying the same error/failure backoff to the
                    # whole group.
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
        plans_by_robot = {
            str(plan.get("robot")): plan
            for plan in result.get("plans", [])
            if isinstance(plan, dict)
        }
        accepted: list[dict[str, Any]] = []
        accepted_entries: list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, dict[str, Any]]] = []
        finish_now = self._now()
        for order, robot, request, final_goal in entries:
            plan = plans_by_robot.get(robot.name)
            if plan is None:
                self._set_order_error(order, "planner did not return robot plan")
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
            corridor_gate = gate_by_robot.get(robot.name)
            if isinstance(corridor_gate, dict):
                if bool(plan.get("corridorPassageDeferred")):
                    intent = corridor_gate.get("intent")
                    if (
                        self._controlled_corridor_prefetch_intents.get(
                            robot.name
                        )
                        is intent
                    ):
                        self._controlled_corridor_prefetch_intents.pop(
                            robot.name,
                            None,
                        )
                    corridor_gate = None
            if isinstance(corridor_gate, dict):
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
                    gate_reason = (
                        "corridor slot changed before command commit"
                    )
                if not gate_current:
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
                    continue
            if self._wait_only_rolling_plan(plan, final_goal):
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
                continue
            accepted.append(plan)
            accepted_entries.append((order, robot, request, final_goal, plan))

        if not accepted:
            return 0, handled
        accepted_result = {**result, "plans": accepted}
        now = self._now()
        self._apply_planner_result(accepted_result, now)
        for order, robot, request, _, plan in accepted_entries:
            self._dispatch_conflict_dependencies.pop(
                order.order_id,
                None,
            )
            order.route_nodes = [str(item) for item in plan.get("nodes", [])]
            robot.active_order_id = order.order_id
            self._apply_simulated_route_metadata(robot, order, plan, now)
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
        return len(accepted_entries), handled

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

    def _async_simulated_dispatch_active(self) -> bool:
        with self._dispatch_job_lock:
            return self._dispatch_job is not None

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

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background planner failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-dispatch",
            daemon=True,
        ).start()
        return True

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

        def run() -> None:
            try:
                result = self._plan_valid_requests([request], payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background runtime replan failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-runtime-replan",
            daemon=True,
        ).start()
        return True

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
                return
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

    def _rolling_prefetch_candidates(
        self,
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]:
        lead = self._rolling_prefetch_lead()
        now = self._now()
        required_stopped_blockers: set[str] = set()
        for requester_name in list(self._rolling_prefetch_blockers):
            for blocker_name in self._valid_rolling_prefetch_blockers(
                requester_name
            ):
                blocker = self.robots.get(blocker_name)
                if (
                    blocker is not None
                    and self._robot_waits_at_rolling_boundary(blocker)
                ):
                    required_stopped_blockers.add(blocker_name)
        candidates: list[
            tuple[
                tuple[float, float, float, str],
                FleetOrder,
                FleetRobot,
                dict[str, Any],
                str,
                float,
            ]
        ] = []
        for robot in self._runtime_robots():
            if (
                robot.is_remote()
                or robot.pending_route is not None
                or robot.status not in {"MOVING", "WAITING"}
                or not robot.active_order_id
                or not robot.route_chunk_goal_lm
                or not robot.trajectory
            ):
                continue
            order = self.orders.get(robot.active_order_id)
            if order is None or order.status in TERMINAL_ORDER_STATUSES:
                continue
            final_goal = self._active_order_target(order)
            if not final_goal or robot.route_chunk_goal_lm == final_goal:
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            remaining = max(0.0, final_time - robot.route_clock)
            if remaining > lead:
                continue
            self._rolling_prefetch_eligible_since.setdefault(robot.name, now)
            if remaining <= 0.000001 and robot.rolling_boundary_since is None:
                # Normal physics completion records this once. The timestamp
                # fallback also covers restored state and deterministic tests.
                robot.rolling_boundary_since = self._rolling_boundary_wait_since(
                    order,
                    robot,
                    now,
                )
            if (
                now + 0.000001
                < self._rolling_prefetch_retry_at.get(robot.name, 0.0)
                and robot.name not in required_stopped_blockers
            ):
                continue
            start_lm = robot.route_chunk_goal_lm
            try:
                planning_goal = self._rolling_planning_goal(
                    start_lm,
                    final_goal,
                    order,
                )
                handoff_pose = self._pose_at_trajectory(
                    robot.trajectory,
                    final_time,
                ) or self._pose_at_landmark(start_lm)
                request: dict[str, Any] = {
                    "name": robot.name,
                    "startLm": start_lm,
                    "goalLm": planning_goal,
                    # Preserve the exact arrival yaw. Resetting to the landmark's
                    # synthetic yaw=0 made the body rotate instantaneously at the
                    # rolling handoff and could sweep through a nearby robot.
                    "startPose": handoff_pose,
                }
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    final_goal,
                )
            except ValueError as exc:
                if order.internal_kind != "traffic_clearance":
                    raise
                self._defer_invalid_clearance_route(order, robot, now, exc)
                continue
            priority = self._rolling_prefetch_candidate_priority(
                order,
                robot,
                remaining,
                now,
            )
            candidates.append(
                (priority, order, robot, request, final_goal, remaining)
            )
        return [
            (order, robot, request, final_goal, remaining)
            for _, order, robot, request, final_goal, remaining in sorted(
                candidates,
                key=lambda item: item[0],
            )
        ]

    def _defer_invalid_clearance_route(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        now: float,
        error: Exception,
    ) -> None:
        """Hold a broken maintenance route for bounded lifecycle cleanup."""
        reason = f"traffic clearance route invalid: {error}"
        order.error = reason
        order.updated_at = now
        if robot.status == "WAITING":
            robot.last_reason = reason
        robot.last_tick_at = now
        robot.updated_at = now
        self._rolling_prefetch_retry_at[robot.name] = (
            now + self._order_dispatch_retry_interval(order)
        )

    def _rolling_boundary_wait_since(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        now: float,
    ) -> float:
        """Return a stable age for a stopped continuation holder."""
        if robot.rolling_boundary_since is not None:
            return min(now, float(robot.rolling_boundary_since))
        eligible = self._rolling_prefetch_eligible_since.get(robot.name)
        timestamps = [
            float(value)
            for value in (
                eligible,
                order.updated_at,
                robot.updated_at,
            )
            if value is not None and float(value) > 0.0
        ]
        return min([now, *timestamps]) if timestamps else now

    def _rolling_boundary_priority(
        self,
        entry: tuple[FleetOrder, FleetRobot, dict[str, Any], str, float],
        now: float,
    ) -> tuple[float, float, str]:
        """Least-recently-served ordering with oldest-waiter fallback."""
        order, robot, _, _, _ = entry
        waiting_since = self._rolling_boundary_wait_since(order, robot, now)
        last_attempt = self._rolling_prefetch_last_attempt_at.get(robot.name)
        if last_attempt is None:
            # Tests and restored older snapshots may contain a failure count
            # without the new service timestamp. Approximate those completed
            # turns without allowing failures to erase a genuinely old age.
            last_attempt = waiting_since + (
                min(
                    8,
                    max(
                        0,
                        int(self._rolling_prefetch_failures.get(robot.name, 0)),
                    ),
                )
                * self._rolling_boundary_retry_interval(order)
            )
        service_anchor = max(waiting_since, float(last_attempt))
        return service_anchor, waiting_since, robot.name

    def _rolling_prefetch_candidate_priority(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        remaining: float,
        now: float,
    ) -> tuple[float, float, float, str]:
        """Earliest-deadline scheduling across motion and stopped holders.

        A moving route's hard deadline is its chunk end. A boundary holder's
        service deadline is one short retry quantum after its least-recent
        attempt. This prevents both failure modes: an endless urgent stream
        cannot starve stopped robots, while a just-serviced blocked holder
        cannot repeatedly preempt a route that is about to expire.
        """
        if remaining <= 0.000001:
            service_anchor, waiting_since, name = self._rolling_boundary_priority(
                (order, robot, {}, "", remaining),
                now,
            )
            return (
                service_anchor + self._rolling_boundary_retry_interval(order),
                1.0,
                waiting_since,
                name,
            )
        return (
            now + remaining,
            0.0,
            self._rolling_prefetch_eligible_since.get(robot.name, now),
            robot.name,
        )

    def _rolling_full_collapse_release_entries(
        self,
    ) -> (
        list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]
        | None
    ):
        """Release a completely stopped rolling cohort in vacancy order.

        Ordinary rolling retries remain authoritative while even one active
        simulated robot can still make progress. This path is intentionally
        armed only after every active non-terminal order has exhausted its
        chunk, has no pending/retreat motion, and has already failed a normal
        continuation attempt.
        """
        cohort: list[tuple[FleetOrder, FleetRobot]] = []
        for robot in self._runtime_robots():
            if robot.is_remote() or not robot.active_order_id:
                continue
            order = self.orders.get(robot.active_order_id)
            if order is None or order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.internal_kind == "traffic_clearance":
                # Full-collapse vacancy recovery deliberately replaces one
                # spatial suffix with a route to an arbitrary free pocket.
                # A maintenance clearance already *is* such a bounded escape
                # and its authored route is immutable until completion/cancel.
                return None
            cohort.append((order, robot))
        if len(cohort) < 2:
            return None
        if any(
            robot.status in {"MOVING", "RETREATING"}
            or robot.pending_route is not None
            or not self._robot_waits_at_rolling_boundary(robot)
            or self._rolling_prefetch_failures.get(robot.name, 0) < 1
            for _, robot in cohort
        ):
            return None

        signature = tuple(sorted(
            (
                robot.name,
                str(robot.route_chunk_goal_lm or ""),
                int(robot.route_revision),
            )
            for _, robot in cohort
        ))
        if signature != self._rolling_vacancy_recovery_signature:
            self._rolling_vacancy_recovery_signature = signature
            self._rolling_vacancy_recovery_blacklist.clear()

        starts = {
            robot.name: str(robot.route_chunk_goal_lm or "")
            for _, robot in cohort
        }
        unique_starts = (
            len(set(starts.values())) == len(starts)
            and all(starts.values())
        )
        dependencies: dict[str, set[str]] = {
            robot.name: set()
            for _, robot in cohort
        }
        by_name = {robot.name: (order, robot) for order, robot in cohort}
        dependencies_complete = unique_starts
        dynamic_blocked_edges = self._dynamic_blocked_edges()
        if dependencies_complete:
            for order, robot in cohort:
                route_info = self._rolling_collapse_route_prefix(
                    order,
                    starts[robot.name],
                    self._active_order_target(order),
                    dynamic_blocked_edges=dynamic_blocked_edges,
                )
                if route_info is None:
                    dependencies_complete = False
                    break
                route_prefix, graph = route_info
                prefix_resources = set()
                for node in route_prefix:
                    prefix_resources.update(graph.vertex_resources(node))
                for src, dst in zip(route_prefix, route_prefix[1:]):
                    lane = graph.lane_for(src, dst)
                    if lane is None:
                        dependencies_complete = False
                        break
                    prefix_resources.update(graph.lane_resources(lane))
                if not dependencies_complete:
                    break
                for other_name, other_start in starts.items():
                    if other_name == robot.name:
                        continue
                    occupancy_resources = set(
                        graph.vertex_resources(other_start)
                    )
                    if prefix_resources.intersection(occupancy_resources):
                        dependencies[robot.name].add(other_name)
            for requester_name in dependencies:
                dependencies[requester_name].update(
                    blocker_name
                    for blocker_name in self._valid_rolling_prefetch_blockers(
                        requester_name
                    )
                    if blocker_name in dependencies
                    and blocker_name != requester_name
                )

        sinks = (
            [
                name
                for name, blocked_by in dependencies.items()
                if not blocked_by
            ]
            if dependencies_complete
            else []
        )
        if dependencies_complete and sinks:
            incoming = {
                name: sum(
                    name in blocked_by
                    for blocked_by in dependencies.values()
                )
                for name in sinks
            }
            sink_name = min(
                sinks,
                key=lambda name: (-incoming[name], name),
            )
            order, robot = by_name[sink_name]
            return [self._rolling_collapse_prefetch_entry(order, robot)]

        vacancy_entry = self._rolling_vacancy_escape_entry(
            cohort,
            signature,
        )
        # No free pocket is not a terminal scheduler state. Falling back to
        # the ordinary fair endpoint queue lets SIPP/local recovery retry when
        # traffic changes instead of suppressing all continuations forever.
        return [vacancy_entry] if vacancy_entry is not None else None

    def _rolling_collapse_route_prefix(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        *,
        dynamic_blocked_edges: set[tuple[str, str]],
    ) -> tuple[list[str], Any] | None:
        """Recover a non-mutating, resource-sized boundary dependency route."""
        route_nodes = [
            str(node)
            for node in order.spatial_route_nodes
            if str(node) in self.landmarks
        ]
        if start_lm in route_nodes:
            route_nodes = route_nodes[route_nodes.index(start_lm):]
        cached_route_is_valid = bool(
            len(route_nodes) >= 2
            and route_nodes[0] == start_lm
            and route_nodes[-1] == final_goal_lm
            and all(
                dst in self.planner.graph.get(src, [])
                for src, dst in zip(route_nodes, route_nodes[1:])
            )
        )
        if not cached_route_is_valid:
            try:
                route_nodes = [
                    str(node)
                    for node in self.planner.route_planner.find_route(
                        start_lm,
                        final_goal_lm,
                        blocked_edges=(
                            set(order.traffic_detour_edges)
                            | dynamic_blocked_edges
                        ),
                    ).nodes
                ]
            except (RuntimeError, ValueError):
                return None
        if len(route_nodes) < 2:
            return None

        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        graph = self.planner._traffic_graph(
            self.planner._route_speed(route_payload),
        )
        selected_index = graph.extend_route_index_to_controlled_exit(
            route_nodes,
            1,
        )
        selected_index = self._rolling_safe_hold_index(
            route_nodes,
            selected_index,
            final_goal_lm,
            traffic_graph=graph,
        )
        selected_index = min(len(route_nodes) - 1, max(1, selected_index))
        route_prefix = route_nodes[:selected_index + 1]
        if len(route_prefix) < 2:
            return None
        return route_prefix, graph

    def _rolling_collapse_prefetch_entry(
        self,
        order: FleetOrder,
        robot: FleetRobot,
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]:
        final_goal = self._active_order_target(order)
        start_lm = str(robot.route_chunk_goal_lm or "")
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        handoff_pose = (
            self._pose_at_trajectory(robot.trajectory, final_time)
            or self._pose_at_landmark(start_lm)
        )
        request: dict[str, Any] = {
            "name": robot.name,
            "startLm": start_lm,
            "goalLm": final_goal,
        }
        if handoff_pose is not None:
            request["startPose"] = handoff_pose
        return order, robot, request, final_goal, 0.0

    def _rolling_vacancy_escape_entry(
        self,
        cohort: list[tuple[FleetOrder, FleetRobot]],
        signature: tuple[tuple[str, str, int], ...],
    ) -> tuple[FleetOrder, FleetRobot, dict[str, Any], str, float] | None:
        """Find one fixed route from a dependency cycle to a free wait pocket."""
        occupied_lms = {
            self._nearest_lm_for_robot(robot)
            for robot in self._runtime_robots()
        }
        occupied_lms.discard("")
        occupied_lms.update(
            str(robot.route_chunk_goal_lm or "")
            for _, robot in cohort
            if robot.route_chunk_goal_lm
        )
        blocked_edges = self._dynamic_blocked_edges()
        candidates: list[
            tuple[
                float,
                str,
                str,
                FleetOrder,
                FleetRobot,
                list[str],
            ]
        ] = []
        for order, robot in sorted(cohort, key=lambda item: item[1].name):
            start_lm = str(robot.route_chunk_goal_lm or "")
            if start_lm not in self.landmarks:
                continue
            route_payload: dict[str, Any] = {}
            if order.speed > 0.0:
                route_payload["speed"] = order.speed
            if order.acceleration > 0.0:
                route_payload["acceleration"] = order.acceleration
            speed = self.planner._route_speed(route_payload)
            acceleration = self.planner._route_acceleration(route_payload)
            graph = self.planner._traffic_graph(speed)
            source_shared_resources = {
                resource
                for resource in graph.vertex_resources(start_lm)
                if resource.kind in {
                    "controlled_region",
                    "mutex_zone",
                    "clearance",
                }
            }
            other_cohort_occupancy = set()
            for _, other in cohort:
                if other.name == robot.name:
                    continue
                other_start = str(other.route_chunk_goal_lm or "")
                if other_start:
                    other_cohort_occupancy.update(
                        graph.vertex_resources(other_start)
                    )
            blocked_lms = occupied_lms - {start_lm}
            horizon = self._rolling_horizon()
            step_limit = self._rolling_horizon_steps()
            queue: list[
                tuple[float, int, str, tuple[str, ...]]
            ] = [(0.0, 0, start_lm, (start_lm,))]
            best_elapsed = {start_lm: 0.0}
            while queue:
                elapsed, edge_count, node, path_tuple = heappop(queue)
                if elapsed > best_elapsed.get(node, float("inf")) + 0.000001:
                    continue
                for neighbour in sorted(self.planner.graph.get(node, [])):
                    lane = graph.lane_for(node, neighbour)
                    if (
                        lane is None
                        or neighbour in blocked_lms
                        or (node, neighbour) in blocked_edges
                        or set(graph.lane_resources(lane)).intersection(
                            other_cohort_occupancy
                        )
                    ):
                        continue
                    next_edge_count = edge_count + 1
                    next_elapsed = elapsed + (
                        self.planner._edge_tick_cost(
                            node,
                            neighbour,
                            speed,
                            acceleration,
                        )
                        * max(0.001, self.planner.time_step_sec)
                    )
                    if (
                        step_limit > 0
                        and next_edge_count > max(1, step_limit)
                    ):
                        continue
                    if (
                        horizon > 0.0
                        and next_edge_count > 1
                        and next_elapsed > horizon + 0.000001
                    ):
                        continue
                    previous_best = best_elapsed.get(neighbour)
                    if (
                        previous_best is not None
                        and previous_best <= next_elapsed + 0.000001
                    ):
                        continue
                    best_elapsed[neighbour] = next_elapsed
                    next_path = (*path_tuple, neighbour)
                    vertex = graph.vertices.get(neighbour)
                    blacklist_key = (signature, robot.name, neighbour)
                    goal_resources = set(
                        graph.vertex_resources(neighbour)
                    )
                    if (
                        neighbour not in occupied_lms
                        and vertex is not None
                        and vertex.can_wait
                        and not source_shared_resources.intersection(
                            goal_resources
                        )
                        and blacklist_key
                        not in self._rolling_vacancy_recovery_blacklist
                    ):
                        candidates.append(
                            (
                                next_elapsed,
                                robot.name,
                                neighbour,
                                order,
                                robot,
                                list(next_path),
                            )
                        )
                        queue.clear()
                        break
                    heappush(
                        queue,
                        (
                            next_elapsed,
                            next_edge_count,
                            neighbour,
                            next_path,
                        ),
                    )

        if not candidates:
            return None
        _, _, pocket_lm, order, robot, route_nodes = min(
            candidates,
            key=lambda item: item[:3],
        )
        entry = self._rolling_collapse_prefetch_entry(order, robot)
        request = entry[2]
        request.update({
            "goalLm": pocket_lm,
            "routeNodes": route_nodes,
            "vacancyRecovery": True,
        })
        return entry

    def _ready_rolling_prefetch_entries(
        self,
    ) -> list[tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]]:
        collapse_release = self._rolling_full_collapse_release_entries()
        if collapse_release is not None:
            return collapse_release
        candidates = self._rolling_prefetch_candidates()
        if not candidates:
            return []
        release_pressure = self._rolling_boundary_release_pressure()
        now = self._now()
        forced_release = None
        if release_pressure:
            # A terminal holder is the sink of the physical wait chain. Plan
            # it alone first: grouping it with unrelated stopped boundaries
            # can make one CBS/resource failure keep the whole aisle boxed.
            direct_releases = [
                entry
                for entry in candidates
                if float(entry[-1]) <= 0.000001
                and entry[1].name in release_pressure
            ]
            if direct_releases:
                direct_releases.sort(
                    key=lambda entry: (
                        -release_pressure[entry[1].name],
                        *self._rolling_boundary_priority(entry, now),
                    )
                )
                forced_release = direct_releases[0]
                if (
                    self._rolling_prefetch_failures.get(
                        forced_release[1].name,
                        0,
                    )
                    == 0
                ):
                    return [forced_release]

        first = forced_release or candidates[0]
        if float(first[-1]) > 0.000001:
            # An ahead-of-time continuation has a different prediction offset
            # from every peer. Keep that inexpensive request independent.
            return [first]

        motion_key = self._order_motion_key(first[0])
        limit = self._rolling_prefetch_recovery_batch_size()
        # Robots already holding at a chunk boundary must be released
        # together. Planning them one-by-one makes every other holder look
        # like a permanent obstacle and causes planner starvation.
        endpoint_entries = [
            entry
            for entry in candidates
            if float(entry[-1]) <= 0.000001
            and self._order_motion_key(entry[0]) == motion_key
        ]
        # Rotate cheap pair attempts through the stopped fleet. Otherwise the
        # lexicographically first pair is retried after every planner timeout
        # while equally blocked neighbours never become movable participants.
        endpoint_entries.sort(
            key=lambda entry: (
                entry[1].name
                != (forced_release[1].name if forced_release is not None else ""),
                *self._rolling_boundary_priority(entry, now),
            )
        )
        first = forced_release or endpoint_entries[0]
        endpoint_entries = self._rolling_boundary_dependency_component(
            endpoint_entries,
            first,
        )
        seed_failures = self._rolling_prefetch_failures.get(
            first[1].name,
            0,
        )
        if forced_release is None and seed_failures <= 0:
            # First isolate a fresh stopped endpoint.  Coupling two unrelated
            # boundary holders makes one blocked route reject both, and under
            # a full fleet wave that quickly turns every otherwise movable
            # robot into one global retry batch.  Same-LM starts are a real
            # physical component and still need an immediate joint release.
            first_start = str(first[2].get("startLm") or "")
            same_start = [
                entry
                for entry in endpoint_entries
                if str(entry[2].get("startLm") or "") == first_start
            ]
            if len(same_start) <= 1:
                return [first]
            endpoint_entries = same_start
        failures = max(
            (
                self._rolling_prefetch_failures.get(entry[1].name, 0)
                for entry in endpoint_entries
            ),
            default=0,
        )
        # A fixed pair is enough for the usual head-on handoff, but it cannot
        # open a dense boundary where three or more stopped robots mutually
        # occupy the only usable exits. Escalate the local group immediately
        # after a failed attempt while retaining the configured cheap fast
        # path for healthy traffic.
        # Expand only through route dependencies.  Treating every exhausted
        # rolling chunk on the map as one component coupled unrelated aisles:
        # a conflict in one narrow passage then rejected otherwise movable
        # robots in every other passage.
        # A route-overlap component can span most of a dense warehouse even
        # though only its nearest members can release the seed.  Never hand a
        # fleet-wide component to one prioritized-SIPP retry: bounded local
        # waves rotate quickly and let successful endpoints disappear from
        # the next component.
        hard_limit = min(
            len(endpoint_entries),
            self.planner.local_cbs_max_robots,
        )
        limit = min(
            hard_limit,
            max(limit, limit * (2 ** min(4, failures))),
        )
        return endpoint_entries[:limit]

    def _rolling_boundary_dependency_component(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
        ],
        seed: tuple[FleetOrder, FleetRobot, dict[str, Any], str, float],
    ) -> list[
        tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
    ]:
        """Return the local stopped component reachable from ``seed``.

        A boundary holder is a dependency when its start LM lies on another
        holder's committed spatial suffix, or when the preceding SIPP/exact
        footprint validation named it as the actual blocker.  The relation is
        made undirected for recovery because either robot may have to move
        first.  Breadth first ordering lets the cheap 2/4-robot attempts
        include the nearest blockers before a genuinely connected component
        is expanded.
        """
        by_name = {entry[1].name: entry for entry in entries}
        seed_name = seed[1].name
        if seed_name not in by_name:
            return [seed]

        starts = {
            name: str(entry[2].get("startLm") or "")
            for name, entry in by_name.items()
        }
        route_nodes: dict[str, set[str]] = {}
        for name, entry in by_name.items():
            raw_nodes = entry[2].get("routeNodes", [])
            route_nodes[name] = {
                str(node)
                for node in (
                    raw_nodes if isinstance(raw_nodes, list) else []
                )
                if str(node)
            }
        adjacency = {name: set() for name in by_name}
        names = sorted(by_name)
        for index, name in enumerate(names):
            for other_name in names[index + 1:]:
                same_start = bool(
                    starts[name]
                    and starts[name] == starts[other_name]
                )
                route_dependency = bool(
                    starts[other_name] in route_nodes[name]
                    or starts[name] in route_nodes[other_name]
                )
                if not same_start and not route_dependency:
                    continue
                adjacency[name].add(other_name)
                adjacency[other_name].add(name)
        for name in names:
            for blocker_name in self._valid_rolling_prefetch_blockers(name):
                if blocker_name not in by_name or blocker_name == name:
                    continue
                adjacency[name].add(blocker_name)
                adjacency[blocker_name].add(name)

        ordered_names: list[str] = []
        queued = [seed_name]
        seen = {seed_name}
        while queued:
            name = queued.pop(0)
            ordered_names.append(name)
            neighbours = sorted(
                adjacency[name] - seen,
                key=lambda neighbour: self._rolling_boundary_priority(
                    by_name[neighbour],
                    self._now(),
                ),
            )
            seen.update(neighbours)
            queued.extend(neighbours)
        return [by_name[name] for name in ordered_names]

    def _valid_rolling_prefetch_blockers(
        self,
        robot_name: str,
    ) -> set[str]:
        """Return unchanged blocker evidence for one rolling continuation."""
        evidence = self._rolling_prefetch_blockers.get(robot_name)
        robot = self.robots.get(robot_name)
        if not isinstance(evidence, dict) or robot is None:
            self._rolling_prefetch_blockers.pop(robot_name, None)
            return set()
        requester_signature = (
            int(robot.route_revision),
            str(robot.route_chunk_goal_lm or ""),
            str(robot.active_order_id or ""),
        )
        raw_requester_signature = evidence.get("requester")
        if (
            not isinstance(raw_requester_signature, tuple)
            or requester_signature != raw_requester_signature
        ):
            self._rolling_prefetch_blockers.pop(robot_name, None)
            return set()

        blockers = evidence.get("blockers")
        if not isinstance(blockers, dict):
            self._rolling_prefetch_blockers.pop(robot_name, None)
            return set()
        valid: set[str] = set()
        for blocker_name, raw_signature in list(blockers.items()):
            blocker = self.robots.get(str(blocker_name))
            if (
                blocker is None
                or blocker.name == robot_name
                or not isinstance(raw_signature, tuple)
                or (
                    int(blocker.route_revision),
                    str(blocker.route_chunk_goal_lm or ""),
                    str(blocker.active_order_id or ""),
                )
                != raw_signature
            ):
                blockers.pop(blocker_name, None)
                continue
            valid.add(blocker.name)
        if not blockers:
            self._rolling_prefetch_blockers.pop(robot_name, None)
        return valid

    def _record_rolling_prefetch_blockers(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str, float]
        ],
        debug: dict[str, Any],
        *,
        conflict_robot: str = "",
    ) -> None:
        """Persist exact planner dependencies until either route advances."""
        entry_names = {entry[1].name for entry in entries}
        blockers_by_requester: dict[str, set[str]] = {
            name: set()
            for name in entry_names
        }
        raw_conflicts = debug.get("continuousUnresolvedConflicts", ())
        if isinstance(raw_conflicts, (list, tuple)):
            for raw_conflict in raw_conflicts:
                if not isinstance(raw_conflict, dict):
                    continue
                requester = str(raw_conflict.get("robot") or "").strip()
                blocker = str(raw_conflict.get("other") or "").strip()
                if requester in entry_names and blocker:
                    blockers_by_requester[requester].add(blocker)
                elif blocker in entry_names and requester:
                    # Batch collision reports are directional for scheduling,
                    # but both moving participants form one recovery component.
                    blockers_by_requester[blocker].add(requester)

        reservation_blockers = {
            str(name).strip()
            for name in debug.get("reservationBlockerRobots", ())
            if str(name).strip()
        } if isinstance(
            debug.get("reservationBlockerRobots", ()),
            (list, tuple, set),
        ) else set()
        reservation_owner = (
            conflict_robot
            if conflict_robot in entry_names
            else (
                next(iter(entry_names))
                if len(entry_names) == 1
                else ""
            )
        )
        if reservation_owner:
            blockers_by_requester[reservation_owner].update(
                reservation_blockers
            )
            fallback = str(
                debug.get("continuousConflictRobot") or ""
            ).strip()
            if fallback:
                blockers_by_requester[reservation_owner].add(fallback)

        for _, robot, _, _, _ in entries:
            # One planning result is the newest authoritative evidence for
            # this unchanged request. Do not retain a blocker from an older
            # failure when the latest diagnostic names nobody.
            self._rolling_prefetch_blockers.pop(robot.name, None)
            blocker_signatures: dict[str, tuple[int, str, str]] = {}
            for blocker_name in blockers_by_requester.get(robot.name, set()):
                blocker = self.robots.get(blocker_name)
                if blocker is None or blocker.name == robot.name:
                    continue
                blocker_signatures[blocker.name] = (
                    int(blocker.route_revision),
                    str(blocker.route_chunk_goal_lm or ""),
                    str(blocker.active_order_id or ""),
                )
            if not blocker_signatures:
                continue
            self._rolling_prefetch_blockers[robot.name] = {
                "requester": (
                    int(robot.route_revision),
                    str(robot.route_chunk_goal_lm or ""),
                    str(robot.active_order_id or ""),
                ),
                "blockers": blocker_signatures,
            }

    def _rolling_boundary_release_pressure(self) -> dict[str, int]:
        """Count wait-chain robots trapped behind each exhausted chunk."""
        pressure: dict[str, int] = {}
        for waiter in self._runtime_robots():
            if (
                waiter.status != "WAITING"
                or not waiter.trajectory
                or not self._is_robot_conflict(waiter.last_reason)
            ):
                continue
            current = waiter
            visited = {waiter.name}
            depth = 0
            while True:
                blocker_name = (
                    current.wait_for_robot
                    or self._robot_name_from_conflict_reason(current.last_reason)
                )
                if not blocker_name or blocker_name in visited:
                    break
                visited.add(blocker_name)
                blocker = self.robots.get(blocker_name)
                if blocker is None:
                    break
                depth += 1
                if self._robot_waits_at_rolling_boundary(blocker):
                    pressure[blocker.name] = (
                        pressure.get(blocker.name, 0)
                        + max(1, depth)
                    )
                    break
                if (
                    blocker.status != "WAITING"
                    or not blocker.trajectory
                    or not self._is_robot_conflict(blocker.last_reason)
                ):
                    break
                current = blocker
        return pressure

    def _robot_waits_at_rolling_boundary(self, robot: FleetRobot) -> bool:
        if (
            robot.status != "WAITING"
            or not robot.trajectory
            or not robot.active_order_id
            or not robot.route_chunk_goal_lm
        ):
            return False
        order = self.orders.get(robot.active_order_id)
        if (
            order is None
            or order.status != "PLANNING"
            or self._active_order_target(order) == robot.route_chunk_goal_lm
        ):
            return False
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        return (
            robot.route_clock >= final_time - 0.000001
            or str(robot.last_reason or "") == "rolling continuation pending"
        )

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
                corridor_gate_pending.append(robot)
                continue
            if not bool(gate.get("ready")):
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

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background prefetch failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-prefetch",
            daemon=True,
        ).start()
        return True

    def _rolling_recovery_planning_goal(
        self,
        start_lm: str,
        final_goal_lm: str,
        order: FleetOrder,
        *,
        release_robot_names: set[str],
    ) -> str:
        """Release a stopped boundary with one useful rolling chunk.

        A controlled corridor must be committed through its next safe exit.
        Outside an explicitly controlled corridor, however, returning the
        first neighbouring LM creates a three/four-second chunk that becomes
        an urgent prefetch again immediately. A few recovered robots can then
        monopolise the single planner worker and leave the rest of the fleet
        at ``rolling continuation pending``. Ordinary graph space therefore
        uses the normal horizon-sized rolling goal.
        """
        try:
            route_nodes = self._ensure_order_spatial_route(
                order,
                start_lm,
                final_goal_lm,
                release_robot_names=release_robot_names,
            )
        except ValueError:
            if order.internal_kind == "traffic_clearance":
                raise
            return final_goal_lm
        if len(route_nodes) < 2:
            return final_goal_lm
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        speed = self.planner._route_speed(route_payload)
        traffic_graph = self.planner._traffic_graph(speed)
        first_lane = traffic_graph.lane_for(route_nodes[0], route_nodes[1])
        start_vertex = traffic_graph.vertices.get(route_nodes[0])
        if not (
            first_lane is not None
            and first_lane.controlled_region_ids
        ) and not (
            start_vertex is not None
            and start_vertex.controlled_region_ids
        ):
            return self._rolling_planning_goal(
                start_lm,
                final_goal_lm,
                order,
                release_robot_names=release_robot_names,
            )
        exit_index = traffic_graph.extend_route_index_to_controlled_exit(
            route_nodes,
            1,
        )
        exit_index = self._rolling_safe_hold_index(
            route_nodes,
            exit_index,
            final_goal_lm,
            traffic_graph=traffic_graph,
        )
        return str(route_nodes[min(len(route_nodes) - 1, exit_index)])

    def _finish_async_simulated_dispatch(self) -> int:
        with self._dispatch_job_lock:
            job = self._dispatch_job
            if job is None or not bool(job.get("done")):
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
        raw_entries = job.get("entries", [])
        if not isinstance(raw_entries, list):
            return 0
        route_revisions = job.get("route_revisions", {})
        entries = [
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
        if not entries:
            return 0
        result = job.get("result")
        if not isinstance(result, dict) or not result.get("ok") or not result.get("plans"):
            reason = (
                self._planner_failure_reason(result)
                if isinstance(result, dict)
                else "rolling prefetch returned no result"
            )
            debug = (
                result.get("debug", {})
                if isinstance(result, dict)
                and isinstance(result.get("debug"), dict)
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
                self._blacklist_failed_rolling_vacancy(
                    job,
                    robot,
                    request,
                )
                if self._stationary_failure_applies_to_robot(
                    debug,
                    robot.name,
                ):
                    # Continuous footprint validation can reject a spatial
                    # route which avoids the parked robot's LM but still turns
                    # on a neighbouring LM through its swept body envelope.
                    # Rolling-boundary failures used to discard that exact
                    # evidence and retry the identical suffix every 0.5 s.
                    # Feed the blocker into the ordinary bounded relocation
                    # policy and, when possible, exclude the unsafe turn LM
                    # from the next congestion-aware spatial route.
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
                if not isolated_member_failure or robot.name == conflict_robot:
                    self._rolling_prefetch_failures[robot.name] = (
                        self._rolling_prefetch_failures.get(robot.name, 0) + 1
                    )
                self._defer_rolling_prefetch(
                    robot,
                    order,
                    # Keep failed boundary members eligible together. Staggered
                    # retries split the recovery wave back into independent
                    # requests and recreated the same stationary blockers.
                    retry_multiplier=(
                        2.0
                        if isolated_member_failure
                        and robot.name == conflict_robot
                        else 1.0
                    ),
                )
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
        for order, robot, request, final_goal, _ in entries:
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
                continue
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
            ):
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
                    gate_reason = (
                        "corridor slot changed before command commit"
                    )
                if not gate_current:
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
                        robot.last_reason = (
                            "waiting for refreshed corridor slot"
                        )
                        robot.updated_at = finish_now
                    self._event(
                        "info",
                        f"{robot.name} corridor continuation rescheduled: "
                        f"{gate_reason}",
                    )
                    continue
            if plan is None or self._wait_only_rolling_plan(plan, final_goal):
                self._record_rolling_prefetch_blockers(
                    [(order, robot, request, final_goal, 0.0)],
                    (
                        result.get("debug", {})
                        if isinstance(result.get("debug"), dict)
                        else {}
                    ),
                    conflict_robot=robot.name,
                )
                self._blacklist_failed_rolling_vacancy(
                    job,
                    robot,
                    request,
                )
                self._rolling_prefetch_failures[robot.name] = (
                    self._rolling_prefetch_failures.get(robot.name, 0) + 1
                )
                self._defer_rolling_prefetch(
                    robot,
                    order,
                    retry_multiplier=1.0,
                )
                continue
            self._rolling_prefetch_retry_at.pop(robot.name, None)
            self._rolling_prefetch_failures.pop(robot.name, None)
            if self._append_rolling_prefetch(robot, order, plan, final_goal):
                continue
            robot.pending_route = {
                "order_id": order.order_id,
                "start_lm": str(request.get("startLm") or ""),
                "final_goal": final_goal,
                "result": {**result, "plans": [plan]},
            }
        return 0

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

    def _defer_rolling_prefetch(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        *,
        retry_multiplier: float = 1.0,
    ) -> None:
        if self._robot_waits_at_rolling_boundary(robot):
            # A stopped robot already holds a physical graph resource. Long
            # exponential backoff only keeps the aisle closed; fairness is
            # provided by least-recently-served rotation instead.
            self._rolling_prefetch_retry_at[robot.name] = (
                self._now() + self._rolling_boundary_retry_interval(order)
            )
            return
        failures = max(
            1,
            int(self._rolling_prefetch_failures.get(robot.name, 0) or 0),
        )
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        self._rolling_prefetch_retry_at[robot.name] = (
            self._now()
            + (
                self._order_dispatch_retry_interval(order)
                * (2 ** min(3, failures - 1))
                * time_scale
                * max(1.0, retry_multiplier)
            )
        )

    def _rolling_boundary_retry_interval(
        self,
        order: FleetOrder | None = None,
    ) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = max(
                0.1,
                float(fleet.get("rolling_boundary_retry_sec", 0.5) or 0.5),
            )
        except (TypeError, ValueError):
            configured = 0.5
        try:
            maximum = max(
                0.1,
                float(
                    fleet.get("order_dispatch_retry_max_sec", 4.0)
                    or 4.0
                ),
            )
        except (TypeError, ValueError):
            maximum = 4.0
        return min(configured, maximum)

    def _compact_rolling_trajectory_history(
        self,
        robot: FleetRobot,
    ) -> list[dict[str, Any]]:
        """Drop executed samples while retaining a graph-safe retreat tail.

        Rolling chunks used to append onto the complete order trajectory.
        Reservations, websocket serialization and Babylon route updates then
        became progressively more expensive throughout a long order. Keep
        the current/future timeline plus the previous distinct LM required by
        deadlock retreat. Keep the route clock/timestamps monotonic across the
        append so browser interpolation does not see an artificial route reset.
        """
        trajectory = [
            sample
            for sample in robot.trajectory
            if isinstance(sample, dict)
        ]
        if len(trajectory) < 3 or robot.route_clock <= 0.000001:
            return trajectory

        active_index = self._trajectory_segment_index(
            trajectory,
            robot.route_clock,
            boundary_belongs_to_previous=True,
        )
        keep_index = 0
        distinct_lms: list[str] = []
        for index in range(min(active_index, len(trajectory) - 1), -1, -1):
            sample = trajectory[index]
            if float(sample.get("t", 0.0) or 0.0) > robot.route_clock + 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if lm_name not in self.landmarks:
                continue
            if not distinct_lms or distinct_lms[-1] != lm_name:
                distinct_lms.append(lm_name)
            if len(distinct_lms) >= 2:
                keep_index = index
                break

        if keep_index <= 0:
            return trajectory
        compacted = [dict(sample) for sample in trajectory[keep_index:]]
        robot.trajectory = compacted
        compacted_nodes: list[str] = []
        for sample in compacted:
            lm_name = str(sample.get("lm") or "").strip()
            if (
                lm_name in self.landmarks
                and (not compacted_nodes or compacted_nodes[-1] != lm_name)
            ):
                compacted_nodes.append(lm_name)
        if len(compacted_nodes) >= 2:
            robot.plan_nodes = compacted_nodes
        robot.trajectory_dirty = True
        return compacted

    def _append_rolling_prefetch(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
        final_goal: str,
    ) -> bool:
        """Atomically append a safe future chunk without touching execution.

        A rolling continuation is planned from the current chunk's terminal
        LM. Once it is ready, keeping it in a separate pending route forces a
        zero-based route-clock handoff at that LM. The browser then observes a
        short stop even when both chunks are the same straight graph line.

        Joining the two already time-parameterised trajectories preserves the
        current pose, status, route clock and physics tick. Runtime collision
        checking remains authoritative for every future sample.
        """
        current = self._compact_rolling_trajectory_history(robot)
        continuation = [sample for sample in plan.get("trajectory", []) if isinstance(sample, dict)]
        if len(current) < 2 or len(continuation) < 2:
            return False

        expected_start = robot.route_chunk_goal_lm
        plan_start = str(plan.get("startLm") or "").strip()
        if not expected_start or plan_start != expected_start:
            return False
        current_end = current[-1]
        continuation_start = continuation[0]
        position_gap = math.hypot(
            float(current_end.get("x", 0.0) or 0.0)
            - float(continuation_start.get("x", 0.0) or 0.0),
            float(current_end.get("y", 0.0) or 0.0)
            - float(continuation_start.get("y", 0.0) or 0.0),
        )
        if position_gap > self._runtime_replan_lm_tolerance():
            return False

        current_end_time = float(current_end.get("t", 0.0) or 0.0)
        continuation_start_time = float(continuation_start.get("t", 0.0) or 0.0)
        appended: list[dict[str, Any]] = [dict(sample) for sample in current]
        for sample in continuation[1:]:
            shifted = dict(sample)
            shifted["t"] = current_end_time + max(
                0.0,
                float(sample.get("t", continuation_start_time) or continuation_start_time)
                - continuation_start_time,
            )
            appended.append(shifted)
        if float(appended[-1].get("t", 0.0) or 0.0) <= current_end_time + 0.000001:
            return False

        current_nodes = [str(node) for node in robot.plan_nodes]
        continuation_nodes = [str(node) for node in plan.get("nodes", [])]
        if not current_nodes or not continuation_nodes or current_nodes[-1] != continuation_nodes[0]:
            return False
        combined_nodes = current_nodes + continuation_nodes[1:]
        chunk_goal = str(plan.get("goalLm") or continuation_nodes[-1]).strip()
        if not chunk_goal:
            return False

        robot.trajectory = appended
        robot.trajectory_dirty = True
        robot.plan_nodes = combined_nodes
        robot.target_lm = chunk_goal
        robot.route_chunk_goal_lm = chunk_goal
        robot.route_chunk_index = max(0, robot.route_chunk_index + 1)
        robot.route_final_lm = str(plan.get("finalGoalLm") or final_goal).strip()
        robot.route_revision = self._next_route_revision()
        robot.pending_route = None
        robot.has_executed_route = True
        robot.status = "MOVING"
        robot.last_reason = "rolling route continued"
        robot.blocked_since = None
        robot.updated_at = self._now()
        self._clear_rolling_prefetch_state(robot.name)
        order.route_nodes = list(combined_nodes)
        order.status = "EXECUTING"
        order.error = ""
        order.updated_at = robot.updated_at
        self._update_route_preview(
            robot,
            robot.current_lm,
            robot.route_final_lm,
            blocked_edges=set(order.traffic_detour_edges),
            committed_trajectory=appended,
            committed_nodes=combined_nodes,
            spatial_route_nodes=order.spatial_route_nodes,
        )
        self._event(
            "info",
            f"route continuation committed without stop: {order.order_id} "
            f"{robot.name}->{chunk_goal}",
        )
        return True

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

    def _start_async_coupled_replan(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
        now: float,
    ) -> bool:
        robots = self._expand_coupled_replan_component(robots, winner)
        if any(
            (
                order := self._active_order_for_robot(robot)
            ) is not None
            and order.internal_kind == "traffic_clearance"
            for robot in robots
        ):
            # Local CBS is allowed to alter spatial paths. A hidden clearance
            # move, by contrast, owns one immutable outward route; ordinary
            # SIPP/admission may add waits to it, but CBS must never reroute it
            # through the controlled resource it is evacuating.
            return False
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
                # Wait-cycle arbitration runs before dispatch. Preserve an
                # earliest-deadline turn for an expiring rolling trajectory;
                # after that prefetch attempt, a failed cycle may use the
                # following turn so neither recovery class starves.
                return False
        if (
            self._last_async_job_kind != "dispatch"
            and self._queued_simulated_dispatch_waiting(now)
        ):
            # Runtime arbitration runs before the dispatcher on every tick.
            # Without an explicit turn boundary, a different wait-cycle key
            # could occupy the sole planner worker again immediately and
            # leave freshly ARRIVED robots queued indefinitely.
            return False
        cycle_key = tuple(sorted(robot.name for robot in robots))
        if len(cycle_key) < 2:
            return False
        if len(cycle_key) > self.planner.local_cbs_max_robots:
            if cycle_key not in self._coupled_replan_failures:
                self._coupled_replan_failures[cycle_key] = 1
                self.traffic_metrics["coupledReplansFailed"] += 1
                self._event(
                    "warn",
                    f"wait cycle has {len(cycle_key)} robots; local CBS cap is "
                    f"{self.planner.local_cbs_max_robots}",
                )
            return False
        # Re-running the same expensive CBS against unchanged poses cannot
        # produce a different answer. One failed attempt arms deterministic
        # corridor evacuation; another attempt is allowed only after actual
        # traffic progress clears this episode in _record_traffic_progress().
        if self._coupled_replan_failures.get(cycle_key, 0) > 0:
            return False
        last_attempt = self._coupled_replan_last_attempt.get(cycle_key, 0.0)
        if now - last_attempt < self._deadlock_coupled_replan_interval():
            return False

        requests: list[dict[str, Any]] = []
        entries: list[dict[str, Any]] = []
        released_names = set(cycle_key)
        protected_starts = {
            start_lm
            for robot in robots
            if (start_lm := self._safe_replan_start_lm(robot))
            in self.landmarks
        }
        reserved_goals: set[str] = set()
        ordered = [winner] + sorted(
            (robot for robot in robots if robot.name != winner.name),
            key=lambda robot: robot.name,
        )
        for robot in ordered:
            order = self._active_order_for_robot(robot)
            start_lm = self._safe_replan_start_lm(robot)
            final_goal = (
                self._active_order_target(order)
                if order is not None
                else robot.route_final_lm or robot.target_lm
            )
            if (
                order is None
                or not start_lm
                or start_lm not in self.landmarks
                or final_goal not in self.landmarks
            ):
                if cycle_key not in self._coupled_replan_failures:
                    self._coupled_replan_failures[cycle_key] = 1
                    self.traffic_metrics["coupledReplansFailed"] += 1
                    self._event(
                        "warn",
                        f"local CBS cannot start for wait cycle "
                        f"{', '.join(cycle_key)}: {robot.name} is between graph LMs; "
                        "corridor evacuation armed",
                    )
                return False
            planning_goal = self._rolling_planning_goal(
                start_lm,
                final_goal,
                order,
                release_robot_names=released_names,
            )
            planning_goal = self._distinct_rolling_batch_goal(
                order,
                start_lm,
                final_goal,
                planning_goal,
                reserved_goals=reserved_goals,
                protected_starts=protected_starts,
                release_robot_names=released_names,
            )
            request: dict[str, Any] = {
                "name": robot.name,
                "startLm": start_lm,
                "goalLm": planning_goal,
                "startPose": (
                    dict(robot.pose)
                    if robot.pose is not None
                    else self._pose_at_landmark(start_lm)
                ),
            }
            try:
                self._attach_spatial_route_to_request(
                    request,
                    order,
                    start_lm,
                    planning_goal,
                    final_goal,
                    release_robot_names=released_names,
                )
            except ValueError:
                if cycle_key not in self._coupled_replan_failures:
                    self._coupled_replan_failures[cycle_key] = 1
                    self.traffic_metrics["coupledReplansFailed"] += 1
                return False
            reserved_goals.add(planning_goal)
            requests.append(request)
            entries.append(
                {
                    "robot": robot.name,
                    "order": order.order_id,
                    "start": start_lm,
                    "finalGoal": final_goal,
                    "routeRevision": robot.route_revision,
                    # A robot can advance along the same trajectory without
                    # changing its graph LM or route revision while local CBS
                    # is running. Committing a plan built from that older
                    # continuous pose would make it jump back to the request
                    # origin.
                    "routeClock": float(robot.route_clock),
                }
            )

        first_order = self.orders[str(entries[0]["order"])]
        payload = self._order_plan_payload(first_order, requests[0]) | {
            "robots": requests,
            # Keep the production hierarchy intact: stable congestion-aware
            # routes first, fast prioritized Rolling SIPP second, and local
            # CBS only if that exact small component cannot be ordered.
            "plannerBackend": "hybrid",
            "allowCbsFallback": True,
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
            "reservedEdgeDetourEnabled": False,
        }
        # Each request already carries its own congestion route, including its
        # order-specific detour exclusions. Applying the first order's old
        # global blocked edge to every participant is arbitrary and can cut
        # another robot's only outward branch.
        payload.pop("blocked_edges", None)
        job: dict[str, Any] = {
            "kind": "coupled_replan",
            "cycle": cycle_key,
            "entries": entries,
            "requests": requests,
            "result": None,
            "done": False,
        }
        with self._dispatch_job_lock:
            if self._dispatch_job is not None:
                return False
            self._dispatch_job = job
        self._coupled_replan_last_attempt[cycle_key] = now
        self.traffic_metrics["coupledReplansStarted"] += 1

        def run() -> None:
            try:
                result = self._plan_valid_requests(requests, payload)
            except Exception as exc:  # pragma: no cover - defensive worker guard
                result = {
                    "ok": False,
                    "plans": [],
                    "debug": {"reason": f"background coupled CBS failed: {exc}"},
                }
            with self._dispatch_job_lock:
                if self._dispatch_job is job:
                    job["result"] = result
                    job["done"] = True

        Thread(
            target=run,
            name="fleet-mapf-coupled-cbs",
            daemon=True,
        ).start()
        self._event(
            "warn",
            f"local CBS started for wait cycle: {', '.join(cycle_key)}",
        )
        return True

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

    def _order_motion_key(
        self,
        order: FleetOrder,
    ) -> tuple[
        float,
        float,
        bool,
        float,
        bool,
        tuple[tuple[str, str], ...],
    ]:
        return (
            round(float(order.speed), 6),
            round(float(order.acceleration), 6),
            bool(order.rotate),
            round(float(order.turn_speed), 6),
            bool(order.stretch_motion_to_reservation_ticks),
            tuple(
                sorted(
                    (str(src), str(dst))
                    for src, dst in order.traffic_detour_edges
                )
            ),
        )

    def _dispatch_plan_budget(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 2
        try:
            return max(1, min(8, int(fleet.get("dispatch_plan_budget_per_tick", 2) or 2)))
        except (TypeError, ValueError):
            return 2

    def _dispatch_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1
        try:
            return max(
                1,
                min(
                    8,
                    int(
                        fleet.get(
                            "dispatch_joint_batch_size",
                            1,
                        )
                        or 1
                    ),
                ),
            )
        except (TypeError, ValueError):
            return 1

    def _dispatch_rolling_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1
        try:
            return max(1, min(4, int(fleet.get("dispatch_rolling_batch_size", 1) or 1)))
        except (TypeError, ValueError):
            return 1

    def _rolling_prefetch_recovery_batch_size(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured = int(
                fleet.get(
                    "rolling_prefetch_recovery_batch_size",
                    self.planner.local_cbs_max_robots,
                )
                or self.planner.local_cbs_max_robots
            )
        except (TypeError, ValueError):
            configured = self.planner.local_cbs_max_robots
        return max(
            2,
            min(4, self.planner.local_cbs_max_robots, configured),
        )

    def _dispatch_recovery_group_limit(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        batch_size: int,
    ) -> int:
        """Keep normal rolling jobs cheap, but jointly release starved peers."""
        del order
        if robot.has_executed_route:
            # Healthy rolling continuations already see committed peers as
            # reservations and should remain small at 50-robot scale.
            return min(batch_size, self._dispatch_rolling_batch_size())
        return batch_size

    def _dispatch_request_signature(
        self,
        order: FleetOrder,
        robot: FleetRobot,
    ) -> tuple[Any, ...]:
        return (
            order.order_id,
            robot.name,
            self._safe_replan_start_lm(robot),
            self._active_order_target(order),
            int(order.spatial_route_revision or 0),
        )

    def _dispatch_blocker_signature(
        self,
        robot: FleetRobot,
    ) -> tuple[Any, ...]:
        active = self._active_order_for_robot(robot)
        active_id = (
            active.order_id
            if active is not None
            and active.status not in TERMINAL_ORDER_STATUSES
            else str(robot.active_order_id or "")
        )
        return (
            robot.name,
            self._traffic_lm_for_robot(robot),
            int(robot.route_revision),
            str(robot.route_chunk_goal_lm or ""),
            active_id,
            bool(robot.trajectory),
            str(robot.status or ""),
        )

    def _dispatch_conflict_dependency_ready(
        self,
        order: FleetOrder,
    ) -> bool:
        """Wake a failed fresh departure only after its real blocker changes."""
        state = self._dispatch_conflict_dependencies.get(order.order_id)
        if not isinstance(state, dict):
            return True
        owner = str(order.vehicle or order.assigned_robot or "")
        robot = self.robots.get(owner)
        blocker = self.robots.get(str(state.get("blocker") or ""))
        if (
            robot is None
            or blocker is None
            or self._dispatch_request_signature(order, robot)
            != state.get("requester")
            or self._dispatch_blocker_signature(blocker)
            != state.get("blocker_signature")
        ):
            self._dispatch_conflict_dependencies.pop(
                order.order_id,
                None,
            )
            return True
        if self._now() >= float(state.get("probe_at", float("inf"))):
            # A bounded safety probe covers a long graph edge on which the
            # nearest-LM signature may remain unchanged after the conflict
            # footprint has already cleared. It is deliberately much slower
            # than the old 0.5 s retry loop.
            self._dispatch_conflict_dependencies.pop(
                order.order_id,
                None,
            )
            return True
        return False

    def _prune_dispatch_conflict_dependencies(self) -> None:
        for order_id in list(self._dispatch_conflict_dependencies):
            order = self.orders.get(order_id)
            if (
                order is None
                or order.status in TERMINAL_ORDER_STATUSES
            ):
                self._dispatch_conflict_dependencies.pop(
                    order_id,
                    None,
                )

    def _record_dispatch_conflict_dependencies(
        self,
        entries: list[
            tuple[FleetOrder, FleetRobot, dict[str, Any], str]
        ],
        debug: dict[str, Any],
    ) -> dict[str, set[str]]:
        """Persist exact continuous requester→blocker evidence.

        The continuous validator already reports structured identities.  The
        old dispatch path discarded them, copied one failure to an arbitrary
        recovery batch, and progressively grew that batch to eight robots.
        That positive feedback was the main long-running degradation.
        """
        entries_by_robot = {
            robot.name: (order, robot)
            for order, robot, _, _ in entries
        }
        conflicts_by_requester: dict[str, set[str]] = {}
        raw_conflicts = debug.get("continuousUnresolvedConflicts", ())
        if not isinstance(raw_conflicts, (list, tuple)):
            return conflicts_by_requester
        now = self._now()
        for raw_conflict in raw_conflicts:
            if not isinstance(raw_conflict, dict):
                continue
            requester_name = str(raw_conflict.get("robot") or "").strip()
            blocker_name = str(raw_conflict.get("other") or "").strip()
            if (
                requester_name not in entries_by_robot
                or blocker_name not in self.robots
                or blocker_name == requester_name
            ):
                continue
            conflicts_by_requester.setdefault(
                requester_name,
                set(),
            ).add(blocker_name)
            order, requester = entries_by_robot[requester_name]
            blocker = self.robots[blocker_name]
            # Parked, uncommanded bodies already use the relocation/quarantine
            # lifecycle.  This registry is specifically for a blocker whose
            # committed motion can make the same request succeed later.
            blocker_order = self._active_order_for_robot(blocker)
            blocker_is_commanded = bool(
                blocker.trajectory
                or blocker.active_order_id
                or (
                    blocker_order is not None
                    and blocker_order.status
                    not in TERMINAL_ORDER_STATUSES
                )
            )
            if not blocker_is_commanded:
                continue
            try:
                conflict_time = max(
                    0.0,
                    float(raw_conflict.get("time", 0.0) or 0.0),
                )
            except (TypeError, ValueError):
                conflict_time = 0.0
            probe_delay = max(3.0, min(10.0, conflict_time + 1.0))
            self._dispatch_conflict_dependencies[order.order_id] = {
                "requester": self._dispatch_request_signature(
                    order,
                    requester,
                ),
                "blocker": blocker.name,
                "blocker_signature": self._dispatch_blocker_signature(
                    blocker,
                ),
                "resource": str(
                    raw_conflict.get("edge") or "unknown"
                ),
                "source": str(raw_conflict.get("source") or ""),
                "recorded_at": now,
                "probe_at": now + probe_delay,
            }
            requester.last_reason = (
                f"waiting for {blocker.name} before dispatch"
            )
            requester.updated_at = now
        return conflicts_by_requester

    def _rolling_prefetch_lead(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 5.0
        try:
            configured = float(fleet.get("rolling_prefetch_lead_sec", 5.0) or 5.0)
        except (TypeError, ValueError):
            configured = 5.0
        # The planner runs in wall time while route clocks run in simulation
        # time. At 4x, a fixed 8 simulation-second lead leaves only two real
        # seconds and synchronises the fleet at the rolling boundary. Scale
        # the lead to preserve approximately the same planner deadline.
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        configured *= time_scale
        horizon = self._rolling_horizon()
        upper = max(0.5, horizon * 0.8) if horizon > 0.0 else 5.0
        return max(0.5, min(upper, configured))

    def _rolling_prefetch_urgent_lead(self) -> float:
        # Protect an executing route before starting another robot. The old
        # two-second cap was shorter than the bounded local-CBS budget, so a
        # busy dispatch queue could postpone a continuation until the robot
        # had already stopped at its horizon boundary.
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        return max(
            1.0,
            min(8.0 * time_scale, self._rolling_prefetch_lead() * 0.4),
        )

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
                robot.current_lm = order.target_lm
                robot.target_lm = ""
                robot.status = "ARRIVED"
                robot.active_order_id = ""
                robot.last_reason = "order already at target"
                robot.updated_at = self._now()
                if self._advance_or_complete_order(order, robot, self._now()):
                    self._event("info", f"order completed: {order.order_id} {robot.name}@{order.target_lm}")
                    return True
                return self._dispatch_order(order, force=force)

            try:
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
            except ValueError as exc:
                if order.internal_kind != "traffic_clearance":
                    raise
                failed_reason = f"traffic clearance route invalid: {exc}"
                continue

            self._set_order_status(order, "PLANNING", robot=robot, start_lm=start_lm)
            result = self._plan_valid_requests([request], self._order_plan_payload(order, request))
            if result.get("ok") and result.get("plans"):
                now = self._now()
                plan = self._plan_for_robot(result, robot.name)
                remote_route: dict[str, Any] | None = None
                if (
                    plan is not None
                    and not self._plan_follows_requested_clearance_route(
                        order,
                        request,
                        plan,
                    )
                ):
                    failed_reason = (
                        "traffic clearance planner changed its fixed route"
                    )
                    continue
                if robot.is_remote():
                    if plan is None:
                        failed_reason = "planner did not return robot plan"
                        continue
                    try:
                        remote_route = self._execute_remote_plan(robot, order, plan, result)
                    except Exception as exc:
                        failed_reason = f"remote execute failed: {exc}"
                        robot.remote_error = str(exc)
                        control_conflict = self._is_remote_control_conflict(exc)
                        robot.remote_online = control_conflict
                        robot.status = "MANUAL" if control_conflict else "OFFLINE"
                        robot.last_reason = failed_reason
                        robot.updated_at = now
                        continue
                if not robot.is_remote():
                    result = self._rolling_result(
                        result,
                        {robot.name: order.target_lm},
                    )
                    plan = self._plan_for_robot(result, robot.name)
                    if plan is None:
                        failed_reason = "planner did not return rolling route chunk"
                        continue
                    if not self._plan_follows_requested_clearance_route(
                        order,
                        request,
                        plan,
                    ):
                        failed_reason = (
                            "traffic clearance rolling result changed its fixed route"
                        )
                        continue
                    if self._wait_only_rolling_plan(plan, order.target_lm):
                        detour_queued = (
                            self._order_stall_allows_detour(order)
                            and self._queue_alternate_corridor_detour(
                                order,
                                start_lm,
                                order.target_lm,
                            )
                        )
                        failed_reason = (
                            "traffic window has no progress; alternate corridor queued"
                            if detour_queued
                            else "traffic window has no progress; joint retry pending"
                        )
                        self._event(
                            "warn",
                            f"{robot.name} wait-only route rejected; order kept queued",
                        )
                        continue
                order.route_nodes = [
                    str(item)
                    for plan in result.get("plans", [])
                    if isinstance(plan, dict)
                    for item in plan.get("nodes", [])
                ]
                self._apply_planner_result(result, now, order_id=order.order_id)
                if remote_route is not None:
                    self._apply_remote_route_metadata(robot, remote_route, now)
                elif plan is not None:
                    self._apply_simulated_route_metadata(robot, order, plan, now)
                self._set_order_status(order, "EXECUTING", robot=robot, start_lm=start_lm)
                order.traffic_detour_edges = []
                self._event(
                    "info",
                    f"order dispatched: {order.order_id} {robot.name} {start_lm}->{order.target_lm}",
                )
                return True
            failed_reason = self._planner_failure_reason(result)
            debug = result.get("debug")
            if not isinstance(debug, dict):
                debug = {}
            if (
                debug.get("stationaryRobotWait")
                or "stationary_robot_blocks_route" in failed_reason
            ):
                # Remote orders bypass the simulated batch completion path.
                # Feed the same causal evidence into parked-body recovery so
                # a real robot does not retry forever through an idle peer.
                stationary_failure_debug = debug
            if self._planner_deadlock_result(result):
                robot.status = "WAITING"
                robot.last_reason = failed_reason
                robot.blocked_since = self._now()
                robot.updated_at = self._now()
            if order.vehicle:
                order.status = "QUEUED"
                order.assigned_robot = robot.name
                order.start_lm = start_lm
                order.updated_at = self._now()
            else:
                order.status = "QUEUED"
                order.start_lm = start_lm
                order.updated_at = self._now()

        self._set_order_error(order, failed_reason or "dispatch pending")
        if stationary_failure_debug is not None:
            self._record_stationary_order_failure(
                order,
                stationary_failure_debug,
            )
        return False

    def _order_plan_payload(self, order: FleetOrder, request: dict[str, Any]) -> dict[str, Any]:
        robot = self.robots.get(str(request.get("name") or ""))
        rolling_continuation = bool(robot is not None and robot.has_executed_route)
        recovery_group = int(order.dispatch_failures or 0) >= 2
        # Preparing the request transitions the order to PLANNING and clears
        # its visible error string. The occupancy record is the durable marker
        # that this is the same stationary retry.
        stationary_retry = (
            order.order_id in self._stationary_order_retry_state
        )
        traffic_detour = bool(order.traffic_detour_edges)
        payload: dict[str, Any] = {
            "robots": [request],
            # A 10 second rolling waypoint never needs the global 160 second
            # low-level search.  Bounding the background search prevents one
            # congested request from monopolising Python's GIL and freezing
            # the simulation clock.
            "lowLevelMaxTime": self._runtime_low_level_max_time(),
            # CBS coordinates a newly released group.  For a continuation the
            # other robots are fixed time reservations; CBS cannot move them,
            # so falling back from SIPP only burns several seconds.  Traffic
            # retries/deadlock priority leases handle that case on a fresh tick.
            # After repeated failures the dispatcher deliberately builds a
            # small recovery group, where CBS can coordinate the participants
            # instead of treating every neighbour as an immutable obstacle.
            "allowCbsFallback": (
                not rolling_continuation
                or recovery_group
                or stationary_retry
            ),
            # A robot without an executable trajectory is a physical obstacle,
            # not a temporal reservation that may disappear after the rolling
            # horizon. Its LM is excluded on the first attempt and a failed
            # detour is kept queued instead of falling back through the body.
            "skipSoftBlockedDetour": False,
            "strictStationaryRobotAvoidance": True,
            "reservedEdgeDetourEnabled": traffic_detour,
        }
        if traffic_detour:
            payload["blocked_edges"] = [
                {"from": src, "to": dst}
                for src, dst in order.traffic_detour_edges
            ]
        if order.speed > 0.0:
            payload["speed"] = order.speed
        if order.acceleration > 0.0:
            payload["acceleration"] = order.acceleration
        payload["rotate"] = bool(order.rotate)
        payload["stretchMotionToReservationTicks"] = bool(order.stretch_motion_to_reservation_ticks)
        if order.turn_speed > 0.0:
            payload["turnSpeed"] = order.turn_speed
        return payload

    def _runtime_low_level_max_time(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured_max = max(8, int(fleet.get("cbs_low_level_max_time", 160) or 160))
        except (TypeError, ValueError):
            configured_max = 160
        # The request may spend one reservation horizon waiting and then one
        # rolling horizon moving.  A small guard absorbs rounding and safety
        # margins while keeping the state space proportional to the window.
        window_sec = self._rolling_horizon() + self._reservation_horizon()
        guard_sec = max(2.0, self._reservation_safety_time() * 4.0)
        ticks = math.ceil((window_sec + guard_sec) / self._reservation_time_step())
        corridor_ticks = self.planner.controlled_corridor_max_ticks()
        if corridor_ticks:
            reservation_ticks = math.ceil(
                self._reservation_horizon() / self._reservation_time_step()
            )
            guard_ticks = math.ceil(guard_sec / self._reservation_time_step())
            ticks = max(ticks, reservation_ticks + corridor_ticks + guard_ticks)
        return max(8, min(configured_max, int(ticks)))

    def _candidate_robots_for_order(self, order: FleetOrder) -> list[FleetRobot]:
        if order.vehicle:
            robot = self.robots.get(order.vehicle)
            if robot is None:
                return []
            return [robot] if self._robot_can_accept_order(robot, explicit=True) else []

        candidates = [
            robot for robot in self._runtime_robots()
            if self._robot_can_accept_order(robot, explicit=False)
        ]
        candidates.sort(
            key=lambda robot: (
                self._lm_distance(self._nearest_lm_for_robot(robot), order.target_lm),
                robot.name,
            )
        )
        return candidates

    def _robot_can_accept_order(self, robot: FleetRobot, explicit: bool = False) -> bool:
        if not self._robot_enabled(robot):
            return False
        if robot.is_remote():
            self._sync_remote_robot(robot, self._now(), force=False)
            if not robot.remote_online:
                return False
            owner_id, _ = self._remote_control_owner(robot)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return False
            if robot.status in {"LOCALIZING", "OFFLINE", "ERROR"}:
                return False
        if robot.active_order_id:
            return False
        if robot.target_lm or robot.trajectory:
            return False
        if robot.status == "STOPPED" and not explicit:
            return False
        if robot.status == "MANUAL" and not explicit:
            return False
        if robot.status in {"MOVING", "WAITING", "PLANNING", "BLOCKED"}:
            return False
        return True

    def _lm_distance(self, start_lm: str, goal_lm: str) -> float:
        start = self.landmarks.get(start_lm)
        goal = self.landmarks.get(goal_lm)
        if start is None or goal is None:
            return float("inf")
        return math.hypot(goal.x - start.x, goal.y - start.y)

    def _set_order_status(
        self,
        order: FleetOrder,
        status: str,
        robot: FleetRobot | None = None,
        start_lm: str = "",
        error: str = "",
    ) -> None:
        order.status = status
        order.updated_at = self._now()
        order.error = error
        if status in {"EXECUTING", "COMPLETED"}:
            order.dispatch_failures = 0
            order.traffic_blocked_since = None
        if status not in {"QUEUED", "PLANNING"}:
            self._clear_stationary_order_retry_state(order.order_id)
        if robot is not None:
            order.assigned_robot = robot.name
            if not order.vehicle and status not in {"PLANNING", "QUEUED"}:
                order.vehicle = robot.name
        if start_lm:
            order.start_lm = start_lm

    def _set_order_error(self, order: FleetOrder, error: str) -> None:
        if order.error != error:
            self._event("warn", f"order pending: {order.order_id} {error}")
        order.status = "QUEUED"
        order.error = error
        order.dispatch_failures += 1
        error_value = str(error or "").lower()
        if "stationary_robot_blocks_route" not in error_value:
            self._clear_stationary_order_retry_state(order.order_id)
        traffic_failure = any(
            marker in error_value
            for marker in (
                "no_sipp_path",
                "reserved_",
                "traffic window",
                "deadlock",
                "continuous reservation conflict",
                "resource_conflict",
                "resource_constrained",
                "priority_cycle",
                "no_solution",
                "blocked edge",
            )
        )
        if traffic_failure:
            now = self._now()
            if order.traffic_blocked_since is None:
                order.traffic_blocked_since = now
            elif (
                order.internal_kind != "traffic_clearance"
                and now - order.traffic_blocked_since
                >= self._traffic_replan_after()
            ):
                self._queue_alternate_corridor_detour(
                    order,
                    order.start_lm,
                    self._active_order_target(order),
                )
                order.spatial_route_nodes = []
        if not order.vehicle:
            order.assigned_robot = ""
        order.updated_at = self._now()

    def _queue_alternate_corridor_detour(
        self,
        order: FleetOrder,
        start_lm: str,
        final_goal_lm: str,
        *,
        avoid_lm: str = "",
        replace_existing: bool = False,
    ) -> bool:
        """Exclude the next corridor only when the same goal stays reachable."""
        if order.internal_kind == "traffic_clearance":
            # The route is an authoritative outward evacuation selected by
            # _stationary_clearance_route().  A generic detour can cross the
            # corridor lease held by the robot this order is meant to free.
            return False
        if start_lm not in self.landmarks or final_goal_lm not in self.landmarks:
            return False
        # A detour is a one-chunk traffic decision, not permanent map editing.
        # Retry an existing exclusion after backoff instead of accumulating
        # enough exclusions to erode the graph of a long-running order.
        if order.traffic_detour_edges and not replace_existing:
            return False
        owner = str(order.vehicle or order.assigned_robot or "").strip()
        stationary_lms = self._stationary_robot_blocked_lms(
            exclude_robot_names={owner} if owner else set(),
        )
        stationary_edges = self._blocked_edges_for_lms(stationary_lms)
        if avoid_lm and (
            avoid_lm not in self.landmarks
            or avoid_lm in {start_lm, final_goal_lm}
        ):
            # A robot cannot route around its own source, and an occupied goal
            # needs ordinary wait/clearance rather than a misleading detour.
            return False
        avoid_edges = (
            self._blocked_edges_for_lms({avoid_lm})
            if avoid_lm
            else set()
        )
        route = None
        existing_nodes = (
            []
            if replace_existing
            else [
                str(node)
                for node in order.spatial_route_nodes
                if str(node) in self.landmarks
            ]
        )
        if start_lm in existing_nodes and existing_nodes[-1:] == [final_goal_lm]:
            suffix = existing_nodes[existing_nodes.index(start_lm):]
            if all(
                self.planner.route_planner.get_edge(src, dst) is not None
                for src, dst in zip(suffix, suffix[1:])
            ):
                route = self._planned_route_from_nodes(suffix)
        edge_penalties = (
            self._traffic_route_edge_penalties(order, start_lm, final_goal_lm)
            if self._congestion_routing_enabled()
            else None
        )
        if route is None:
            try:
                route = self.planner.route_planner.find_route(
                    start_lm,
                    final_goal_lm,
                    blocked_edges=stationary_edges,
                    edge_penalties=edge_penalties,
                )
            except ValueError:
                return False
        if len(route.nodes) < 2:
            return False
        if avoid_lm and avoid_lm not in route.nodes:
            # The whole-route selector has already avoided this stationary
            # body. During an explicit replacement, retiring the stale
            # one-chunk exclusion and cache is itself the required route
            # change; otherwise do not ban an unrelated first edge.
            if not replace_existing:
                return False
            order.traffic_detour_edges = []
            order.traffic_detour_attempts += 1
            order.spatial_route_nodes = []
            self._event(
                "warn",
                f"{order.vehicle or order.assigned_robot} stale traffic detour "
                f"retired; congestion route avoids {avoid_lm}",
            )
            return True

        src, dst = str(route.nodes[0]), str(route.nodes[1])
        candidate = (
            set(avoid_edges)
            if avoid_lm
            else {(src, dst), (dst, src)}
        )
        try:
            alternate = self.planner.route_planner.find_route(
                start_lm,
                final_goal_lm,
                blocked_edges=candidate | stationary_edges,
                edge_penalties=edge_penalties,
            )
        except ValueError:
            # A single-exit landmark must wait; banning its only corridor
            # would turn temporary congestion into a permanent no-path error.
            return False
        if alternate.nodes == route.nodes:
            return False

        order.traffic_detour_edges = sorted(candidate)
        order.traffic_detour_attempts += 1
        order.spatial_route_nodes = []
        self._event(
            "warn",
            f"{order.vehicle or order.assigned_robot} alternate corridor queued: "
            + (
                f"avoid occupied LM {avoid_lm}, keep goal {final_goal_lm}"
                if avoid_lm
                else f"avoid {src}<->{dst}, keep goal {final_goal_lm}"
            ),
        )
        return True

    def _order_stall_allows_detour(self, order: FleetOrder) -> bool:
        now = self._now()
        if order.traffic_blocked_since is None:
            order.traffic_blocked_since = now
            return False
        return now - order.traffic_blocked_since >= self._traffic_replan_after()

    def _cancel_order(self, order: FleetOrder, reason: str) -> None:
        for robot in self._runtime_robots():
            if robot.active_order_id == order.order_id:
                self._stop_robot(robot, cancel_active_order=False)
        self._set_order_status(order, "CANCELED", error=reason)
        self._event("warn", f"order canceled: {order.order_id}")

    def _pause_order(self, order: FleetOrder, reason: str) -> None:
        paused_robot: FleetRobot | None = None
        for robot in self._runtime_robots():
            if robot.active_order_id == order.order_id:
                paused_robot = robot
                break
        if paused_robot is None and order.assigned_robot:
            paused_robot = self.robots.get(order.assigned_robot)
        if paused_robot is None and order.vehicle:
            paused_robot = self.robots.get(order.vehicle)

        if paused_robot is not None:
            self._cancel_remote_route(paused_robot, reason)
            nearest_lm = self._nearest_lm_for_robot(paused_robot)
            if nearest_lm in self.landmarks:
                paused_robot.current_lm = nearest_lm
            paused_robot.target_lm = ""
            paused_robot.status = "PAUSED"
            paused_robot.trajectory = []
            paused_robot.plan_nodes = []
            paused_robot.trajectory_dirty = True
            paused_robot.route_started_at = None
            paused_robot.route_clock = 0.0
            paused_robot.last_tick_at = None
            paused_robot.blocked_since = None
            paused_robot.last_replan_at = None
            paused_robot.last_reason = reason
            paused_robot.route_note = ""
            paused_robot.active_order_id = ""
            self._clear_remote_route_metadata(paused_robot)
            paused_robot.updated_at = self._now()
            order.assigned_robot = paused_robot.name
            if not order.vehicle:
                order.vehicle = paused_robot.name
            order.start_lm = paused_robot.current_lm

        order.status = "PAUSED"
        order.error = reason
        order.updated_at = self._now()
        self._event("warn", f"order paused: {order.order_id}")

    def _cancel_active_order_for_robot(self, robot: FleetRobot, reason: str) -> None:
        if not robot.active_order_id:
            return
        self._cancel_remote_route(robot, reason)
        order = self.orders.get(robot.active_order_id)
        if order is not None and order.status not in TERMINAL_ORDER_STATUSES:
            self._set_order_status(order, "CANCELED", error=reason)
            self._event("warn", f"order canceled: {order.order_id} {reason}")
        robot.active_order_id = ""

    def _cancel_orders_for_robot(self, robot_name: str, reason: str) -> None:
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.vehicle == robot_name or order.assigned_robot == robot_name:
                self._set_order_status(order, "CANCELED", error=reason)
                self._event("warn", f"order canceled: {order.order_id} {reason}")

    def _replace_orders_for_robot(self, robot_name: str, reason: str) -> None:
        robot = self.robots.get(robot_name)
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.vehicle == robot_name or order.assigned_robot == robot_name:
                self._set_order_status(order, "CANCELED", error=reason)
                self._event("warn", f"order canceled: {order.order_id} {reason}")

        if robot is None:
            return
        self._cancel_remote_route(robot, reason)
        nearest_lm = self._nearest_lm_for_robot(robot)
        if nearest_lm in self.landmarks:
            robot.current_lm = nearest_lm
        robot.target_lm = ""
        robot.status = "IDLE"
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_started_at = None
        robot.route_clock = 0.0
        robot.last_tick_at = None
        robot.blocked_since = None
        robot.last_replan_at = None
        robot.last_reason = reason
        robot.route_note = ""
        robot.active_order_id = ""
        self._clear_remote_route_metadata(robot)
        robot.updated_at = self._now()

    def _cancel_all_orders(self, reason: str) -> None:
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            self._set_order_status(order, "CANCELED", error=reason)
            self._event("warn", f"order canceled: {order.order_id} {reason}")

    def _active_order_target(self, order: FleetOrder) -> str:
        targets = order.targets or ([order.target_lm] if order.target_lm else [])
        if not targets:
            return order.target_lm
        order.step_index = max(0, min(order.step_index, len(targets) - 1))
        order.target_lm = targets[order.step_index]
        return order.target_lm

    def _advance_or_complete_order(self, order: FleetOrder, robot: FleetRobot, now: float) -> bool:
        # A reached step invalidates any parked-blocker signature collected
        # while trying to depart toward the previous target.
        self._clear_stationary_order_retry_state(order.order_id)
        targets = order.targets or ([order.target_lm] if order.target_lm else [])
        if order.step_index + 1 >= len(targets):
            order.status = "COMPLETED"
            order.error = ""
            order.updated_at = now
            order.assigned_robot = robot.name
            order.route_nodes = list(robot.plan_nodes)
            robot.active_order_id = ""
            return True

        previous_target = targets[order.step_index]
        order.step_index += 1
        order.target_lm = targets[order.step_index]
        order.status = "QUEUED"
        order.error = ""
        order.updated_at = now
        order.assigned_robot = robot.name
        order.start_lm = robot.current_lm
        order.route_nodes = []
        order.spatial_route_nodes = []
        order.spatial_route_revision = 0
        order.traffic_blocked_since = None
        robot.active_order_id = ""
        self._event(
            "info",
            f"order step completed: {order.order_id} {robot.name}@{previous_target}; next {order.target_lm}",
        )
        return False

    def _complete_active_order(self, robot: FleetRobot, now: float) -> None:
        if not robot.active_order_id:
            return
        order = self.orders.get(robot.active_order_id)
        if order is None:
            robot.active_order_id = ""
            return
        order.route_nodes = list(robot.plan_nodes)
        self._advance_or_complete_order(order, robot, now)
