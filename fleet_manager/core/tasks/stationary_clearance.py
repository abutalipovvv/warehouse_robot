"""Manage bounded stationary-body clearance relocations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from fleet_manager.core.fleet.domain.constants import (
    FLEET_CONTROL_OWNER_ID,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.fleet.domain.models import FleetOrder, FleetRobot


ClearanceWaiterSignature = tuple[str, str, str, str]


@dataclass(frozen=True, slots=True)
class _ClearanceEpisode:
    """Stable inputs for one bounded parked-body relocation attempt."""

    waiter_order: FleetOrder
    waiter_signature: ClearanceWaiterSignature
    now: float
    forbidden_lms: set[str]


class StationaryClearanceMixin:
    """Manage bounded stationary-body clearance relocations."""

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
        waiter_order = self._stationary_clearance_waiter_order(waiter, blocker)
        if waiter_order is None:
            return False
        episode = self._stationary_clearance_episode(
            waiter,
            blocker,
            waiter_order,
        )
        if episode is None:
            return False
        route_nodes = self._select_stationary_clearance_route(
            waiter,
            blocker,
            episode.forbidden_lms,
        )
        if len(route_nodes) < 2:
            return False
        start_lm = str(route_nodes[0])
        target_lm = str(route_nodes[-1])
        order_id = self._stationary_clearance_order_id(blocker, episode.now)
        relocation = self._build_stationary_clearance_order(
            order_id,
            blocker,
            episode.waiter_order,
            target_lm,
            route_nodes,
            episode.now,
        )
        self.orders[order_id] = relocation
        self._record_stationary_clearance_episode(
            waiter,
            blocker,
            episode,
            order_id,
            start_lm,
            target_lm,
        )
        self._event(
            "warn",
            f"traffic clearance move queued: {blocker.name} "
            f"{start_lm}->{target_lm}; releases {waiter.name} ({cause})",
        )
        return True

    def _stationary_clearance_waiter_order(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
    ) -> FleetOrder | None:
        """Validate both robots and return the causal active waiter order."""
        if (
            not self._stationary_clearance_enabled()
            or waiter.name == blocker.name
            or blocker.status not in {"IDLE", "ARRIVED"}
            or blocker.trajectory
            or blocker.active_order_id
            or blocker.target_lm
            or not blocker.remote_online
        ):
            return None
        pending = self._active_order_for_robot(blocker)
        if pending is not None and pending.status not in TERMINAL_ORDER_STATUSES:
            return None
        if blocker.is_remote():
            owner_id, _ = self._remote_control_owner(blocker)
            if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
                return None
        waiter_order = self._active_order_for_robot(waiter)
        if (
            waiter_order is None
            or waiter_order.status in TERMINAL_ORDER_STATUSES
            or waiter_order.internal_kind
        ):
            return None
        return waiter_order

    def _stationary_clearance_episode(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        waiter_order: FleetOrder,
    ) -> _ClearanceEpisode | None:
        """Restore cooldown and visited-pocket state for this causal waiter."""
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
                return None
            if now < float(current_state.get("cooldown_until", 0.0) or 0.0):
                return None
            if tuple(current_state.get("waiter_signature", ())) == waiter_signature:
                forbidden_lms.update(
                    str(lm_name)
                    for lm_name in current_state.get("visited_lms", ())
                    if str(lm_name) in self.landmarks
                )
        return _ClearanceEpisode(
            waiter_order=waiter_order,
            waiter_signature=waiter_signature,
            now=now,
            forbidden_lms=forbidden_lms,
        )

    def _select_stationary_clearance_route(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        forbidden_lms: set[str],
    ) -> list[str]:
        """Prefer an external pocket without acquiring a corridor token."""
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
        return route_nodes

    def _stationary_clearance_order_id(
        self,
        blocker: FleetRobot,
        now: float,
    ) -> str:
        """Allocate a deterministic, collision-free maintenance order id."""
        order_id = f"traffic-clearance-{blocker.name}-{int(now * 1000)}"
        suffix = 1
        while order_id in self.orders:
            suffix += 1
            order_id = (
                f"traffic-clearance-{blocker.name}-{int(now * 1000)}-{suffix}"
            )
        return order_id

    def _build_stationary_clearance_order(
        self,
        order_id: str,
        blocker: FleetRobot,
        waiter_order: FleetOrder,
        target_lm: str,
        route_nodes: list[str],
        now: float,
    ) -> FleetOrder:
        """Create the internal order that executes through ordinary MAPF."""
        return FleetOrder(
            order_id=order_id,
            target_lm=target_lm,
            vehicle=blocker.name,
            priority=max(10_000, int(waiter_order.priority or 0) + 1),
            created_at=now,
            updated_at=now,
            targets=[target_lm],
            speed=float(waiter_order.speed or 0.0),
            acceleration=float(waiter_order.acceleration or 0.0),
            rotate=bool(waiter_order.rotate),
            turn_speed=float(waiter_order.turn_speed or 0.0),
            stretch_motion_to_reservation_ticks=True,
            spatial_route_nodes=list(route_nodes),
            spatial_route_revision=self._next_route_revision(),
            internal_kind="traffic_clearance",
        )

    def _record_stationary_clearance_episode(
        self,
        waiter: FleetRobot,
        blocker: FleetRobot,
        episode: _ClearanceEpisode,
        order_id: str,
        start_lm: str,
        target_lm: str,
    ) -> None:
        """Persist bounded retry state only after the order is installed."""
        visited_lms = set(episode.forbidden_lms)
        visited_lms.update({start_lm, target_lm})
        self._stationary_clearance_relocations[blocker.name] = {
            "order_id": order_id,
            "waiter": waiter.name,
            "waiter_signature": episode.waiter_signature,
            "origin_lm": start_lm,
            "target_lm": target_lm,
            "visited_lms": tuple(sorted(visited_lms)),
            "queued_at": episode.now,
            "cooldown_until": 0.0,
            "cooldown_sec": self._stationary_clearance_cooldown(),
        }

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
