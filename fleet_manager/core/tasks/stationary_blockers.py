"""Detect stationary blockers and stage vacancy recovery."""

from __future__ import annotations

from heapq import heappop, heappush
import math
from typing import Any

from fleet_manager.core.constants import (
    FLEET_CONTROL_OWNER_ID,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.models import FleetOrder, FleetRobot


CommandedVacancyCandidate = tuple[
    tuple[float, int, str, str],
    FleetRobot,
    FleetOrder,
    FleetRobot,
    list[str],
    tuple[tuple[str, str, int], ...],
]


class StationaryBlockerRecoveryMixin:
    """Detect stationary blockers and stage vacancy recovery."""

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
        """Open a safe pocket when a queued departure is boxed by its waiters."""
        if self._async_simulated_dispatch_active():
            return False

        candidates, live_episode_sinks = (
            self._commanded_sink_vacancy_candidates()
        )
        self._prune_commanded_sink_vacancy_episodes(
            live_episode_sinks
        )
        if not candidates:
            return False
        candidate = min(candidates, key=lambda item: item[0])
        return self._commit_commanded_sink_vacancy(candidate, now)

    def _commanded_sink_vacancy_candidates(
        self,
    ) -> tuple[list[CommandedVacancyCandidate], set[str]]:
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
        return candidates, live_episode_sinks

    def _prune_commanded_sink_vacancy_episodes(
        self,
        live_episode_sinks: set[str],
    ) -> None:
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

    def _commit_commanded_sink_vacancy(
        self,
        candidate: CommandedVacancyCandidate,
        now: float,
    ) -> bool:
        _, sink, _, waiter, escape, signature = candidate
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

    def _queue_controlled_corridor_exit_clearance(
        self,
        waiter: FleetRobot,
    ) -> bool:
        """Move an inactive body which keeps a corridor slot unavailable.

        The calendar rejects an unsafe passage before MAPF is invoked.  That
        is normally correct, but it also meant the ordinary stationary-body
        recovery never saw a planner failure and could not clear a parked
        robot from the exit pocket.  Bridge the calendar's exact downstream
        blocker evidence into the same bounded maintenance-order mechanism.
        """
        blocker_name = str(
            self._controlled_corridor_blockers.get(waiter.name, "")
            or ""
        ).strip()
        blocker = self.robots.get(blocker_name)
        if blocker is None or blocker.name == waiter.name:
            return False
        queued = self._queue_stationary_clearance_relocation(
            waiter,
            blocker,
            cause="controlled corridor exit occupied",
        )
        if not queued:
            return False
        now = self._now()
        self._rolling_prefetch_last_attempt_at[waiter.name] = now
        self._rolling_prefetch_retry_at[waiter.name] = max(
            self._rolling_prefetch_retry_at.get(waiter.name, 0.0),
            now + self._rolling_boundary_retry_interval(
                self._active_order_for_robot(waiter),
            ),
        )
        if self._robot_waits_at_rolling_boundary(waiter):
            waiter.last_reason = (
                f"waiting for corridor exit clearance by {blocker.name}"
            )
            waiter.updated_at = now
        return True


__all__ = ["StationaryBlockerRecoveryMixin"]
