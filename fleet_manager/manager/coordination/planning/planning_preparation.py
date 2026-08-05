"""Preparation and orchestration of traffic-planning requests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from fleet_manager.manager.planning import PlanCandidate, PlanningSnapshot


@dataclass(frozen=True, slots=True)
class _TrafficPlanningContext:
    """Prepared reservations and ownership for one planning transaction."""

    reservation_offset: float
    hard_blocked_lms: set[str]
    held_snapshot_owners: set[str]
    release_owners: set[str]
    planner_payload: dict[str, Any]


class TrafficPlanPreparationMixin:
    """Build planner inputs and choose stationary-obstacle fallbacks."""

    def _planning_snapshot_for(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> PlanningSnapshot:
        """Freeze exactly the live data needed by one solver job."""

        clean_payload = payload or {}
        context = self._traffic_planning_context(
            valid_requests,
            clean_payload,
        )
        soft_blocked_lms = (
            set()
            if bool(clean_payload.get("skipSoftBlockedDetour", False))
            else self._soft_blocked_lms(
                valid_requests,
                context.hard_blocked_lms,
            )
        )
        primary_payload = {
            **context.planner_payload,
            "blocked_lms": sorted(
                context.hard_blocked_lms | soft_blocked_lms
            ),
        }
        fallback_payload = (
            {
                **context.planner_payload,
                "blocked_lms": sorted(context.hard_blocked_lms),
            }
            if soft_blocked_lms
            else None
        )
        return self._planning_snapshot_factory.create(
            created_at=self._now(),
            requests=valid_requests,
            primary_payload=primary_payload,
            fallback_payload=fallback_payload,
            blockers=context.hard_blocked_lms | soft_blocked_lms,
            soft_blocked_lms=soft_blocked_lms,
            strict_stationary_avoidance=bool(
                clean_payload.get("strictStationaryRobotAvoidance", True)
            ),
            reservation_offset=context.reservation_offset,
            held_snapshot_owners=context.held_snapshot_owners,
            release_owners=context.release_owners,
            graph_revision=f"{len(self.landmarks)}:{len(self.edges)}",
            map_revision=str(getattr(self, "map_dir", "") or "") or None,
        )

    def _finalize_planning_candidate(
        self,
        snapshot: PlanningSnapshot,
        candidate: PlanCandidate,
    ) -> dict[str, Any]:
        """Apply runtime-only validation and policy to solver output."""

        result = candidate.result.to_dict()
        context = _TrafficPlanningContext(
            reservation_offset=snapshot.reservation_offset,
            hard_blocked_lms=set(snapshot.blockers) - set(snapshot.soft_blocked_lms),
            held_snapshot_owners=set(snapshot.held_snapshot_owners),
            release_owners=set(snapshot.release_owners),
            planner_payload=snapshot.primary_payload_dict(),
        )
        soft_blocked_lms = set(snapshot.soft_blocked_lms)
        if not soft_blocked_lms:
            if result.get("ok"):
                return self._apply_planning_continuous_waits(result, context)
            return result

        if result.get("ok"):
            debug = result.setdefault("debug", {})
            debug["reason"] = (
                f"{debug.get('reason', 'success')}:detour_soft_blocks"
            )
            debug["softBlockedLms"] = sorted(soft_blocked_lms)
            result = self._apply_planning_continuous_waits(result, context)
            self._event(
                "info",
                "planner detour around occupied LM(s): "
                + ", ".join(sorted(soft_blocked_lms)),
            )
            return result

        failed_reason = result.get("debug", {}).get("reason", "unknown")
        fallback = candidate.metadata.get("fallbackResult")
        fallback_result = fallback if isinstance(fallback, dict) else {}
        if snapshot.strict_stationary_avoidance:
            return self._finalize_strict_snapshot_fallback(
                snapshot,
                result,
                fallback_result,
                context,
                soft_blocked_lms,
                failed_reason=failed_reason,
            )
        if fallback_result.get("ok"):
            debug = fallback_result.setdefault("debug", {})
            debug["reason"] = (
                f"{debug.get('reason', 'success')}:fallback_wait"
            )
            debug["softBlockedLms"] = sorted(soft_blocked_lms)
            debug["softBlockFailure"] = failed_reason
            fallback_result = self._apply_planning_continuous_waits(
                fallback_result,
                context,
            )
            self._event(
                "warn",
                "planner found no detour; using original route and "
                "runtime waiting",
            )
        return fallback_result or result

    def _finalize_strict_snapshot_fallback(
        self,
        snapshot: PlanningSnapshot,
        result: dict[str, Any],
        diagnostic: dict[str, Any],
        context: _TrafficPlanningContext,
        soft_blocked_lms: set[str],
        *,
        failed_reason: Any,
    ) -> dict[str, Any]:
        """Classify a failed immutable soft-block planning attempt."""

        debug = result.setdefault("debug", {})
        debug["softBlockedLms"] = sorted(soft_blocked_lms)
        debug["softBlockDetourFailure"] = failed_reason
        request_names = {
            str(request.get("name") or "")
            for request in snapshot.request_dicts()
        }
        blocker_names: list[str] = []
        if diagnostic.get("ok"):
            diagnostic_debug = diagnostic.setdefault("debug", {})
            diagnostic_debug["reason"] = (
                f"{diagnostic_debug.get('reason', 'success')}:"
                "diagnostic_soft_fallback"
            )
            diagnostic_debug["softBlockedLms"] = sorted(soft_blocked_lms)
            diagnostic_soft_lms = {
                str(node)
                for plan in diagnostic.get("plans", [])
                if isinstance(plan, dict)
                for node in plan.get("nodes", [])
                if str(node) in soft_blocked_lms
            }
            blocker_names = self._stationary_blockers_named_by_failure(
                " ".join(sorted(diagnostic_soft_lms)),
                soft_blocked_lms,
                request_names=request_names,
            )
            if not blocker_names:
                diagnostic = self._apply_planning_continuous_waits(
                    diagnostic,
                    context,
                )
                if (
                    diagnostic.get("ok")
                    or diagnostic.get("debug", {}).get(
                        "stationaryBlockerRobots"
                    )
                ):
                    return diagnostic
        diagnostic_reason = str(
            diagnostic.get("debug", {}).get("reason", "")
            if isinstance(diagnostic, dict)
            else ""
        )
        if not blocker_names:
            blocker_names = self._stationary_blockers_named_by_failure(
                diagnostic_reason,
                soft_blocked_lms,
                request_names=request_names,
            )
        if blocker_names:
            debug["reason"] = (
                f"{failed_reason}:stationary_robot_blocks_route"
            )
            debug["stationaryRobotWait"] = True
            debug["stationaryBlockerRobots"] = blocker_names
            self._event(
                "warn",
                "planner found no route around proven stationary "
                "robot(s); order remains queued",
            )
        else:
            debug["reason"] = failed_reason
            debug["temporalResourceFailure"] = True
        return result

    def _plan_valid_requests_unlocked(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        payload = payload or {}
        context = self._traffic_planning_context(
            valid_requests,
            payload,
        )
        soft_blocked_lms = (
            set()
            if bool(payload.get("skipSoftBlockedDetour", False))
            else self._soft_blocked_lms(
                valid_requests,
                context.hard_blocked_lms,
            )
        )
        if soft_blocked_lms:
            return self._plan_with_soft_block_detour(
                valid_requests,
                payload,
                context,
                soft_blocked_lms,
            )
        result = self.planner.plan(
            {
                **context.planner_payload,
                "blocked_lms": sorted(context.hard_blocked_lms),
            }
        )
        if result.get("ok"):
            result = self._apply_planning_continuous_waits(
                result,
                context,
            )
        return result

    def _traffic_planning_context(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any],
    ) -> _TrafficPlanningContext:
        """Normalize reservations and ownership before invoking MAPF."""

        try:
            reservation_offset = max(
                0.0,
                float(payload.get("reservationOffsetSec", 0.0) or 0.0),
            )
        except (TypeError, ValueError):
            reservation_offset = 0.0
        hard_blocked_lms = self._hard_blocked_lms(payload)
        blocked_edges = (
            self._hard_blocked_edges(payload)
            | self._dynamic_blocked_edges()
        )
        held_blockers = self._release_blocker_names_for_requests(
            valid_requests
        )
        superseded_holders = (
            self._superseded_runtime_replan_holder_names(valid_requests)
        )
        held_snapshot_owners = held_blockers | superseded_holders
        release_owners = self._bootstrap_departure_robot_names(valid_requests)
        reservation_ignored_owners = (
            held_snapshot_owners
            | release_owners
        )
        release_start_lms = {
            str(request.get("startLm", "")).strip()
            for request in valid_requests
            if str(request.get("startLm", "")).strip() in self.landmarks
        }
        reserved_edge_intervals = self._reserved_edge_intervals(
            valid_requests,
            ignore_robot_names=reservation_ignored_owners,
            prediction_offset=reservation_offset,
        )
        reserved_vertex_intervals = self._reserved_vertex_intervals(
            valid_requests,
            ignore_robot_names=reservation_ignored_owners,
            ignore_nodes=release_start_lms,
            prediction_offset=reservation_offset,
        )
        reserved_vertex_intervals.extend(
            self._held_blocker_vertex_intervals(
                held_snapshot_owners,
                prediction_offset=reservation_offset,
            )
        )
        planner_payload = {
            **payload,
            "robots": valid_requests,
            "blocked_edges": [
                {"from": src, "to": dst}
                for src, dst in sorted(blocked_edges)
            ],
            "reserved_vertex_constraints": [],
            "reserved_edge_constraints": [],
            "reserved_vertex_intervals": [
                {
                    "node": node,
                    "start": start,
                    "end": end,
                    "robot": robot_name,
                }
                for node, start, end, robot_name in reserved_vertex_intervals
            ],
            "reserved_edge_intervals": [
                {
                    "from": src,
                    "to": dst,
                    "start": start,
                    "end": end,
                    "robot": robot_name,
                }
                for src, dst, start, end, robot_name in reserved_edge_intervals
            ],
        }
        return _TrafficPlanningContext(
            reservation_offset=reservation_offset,
            hard_blocked_lms=hard_blocked_lms,
            held_snapshot_owners=held_snapshot_owners,
            release_owners=release_owners,
            planner_payload=planner_payload,
        )

    def _plan_with_soft_block_detour(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any],
        context: _TrafficPlanningContext,
        soft_blocked_lms: set[str],
    ) -> dict[str, Any]:
        result = self.planner.plan(
            {
                **context.planner_payload,
                "blocked_lms": sorted(
                    context.hard_blocked_lms | soft_blocked_lms
                ),
            }
        )
        if result.get("ok"):
            debug = result.setdefault("debug", {})
            debug["reason"] = (
                f"{debug.get('reason', 'success')}:detour_soft_blocks"
            )
            debug["softBlockedLms"] = sorted(soft_blocked_lms)
            result = self._apply_planning_continuous_waits(result, context)
            self._event(
                "info",
                "planner detour around occupied LM(s): "
                + ", ".join(sorted(soft_blocked_lms)),
            )
            return result

        failed_reason = result.get("debug", {}).get("reason", "unknown")
        if bool(payload.get("strictStationaryRobotAvoidance", True)):
            return self._strict_soft_block_failure(
                valid_requests,
                result,
                context,
                soft_blocked_lms,
                failed_reason=failed_reason,
            )
        return self._fallback_after_soft_block_failure(
            context,
            soft_blocked_lms,
            failed_reason=failed_reason,
        )

    def _strict_soft_block_failure(
        self,
        valid_requests: list[dict[str, Any]],
        result: dict[str, Any],
        context: _TrafficPlanningContext,
        soft_blocked_lms: set[str],
        *,
        failed_reason: Any,
    ) -> dict[str, Any]:
        """Distinguish a parked-body cut from an unrelated temporal failure."""

        debug = result.setdefault("debug", {})
        debug["softBlockedLms"] = sorted(soft_blocked_lms)
        debug["softBlockDetourFailure"] = failed_reason
        diagnostic = self.planner.plan(
            {
                **context.planner_payload,
                "blocked_lms": sorted(context.hard_blocked_lms),
            }
        )
        request_names = {
            str(request.get("name") or "")
            for request in valid_requests
        }
        blocker_names: list[str] = []
        if diagnostic.get("ok"):
            diagnostic_debug = diagnostic.setdefault("debug", {})
            diagnostic_debug["reason"] = (
                f"{diagnostic_debug.get('reason', 'success')}:"
                "diagnostic_soft_fallback"
            )
            diagnostic_debug["softBlockedLms"] = sorted(soft_blocked_lms)
            diagnostic_soft_lms = {
                str(node)
                for plan in diagnostic.get("plans", [])
                if isinstance(plan, dict)
                for node in plan.get("nodes", [])
                if str(node) in soft_blocked_lms
            }
            blocker_names = self._stationary_blockers_named_by_failure(
                " ".join(sorted(diagnostic_soft_lms)),
                soft_blocked_lms,
                request_names=request_names,
            )
            if not blocker_names:
                diagnostic = self._apply_planning_continuous_waits(
                    diagnostic,
                    context,
                )
                if (
                    diagnostic.get("ok")
                    or diagnostic.get("debug", {}).get(
                        "stationaryBlockerRobots"
                    )
                ):
                    return diagnostic
        diagnostic_reason = str(
            diagnostic.get("debug", {}).get("reason", "")
            if isinstance(diagnostic, dict)
            else ""
        )
        if not blocker_names:
            blocker_names = self._stationary_blockers_named_by_failure(
                diagnostic_reason,
                soft_blocked_lms,
                request_names=request_names,
            )
        if blocker_names:
            debug["reason"] = (
                f"{failed_reason}:stationary_robot_blocks_route"
            )
            debug["stationaryRobotWait"] = True
            debug["stationaryBlockerRobots"] = blocker_names
            self._event(
                "warn",
                "planner found no route around proven stationary "
                "robot(s); order remains queued",
            )
        else:
            debug["reason"] = failed_reason
            debug["temporalResourceFailure"] = True
        return result

    def _fallback_after_soft_block_failure(
        self,
        context: _TrafficPlanningContext,
        soft_blocked_lms: set[str],
        *,
        failed_reason: Any,
    ) -> dict[str, Any]:
        result = self.planner.plan(
            {
                **context.planner_payload,
                "blocked_lms": sorted(context.hard_blocked_lms),
            }
        )
        if result.get("ok"):
            debug = result.setdefault("debug", {})
            debug["reason"] = (
                f"{debug.get('reason', 'success')}:fallback_wait"
            )
            debug["softBlockedLms"] = sorted(soft_blocked_lms)
            debug["softBlockFailure"] = failed_reason
            result = self._apply_planning_continuous_waits(result, context)
            self._event(
                "warn",
                "planner found no detour; using original route and "
                "runtime waiting",
            )
        return result

    def _apply_planning_continuous_waits(
        self,
        result: dict[str, Any],
        context: _TrafficPlanningContext,
    ) -> dict[str, Any]:
        return self._apply_continuous_reservation_waits(
            result,
            ignore_robot_names=context.release_owners,
            stationary_robot_names=context.held_snapshot_owners,
            prediction_offset=context.reservation_offset,
        )

    def _superseded_runtime_replan_holder_names(
        self,
        requests: list[dict[str, Any]],
    ) -> set[str]:
        """Return held robots whose old futures are no longer executable.

        A post-evacuation transaction retains its previous trajectory only as
        an atomic rollback/collision snapshot. Advertising that suffix to
        another planner as future motion creates false capacity and circular
        reservations. Keep the current body for the full horizon instead.
        """
        request_names = {
            str(request.get("name") or "")
            for request in requests
            if str(request.get("name") or "")
        }
        holders: set[str] = set()
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.status != "WAITING":
                continue
            state = self._runtime_replans.get(robot.name)
            if (
                not isinstance(state, dict)
                or not bool(state.get("retained_route_superseded"))
                or not self._runtime_replan_state_is_current(
                    robot,
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
            holders.add(robot.name)
        return holders

    def _held_blocker_vertex_intervals(
        self,
        robot_names: set[str],
        *,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, float, float, str]]:
        """Reserve upstream waiters as stopped bodies, not vanished routes.

        A request that releases the terminal member of a wait chain must not
        reserve an upstream waiter's old *future* trajectory: that trajectory
        is intentionally held until the terminal moves. Removing the waiter
        from every reservation is also unsafe, however, because a local CBS
        route may then cross its current footprint and conflict as soon as the
        old route resumes. Keep the current graph LM occupied for the complete
        temporal horizon while omitting only the stale future edges.
        """
        if not robot_names:
            return []
        horizon = self._reservation_horizon()
        intervals: list[tuple[str, float, float, str]] = []
        for name in sorted(robot_names):
            robot = self.robots.get(name)
            if robot is None:
                continue
            future_pose = self._predicted_robot_pose(
                robot,
                max(0.0, prediction_offset),
            )
            lm_name = (
                self._nearest_lm_for_pose(future_pose)
                if future_pose is not None
                else self._nearest_lm_for_robot(robot)
            )
            if lm_name in self.landmarks:
                intervals.append((lm_name, 0.0, horizon, robot.name))
        return intervals

    def _bootstrap_departure_robot_names(
        self,
        requests: list[dict[str, Any]],
    ) -> set[str]:
        """Temporarily release stationary robots that are queued to depart.

        Benchmark fleets are dispatched by a serialized worker, so most fresh
        robots—and later, robots between two orders—do not yet have a committed
        timeline while another departure is planned. Treating commanded
        departures as infinite reservations makes a dense fleet impossible to
        restart. Runtime footprint checks remain authoritative until each
        departure is committed. STOPPED robots and robots without a pending
        assignment remain persistent obstacles.
        """
        request_names = {
            str(request.get("name") or "").strip()
            for request in requests
        }
        released: set[str] = set()
        for robot in self._runtime_robots():
            if (
                robot.name in request_names
                or robot.trajectory
                or robot.status not in {"IDLE", "ARRIVED"}
            ):
                continue
            order = self._active_order_for_robot(robot)
            if (
                order is not None
                and order.status in {"QUEUED", "PLANNING"}
            ):
                released.add(robot.name)
        return released
