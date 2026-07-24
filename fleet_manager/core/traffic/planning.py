"""MAPF invocation, reservations and continuous trajectory scheduling."""

from __future__ import annotations

from collections.abc import Callable
import math
from time import time
from typing import Any

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.route_core.models import PlannedRoute


class TrafficPlanningMixin:
    """Shared space-time planning policy for all fleet transports."""

    def _plan_valid_requests(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        # FleetMapfPlanner keeps reusable graph/planner objects.  Dynamic
        # dispatch runs in one background thread, while explicit operator
        # requests may still arrive from the HTTP server, so serialize planner
        # use without serializing runtime ticks and rendering.
        with self._planner_lock:
            return self._plan_valid_requests_unlocked(valid_requests, payload)

    def _plan_valid_requests_unlocked(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        payload = payload or {}
        try:
            reservation_offset = max(0.0, float(payload.get("reservationOffsetSec", 0.0) or 0.0))
        except (TypeError, ValueError):
            reservation_offset = 0.0
        hard_blocked_lms = self._hard_blocked_lms(payload)
        blocked_edges = self._hard_blocked_edges(payload) | self._dynamic_blocked_edges()
        held_blockers = self._release_blocker_names_for_requests(valid_requests)
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
        soft_blocked_lms = (
            set()
            if bool(payload.get("skipSoftBlockedDetour", False))
            else self._soft_blocked_lms(valid_requests, hard_blocked_lms)
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

        if soft_blocked_lms:
            result = self.planner.plan(
                {
                    **planner_payload,
                    "blocked_lms": sorted(hard_blocked_lms | soft_blocked_lms),
                }
            )
            if result.get("ok"):
                debug = result.setdefault("debug", {})
                debug["reason"] = f"{debug.get('reason', 'success')}:detour_soft_blocks"
                debug["softBlockedLms"] = sorted(soft_blocked_lms)
                result = self._apply_continuous_reservation_waits(
                    result,
                    ignore_robot_names=release_owners,
                    stationary_robot_names=held_snapshot_owners,
                    prediction_offset=reservation_offset,
                )
                self._event(
                    "info",
                    f"planner detour around occupied LM(s): {', '.join(sorted(soft_blocked_lms))}",
                )
                return result

            failed_reason = result.get("debug", {}).get("reason", "unknown")
            if bool(payload.get("strictStationaryRobotAvoidance", True)):
                debug = result.setdefault("debug", {})
                debug["softBlockedLms"] = sorted(soft_blocked_lms)
                debug["softBlockDetourFailure"] = failed_reason

                # A soft-block detour can fail on an unrelated temporal SIPP
                # resource. Merely having parked robots elsewhere on the map
                # is not evidence that one of them caused that failure. Run a
                # no-soft diagnostic under the same exact reservations. A
                # successful, continuously validated result is safe to use;
                # a failure is classified as stationary only when its named
                # resource is one of the occupied LMs (or continuous
                # validation supplies an explicit blocker identity).
                diagnostic = self.planner.plan(
                    {
                        **planner_payload,
                        "blocked_lms": sorted(hard_blocked_lms),
                    }
                )
                blocker_names: list[str] = []
                if diagnostic.get("ok"):
                    diagnostic_debug = diagnostic.setdefault("debug", {})
                    diagnostic_debug["reason"] = (
                        f"{diagnostic_debug.get('reason', 'success')}:"
                        "diagnostic_soft_fallback"
                    )
                    diagnostic_debug["softBlockedLms"] = sorted(
                        soft_blocked_lms
                    )
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
                        request_names={
                            str(request.get("name") or "")
                            for request in valid_requests
                        },
                    )
                    if not blocker_names:
                        diagnostic = self._apply_continuous_reservation_waits(
                            diagnostic,
                            ignore_robot_names=release_owners,
                            stationary_robot_names=held_snapshot_owners,
                            prediction_offset=reservation_offset,
                        )
                        if diagnostic.get("ok") or diagnostic.get("debug", {}).get(
                            "stationaryBlockerRobots"
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
                        request_names={
                            str(request.get("name") or "")
                            for request in valid_requests
                        },
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
            result = self.planner.plan(
                {
                    **planner_payload,
                    "blocked_lms": sorted(hard_blocked_lms),
                }
            )
            if result.get("ok"):
                debug = result.setdefault("debug", {})
                debug["reason"] = f"{debug.get('reason', 'success')}:fallback_wait"
                debug["softBlockedLms"] = sorted(soft_blocked_lms)
                debug["softBlockFailure"] = failed_reason
                result = self._apply_continuous_reservation_waits(
                    result,
                    ignore_robot_names=release_owners,
                    stationary_robot_names=held_snapshot_owners,
                    prediction_offset=reservation_offset,
                )
                self._event(
                    "warn",
                    "planner found no detour; using original route and runtime waiting",
                )
            return result

        result = self.planner.plan(
            {
                **planner_payload,
                "blocked_lms": sorted(hard_blocked_lms),
            }
        )
        if result.get("ok"):
            result = self._apply_continuous_reservation_waits(
                result,
                ignore_robot_names=release_owners,
                stationary_robot_names=held_snapshot_owners,
                prediction_offset=reservation_offset,
            )
        return result

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

    def _apply_planner_result(
        self,
        result: dict[str, Any],
        now: float | None = None,
        order_id: str | None = None,
    ) -> None:
        now = now or self._now()
        for plan in result.get("plans", []):
            if not isinstance(plan, dict):
                continue
            name = str(plan.get("robot", ""))
            robot = self.robots.get(name)
            if robot is None:
                continue
            trajectory = [
                item for item in plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            robot.status = "MOVING" if trajectory else "BLOCKED"
            robot.current_lm = str(plan.get("startLm") or robot.current_lm)
            robot.target_lm = str(plan.get("goalLm") or robot.target_lm)
            robot.trajectory = trajectory
            robot.trajectory_dirty = True
            robot.plan_nodes = [
                str(item) for item in plan.get("nodes", [])
            ]
            robot.route_started_at = now
            if robot.is_remote() and robot.pose:
                robot.route_clock = self._nearest_trajectory_clock(trajectory, robot.pose)
            else:
                robot.route_clock = 0.0
            robot.last_tick_at = now
            if not robot.is_remote():
                robot.pose = self._pose_at_trajectory(robot.trajectory, 0.0) or robot.pose
            robot.route_note = self._plan_note(result)
            robot.last_reason = robot.route_note if trajectory else "empty trajectory"
            robot.blocked_since = None
            robot.traffic_stall_since = None
            self._clear_wait_dependency(robot)
            self._clear_deadlock_retreat(robot)
            if order_id is not None:
                robot.active_order_id = order_id
            robot.updated_at = now

    def _plan_note(self, result: dict[str, Any]) -> str:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return "planner accepted"
        reason = str(debug.get("reason", "") or "")
        blocked_edges = debug.get("hardBlockedEdges") or debug.get("blockedEdges", [])
        reserved_detour_edges = debug.get("reservedDetourEdges", [])
        reserved_edges = int(debug.get("reservedEdges", 0) or 0)
        continuous_waits = int(debug.get("continuousWaits", 0) or 0)
        if "fallback_wait" in reason or "reserved_interval_fallback_wait" in reason:
            return "FALLBACK_WAIT"
        if "reserved_edge_detour" in reason:
            return "DETOUR: reserved edge"
        if "detour_soft_blocks" in reason:
            return "DETOUR"
        if isinstance(blocked_edges, list) and blocked_edges:
            return "DETOUR: edge blocked"
        if continuous_waits > 0:
            return "WAIT: reserved corridor"
        if reserved_edges > 0 or (isinstance(reserved_detour_edges, list) and reserved_detour_edges):
            return "DETOUR: reserved edge"
        return "planner accepted"

    def _planner_deadlock_result(self, result: dict[str, Any]) -> bool:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return False
        if bool(debug.get("deadlock")):
            return True
        try:
            return int(debug.get("continuousUnresolved", 0) or 0) > 0
        except (TypeError, ValueError):
            return False

    def _planner_failure_reason(self, result: dict[str, Any]) -> str:
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            return "planner rejected"
        if self._planner_deadlock_result(result):
            detail = str(debug.get("deadlockReason") or "").strip()
            return f"deadlock: {detail or 'planner could not resolve robot traffic; robots hold position'}"
        return str(debug.get("reason") or "planner rejected")

    def _apply_continuous_reservation_waits(
        self,
        result: dict[str, Any],
        ignore_robot_names: set[str] | None = None,
        stationary_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> dict[str, Any]:
        plans = result.get("plans", [])
        if not isinstance(plans, list) or not plans:
            return result

        total_wait = 0.0
        total_conflicts = 0
        wait_count = 0
        unresolved_count = 0
        unresolved_conflicts: list[dict[str, Any]] = []
        planned_names = {
            str(plan.get("robot", ""))
            for plan in plans
            if isinstance(plan, dict) and str(plan.get("robot", ""))
        }
        ignored_names = planned_names | (ignore_robot_names or set())
        for plan in plans:
            if not isinstance(plan, dict):
                continue
            robot_name = str(plan.get("robot", ""))
            trajectory = [
                item for item in plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            if not robot_name or len(trajectory) < 2:
                continue
            trajectory, stats = self._schedule_trajectory_against_corridors(
                robot_name,
                trajectory,
                ignore_robot_names=ignored_names,
                stationary_robot_names=stationary_robot_names,
                prediction_offset=prediction_offset,
                unresolved_conflicts=unresolved_conflicts,
            )
            if stats["conflicts"] > 0:
                plan["trajectory"] = trajectory
                plan["arrivalTime"] = float(trajectory[-1].get("t", 0.0) or 0.0)
                total_wait += stats["wait"]
                total_conflicts += stats["conflicts"]
                wait_count += stats["waits"]
            unresolved_count += int(stats.get("unresolved", 0) or 0)

        batch_trajectory_stats = self._schedule_batch_trajectories(
            plans,
            unresolved_conflicts=unresolved_conflicts,
        )
        total_wait += batch_trajectory_stats["wait"]
        total_conflicts += batch_trajectory_stats["conflicts"]
        wait_count += batch_trajectory_stats["waits"]
        unresolved_count += int(batch_trajectory_stats.get("unresolved", 0) or 0)

        if total_conflicts > 0 or unresolved_count > 0:
            debug = result.setdefault("debug", {})
            debug["continuousConflicts"] = int(debug.get("continuousConflicts", 0) or 0) + total_conflicts
            debug["continuousWaits"] = int(debug.get("continuousWaits", 0) or 0) + wait_count
            debug["continuousWaitSec"] = round(float(debug.get("continuousWaitSec", 0.0) or 0.0) + total_wait, 3)
            debug["continuousUnresolved"] = int(debug.get("continuousUnresolved", 0) or 0) + unresolved_count
            if unresolved_conflicts:
                debug["continuousUnresolvedConflicts"] = [
                    dict(conflict) for conflict in unresolved_conflicts
                ]
                first_conflict = unresolved_conflicts[0]
                debug["continuousConflictRobot"] = str(
                    first_conflict.get("other") or ""
                )
                debug["continuousConflictEdge"] = str(
                    first_conflict.get("edge") or "unknown"
                )
            if batch_trajectory_stats["conflicts"] > 0:
                debug["batchContinuousConflicts"] = int(debug.get("batchContinuousConflicts", 0) or 0) + batch_trajectory_stats["conflicts"]
                debug["batchContinuousWaits"] = int(debug.get("batchContinuousWaits", 0) or 0) + batch_trajectory_stats["waits"]
                debug["batchContinuousWaitSec"] = round(float(debug.get("batchContinuousWaitSec", 0.0) or 0.0) + batch_trajectory_stats["wait"], 3)
            debug["reason"] = f"{debug.get('reason', 'success')}:reserved_corridor_wait"
            if unresolved_count > 0:
                debug["reason"] = f"{debug.get('reason', 'success')}:continuous_conflict_unresolved"
                debug["deadlock"] = True
                stationary_conflicts: list[dict[str, Any]] = []
                for conflict in unresolved_conflicts:
                    blocker = self.robots.get(str(conflict.get("other") or ""))
                    if (
                        blocker is None
                        or blocker.trajectory
                        or blocker.status not in {"IDLE", "ARRIVED", "BLOCKED"}
                        or self._robot_departure_pending(blocker)
                    ):
                        continue
                    lm_name = self._nearest_lm_for_robot(blocker)
                    if lm_name not in self.landmarks:
                        continue
                    stationary_conflicts.append({**conflict, "lm": lm_name})

                if stationary_conflicts:
                    blocker_lms = sorted({
                        str(conflict["lm"]) for conflict in stationary_conflicts
                    })
                    blocker_names = sorted({
                        str(conflict["other"]) for conflict in stationary_conflicts
                    })
                    debug["stationaryRobotWait"] = True
                    debug["stationaryTurnEnvelopeBlock"] = True
                    debug["stationaryBlockerRobots"] = blocker_names
                    debug["softBlockedLms"] = sorted(
                        set(debug.get("softBlockedLms", [])) | set(blocker_lms)
                    )
                    first = stationary_conflicts[0]
                    debug["reason"] = (
                        f"{debug['reason']}:stationary_robot_blocks_route"
                    )
                    debug["deadlockReason"] = (
                        "stationary turn-envelope block: "
                        f"{first.get('robot') or 'planned robot'} by "
                        f"{first.get('other')} at {first.get('lm')} on "
                        f"{first.get('edge') or 'unknown'}; "
                        "stationary_robot_blocks_route"
                    )
                else:
                    debug["deadlockReason"] = (
                        "continuous reservation conflict could not be resolved; robots will hold position"
                    )
                debug["rejectedPlanCount"] = len(plans)
                result["ok"] = False
                result["plans"] = []
        return result

    def _schedule_trajectory_against_corridors(
        self,
        robot_name: str,
        trajectory: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        stationary_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
        unresolved_conflicts: list[dict[str, Any]] | None = None,
    ) -> tuple[list[dict[str, Any]], dict[str, float | int]]:
        conflicts = 0
        waits = 0
        total_wait = 0.0
        max_iterations = 10
        ignored = ignore_robot_names or set()
        for _ in range(max_iterations):
            conflict = self._first_continuous_corridor_conflict(
                robot_name,
                trajectory,
                ignore_robot_names=ignored,
                stationary_robot_names=stationary_robot_names,
                prediction_offset=prediction_offset,
            )
            if conflict is None:
                break
            conflicts += 1
            wait_point = self._wait_insert_point(
                trajectory,
                max(0.0, float(conflict["time"]) - self._reservation_safety_time()),
            )
            wait_duration = self._wait_duration_for_conflict(
                trajectory,
                float(conflict["time"]),
                str(conflict["other"]),
                stationary_robot_names=stationary_robot_names,
                prediction_offset=prediction_offset,
            )
            if wait_duration >= self._reservation_horizon() - 0.000001:
                # ``_wait_duration_for_conflict`` returns a full-horizon wait
                # when the owner never clears.  That value is a no-clearance
                # sentinel, not a valid schedule.  Inserting it used to push a
                # future collision just beyond the finite validation horizon,
                # so the poisoned trajectory was accepted and permanently
                # reserved both ends of a narrow corridor.
                self._event(
                    "warn",
                    (
                        f"{robot_name} corridor block by {conflict['other']} "
                        "does not clear inside the reservation horizon; "
                        "detour required"
                    ),
                )
                break
            trajectory = self._insert_trajectory_wait(
                trajectory,
                wait_point["index"],
                wait_duration,
            )
            waits += 1
            total_wait += wait_duration
            self._event(
                "warn",
                (
                    f"{robot_name} reservation wait: t={float(conflict['time']):.2f}s "
                    f"edge={conflict['edge']} other={conflict['other']} "
                    f"wait={wait_duration:.2f}s"
                ),
            )
        remaining_conflict = self._first_continuous_corridor_conflict(
            robot_name,
            trajectory,
            ignore_robot_names=ignored,
            stationary_robot_names=stationary_robot_names,
            prediction_offset=prediction_offset,
        )
        if remaining_conflict is not None:
            if unresolved_conflicts is not None:
                unresolved_conflicts.append({
                    "source": "committed",
                    "robot": robot_name,
                    "other": str(remaining_conflict.get("other") or ""),
                    "edge": str(remaining_conflict.get("edge") or "unknown"),
                    "time": float(remaining_conflict.get("time", 0.0) or 0.0),
                })
            self._event(
                "error",
                (
                    f"{robot_name} reservation deadlock: "
                    f"t={float(remaining_conflict['time']):.2f}s "
                    f"edge={remaining_conflict['edge']} "
                    f"other={remaining_conflict['other']}"
                ),
            )
            return trajectory, {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 1}
        return trajectory, {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 0}

    def _schedule_batch_trajectories(
        self,
        plans: list[Any],
        *,
        unresolved_conflicts: list[dict[str, Any]] | None = None,
    ) -> dict[str, float | int]:
        scheduled = [
            plan for plan in plans
            if isinstance(plan, dict)
            and str(plan.get("robot", ""))
            and isinstance(plan.get("trajectory"), list)
            and len(plan.get("trajectory", [])) >= 2
        ]
        if len(scheduled) < 2:
            return {"conflicts": 0, "waits": 0, "wait": 0.0, "unresolved": 0}

        conflicts = 0
        waits = 0
        total_wait = 0.0
        max_iterations = self._batch_wait_max_iterations()
        seen_dependencies: set[tuple[int, int, str]] = set()
        for _ in range(max_iterations):
            conflict = self._first_batch_trajectory_conflict(scheduled)
            if conflict is None:
                break
            dependency = (
                int(conflict["priorityIndex"]),
                int(conflict["waitIndex"]),
                str(conflict["edge"]),
            )
            if dependency in seen_dependencies:
                self._event(
                    "warn",
                    (
                        "batch reservation made no progress for "
                        f"{scheduled[dependency[1]].get('robot')} behind "
                        f"{scheduled[dependency[0]].get('robot')} on "
                        f"{dependency[2]}"
                    ),
                )
                break
            seen_dependencies.add(dependency)
            waiting_plan = scheduled[int(conflict["waitIndex"])]
            priority_plan = scheduled[int(conflict["priorityIndex"])]
            trajectory = [
                item for item in waiting_plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            priority_trajectory = [
                item for item in priority_plan.get("trajectory", [])
                if isinstance(item, dict)
            ]
            if len(trajectory) < 2 or len(priority_trajectory) < 2:
                break

            wait_point = self._wait_insert_point(trajectory, float(conflict["time"]))
            wait_duration = self._wait_duration_for_peer_conflict(
                trajectory,
                priority_trajectory,
                float(conflict["time"]),
            )
            if wait_duration >= max(
                2.0,
                self._reservation_horizon(),
            ) - 0.000001:
                # This is the peer equivalent of the no-clearance sentinel
                # used by corridor scheduling.  Appending it only shifts the
                # same collision past a finite validation window.
                self._event(
                    "warn",
                    (
                        f"{waiting_plan.get('robot')} cannot wait for "
                        f"{priority_plan.get('robot')}: peer does not clear "
                        "inside the reservation horizon"
                    ),
                )
                break
            if (
                total_wait + wait_duration
                > self._reservation_horizon() + 0.000001
            ):
                self._event(
                    "warn",
                    "batch reservation wait budget exhausted; joint replan required",
                )
                break
            trajectory = self._insert_trajectory_wait(
                trajectory,
                int(wait_point["index"]),
                wait_duration,
            )
            waiting_plan["trajectory"] = trajectory
            waiting_plan["arrivalTime"] = float(trajectory[-1].get("t", 0.0) or 0.0)
            conflicts += 1
            waits += 1
            total_wait += wait_duration
            self._event(
                "warn",
                (
                    f"{waiting_plan.get('robot')} batch reservation wait: "
                    f"t={float(conflict['time']):.2f}s "
                    f"edge={conflict['edge']} "
                    f"priority={priority_plan.get('robot')} "
                    f"wait={wait_duration:.2f}s"
                ),
            )
        remaining_conflict = self._first_batch_trajectory_conflict(scheduled)
        if remaining_conflict is not None:
            if unresolved_conflicts is not None:
                priority_index = int(remaining_conflict["priorityIndex"])
                wait_index = int(remaining_conflict["waitIndex"])
                unresolved_conflicts.append({
                    "source": "batch",
                    "robot": str(scheduled[wait_index].get("robot") or ""),
                    "other": str(scheduled[priority_index].get("robot") or ""),
                    "edge": str(remaining_conflict.get("edge") or "unknown"),
                    "time": float(remaining_conflict.get("time", 0.0) or 0.0),
                })
            self._event(
                "error",
                (
                    "batch reservation unresolved: "
                    f"t={float(remaining_conflict['time']):.2f}s "
                    f"edge={remaining_conflict['edge']}"
                ),
            )
            return {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 1}
        return {"conflicts": conflicts, "waits": waits, "wait": total_wait, "unresolved": 0}

    def _first_batch_trajectory_conflict(
        self,
        plans: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        step = self._continuous_collision_step()
        final_time = 0.0
        for plan in plans:
            trajectory = plan.get("trajectory", [])
            if isinstance(trajectory, list) and trajectory:
                final_time = max(final_time, float(trajectory[-1].get("t", 0.0) or 0.0))
        horizon = min(final_time, self._batch_collision_horizon(final_time))
        initial_clearance_release: dict[tuple[int, int], float] = {}
        for first_index, first_plan in enumerate(plans):
            first_trajectory = first_plan.get("trajectory", [])
            if not isinstance(first_trajectory, list):
                continue
            for second_index in range(first_index + 1, len(plans)):
                second_trajectory = plans[second_index].get("trajectory", [])
                if not isinstance(second_trajectory, list):
                    continue
                release = self._initial_clearance_release_time(
                    lambda elapsed, trajectory=first_trajectory: self._pose_at_trajectory(
                        trajectory,
                        elapsed,
                    ),
                    lambda elapsed, trajectory=second_trajectory: self._pose_at_trajectory(
                        trajectory,
                        elapsed,
                    ),
                    horizon,
                )
                if release is not None:
                    initial_clearance_release[(first_index, second_index)] = release
        t = 0.0
        while t <= horizon + 0.000001:
            # Pose interpolation used to run once for every robot pair.  At 20
            # robots that means 190 scans through sampled trajectories for
            # every time slice.  Cache the N poses for this slice and keep the
            # same exact footprint checks below.
            poses = [
                self._pose_at_trajectory(
                    plan.get("trajectory", [])
                    if isinstance(plan.get("trajectory", []), list)
                    else [],
                    t,
                )
                for plan in plans
            ]
            cell_size = max(0.05, self.collision.robot_broadphase_distance())
            grid: dict[tuple[int, int], list[int]] = {}
            candidate_pairs: set[tuple[int, int]] = set()
            for index, pose in enumerate(poses):
                if pose is None:
                    continue
                cell = (
                    math.floor(float(pose.get("x", 0.0) or 0.0) / cell_size),
                    math.floor(float(pose.get("y", 0.0) or 0.0) / cell_size),
                )
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for other_index in grid.get((cell[0] + dx, cell[1] + dy), ()):
                            candidate_pairs.add((other_index, index))
                grid.setdefault(cell, []).append(index)

            for priority_index, wait_index in sorted(candidate_pairs):
                priority_trajectory = plans[priority_index].get("trajectory", [])
                if not isinstance(priority_trajectory, list):
                    continue
                priority_pose = poses[priority_index]
                if priority_pose is None:
                    continue
                waiting_trajectory = plans[wait_index].get("trajectory", [])
                if not isinstance(waiting_trajectory, list):
                    continue
                waiting_pose = poses[wait_index]
                if waiting_pose is None:
                    continue
                if self.collision.robot_footprints_conflict(priority_pose, waiting_pose):
                    release = initial_clearance_release.get((priority_index, wait_index))
                    if (
                        release is not None
                        and t <= release + 0.000001
                        and not self.collision.footprints_overlap(priority_pose, waiting_pose)
                    ):
                        continue
                    priority_edge = self._edge_id_at_trajectory(priority_trajectory, t) or "unknown"
                    waiting_edge = self._edge_id_at_trajectory(waiting_trajectory, t) or "unknown"
                    # Keep one deterministic winner for the complete pair.
                    # Flipping priority after an inserted WAIT made two
                    # head-on plans alternately delay each other forever.
                    # ``candidate_pairs`` is sorted by plan index, which
                    # preserves the request/MAPF priority order.
                    return {
                        "time": t,
                        "priorityIndex": priority_index,
                        "waitIndex": wait_index,
                        "edge": waiting_edge,
                    }
            t += step
        return None

    def _edge_start_time_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> float:
        if not trajectory:
            return 0.0
        segment_index = 0
        for index in range(len(trajectory) - 1):
            start_t = float(trajectory[index].get("t", 0.0) or 0.0)
            end_t = float(trajectory[index + 1].get("t", 0.0) or 0.0)
            if start_t <= elapsed <= end_t:
                segment_index = index
                break
        edge_id = str(
            trajectory[min(segment_index + 1, len(trajectory) - 1)].get("edgeId")
            or trajectory[segment_index].get("edgeId")
            or ""
        )
        insert_index = segment_index
        while insert_index > 0:
            previous_edge = str(trajectory[insert_index].get("edgeId", "") or "")
            if previous_edge != edge_id:
                break
            insert_index -= 1
        return float(trajectory[max(0, insert_index)].get("t", 0.0) or 0.0)

    def _wait_duration_for_peer_conflict(
        self,
        trajectory: list[dict[str, Any]],
        priority_trajectory: list[dict[str, Any]],
        conflict_time: float,
    ) -> float:
        conflict_pose = self._pose_at_trajectory(trajectory, conflict_time)
        if conflict_pose is None:
            return self._reservation_safety_time()

        step = self._continuous_collision_step()
        safety = self._reservation_safety_time()
        max_wait = max(2.0, self._reservation_horizon())
        wait = 0.0
        while wait <= max_wait + 0.000001:
            priority_pose = self._pose_at_trajectory(priority_trajectory, conflict_time + wait)
            if priority_pose is None or not self.collision.robot_footprints_conflict(conflict_pose, priority_pose):
                return max(safety, wait + safety)
            wait += step
        return max_wait + safety

    def _first_continuous_corridor_conflict(
        self,
        robot_name: str,
        trajectory: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        stationary_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> dict[str, Any] | None:
        ignored = ignore_robot_names or set()
        stationary = stationary_robot_names or set()
        corridor_robots = [
            robot
            for robot in self._runtime_robots()
            if robot.name != robot_name
            and robot.name not in ignored
            and robot.pose is not None
        ]
        if not corridor_robots:
            return None
        final_time = float(trajectory[-1].get("t", 0.0) or 0.0)
        step = self._continuous_collision_step()
        horizon = min(final_time, self._reservation_horizon())
        initial_clearance_release: dict[str, float] = {}
        for other in corridor_robots:
            def peer_pose_at(
                elapsed: float,
                peer: FleetRobot = other,
            ) -> dict[str, float] | None:
                if peer.name in stationary:
                    return dict(peer.pose) if peer.pose is not None else None
                return self._predicted_robot_pose(
                    peer,
                    prediction_offset + elapsed,
                )

            release = self._initial_clearance_release_time(
                lambda elapsed: self._pose_at_trajectory(trajectory, elapsed),
                peer_pose_at,
                horizon,
            )
            if release is not None:
                initial_clearance_release[other.name] = release
        t = 0.0
        while t <= horizon + 0.000001:
            pose = self._pose_at_trajectory(trajectory, t)
            if pose is None:
                t += step
                continue
            for other in corridor_robots:
                other_pose = (
                    dict(other.pose)
                    if other.name in stationary and other.pose is not None
                    else self._predicted_robot_pose(
                        other,
                        prediction_offset + t,
                    )
                )
                if other_pose is None:
                    continue
                if self.collision.robot_footprints_conflict(pose, other_pose):
                    release = initial_clearance_release.get(other.name)
                    if (
                        release is not None
                        and t <= release + 0.000001
                        and not self.collision.footprints_overlap(pose, other_pose)
                    ):
                        continue
                    edge = self._edge_id_at_trajectory(trajectory, t)
                    return {
                        "time": t,
                        "other": other.name,
                        "edge": edge or "unknown",
                    }
            t += step
        return None

    def _initial_clearance_release_time(
        self,
        first_pose_at: Callable[[float], dict[str, float] | None],
        second_pose_at: Callable[[float], dict[str, float] | None],
        horizon: float,
    ) -> float | None:
        """Return when an initially tight, physically safe pair separates.

        Clearance is a preventive envelope, not an obstacle that may imprison
        robots which already start inside it. The waiver lasts only until the
        pair first exits the envelope and is granted only when their first
        translational motion increases distance without physical overlap.
        """
        first_start = first_pose_at(0.0)
        second_start = second_pose_at(0.0)
        if first_start is None or second_start is None:
            return None
        if not self.collision.robot_footprints_conflict(first_start, second_start):
            return None
        if self.collision.footprints_overlap(first_start, second_start):
            return None

        initial_distance = math.hypot(
            float(first_start.get("x", 0.0) or 0.0)
            - float(second_start.get("x", 0.0) or 0.0),
            float(first_start.get("y", 0.0) or 0.0)
            - float(second_start.get("y", 0.0) or 0.0),
        )
        step = self._continuous_collision_step()
        elapsed = step
        departed = False
        while elapsed <= horizon + 0.000001:
            first_pose = first_pose_at(elapsed)
            second_pose = second_pose_at(elapsed)
            if first_pose is None or second_pose is None:
                return None
            if self.collision.footprints_overlap(first_pose, second_pose):
                return None
            distance = math.hypot(
                float(first_pose.get("x", 0.0) or 0.0)
                - float(second_pose.get("x", 0.0) or 0.0),
                float(first_pose.get("y", 0.0) or 0.0)
                - float(second_pose.get("y", 0.0) or 0.0),
            )
            translation = max(
                math.hypot(
                    float(first_pose.get("x", 0.0) or 0.0)
                    - float(first_start.get("x", 0.0) or 0.0),
                    float(first_pose.get("y", 0.0) or 0.0)
                    - float(first_start.get("y", 0.0) or 0.0),
                ),
                math.hypot(
                    float(second_pose.get("x", 0.0) or 0.0)
                    - float(second_start.get("x", 0.0) or 0.0),
                    float(second_pose.get("y", 0.0) or 0.0)
                    - float(second_start.get("y", 0.0) or 0.0),
                ),
            )
            if not departed and translation >= 0.02:
                if distance <= initial_distance + 0.01:
                    return None
                departed = True
            if departed and not self.collision.robot_footprints_conflict(
                first_pose,
                second_pose,
            ):
                return elapsed
            elapsed += step
        return None

    def _wait_duration_for_conflict(
        self,
        trajectory: list[dict[str, Any]],
        conflict_time: float,
        other_name: str,
        stationary_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> float:
        conflict_pose = self._pose_at_trajectory(trajectory, conflict_time)
        other = self.robots.get(other_name)
        if conflict_pose is None or other is None:
            return self._reservation_safety_time()

        step = self._continuous_collision_step()
        safety = self._reservation_safety_time()
        max_wait = max(2.0, self._reservation_horizon())
        stationary = stationary_robot_names or set()
        wait = 0.0
        while wait <= max_wait + 0.000001:
            other_pose = (
                dict(other.pose)
                if other.name in stationary and other.pose is not None
                else self._predicted_robot_pose(
                    other,
                    prediction_offset + conflict_time + wait,
                )
            )
            if other_pose is None or not self.collision.robot_footprints_conflict(conflict_pose, other_pose):
                return max(safety, wait + safety)
            wait += step
        return max_wait + safety

    def _wait_insert_point(
        self,
        trajectory: list[dict[str, Any]],
        conflict_time: float,
        clamp_to_edge_start: bool = True,
    ) -> dict[str, float | int]:
        if conflict_time <= 0.0 or len(trajectory) <= 1:
            return {"index": 0, "time": 0.0}
        segment_index = 0
        for index in range(len(trajectory) - 1):
            start_t = float(trajectory[index].get("t", 0.0) or 0.0)
            end_t = float(trajectory[index + 1].get("t", 0.0) or 0.0)
            if start_t <= conflict_time <= end_t:
                segment_index = index
                break
        if not clamp_to_edge_start:
            return {
                "index": max(0, segment_index),
                "time": float(trajectory[max(0, segment_index)].get("t", 0.0) or 0.0),
            }
        edge_id = str(
            trajectory[min(segment_index + 1, len(trajectory) - 1)].get("edgeId")
            or trajectory[segment_index].get("edgeId")
            or ""
        )
        insert_index = segment_index
        while insert_index > 0:
            previous_edge = str(trajectory[insert_index].get("edgeId", "") or "")
            if previous_edge != edge_id:
                break
            insert_index -= 1
        return {
            "index": max(0, insert_index),
            "time": float(trajectory[max(0, insert_index)].get("t", 0.0) or 0.0),
        }

    def _insert_trajectory_wait(
        self,
        trajectory: list[dict[str, Any]],
        insert_index: int,
        wait_duration: float,
    ) -> list[dict[str, Any]]:
        if wait_duration <= 0.0 or not trajectory:
            return trajectory
        insert_index = max(0, min(insert_index, len(trajectory) - 1))
        wait_duration = max(0.0, wait_duration)
        anchor = dict(trajectory[insert_index])
        anchor_time = float(anchor.get("t", 0.0) or 0.0)
        hold = {
            **anchor,
            "t": anchor_time + wait_duration,
            "edgeId": f"WAIT@{anchor.get('edgeId', 'route')}",
        }
        shifted = [
            {
                **sample,
                "t": float(sample.get("t", 0.0) or 0.0) + wait_duration,
            }
            for sample in trajectory[insert_index + 1:]
        ]
        return trajectory[: insert_index + 1] + [hold] + shifted

    def _dynamic_blocked_edges(self) -> set[tuple[str, str]]:
        blocked = set(self._static_blocked_edges)
        if not self.obstacles and not self.obstacle_areas:
            return blocked
        for edge in self.edges:
            route = PlannedRoute(
                nodes=[edge.from_name, edge.to_name],
                edges=[edge],
                length=edge.length,
            )
            try:
                samples = self.planner.route_planner.sample_route(route, sample_distance=0.15)
            except Exception:
                continue
            for sample in samples:
                pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                }
                reason = self.collision.dynamic_blocked_reason(
                    pose,
                    self.obstacles,
                    self.obstacle_areas,
                )
                if reason:
                    blocked.add((edge.from_name, edge.to_name))
                    break
        return blocked

    def _static_map_blocked_edges(self) -> set[tuple[str, str]]:
        if self.collision.map_pixels is None or self.collision.map_metadata is None:
            return set()
        blocked: set[tuple[str, str]] = set()
        for edge in self.edges:
            route = PlannedRoute(
                nodes=[edge.from_name, edge.to_name],
                edges=[edge],
                length=edge.length,
            )
            try:
                samples = self.planner.route_planner.sample_route(
                    route,
                    sample_distance=0.10,
                )
            except Exception:
                blocked.add((edge.from_name, edge.to_name))
                continue
            for sample in samples:
                pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(sample.get("yaw", 0.0) or 0.0),
                }
                if self.collision.blocked_reason(pose, [], []):
                    blocked.add((edge.from_name, edge.to_name))
                    break
        return blocked

    def _hard_blocked_edges(self, payload: dict[str, Any]) -> set[tuple[str, str]]:
        raw_edges = payload.get("blocked_edges") or payload.get("blockedEdges") or []
        if not isinstance(raw_edges, list):
            return set()
        blocked: set[tuple[str, str]] = set()
        for item in raw_edges:
            if isinstance(item, str) and "->" in item:
                src, dst = item.split("->", 1)
                src = src.strip()
                dst = dst.strip()
            elif isinstance(item, dict):
                src = str(item.get("from") or item.get("fromLm") or "").strip()
                dst = str(item.get("to") or item.get("toLm") or "").strip()
            else:
                continue
            if src in self.landmarks and dst in self.landmarks:
                blocked.add((src, dst))
        return blocked

    def _reserved_constraints(
        self,
        requests: list[dict[str, Any]],
    ) -> tuple[list[tuple[int, str]], list[tuple[int, str, str]]]:
        request_names = {str(request.get("name", "")) for request in requests}
        time_step = self._reservation_time_step()
        horizon = self._reservation_horizon()
        vertices: set[tuple[int, str]] = set()
        edges: set[tuple[int, str, str]] = set()

        for robot in self._runtime_robots():
            if robot.name in request_names:
                continue
            if not robot.trajectory or robot.status not in {"MOVING", "WAITING"}:
                continue
            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            tick = 0
            offset = 0.0
            while offset <= horizon + 0.000001:
                sample_clock = min(final_time, robot.route_clock + offset)
                pose = self._pose_at_trajectory(robot.trajectory, sample_clock)
                if pose is None:
                    break
                lm_name = self._nearest_lm_for_pose(pose)
                if lm_name:
                    vertices.add((tick, lm_name))
                edge_id = self._edge_id_at_trajectory(robot.trajectory, sample_clock)
                edge = self._parse_edge_id(edge_id)
                if edge is not None:
                    src, dst = edge
                    edges.add((tick, src, dst))
                    edges.add((tick, dst, src))
                if sample_clock >= final_time:
                    break
                tick += 1
                offset += time_step
        return sorted(vertices), sorted(edges)

    def _reserved_edge_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, str, float, float, str]]:
        request_names = {str(request.get("name", "")) for request in requests}
        ignored = ignore_robot_names or set()
        horizon = self._reservation_horizon()
        safety = self._reservation_safety_time()
        intervals: list[tuple[str, str, float, float, str]] = []
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.name in ignored:
                continue
            if robot.status not in {"MOVING", "WAITING"} or len(robot.trajectory) < 2:
                continue
            trajectory = robot.trajectory
            future_clock = robot.route_clock + max(0.0, prediction_offset)
            first_relevant_clock = max(
                float(trajectory[0].get("t", 0.0) or 0.0),
                future_clock - safety,
            )
            first_index = max(
                0,
                self._trajectory_segment_index(
                    trajectory,
                    first_relevant_clock,
                    boundary_belongs_to_previous=True,
                ) - 1,
            )
            active_edge: tuple[str, str] | None = None
            active_start = 0.0
            active_end = 0.0

            def flush_edge() -> None:
                nonlocal active_edge, active_start, active_end
                if active_edge is None:
                    return
                start_time = active_start - safety
                end_time = active_end + safety
                if end_time >= 0.0 and start_time <= horizon:
                    src, dst = active_edge
                    intervals.append(
                        (
                            src,
                            dst,
                            max(0.0, start_time),
                            min(horizon, end_time),
                            robot.name,
                        )
                    )
                active_edge = None

            for index in range(first_index, len(trajectory) - 1):
                start = trajectory[index]
                end = trajectory[index + 1]
                start_time = float(start.get("t", 0.0) or 0.0) - future_clock
                end_time = float(end.get("t", 0.0) or 0.0) - future_clock
                if end_time < -safety:
                    continue
                if start_time > horizon + safety:
                    break
                edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
                edge = self._parse_edge_id(edge_id)
                if edge is None:
                    flush_edge()
                    continue
                if edge != active_edge:
                    flush_edge()
                    active_edge = edge
                    active_start = start_time
                    active_end = end_time
                else:
                    active_end = end_time
            flush_edge()
        return intervals

    def _reserved_vertex_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        ignore_nodes: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, float, float, str]]:
        request_names = {str(request.get("name", "")) for request in requests}
        ignored = ignore_robot_names or set()
        skipped_nodes = ignore_nodes or set()
        horizon = self._reservation_horizon()
        safety = self._reservation_safety_time()
        intervals: list[tuple[str, float, float, str]] = []

        def add_interval(node: str, start: float, end: float, owner: str) -> None:
            if node not in self.landmarks:
                return
            if node in skipped_nodes:
                return
            start_time = max(0.0, min(start, end))
            end_time = min(horizon, max(start, end))
            if end_time < 0.0 or start_time > horizon:
                return
            intervals.append((node, start_time, end_time, owner))

        for robot in self._runtime_robots():
            if robot.name in request_names or robot.name in ignored:
                continue

            future_pose = self._predicted_robot_pose(robot, max(0.0, prediction_offset))
            current_lm = (
                self._nearest_lm_for_pose(future_pose)
                if future_pose is not None
                else self._nearest_lm_for_robot(robot)
            )
            if current_lm:
                add_interval(current_lm, 0.0, safety * 2.0, robot.name)

            if robot.status not in {"MOVING", "WAITING"} or len(robot.trajectory) < 2:
                if current_lm:
                    add_interval(current_lm, 0.0, horizon, robot.name)
                continue

            final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
            future_clock = robot.route_clock + max(0.0, prediction_offset)
            if future_clock >= final_time:
                if current_lm:
                    add_interval(current_lm, 0.0, horizon, robot.name)
                continue

            active_edge: tuple[str, str] | None = None
            active_start = 0.0
            active_end = 0.0
            first_relevant_clock = max(
                float(robot.trajectory[0].get("t", 0.0) or 0.0),
                future_clock - safety,
            )
            first_index = max(
                0,
                self._trajectory_segment_index(
                    robot.trajectory,
                    first_relevant_clock,
                    boundary_belongs_to_previous=True,
                ) - 1,
            )

            def flush_edge_vertices() -> None:
                nonlocal active_edge
                if active_edge is None:
                    return
                src, dst = active_edge
                add_interval(src, active_start - safety, active_start + safety, robot.name)
                add_interval(dst, active_end - safety, active_end + safety, robot.name)
                active_edge = None

            for index in range(first_index, len(robot.trajectory) - 1):
                start = robot.trajectory[index]
                end = robot.trajectory[index + 1]
                start_time = float(start.get("t", 0.0) or 0.0) - future_clock
                end_time = float(end.get("t", 0.0) or 0.0) - future_clock
                if end_time < -safety:
                    continue
                if start_time > horizon + safety:
                    break

                edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
                edge = self._parse_edge_id(edge_id)
                if edge is not None:
                    if edge != active_edge:
                        flush_edge_vertices()
                        active_edge = edge
                        active_start = start_time
                        active_end = end_time
                    else:
                        active_end = end_time
                    continue

                flush_edge_vertices()
                wait_lm = self._lm_from_wait_segment(start, end)
                if wait_lm:
                    add_interval(wait_lm, start_time - safety, end_time + safety, robot.name)
            flush_edge_vertices()

            final_time = (
                float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                - future_clock
            )
            final_lm = robot.target_lm if robot.target_lm in self.landmarks else self._nearest_lm_for_robot(robot)
            if final_lm and final_time <= horizon:
                add_interval(final_lm, final_time - safety, horizon, robot.name)
        return intervals

    def _lm_from_wait_segment(
        self,
        start: dict[str, Any],
        end: dict[str, Any],
    ) -> str:
        for sample in (end, start):
            lm = str(sample.get("lm") or "").strip()
            if lm in self.landmarks:
                return lm
        edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
        if edge_id.startswith("WAIT@"):
            edge_id = edge_id[5:]
        if "->" in edge_id:
            src, dst = edge_id.split("->", 1)
            src = src.strip()
            dst = dst.strip()
            if src == dst and src in self.landmarks:
                return src
        pose = self._pose_from_sample(end)
        return self._nearest_lm_for_pose(pose)

    def _reservation_time_step(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(0.25, float(fleet.get("reservation_time_step_sec", 1.0) or 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _reservation_horizon(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            configured = 8.0
        else:
            try:
                configured = max(
                    1.0,
                    float(fleet.get("reservation_horizon_sec", 8.0) or 8.0),
                )
            except (TypeError, ValueError):
                configured = 8.0
        corridor_ticks = self.planner.controlled_corridor_max_ticks()
        if corridor_ticks <= 0:
            return configured
        # A committed robot must remain visible until it has left the longest
        # controlled region. Clipping its edge reservations to the ordinary
        # rolling window lets a later request schedule an entry while the
        # first robot is still physically inside the corridor.
        corridor_sec = corridor_ticks * self._reservation_time_step()
        return max(
            configured,
            corridor_sec + (2.0 * self._reservation_safety_time()),
        )

    def _rolling_horizon(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 10.0
        try:
            configured = max(
                0.0,
                float(fleet.get("rolling_horizon_sec", 10.0) or 0.0),
            )
        except (TypeError, ValueError):
            configured = 10.0
        raw_scale_enabled = fleet.get(
            "rolling_horizon_scale_with_simulation_time",
            True,
        )
        if isinstance(raw_scale_enabled, str):
            scale_enabled = raw_scale_enabled.strip().lower() not in {
                "",
                "0",
                "false",
                "no",
                "off",
                "disabled",
            }
        else:
            scale_enabled = bool(raw_scale_enabled)
        if not scale_enabled or configured <= 0.0:
            return configured
        try:
            time_scale = max(1.0, float(self.simulation_time_scale()))
        except (AttributeError, TypeError, ValueError):
            time_scale = 1.0
        try:
            maximum = max(
                configured,
                float(fleet.get("rolling_horizon_max_sec", 120.0) or 120.0),
            )
        except (TypeError, ValueError):
            maximum = max(configured, 120.0)
        # Motion clocks accelerate, MAPF computation does not. Without this
        # wall-time invariant window, 10 simulated seconds become only 2.5
        # real seconds at 4x and the fleet reaches every rolling boundary
        # before the serialized planner can prepare its continuation.
        return min(maximum, configured * time_scale)

    def _rolling_horizon_steps(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0
        try:
            return max(0, int(fleet.get("rolling_horizon_steps", 0) or 0))
        except (TypeError, ValueError):
            return 0

    def _reservation_safety_time(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.35
        try:
            return max(0.05, float(fleet.get("reservation_safety_time_sec", 0.35) or 0.35))
        except (TypeError, ValueError):
            return 0.35

    def _continuous_collision_step(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.10
        try:
            return max(0.04, min(0.25, float(fleet.get("continuous_collision_step_sec", 0.10) or 0.10)))
        except (TypeError, ValueError):
            return 0.10

    def _batch_collision_horizon(self, final_time: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return max(1.0, final_time)
        raw_value = fleet.get("batch_collision_horizon_sec")
        if raw_value is None:
            return max(1.0, final_time)
        try:
            return max(1.0, float(raw_value))
        except (TypeError, ValueError):
            return max(1.0, final_time)

    def _batch_wait_max_iterations(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 60
        try:
            return max(1, int(fleet.get("batch_wait_max_iterations", 60) or 60))
        except (TypeError, ValueError):
            return 60

    def _nearest_lm_for_pose(self, pose: dict[str, float]) -> str:
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(
                landmark.x - float(pose.get("x", 0.0) or 0.0),
                landmark.y - float(pose.get("y", 0.0) or 0.0),
            ),
        )
        return nearest.name

    def _edge_id_at_trajectory(self, trajectory: list[dict[str, Any]], elapsed: float) -> str:
        if not trajectory:
            return ""
        if len(trajectory) == 1 or elapsed <= float(trajectory[0].get("t", 0.0) or 0.0):
            return str(trajectory[0].get("edgeId", "") or "")
        index = self._trajectory_segment_index(
            trajectory,
            elapsed,
            boundary_belongs_to_previous=True,
        )
        start = trajectory[index]
        goal = trajectory[index + 1]
        return str(goal.get("edgeId") or start.get("edgeId") or "")

    @staticmethod
    def _trajectory_segment_index(
        trajectory: list[dict[str, Any]],
        elapsed: float,
        *,
        boundary_belongs_to_previous: bool = False,
    ) -> int:
        """Find the active monotonic trajectory segment in O(log N)."""
        if len(trajectory) < 2:
            return 0
        low = 0
        high = len(trajectory) - 1
        while low + 1 < high:
            middle = (low + high) // 2
            middle_time = float(trajectory[middle].get("t", 0.0) or 0.0)
            before_or_at = (
                middle_time < elapsed
                if boundary_belongs_to_previous
                else middle_time <= elapsed
            )
            if before_or_at:
                low = middle
            else:
                high = middle
        return min(low, len(trajectory) - 2)

    @staticmethod
    def _trajectory_sample_index_at_or_before(
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> int:
        low = 0
        high = len(trajectory)
        while low < high:
            middle = (low + high) // 2
            middle_time = float(trajectory[middle].get("t", 0.0) or 0.0)
            if middle_time <= elapsed:
                low = middle + 1
            else:
                high = middle
        return low - 1

    def _parse_edge_id(self, edge_id: str) -> tuple[str, str] | None:
        if "->" not in edge_id:
            return None
        src, dst = edge_id.split("->", 1)
        src = src.strip()
        dst = dst.strip()
        if not src or not dst or src == dst:
            return None
        if src not in self.landmarks or dst not in self.landmarks:
            return None
        return src, dst

    def _hard_blocked_lms(self, payload: dict[str, Any]) -> set[str]:
        blocked_lms = payload.get("blocked_lms", [])
        if not isinstance(blocked_lms, list):
            return set()
        return {
            str(name)
            for name in blocked_lms
            if isinstance(name, str) and name in self.landmarks
        }

    def _soft_blocked_lms(
        self,
        requests: list[dict[str, Any]],
        hard_blocked_lms: set[str],
    ) -> set[str]:
        request_names = {request["name"] for request in requests}
        protected_lms = set(hard_blocked_lms)
        for request in requests:
            protected_lms.add(request["startLm"])

        # Goal LMs are intentionally not protected: arriving through a robot
        # parked on the goal is still a collision. The order waits until that
        # body moves instead of receiving a physically impossible route.
        return self._stationary_robot_blocked_lms(
            exclude_robot_names=request_names,
        ) - protected_lms

    def _stationary_blockers_named_by_failure(
        self,
        reason: str,
        candidate_lms: set[str],
        *,
        request_names: set[str],
    ) -> list[str]:
        """Resolve a failed resource to an actual inactive robot body.

        Planner failure strings name the constrained LM (for example
        ``resource_constrained:S003013@7``), but do not always include the
        owner robot. Only an LM that is both named in that diagnostic and in
        the soft stationary occupancy set is causal enough to relocate.
        """
        text = str(reason or "")
        mentioned_lms = {
            lm_name
            for lm_name in candidate_lms
            if lm_name and lm_name in text
        }
        if not mentioned_lms:
            return []
        blockers: list[str] = []
        for robot in self._runtime_robots():
            if robot.name in request_names or robot.trajectory:
                continue
            if (
                robot.status in {"IDLE", "ARRIVED"}
                and self._robot_departure_pending(robot)
            ):
                # A commanded chain sink must receive its own departure plan,
                # never be mistaken for warehouse storage and relocated by a
                # different order's recovery.
                continue
            if self._nearest_lm_for_robot(robot) in mentioned_lms:
                blockers.append(robot.name)
        return sorted(blockers)

    def _release_blocker_names_for_requests(self, requests: list[dict[str, Any]]) -> set[str]:
        request_names = {
            str(request.get("name", "")).strip()
            for request in requests
            if str(request.get("name", "")).strip()
        }
        if not request_names:
            return set()

        # Dependencies point from an upstream waiter to the robot whose
        # departure will release it.  The old one-hop lookup correctly held
        # A for a request that moved B, but kept A's upstream waiters' *future*
        # suffixes reserved.  In A->B->C->terminal those stale reservations
        # can occupy the terminal's only exit even though every upstream body
        # is physically stopped.  Walk the complete wait-for closure: keep
        # each current body as a full-horizon vertex reservation (the caller
        # does that via ``_held_blocker_vertex_intervals``), while omitting all
        # of their unexecutable future edge/vertex reservations.
        release_owners: set[str] = set()
        dependency_frontier = set(request_names)

        # Rolling SIPP can identify an external future reservation before the
        # runtime wait graph has observed the reciprocal hold. If that owner
        # is already WAITING, its advertised suffix is not executable: using
        # it again can reserve the requester's current LM and create the
        # artificial cycle "A cannot leave because B plans to arrive at A;
        # B cannot move because it is yielding to A". Preserve B's physical
        # body at its current LM, but omit that stale future on the next
        # attempt. Moving owners keep their timelines unchanged and remain
        # ordinary temporal reservations.
        for requester_name in sorted(request_names):
            for blocker_name in self._valid_rolling_prefetch_blockers(
                requester_name
            ):
                blocker = self.robots.get(blocker_name)
                if (
                    blocker is None
                    or blocker.status != "WAITING"
                    or not blocker.trajectory
                ):
                    continue
                release_owners.add(blocker.name)
                dependency_frontier.add(blocker.name)

        changed = True
        while changed:
            changed = False
            for robot in self._runtime_robots():
                if (
                    robot.name in request_names
                    or robot.name in release_owners
                    or robot.status != "WAITING"
                ):
                    continue
                dependencies: set[str] = set()
                if robot.wait_for_robot:
                    dependencies.add(str(robot.wait_for_robot))

                replan_state = self._runtime_replans.get(robot.name)
                if isinstance(replan_state, dict):
                    order = self.orders.get(
                        str(replan_state.get("order_id") or "")
                    )
                    valid_transaction = bool(
                        order is not None
                        and robot.active_order_id == order.order_id
                        and int(replan_state.get("route_revision", -1))
                        == int(robot.route_revision)
                        and abs(
                            float(replan_state.get("route_clock", 0.0) or 0.0)
                            - float(robot.route_clock)
                        ) <= 0.000001
                        and str(replan_state.get("start_lm") or "")
                        == self._safe_replan_start_lm(robot)
                    )
                    if valid_transaction:
                        dependencies.update(
                            str(name)
                            for name in replan_state.get("blocker_names", ())
                            if str(name)
                        )

                reason = str(robot.last_reason or "")
                reason_blocker = self._robot_name_from_conflict_reason(reason)
                if reason_blocker and (
                    self._is_robot_conflict(reason)
                    or reason.startswith("keep clearance from ")
                ):
                    dependencies.add(reason_blocker)

                if not dependencies.intersection(dependency_frontier):
                    continue
                release_owners.add(robot.name)
                dependency_frontier.add(robot.name)
                changed = True
        return release_owners

    def _nearest_lm_for_robot(self, robot: FleetRobot) -> str:
        pose = robot.pose
        if not pose:
            return robot.current_lm if robot.current_lm in self.landmarks else ""
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(
                landmark.x - float(pose.get("x", 0.0) or 0.0),
                landmark.y - float(pose.get("y", 0.0) or 0.0),
            ),
        )
        return nearest.name

    def _traffic_lm_for_robot(self, robot: FleetRobot) -> str:
        """Return the graph-authoritative LM without an O(|V|) pose scan.

        Runtime updates ``current_lm`` only after a tagged trajectory sample,
        so while traversing an edge it deliberately remains the source LM.
        That is exactly the occupancy side required by admission control.
        """
        if robot.current_lm in self.landmarks:
            return robot.current_lm
        return self._nearest_lm_for_robot(robot)
