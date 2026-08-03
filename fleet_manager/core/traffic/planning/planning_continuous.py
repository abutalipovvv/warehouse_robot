"""Continuous collision validation and trajectory wait insertion."""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import Any

from fleet_manager.core.domain.models import FleetRobot


class TrafficContinuousWaitSchedulingMixin:
    """Schedule trajectories against committed and batch robot motion."""


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
__all__ = ["TrafficContinuousWaitSchedulingMixin"]
