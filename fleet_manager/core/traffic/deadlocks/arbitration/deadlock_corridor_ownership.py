"""Controlled-corridor ownership and portal queue reasoning."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.domain.models import FleetRobot


class CorridorOwnershipMixin:
    """Choose physical corridor owners and bounded portal queue components."""

    def _controlled_corridor_cycle_owner(
        self,
        robots: list[FleetRobot],
    ) -> FleetRobot | None:
        """Return a physical or central-calendar corridor cycle owner."""
        by_name = {robot.name: robot for robot in robots}
        physically_owned_regions: dict[str, set[str]] = {
            name: set()
            for name in by_name
        }
        for region_id, owner_names in self._controlled_corridor_occupancy.items():
            for owner_name in owner_names:
                if owner_name in physically_owned_regions:
                    physically_owned_regions[owner_name].add(str(region_id))

        physical_owners: list[FleetRobot] = []
        for robot in robots:
            physical_regions = set(
                self._controlled_regions_for_robot(robot)
            )
            physical_regions.update(
                physically_owned_regions.get(robot.name, set())
            )
            if physical_regions:
                physical_owners.append(robot)
        if len(physical_owners) == 1:
            return physical_owners[0]

        # A corridor admission message explicitly names the robot whose
        # passage is being protected.  If that named owner is part of the
        # reciprocal wait cycle, preserve its downstream/right-of-way role
        # even when the calendar has just rolled the old slot out.  Otherwise
        # the generic name/age tiebreak can grant the red-light waiter and
        # reinforce the exact cycle the signal was meant to prevent.
        declared_owners = {
            blocker_name
            for robot in robots
            if str(robot.last_reason or "").startswith(
                "corridor admission wait at "
            )
            and (
                blocker_name := (
                    str(robot.wait_for_robot or "").strip()
                    or self._robot_name_from_conflict_reason(
                        robot.last_reason
                    )
                )
            ) in by_name
            and blocker_name != robot.name
        }
        if len(declared_owners) == 1:
            return by_name[next(iter(declared_owners))]
        if self._controlled_corridor_scheduler is None:
            return None

        active_calendar_owners: list[
            tuple[float, str, FleetRobot]
        ] = []
        schedule = self._controlled_corridor_schedule
        for robot in robots:
            slot = (
                schedule.slot_for(robot.name)
                if schedule is not None
                else None
            )
            if (
                slot is not None
                and self._controlled_corridor_has_grant(
                    robot.name,
                    slot.regions,
                )
            ):
                first_resource_entry = min(
                    (
                        slot.entry_time
                        + window.entry_offset_sec
                        for window in slot.resource_windows
                    ),
                    default=slot.entry_time,
                )
                active_calendar_owners.append(
                    (
                        first_resource_entry,
                        slot.robot_id,
                        robot,
                    )
                )
        if active_calendar_owners:
            # A same-flow convoy can hold several committed slots. The first
            # resource entrant remains the physical queue leader.
            return min(active_calendar_owners)[2]
        return None

    def _controlled_corridor_downstream_clearer(
        self,
        robots: list[FleetRobot],
    ) -> FleetRobot | None:
        """Return the robot which can open a physical corridor exit.

        A corridor owner normally keeps right of way. The one exception is an
        external body already occupying its exit pocket: commanding the owner
        forward only tightens the blockage. If that body has a committed
        trajectory which moves away from the owner, it receives the short
        local lease first. This is still one central decision; it does not
        alter corridor admission or permit a new entrant.
        """
        by_name = {robot.name: robot for robot in robots}
        candidates: list[tuple[float, str, FleetRobot]] = []
        schedule = self._controlled_corridor_schedule
        for owner_name, blocker_name in (
            self._controlled_corridor_blockers.items()
        ):
            owner = by_name.get(owner_name)
            blocker = by_name.get(blocker_name)
            if (
                owner is None
                or blocker is None
                or owner.pose is None
                or blocker.pose is None
                or not blocker.trajectory
            ):
                continue
            physical_regions = set(
                self._controlled_regions_for_robot(owner)
            )
            physical_regions.update(
                region_id
                for region_id, owner_names
                in self._controlled_corridor_occupancy.items()
                if owner_name in owner_names
            )
            if not physical_regions:
                continue
            moves_away = False
            for sample in blocker.trajectory:
                sample_clock = float(sample.get("t", 0.0) or 0.0)
                if sample_clock <= blocker.route_clock + 0.000001:
                    continue
                candidate_pose = {
                    "x": float(sample.get("x", 0.0) or 0.0),
                    "y": float(sample.get("y", 0.0) or 0.0),
                    "yaw": float(
                        sample.get(
                            "yaw",
                            blocker.pose.get("yaw", 0.0),
                        )
                        or 0.0
                    ),
                }
                if self._candidate_moves_away(
                    blocker.pose,
                    candidate_pose,
                    owner.pose,
                ):
                    moves_away = True
                    break
            if not moves_away:
                continue
            slot = (
                schedule.slot_for(owner_name)
                if schedule is not None
                else None
            )
            candidates.append(
                (
                    float(slot.entry_time if slot is not None else 0.0),
                    blocker.name,
                    blocker,
                )
            )
        return min(candidates)[2] if candidates else None

    def _controlled_corridor_follower_yields_to(
        self,
        follower: FleetRobot,
        leader: FleetRobot,
        passage: dict[str, Any],
    ) -> bool:
        """Return whether an external passage owner is behind its dependency."""
        if follower.name == leader.name or bool(passage.get("entered")):
            return False
        regions = {
            str(region_id)
            for region_id in passage.get("regions", ())
            if str(region_id)
        }
        if (
            not regions
            or self._controlled_regions_for_robot(follower).intersection(
                regions
            )
        ):
            return False
        dependency_name = (
            str(follower.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(
                follower.last_reason
            )
        )
        direct_reason = str(follower.last_reason or "").strip().lower()
        if (
            dependency_name != leader.name
            or not direct_reason.startswith(
                f"occupied by {leader.name.lower()}"
            )
        ):
            return False
        follower_entry = self._next_controlled_corridor_entry(follower)
        leader_entry = self._next_controlled_corridor_entry(leader)
        if not isinstance(follower_entry, dict) or not isinstance(
            leader_entry,
            dict,
        ):
            return False
        follower_regions = set(
            self._controlled_corridor_entry_regions(follower_entry)
        )
        leader_regions = set(
            self._controlled_corridor_entry_regions(leader_entry)
        )
        entry_lm = str(follower_entry.get("src") or "")
        entry = self.landmarks.get(entry_lm)
        if (
            entry is None
            or follower.pose is None
            or leader.pose is None
        ):
            return False
        follower_distance = math.hypot(
            float(follower.pose.get("x", 0.0)) - float(entry.x),
            float(follower.pose.get("y", 0.0)) - float(entry.y),
        )
        leader_distance = math.hypot(
            float(leader.pose.get("x", 0.0)) - float(entry.x),
            float(leader.pose.get("y", 0.0)) - float(entry.y),
        )
        return bool(
            regions.intersection(follower_regions, leader_regions)
            and str(follower_entry.get("src") or "")
            == str(leader_entry.get("src") or "")
            and str(follower_entry.get("dst") or "")
            == str(leader_entry.get("dst") or "")
            and leader_distance + 0.001 < follower_distance
        )

    def _cycle_forward_clearance(
        self,
        robot: FleetRobot,
        cycle_robots: list[FleetRobot],
    ) -> float:
        """Measure how far this candidate can move past stationary cycle peers."""
        if not robot.trajectory:
            return 0.0
        final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
        # Arbitration must see beyond the ordinary braking preview. In a
        # perpendicular crossing both candidates can look clear for the next
        # ~2 s, while only one can finish its committed rolling chunk without
        # entering the stationary peer's footprint. Chunks are already bounded
        # by the rolling horizon, so inspecting the remainder is cheap.
        horizon = final_time
        step = max(self._runtime_motion_step(), self.collision.sample_time_step())
        clock = min(horizon, robot.route_clock + step)
        while clock <= horizon + 0.000001:
            candidate = self._pose_at_trajectory(robot.trajectory, clock)
            if candidate is None:
                break
            if any(
                other.name != robot.name
                and other.pose is not None
                and self.collision.footprints_overlap(candidate, other.pose)
                for other in cycle_robots
            ):
                return max(0.0, clock - robot.route_clock - step)
            clock += step
        return max(0.0, horizon - robot.route_clock)

    def _deadlock_portal_queue_limits(self) -> tuple[int, float]:
        """Return bounded breadth/time for physical portal-queue recovery."""
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            configured_max = int(
                fleet.get(
                    "deadlock_portal_queue_max_robots",
                    self.planner.local_cbs_max_robots,
                )
                or self.planner.local_cbs_max_robots
            )
        except (TypeError, ValueError):
            configured_max = self.planner.local_cbs_max_robots
        try:
            lookahead = float(
                fleet.get("deadlock_portal_queue_lookahead_sec", 4.0)
                or 4.0
            )
        except (TypeError, ValueError):
            lookahead = 4.0
        return (
            max(
                2,
                min(
                    12,
                    int(self.planner.local_cbs_max_robots),
                    configured_max,
                ),
            ),
            max(1.0, min(10.0, lookahead)),
        )

    def _controlled_corridor_portal_queue_component(
        self,
        robots: list[FleetRobot],
        winner: FleetRobot,
    ) -> tuple[list[FleetRobot], dict[str, int]]:
        """Discover only the physical tail feeding a blocked corridor mouth.

        The wait-for cycle contains the entered owner and the body directly at
        its exit.  Admission losers behind that body commonly all point to the
        *owner*, so dependency strings alone either miss the tail or pull in
        every remote queue for a bundled corridor.  Instead follow current
        bodies along each waiter's short committed forward trajectory.  Each
        newly discovered waiter must physically run into an already selected
        external member before it can reach the portal.
        """
        component: list[FleetRobot] = []
        by_name: dict[str, FleetRobot] = {}
        for robot in robots:
            if robot.name in by_name:
                continue
            by_name[robot.name] = robot
            component.append(robot)
        depths = {
            robot.name: (-1 if robot.name == winner.name else 0)
            for robot in component
        }
        external_names = {
            robot.name for robot in component if robot.name != winner.name
        }
        if not external_names:
            return component, depths

        max_robots, lookahead = self._deadlock_portal_queue_limits()
        while len(component) < max_robots:
            additions: list[tuple[int, str, FleetRobot]] = []
            for candidate in self._runtime_robots():
                if (
                    candidate.name in by_name
                    or candidate.status != "WAITING"
                    or not candidate.trajectory
                    or not candidate.active_order_id
                    or not self._is_robot_conflict(candidate.last_reason)
                ):
                    continue
                final_clock = float(
                    candidate.trajectory[-1].get("t", candidate.route_clock)
                    or candidate.route_clock
                )
                check_until = min(
                    final_clock,
                    float(candidate.route_clock) + lookahead,
                )
                blocker_name = self._trajectory_current_body_blocker(
                    candidate,
                    candidate.trajectory,
                    float(candidate.route_clock),
                    check_until,
                )
                if blocker_name not in external_names:
                    continue
                additions.append((
                    depths.get(blocker_name, 0) + 1,
                    candidate.name,
                    candidate,
                ))
            if not additions:
                break
            added = False
            for depth, _, candidate in sorted(additions):
                if candidate.name in by_name or len(component) >= max_robots:
                    continue
                by_name[candidate.name] = candidate
                component.append(candidate)
                depths[candidate.name] = max(1, int(depth))
                external_names.add(candidate.name)
                added = True
            if not added:
                break
        return component, depths


__all__ = ['CorridorOwnershipMixin']
