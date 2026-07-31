"""Corridor clearance holds and recovery latches."""

from __future__ import annotations

from typing import Any

from fleet_manager.core.models import FleetRobot


class EvacuationLatchMixin:
    """Keep one physical corridor recovery active until clearance is proven."""

    def _corridor_clearance_hold_for(
        self,
        winner: FleetRobot,
        winner_regions: set[str],
    ) -> dict[str, Any] | None:
        """Capture the local resource an evacuated portal tail must await."""
        physical_regions = set(self._controlled_regions_for_robot(winner))
        for region_id, owners in self._controlled_corridor_occupancy.items():
            if winner.name in owners:
                physical_regions.add(str(region_id))
        regions = physical_regions or set(winner_regions)
        if not regions:
            return None
        return {
            "owner": winner.name,
            "owner_order_id": str(winner.active_order_id or ""),
            "regions": tuple(sorted(regions)),
            "physical_only": bool(physical_regions),
        }

    def _corridor_clearance_hold_active(
        self,
        hold: dict[str, Any],
        cleared_robot_name: str = "",
    ) -> bool:
        """Return whether a captured passage owner still occupies its mouth."""
        owner_name = str(hold.get("owner") or "")
        owner = self.robots.get(owner_name)
        if owner is None:
            return False
        owner_order_id = str(hold.get("owner_order_id") or "")
        if owner_order_id and owner.active_order_id != owner_order_id:
            return False
        owner_dependency = (
            str(owner.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(owner.last_reason)
        )
        if (
            cleared_robot_name
            and owner_dependency == cleared_robot_name
        ):
            # The selected historical LM was graph-safe, but the owner's
            # actual exit suffix may continue along the same external aisle.
            # In that geometry the held robot is still the direct blocker;
            # keeping it frozen until the owner exits is circular. Release its
            # transactional replan so SIPP can move it to another branch.
            return False
        regions = {
            str(region_id)
            for region_id in hold.get("regions", ())
            if str(region_id)
        }
        if not regions:
            return False
        physical = set(self._controlled_regions_for_robot(owner))
        for region_id, owners in self._controlled_corridor_occupancy.items():
            if owner_name in owners:
                physical.add(str(region_id))
        if physical.intersection(regions):
            return True
        if bool(hold.get("physical_only")):
            return False
        passage = self._controlled_corridor_passages.get(owner_name)
        if not isinstance(passage, dict):
            return False
        passage_regions = {
            str(region_id)
            for region_id in passage.get("regions", ())
            if str(region_id)
        }
        return bool(
            passage_regions.intersection(regions)
            and (
                bool(passage.get("entered"))
                or bool(passage.get("committed"))
            )
        )

    def _controlled_corridor_recovery_physical_regions(
        self,
        robot: FleetRobot,
    ) -> set[str]:
        """Return authored regions which this body physically owns."""
        regions = set(self._controlled_regions_for_robot(robot))
        regions.update(
            str(region_id)
            for region_id, owners
            in self._controlled_corridor_occupancy.items()
            if robot.name in owners
        )
        passage = self._controlled_corridor_passages.get(robot.name)
        if (
            isinstance(passage, dict)
            and (
                bool(passage.get("entered"))
                or bool(passage.get("past_commit_point"))
            )
        ):
            regions.update(
                str(region_id)
                for region_id in passage.get("regions", ())
                if str(region_id)
            )
        return regions

    def _prune_controlled_corridor_recovery_latches(self) -> None:
        for key in list(self._controlled_corridor_recovery_latches):
            region_ids, owner_name, order_id, route_revision = key
            owner = self.robots.get(owner_name)
            if (
                owner is None
                or str(owner.active_order_id or "") != order_id
                or int(owner.route_revision) != route_revision
                or not set(region_ids).intersection(
                    self._controlled_corridor_recovery_physical_regions(
                        owner
                    )
                )
            ):
                self._controlled_corridor_recovery_latches.pop(key, None)

    @staticmethod
    def _controlled_corridor_recovery_latch_key(
        owner: FleetRobot,
        region_ids: set[str],
    ) -> tuple[tuple[str, ...], str, str, int]:
        return (
            tuple(sorted(region_ids)),
            owner.name,
            str(owner.active_order_id or ""),
            int(owner.route_revision),
        )

    def _latch_controlled_corridor_recovery(
        self,
        key: tuple[tuple[str, ...], str, str, int] | None,
        victim_name: str,
    ) -> None:
        if key is not None and victim_name:
            self._controlled_corridor_recovery_latches[key] = victim_name


__all__ = ['EvacuationLatchMixin']
