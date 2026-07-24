"""Canonical task store shared by every fleet runtime."""

from __future__ import annotations

from collections.abc import Callable, Iterable

from fleet_manager.core.constants import TERMINAL_ORDER_STATUSES
from fleet_manager.core.models import FleetOrder


class FleetTaskManager:
    """Own orders independently from simulation or robot transport.

    Scheduling algorithms still live in ``FleetManagerCore`` for now, while
    this class provides the stable storage/query boundary used during their
    incremental extraction.
    """

    STATUS_RANK = {
        "EXECUTING": 0,
        "WAITING_TRAFFIC": 1,
        "WAITING_OBSTACLE": 1,
        "PLANNING": 2,
        "PAUSED": 2,
        "ASSIGNED": 2,
        "QUEUED": 3,
        "COMPLETED": 4,
        "FAILED": 5,
        "CANCELED": 6,
    }

    def __init__(self, orders: dict[str, FleetOrder] | None = None) -> None:
        self.orders = orders if orders is not None else {}

    def replace_storage(self, orders: dict[str, FleetOrder]) -> None:
        self.orders = orders

    def active_for_robot(
        self,
        robot_name: str,
        *,
        preferred_order_id: str = "",
    ) -> FleetOrder | None:
        if preferred_order_id:
            preferred = self.orders.get(preferred_order_id)
            if (
                preferred is not None
                and preferred.status not in TERMINAL_ORDER_STATUSES
            ):
                return preferred
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            if order.assigned_robot == robot_name or order.vehicle == robot_name:
                return order
        return None

    def ordered_payloads(
        self,
        *,
        enabled: Callable[[FleetOrder], bool] | None = None,
        limit: int = 120,
    ) -> list[dict[str, object]]:
        predicate = enabled or (lambda _order: True)
        ordered = sorted(
            (order for order in self.orders.values() if predicate(order)),
            key=lambda order: (
                self.STATUS_RANK.get(order.status, 9),
                -int(order.priority or 0),
                order.created_at,
            ),
        )
        return [order.to_dict() for order in ordered[:max(0, int(limit))]]

    def pending_for_robot(self, robot_name: str) -> list[FleetOrder]:
        pending = [
            order
            for order in self.orders.values()
            if order.status not in TERMINAL_ORDER_STATUSES
            and (order.vehicle == robot_name or order.assigned_robot == robot_name)
        ]
        pending.sort(key=lambda order: (order.created_at, order.order_id))
        return pending

    def pending_by_robot(self) -> dict[str, list[FleetOrder]]:
        """Build one active-order index for a complete fleet snapshot.

        Calling ``pending_for_robot`` once per robot made every websocket
        snapshot O(robots * lifetime_orders). Lifelong benchmarks retain a
        bounded terminal history for the operator, so that repeated scan
        gradually stole the runtime thread from rolling prefetch.
        """
        pending: dict[str, list[FleetOrder]] = {}
        for order in self.orders.values():
            if order.status in TERMINAL_ORDER_STATUSES:
                continue
            owners = {
                order.vehicle,
                order.assigned_robot,
            }
            owners.discard("")
            owners.discard(None)
            for owner in owners:
                pending.setdefault(owner, []).append(order)
        for orders in pending.values():
            orders.sort(key=lambda order: (order.created_at, order.order_id))
        return pending

    def prune_terminal_history(self, limit: int = 120) -> tuple[str, ...]:
        """Drop terminal records older than the bounded operator history.

        The task store is also the dispatcher's working set.  Keeping every
        completed order forever therefore makes otherwise constant-time
        lifelong operation progressively scan the complete order history.
        Active orders are never removed; among terminal records the newest
        ``limit`` entries are retained for the operator/audit view.
        """
        maximum = max(0, int(limit))
        terminal = sorted(
            (
                order
                for order in self.orders.values()
                if order.status in TERMINAL_ORDER_STATUSES
            ),
            key=lambda order: (order.updated_at, order.order_id),
            reverse=True,
        )
        removed: list[str] = []
        for order in terminal[maximum:]:
            if self.orders.get(order.order_id) is not order:
                continue
            self.orders.pop(order.order_id, None)
            removed.append(order.order_id)
        return tuple(removed)

    def clear(self) -> None:
        self.orders.clear()

    def values(self) -> Iterable[FleetOrder]:
        return self.orders.values()


__all__ = ["FleetTaskManager"]
