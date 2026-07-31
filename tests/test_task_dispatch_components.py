from __future__ import annotations

import random
from threading import Lock
from types import SimpleNamespace
from typing import Any

from fleet_manager.core.constants import ORDER_SEQUENCE_KEYS, ORDER_TARGET_KEYS
from fleet_manager.core.models import FleetOrder
from fleet_manager.core.tasks.dispatch import FleetTaskDispatchMixin
from fleet_manager.core.tasks.dispatch_requests import DispatchRequestBatchMixin
from fleet_manager.core.tasks.dispatch_results import DispatchResultMixin
from fleet_manager.core.tasks.order_admission import OrderAdmissionMixin
from fleet_manager.core.tasks.order_lifecycle import OrderLifecycleMixin
from fleet_manager.core.tasks.planning_jobs import AsyncPlanningJobMixin
from fleet_manager.core.tasks.rolling_continuation import (
    RollingContinuationMixin,
)
from fleet_manager.core.tasks.runtime_replans import RuntimeReplanMixin
from fleet_manager.core.tasks.stationary_blockers import (
    StationaryBlockerRecoveryMixin,
)
from fleet_manager.core.tasks.stationary_clearance import (
    StationaryClearanceMixin,
)


def _legacy_payload_target(item: Any) -> str:
    if isinstance(item, dict):
        for key in ORDER_TARGET_KEYS:
            target = str(item.get(key) or "").strip()
            if target:
                return target
        return ""
    return str(item or "").strip()


def _legacy_payload_targets(payload: dict[str, Any]) -> list[str]:
    targets: list[str] = []
    for key in ORDER_SEQUENCE_KEYS:
        raw_sequence = payload.get(key)
        if not isinstance(raw_sequence, list):
            continue
        for item in raw_sequence:
            target = _legacy_payload_target(item)
            if target:
                targets.append(target)
        if targets:
            return targets
    target = _legacy_payload_target(payload)
    return [target] if target else []


def _legacy_failure_resource(reason: str) -> str:
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
        if marker not in str(reason or ""):
            continue
        resource = str(reason).rsplit(marker, 1)[-1].split("@", 1)[0].strip()
        if resource:
            return resource
    return ""


def test_dispatch_facade_composes_all_capabilities() -> None:
    assert FleetTaskDispatchMixin.__bases__ == (
        OrderAdmissionMixin,
        StationaryBlockerRecoveryMixin,
        StationaryClearanceMixin,
        DispatchRequestBatchMixin,
        AsyncPlanningJobMixin,
        RuntimeReplanMixin,
        RollingContinuationMixin,
        DispatchResultMixin,
        OrderLifecycleMixin,
    )
    assert FleetTaskDispatchMixin._dispatch_orders is OrderAdmissionMixin._dispatch_orders
    assert (
        FleetTaskDispatchMixin._queue_commanded_sink_vacancy_replan
        is StationaryBlockerRecoveryMixin._queue_commanded_sink_vacancy_replan
    )
    assert (
        FleetTaskDispatchMixin._queue_stationary_clearance_relocation
        is StationaryClearanceMixin._queue_stationary_clearance_relocation
    )
    assert (
        FleetTaskDispatchMixin._prepare_simulated_order_batch
        is DispatchRequestBatchMixin._prepare_simulated_order_batch
    )
    assert (
        FleetTaskDispatchMixin._submit_async_planning_job
        is AsyncPlanningJobMixin._submit_async_planning_job
    )
    assert (
        FleetTaskDispatchMixin._defer_runtime_replan
        is RuntimeReplanMixin._defer_runtime_replan
    )
    assert (
        FleetTaskDispatchMixin._ready_rolling_prefetch_entries
        is RollingContinuationMixin._ready_rolling_prefetch_entries
    )
    assert (
        FleetTaskDispatchMixin._finish_simulated_order_batch
        is DispatchResultMixin._finish_simulated_order_batch
    )
    assert (
        FleetTaskDispatchMixin._set_order_status
        is OrderLifecycleMixin._set_order_status
    )


def test_seeded_payload_and_failure_helpers_match_legacy_algorithms() -> None:
    rng = random.Random(20260731)
    sequence_keys = tuple(ORDER_SEQUENCE_KEYS)
    target_keys = tuple(ORDER_TARGET_KEYS)
    values: tuple[Any, ...] = (
        None,
        "",
        " A ",
        "B",
        17,
        {"targetLm": "C"},
        {"goalLm": "D"},
        {"unknown": "E"},
    )
    markers = (
        "",
        "resource_constrained:",
        "reserved_edge:",
        "rotation_vertex_reserved:",
    )
    instance = FleetTaskDispatchMixin.__new__(FleetTaskDispatchMixin)

    for _ in range(2_000):
        payload: dict[str, Any] = {}
        if rng.random() < 0.75:
            payload[rng.choice(sequence_keys)] = [
                rng.choice(values) for _ in range(rng.randrange(0, 12))
            ]
        if rng.random() < 0.5:
            payload[rng.choice(target_keys)] = rng.choice(values)

        assert instance._target_lms_from_payload(payload) == (
            _legacy_payload_targets(payload)
        )

        marker = rng.choice(markers)
        reason = (
            f"planner:{marker}{rng.choice(('A', 'B->C', 'zone:4', ''))}"
            f"@{rng.uniform(0.0, 20.0):.3f}"
        )
        assert instance._runtime_replan_failure_resource(reason) == (
            _legacy_failure_resource(reason)
        )


def test_seeded_motion_keys_preserve_batch_equivalence() -> None:
    rng = random.Random(20260732)
    instance = FleetTaskDispatchMixin.__new__(FleetTaskDispatchMixin)
    for index in range(2_000):
        order = FleetOrder(
            order_id=f"order-{index}",
            target_lm="goal",
            speed=rng.uniform(-2.0, 5.0),
            acceleration=rng.uniform(-2.0, 5.0),
            rotate=bool(rng.randrange(2)),
            turn_speed=rng.uniform(-2.0, 5.0),
            stretch_motion_to_reservation_ticks=bool(rng.randrange(2)),
            traffic_detour_edges=[
                (
                    rng.choice(("A", "B", "C")),
                    rng.choice(("A", "B", "C")),
                )
                for _ in range(rng.randrange(0, 8))
            ],
        )
        expected = (
            round(float(order.speed), 6),
            round(float(order.acceleration), 6),
            bool(order.rotate),
            round(float(order.turn_speed), 6),
            bool(order.stretch_motion_to_reservation_ticks),
            tuple(sorted((str(src), str(dst)) for src, dst in order.traffic_detour_edges)),
        )
        assert instance._order_motion_key(order) == expected


def test_dispatch_cycle_keeps_phase_order_and_short_circuits() -> None:
    class Harness(OrderAdmissionMixin):
        def __init__(self, stop_at: str = "") -> None:
            self.stop_at = stop_at
            self.calls: list[str] = []

        def _prepare_dispatch_cycle(self, **_: Any) -> SimpleNamespace:
            self.calls.append("prepare")
            return SimpleNamespace(dispatched=7)

        def _start_dispatch_runtime_replan(self, cycle: Any) -> int | None:
            self.calls.append("runtime")
            return cycle.dispatched if self.stop_at == "runtime" else None

        def _collect_ready_dispatch_entries(self, _: Any) -> None:
            self.calls.append("collect")

        def _start_dispatch_prefetch(self, cycle: Any) -> int | None:
            self.calls.append("prefetch")
            return cycle.dispatched if self.stop_at == "prefetch" else None

        def _dispatch_ready_entry_batches(self, _: Any) -> int:
            self.calls.append("batches")
            return 2

        def _dispatch_remaining_orders(self, _: Any, calls: int) -> None:
            assert calls == 2
            self.calls.append("remaining")

    normal = Harness()
    assert normal._dispatch_orders(async_simulated=True) == 7
    assert normal.calls == [
        "prepare",
        "runtime",
        "collect",
        "prefetch",
        "batches",
        "remaining",
    ]

    runtime = Harness("runtime")
    assert runtime._dispatch_orders(async_simulated=True) == 7
    assert runtime.calls == ["prepare", "runtime"]

    prefetch = Harness("prefetch")
    assert prefetch._dispatch_orders(async_simulated=True) == 7
    assert prefetch.calls == ["prepare", "runtime", "collect", "prefetch"]


def test_planner_worker_publishes_only_to_the_current_job() -> None:
    class Worker:
        callback: Any = None

        def submit(self, callback: Any, *, thread_name: str) -> bool:
            assert thread_name == "planner-test"
            self.callback = callback
            return True

    instance = AsyncPlanningJobMixin.__new__(AsyncPlanningJobMixin)
    instance._dispatch_job_lock = Lock()
    instance._planning_worker = Worker()
    instance._plan_valid_requests = lambda requests, payload: {
        "ok": True,
        "plans": [requests, payload],
    }
    job = {"result": None, "done": False}
    instance._dispatch_job = job

    assert instance._submit_async_planning_job(
        job,
        [{"name": "robot"}],
        {"mode": "test"},
        failure_reason="failed",
        thread_name="planner-test",
    )

    replacement = {"result": None, "done": False}
    instance._dispatch_job = replacement
    instance._planning_worker.callback()
    assert job == {"result": None, "done": False}
    assert replacement == {"result": None, "done": False}
