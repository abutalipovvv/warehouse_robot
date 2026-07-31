from __future__ import annotations

import math
import random
from types import SimpleNamespace
from typing import Any

from fleet_manager.core.traffic.controlled_corridor_admission import (
    ControlledCorridorAdmissionMixin,
)
from fleet_manager.core.traffic.controlled_corridor_passage import (
    ControlledCorridorPassageMixin,
)
from fleet_manager.core.traffic.controlled_corridor_prefetch import (
    ControlledCorridorPrefetchMixin,
)
from fleet_manager.core.traffic.corridor_scheduler import CorridorRequest
from fleet_manager.core.traffic.rolling_route_helpers import RollingRouteMixin
from fleet_manager.core.traffic.routing import TrafficRoutingMixin
from fleet_manager.core.traffic.spatial_detours import SpatialDetourMixin
from fleet_manager.core.traffic.traffic_zone_admission import (
    TrafficZoneAdmissionMixin,
)


def _legacy_chunk_end_index(
    trajectory: list[dict[str, Any]],
    chunk_goal: str,
    arrival_time: float,
) -> int | None:
    if not trajectory:
        return None
    candidates = [
        index
        for index, sample in enumerate(trajectory)
        if str(sample.get("lm") or "").strip() == chunk_goal
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda index: (
            abs(
                float(trajectory[index].get("t", 0.0) or 0.0)
                - arrival_time
            ),
            index,
        ),
    )


def _legacy_entry_regions(entry: dict[str, Any] | None) -> tuple[str, ...]:
    if not isinstance(entry, dict):
        return ()
    raw = entry.get("regions")
    if isinstance(raw, (list, tuple)):
        regions = tuple(str(item) for item in raw if str(item))
        if regions:
            return tuple(dict.fromkeys(regions))
    region = str(entry.get("region") or "")
    return (region,) if region else ()


def _legacy_corridor_bounds(
    nodes: list[str],
    gate: dict[str, Any] | None,
) -> tuple[int, int] | None:
    if not isinstance(gate, dict):
        return None
    intent = gate.get("intent")
    if not isinstance(intent, dict):
        return None
    request = intent.get("request")
    entry = intent.get("entry")
    if not isinstance(request, CorridorRequest) or not isinstance(entry, dict):
        return None
    src = str(entry.get("src") or "")
    dst = str(entry.get("dst") or "")
    transition_index = next(
        (
            index
            for index in range(1, len(nodes))
            if nodes[index - 1] == src and nodes[index] == dst
        ),
        None,
    )
    if transition_index is None:
        return None
    staging_indices = [
        index
        for index, node in enumerate(nodes[:transition_index])
        if node == request.staging_lm
    ]
    if not staging_indices:
        return None
    exit_index = next(
        (
            index
            for index in range(transition_index, len(nodes))
            if nodes[index] == request.exit_lm
        ),
        None,
    )
    if exit_index is None:
        return None
    return staging_indices[-1], exit_index


def test_routing_facade_composes_focused_components() -> None:
    assert TrafficRoutingMixin.__bases__ == (
        SpatialDetourMixin,
        ControlledCorridorPassageMixin,
        ControlledCorridorPrefetchMixin,
        ControlledCorridorAdmissionMixin,
        TrafficZoneAdmissionMixin,
        RollingRouteMixin,
    )
    assert (
        TrafficRoutingMixin._ensure_order_spatial_route
        is SpatialDetourMixin._ensure_order_spatial_route
    )
    assert (
        TrafficRoutingMixin._next_controlled_corridor_entry
        is ControlledCorridorPassageMixin._next_controlled_corridor_entry
    )
    assert (
        TrafficRoutingMixin._controlled_corridor_prefetch_gate
        is ControlledCorridorPrefetchMixin._controlled_corridor_prefetch_gate
    )
    assert (
        TrafficRoutingMixin._prepare_controlled_corridor_admissions
        is ControlledCorridorAdmissionMixin._prepare_controlled_corridor_admissions
    )
    assert (
        TrafficRoutingMixin._prepare_traffic_zone_admissions
        is TrafficZoneAdmissionMixin._prepare_traffic_zone_admissions
    )
    assert (
        TrafficRoutingMixin._rolling_result
        is RollingRouteMixin._rolling_result
    )


def test_seeded_route_boundaries_match_legacy_algorithms() -> None:
    rng = random.Random(20260731)
    names = ("A", "B", "C", "D", "", None)
    for _ in range(2_000):
        trajectory = [
            {
                "lm": rng.choice(names),
                "t": rng.uniform(0.0, 120.0),
            }
            for _ in range(rng.randrange(0, 80))
        ]
        goal = rng.choice(("A", "B", "C", "D", "missing"))
        arrival = rng.uniform(-10.0, 140.0)

        actual = RollingRouteMixin._trajectory_chunk_end_index(
            object(),
            trajectory,
            goal,
            arrival,
        )

        assert actual == _legacy_chunk_end_index(
            trajectory,
            goal,
            arrival,
        )


def test_seeded_corridor_helpers_match_legacy_algorithms() -> None:
    rng = random.Random(20260732)
    alphabet = ("A", "B", "C", "D", "E")
    for index in range(2_000):
        nodes = [rng.choice(alphabet) for _ in range(rng.randrange(0, 24))]
        request = CorridorRequest(
            robot_id=f"robot-{index}",
            regions=("corridor:main",),
            direction="east",
            earliest_entry=0.0,
            duration_sec=5.0,
            staging_lm=rng.choice(alphabet),
            exit_lm=rng.choice(alphabet),
            route_revision=index,
        )
        entry = {
            "src": rng.choice(alphabet),
            "dst": rng.choice(alphabet),
        }
        gate: dict[str, Any] | None = {
            "intent": {"request": request, "entry": entry}
        }
        if rng.random() < 0.08:
            gate = None

        assert RollingRouteMixin._corridor_plan_bounds(nodes, gate) == (
            _legacy_corridor_bounds(nodes, gate)
        )

        raw_regions = [rng.choice(("", "a", "b", "c")) for _ in range(8)]
        corridor_entry: dict[str, Any] | None = {
            "regions": raw_regions,
            "region": rng.choice(("", "fallback")),
        }
        if rng.random() < 0.08:
            corridor_entry = None
        assert (
            ControlledCorridorPassageMixin._controlled_corridor_entry_regions(
                corridor_entry
            )
            == _legacy_entry_regions(corridor_entry)
        )


def test_spatial_parameters_and_zone_grid_keep_legacy_semantics() -> None:
    rng = random.Random(20260733)
    invalid_values: tuple[Any, ...] = (None, "", "bad", object())
    for _ in range(1_000):
        default = rng.uniform(-5.0, 10.0)
        raw: Any = (
            rng.uniform(-10.0, 20.0)
            if rng.random() < 0.7
            else rng.choice(invalid_values)
        )
        values = {"value": raw}
        try:
            expected = max(0.0, float(default if raw is None else raw))
        except (TypeError, ValueError):
            expected = max(0.0, float(default))

        assert SpatialDetourMixin._positive_float_param(
            object(),
            values,
            "value",
            default,
        ) == expected

    class ZoneHarness(TrafficZoneAdmissionMixin, SpatialDetourMixin):
        params = {
            "fleet": {
                "traffic_zone_control_enabled": True,
                "traffic_zone_size_m": 2.5,
            }
        }
        landmarks = {
            "origin": SimpleNamespace(x=-1.0, y=-4.0),
            "east": SimpleNamespace(x=4.1, y=-4.0),
            "south": SimpleNamespace(x=-1.0, y=1.1),
        }

    assert ZoneHarness()._build_traffic_zone_index() == {
        "origin": "flow:0:0",
        "east": f"flow:{math.floor(5.1 / 2.5)}:0",
        "south": f"flow:0:{math.floor(5.1 / 2.5)}",
    }
