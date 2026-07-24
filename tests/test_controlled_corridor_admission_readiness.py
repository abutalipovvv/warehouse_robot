from __future__ import annotations

from types import SimpleNamespace

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.simulation.manager import FleetManagerSim


def _manager() -> FleetManagerSim:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    manager = FleetManagerSim(
        landmarks,
        [
            GraphEdge(
                from_name="A",
                to_name="B",
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(0.0, 0.0),
                    WorldPoint(1.0, 0.0),
                ),
                properties={"direction": 2},
            )
        ],
        params={"fleet": {"traffic_zone_control_enabled": False}},
    )
    manager._controlled_corridor_graph = SimpleNamespace()
    return manager


def _robot(name: str, *, status: str = "MOVING") -> FleetRobot:
    return FleetRobot(
        name=name,
        current_lm="A",
        target_lm="B",
        status=status,
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        trajectory=[
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "A",
            },
            {
                "t": 10.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": "A->B",
                "lm": "B",
            },
        ],
    )


def test_unmarked_map_never_honors_legacy_corridor_leases() -> None:
    manager = _manager()
    owner = _robot("owner")
    manager.robots = {owner.name: owner}
    regions = ("corridor:a", "corridor:b")
    manager._controlled_corridor_leases = {
        region: (owner.name, 2_000.0)
        for region in regions
    }

    manager._prepare_controlled_corridor_admissions(1_000.0)

    assert not manager._controlled_corridor_has_grant(owner.name, regions)
    assert manager._controlled_corridor_leases == {}
    assert manager._controlled_corridor_passages == {}
    assert manager._controlled_corridor_queues == {}


def test_corridor_owner_finishes_at_safe_exit_before_future_reentry() -> None:
    old_region = "corridor:old"
    next_region = "corridor:next"
    coordinates = {
        "H": -4.0,
        "A": -1.0,
        "P": 0.0,
        "B": 1.0,
        "X": 2.0,
    }
    landmarks = {
        name: Landmark(
            name=name,
            x=x,
            y=0.0,
            properties=(
                {
                    "can_wait": False,
                    "controlled_region": (
                        old_region if name == "A" else next_region
                    ),
                }
                if name in {"A", "B"}
                else {"can_wait": True}
            ),
        )
        for name, x in coordinates.items()
    }
    edge_regions = (
        ("H", "A", old_region),
        ("A", "P", old_region),
        ("P", "B", next_region),
        ("B", "X", next_region),
    )
    manager = FleetManagerSim(
        landmarks,
        [
            GraphEdge(
                from_name=src,
                to_name=dst,
                length=abs(coordinates[dst] - coordinates[src]),
                kind="line",
                edge_type="FeatureLine",
                world_points=(
                    WorldPoint(coordinates[src], 0.0),
                    WorldPoint(coordinates[dst], 0.0),
                ),
                properties={
                    "direction": 2,
                    "controlled_region": region,
                },
            )
            for src, dst, region in edge_regions
        ],
        params={"fleet": {"traffic_zone_control_enabled": False}},
    )
    nodes = ["H", "A", "P", "B", "X"]
    owner = FleetRobot(
        name="owner",
        current_lm="A",
        target_lm="X",
        status="MOVING",
        pose={"x": -1.0, "y": 0.0, "yaw": 0.0},
        route_clock=1.0,
        trajectory=[
            {
                "t": float(index),
                "x": coordinates[node],
                "y": 0.0,
                "yaw": 0.0,
                "edgeId": (
                    "H->A"
                    if index == 0
                    else f"{nodes[index - 1]}->{node}"
                ),
                "lm": node,
            }
            for index, node in enumerate(nodes)
        ],
    )
    manager.robots = {owner.name: owner}

    assert manager._controlled_regions_for_robot(owner) == {old_region}
    assert manager._next_controlled_corridor_entry(owner) is None
    assert manager._controlled_corridor_admission_reason(owner, 1.05) == ""


def test_corridor_diagnostic_never_creates_self_wait_dependency() -> None:
    manager = _manager()
    owner = _robot("owner", status="WAITING")
    manager.robots = {owner.name: owner}

    manager._set_wait_dependency(
        owner,
        "corridor admission wait at A for corridor:narrow; owner owner",
        1_000.0,
    )

    assert owner.wait_for_robot == ""
    assert owner.wait_resource == ""
    assert owner.wait_release_at == 0.0
