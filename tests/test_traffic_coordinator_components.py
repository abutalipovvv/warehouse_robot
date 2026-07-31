from __future__ import annotations

import random

from fleet_manager.core.models import FleetRobot
from fleet_manager.core.traffic.coordinator import TrafficCoordinatorMixin
from fleet_manager.core.traffic.deadlock_arbitration import (
    DeadlockArbitrationMixin,
)
from fleet_manager.core.traffic.deadlock_evacuation import (
    DeadlockEvacuationMixin,
    _EvacuationCandidate,
)
from fleet_manager.core.traffic.runtime_conflicts import RuntimeConflictMixin
from fleet_manager.core.traffic.wait_graph import WaitForGraph


def _legacy_cycles(dependencies: dict[str, str]) -> list[tuple[str, ...]]:
    handled: set[str] = set()
    cycles: list[tuple[str, ...]] = []
    for start_name in sorted(dependencies):
        chain: list[str] = []
        positions: dict[str, int] = {}
        current = start_name
        while current in dependencies and current not in handled:
            if current in positions:
                cycle = tuple(chain[positions[current]:])
                cycles.append(cycle)
                handled.update(cycle)
                break
            positions[current] = len(chain)
            chain.append(current)
            current = dependencies[current]
        handled.update(chain)
    return cycles


def _legacy_walk(
    dependencies: dict[str, str],
    start_name: str,
) -> tuple[tuple[str, ...], str]:
    chain: list[str] = []
    current = start_name
    while current in dependencies and current not in chain:
        chain.append(current)
        current = dependencies.get(current, "")
    return tuple(chain), current


def test_wait_graph_reports_cycles_and_upstream_tail() -> None:
    graph = WaitForGraph(
        {
            "a": "b",
            "b": "a",
            "tail": "a",
            "free": "outside",
            "self": "self",
        }
    )

    assert list(graph.cycles()) == [("a", "b"), ("self",)]
    tail = graph.walk("tail")
    assert tail.members == ("tail", "a", "b")
    assert tail.terminal == "a"


def test_wait_graph_matches_seeded_legacy_traversal() -> None:
    rng = random.Random(20260731)
    for _ in range(2_000):
        names = tuple(f"r{index}" for index in range(rng.randrange(1, 50)))
        targets = (*names, "outside", "")
        dependencies = {
            name: rng.choice(targets)
            for name in names
            if rng.random() < 0.85
        }
        graph = WaitForGraph(dependencies)

        assert list(graph.cycles()) == _legacy_cycles(dependencies)
        for start_name in names:
            walk = graph.walk(start_name)
            assert (walk.members, walk.terminal) == _legacy_walk(
                dependencies,
                start_name,
            )


def test_compatibility_mixin_composes_all_private_hook_components() -> None:
    assert issubclass(TrafficCoordinatorMixin, DeadlockArbitrationMixin)
    assert issubclass(TrafficCoordinatorMixin, DeadlockEvacuationMixin)
    assert issubclass(TrafficCoordinatorMixin, RuntimeConflictMixin)
    assert (
        TrafficCoordinatorMixin._resolve_runtime_wait_cycles
        is DeadlockArbitrationMixin._resolve_runtime_wait_cycles
    )
    assert (
        TrafficCoordinatorMixin._start_deadlock_corridor_evacuation
        is DeadlockEvacuationMixin._start_deadlock_corridor_evacuation
    )
    assert (
        TrafficCoordinatorMixin._blocked_at_clock
        is RuntimeConflictMixin._blocked_at_clock
    )


def test_evacuation_candidate_keeps_legacy_tuple_ordering() -> None:
    first = FleetRobot(name="a", current_lm="A")
    second = FleetRobot(name="b", current_lm="B")

    candidates = [
        _EvacuationCandidate(
            distance=2.0,
            priority=0,
            robot_name=first.name,
            robot=first,
            target_clock=0.0,
            target_lm="A",
            retreat_is_noop_at_current_lm=False,
            graph_escape_route=[],
            portal_blocked_edges=[],
        ),
        _EvacuationCandidate(
            distance=1.0,
            priority=100,
            robot_name=second.name,
            robot=second,
            target_clock=0.0,
            target_lm="B",
            retreat_is_noop_at_current_lm=False,
            graph_escape_route=[],
            portal_blocked_edges=[],
        ),
    ]

    assert min(candidates).robot is second
