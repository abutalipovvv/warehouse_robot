from __future__ import annotations

from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.coordination.coordinator import TrafficCoordinatorMixin
from fleet_manager.manager.coordination.deadlocks.corridor_ownership import (
    CorridorOwnershipMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery import (
    WaitCycleRecoveryMixin,
)
from fleet_manager.manager.coordination.deadlocks.escape_install import (
    GraphEscapeInstallMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation import (
    EvacuationActivationMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation_candidates import (
    EvacuationCandidateMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation_geometry import (
    EvacuationGeometryMixin,
)
from fleet_manager.manager.coordination.deadlocks.evacuation_latches import (
    EvacuationLatchMixin,
)
from fleet_manager.manager.coordination.deadlocks.models import (
    _EvacuationCandidate as EvacuationCandidateModel,
)
from fleet_manager.manager.coordination.deadlocks.leases import DeadlockLeaseMixin
from fleet_manager.manager.coordination.deadlocks.policy import DeadlockPolicyMixin
from fleet_manager.manager.coordination.deadlocks.priority import (
    DeadlockPriorityMixin,
)
from fleet_manager.manager.coordination.deadlocks.detection import (
    WaitCycleDetectionMixin,
    _RuntimeWaitSnapshot as RuntimeWaitSnapshotModel,
)

def test_arbitration_facade_composes_focused_decision_stages() -> None:
    arbitration_bases = (
        WaitCycleDetectionMixin,
        DeadlockPriorityMixin,
        WaitCycleRecoveryMixin,
        CorridorOwnershipMixin,
        DeadlockLeaseMixin,
        DeadlockPolicyMixin,
    )
    assert all(owner in TrafficCoordinatorMixin.__bases__ for owner in arbitration_bases)
    assert (
        TrafficCoordinatorMixin._resolve_runtime_wait_cycles
        is WaitCycleDetectionMixin._resolve_runtime_wait_cycles
    )
    assert (
        TrafficCoordinatorMixin._grant_wait_chain_priority
        is DeadlockPriorityMixin._grant_wait_chain_priority
    )
    assert (
        TrafficCoordinatorMixin._break_runtime_wait_cycle
        is WaitCycleRecoveryMixin._break_runtime_wait_cycle
    )
    assert (
        TrafficCoordinatorMixin._controlled_corridor_cycle_owner
        is CorridorOwnershipMixin._controlled_corridor_cycle_owner
    )
    assert (
        TrafficCoordinatorMixin._maintain_runtime_wait_cycle_lease
        is DeadlockLeaseMixin._maintain_runtime_wait_cycle_lease
    )
    assert (
        TrafficCoordinatorMixin._deadlock_retreat_after
        is DeadlockPolicyMixin._deadlock_retreat_after
    )
    assert RuntimeWaitSnapshotModel.__module__.endswith(
        "detection"
    )


def test_evacuation_facade_composes_focused_recovery_stages() -> None:
    evacuation_bases = (
        EvacuationGeometryMixin,
        EvacuationLatchMixin,
        EvacuationCandidateMixin,
        EvacuationActivationMixin,
        GraphEscapeInstallMixin,
    )
    assert all(owner in TrafficCoordinatorMixin.__bases__ for owner in evacuation_bases)
    assert (
        TrafficCoordinatorMixin._graph_escape_route_current_body_blocker
        is EvacuationGeometryMixin._graph_escape_route_current_body_blocker
    )
    assert (
        TrafficCoordinatorMixin._corridor_clearance_hold_active
        is EvacuationLatchMixin._corridor_clearance_hold_active
    )
    assert (
        TrafficCoordinatorMixin._build_deadlock_evacuation_candidates
        is EvacuationCandidateMixin._build_deadlock_evacuation_candidates
    )
    assert (
        TrafficCoordinatorMixin._activate_deadlock_evacuation
        is EvacuationActivationMixin._activate_deadlock_evacuation
    )
    assert (
        TrafficCoordinatorMixin._install_graph_escape_retreat
        is GraphEscapeInstallMixin._install_graph_escape_retreat
    )
    assert EvacuationCandidateModel.__module__.endswith(
        "models"
    )


def test_corridor_recovery_latch_key_remains_static_and_deterministic() -> None:
    owner = FleetRobot(
        name="robot-b",
        current_lm="B",
        active_order_id="order-7",
        route_revision=12,
    )

    assert TrafficCoordinatorMixin._controlled_corridor_recovery_latch_key(
        owner,
        {"region-z", "region-a"},
    ) == (
        ("region-a", "region-z"),
        "robot-b",
        "order-7",
        12,
    )
