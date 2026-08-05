from __future__ import annotations

import inspect

from fleet_manager.robot.model import FleetRobot
from fleet_manager.manager.coordination.deadlocks.arbitration.deadlock_arbitration import (
    DeadlockArbitrationMixin,
)
from fleet_manager.manager.coordination.deadlocks.arbitration.deadlock_corridor_ownership import (
    CorridorOwnershipMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.cycles.deadlock_cycle_recovery import (
    WaitCycleRecoveryMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_escape_install import (
    GraphEscapeInstallMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_evacuation import (
    DeadlockEvacuationMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_evacuation_activation import (
    EvacuationActivationMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_evacuation_candidates import (
    EvacuationCandidateMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_evacuation_geometry import (
    EvacuationGeometryMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_evacuation_latches import (
    EvacuationLatchMixin,
)
from fleet_manager.manager.coordination.deadlocks.recovery.evacuation.deadlock_evacuation_models import (
    _EvacuationCandidate as EvacuationCandidateModel,
)
from fleet_manager.manager.coordination.deadlocks.arbitration.deadlock_leases import DeadlockLeaseMixin
from fleet_manager.manager.coordination.deadlocks.arbitration.deadlock_policy import DeadlockPolicyMixin
from fleet_manager.manager.coordination.deadlocks.arbitration.deadlock_priority import (
    DeadlockPriorityMixin,
)
from fleet_manager.manager.coordination.deadlocks.arbitration.deadlock_wait_detection import (
    WaitCycleDetectionMixin,
    _RuntimeWaitSnapshot as RuntimeWaitSnapshotModel,
)


def _hooks(owner: type) -> set[str]:
    return {
        name
        for name, value in inspect.getmembers(owner, inspect.isfunction)
        if not name.startswith("__")
    }


def test_arbitration_facade_composes_focused_decision_stages() -> None:
    assert DeadlockArbitrationMixin.__bases__ == (
        WaitCycleDetectionMixin,
        DeadlockPriorityMixin,
        WaitCycleRecoveryMixin,
        CorridorOwnershipMixin,
        DeadlockLeaseMixin,
        DeadlockPolicyMixin,
    )
    assert len(_hooks(DeadlockArbitrationMixin)) == 45
    assert (
        DeadlockArbitrationMixin._resolve_runtime_wait_cycles
        is WaitCycleDetectionMixin._resolve_runtime_wait_cycles
    )
    assert (
        DeadlockArbitrationMixin._grant_wait_chain_priority
        is DeadlockPriorityMixin._grant_wait_chain_priority
    )
    assert (
        DeadlockArbitrationMixin._break_runtime_wait_cycle
        is WaitCycleRecoveryMixin._break_runtime_wait_cycle
    )
    assert (
        DeadlockArbitrationMixin._controlled_corridor_cycle_owner
        is CorridorOwnershipMixin._controlled_corridor_cycle_owner
    )
    assert (
        DeadlockArbitrationMixin._maintain_runtime_wait_cycle_lease
        is DeadlockLeaseMixin._maintain_runtime_wait_cycle_lease
    )
    assert (
        DeadlockArbitrationMixin._deadlock_retreat_after
        is DeadlockPolicyMixin._deadlock_retreat_after
    )
    assert RuntimeWaitSnapshotModel.__module__.endswith(
        "deadlock_wait_detection"
    )


def test_evacuation_facade_composes_focused_recovery_stages() -> None:
    assert DeadlockEvacuationMixin.__bases__ == (
        EvacuationGeometryMixin,
        EvacuationLatchMixin,
        EvacuationCandidateMixin,
        EvacuationActivationMixin,
        GraphEscapeInstallMixin,
    )
    # The facade keeps the original hooks and now also exposes the named
    # geometry, activation and transactional escape-installation stages used
    # to replace three monolithic methods.
    assert len(_hooks(DeadlockEvacuationMixin)) == 41
    assert (
        DeadlockEvacuationMixin._graph_escape_route_current_body_blocker
        is EvacuationGeometryMixin._graph_escape_route_current_body_blocker
    )
    assert (
        DeadlockEvacuationMixin._corridor_clearance_hold_active
        is EvacuationLatchMixin._corridor_clearance_hold_active
    )
    assert (
        DeadlockEvacuationMixin._build_deadlock_evacuation_candidates
        is EvacuationCandidateMixin._build_deadlock_evacuation_candidates
    )
    assert (
        DeadlockEvacuationMixin._activate_deadlock_evacuation
        is EvacuationActivationMixin._activate_deadlock_evacuation
    )
    assert (
        DeadlockEvacuationMixin._install_graph_escape_retreat
        is GraphEscapeInstallMixin._install_graph_escape_retreat
    )
    assert EvacuationCandidateModel.__module__.endswith(
        "deadlock_evacuation_models"
    )


def test_corridor_recovery_latch_key_remains_static_and_deterministic() -> None:
    owner = FleetRobot(
        name="robot-b",
        current_lm="B",
        active_order_id="order-7",
        route_revision=12,
    )

    assert DeadlockEvacuationMixin._controlled_corridor_recovery_latch_key(
        owner,
        {"region-z", "region-a"},
    ) == (
        ("region-a", "region-z"),
        "robot-b",
        "order-7",
        12,
    )
