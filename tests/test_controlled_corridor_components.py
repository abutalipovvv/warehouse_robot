from __future__ import annotations

import ast
from pathlib import Path

import pytest

from fleet_manager.manager.traffic.corridors.admission.controlled_corridor_admission import (
    ControlledCorridorAdmissionMixin,
)
from fleet_manager.manager.traffic.corridors.admission.controlled_corridor_admission_decisions import (
    ControlledCorridorAdmissionDecisionMixin,
)
from fleet_manager.manager.traffic.corridors.admission.controlled_corridor_admission_models import (
    _CentralCorridorBuild,
    _CentralCorridorPublication,
    _CentralCorridorWaitContext,
)
from fleet_manager.manager.traffic.corridors.admission.controlled_corridor_admission_requests import (
    ControlledCorridorRequestCollectionMixin,
)
from fleet_manager.manager.traffic.corridors.admission.controlled_corridor_admission_runtime import (
    ControlledCorridorRuntimePublicationMixin,
)
from fleet_manager.manager.traffic.corridors.prefetch.controlled_corridor_prefetch import (
    ControlledCorridorPrefetchMixin,
)
from fleet_manager.manager.traffic.corridors.prefetch.controlled_corridor_prefetch_gate import (
    ControlledCorridorPrefetchGateMixin,
)
from fleet_manager.manager.traffic.corridors.prefetch.controlled_corridor_prefetch_intent import (
    ControlledCorridorPrefetchIntentMixin,
)
from fleet_manager.manager.traffic.corridors.prefetch.controlled_corridor_prefetch_models import (
    _CorridorIntentDraft,
    _CorridorPlannedPassage,
    _CorridorRouteDraft,
    _CorridorValidationContext,
)
from fleet_manager.manager.traffic.corridors.prefetch.controlled_corridor_prefetch_validation import (
    ControlledCorridorPrefetchValidationMixin,
)


ADMISSION_HOOKS = {
    ControlledCorridorAdmissionDecisionMixin: (
        "_controlled_corridor_has_grant",
        "_controlled_corridor_entry_lookahead",
        "_retained_route_is_superseded",
        "_central_corridor_manages_wait",
        "_central_corridor_owner_is_clearing",
        "_controlled_corridor_downstream_blocker",
        "_controlled_corridor_physical_exit_time",
        "_controlled_corridor_admission_reason",
        "_transfer_controlled_corridor_lease",
        "_controlled_corridor_wait_reason",
    ),
    ControlledCorridorRequestCollectionMixin: (
        "_capture_controlled_corridor_occupancy",
        "_collect_controlled_corridor_requests",
        "_collect_controlled_corridor_prefetch_requests",
        "_maintain_controlled_corridor_waits",
    ),
    ControlledCorridorRuntimePublicationMixin: (
        "_prepare_controlled_corridor_admissions",
        "_prepare_central_controlled_corridor_schedule",
        "_update_controlled_corridor_calendar",
        "_publish_controlled_corridor_runtime",
    ),
}

PREFETCH_HOOKS = {
    ControlledCorridorPrefetchIntentMixin: (
        "_controlled_corridor_prefetch_intent",
        "_controlled_corridor_intent_is_current",
    ),
    ControlledCorridorPrefetchGateMixin: (
        "_controlled_corridor_prefetch_gate",
        "_corridor_approach_gate",
        "_prepare_corridor_approach_request",
        "_controlled_corridor_approach_holding_lm",
    ),
    ControlledCorridorPrefetchValidationMixin: (
        "_controlled_corridor_prefetch_plan_is_current",
        "_commit_controlled_corridor_prefetch_slot",
        "_pin_controlled_corridor_gates",
        "_release_controlled_corridor_gate_pins",
        "_handle_controlled_corridor_gate_rejection",
    ),
}


def test_admission_facade_preserves_all_runtime_hooks() -> None:
    assert ControlledCorridorAdmissionMixin.__bases__ == (
        ControlledCorridorRuntimePublicationMixin,
        ControlledCorridorRequestCollectionMixin,
        ControlledCorridorAdmissionDecisionMixin,
    )
    for owner, hook_names in ADMISSION_HOOKS.items():
        for hook_name in hook_names:
            assert getattr(ControlledCorridorAdmissionMixin, hook_name) is (
                getattr(owner, hook_name)
            )


def test_prefetch_facade_preserves_all_runtime_hooks() -> None:
    assert ControlledCorridorPrefetchMixin.__bases__ == (
        ControlledCorridorPrefetchGateMixin,
        ControlledCorridorPrefetchIntentMixin,
        ControlledCorridorPrefetchValidationMixin,
    )
    for owner, hook_names in PREFETCH_HOOKS.items():
        for hook_name in hook_names:
            assert getattr(ControlledCorridorPrefetchMixin, hook_name) is (
                getattr(owner, hook_name)
            )


@pytest.mark.parametrize(
    "model",
    (
        _CentralCorridorWaitContext,
        _CorridorRouteDraft,
        _CorridorIntentDraft,
        _CorridorValidationContext,
        _CorridorPlannedPassage,
    ),
)
def test_stage_values_are_frozen_and_slot_based(model: type[object]) -> None:
    assert model.__dataclass_params__.frozen
    assert "__dict__" not in model.__dict__
    assert "__slots__" in model.__dict__


@pytest.mark.parametrize(
    "model",
    (_CentralCorridorBuild, _CentralCorridorPublication),
)
def test_snapshot_accumulators_are_explicitly_mutable(model: type[object]) -> None:
    assert not model.__dataclass_params__.frozen
    assert "__slots__" in model.__dict__


def test_controlled_corridor_stages_keep_methods_reviewable() -> None:
    traffic_dir = Path(__file__).parents[1] / "fleet_manager/manager/traffic"
    component_paths = (
        "corridors/admission/controlled_corridor_admission_decisions.py",
        "corridors/admission/controlled_corridor_admission_requests.py",
        "corridors/admission/controlled_corridor_admission_runtime.py",
        "corridors/prefetch/controlled_corridor_prefetch_intent.py",
        "corridors/prefetch/controlled_corridor_prefetch_gate.py",
        "corridors/prefetch/controlled_corridor_prefetch_validation.py",
    )
    oversized: dict[str, tuple[str, int]] = {}
    for component_path in component_paths:
        tree = ast.parse((traffic_dir / component_path).read_text())
        methods = (
            node
            for parent in tree.body
            if isinstance(parent, ast.ClassDef)
            for node in parent.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        )
        for method in methods:
            length = method.end_lineno - method.lineno + 1
            if length > 140:
                oversized[component_path] = (method.name, length)
    assert oversized == {}
