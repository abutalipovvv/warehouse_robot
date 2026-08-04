from __future__ import annotations

import inspect
from typing import Any

import fleet_manager.core.traffic.corridors.scheduling.corridor_planner as planner_module
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSchedule,
    CorridorSchedulerConfig,
    CorridorSlot,
    CorridorSlotState,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_planner import (
    CorridorScheduleBuilder,
    _PlanningContext,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_planning_models import (
    _PlanningContext as PlanningContextModel,
)


REGION = "corridor:main"


def _request(
    robot_id: str,
    *,
    earliest_entry: float = 0.0,
    predecessor: str | None = None,
) -> CorridorRequest:
    return CorridorRequest(
        robot_id=robot_id,
        regions=(REGION,),
        direction="east",
        earliest_entry=earliest_entry,
        duration_sec=3.0,
        staging_lm=f"{robot_id}:staging",
        exit_lm=f"{robot_id}:exit",
        route_revision=1,
        predecessor_robot_id=predecessor,
    )


def test_builder_stage_hooks_keep_static_api_and_delegate(
    monkeypatch,
) -> None:
    calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []

    def retain(*args: Any, **kwargs: Any) -> None:
        calls.append(("retain", args, kwargs))

    def schedule(*args: Any, **kwargs: Any) -> None:
        calls.append(("schedule", args, kwargs))

    monkeypatch.setattr(
        planner_module,
        "_retain_previous_slots",
        retain,
    )
    monkeypatch.setattr(
        planner_module,
        "_schedule_pending_requests",
        schedule,
    )
    context = object()
    previous = object()
    pending: list[CorridorRequest] = []
    preferred_entries: dict[str, float] = {}

    CorridorScheduleBuilder._retain_previous_slots(context, previous)
    CorridorScheduleBuilder._schedule_pending(
        context,
        pending,
        preferred_entries,
    )

    assert _PlanningContext is PlanningContextModel
    assert isinstance(
        inspect.getattr_static(
            CorridorScheduleBuilder,
            "_retain_previous_slots",
        ),
        staticmethod,
    )
    assert isinstance(
        inspect.getattr_static(
            CorridorScheduleBuilder,
            "_schedule_pending",
        ),
        staticmethod,
    )
    assert calls == [
        (
            "retain",
            (context, previous),
            {"same_passage": planner_module._same_passage},
        ),
        (
            "schedule",
            (context, pending, preferred_entries),
            {
                "has_flow_predecessor": (
                    planner_module._has_pending_flow_predecessor
                ),
                "phase_fairness_ranks": (
                    planner_module._phase_fairness_ranks
                ),
            },
        ),
    ]


def test_retention_rebases_expired_unobserved_commit() -> None:
    window = CorridorResourceWindow(
        region_id=REGION,
        entry_offset_sec=2.0,
        exit_offset_sec=5.0,
        direction="east",
    )
    request = CorridorRequest(
        robot_id="owner",
        regions=(REGION,),
        direction="east",
        earliest_entry=9.0,
        duration_sec=5.0,
        staging_lm="owner:staging",
        exit_lm="owner:exit",
        route_revision=2,
        past_commit_point=True,
        resource_windows=(window,),
    )
    old_slot = CorridorSlot(
        robot_id="owner",
        regions=(REGION,),
        direction="east",
        entry_time=0.0,
        exit_time=5.0,
        staging_lm="owner:staging",
        exit_lm="owner:exit",
        route_revision=1,
        state=CorridorSlotState.COMMITTED,
        resource_windows=(window,),
        past_commit_point=True,
    )
    previous = CorridorSchedule(
        epoch=1,
        generated_at=0.0,
        horizon_end=30.0,
        slots=(old_slot,),
    )

    schedule = CorridorScheduleBuilder().build(
        (request,),
        controlled_regions={REGION},
        now=10.0,
        previous=previous,
        occupancies=None,
    )

    retained = schedule.slot_for("owner")
    assert retained is not None
    assert retained.entry_time == 10.0
    assert retained.exit_time == 13.0
    assert retained.route_revision == 2
    assert retained.resource_windows == (
        CorridorResourceWindow(
            region_id=REGION,
            entry_offset_sec=0.0,
            exit_offset_sec=3.0,
            direction="east",
        ),
    )


def test_pending_stage_never_schedules_follower_before_predecessor() -> None:
    leader = _request("leader", earliest_entry=4.0)
    follower = _request(
        "follower",
        earliest_entry=0.0,
        predecessor="leader",
    )

    schedule = CorridorScheduleBuilder(
        CorridorSchedulerConfig(
            horizon_sec=30.0,
            commit_horizon_sec=30.0,
        ),
    ).build(
        (follower, leader),
        controlled_regions={REGION},
        now=0.0,
    )

    assert tuple(slot.robot_id for slot in schedule.slots) == (
        "leader",
        "follower",
    )
    assert schedule.slots[1].entry_time > schedule.slots[0].entry_time
