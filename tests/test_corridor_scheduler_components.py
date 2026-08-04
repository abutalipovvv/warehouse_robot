from __future__ import annotations

from dataclasses import FrozenInstanceError
from itertools import permutations

import pytest

import fleet_manager.core.traffic.corridors.scheduling.corridor_models as corridor_models
from fleet_manager.core.traffic.corridors.scheduling.corridor_calendar import CorridorCalendar
from fleet_manager.core.traffic.corridors.scheduling.corridor_planner import (
    CorridorScheduleBuilder,
)
from fleet_manager.core.traffic.corridors.scheduling.corridor_scheduler import (
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSchedulerConfig,
    CorridorSlot,
    CorridorSlotState,
    build_corridor_schedule,
)


REGION = "corridor:main"


def _request(
    robot_id: str,
    *,
    direction: str = "east",
    eta: float = 0.0,
    duration: float = 3.0,
    wait_age: float = 0.0,
) -> CorridorRequest:
    return CorridorRequest(
        robot_id=robot_id,
        regions=(REGION,),
        direction=direction,
        earliest_entry=eta,
        duration_sec=duration,
        staging_lm=f"{robot_id}:holding",
        exit_lm=f"{robot_id}:exit",
        route_revision=1,
        wait_age_sec=wait_age,
    )


def test_public_facade_reexports_immutable_domain_models() -> None:
    assert CorridorRequest is corridor_models.CorridorRequest
    assert CorridorResourceWindow is corridor_models.CorridorResourceWindow
    assert CorridorSchedulerConfig is corridor_models.CorridorSchedulerConfig

    request = _request("robot")
    with pytest.raises(FrozenInstanceError):
        request.direction = "west"  # type: ignore[misc]


def test_function_facade_and_builder_publish_the_same_schedule() -> None:
    config = CorridorSchedulerConfig(
        horizon_sec=40.0,
        headway_sec=0.25,
        direction_change_sec=1.0,
    )
    requests = (
        _request("east", direction="east"),
        _request("west", direction="west", wait_age=10.0),
    )

    expected = CorridorScheduleBuilder(config).build(
        requests,
        controlled_regions={REGION},
        now=0.0,
    )
    actual = build_corridor_schedule(
        requests,
        controlled_regions={REGION},
        now=0.0,
        config=config,
    )

    assert actual == expected


def test_calendar_places_resource_windows_instead_of_whole_passages() -> None:
    config = CorridorSchedulerConfig(
        horizon_sec=30.0,
        headway_sec=0.25,
        direction_change_sec=1.0,
    )
    calendar = CorridorCalendar(config)
    occupied = CorridorSlot(
        robot_id="owner",
        regions=(REGION,),
        direction="west",
        entry_time=2.0,
        exit_time=4.0,
        staging_lm="owner:holding",
        exit_lm="owner:exit",
        route_revision=1,
        state=CorridorSlotState.COMMITTED,
    )
    pipelined = CorridorRequest(
        robot_id="next",
        regions=(REGION,),
        direction="east",
        earliest_entry=0.0,
        duration_sec=10.0,
        staging_lm="next:holding",
        exit_lm="next:exit",
        route_revision=1,
        resource_windows=(
            CorridorResourceWindow(
                region_id=REGION,
                entry_offset_sec=6.0,
                exit_offset_sec=9.0,
                direction="east",
            ),
        ),
    )

    placement = calendar.earliest_placement(
        pipelined,
        slots=(occupied,),
        now=0.0,
        horizon_end=30.0,
    )

    assert placement == (0.0, 10.0, 1)


def test_builder_is_deterministic_across_request_order() -> None:
    config = CorridorSchedulerConfig(
        horizon_sec=50.0,
        headway_sec=0.5,
        direction_change_sec=1.0,
        starvation_sec=5.0,
        max_direction_batch=2,
    )
    requests = (
        _request("east-old", direction="east", wait_age=12.0),
        _request("west-old", direction="west", wait_age=11.0),
        _request("east-new", direction="east"),
        _request("west-new", direction="west"),
    )
    builder = CorridorScheduleBuilder(config)

    schedules = [
        builder.build(
            order,
            controlled_regions={REGION},
            now=0.0,
        )
        for order in permutations(requests)
    ]

    first = schedules[0]
    assert all(schedule.slots == first.slots for schedule in schedules)
    assert all(schedule.decisions == first.decisions for schedule in schedules)


def test_schedule_fingerprint_ignores_the_publication_clock() -> None:
    builder = CorridorScheduleBuilder(
        CorridorSchedulerConfig(horizon_sec=30.0),
    )
    requests = (_request("robot", eta=10.0),)
    first = builder.build(
        requests,
        controlled_regions={REGION},
        now=0.0,
    )

    rolled = builder.build(
        requests,
        controlled_regions={REGION},
        now=0.5,
        previous=first,
    )

    assert rolled.generated_at == 0.5
    assert rolled.horizon_end != first.horizon_end
    assert rolled.epoch == first.epoch
    assert not rolled.changed
