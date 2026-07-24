from __future__ import annotations

from dataclasses import replace
from itertools import permutations

import pytest

from fleet_manager.core.traffic.corridor_scheduler import (
    CentralCorridorScheduler,
    CorridorDecisionStatus,
    CorridorOccupancy,
    CorridorRequest,
    CorridorResourceWindow,
    CorridorSchedulerConfig,
    CorridorSlot,
    CorridorSlotState,
)


REGION = "corridor:main"
LONG_PASSAGE_REGIONS = tuple(
    f"corridor:section-{index}" for index in range(4)
)


def _request(
    robot_id: str,
    *,
    direction: str = "east",
    eta: float = 0.0,
    duration: float = 3.0,
    wait_age: float = 0.0,
    priority: float = 0.0,
    deadline: float | None = None,
    revision: int = 1,
    regions: tuple[str, ...] = (REGION,),
    downstream_ready_at: float | None = None,
    downstream_available: bool = True,
    predecessor_robot_id: str | None = None,
    entered: bool = False,
    past_commit_point: bool = False,
    requires_explicit_commit: bool = False,
    staging_lm: str | None = None,
    exit_lm: str | None = None,
) -> CorridorRequest:
    return CorridorRequest(
        robot_id=robot_id,
        regions=regions,
        direction=direction,
        earliest_entry=eta,
        duration_sec=duration,
        staging_lm=staging_lm or f"{robot_id}:holding",
        exit_lm=exit_lm or f"{robot_id}:exit",
        route_revision=revision,
        wait_age_sec=wait_age,
        priority=priority,
        deadline=deadline,
        downstream_ready_at=downstream_ready_at,
        downstream_available=downstream_available,
        predecessor_robot_id=predecessor_robot_id,
        entered=entered,
        past_commit_point=past_commit_point,
        requires_explicit_commit=requires_explicit_commit,
    )


def _scheduler(
    **overrides: float | int,
) -> CentralCorridorScheduler:
    values = {
        "horizon_sec": 30.0,
        "commit_horizon_sec": 1.0,
        "headway_sec": 0.25,
        "direction_change_sec": 1.0,
        "starvation_sec": 8.0,
        "direction_switch_cost_sec": 2.0,
        "priority_cost_sec": 0.05,
        "wait_age_cost_sec": 0.03,
    }
    values.update(overrides)
    return CentralCorridorScheduler(
        {REGION},
        config=CorridorSchedulerConfig(**values),
    )


def _long_passage_request(
    robot_id: str,
    *,
    direction: str,
    wait_age: float = 8.0,
) -> CorridorRequest:
    """Model the four-zone, 41 s no-wait passage used by smart Kiva."""
    east_windows = (
        (0.0, 13.0),
        (10.0, 22.0),
        (19.0, 31.0),
        (28.0, 41.0),
    )
    offsets = (
        east_windows
        if direction == "east"
        else tuple(reversed(east_windows))
    )
    return CorridorRequest(
        robot_id=robot_id,
        regions=LONG_PASSAGE_REGIONS,
        direction=f"route:{direction}",
        earliest_entry=0.0,
        duration_sec=41.0,
        staging_lm=f"{robot_id}:holding",
        exit_lm=f"{robot_id}:exit",
        route_revision=1,
        wait_age_sec=wait_age,
        resource_windows=tuple(
            CorridorResourceWindow(
                region_id=region_id,
                entry_offset_sec=entry_offset,
                exit_offset_sec=exit_offset,
                direction=f"flow:{direction}",
            )
            for region_id, (entry_offset, exit_offset) in zip(
                LONG_PASSAGE_REGIONS,
                offsets,
                strict=True,
            )
        ),
    )


def _adaptive_scheduler() -> CentralCorridorScheduler:
    return CentralCorridorScheduler(
        LONG_PASSAGE_REGIONS,
        config=CorridorSchedulerConfig(
            horizon_sec=200.0,
            commit_horizon_sec=1.0,
            headway_sec=1.0,
            direction_change_sec=1.0,
            starvation_sec=8.0,
            starvation_age_quantum_sec=2.0,
            max_direction_batch=3,
            max_adaptive_direction_batch=12,
            phase_amortization_sec=4.0,
            max_phase_extension_sec=8.0,
        ),
    )


def test_scheduler_only_accepts_explicit_controlled_regions() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request("corridor-robot"),
            _request("free-space-robot", regions=("ordinary:edge",)),
        ],
        now=0.0,
    )

    assert schedule.decisions["corridor-robot"].status is (
        CorridorDecisionStatus.GRANTED
    )
    rejected = schedule.decisions["free-space-robot"]
    assert rejected.status is CorridorDecisionStatus.REJECTED
    assert "outside explicit controlled corridors" in rejected.reason
    assert {slot.robot_id for slot in schedule.slots} == {"corridor-robot"}


def test_far_starved_slot_is_protected_but_near_robots_fill_its_gap() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request("far", eta=12.0, wait_age=20.0),
            _request("near-1", eta=0.0),
            _request("near-2", eta=0.0),
        ],
        now=0.0,
    )

    slots = {slot.robot_id: slot for slot in schedule.slots}
    assert slots["near-1"].entry_time == 0.0
    assert slots["near-2"].entry_time == 0.25
    assert slots["near-2"].exit_time == slots["near-1"].exit_time + 0.25
    assert slots["near-2"].exit_time < slots["far"].entry_time
    assert slots["far"].entry_time == 12.0


def test_ready_robots_are_batched_in_one_direction_before_switch() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request("a-east", direction="east"),
            _request("b-west", direction="west"),
            _request("c-east", direction="east"),
        ],
        now=0.0,
    )

    assert [slot.robot_id for slot in schedule.slots] == [
        "a-east",
        "c-east",
        "b-west",
    ]
    first, second, third = schedule.slots
    assert second.entry_time == first.entry_time + 0.25
    assert second.exit_time == first.exit_time + 0.25
    assert third.entry_time == second.exit_time + 1.0


def test_same_direction_convoy_overlaps_without_overtaking() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request("leader", eta=0.0, duration=8.0, direction="east"),
            _request("follower", eta=0.1, duration=2.0, direction="east"),
            _request("opposing", eta=0.0, duration=2.0, direction="west"),
        ],
        now=0.0,
    )

    leader = schedule.slot_for("leader")
    follower = schedule.slot_for("follower")
    opposing = schedule.slot_for("opposing")
    assert follower.entry_time >= leader.entry_time + 0.25
    assert follower.exit_time >= leader.exit_time + 0.25
    assert follower.entry_time < leader.exit_time
    assert (
        opposing.exit_time + 1.0 <= leader.entry_time
        or opposing.entry_time >= follower.exit_time + 1.0
    )


def test_same_direction_follower_never_overtakes_physical_queue_leader() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request(
                "follower",
                eta=0.0,
                past_commit_point=True,
                predecessor_robot_id="leader",
            ),
            _request("leader", eta=0.0),
        ],
        now=0.0,
    )

    assert [slot.robot_id for slot in schedule.slots] == [
        "leader",
        "follower",
    ]


def test_starvation_breaks_an_existing_direction_phase() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request("a-east", direction="east"),
            _request("b-east", direction="east"),
            _request("c-west-starved", direction="west", wait_age=8.0),
        ],
        now=0.0,
    )

    assert schedule.slots[0].robot_id == "c-west-starved"
    assert {
        slot.robot_id for slot in schedule.slots[1:]
    } == {"a-east", "b-east"}


def test_oldest_starved_request_wins_even_when_its_passage_is_longer() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request(
                "older-long",
                duration=12.0,
                wait_age=30.0,
                direction="west",
            ),
            _request(
                "younger-short",
                duration=1.0,
                wait_age=10.0,
                direction="east",
            ),
        ],
        now=0.0,
    )

    assert schedule.slots[0].robot_id == "older-long"


def test_aged_requests_are_served_in_bounded_direction_phases() -> None:
    scheduler = _scheduler(
        starvation_age_quantum_sec=4.0,
        max_direction_batch=2,
    )

    schedule = scheduler.update(
        [
            _request("e1", direction="east", wait_age=20.0),
            _request("w1", direction="west", wait_age=19.5),
            _request("e2", direction="east", wait_age=19.0),
            _request("w2", direction="west", wait_age=18.5),
            _request("e3", direction="east", wait_age=18.0),
            _request("w3", direction="west", wait_age=17.5),
        ],
        now=0.0,
    )

    assert [slot.direction for slot in schedule.slots] == [
        "east",
        "east",
        "west",
        "west",
        "east",
        "west",
    ]


def test_short_passage_keeps_the_base_direction_batch() -> None:
    scheduler = CentralCorridorScheduler(
        {REGION},
        config=CorridorSchedulerConfig(
            horizon_sec=100.0,
            commit_horizon_sec=1.0,
            headway_sec=1.0,
            direction_change_sec=1.0,
            starvation_sec=8.0,
            max_direction_batch=3,
            max_adaptive_direction_batch=12,
            phase_amortization_sec=4.0,
            max_phase_extension_sec=8.0,
        ),
    )
    requests = [
        _request(
            f"e{index:02d}",
            direction="east",
            duration=5.16,
            wait_age=8.0,
        )
        for index in range(6)
    ]
    requests.append(
        _request(
            "w00",
            direction="west",
            duration=5.16,
            wait_age=8.0,
        )
    )

    schedule = scheduler.update(requests, now=0.0)

    assert [slot.direction for slot in schedule.slots[:4]] == [
        "east",
        "east",
        "east",
        "west",
    ]


def test_long_atomic_passage_uses_an_adaptive_eleven_robot_convoy() -> None:
    scheduler = _adaptive_scheduler()
    requests = [
        _long_passage_request(f"e{index:02d}", direction="east")
        for index in range(12)
    ]
    requests.append(_long_passage_request("w00", direction="west"))

    schedule = scheduler.update(requests, now=0.0)

    assert [slot.direction for slot in schedule.slots[:12]] == [
        *(["route:east"] * 11),
        "route:west",
    ]
    assert schedule.slots[12].direction == "route:east"


def test_active_long_phase_is_not_broken_by_small_wait_age_differences() -> None:
    scheduler = _adaptive_scheduler()
    active = replace(
        _long_passage_request(
            "a-active-east",
            direction="east",
            wait_age=8.0,
        ),
        entered=True,
    )
    requests = [
        active,
        *(
            _long_passage_request(
                f"b-east-{index:02d}",
                direction="east",
                wait_age=8.0,
            )
            for index in range(5)
        ),
        _long_passage_request(
            "c-west-older",
            direction="west",
            wait_age=10.5,
        ),
    ]

    schedule = scheduler.update(requests, now=0.0, occupancies=[])

    # Two seconds of queue-age jitter must not pay a 42-second direction
    # reversal. The bounded adaptive cap still forces the opposite flow later.
    assert [slot.direction for slot in schedule.slots[:6]] == [
        *(["route:east"] * 6),
    ]
    assert schedule.slots[6].direction == "route:west"


def test_opposing_wait_age_spends_the_adaptive_phase_extension() -> None:
    scheduler = _adaptive_scheduler()
    requests = [
        _long_passage_request(
            f"e{index:02d}",
            direction="east",
            wait_age=16.0,
        )
        for index in range(6)
    ]
    requests.append(
        _long_passage_request(
            "w00",
            direction="west",
            wait_age=16.0,
        )
    )

    schedule = scheduler.update(requests, now=0.0)

    assert [slot.direction for slot in schedule.slots[:4]] == [
        "route:east",
        "route:east",
        "route:east",
        "route:west",
    ]


def test_adaptive_batch_does_not_throttle_a_single_direction() -> None:
    scheduler = _adaptive_scheduler()
    requests = [
        _long_passage_request(f"e{index:02d}", direction="east")
        for index in range(20)
    ]

    schedule = scheduler.update(requests, now=0.0)

    assert len(schedule.slots) == len(requests)
    assert {slot.direction for slot in schedule.slots} == {"route:east"}
    assert [
        slot.entry_time for slot in schedule.slots
    ] == pytest.approx([float(index) for index in range(20)])


def test_adaptive_multi_region_calendar_is_safe_and_deterministic() -> None:
    requests = [
        *(
            _long_passage_request(f"e{index:02d}", direction="east")
            for index in range(12)
        ),
        _long_passage_request("w00", direction="west"),
    ]

    forward = _adaptive_scheduler().update(requests, now=0.0)
    reversed_input = _adaptive_scheduler().update(
        list(reversed(requests)),
        now=0.0,
    )

    def fingerprint(schedule: object) -> list[tuple[object, ...]]:
        return [
            (
                slot.robot_id,
                slot.entry_time,
                slot.exit_time,
                slot.state,
                slot.resource_windows,
            )
            for slot in schedule.slots
        ]

    assert fingerprint(forward) == fingerprint(reversed_input)
    for index, first in enumerate(forward.slots):
        for second in forward.slots[index + 1 :]:
            first_windows = {
                window.region_id: window
                for window in first.resource_windows
            }
            second_windows = {
                window.region_id: window
                for window in second.resource_windows
            }
            for region_id in set(first_windows) & set(second_windows):
                first_window = first_windows[region_id]
                second_window = second_windows[region_id]
                if first_window.direction == second_window.direction:
                    continue
                first_exit = (
                    first.entry_time + first_window.exit_offset_sec
                )
                second_entry = (
                    second.entry_time + second_window.entry_offset_sec
                )
                second_exit = (
                    second.entry_time + second_window.exit_offset_sec
                )
                first_entry = (
                    first.entry_time + first_window.entry_offset_sec
                )
                assert (
                    first_exit + 1.0 <= second_entry + 1e-9
                    or second_exit + 1.0 <= first_entry + 1e-9
                )


def test_downstream_constraint_delays_entry_and_blocked_exit_defers() -> None:
    scheduler = _scheduler()

    schedule = scheduler.update(
        [
            _request(
                "delayed",
                duration=3.0,
                downstream_ready_at=10.0,
            ),
            _request("blocked", downstream_available=False),
        ],
        now=0.0,
    )

    delayed = schedule.slot_for("delayed")
    assert delayed is not None
    assert delayed.entry_time == 7.0
    assert delayed.exit_time == 10.0
    assert schedule.decisions["blocked"].status is (
        CorridorDecisionStatus.DEFERRED
    )
    assert "downstream exit" in schedule.decisions["blocked"].reason


def test_epoch_is_stable_until_calendar_semantics_change() -> None:
    scheduler = _scheduler()
    requests = [_request("robot", eta=10.0)]

    first = scheduler.update(requests, now=0.0)
    identical = scheduler.update(requests, now=0.5)
    revised = scheduler.update(
        [_request("robot", eta=10.0, revision=2)],
        now=0.5,
    )

    assert first.epoch == 1
    assert first.changed
    assert identical.epoch == first.epoch
    assert not identical.changed
    assert revised.epoch == first.epoch + 1
    assert revised.changed
    assert revised.slot_for("robot").route_revision == 2


def test_committed_slot_survives_rolling_route_revision_handoff() -> None:
    scheduler = _scheduler(commit_horizon_sec=2.0)

    first = scheduler.update(
        [_request("robot", eta=1.0, revision=1)],
        now=0.0,
    )
    old_slot = first.slot_for("robot")
    assert old_slot is not None
    assert old_slot.state is CorridorSlotState.COMMITTED

    revised = scheduler.update(
        [_request("robot", eta=0.5, revision=2)],
        now=0.5,
    )

    revised_slot = revised.slot_for("robot")
    assert revised_slot.entry_time == old_slot.entry_time
    assert revised_slot.exit_time == old_slot.exit_time
    assert revised_slot.route_revision == 2
    assert revised.epoch == first.epoch + 1
    assert revised.changed


def test_revision_handoff_reuses_only_identical_passage_direction() -> None:
    scheduler = _scheduler(commit_horizon_sec=2.0)
    first = scheduler.update(
        [_request("robot", direction="east", duration=10.0, revision=1)],
        now=0.0,
    )

    same_passage = scheduler.update(
        [_request("robot", direction="east", duration=10.0, revision=2)],
        now=1.0,
    )
    same_slot = same_passage.slot_for("robot")
    assert same_slot.entry_time == first.slot_for("robot").entry_time
    assert same_slot.route_revision == 2

    changed_direction = scheduler.update(
        [_request("robot", direction="west", duration=10.0, revision=3)],
        now=1.5,
    )
    changed_slot = changed_direction.slot_for("robot")
    assert changed_slot.direction == "west"
    assert changed_slot.route_revision == 3
    assert changed_slot.entry_time == 1.5

    exit_scheduler = _scheduler(commit_horizon_sec=2.0)
    exit_scheduler.update(
        [_request("exit-change", duration=10.0, revision=1)],
        now=0.0,
    )
    changed_exit = exit_scheduler.update(
        [
            _request(
                "exit-change",
                duration=10.0,
                revision=2,
                exit_lm="different-exit",
            )
        ],
        now=1.0,
    )
    assert changed_exit.slot_for("exit-change").entry_time == 1.0
    assert changed_exit.slot_for("exit-change").exit_lm == "different-exit"


def test_past_commit_point_rolls_authority_past_predicted_exit() -> None:
    scheduler = _scheduler(occupancy_recheck_sec=0.5)
    initial_request = _request("owner", eta=0.0, duration=2.0)

    initial = scheduler.update([initial_request], now=0.0)
    assert initial.slot_for("owner").exit_time == 2.0

    overdue = scheduler.update(
        [
            _request(
                "owner",
                eta=0.0,
                duration=2.0,
                past_commit_point=True,
            ),
            _request("waiter", direction="west"),
        ],
        now=3.0,
    )

    owner = overdue.slot_for("owner")
    assert owner.entry_time == 3.0
    assert owner.exit_time == 5.0
    assert owner.duration_sec == 2.0
    assert owner.state is CorridorSlotState.COMMITTED
    assert overdue.slot_for("waiter").entry_time >= owner.exit_time + 1.0

    superseded = scheduler.update(
        [
            _request(
                "owner",
                eta=0.0,
                duration=2.0,
                revision=2,
                past_commit_point=True,
            )
        ],
        now=3.2,
    )

    replacement = superseded.slot_for("owner")
    assert replacement.route_revision == 2
    assert replacement.entry_time == 3.0
    assert replacement.duration_sec == 2.0


def test_time_committed_slot_without_commit_point_can_be_rescheduled() -> None:
    scheduler = _scheduler(occupancy_recheck_sec=0.5)
    request = _request("owner", eta=0.0, duration=2.0)

    scheduler.update([request], now=0.0)
    overdue = scheduler.update([request], now=3.0)

    assert overdue.slot_for("owner").entry_time == 3.0
    assert overdue.slot_for("owner").exit_time == 5.0


def test_missed_pre_entry_commit_is_retimed_from_authoritative_snapshot() -> None:
    scheduler = _scheduler(
        commit_horizon_sec=2.0,
        occupancy_recheck_sec=0.2,
    )
    initial = scheduler.update(
        [_request("owner", eta=1.0, duration=10.0)],
        occupancies=[],
        now=0.0,
    )
    assert initial.slot_for("owner").state is CorridorSlotState.COMMITTED
    assert initial.slot_for("owner").entry_time == 1.0

    delayed = scheduler.update(
        [_request("owner", eta=4.0, duration=10.0)],
        occupancies=[],
        now=0.5,
    )

    assert delayed.slot_for("owner").entry_time == 4.0
    assert delayed.slot_for("owner").exit_time == 14.0


def test_started_staging_approach_keeps_committed_slot_without_occupancy() -> None:
    scheduler = _scheduler(
        commit_horizon_sec=2.0,
        occupancy_recheck_sec=0.2,
    )
    scheduler.update(
        [_request("owner", eta=1.0, duration=10.0)],
        occupancies=[],
        now=0.0,
    )

    retained = scheduler.update(
        [
            _request(
                "owner",
                eta=4.0,
                duration=10.0,
                past_commit_point=True,
            )
        ],
        occupancies=[],
        now=0.5,
    )

    assert retained.slot_for("owner").entry_time == 1.0
    assert retained.slot_for("owner").past_commit_point


def test_clearing_commit_point_releases_rolled_slot_after_physical_exit() -> None:
    scheduler = _scheduler(occupancy_recheck_sec=0.5)
    request = _request("owner", eta=0.0, duration=2.0)
    scheduler.update([request], now=0.0)
    scheduler.update(
        [_request("owner", duration=2.0, past_commit_point=True)],
        now=3.0,
    )

    after_exit = scheduler.update([request], now=3.2)

    assert after_exit.slot_for("owner").entry_time == 3.2
    assert not after_exit.slot_for("owner").past_commit_point


def test_tentative_slot_is_frozen_when_it_enters_commit_horizon() -> None:
    scheduler = _scheduler(commit_horizon_sec=2.0)
    request = _request("reserved", eta=10.0)

    tentative = scheduler.update([request], now=0.0)
    assert tentative.slot_for("reserved").state is CorridorSlotState.TENTATIVE

    committed = scheduler.update(
        [
            request,
            _request(
                "new-starved",
                eta=8.5,
                wait_age=20.0,
                direction="west",
            ),
        ],
        now=8.0,
    )

    reserved = committed.slot_for("reserved")
    assert reserved.entry_time == 10.0
    assert reserved.state is CorridorSlotState.COMMITTED
    assert committed.slot_for("new-starved").entry_time >= reserved.exit_time + 1.0


def test_future_intent_remains_tentative_until_mapf_commit() -> None:
    scheduler = _scheduler(commit_horizon_sec=10.0)
    request = _request(
        "future",
        eta=0.0,
        requires_explicit_commit=True,
    )

    tentative = scheduler.update([request], now=0.0)
    slot = tentative.slot_for("future")
    assert slot is not None
    assert slot.state is CorridorSlotState.TENTATIVE

    committed = scheduler.commit_slot("future", expected=slot)
    assert committed is not None
    assert committed.slot_for("future").state is CorridorSlotState.COMMITTED


def test_inflight_pin_holds_exact_tentative_slot_until_worker_release() -> None:
    scheduler = _scheduler(commit_horizon_sec=0.0)
    request = _request(
        "worker",
        eta=5.0,
        requires_explicit_commit=True,
    )
    first = scheduler.update([request], now=0.0, occupancies=[])
    captured = first.slot_for("worker")
    assert captured is not None
    assert scheduler.pin_slot("worker", expected=captured)

    delayed_request = replace(request, earliest_entry=10.0)
    pinned = scheduler.update(
        [
            delayed_request,
            _request(
                "opposing",
                direction="west",
                eta=4.0,
                requires_explicit_commit=True,
            ),
        ],
        now=1.0,
        occupancies=[],
    )

    assert pinned.slot_for("worker") == captured
    assert pinned.slot_for("worker").state is CorridorSlotState.TENTATIVE
    assert pinned.decisions["worker"].reason == "in-flight MAPF slot pinned"

    scheduler.release_slot_pin("worker", expected=captured)
    moved = scheduler.update(
        [delayed_request],
        now=1.0,
        occupancies=[],
    )
    assert moved.slot_for("worker") != captured
    assert moved.slot_for("worker").entry_time >= 10.0


def test_physical_occupancy_invalidates_an_inflight_pin() -> None:
    scheduler = _scheduler(commit_horizon_sec=0.0)
    request = _request(
        "worker",
        eta=5.0,
        requires_explicit_commit=True,
    )
    first = scheduler.update([request], now=0.0, occupancies=[])
    captured = first.slot_for("worker")
    assert captured is not None
    assert scheduler.pin_slot("worker", expected=captured)
    occupancy = CorridorOccupancy(
        robot_id="physical",
        regions=(REGION,),
        direction="west",
        entered_at=4.0,
        expected_exit_time=9.0,
        exit_lm="physical:exit",
        route_revision=1,
    )

    displaced = scheduler.update(
        [request],
        now=4.0,
        occupancies=[occupancy],
    )

    assert displaced.slot_for("physical").physically_observed
    assert displaced.slot_for("worker") != captured
    assert displaced.slot_for("worker").entry_time >= 10.0
    assert scheduler.commit_slot(
        "worker",
        expected=captured,
    ) is None


def test_validated_command_displaces_only_conflicting_tentative_slot() -> None:
    scheduler = _scheduler(commit_horizon_sec=0.0)
    schedule = scheduler.update(
        [
            _request(
                "validated",
                direction="east",
                requires_explicit_commit=True,
            ),
            _request(
                "proposal",
                direction="west",
                requires_explicit_commit=True,
            ),
        ],
        now=0.0,
    )
    validated = schedule.slot_for("validated")
    proposal = schedule.slot_for("proposal")
    assert validated is not None
    assert proposal is not None
    assert scheduler.pin_slot("proposal", expected=proposal)
    shifted = replace(
        validated,
        entry_time=proposal.entry_time,
        exit_time=proposal.entry_time + validated.duration_sec,
    )

    committed = scheduler.commit_slot(
        "validated",
        expected=validated,
        actual=shifted,
    )

    assert committed is not None
    assert committed.slot_for("validated").state is CorridorSlotState.COMMITTED
    assert committed.slot_for("proposal") is None
    assert committed.decisions["proposal"].status is (
        CorridorDecisionStatus.DEFERRED
    )
    assert scheduler.commit_slot(
        "proposal",
        expected=proposal,
    ) is None


def test_installed_command_precedes_older_unvalidated_intent() -> None:
    scheduler = _scheduler(commit_horizon_sec=0.0)

    schedule = scheduler.update(
        [
            _request(
                "a-future",
                direction="west",
                wait_age=100.0,
                requires_explicit_commit=True,
            ),
            _request("z-command", direction="east"),
        ],
        now=0.0,
    )

    ordered = sorted(
        schedule.slots,
        key=lambda slot: (slot.entry_time, slot.robot_id),
    )
    assert ordered[0].robot_id == "z-command"


def test_tentative_hysteresis_avoids_small_eta_reshuffle() -> None:
    scheduler = _scheduler(tentative_change_penalty_sec=2.0)

    first = scheduler.update([_request("robot", eta=10.0)], now=0.0)
    small_gain = scheduler.update(
        [_request("robot", eta=8.0)],
        now=0.0,
    )
    material_gain = scheduler.update(
        [_request("robot", eta=7.0)],
        now=0.0,
    )

    assert first.slot_for("robot").entry_time == 10.0
    assert small_gain.slot_for("robot").entry_time == 10.0
    assert small_gain.epoch == first.epoch
    assert material_gain.slot_for("robot").entry_time == 7.0
    assert material_gain.epoch == first.epoch + 1


def test_physical_owner_stays_first_past_predicted_exit() -> None:
    scheduler = _scheduler(occupancy_recheck_sec=0.5)
    stale_prediction = CorridorOccupancy(
        robot_id="owner",
        regions=(REGION,),
        direction="east",
        entered_at=0.0,
        expected_exit_time=1.0,
        exit_lm="owner:exit",
        route_revision=1,
    )

    occupied = scheduler.update(
        [_request("waiter", direction="west")],
        occupancies=[stale_prediction],
        now=5.0,
    )

    owner = occupied.slot_for("owner")
    waiter = occupied.slot_for("waiter")
    assert owner.state is CorridorSlotState.COMMITTED
    assert owner.entry_time == 0.0
    assert owner.exit_time == 5.5
    assert waiter.entry_time >= owner.exit_time + 1.0
    assert occupied.slots[0].robot_id == "owner"

    # One missed observation remains bounded by the recheck reservation.
    missing_once = scheduler.update([], occupancies=[], now=5.2)
    assert missing_once.slot_for("owner") is not None
    assert [slot.robot_id for slot in missing_once.slots] == ["owner"]

    cleared = scheduler.update([], occupancies=[], now=6.1)
    assert cleared.slots == ()


def test_authoritative_absence_ends_stale_past_commit_after_grace() -> None:
    scheduler = _scheduler(occupancy_recheck_sec=0.1)
    stale_request = _request(
        "owner",
        duration=2.0,
        past_commit_point=True,
    )
    observed = CorridorOccupancy(
        robot_id="owner",
        regions=(REGION,),
        direction="east",
        entered_at=4.0,
        expected_exit_time=5.2,
        exit_lm="owner:exit",
        staging_lm="owner:holding",
        route_revision=1,
    )

    initial = scheduler.update(
        [stale_request],
        occupancies=[observed],
        now=5.0,
    )
    assert initial.slot_for("owner").physically_observed
    assert initial.slot_for("owner").exit_time == 5.2

    grace = scheduler.update(
        [stale_request],
        occupancies=[],
        now=5.1,
    )
    assert grace.slot_for("owner").exit_time == 5.2

    cleared = scheduler.update(
        [stale_request],
        occupancies=[],
        now=5.21,
    )
    assert cleared.slot_for("owner") is None
    assert cleared.decisions["owner"].status is (
        CorridorDecisionStatus.DEFERRED
    )
    assert "observed clear" in cleared.decisions["owner"].reason


def test_request_permutation_does_not_change_calendar() -> None:
    requests = (
        _request("a-east", direction="east"),
        _request("b-west", direction="west"),
        _request("c-east", direction="east", eta=1.0),
    )
    fingerprints = set()

    for ordering in permutations(requests):
        schedule = _scheduler().update(ordering, now=0.0)
        fingerprints.add(
            tuple(
                (
                    slot.robot_id,
                    slot.entry_time,
                    slot.exit_time,
                    slot.state,
                )
                for slot in schedule.slots
            )
        )

    assert len(fingerprints) == 1


def test_disjoint_explicit_corridors_can_run_in_parallel() -> None:
    other_region = "corridor:other"
    scheduler = CentralCorridorScheduler(
        {REGION, other_region},
        config=CorridorSchedulerConfig(commit_horizon_sec=1.0),
    )

    schedule = scheduler.update(
        [
            _request("first", regions=(REGION,)),
            _request("second", regions=(other_region,)),
        ],
        now=0.0,
    )

    assert schedule.slot_for("first").entry_time == 0.0
    assert schedule.slot_for("second").entry_time == 0.0


def test_multi_region_passage_releases_each_resource_on_its_own_window() -> None:
    region_a = "corridor:a"
    region_b = "corridor:b"
    scheduler = CentralCorridorScheduler(
        {region_a, region_b},
        config=CorridorSchedulerConfig(
            commit_horizon_sec=1.0,
            headway_sec=0.25,
            direction_change_sec=1.0,
        ),
    )
    leader = _request(
        "leader",
        regions=(region_a, region_b),
        direction="route:east",
        duration=10.0,
        entered=True,
    )
    leader = replace(
        leader,
        resource_windows=(
            CorridorResourceWindow(region_a, 0.0, 2.0, "flow:east"),
            CorridorResourceWindow(region_b, 8.0, 10.0, "flow:east"),
        ),
    )
    opposing_a = _request(
        "opposing-a",
        regions=(region_a,),
        direction="route:west",
        duration=2.0,
    )

    schedule = scheduler.update(
        [leader, opposing_a],
        occupancies=[],
        now=0.0,
    )

    leader_slot = schedule.slot_for("leader")
    opposing_slot = schedule.slot_for("opposing-a")
    assert opposing_slot.entry_time == 3.0
    assert opposing_slot.entry_time < leader_slot.exit_time


def test_local_resource_direction_controls_convoy_compatibility() -> None:
    scheduler = _scheduler()
    first = _request(
        "first",
        direction="route:south-west",
        entered=True,
    )
    first = replace(
        first,
        resource_windows=(
            CorridorResourceWindow(REGION, 0.0, 3.0, "flow:south"),
        ),
    )
    second = _request(
        "second",
        direction="route:south",
    )
    second = replace(
        second,
        resource_windows=(
            CorridorResourceWindow(REGION, 0.0, 3.0, "flow:south"),
        ),
    )

    schedule = scheduler.update(
        [first, second],
        occupancies=[],
        now=0.0,
    )

    assert schedule.slot_for("second").entry_time == 0.25


def test_opposite_local_flow_serializes_even_with_equal_route_direction() -> None:
    scheduler = _scheduler()
    first = _request("first", direction="route:shared", entered=True)
    first = replace(
        first,
        resource_windows=(
            CorridorResourceWindow(REGION, 0.0, 3.0, "flow:north"),
        ),
    )
    second = _request("second", direction="route:shared")
    second = replace(
        second,
        resource_windows=(
            CorridorResourceWindow(REGION, 0.0, 3.0, "flow:south"),
        ),
    )

    schedule = scheduler.update(
        [first, second],
        occupancies=[],
        now=0.0,
    )

    assert schedule.slot_for("second").entry_time == 4.0


def test_one_resource_conflict_shifts_the_complete_atomic_passage() -> None:
    region_a = "corridor:a"
    region_b = "corridor:b"
    scheduler = CentralCorridorScheduler(
        {region_a, region_b},
        config=CorridorSchedulerConfig(
            commit_horizon_sec=1.0,
            headway_sec=0.25,
            direction_change_sec=1.0,
        ),
    )
    leader = CorridorRequest(
        robot_id="leader",
        regions=(region_a, region_b),
        direction="route:east",
        earliest_entry=0.0,
        duration_sec=10.0,
        staging_lm="leader:start",
        exit_lm="leader:exit",
        route_revision=1,
        entered=True,
        resource_windows=(
            CorridorResourceWindow(region_a, 0.0, 2.0, "flow:east"),
            CorridorResourceWindow(region_b, 8.0, 10.0, "flow:east"),
        ),
    )
    follower = CorridorRequest(
        robot_id="follower",
        regions=(region_a, region_b),
        direction="route:west",
        earliest_entry=0.0,
        duration_sec=10.0,
        staging_lm="follower:start",
        exit_lm="follower:exit",
        route_revision=1,
        resource_windows=(
            CorridorResourceWindow(region_a, 0.0, 2.0, "flow:west"),
            CorridorResourceWindow(region_b, 8.0, 10.0, "flow:west"),
        ),
    )

    schedule = scheduler.update(
        [leader, follower],
        occupancies=[],
        now=0.0,
    )

    follower_slot = schedule.slot_for("follower")
    assert follower_slot.entry_time == 3.0
    follower_windows = {
        window.region_id: window
        for window in follower_slot.resource_windows
    }
    assert (
        follower_slot.entry_time
        + follower_windows[region_b].entry_offset_sec
        == 11.0
    )


def test_epoch_clock_does_not_false_reject_exact_resource_window() -> None:
    entry_time = 1_784_815_026.5018857
    duration = 0.001
    window = CorridorResourceWindow(
        region_id=REGION,
        entry_offset_sec=0.0,
        exit_offset_sec=duration,
        direction="flow:east",
    )

    slot = CorridorSlot(
        robot_id="robot",
        regions=(REGION,),
        direction="route:east",
        entry_time=entry_time,
        exit_time=entry_time + duration,
        staging_lm="A",
        exit_lm="B",
        route_revision=1,
        state=CorridorSlotState.COMMITTED,
        resource_windows=(window,),
    )
    occupancy = CorridorOccupancy(
        robot_id="robot",
        regions=(REGION,),
        direction="route:east",
        entered_at=entry_time,
        expected_exit_time=entry_time + duration,
        staging_lm="A",
        exit_lm="B",
        route_revision=1,
        resource_windows=(window,),
    )

    assert slot.resource_windows == (window,)
    assert occupancy.resource_windows == (window,)


def test_epoch_clock_still_rejects_real_resource_window_overshoot() -> None:
    entry_time = 1_784_815_026.5018857
    duration = 0.001
    window = CorridorResourceWindow(
        region_id=REGION,
        entry_offset_sec=0.0,
        exit_offset_sec=duration + 0.001,
        direction="flow:east",
    )

    with pytest.raises(
        ValueError,
        match="slot resource window cannot extend",
    ):
        CorridorSlot(
            robot_id="robot",
            regions=(REGION,),
            direction="route:east",
            entry_time=entry_time,
            exit_time=entry_time + duration,
            staging_lm="A",
            exit_lm="B",
            route_revision=1,
            state=CorridorSlotState.COMMITTED,
            resource_windows=(window,),
        )
    with pytest.raises(
        ValueError,
        match="occupancy resource window cannot extend",
    ):
        CorridorOccupancy(
            robot_id="robot",
            regions=(REGION,),
            direction="route:east",
            entered_at=entry_time,
            expected_exit_time=entry_time + duration,
            staging_lm="A",
            exit_lm="B",
            route_revision=1,
            resource_windows=(window,),
        )
