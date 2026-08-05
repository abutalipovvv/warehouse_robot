from __future__ import annotations

from contextlib import AbstractContextManager
from types import SimpleNamespace
from typing import Any

from fleet_manager.manager.coordination.planning.planning import TrafficPlanningMixin
from fleet_manager.manager.coordination.planning.continuous import (
    TrafficContinuousWaitSchedulingMixin,
)
from fleet_manager.manager.coordination.planning.preparation import (
    TrafficPlanPreparationMixin,
)
from fleet_manager.manager.coordination.planning.reservations import (
    TrafficReservationMixin,
)
from fleet_manager.manager.coordination.planning.results import (
    TrafficPlanResultMixin,
)


class RecordingLock(AbstractContextManager["RecordingLock"]):
    def __init__(self) -> None:
        self.entries = 0
        self.exits = 0

    def __enter__(self) -> RecordingLock:
        self.entries += 1
        return self

    def __exit__(self, *args: object) -> None:
        self.exits += 1


class PlanningHarness(TrafficPlanningMixin):
    def __init__(self) -> None:
        self._planner_lock = RecordingLock()
        self.calls: list[
            tuple[list[dict[str, Any]], dict[str, Any] | None]
        ] = []

    def _plan_valid_requests_unlocked(
        self,
        valid_requests: list[dict[str, Any]],
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        self.calls.append((valid_requests, payload))
        return {"ok": True}


class RecordingPlanner:
    def __init__(self) -> None:
        self.payloads: list[dict[str, Any]] = []

    def plan(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.payloads.append(payload)
        return {
            "ok": True,
            "plans": [],
            "debug": {"reason": "success"},
        }


class PreparationHarness(TrafficPlanPreparationMixin):
    def __init__(self) -> None:
        self.landmarks = {
            name: SimpleNamespace(name=name)
            for name in ("A", "B", "C", "H")
        }
        self.planner = RecordingPlanner()
        self.continuous_call: dict[str, Any] = {}
        self.edge_ignore: set[str] = set()
        self.vertex_ignore: set[str] = set()
        self.vertex_ignore_nodes: set[str] = set()

    def _hard_blocked_lms(
        self,
        payload: dict[str, Any],
    ) -> set[str]:
        return {"H"}

    def _hard_blocked_edges(
        self,
        payload: dict[str, Any],
    ) -> set[tuple[str, str]]:
        return {("A", "B")}

    def _dynamic_blocked_edges(self) -> set[tuple[str, str]]:
        return {("B", "C")}

    def _release_blocker_names_for_requests(
        self,
        requests: list[dict[str, Any]],
    ) -> set[str]:
        return {"held"}

    def _superseded_runtime_replan_holder_names(
        self,
        requests: list[dict[str, Any]],
    ) -> set[str]:
        return {"superseded"}

    def _bootstrap_departure_robot_names(
        self,
        requests: list[dict[str, Any]],
    ) -> set[str]:
        return {"departing"}

    def _reserved_edge_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, str, float, float, str]]:
        self.edge_ignore = set(ignore_robot_names or ())
        assert prediction_offset == 1.5
        return [("A", "B", 0.25, 1.25, "peer")]

    def _reserved_vertex_intervals(
        self,
        requests: list[dict[str, Any]],
        ignore_robot_names: set[str] | None = None,
        ignore_nodes: set[str] | None = None,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, float, float, str]]:
        self.vertex_ignore = set(ignore_robot_names or ())
        self.vertex_ignore_nodes = set(ignore_nodes or ())
        assert prediction_offset == 1.5
        return [("B", 0.1, 0.8, "peer")]

    def _held_blocker_vertex_intervals(
        self,
        robot_names: set[str],
        *,
        prediction_offset: float = 0.0,
    ) -> list[tuple[str, float, float, str]]:
        assert robot_names == {"held", "superseded"}
        assert prediction_offset == 1.5
        return [("H", 0.0, 8.0, "held")]

    def _soft_blocked_lms(
        self,
        requests: list[dict[str, Any]],
        hard_blocked_lms: set[str],
    ) -> set[str]:
        return set()

    def _apply_continuous_reservation_waits(
        self,
        result: dict[str, Any],
        **kwargs: Any,
    ) -> dict[str, Any]:
        self.continuous_call = kwargs
        return result

    def _event(self, level: str, message: str) -> None:
        raise AssertionError("no event expected")


class ReservationHarness(TrafficReservationMixin):
    def __init__(self) -> None:
        self.landmarks = {
            name: SimpleNamespace(name=name)
            for name in ("A", "B")
        }


class ResultRobot:
    def __init__(self) -> None:
        self.status = "IDLE"
        self.current_lm = "A"
        self.target_lm = ""
        self.trajectory: list[dict[str, Any]] = []
        self.trajectory_dirty = False
        self.plan_nodes: list[str] = []
        self.route_started_at = 0.0
        self.route_clock = 9.0
        self.last_tick_at = 0.0
        self.pose: dict[str, float] | None = {"x": -1.0}
        self.route_note = ""
        self.last_reason = ""
        self.blocked_since = 1.0
        self.traffic_stall_since = 2.0
        self.active_order_id = ""
        self.updated_at = 0.0

    def is_remote(self) -> bool:
        return False


class ResultHarness(TrafficPlanResultMixin):
    def __init__(self) -> None:
        self.robot = ResultRobot()
        self.robots = {"robot": self.robot}
        self.cleared_wait: list[str] = []
        self.cleared_retreat: list[str] = []

    def _now(self) -> float:
        return 99.0

    def _nearest_trajectory_clock(
        self,
        trajectory: list[dict[str, Any]],
        pose: dict[str, float],
    ) -> float:
        return 7.0

    def _pose_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> dict[str, float] | None:
        return {"x": float(trajectory[0]["x"])}

    def _clear_wait_dependency(self, robot: ResultRobot) -> None:
        self.cleared_wait.append(robot.current_lm)

    def _clear_deadlock_retreat(self, robot: ResultRobot) -> None:
        self.cleared_retreat.append(robot.current_lm)


def test_facade_composes_focused_planning_components() -> None:
    assert TrafficPlanningMixin.__bases__ == (
        TrafficPlanPreparationMixin,
        TrafficContinuousWaitSchedulingMixin,
        TrafficReservationMixin,
        TrafficPlanResultMixin,
    )
    assert (
        TrafficPlanningMixin._plan_valid_requests_unlocked
        is TrafficPlanPreparationMixin._plan_valid_requests_unlocked
    )


def test_facade_serializes_reusable_planner_access() -> None:
    harness = PlanningHarness()
    requests = [{"name": "robot", "startLm": "A", "goalLm": "B"}]
    payload = {"reservationOffsetSec": 1.25}

    assert harness._plan_valid_requests(requests, payload) == {"ok": True}
    assert harness.calls == [(requests, payload)]
    assert harness._planner_lock.entries == 1
    assert harness._planner_lock.exits == 1


def test_preparation_builds_one_normalized_planner_payload() -> None:
    harness = PreparationHarness()
    requests = [{"name": "moving", "startLm": "A", "goalLm": "C"}]

    result = harness._plan_valid_requests_unlocked(
        requests,
        {"reservationOffsetSec": "1.5"},
    )

    assert result["ok"]
    assert len(harness.planner.payloads) == 1
    payload = harness.planner.payloads[0]
    assert payload["robots"] == requests
    assert payload["blocked_lms"] == ["H"]
    assert payload["blocked_edges"] == [
        {"from": "A", "to": "B"},
        {"from": "B", "to": "C"},
    ]
    assert payload["reserved_edge_intervals"] == [
        {
            "from": "A",
            "to": "B",
            "start": 0.25,
            "end": 1.25,
            "robot": "peer",
        }
    ]
    assert payload["reserved_vertex_intervals"] == [
        {
            "node": "B",
            "start": 0.1,
            "end": 0.8,
            "robot": "peer",
        },
        {
            "node": "H",
            "start": 0.0,
            "end": 8.0,
            "robot": "held",
        },
    ]
    expected_ignored = {"held", "superseded", "departing"}
    assert harness.edge_ignore == expected_ignored
    assert harness.vertex_ignore == expected_ignored
    assert harness.vertex_ignore_nodes == {"A"}
    assert harness.continuous_call == {
        "ignore_robot_names": {"departing"},
        "stationary_robot_names": {"held", "superseded"},
        "prediction_offset": 1.5,
    }


def test_continuous_scheduler_inserts_wait_without_mutating_tail_times() -> None:
    scheduler = TrafficContinuousWaitSchedulingMixin()
    trajectory = [
        {"t": 0.0, "x": 0.0, "edgeId": "A->B"},
        {"t": 2.0, "x": 2.0, "edgeId": "A->B"},
    ]

    scheduled = scheduler._insert_trajectory_wait(
        trajectory,
        insert_index=0,
        wait_duration=1.5,
    )

    assert [sample["t"] for sample in scheduled] == [0.0, 1.5, 3.5]
    assert scheduled[1]["edgeId"] == "WAIT@A->B"
    assert [sample["t"] for sample in trajectory] == [0.0, 2.0]


def test_reservation_helpers_keep_boundary_semantics() -> None:
    harness = ReservationHarness()
    trajectory = [
        {"t": 0.0},
        {"t": 1.0},
        {"t": 2.0},
        {"t": 3.0},
    ]

    assert harness._trajectory_segment_index(trajectory, 1.0) == 1
    assert (
        harness._trajectory_segment_index(
            trajectory,
            1.0,
            boundary_belongs_to_previous=True,
        )
        == 0
    )
    assert harness._trajectory_sample_index_at_or_before(
        trajectory,
        1.0,
    ) == 1
    assert harness._trajectory_sample_index_at_or_before(
        trajectory,
        -0.1,
    ) == -1
    assert harness._parse_edge_id("A->B") == ("A", "B")
    assert harness._parse_edge_id("B->B") is None
    assert harness._parse_edge_id("A->missing") is None


def test_result_component_explains_waits_and_deadlocks() -> None:
    harness = ResultHarness()

    assert harness._plan_note(
        {"debug": {"reason": "success", "continuousWaits": 2}}
    ) == "WAIT: reserved corridor"
    assert harness._plan_note(
        {"debug": {"reason": "success:fallback_wait"}}
    ) == "FALLBACK_WAIT"
    assert harness._planner_deadlock_result(
        {"debug": {"continuousUnresolved": "1"}}
    )
    assert harness._planner_failure_reason(
        {
            "debug": {
                "deadlock": True,
                "deadlockReason": "peer never clears",
            }
        }
    ) == "deadlock: peer never clears"


def test_result_component_applies_an_accepted_plan() -> None:
    harness = ResultHarness()
    trajectory = [
        {"t": 0.0, "x": 1.0},
        {"t": 2.0, "x": 3.0},
    ]

    harness._apply_planner_result(
        {
            "plans": [
                {
                    "robot": "robot",
                    "startLm": "A",
                    "goalLm": "B",
                    "nodes": ["A", "B"],
                    "trajectory": trajectory,
                }
            ],
            "debug": {"reason": "success"},
        },
        now=123.0,
        order_id="order-1",
    )

    robot = harness.robot
    assert robot.status == "MOVING"
    assert robot.target_lm == "B"
    assert robot.trajectory == trajectory
    assert robot.trajectory_dirty
    assert robot.plan_nodes == ["A", "B"]
    assert robot.route_started_at == 123.0
    assert robot.route_clock == 0.0
    assert robot.pose == {"x": 1.0}
    assert robot.route_note == "planner accepted"
    assert robot.active_order_id == "order-1"
    assert robot.updated_at == 123.0
    assert harness.cleared_wait == ["A"]
    assert harness.cleared_retreat == ["A"]
