from __future__ import annotations

from pathlib import Path
import sys

import pytest


MOTION_GATEWAY_SRC = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "robot_driver"
    / "src"
    / "robot_motion_gateway"
)
if str(MOTION_GATEWAY_SRC) not in sys.path:
    sys.path.insert(0, str(MOTION_GATEWAY_SRC))

from robot_motion_gateway.config import load_gateway_settings
from robot_motion_gateway.model import (
    MotionArbiter,
    MotionLimits,
    MotionMode,
    MotionTimeouts,
)


def test_gateway_only_forwards_the_source_owned_by_the_mode() -> None:
    arbiter = MotionArbiter()
    arbiter.accept("route", linear=0.4, angular=0.2, received_at=10.0)
    arbiter.accept("teleop", linear=1.0, angular=-0.5, received_at=10.0)

    assert arbiter.decide(now=10.1).linear == 0.0

    arbiter.transition(MotionMode.ROUTE, reason="route accepted")
    # A mode transition clears commands from the previous ownership epoch.
    assert arbiter.decide(now=10.1).watchdog_stop is True
    arbiter.accept("route", linear=0.4, angular=0.2, received_at=10.1)
    decision = arbiter.decide(now=10.2)

    assert decision.active_source == "route"
    assert decision.linear == pytest.approx(0.4)
    assert decision.angular == pytest.approx(0.2)


def test_gateway_watchdog_stops_a_stale_source() -> None:
    arbiter = MotionArbiter(
        timeouts=MotionTimeouts(route=0.2, teleop=0.4, nav2=0.3),
    )
    arbiter.transition("TELEOP")
    arbiter.accept("teleop", linear=0.8, angular=0.0, received_at=2.0)

    assert arbiter.decide(now=2.39).watchdog_stop is False
    stopped = arbiter.decide(now=2.41)
    assert stopped.watchdog_stop is True
    assert stopped.linear == 0.0
    assert stopped.angular == 0.0


def test_gateway_clamps_forward_backward_and_angular_velocity() -> None:
    arbiter = MotionArbiter(
        limits=MotionLimits(
            max_forward_speed=1.2,
            max_backward_speed=0.4,
            max_angular_speed=0.8,
        ),
    )
    arbiter.transition("ROUTE")
    arbiter.accept("route", linear=-4.0, angular=3.0, received_at=5.0)

    decision = arbiter.decide(now=5.1)
    assert decision.linear == pytest.approx(-0.4)
    assert decision.angular == pytest.approx(0.8)


def test_gateway_settings_follow_shared_robot_params(tmp_path: Path) -> None:
    params = tmp_path / "params.yaml"
    params.write_text(
        """
manual:
  linear_speed: 1.37
  angular_speed: 1.6
navigation:
  max_angular_speed: 0.9
  trajectory_speed_profile:
    max_forward_speed: 0.8
    max_backward_speed: 0.3
motion_gateway:
  route_timeout_sec: 0.22
  publish_rate_hz: 60
""".strip(),
        encoding="utf-8",
    )

    settings = load_gateway_settings(params)

    assert settings.limits.max_forward_speed == pytest.approx(1.37)
    assert settings.limits.max_backward_speed == pytest.approx(1.37)
    assert settings.limits.max_angular_speed == pytest.approx(1.6)
    assert settings.timeouts.route == pytest.approx(0.22)
    assert settings.publish_rate_hz == pytest.approx(60.0)
