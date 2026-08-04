"""Shared public contracts for operator application state."""

from __future__ import annotations

from datetime import datetime, timezone


OPERATOR_CONTROL_OWNER_ID = "operator-app"
OPERATOR_CONTROL_OWNER_NAME = "Operator App"


class RobotProbeError(RuntimeError):
    """The requested robot endpoint could not be probed."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
