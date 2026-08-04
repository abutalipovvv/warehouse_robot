"""Events emitted by Fleet Manager orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class FleetEvent:
    stamp: float
    level: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp": self.stamp,
            "level": self.level,
            "message": self.message,
        }
