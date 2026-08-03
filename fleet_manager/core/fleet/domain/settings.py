"""Typed, live views over the fleet's backward-compatible parameter mapping."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True, slots=True)
class SettingsSection:
    """Read one configuration section with consistent numeric validation."""

    values: Mapping[str, Any]

    def number(
        self,
        name: str,
        default: float,
        *,
        minimum: float | None = None,
        maximum: float | None = None,
        default_if_falsy: bool = False,
    ) -> float:
        raw = self.values.get(name, default)
        if raw is None or (default_if_falsy and not raw):
            raw = default
        try:
            value = float(raw)
        except (TypeError, ValueError, OverflowError):
            value = float(default)
        if not math.isfinite(value):
            value = float(default)
        if minimum is not None:
            value = max(float(minimum), value)
        if maximum is not None:
            value = min(float(maximum), value)
        return value

    def integer(
        self,
        name: str,
        default: int,
        *,
        minimum: int | None = None,
        maximum: int | None = None,
        default_if_falsy: bool = False,
    ) -> int:
        raw = self.values.get(name, default)
        if raw is None or (default_if_falsy and not raw):
            raw = default
        try:
            value = int(raw)
        except (TypeError, ValueError, OverflowError):
            value = int(default)
        if minimum is not None:
            value = max(int(minimum), value)
        if maximum is not None:
            value = min(int(maximum), value)
        return value

    def flag(self, name: str, default: bool = False) -> bool:
        raw = self.values.get(name, default)
        if isinstance(raw, str):
            normalized = raw.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return True
            if normalized in {"0", "false", "no", "off", ""}:
                return False
            return bool(default)
        return bool(raw)

    def text(self, name: str, default: str = "") -> str:
        raw = self.values.get(name, default)
        return str(default if raw is None else raw)


class FleetSettings:
    """Live section access that remains valid when callers mutate ``params``."""

    __slots__ = ("_root",)

    def __init__(self, root: Mapping[str, Any]) -> None:
        self._root = root

    @property
    def fleet(self) -> SettingsSection:
        return self.section("fleet")

    @property
    def navigation(self) -> SettingsSection:
        return self.section("navigation")

    @property
    def planner(self) -> SettingsSection:
        return self.section("planner")

    @property
    def localization(self) -> SettingsSection:
        return self.section("localization")

    @property
    def robot_model(self) -> SettingsSection:
        return self.section("robot_model")

    def section(self, name: str) -> SettingsSection:
        value = self._root.get(name)
        return SettingsSection(value if isinstance(value, dict) else {})


__all__ = ["FleetSettings", "SettingsSection"]
