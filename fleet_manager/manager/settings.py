"""Typed, live views over the fleet's backward-compatible parameter mapping."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
from typing import Any, Mapping

from fleet_manager.core.mapping.navigation.params import ConfigurationError


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class SettingsSection:
    """Read one configuration section with consistent numeric validation."""

    values: Mapping[str, Any]
    path: str = "configuration"
    strict: bool = False

    def _corrected(
        self,
        name: str,
        raw: Any,
        corrected: Any,
        reason: str,
    ) -> Any:
        full_path = f"{self.path}.{name}"
        if self.strict:
            raise ConfigurationError(
                f"{full_path}: {reason}; received {raw!r}"
            )
        LOGGER.warning(
            "configuration compatibility correction: %s received=%r "
            "using=%r (%s)",
            full_path,
            raw,
            corrected,
            reason,
        )
        return corrected

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
            if name in self.values:
                raw = self._corrected(
                    name,
                    raw,
                    default,
                    "expected a finite number",
                )
            else:
                raw = default
        try:
            value = float(raw)
        except (TypeError, ValueError, OverflowError):
            value = float(
                self._corrected(
                    name,
                    raw,
                    default,
                    "expected a finite number",
                )
            )
        else:
            if name in self.values and (
                isinstance(raw, bool)
                or not isinstance(raw, (int, float))
            ):
                value = float(
                    self._corrected(
                        name,
                        raw,
                        value,
                        "expected a number",
                    )
                )
        if not math.isfinite(value):
            value = float(
                self._corrected(
                    name,
                    raw,
                    default,
                    "expected a finite number",
                )
            )
        if minimum is not None and value < float(minimum):
            value = float(
                self._corrected(
                    name,
                    raw,
                    minimum,
                    f"must be >= {minimum}",
                )
            )
        if maximum is not None and value > float(maximum):
            value = float(
                self._corrected(
                    name,
                    raw,
                    maximum,
                    f"must be <= {maximum}",
                )
            )
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
            if name in self.values:
                raw = self._corrected(
                    name,
                    raw,
                    default,
                    "expected an integer",
                )
            else:
                raw = default
        try:
            value = int(raw)
        except (TypeError, ValueError, OverflowError):
            value = int(
                self._corrected(
                    name,
                    raw,
                    default,
                    "expected an integer",
                )
            )
        else:
            if name in self.values and (
                isinstance(raw, bool) or not isinstance(raw, int)
            ):
                value = int(
                    self._corrected(
                        name,
                        raw,
                        value,
                        "expected an integer",
                    )
                )
        if minimum is not None and value < int(minimum):
            value = int(
                self._corrected(
                    name,
                    raw,
                    minimum,
                    f"must be >= {minimum}",
                )
            )
        if maximum is not None and value > int(maximum):
            value = int(
                self._corrected(
                    name,
                    raw,
                    maximum,
                    f"must be <= {maximum}",
                )
            )
        return value

    def flag(self, name: str, default: bool = False) -> bool:
        raw = self.values.get(name, default)
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, str):
            normalized = raw.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return bool(
                    self._corrected(
                        name,
                        raw,
                        True,
                        "expected a boolean",
                    )
                )
            if normalized in {"0", "false", "no", "off", ""}:
                return bool(
                    self._corrected(
                        name,
                        raw,
                        False,
                        "expected a boolean",
                    )
                )
        if name not in self.values:
            return bool(default)
        return bool(
            self._corrected(
                name,
                raw,
                default,
                "expected a boolean",
            )
        )

    def text(self, name: str, default: str = "") -> str:
        raw = self.values.get(name, default)
        if isinstance(raw, str):
            return raw
        if name not in self.values:
            return str(default)
        return str(
            self._corrected(
                name,
                raw,
                default,
                "expected text",
            )
        )


class FleetSettings:
    """Live section access that remains valid when callers mutate ``params``."""

    __slots__ = ("_root", "_strict")

    def __init__(
        self,
        root: Mapping[str, Any],
        *,
        strict: bool | None = None,
    ) -> None:
        self._root = root
        self._strict = (
            bool(root.get("strict_configuration", False))
            if strict is None
            else bool(strict)
        )

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
        if not isinstance(value, dict):
            if value is not None:
                section = SettingsSection(
                    {},
                    path="configuration",
                    strict=self._strict,
                )
                section._corrected(
                    name,
                    value,
                    {},
                    "expected a mapping",
                )
            value = {}
        return SettingsSection(
            value,
            path=f"configuration.{name}",
            strict=self._strict,
        )
