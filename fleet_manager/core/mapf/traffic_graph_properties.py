"""Read traffic-policy values from loosely typed map properties."""

from __future__ import annotations

from collections.abc import Mapping


def property_map(value: object) -> Mapping[str, object]:
    """Return a safe mapping view for an optional properties value."""

    return value if isinstance(value, Mapping) else {}


def read_bool(
    properties: Mapping[str, object],
    keys: tuple[str, ...],
    default: bool,
) -> bool:
    for key in keys:
        if key not in properties:
            continue
        value = properties.get(key)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
    return default


def read_float(
    properties: Mapping[str, object],
    keys: tuple[str, ...],
    default: float,
) -> float:
    for key in keys:
        if key not in properties:
            continue
        try:
            return max(0.02, float(properties[key]))
        except (TypeError, ValueError):
            continue
    return default


def read_int(
    properties: Mapping[str, object],
    keys: tuple[str, ...],
    default: int,
) -> int:
    for key in keys:
        if key not in properties:
            continue
        try:
            return max(1, int(properties[key]))
        except (TypeError, ValueError):
            continue
    return default


def read_text(
    properties: Mapping[str, object],
    keys: tuple[str, ...],
) -> str:
    for key in keys:
        value = str(properties.get(key) or "").strip()
        if value:
            return value
    return ""


def read_text_tuple(
    properties: Mapping[str, object],
    keys: tuple[str, ...],
) -> tuple[str, ...]:
    value = read_text(properties, keys)
    if not value:
        return ()
    return tuple(item.strip() for item in value.split(",") if item.strip())


def has_any(
    properties: Mapping[str, object],
    keys: tuple[str, ...],
) -> bool:
    return any(key in properties for key in keys)
