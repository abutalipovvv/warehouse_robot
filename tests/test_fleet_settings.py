from __future__ import annotations

import math
from io import StringIO
import logging

import pytest

from fleet_manager.core.mapping.navigation.params import ConfigurationError
from fleet_manager.manager.settings import LOGGER, FleetSettings, SettingsSection


def test_numeric_settings_validate_and_clamp_values() -> None:
    section = SettingsSection(
        {
            "valid": "2.5",
            "bad": "not-a-number",
            "infinite": math.inf,
            "zero": 0,
        }
    )

    assert section.number("valid", 1.0, minimum=0.0, maximum=2.0) == 2.0
    assert section.number("bad", 1.5) == 1.5
    assert section.number("infinite", 1.5) == 1.5
    assert section.number("zero", 1.5, default_if_falsy=True) == 1.5


def test_integer_and_boolean_settings_have_predictable_coercion() -> None:
    section = SettingsSection(
        {
            "workers": "5",
            "too_many": 100,
            "enabled": "yes",
            "disabled": "false",
            "unknown": "perhaps",
        }
    )

    assert section.integer("workers", 1, minimum=1) == 5
    assert section.integer("too_many", 1, maximum=8) == 8
    assert section.flag("enabled") is True
    assert section.flag("disabled", True) is False
    assert section.flag("unknown", True) is True


def test_fleet_settings_are_a_live_view_of_mutable_params() -> None:
    params = {"fleet": {"timeout": 1.0}}
    settings = FleetSettings(params)

    assert settings.fleet.number("timeout", 0.8) == 1.0

    params["fleet"]["timeout"] = 2.0
    assert settings.fleet.number("timeout", 0.8) == 2.0

    params["fleet"] = {"timeout": 3.0}
    assert settings.fleet.number("timeout", 0.8) == 3.0


@pytest.mark.parametrize(
    ("values", "read", "path"),
    (
        ({"timeout": "bad"}, lambda section: section.number("timeout", 1.0), "fleet.timeout"),
        ({"timeout": 100.0}, lambda section: section.number("timeout", 1.0, maximum=10.0), "fleet.timeout"),
        ({"workers": "4"}, lambda section: section.integer("workers", 1), "fleet.workers"),
        ({"enabled": "perhaps"}, lambda section: section.flag("enabled"), "fleet.enabled"),
    ),
)
def test_strict_settings_reject_invalid_values_with_full_path(
    values,
    read,
    path: str,
) -> None:
    section = SettingsSection(
        values,
        path="configuration.fleet",
        strict=True,
    )

    with pytest.raises(ConfigurationError, match=path):
        read(section)


def test_compatibility_settings_warn_for_fallback_and_clamp() -> None:
    settings = FleetSettings({"fleet": {"timeout": "too-fast"}})
    stream = StringIO()
    handler = logging.StreamHandler(stream)
    LOGGER.addHandler(handler)

    try:
        value = settings.fleet.number(
            "timeout",
            1.5,
            minimum=0.2,
        )
    finally:
        LOGGER.removeHandler(handler)

    assert value == 1.5
    warning = stream.getvalue()
    assert "configuration.fleet.timeout" in warning
    assert "received='too-fast'" in warning
    assert "using=1.5" in warning


def test_strict_settings_reject_non_mapping_section() -> None:
    settings = FleetSettings(
        {"strict_configuration": True, "fleet": "invalid"}
    )

    with pytest.raises(ConfigurationError, match="configuration.fleet"):
        _ = settings.fleet
