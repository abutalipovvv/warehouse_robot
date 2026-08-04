from __future__ import annotations

import math

from fleet_manager.core.fleet.domain.settings import FleetSettings, SettingsSection


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
