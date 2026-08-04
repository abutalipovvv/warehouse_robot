from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from fleet_manager.core.mapf.fleet.fleet_planner_backends import BackendSelector
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.core.mapping.navigation.params import (
    ConfigurationError,
    load_route_params,
)
from fleet_manager.core.mapping.navigation.planner import LmRoutePlanner


def _edge(source: str, target: str, length: float = 1.0) -> GraphEdge:
    return GraphEdge(
        from_name=source,
        to_name=target,
        length=length,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(length, 0.0)),
        properties={"direction": 1},
    )


def _route_planner() -> LmRoutePlanner:
    landmarks = {
        "S": Landmark(name="S", x=0.0, y=0.0),
        "A": Landmark(name="A", x=1.0, y=1.0),
        "B": Landmark(name="B", x=1.0, y=-1.0),
        "G": Landmark(name="G", x=2.0, y=0.0),
        "X": Landmark(name="X", x=8.0, y=8.0),
    }
    # B is intentionally authored first. Historical spatial A* still chose A
    # because its heap tie-break was the landmark name.
    edges = [
        _edge("S", "B"),
        _edge("S", "A"),
        _edge("B", "G"),
        _edge("A", "G"),
    ]
    return LmRoutePlanner(
        landmarks,
        edges,
        params={"planner": {"trajectory_sample_distance": 0.05}},
    )


def test_spatial_route_uses_shared_astar_with_old_tie_breaking() -> None:
    planner = _route_planner()

    assert planner.find_route("S", "G").nodes == ["S", "A", "G"]
    assert planner.find_route("S", "G").nodes == ["S", "A", "G"]


def test_spatial_route_preserves_forbidden_edges_and_congestion_costs() -> None:
    planner = _route_planner()

    blocked = planner.find_route("S", "G", blocked_edges={("S", "A")})
    congested = planner.find_route(
        "S",
        "G",
        edge_penalties={("S", "A"): 5.0},
    )

    assert blocked.nodes == ["S", "B", "G"]
    assert congested.nodes == ["S", "B", "G"]
    assert congested.length == 2.0


def test_spatial_route_reports_unreachable_cancellation_and_limit() -> None:
    planner = _route_planner()

    with pytest.raises(ValueError, match="No route found"):
        planner.find_route("S", "X")
    with pytest.raises(ValueError, match="cancelled"):
        planner.find_route("S", "G", should_cancel=lambda: True)
    with pytest.raises(ValueError, match="expansion limit"):
        planner.find_route("S", "G", max_expansions=1)


def _write_yaml(path: Path, values: dict[str, object]) -> None:
    path.write_text(yaml.safe_dump(values), encoding="utf-8")


def test_current_production_config_passes_strict_validation() -> None:
    params = load_route_params(strict=True)

    assert params["strict_configuration"] is True
    assert params["fleet"]["planner_backend"] == "hybrid"


def test_strict_config_rejects_unknown_key_with_full_path(
    tmp_path: Path,
) -> None:
    path = tmp_path / "params.yaml"
    _write_yaml(path, {"fleet": {"planner_backned": "hybrid"}})

    with pytest.raises(
        ConfigurationError,
        match=r"configuration\.fleet\.planner_backned.*'hybrid'",
    ):
        load_route_params(path, strict=True)


def test_strict_config_rejects_backend_type_and_contradiction(
    tmp_path: Path,
) -> None:
    backend_path = tmp_path / "backend.yaml"
    _write_yaml(backend_path, {"fleet": {"planner_backend": "hiprid"}})
    with pytest.raises(ConfigurationError, match="unknown backend"):
        load_route_params(backend_path, strict=True)

    type_path = tmp_path / "type.yaml"
    _write_yaml(
        type_path,
        {"fleet": {"reservation_time_step_sec": "fast"}},
    )
    with pytest.raises(
        ConfigurationError,
        match=r"configuration\.fleet\.reservation_time_step_sec.*'fast'",
    ):
        load_route_params(type_path, strict=True)

    contradiction_path = tmp_path / "contradiction.yaml"
    _write_yaml(
        contradiction_path,
        {
            "fleet": {
                "controlled_corridor_max_direction_batch": 5,
                "controlled_corridor_max_adaptive_direction_batch": 2,
            }
        },
    )
    with pytest.raises(ConfigurationError, match="must be >="):
        load_route_params(contradiction_path, strict=True)


def test_compatibility_backend_fallback_emits_warning(capsys) -> None:
    selector = BackendSelector(strict=False)

    selected = selector.normalize("hiprid")

    assert selected == "cbs"
    assert "unknown backend" in capsys.readouterr().err
    with pytest.raises(ValueError, match="payload.plannerBackend"):
        BackendSelector(strict=True).normalize(
            "hiprid",
            path="payload.plannerBackend",
        )
