from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = ROOT / "sim_robot" / "ws" / "src" / "launch" / "launch" / "launch.py"


def _launch_module():
    spec = importlib.util.spec_from_file_location("robot_launch_paths_test", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_installed_robot_launch_defaults_stay_inside_workspace(monkeypatch) -> None:
    module = _launch_module()
    installed_share = ROOT / "install" / "robot_launch" / "share" / "robot_launch"
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(installed_share),
    )

    project_root = module._project_root()
    maps_root = module._maps_root(project_root)

    assert project_root == ROOT
    assert maps_root == ROOT / "sim_robot" / "ws" / "src" / "robot_map_manager" / "maps_out"
    assert module._params_path(project_root) == ROOT / "sim_robot" / "ws" / "src" / "params.yaml"
    assert module._default_active_map_dir(maps_root).is_dir()
    assert module._map_state_file(maps_root) == (
        ROOT / "sim_robot" / "ws" / "src" / "robot_map_manager" / ".active_map.json"
    )


def test_colcon_install_fallback_returns_workspace_not_home(tmp_path, monkeypatch) -> None:
    module = _launch_module()
    installed_share = tmp_path / "install" / "robot_launch" / "share" / "robot_launch"
    installed_share.mkdir(parents=True)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(installed_share),
    )

    assert module._project_root() == tmp_path
