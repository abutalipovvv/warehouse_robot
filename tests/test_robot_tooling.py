from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "robot" / "tools"


def test_all_colcon_build_scripts_use_release_mode() -> None:
    for name in (
        "build_ros2_libs.sh",
        "build_robot_driver.sh",
        "build_simulation.sh",
    ):
        script = (TOOLS / name).read_text(encoding="utf-8")
        assert "--cmake-args -DCMAKE_BUILD_TYPE=Release" in script


def test_simulation_runner_activates_overlay_and_launches_full_stack() -> None:
    script = (TOOLS / "run_simulation.sh").read_text(encoding="utf-8")
    assert 'set +u\nsource "${SETUP_FILE}"\nset -u' in script
    assert 'source "${SETUP_FILE}"' in script
    assert "exec ros2 launch stage_ros2 simulation.launch.py" in script
