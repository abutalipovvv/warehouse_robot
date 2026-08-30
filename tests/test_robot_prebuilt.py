from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RELOCATE_SCRIPT = ROOT / "robot" / "tools" / "relocate_prebuilt.py"
SPEC = importlib.util.spec_from_file_location("relocate_prebuilt", RELOCATE_SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_prebuilt_bundle_matches_supported_platform() -> None:
    bundle = (
        ROOT
        / "robot"
        / "prebuilt"
        / "ubuntu-24.04-x86_64-ros-jazzy"
    )
    assert (bundle / "ros2_libs-install.tar.zst").is_file()
    assert (bundle / "simulation-install.tar.zst").is_file()
    assert (bundle / "SHA256SUMS").is_file()


def test_relocator_rewrites_metadata_but_not_binary_payloads(
    tmp_path: Path,
) -> None:
    install = tmp_path / "install"
    install.mkdir()
    old_prefix = "/build/host/robot/ros2_libs/install"
    new_prefix = "/checkout/robot/ros2_libs/install"
    metadata = install / "package.sh"
    metadata.write_text(f"prefix={old_prefix}\n", encoding="utf-8")
    binary = install / "library.so"
    binary.write_bytes(b"\x7fELF\0" + old_prefix.encode("utf-8"))

    changed, skipped = MODULE.relocate_prefix(
        install,
        old_prefix,
        new_prefix,
    )

    assert changed == 1
    assert skipped == 1
    assert metadata.read_text(encoding="utf-8") == f"prefix={new_prefix}\n"
    assert old_prefix.encode("utf-8") in binary.read_bytes()
