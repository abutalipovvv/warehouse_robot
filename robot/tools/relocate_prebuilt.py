#!/usr/bin/env python3
"""Relocate text metadata inside a prebuilt colcon install prefix."""

from __future__ import annotations

import argparse
from pathlib import Path


def relocate_prefix(
    root: Path,
    old_prefix: str,
    new_prefix: str | None = None,
) -> tuple[int, int]:
    root = root.resolve()
    if not root.is_dir():
        raise ValueError(f"install prefix does not exist: {root}")
    old = str(old_prefix or "").strip().encode("utf-8")
    if not old:
        raise ValueError("build-time install prefix is empty")
    new = str(new_prefix or root).strip().encode("utf-8")
    if not new:
        raise ValueError("destination install prefix is empty")
    changed = 0
    skipped_binary = 0
    for path in root.rglob("*"):
        if not path.is_file() or path.is_symlink():
            continue
        payload = path.read_bytes()
        if old not in payload:
            continue
        if b"\0" in payload:
            skipped_binary += 1
            continue
        path.write_bytes(payload.replace(old, new))
        changed += 1
    return changed, skipped_binary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--from-prefix", required=True)
    parser.add_argument("--to-prefix", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    changed, skipped_binary = relocate_prefix(
        args.root,
        args.from_prefix,
        args.to_prefix or None,
    )
    print(
        f"relocated {changed} metadata files in {args.root.resolve()}; "
        f"left {skipped_binary} binaries unchanged"
    )


if __name__ == "__main__":
    main()
