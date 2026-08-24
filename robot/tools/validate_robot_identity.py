#!/usr/bin/env python3
"""Validate the persistent ROS/CycloneDDS identity of one physical robot."""

from __future__ import annotations

import argparse
from pathlib import Path
import re


ROBOT_ID_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_-]{1,63}")
NAMESPACE_PATTERN = re.compile(
    r"[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*"
)


def load_env_file(path: Path) -> dict[str, str]:
    if not path.is_file():
        raise ValueError(f"identity file does not exist: {path}")
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator or not key.strip():
            raise ValueError(f"invalid environment line {line_number}: {raw_line}")
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def validate_identity(values: dict[str, str]) -> dict[str, object]:
    robot_id = values.get("ROBOT_ID", "").strip()
    namespace = values.get("ROS_NAMESPACE", "").strip().strip("/")
    if ROBOT_ID_PATTERN.fullmatch(robot_id) is None:
        raise ValueError(
            "ROBOT_ID must start with a letter and contain 2-64 letters, "
            "digits, underscores or hyphens"
        )
    if NAMESPACE_PATTERN.fullmatch(namespace) is None:
        raise ValueError("ROS_NAMESPACE is empty or is not a valid ROS namespace")
    raw_domain = values.get("ROS_DOMAIN_ID", "").strip()
    try:
        domain_id = int(raw_domain)
    except ValueError as exc:
        raise ValueError("ROS_DOMAIN_ID must be an integer") from exc
    if not 0 <= domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be between 0 and 232")
    if values.get("RMW_IMPLEMENTATION", "").strip() != "rmw_cyclonedds_cpp":
        raise ValueError("RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp")
    uri = values.get("CYCLONEDDS_URI", "").strip()
    if not uri.startswith("file://"):
        raise ValueError("CYCLONEDDS_URI must use a file:// path")
    config_path = Path(uri.removeprefix("file://")).expanduser()
    if not config_path.is_file():
        raise ValueError(f"CycloneDDS config does not exist: {config_path}")
    return {
        "robotId": robot_id,
        "namespace": namespace,
        "domainId": domain_id,
        "cycloneDdsConfig": str(config_path.resolve()),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env-file", required=True, type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = validate_identity(load_env_file(args.env_file))
    print(
        "robot identity is valid: "
        f"id={result['robotId']} namespace=/{result['namespace']} "
        f"domain={result['domainId']}"
    )


if __name__ == "__main__":
    main()
