from __future__ import annotations

import json
import os
from pathlib import Path

from .models import KnownRobot


def default_registry_path() -> Path:
    override = os.environ.get("WAREHOUSE_OPERATOR_REGISTRY", "").strip()
    if override:
        return Path(override).expanduser()
    return Path.home() / ".config" / "warehouse_operator" / "robots.json"


class RobotRegistry:
    def __init__(self, path: Path | None = None) -> None:
        self.path = (path or default_registry_path()).expanduser()

    def load(self) -> list[KnownRobot]:
        if not self.path.exists():
            return []
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return []
        items = payload.get("robots", [])
        if not isinstance(items, list):
            return []
        robots: list[KnownRobot] = []
        for item in items:
            if not isinstance(item, dict):
                continue
            robot = KnownRobot.from_dict(item)
            if robot.id and robot.host:
                robots.append(robot)
        return robots

    def save(self, robots: list[KnownRobot]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"robots": [robot.to_dict() for robot in robots]}
        self.path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def upsert(self, robot: KnownRobot) -> KnownRobot:
        robots = self.load()
        updated: list[KnownRobot] = []
        replaced = False
        for item in robots:
            same_endpoint = (
                (not item.is_ros2)
                and (not robot.is_ros2)
                and item.host == robot.host
                and item.port == robot.port
            )
            if item.id == robot.id or same_endpoint:
                updated.append(robot)
                replaced = True
            else:
                updated.append(item)
        if not replaced:
            updated.append(robot)
        self.save(updated)
        return robot

    def remove(self, robot_id: str) -> bool:
        robots = self.load()
        updated = [item for item in robots if item.id != robot_id]
        if len(updated) == len(robots):
            return False
        self.save(updated)
        return True
