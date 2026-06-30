from __future__ import annotations

import json
import os
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover - optional runtime dependency fallback
    yaml = None

from .models import KnownRobot


def default_operator_data_root() -> Path:
    override = os.environ.get("WAREHOUSE_OPERATOR_DATA", "").strip()
    if override:
        return Path(override).expanduser()
    return Path(__file__).resolve().parent / "operator_data"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class OperatorWorkspace:
    def __init__(self, root: Path | None = None) -> None:
        self.root = (root or default_operator_data_root()).expanduser().resolve()

    def robot_dir(self, robot: KnownRobot | str) -> Path:
        if isinstance(robot, KnownRobot):
            identity_name = robot.last_identity.get("robotId") if isinstance(robot.last_identity, dict) else ""
            name = robot.name or identity_name or robot.id
            return self.root / self._safe_name(name or robot.id)
        return self.root / self._safe_name(robot)

    def maps_dir(self, robot: KnownRobot | str) -> Path:
        return self.robot_dir(robot) / "maps"

    def params_dir(self, robot: KnownRobot | str) -> Path:
        return self.robot_dir(robot) / "params"

    def robot_model_dir(self, robot: KnownRobot | str) -> Path:
        return self.robot_dir(robot) / "robot_model"

    def ensure_robot_workspace(self, robot: KnownRobot, *, legacy_maps_dir: Path | None = None) -> dict[str, Any]:
        workspace_dir = self.robot_dir(robot)
        maps_dir = self.maps_dir(robot)
        params_dir = self.params_dir(robot)
        robot_model_dir = self.robot_model_dir(robot)
        for directory in (workspace_dir, maps_dir, params_dir, robot_model_dir):
            directory.mkdir(parents=True, exist_ok=True)
        if legacy_maps_dir and legacy_maps_dir.is_dir() and not any(maps_dir.iterdir()):
            self._copy_legacy_maps(legacy_maps_dir, maps_dir)
        meta = {
            "robotId": robot.id,
            "name": robot.name,
            "host": robot.host,
            "port": robot.port,
            "type": robot.type,
            "updatedAt": utc_now(),
        }
        self._write_json(workspace_dir / "meta.json", meta)
        return self.workspace_payload(robot)

    def workspace_payload(self, robot: KnownRobot) -> dict[str, Any]:
        workspace_dir = self.robot_dir(robot)
        return {
            "root": str(workspace_dir),
            "maps": str(self.maps_dir(robot)),
            "params": str(self.params_dir(robot)),
            "robotModel": str(self.robot_model_dir(robot)),
        }

    def save_map_index(self, robot: KnownRobot, payload: dict[str, Any]) -> None:
        self.ensure_robot_workspace(robot)
        self._write_json(self.maps_dir(robot) / "maps_index.json", payload)

    def save_active_map_meta(self, robot: KnownRobot, payload: dict[str, Any]) -> None:
        self.ensure_robot_workspace(robot)
        self._write_json(self.maps_dir(robot) / "active_map.json", payload)

    def save_params(self, robot: KnownRobot, params: dict[str, Any], *, source: str = "robot") -> dict[str, Any]:
        self.ensure_robot_workspace(robot)
        params_dir = self.params_dir(robot)
        payload = {
            "robotId": robot.id,
            "source": source,
            "savedAt": utc_now(),
            "params": params,
        }
        self._write_json(params_dir / "params.json", payload)
        if yaml is not None:
            (params_dir / "params.yaml").write_text(
                yaml.safe_dump(params, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
        else:
            (params_dir / "params.yaml").write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")
        robot_model = params.get("robot_model") if isinstance(params, dict) else None
        if isinstance(robot_model, dict):
            self.save_robot_model(robot, robot_model, source=source)
        return {
            "path": str(params_dir / "params.yaml"),
            "jsonPath": str(params_dir / "params.json"),
            "savedAt": payload["savedAt"],
        }

    def load_params(self, robot: KnownRobot) -> dict[str, Any] | None:
        params_json = self.params_dir(robot) / "params.json"
        if params_json.exists():
            payload = self._read_json(params_json)
            params = payload.get("params") if isinstance(payload, dict) else None
            if isinstance(params, dict):
                return params
        params_yaml = self.params_dir(robot) / "params.yaml"
        if params_yaml.exists() and yaml is not None:
            try:
                payload = yaml.safe_load(params_yaml.read_text(encoding="utf-8"))
            except Exception:
                payload = None
            if isinstance(payload, dict):
                return payload
        return None

    def save_robot_model(self, robot: KnownRobot, robot_model: dict[str, Any], *, source: str = "robot") -> dict[str, Any]:
        self.ensure_robot_workspace(robot)
        robot_model_dir = self.robot_model_dir(robot)
        payload = {
            "robotId": robot.id,
            "source": source,
            "savedAt": utc_now(),
            "robot_model": robot_model,
        }
        self._write_json(robot_model_dir / "robot_model.json", payload)
        return {
            "path": str(robot_model_dir / "robot_model.json"),
            "savedAt": payload["savedAt"],
        }

    def load_robot_model(self, robot: KnownRobot) -> dict[str, Any] | None:
        payload = self._read_json(self.robot_model_dir(robot) / "robot_model.json")
        robot_model = payload.get("robot_model") if isinstance(payload, dict) else None
        return robot_model if isinstance(robot_model, dict) else None

    def _copy_legacy_maps(self, legacy_maps_dir: Path, maps_dir: Path) -> None:
        maps_dir.mkdir(parents=True, exist_ok=True)
        for item in legacy_maps_dir.iterdir():
            target = maps_dir / item.name
            if item.is_dir():
                if target.exists():
                    continue
                shutil.copytree(item, target)
            elif item.is_file() and not target.exists():
                shutil.copy2(item, target)

    def _read_json(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return payload if isinstance(payload, dict) else {}

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _safe_name(self, value: str) -> str:
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip()).strip("._") or "unnamed"
