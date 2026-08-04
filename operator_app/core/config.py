from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

try:
    import yaml
except ModuleNotFoundError:  # pragma: no cover - PyYAML is available in the normal app environment.
    yaml = None  # type: ignore[assignment]

from .fleet_context import DEFAULT_FLEET_MAP_DIR
from .registry import default_registry_path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parent

DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "config" / "config.yaml"
DEFAULT_STATIC_DIR = PACKAGE_ROOT / "web" / "static"
DEFAULT_FLEET_PARAMS_PATH = PROJECT_ROOT / "fleet_manager" / "config" / "params.yaml"

GRPC_ROBOT_TYPES = {"grpc", "aivison_grpc", "real_grpc"}
WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
DEFAULT_FLEET_WS_INTERVAL_MS = 180
MIN_FLEET_WS_INTERVAL_MS = 50
MAX_FLEET_WS_INTERVAL_MS = 1000
APP_ROUTES = {"/", "/home", "/robot", "/params", "/robot_model", "/map_editor"}


@dataclass(frozen=True, slots=True)
class OperatorAppConfig:
    host: str = "0.0.0.0"
    port: int = 8780
    registry_path: Path = field(default_factory=default_registry_path)
    probe_timeout: float = 1.0
    static_dir: Path = DEFAULT_STATIC_DIR
    open_browser: bool = False
    fleet_params_path: Path = DEFAULT_FLEET_PARAMS_PATH
    fleet_map_dir: Path = DEFAULT_FLEET_MAP_DIR

    @classmethod
    def load(cls, path: Path | None = None) -> "OperatorAppConfig":
        config_path = (path or DEFAULT_CONFIG_PATH).expanduser()
        config = cls()
        if not config_path.exists():
            return config
        payload = _load_yaml(config_path)
        base_dir = config_path.parent
        server = _dict(payload.get("server"))
        paths = _dict(payload.get("paths"))
        fleet = _dict(payload.get("fleet"))
        return cls(
            host=str(server.get("host") or config.host),
            port=_int(server.get("port"), config.port),
            registry_path=_path(paths.get("registry"), base_dir, config.registry_path),
            probe_timeout=_float(server.get("probe_timeout"), config.probe_timeout),
            static_dir=_path(paths.get("static_dir"), base_dir, config.static_dir),
            open_browser=_bool(server.get("open_browser"), config.open_browser),
            fleet_params_path=_path(fleet.get("params_path"), base_dir, config.fleet_params_path),
            fleet_map_dir=_path(fleet.get("map_dir"), base_dir, config.fleet_map_dir),
        )

    def with_overrides(
        self,
        *,
        host: str | None = None,
        port: int | None = None,
        registry_path: Path | None = None,
        probe_timeout: float | None = None,
        static_dir: Path | None = None,
        open_browser: bool | None = None,
        fleet_params_path: Path | None = None,
        fleet_map_dir: Path | None = None,
    ) -> "OperatorAppConfig":
        return replace(
            self,
            host=self.host if host is None else str(host),
            port=self.port if port is None else int(port),
            registry_path=self.registry_path if registry_path is None else registry_path.expanduser(),
            probe_timeout=self.probe_timeout if probe_timeout is None else float(probe_timeout),
            static_dir=self.static_dir if static_dir is None else static_dir.expanduser(),
            open_browser=self.open_browser if open_browser is None else bool(open_browser),
            fleet_params_path=self.fleet_params_path if fleet_params_path is None else fleet_params_path.expanduser(),
            fleet_map_dir=self.fleet_map_dir if fleet_map_dir is None else fleet_map_dir.expanduser(),
        )


def _load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read operator_app/config/config.yaml")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _path(value: Any, base_dir: Path, default: Path) -> Path:
    raw = str(value or "").strip()
    if not raw:
        return default.expanduser()
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path


def _int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}
