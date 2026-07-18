from __future__ import annotations

import argparse
import webbrowser
from functools import partial
from http.server import ThreadingHTTPServer
from pathlib import Path

from .core.config import DEFAULT_CONFIG_PATH, OperatorAppConfig
from .core.state import OperatorAppState
from .web.handler import OperatorRequestHandler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve operator app for managing robots by IP.")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, type=Path)
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", default=None, type=int)
    parser.add_argument("--registry", default=None, type=Path)
    parser.add_argument("--probe-timeout", default=None, type=float)
    parser.add_argument("--static-dir", default=None, type=Path)
    parser.add_argument("--fleet-params", default=None, type=Path)
    parser.add_argument("--fleet-map-dir", "--map-dir", dest="fleet_map_dir", default=None, type=Path)
    parser.add_argument("--start", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--goal", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--output", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--open", dest="open_browser", action="store_true", default=None)
    parser.add_argument("--no-open", dest="open_browser", action="store_false")
    return parser.parse_args()


def config_from_args(args: argparse.Namespace) -> OperatorAppConfig:
    return OperatorAppConfig.load(args.config).with_overrides(
        host=args.host,
        port=args.port,
        registry_path=args.registry,
        probe_timeout=args.probe_timeout,
        static_dir=args.static_dir,
        open_browser=args.open_browser,
        fleet_params_path=args.fleet_params,
        fleet_map_dir=args.fleet_map_dir,
    )


def browser_url(host: str, port: int) -> str:
    if host in {"0.0.0.0", "::"}:
        return f"http://127.0.0.1:{port}/"
    return f"http://{host}:{port}/"


def main() -> None:
    args = parse_args()
    config = config_from_args(args)
    state = OperatorAppState(
        registry_path=config.registry_path,
        probe_timeout=config.probe_timeout,
        fleet_params_path=config.fleet_params_path,
        fleet_map_dir=config.fleet_map_dir,
    )
    OperatorRequestHandler.app_state = state
    handler = partial(OperatorRequestHandler, directory=str(config.static_dir.resolve()))
    server = ThreadingHTTPServer((config.host, config.port), handler)
    server.daemon_threads = True
    url = browser_url(config.host, config.port)
    print(f"Serving operator app: {url}")
    print(f"Config path: {args.config.expanduser()}")
    print(f"Registry path: {config.registry_path.expanduser()}")
    print(f"Fleet params path: {state.fleet_params_path}")
    print(f"Fleet map dir: {state.fleet_manager.map_dir}")
    if config.open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.close()
        server.server_close()


if __name__ == "__main__":
    main()
