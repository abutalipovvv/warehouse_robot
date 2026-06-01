from __future__ import annotations

import argparse
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
import webbrowser

from route_core import DEFAULT_PARAMS_PATH, load_route_params, save_route_params
from route_core import WarehouseMapLoader

from .application import RouteDemoApplication, RouteDemoOptions
from fleet_manager import FleetManager

DEFAULT_WEB_MAP_DIR = Path(__file__).resolve().parents[1] / "map_data" / "maps_out" / "22.05.26_smap.smap"


class WarehouseWebRequestHandler(SimpleHTTPRequestHandler):
    params_path: Path = DEFAULT_PARAMS_PATH
    fleet_manager: FleetManager | None = None

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/params":
            self._send_json(load_route_params(self.params_path, create=True))
            return
        if path == "/api/fleet/state":
            self._handle_fleet_state()
            return
        super().do_GET()

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/params":
            self._handle_save_params()
            return
        if path == "/api/fleet/plan":
            self._handle_fleet_plan()
            return
        if path == "/api/fleet/orders":
            self._handle_fleet_plan()
            return
        if path == "/api/fleet/tick":
            self._handle_fleet_tick()
            return
        if path == "/api/fleet/world":
            self._handle_fleet_world()
            return
        if path == "/api/fleet/robots":
            self._handle_add_robot()
            return
        if path == "/api/fleet/robots/remove":
            self._handle_remove_robot()
            return
        if path == "/api/fleet/robots/update":
            self._handle_update_robot()
            return
        if path == "/api/fleet/robots/stop":
            self._handle_stop_robot()
            return
        if path == "/api/fleet/robots/reset":
            self._handle_reset_robot()
            return

        self.send_error(404, "Not found")

    def _handle_save_params(self) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return

        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        saved_path = save_route_params(payload, self.params_path)
        self._send_json({"ok": True, "path": str(saved_path)})

    def _handle_fleet_plan(self) -> None:
        if self.fleet_manager is None:
            self.send_error(503, "Fleet manager is not ready")
            return

        payload = self._read_json_payload()
        if payload is None:
            return

        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        try:
            self._send_json(self.fleet_manager.plan(payload))
        except ValueError as exc:
            self.send_error(400, str(exc))
        except Exception as exc:
            self.send_error(500, str(exc))

    def _handle_fleet_state(self) -> None:
        if self.fleet_manager is None:
            self.send_error(503, "Fleet manager is not ready")
            return
        self._send_json(self.fleet_manager.state())

    def _handle_fleet_tick(self) -> None:
        if self.fleet_manager is None:
            self.send_error(503, "Fleet manager is not ready")
            return

        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        self._send_json(self.fleet_manager.tick(payload))

    def _handle_fleet_world(self) -> None:
        if self.fleet_manager is None:
            self.send_error(503, "Fleet manager is not ready")
            return

        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        self._send_json(self.fleet_manager.update_world(payload))

    def _handle_add_robot(self) -> None:
        self._handle_fleet_robot_command("add_robot")

    def _handle_remove_robot(self) -> None:
        self._handle_fleet_robot_command("remove_robot")

    def _handle_update_robot(self) -> None:
        self._handle_fleet_robot_command("update_robot")

    def _handle_stop_robot(self) -> None:
        self._handle_fleet_robot_command("stop_robot")

    def _handle_reset_robot(self) -> None:
        self._handle_fleet_robot_command("reset_robot")

    def _handle_fleet_robot_command(self, method_name: str) -> None:
        if self.fleet_manager is None:
            self.send_error(503, "Fleet manager is not ready")
            return

        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        try:
            method = getattr(self.fleet_manager, method_name)
            self._send_json(method(payload))
        except ValueError as exc:
            self.send_error(400, str(exc))
        except Exception as exc:
            self.send_error(500, str(exc))

    def _read_json_payload(self) -> object | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        try:
            return json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
            return None

    def _send_json(self, payload: object) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve the warehouse web simulator.")
    parser.add_argument("--map-dir", default=DEFAULT_WEB_MAP_DIR, type=Path)
    parser.add_argument("--start", default=None)
    parser.add_argument("--goal", default=None)
    parser.add_argument("--output", default=None, type=Path)
    parser.add_argument("--params", default=DEFAULT_PARAMS_PATH, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8090, type=int)
    parser.add_argument("--open", action="store_true")
    return parser.parse_args()


def resolve_map_dir(map_dir: Path) -> Path:
    if map_dir.exists():
        return map_dir
    if map_dir.is_absolute():
        return map_dir

    project_root = Path(__file__).resolve().parents[1]
    relocated = project_root / "map_data" / map_dir
    if relocated.exists():
        return relocated
    return map_dir


def main() -> None:
    args = parse_args()
    map_dir = resolve_map_dir(args.map_dir)
    index_path = RouteDemoApplication().run(
        RouteDemoOptions(
            map_dir=map_dir,
            start=args.start,
            goal=args.goal,
            output=args.output,
            params=args.params,
        )
    )
    output_dir = index_path.parent.resolve()

    handler = partial(WarehouseWebRequestHandler, directory=str(output_dir))
    WarehouseWebRequestHandler.params_path = args.params.resolve()
    loaded_map = WarehouseMapLoader(map_dir).load()
    WarehouseWebRequestHandler.fleet_manager = FleetManager(
        loaded_map.landmarks,
        loaded_map.edges,
        params=load_route_params(args.params, create=True),
        map_dir=loaded_map.map_dir,
        map_metadata=loaded_map.map_metadata,
    )

    server = ThreadingHTTPServer((args.host, args.port), handler)
    url = f"http://{args.host}:{args.port}/"
    print(f"Serving warehouse web simulator: {url}")
    print(f"Saving params to: {WarehouseWebRequestHandler.params_path}")
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
