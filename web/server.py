from __future__ import annotations

import argparse
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
import webbrowser

from route_core import DEFAULT_PARAMS_PATH, LmRoutePlanner, load_route_params, save_editable_map, save_route_params
from route_core import WarehouseMapLoader

from .application import RouteDemoApplication, RouteDemoOptions
from fleet_manager import FleetManager

DEFAULT_WEB_MAP_DIR = Path(__file__).resolve().parents[1] / "map_data" / "maps_out" / "22.05.26_smap.smap"


class WarehouseWebRequestHandler(SimpleHTTPRequestHandler):
    params_path: Path = DEFAULT_PARAMS_PATH
    map_dir: Path = DEFAULT_WEB_MAP_DIR
    maps_root: Path = DEFAULT_WEB_MAP_DIR.parent
    output_dir: Path | None = None
    fleet_manager: FleetManager | None = None

    def end_headers(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/index.html", "/demo-data.js", "/app.js", "/styles.css"}:
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
        super().end_headers()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/params":
            self._send_json(load_route_params(self.params_path, create=True))
            return
        if path == "/api/fleet/state":
            self._handle_fleet_state()
            return
        if path == "/api/maps/list":
            self._handle_list_maps()
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
        if path == "/api/map/save":
            self._handle_save_map()
            return
        if path == "/api/maps/load":
            self._handle_load_map()
            return
        if path == "/api/maps/push":
            self._handle_push_map()
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

    def _handle_save_map(self) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return

        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        try:
            output_name = str(payload.get("outputName") or "").strip()
            loaded_map = save_editable_map(
                self.map_dir,
                payload,
                output_name=output_name,
                overwrite_output=bool(payload.get("overwriteOutput", False)),
            )
            self._regenerate_output_for_map(loaded_map.map_dir)
            self._set_active_map(loaded_map)
            planner = LmRoutePlanner(loaded_map.landmarks, loaded_map.edges, params=load_route_params(self.params_path, create=True))
            self._send_json(
                {
                    "ok": True,
                    "mapDir": str(loaded_map.map_dir),
                    "mapName": loaded_map.map_dir.stem.replace(".smap", ""),
                    "active": True,
                    "routes": planner.build_route_catalog(),
                    "savedAs": bool(output_name),
                }
            )
        except ValueError as exc:
            self.send_error(400, str(exc))
        except Exception as exc:
            self.send_error(500, str(exc))

    def _handle_list_maps(self) -> None:
        maps = []
        for item in sorted(self.maps_root.glob("*.smap")):
            if not item.is_dir():
                continue
            if not (item / "LMs.yaml").exists():
                continue
            maps.append(
                {
                    "name": item.stem,
                    "folder": item.name,
                    "active": item.resolve() == self.map_dir.resolve(),
                }
            )
        self._send_json(
            {
                "ok": True,
                "active": self.map_dir.stem,
                "maps": maps,
            }
        )

    def _handle_load_map(self) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            self.send_error(400, "mapName is required")
            return

        try:
            loaded_map = self._activate_map_by_name(map_name)
            self._send_json(
                {
                    "ok": True,
                    "mapName": loaded_map.map_dir.stem.replace(".smap", ""),
                    "mapDir": str(loaded_map.map_dir),
                }
            )
        except ValueError as exc:
            self.send_error(400, str(exc))
        except Exception as exc:
            self.send_error(500, str(exc))

    def _handle_push_map(self) -> None:
        self._send_json(
            {
                "ok": True,
                "mapName": self.map_dir.stem.replace(".smap", ""),
                "mapDir": str(self.map_dir),
                "simulated": True,
            }
        )

    def _activate_map_by_name(self, map_name: str):
        safe_name = Path(map_name).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        if self.maps_root.resolve() not in target.parents:
            raise ValueError("map must stay inside maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        if self.output_dir is None:
            raise ValueError("web output directory is not ready")

        self._regenerate_output_for_map(target)
        loaded_map = WarehouseMapLoader(target).load()
        self._set_active_map(loaded_map)
        return loaded_map

    def _regenerate_output_for_map(self, map_dir: Path) -> None:
        RouteDemoApplication().run(
            RouteDemoOptions(
                map_dir=map_dir,
                output=self.output_dir,
                params=self.params_path,
            )
        )

    def _set_active_map(self, loaded_map) -> None:
        handler_type = type(self)
        handler_type.map_dir = loaded_map.map_dir
        handler_type.fleet_manager = FleetManager(
            loaded_map.landmarks,
            loaded_map.edges,
            params=load_route_params(self.params_path, create=True),
            map_dir=loaded_map.map_dir,
            map_metadata=loaded_map.map_metadata,
        )

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
    WarehouseWebRequestHandler.map_dir = map_dir.resolve()
    WarehouseWebRequestHandler.maps_root = map_dir.resolve().parent
    WarehouseWebRequestHandler.output_dir = output_dir
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
