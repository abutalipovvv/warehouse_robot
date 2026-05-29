from __future__ import annotations

import argparse
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import webbrowser

from route_core import DEFAULT_PARAMS_PATH, load_route_params, save_route_params

from .application import RouteDemoApplication, RouteDemoOptions


class WarehouseWebRequestHandler(SimpleHTTPRequestHandler):
    params_path: Path = DEFAULT_PARAMS_PATH

    def do_GET(self) -> None:
        if self.path == "/api/params":
            self._send_json(load_route_params(self.params_path, create=True))
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path != "/api/params":
            self.send_error(404, "Not found")
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        try:
            payload = json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
            return

        if not isinstance(payload, dict):
            self.send_error(400, "Expected object")
            return

        saved_path = save_route_params(payload, self.params_path)
        self._send_json({"ok": True, "path": str(saved_path)})

    def _send_json(self, payload: object) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve the warehouse web simulator.")
    parser.add_argument("--map-dir", required=True, type=Path)
    parser.add_argument("--start", default=None)
    parser.add_argument("--goal", default=None)
    parser.add_argument("--output", default=None, type=Path)
    parser.add_argument("--params", default=DEFAULT_PARAMS_PATH, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--open", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    index_path = RouteDemoApplication().run(
        RouteDemoOptions(
            map_dir=args.map_dir,
            start=args.start,
            goal=args.goal,
            output=args.output,
            params=args.params,
        )
    )
    output_dir = index_path.parent.resolve()

    handler = partial(WarehouseWebRequestHandler, directory=str(output_dir))
    WarehouseWebRequestHandler.params_path = args.params.resolve()

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
