from __future__ import annotations

import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import RLock
from time import monotonic, time
from typing import Any


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _lerp_angle(start: float, goal: float, alpha: float) -> float:
    return _normalize_angle(start + _normalize_angle(goal - start) * alpha)


class StubRobotRuntime:
    def __init__(self, robot_id: str, map_name: str, spawn_lm: str, route_speed: float) -> None:
        self.robot_id = robot_id
        self.map_name = map_name
        self.route_speed = max(0.02, float(route_speed))
        self._lock = RLock()
        self._state = "IDLE"
        self._message = "Stub robot ready."
        self._current_lm = str(spawn_lm or "")
        self._target_lm = ""
        self._pose: dict[str, float] | None = None
        self._velocity = {"linear": 0.0, "angular": 0.0}
        self._active_route: dict[str, Any] | None = None
        self._started_at: float | None = None
        self._route_final_time = 0.0
        self._route_progress = 0.0
        self._events: list[dict[str, Any]] = []
        self.add_event("info", "stub robot initialized")

    def add_event(self, level: str, message: str) -> None:
        self._events.append({"stamp": time(), "level": level, "message": message})
        self._events = self._events[-120:]

    def identity_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "robotId": self.robot_id,
            "mapId": self.map_name,
            "api": "robot_http_api_stub",
            "version": 1,
        }

    def status_payload(self) -> dict[str, Any]:
        with self._lock:
            self._advance_locked()
            return {
                "ok": True,
                "robot": {
                    "robotId": self.robot_id,
                    "mapId": self.map_name,
                    "connected": True,
                    "localizationOk": True,
                    "localizationAgeSec": 0.0,
                    "state": self._state,
                    "message": self._message,
                    "targetLm": self._target_lm,
                    "nearestLm": self._current_lm,
                    "currentLm": self._current_lm,
                    "currentEdgeId": self._current_edge_id_locked(),
                    "routeId": str((self._active_route or {}).get("routeId") or ""),
                    "routeProgress": self._route_progress,
                    "pose": dict(self._pose) if self._pose is not None else None,
                    "velocity": dict(self._velocity),
                },
                "route": dict(self._active_route) if self._active_route is not None else None,
                "events": list(self._events[-80:]),
            }

    def execute_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = payload.get("route")
        if not isinstance(route, dict):
            route = payload
        goal_lm = str(route.get("goalLm") or route.get("targetLm") or "").strip()
        if not goal_lm:
            raise ValueError("route.goalLm is required")
        normalized = self._normalize_route(route)
        with self._lock:
            self._active_route = normalized
            self._started_at = monotonic()
            self._route_final_time = self._final_time(normalized)
            self._route_progress = 0.0
            self._target_lm = goal_lm
            start_lm = str(normalized.get("startLm") or "").strip()
            if start_lm:
                self._current_lm = start_lm
            trajectory = normalized.get("trajectory", [])
            if isinstance(trajectory, list) and trajectory:
                self._pose = self._pose_from_sample(trajectory[0])
            self._state = "EXECUTING_ROUTE"
            self._message = f"Executing route to {goal_lm}."
            self._velocity = {"linear": self.route_speed, "angular": 0.0}
            self.add_event("info", f"route accepted: {normalized.get('routeId')} -> {goal_lm}")
            self._advance_locked()
            return {"ok": True, "route": dict(normalized), "status": self.status_payload()}

    def cancel_route_payload(self, message: str = "Route canceled.") -> dict[str, Any]:
        with self._lock:
            self._active_route = None
            self._started_at = None
            self._target_lm = ""
            self._route_progress = 0.0
            self._state = "IDLE"
            self._message = message
            self._velocity = {"linear": 0.0, "angular": 0.0}
            self.add_event("warn", message)
            return self.status_payload()

    def stop_payload(self) -> dict[str, Any]:
        with self._lock:
            self._active_route = None
            self._started_at = None
            self._target_lm = ""
            self._state = "STOPPED"
            self._message = "Robot stopped."
            self._velocity = {"linear": 0.0, "angular": 0.0}
            self.add_event("warn", "robot stopped")
            return self.status_payload()

    def teleop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._active_route = None
            self._started_at = None
            self._target_lm = ""
            self._state = "MANUAL"
            self._message = "Manual control."
            self._velocity = {
                "linear": float(payload.get("linear", 0.0) or 0.0),
                "angular": float(payload.get("angular", 0.0) or 0.0),
            }
            return self.status_payload()

    def teleop_stop_payload(self) -> dict[str, Any]:
        with self._lock:
            self._state = "IDLE"
            self._message = "Manual control released."
            self._velocity = {"linear": 0.0, "angular": 0.0}
            return self.status_payload()

    def maps_active_payload(self) -> dict[str, Any]:
        return {"ok": True, "mapName": self.map_name, "mapDir": "", "mapId": self.map_name}

    def maps_list_payload(self) -> dict[str, Any]:
        return {"ok": True, "activeMapName": self.map_name, "maps": [{"name": self.map_name, "active": True}]}

    def params_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "params": {
                "navigation": {"route_speed": self.route_speed},
                "fleet": {"remote_stub": True},
            },
        }

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        navigation = payload.get("navigation") if isinstance(payload, dict) else None
        if isinstance(navigation, dict) and navigation.get("route_speed") is not None:
            self.route_speed = max(0.02, float(navigation.get("route_speed") or self.route_speed))
        return self.params_payload()

    def _normalize_route(self, route: dict[str, Any]) -> dict[str, Any]:
        trajectory_payload = route.get("trajectory", [])
        trajectory = [
            self._clean_sample(item)
            for item in trajectory_payload
            if isinstance(item, dict)
        ] if isinstance(trajectory_payload, list) else []
        self._ensure_sample_times(trajectory)
        route_id = str(route.get("routeId") or route.get("route_id") or f"stub-route-{int(time() * 1000)}")
        return {
            **route,
            "routeId": route_id,
            "startLm": str(route.get("startLm") or route.get("start_lm") or ""),
            "goalLm": str(route.get("goalLm") or route.get("goal_lm") or route.get("targetLm") or ""),
            "nodes": [str(item) for item in route.get("nodes", []) if str(item)] if isinstance(route.get("nodes"), list) else [],
            "trajectory": trajectory,
        }

    def _clean_sample(self, item: dict[str, Any]) -> dict[str, Any]:
        sample = {
            "x": float(item.get("x", 0.0) or 0.0),
            "y": float(item.get("y", 0.0) or 0.0),
            "yaw": float(item.get("yaw", 0.0) or 0.0),
            "edgeId": str(item.get("edgeId") or item.get("edge_id") or ""),
            "motionDirection": str(item.get("motionDirection") or item.get("motion_direction") or "not_specified"),
        }
        if item.get("t") is not None:
            sample["t"] = float(item.get("t", 0.0) or 0.0)
        return sample

    def _ensure_sample_times(self, trajectory: list[dict[str, Any]]) -> None:
        if not trajectory:
            return
        if all("t" in sample for sample in trajectory):
            first_t = float(trajectory[0].get("t", 0.0) or 0.0)
            if first_t != 0.0:
                for sample in trajectory:
                    sample["t"] = max(0.0, float(sample.get("t", 0.0) or 0.0) - first_t)
            return
        trajectory[0]["t"] = 0.0
        for index in range(1, len(trajectory)):
            previous = trajectory[index - 1]
            sample = trajectory[index]
            distance = math.hypot(sample["x"] - previous["x"], sample["y"] - previous["y"])
            sample["t"] = float(previous.get("t", 0.0) or 0.0) + distance / self.route_speed

    def _advance_locked(self) -> None:
        if self._active_route is None or self._started_at is None or self._state != "EXECUTING_ROUTE":
            return
        trajectory = self._active_route.get("trajectory", [])
        if not isinstance(trajectory, list) or not trajectory:
            self._finish_route_locked()
            return
        elapsed = max(0.0, monotonic() - self._started_at)
        final_time = self._final_time(self._active_route)
        if final_time <= 0.0 or elapsed >= final_time:
            self._pose = self._pose_from_sample(trajectory[-1])
            self._finish_route_locked()
            return
        self._pose = self._interpolate_pose(trajectory, elapsed)
        self._route_progress = max(0.0, min(1.0, elapsed / final_time))
        self._velocity = {"linear": self.route_speed, "angular": 0.0}

    def _finish_route_locked(self) -> None:
        goal_lm = self._target_lm
        if goal_lm:
            self._current_lm = goal_lm
        self._state = "ARRIVED"
        self._message = f"Arrived at {goal_lm}." if goal_lm else "Arrived."
        self._target_lm = ""
        self._route_progress = 1.0
        self._velocity = {"linear": 0.0, "angular": 0.0}
        self.add_event("info", self._message)

    def _final_time(self, route: dict[str, Any]) -> float:
        trajectory = route.get("trajectory", [])
        if not isinstance(trajectory, list) or not trajectory:
            return 0.0
        return max(0.0, float(trajectory[-1].get("t", 0.0) or 0.0))

    def _current_edge_id_locked(self) -> str:
        route = self._active_route
        if route is None or self._started_at is None:
            return ""
        trajectory = route.get("trajectory", [])
        if not isinstance(trajectory, list) or not trajectory:
            return ""
        elapsed = max(0.0, monotonic() - self._started_at)
        current = trajectory[0]
        for sample in trajectory:
            if float(sample.get("t", 0.0) or 0.0) <= elapsed:
                current = sample
            else:
                break
        return str(current.get("edgeId") or "")

    def _interpolate_pose(self, trajectory: list[dict[str, Any]], elapsed: float) -> dict[str, float]:
        previous = trajectory[0]
        for sample in trajectory[1:]:
            start_t = float(previous.get("t", 0.0) or 0.0)
            end_t = float(sample.get("t", 0.0) or 0.0)
            if elapsed <= end_t:
                span = max(0.000001, end_t - start_t)
                alpha = max(0.0, min(1.0, (elapsed - start_t) / span))
                return {
                    "x": previous["x"] + (sample["x"] - previous["x"]) * alpha,
                    "y": previous["y"] + (sample["y"] - previous["y"]) * alpha,
                    "yaw": _lerp_angle(previous["yaw"], sample["yaw"], alpha),
                }
            previous = sample
        return self._pose_from_sample(trajectory[-1])

    def _pose_from_sample(self, sample: dict[str, Any]) -> dict[str, float]:
        return {
            "x": float(sample.get("x", 0.0) or 0.0),
            "y": float(sample.get("y", 0.0) or 0.0),
            "yaw": float(sample.get("yaw", 0.0) or 0.0),
        }


class StubRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    runtime: StubRobotRuntime | None = None

    def do_GET(self) -> None:
        try:
            path = self.path.split("?", 1)[0]
            if path == "/" or path == "/health":
                self._send_json({"ok": True, "service": "robot_http_api_stub"})
                return
            if path == "/api/robot/identity":
                self._send_json(self._runtime().identity_payload())
                return
            if path == "/api/robot/status":
                self._send_json(self._runtime().status_payload())
                return
            if path == "/api/maps/active":
                self._send_json(self._runtime().maps_active_payload())
                return
            if path == "/api/maps/list":
                self._send_json(self._runtime().maps_list_payload())
                return
            if path == "/api/params":
                self._send_json(self._runtime().params_payload())
                return
            self._send_error_json(404, "not found")
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover
            self._send_error_json(500, str(exc))

    def do_POST(self) -> None:
        try:
            path = self.path.split("?", 1)[0]
            if path == "/api/robot/route/execute":
                self._send_json(self._runtime().execute_route_payload(self._read_json_payload()))
                return
            if path == "/api/robot/route/cancel":
                self._send_json({"ok": True, "status": self._runtime().cancel_route_payload()})
                return
            if path == "/api/robot/stop":
                self._send_json({"ok": True, "status": self._runtime().stop_payload()})
                return
            if path == "/api/robot/teleop":
                self._send_json({"ok": True, "status": self._runtime().teleop_payload(self._read_json_payload())})
                return
            if path == "/api/robot/teleop/stop":
                self._send_json({"ok": True, "status": self._runtime().teleop_stop_payload()})
                return
            if path == "/api/params":
                self._send_json(self._runtime().save_params_payload(self._read_json_payload()))
                return
            self._send_error_json(404, "not found")
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover
            self._send_error_json(500, str(exc))

    def _runtime(self) -> StubRobotRuntime:
        if self.runtime is None:
            raise ValueError("stub runtime is not configured")
        return self.runtime

    def _read_json_payload(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
        if not raw:
            return {}
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("expected object payload")
        return payload

    def _send_json(self, payload: object, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json({"ok": False, "error": message}, status=status)

    def log_message(self, format: str, *args: Any) -> None:
        return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="No-ROS robot_http_api stub server.")
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--map-name", default="22.05.26_smap")
    parser.add_argument("--spawn-lm", default="LM101")
    parser.add_argument("--route-speed", type=float, default=0.45)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8791)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    runtime = StubRobotRuntime(
        robot_id=args.robot_id,
        map_name=args.map_name,
        spawn_lm=args.spawn_lm,
        route_speed=args.route_speed,
    )
    handler = type("ConfiguredStubRequestHandler", (StubRequestHandler,), {"runtime": runtime})
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"robot_http_api_stub {args.robot_id} listening on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
