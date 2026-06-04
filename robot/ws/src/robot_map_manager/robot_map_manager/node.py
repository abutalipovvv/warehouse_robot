from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic, sleep

import rclpy
from nav2_msgs.srv import LoadMap as Nav2LoadMap
from rclpy.node import Node

from robot_msgs.srv import GetRobotMapState, LoadRobotMap
from robot_planner import WarehouseMapLoader


def find_ros_map_yaml(map_dir: Path) -> Path:
    directory = Path(map_dir).resolve()
    candidates = sorted(
        path
        for path in directory.glob("*.yaml")
        if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
    )
    if not candidates:
        raise FileNotFoundError(f"No ROS map yaml found in {directory}")
    return candidates[0]


class RobotMapManagerNode(Node):
    def __init__(
        self,
        *,
        map_dir: Path,
        maps_root: Path,
        state_file: Path,
        map_server_load_service: str,
        route_load_map_service: str,
        status_load_map_service: str,
        manager_load_service: str,
        manager_state_service: str,
    ) -> None:
        super().__init__("robot_map_manager")
        self.maps_root = Path(maps_root).resolve()
        self.state_file = Path(state_file).resolve()
        self._active_map_dir = Path(map_dir).resolve()
        self._active_map_id = self._map_id_for_dir(self._active_map_dir)
        self._map_server_client = self.create_client(Nav2LoadMap, map_server_load_service)
        self._route_load_client = self.create_client(LoadRobotMap, route_load_map_service)
        self._status_load_client = self.create_client(LoadRobotMap, status_load_map_service)
        self.create_service(LoadRobotMap, manager_load_service, self._handle_load_map)
        self.create_service(GetRobotMapState, manager_state_service, self._handle_get_state)
        self._persist_state()

    def _handle_get_state(self, _request, response):
        response.ok = True
        response.error = ""
        response.map_name = self._active_map_name()
        response.map_dir = str(self._active_map_dir)
        response.map_id = self._active_map_id
        return response

    def _handle_load_map(self, request, response):
        try:
            target = self._resolve_target_map(
                map_name=str(request.map_name or "").strip(),
                map_dir=str(request.map_dir or "").strip(),
            )
            map_yaml = find_ros_map_yaml(target)
            nav2_request = Nav2LoadMap.Request()
            nav2_request.map_url = str(map_yaml)
            nav2_response = self._call_service(self._map_server_client, nav2_request, "map_server/load_map")
            if int(nav2_response.result) != int(Nav2LoadMap.Response.RESULT_SUCCESS):
                raise ValueError(f"map_server rejected map load with result={int(nav2_response.result)}")

            internal_request = LoadRobotMap.Request()
            internal_request.map_name = target.stem.replace(".smap", "")
            internal_request.map_dir = str(target)
            self._require_ok(self._call_service(self._route_load_client, internal_request, "route/load_map"), "route/load_map")
            self._require_ok(self._call_service(self._status_load_client, internal_request, "status/load_map"), "status/load_map")

            self._active_map_dir = target
            self._active_map_id = self._map_id_for_dir(target)
            self._persist_state()

            response.ok = True
            response.error = ""
            response.map_name = self._active_map_name()
            response.map_dir = str(self._active_map_dir)
            response.map_id = self._active_map_id
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.map_name = ""
            response.map_dir = ""
            response.map_id = ""
        return response

    def _resolve_target_map(self, *, map_name: str, map_dir: str) -> Path:
        if map_dir:
            candidate = Path(map_dir).resolve()
        else:
            safe_name = Path(map_name).name
            if not safe_name:
                raise ValueError("map_name is required")
            if not safe_name.endswith(".smap"):
                safe_name = f"{safe_name}.smap"
            candidate = (self.maps_root / safe_name).resolve()
        if self.maps_root not in candidate.parents:
            raise ValueError("map must stay inside maps_root")
        if not candidate.is_dir():
            raise ValueError(f"map not found: {candidate.name}")
        return candidate

    def _map_id_for_dir(self, map_dir: Path) -> str:
        loaded_map = WarehouseMapLoader(map_dir).load()
        return loaded_map.map_metadata.map_name

    def _active_map_name(self) -> str:
        return self._active_map_dir.stem.replace(".smap", "")

    def _persist_state(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "mapName": self._active_map_name(),
            "mapDir": str(self._active_map_dir),
            "mapId": self._active_map_id,
        }
        self.state_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _call_service(self, client, request, service_label: str):
        if not client.wait_for_service(timeout_sec=2.0):
            raise ValueError(f"{service_label} is not available")
        future = client.call_async(request)
        deadline = monotonic() + 5.0
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            raise ValueError(f"{service_label} timed out")
        if future.exception() is not None:
            raise ValueError(f"{service_label} failed: {future.exception()}")
        response = future.result()
        if response is None:
            raise ValueError(f"{service_label} returned no response")
        return response

    def _require_ok(self, response, service_label: str) -> None:
        if not bool(response.ok):
            raise ValueError(str(response.error or f"{service_label} failed"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run robot map manager node.")
    parser.add_argument("--map-dir", required=True, type=Path)
    parser.add_argument("--maps-root", required=True, type=Path)
    parser.add_argument("--state-file", required=True, type=Path)
    parser.add_argument("--map-server-load-service", default="/map_server/load_map")
    parser.add_argument("--route-load-map-service", default="/route/load_map")
    parser.add_argument("--status-load-map-service", default="/status/load_map")
    parser.add_argument("--manager-load-service", default="/robot/maps/load")
    parser.add_argument("--manager-state-service", default="/robot/maps/state")
    args, _unknown = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init(args=None)
    node = RobotMapManagerNode(
        map_dir=args.map_dir,
        maps_root=args.maps_root,
        state_file=args.state_file,
        map_server_load_service=args.map_server_load_service,
        route_load_map_service=args.route_load_map_service,
        status_load_map_service=args.status_load_map_service,
        manager_load_service=args.manager_load_service,
        manager_state_service=args.manager_state_service,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
