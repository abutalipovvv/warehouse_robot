from __future__ import annotations

import argparse
import json
import shutil
import tempfile
from threading import RLock
from pathlib import Path
from time import monotonic, sleep

import rclpy
from nav2_msgs.srv import LoadMap as Nav2LoadMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_msgs.srv import (
    GetRobotMapBundle,
    GetRobotMapState,
    ListRobotMaps,
    LoadRobotMap,
    PutRobotMapBundle,
)
from robot_planner import (
    WarehouseMapLoader,
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    editable_map_signature,
    restore_editable_map_bundle,
)
from robot_planner.route_core.atomic_storage import atomic_write_text
from robot_planner.route_core.map_exchange import find_ros_map_yaml


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
        manager_list_service: str,
        manager_get_bundle_service: str,
        manager_put_bundle_service: str,
    ) -> None:
        super().__init__("robot_map_manager")
        self.maps_root = Path(maps_root).resolve()
        self.state_file = Path(state_file).resolve()
        self._active_map_dir = Path(map_dir).resolve()
        self._active_map_id = self._map_id_for_dir(self._active_map_dir)
        self._active_map_signature = self._signature_for_dir(self._active_map_dir)
        self._runtime_backup_dir = (self.maps_root / ".active-runtime-backup").resolve()
        self._runtime_backup_map_name = ""
        self._runtime_backup_signature = ""
        self._clear_runtime_backup()
        self._state_lock = RLock()
        self._callback_group = ReentrantCallbackGroup()
        self._map_server_client = self.create_client(
            Nav2LoadMap,
            map_server_load_service,
            callback_group=self._callback_group,
        )
        self._route_load_client = self.create_client(
            LoadRobotMap,
            route_load_map_service,
            callback_group=self._callback_group,
        )
        self._status_load_client = self.create_client(
            LoadRobotMap,
            status_load_map_service,
            callback_group=self._callback_group,
        )
        self.create_service(
            LoadRobotMap,
            manager_load_service,
            self._handle_load_map,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetRobotMapState,
            manager_state_service,
            self._handle_get_state,
            callback_group=self._callback_group,
        )
        self.create_service(
            ListRobotMaps,
            manager_list_service,
            self._handle_list_maps,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetRobotMapBundle,
            manager_get_bundle_service,
            self._handle_get_bundle,
            callback_group=self._callback_group,
        )
        self.create_service(
            PutRobotMapBundle,
            manager_put_bundle_service,
            self._handle_put_bundle,
            callback_group=self._callback_group,
        )
        self._persist_state()

    def _handle_get_state(self, _request, response):
        with self._state_lock:
            response.ok = True
            response.error = ""
            response.map_name = self._active_map_name()
            response.map_dir = str(self._active_map_dir)
            response.map_id = self._active_map_id
            response.signature = self._active_map_signature
        return response

    def _handle_load_map(self, request, response):
        try:
            with self._state_lock:
                target = self._resolve_target_map(
                    map_name=str(request.map_name or "").strip(),
                    map_dir=str(request.map_dir or "").strip(),
                )
                self._load_target_map(target)
                response.ok = True
                response.error = ""
                response.map_name = self._active_map_name()
                response.map_dir = str(self._active_map_dir)
                response.map_id = self._active_map_id
                response.signature = self._active_map_signature
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.map_name = ""
            response.map_dir = ""
            response.map_id = ""
            response.signature = ""
        return response

    def _handle_list_maps(self, _request, response):
        try:
            with self._state_lock:
                names: list[str] = []
                dirs: list[str] = []
                ids: list[str] = []
                signatures: list[str] = []
                for item in sorted(self.maps_root.glob("*.smap")):
                    if not item.is_dir() or not (item / "LMs.yaml").exists():
                        continue
                    names.append(item.stem.replace(".smap", ""))
                    dirs.append(str(item.resolve()))
                    ids.append(self._fast_map_id_for_dir(item))
                    signatures.append(self._signature_for_dir(item))
                response.ok = True
                response.error = ""
                response.active_map_name = self._active_map_name()
                response.active_map_dir = str(self._active_map_dir)
                response.active_map_id = self._active_map_id
                response.active_map_signature = self._active_map_signature
                response.map_names = names
                response.map_dirs = dirs
                response.map_ids = ids
                response.map_signatures = signatures
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.active_map_name = ""
            response.active_map_dir = ""
            response.active_map_id = ""
            response.active_map_signature = ""
            response.map_names = []
            response.map_dirs = []
            response.map_ids = []
            response.map_signatures = []
        return response

    def _handle_get_bundle(self, request, response):
        try:
            with self._state_lock:
                target = self._resolve_target_map(
                    map_name=str(request.map_name or "").strip() or self._active_map_name(),
                    map_dir="",
                )
                payload = build_editable_map_bundle_payload(target)
                response.ok = True
                response.error = ""
                response.map_name = str(payload.get("mapName") or target.stem.replace(".smap", ""))
                response.map_dir = str(target)
                response.map_id = self._map_id_for_dir(target)
                response.signature = str(payload.get("signature") or editable_map_signature(payload))
                response.bundle_json = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.map_name = ""
            response.map_dir = ""
            response.map_id = ""
            response.signature = ""
            response.bundle_json = ""
        return response

    def _handle_put_bundle(self, request, response):
        try:
            with self._state_lock:
                payload = json.loads(str(request.bundle_json or "{}"))
                if not isinstance(payload, dict):
                    raise ValueError("bundle_json must contain an object")
                map_name = str(request.map_name or payload.get("mapName") or "").strip()
                target = self._resolve_put_target_map(map_name)
                signature = self._store_bundle_atomically(target, payload)
                if bool(request.activate):
                    self._load_target_map(target)
                response.ok = True
                response.error = ""
                response.map_name = target.stem.replace(".smap", "")
                response.map_dir = str(target)
                response.map_id = self._map_id_for_dir(target)
                response.signature = signature
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.map_name = ""
            response.map_dir = ""
            response.map_id = ""
            response.signature = ""
        return response

    def _load_target_map(self, target: Path) -> None:
        previous = self._active_map_dir
        previous_name = self._active_map_name()
        previous_id = self._active_map_id
        previous_signature = self._active_map_signature
        rollback_target = previous
        has_runtime_backup = (
            self._runtime_backup_dir.is_dir()
            and self._runtime_backup_map_name == previous_name
            and self._runtime_backup_signature == self._active_map_signature
        )
        if has_runtime_backup:
            rollback_target = self._runtime_backup_dir
        try:
            self._apply_target_map(target)
            new_map_id = self._map_id_for_dir(target)
            new_signature = self._signature_for_dir(target)
            self._active_map_dir = target
            self._active_map_id = new_map_id
            self._active_map_signature = new_signature
            self._persist_state()
        except Exception as exc:
            self._active_map_dir = previous
            self._active_map_id = previous_id
            self._active_map_signature = previous_signature
            rollback_error = ""
            if (
                rollback_target.is_dir()
                and (has_runtime_backup or previous.resolve() != target.resolve())
            ):
                try:
                    self._apply_target_map(rollback_target, map_name=previous_name)
                except Exception as rollback_exc:
                    rollback_error = f"; rollback failed: {rollback_exc}"
            raise ValueError(f"map load transaction failed: {exc}{rollback_error}") from exc

        self._clear_runtime_backup()

    def _apply_target_map(self, target: Path, *, map_name: str = "") -> None:
        map_yaml = find_ros_map_yaml(target)
        nav2_request = Nav2LoadMap.Request()
        nav2_request.map_url = str(map_yaml)
        nav2_response = self._call_service(self._map_server_client, nav2_request, "map_server/load_map")
        if int(nav2_response.result) != int(Nav2LoadMap.Response.RESULT_SUCCESS):
            raise ValueError(f"map_server rejected map load with result={int(nav2_response.result)}")

        internal_request = LoadRobotMap.Request()
        internal_request.map_name = str(map_name or target.stem.replace(".smap", ""))
        internal_request.map_dir = str(target)
        self._require_ok(self._call_service(self._route_load_client, internal_request, "route/load_map"), "route/load_map")
        self._require_ok(self._call_service(self._status_load_client, internal_request, "status/load_map"), "status/load_map")

    def _store_bundle_atomically(self, target: Path, payload: dict) -> str:
        self.maps_root.mkdir(parents=True, exist_ok=True)
        staging = Path(
            tempfile.mkdtemp(
                prefix=f".{target.name}.upload-",
                dir=str(self.maps_root),
            )
        ).resolve()
        backup = (self.maps_root / f".{target.name}.backup").resolve()
        swapped = False
        backed_up = False
        try:
            restore_editable_map_bundle(staging, payload)
            staged_payload = build_editable_map_payload(staging)
            staged_payload["mapName"] = target.stem.replace(".smap", "")
            staged_signature = editable_map_signature(staged_payload)
            expected_signature = str(payload.get("signature") or "").strip()
            if expected_signature and staged_signature != expected_signature:
                raise ValueError(
                    "uploaded map signature mismatch: "
                    f"expected {expected_signature}, got {staged_signature}"
                )
            if (
                target.resolve() == self._active_map_dir.resolve()
                and staged_signature != self._active_map_signature
            ):
                self._preserve_active_runtime_map(target)
            if backup.exists():
                shutil.rmtree(backup)
            if target.exists():
                target.rename(backup)
                backed_up = True
            staging.rename(target)
            swapped = True
            verified_signature = self._signature_for_dir(target)
            if verified_signature != staged_signature:
                raise ValueError(
                    "stored map signature mismatch after atomic replace: "
                    f"expected {staged_signature}, got {verified_signature}"
                )
            if backup.exists():
                shutil.rmtree(backup)
            return verified_signature
        except Exception:
            if swapped and target.exists():
                shutil.rmtree(target)
            if backed_up and backup.exists():
                backup.rename(target)
            raise
        finally:
            if staging.exists():
                shutil.rmtree(staging)

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

    def _resolve_put_target_map(self, map_name: str) -> Path:
        safe_name = Path(map_name).name
        if not safe_name:
            raise ValueError("map_name is required")
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        candidate = (self.maps_root / safe_name).resolve()
        if self.maps_root not in candidate.parents:
            raise ValueError("map must stay inside maps_root")
        return candidate

    def _map_id_for_dir(self, map_dir: Path) -> str:
        loaded_map = WarehouseMapLoader(map_dir).load()
        return loaded_map.map_metadata.map_name

    def _fast_map_id_for_dir(self, map_dir: Path) -> str:
        resolved = Path(map_dir).resolve()
        if resolved == self._active_map_dir.resolve() and self._active_map_id:
            return self._active_map_id
        return resolved.stem.replace(".smap", "")

    def _signature_for_dir(self, map_dir: Path, *, map_name: str = "") -> str:
        resolved = Path(map_dir).resolve()
        payload = build_editable_map_payload(resolved)
        payload["mapName"] = str(map_name or resolved.stem.replace(".smap", ""))
        return editable_map_signature(payload)

    def _preserve_active_runtime_map(self, target: Path) -> None:
        active_name = self._active_map_name()
        if (
            self._runtime_backup_dir.is_dir()
            and self._runtime_backup_map_name == active_name
            and self._runtime_backup_signature == self._active_map_signature
        ):
            return
        current_signature = self._signature_for_dir(target, map_name=active_name)
        if current_signature != self._active_map_signature:
            raise ValueError(
                "cannot preserve active runtime map before Push: "
                f"runtime={self._active_map_signature}, storage={current_signature}"
            )
        staging = Path(
            tempfile.mkdtemp(
                prefix=".active-runtime-backup-",
                dir=str(self.maps_root),
            )
        ).resolve()
        shutil.rmtree(staging)
        try:
            shutil.copytree(target, staging)
            copied_signature = self._signature_for_dir(staging, map_name=active_name)
            if copied_signature != self._active_map_signature:
                raise ValueError("active runtime backup verification failed")
            if self._runtime_backup_dir.exists():
                shutil.rmtree(self._runtime_backup_dir)
            staging.rename(self._runtime_backup_dir)
            self._runtime_backup_map_name = active_name
            self._runtime_backup_signature = copied_signature
        finally:
            if staging.exists():
                shutil.rmtree(staging)

    def _clear_runtime_backup(self) -> None:
        try:
            if self._runtime_backup_dir.exists():
                shutil.rmtree(self._runtime_backup_dir)
        except OSError as exc:
            self.get_logger().warning(f"failed to remove stale runtime map backup: {exc}")
            return
        self._runtime_backup_map_name = ""
        self._runtime_backup_signature = ""

    def _active_map_name(self) -> str:
        return self._active_map_dir.stem.replace(".smap", "")

    def _persist_state(self) -> None:
        payload = {
            "mapName": self._active_map_name(),
            "mapDir": str(self._active_map_dir),
            "mapId": self._active_map_id,
            "signature": self._active_map_signature,
        }
        atomic_write_text(
            self.state_file,
            json.dumps(
                payload,
                ensure_ascii=False,
                indent=2,
            ),
        )

    def _call_service(self, client, request, service_label: str, *, timeout_sec: float = 15.0):
        if not client.wait_for_service(timeout_sec=2.0):
            raise ValueError(f"{service_label} is not available")
        future = client.call_async(request)
        deadline = monotonic() + max(1.0, float(timeout_sec))
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
    parser.add_argument("--manager-list-service", default="/robot/maps/list")
    parser.add_argument("--manager-get-bundle-service", default="/robot/maps/get_bundle")
    parser.add_argument("--manager-put-bundle-service", default="/robot/maps/put_bundle")
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
        manager_list_service=args.manager_list_service,
        manager_get_bundle_service=args.manager_get_bundle_service,
        manager_put_bundle_service=args.manager_put_bundle_service,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
