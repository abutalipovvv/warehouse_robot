"""Manage SLAM sessions and persist captured map artifacts."""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml

from fleet_manager.core.mapping.formats.pgm import read_pgm_size
from fleet_manager.core.io.atomic_files import atomic_write_bytes, atomic_write_text

from .ros_runtime_lifecycle import _clean_node_suffix

class RosRuntimeSlamMixin:
    """Manage SLAM sessions and persist captured map artifacts."""

    def slam_defaults_payload(self) -> dict[str, Any]:
        try:
            payload = yaml.safe_load(self.slam_params_file.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"SLAM params file does not exist: {self.slam_params_file}") from exc
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            raise ValueError(f"SLAM params file must contain a YAML object: {self.slam_params_file}")
        return {"ok": True, "params": payload, "paramsPath": str(self.slam_params_file)}

    def start_slam(
        self,
        params_payload: dict[str, Any] | None = None,
        *,
        use_sim_time: bool = True,
        command_id: str = "",
    ) -> dict[str, Any]:
        del command_id
        with self._lock:
            if bool(self._slam_state.get("active")):
                raise ValueError("SLAM is already running")

        if not self.slam_launch_file.is_file():
            raise ValueError(f"SLAM launch file does not exist: {self.slam_launch_file}")

        params = params_payload if isinstance(params_payload, dict) and params_payload else self.slam_defaults_payload()["params"]
        session_id = f"slam-{self.robot_id}-{int(time.time())}"
        temp_dir = Path(tempfile.mkdtemp(prefix=f"{_clean_node_suffix(self.robot_id)}-slam-"))
        params_file = temp_dir / "mapper_params_online_async.yaml"
        params_file.write_text(yaml.safe_dump(params, sort_keys=False, allow_unicode=True), encoding="utf-8")

        cmd = [
            "ros2",
            "launch",
            str(self.slam_launch_file),
            f"slam_params_file:={params_file}",
            f"use_sim_time:={'true' if use_sim_time else 'false'}",
        ]
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except FileNotFoundError as exc:
            shutil.rmtree(temp_dir, ignore_errors=True)
            raise ValueError("ros2 command is not available; source the ROS environment before starting SLAM") from exc

        with self._lock:
            self._latest_map = None
            self._latest_map_at = None
            self._slam_trail = []
            self._slam_process = process
            self._slam_temp_dir = temp_dir
            self._slam_state = {
                "active": True,
                "state": "mapping",
                "message": "2D SLAM is running. Manual WASD teleop is available.",
                "sessionId": session_id,
                "startedAtSec": time.time(),
                "progress": 0,
                "savedMapName": "",
                "mapDir": "",
            }
            state = dict(self._slam_state)
        return {"ok": True, "state": state}

    def slam_state_payload(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._slam_state)
            message = self._latest_map
            trail_points = len(self._slam_trail)
        if message is not None:
            info = getattr(message, "info", None)
            header = getattr(message, "header", None)
            state.update(
                {
                    "mapWidth": int(getattr(info, "width", 0) or 0),
                    "mapHeight": int(getattr(info, "height", 0) or 0),
                    "resolution": float(getattr(info, "resolution", 0.0) or 0.0),
                    "frameId": str(getattr(header, "frame_id", "") or ""),
                }
            )
        state["trailPoints"] = trail_points
        return {"ok": True, "state": state}

    def slam_map_frame_payload(self, *, include_cells: bool = True) -> dict[str, Any]:
        with self._lock:
            message = self._latest_map
            state = dict(self._slam_state)
            trail = list(self._slam_trail)
            active = bool(state.get("active"))
        if message is None:
            state.setdefault("message", f"Waiting for OccupancyGrid on {self.map_topic}.")
            return {
                "ok": True,
                "robotId": self.robot_id,
                "sessionId": str(state.get("sessionId") or ""),
                "state": state,
                "trail": trail,
                "pose": self._current_pose_payload(),
            }

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        stamp_sec = float(getattr(stamp, "sec", 0.0) or 0.0) + float(getattr(stamp, "nanosec", 0.0) or 0.0) / 1e9
        info = getattr(message, "info", None)
        origin = getattr(info, "origin", None)
        origin_position = getattr(origin, "position", None)
        origin_orientation = getattr(origin, "orientation", None)
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        state.update(
            {
                "active": active,
                "mapWidth": width,
                "mapHeight": height,
                "resolution": float(getattr(info, "resolution", 0.0) or 0.0),
                "frameId": str(getattr(header, "frame_id", "") or ""),
                "trailPoints": len(trail),
            }
        )
        cells = b""
        if include_cells:
            cells = self._slam_cells_to_bytes(getattr(message, "data", []) or [])
        return {
            "ok": True,
            "robotId": self.robot_id,
            "sessionId": str(state.get("sessionId") or ""),
            "frameId": str(getattr(header, "frame_id", "") or ""),
            "stampSec": stamp_sec,
            "width": width,
            "height": height,
            "resolution": float(getattr(info, "resolution", 0.0) or 0.0),
            "originX": float(getattr(origin_position, "x", 0.0) or 0.0),
            "originY": float(getattr(origin_position, "y", 0.0) or 0.0),
            "originYaw": self._yaw_from_quaternion(origin_orientation),
            "cells": cells,
            "pose": self._current_pose_payload(),
            "trail": trail,
            "state": state,
        }

    def finish_slam(self, *, map_name: str, activate: bool = True, command_id: str = "") -> dict[str, Any]:
        del command_id
        safe_name = self._safe_map_name(map_name)
        if not safe_name:
            raise ValueError("map_name is required")
        with self._lock:
            if not bool(self._slam_state.get("active")) and self._latest_map is None:
                raise ValueError("SLAM is not running and no live map is available")
            self._slam_state.update({"state": "saving", "message": "Preparing SLAM map save.", "progress": 5})

        maps_root = self._slam_maps_root()
        target = (maps_root / f"{safe_name}.smap").resolve()
        if maps_root not in target.parents:
            raise ValueError("map must stay inside maps_root")
        if target.exists():
            raise ValueError(f"map already exists: {target.name}")

        self._set_slam_progress(20, "Saving occupancy grid as PGM/YAML.")
        target.mkdir(parents=True, exist_ok=False)
        try:
            self._save_slam_map_files(target, safe_name)
            self._set_slam_progress(55, "Creating editable smap files.")
            self._write_empty_smap_sidecars(target, safe_name)
            self._stop_slam_process()
            loaded = {"ok": True, "mapName": safe_name, "mapDir": str(target), "mapId": safe_name}
            if activate:
                self._set_slam_progress(72, "Loading new map on robot.")
                loaded = self.load_map(safe_name)
            self._set_slam_progress(86, "Building map bundle for operator pull.")
            bundle = self.pull_map_bundle_payload(safe_name)
            with self._lock:
                self._slam_state.update(
                    {
                        "active": False,
                        "state": "done",
                        "message": f"SLAM map saved: {safe_name}.",
                        "progress": 100,
                        "savedMapName": safe_name,
                        "mapDir": str(target),
                    }
                )
                state = dict(self._slam_state)
            return {
                "ok": True,
                "state": state,
                "mapName": safe_name,
                "mapDir": str(target),
                "mapId": str(loaded.get("mapId") or safe_name),
                "signature": str(bundle.get("signature") or ""),
                "bundleJson": json.dumps(bundle, ensure_ascii=False),
            }
        except Exception:
            shutil.rmtree(target, ignore_errors=True)
            with self._lock:
                self._slam_state.update({"state": "error", "progress": max(1, int(self._slam_state.get("progress") or 1))})
            raise

    def cancel_slam(self, *, reason: str = "", command_id: str = "") -> dict[str, Any]:
        del command_id
        self._stop_slam_process()
        with self._lock:
            self._slam_state.update(
                {
                    "active": False,
                    "state": "canceled",
                    "message": reason or "SLAM canceled.",
                    "progress": 0,
                }
            )
            state = dict(self._slam_state)
        return {"ok": True, "state": state}

    def _set_slam_progress(self, progress: int, message: str) -> None:
        with self._lock:
            self._slam_state.update(
                {
                    "progress": max(0, min(100, int(progress))),
                    "message": str(message or ""),
                }
            )

    def _slam_cells_to_bytes(self, values: Any) -> bytes:
        encoded = bytearray()
        for item in values:
            try:
                value = int(item)
            except (TypeError, ValueError):
                value = -1
            value = max(-1, min(100, value))
            encoded.append(value + 1)
        return bytes(encoded)

    def _safe_map_name(self, value: str) -> str:
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip()).strip("._")
        if safe.endswith(".smap"):
            safe = safe[:-5]
        return safe

    def _slam_maps_root(self) -> Path:
        try:
            active = self.active_map_payload()
            active_dir = Path(str(active.get("mapDir") or "")).resolve()
            if active_dir.is_dir():
                return active_dir.parent.resolve()
        except Exception:
            pass
        return self._default_slam_launch_file().parents[2] / "robot_map_manager" / "maps_out"

    def _save_slam_map_files(self, target: Path, safe_name: str) -> None:
        service_error = ""
        if self._slam_save_map_client is not None and self._save_map_type is not None:
            try:
                if self._service_available(self._slam_save_map_client, 0.5):
                    request = self._save_map_type.Request()
                    request.name.data = str(target / safe_name)
                    response = self._call_service(self._slam_save_map_client, request, "slam_toolbox/save_map", timeout_sec=20.0)
                    if int(getattr(response, "result", 255)) == 0 and (target / f"{safe_name}.yaml").is_file():
                        return
                    service_error = f"slam_toolbox/save_map result={int(getattr(response, 'result', 255))}"
            except Exception as exc:
                service_error = str(exc)
        self._write_current_map_files(target, safe_name, service_error=service_error)

    def _write_current_map_files(self, target: Path, safe_name: str, *, service_error: str = "") -> None:
        with self._lock:
            message = self._latest_map
        if message is None:
            detail = f" ({service_error})" if service_error else ""
            raise ValueError(f"No live SLAM map is available to save{detail}")
        info = getattr(message, "info", None)
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        resolution = float(getattr(info, "resolution", 0.05) or 0.05)
        if width <= 0 or height <= 0:
            raise ValueError("Live SLAM map has invalid dimensions")
        data = list(getattr(message, "data", []) or [])
        if len(data) < width * height:
            raise ValueError("Live SLAM map data is shorter than expected")

        pixels = bytearray()
        for image_y in range(height):
            grid_y = height - 1 - image_y
            row = data[grid_y * width : (grid_y + 1) * width]
            for item in row:
                try:
                    value = int(item)
                except (TypeError, ValueError):
                    value = -1
                if value < 0:
                    pixels.append(205)
                elif value >= 65:
                    pixels.append(0)
                elif value <= 25:
                    pixels.append(254)
                else:
                    pixels.append(205)

        atomic_write_bytes(
            target / f"{safe_name}.pgm",
            f"P5\n# Created by warehouse_robot SLAM\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
        )
        origin = getattr(info, "origin", None)
        position = getattr(origin, "position", None)
        orientation = getattr(origin, "orientation", None)
        ros_map = {
            "image": f"{safe_name}.pgm",
            "mode": "trinary",
            "resolution": resolution,
            "origin": [
                float(getattr(position, "x", 0.0) or 0.0),
                float(getattr(position, "y", 0.0) or 0.0),
                self._yaw_from_quaternion(orientation),
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        atomic_write_text(
            target / f"{safe_name}.yaml",
            yaml.safe_dump(ros_map, sort_keys=False),
        )

    def _write_empty_smap_sidecars(self, target: Path, safe_name: str) -> None:
        ros_yaml = yaml.safe_load((target / f"{safe_name}.yaml").read_text(encoding="utf-8"))
        resolution = float(ros_yaml.get("resolution", 0.05) if isinstance(ros_yaml, dict) else 0.05)
        width = height = 0
        try:
            width, height = self._pgm_size(target / f"{safe_name}.pgm")
        except Exception:
            pass
        min_x = 0.0
        min_y = 0.0
        max_x = min_x + (width * resolution)
        max_y = min_y + (height * resolution)
        atomic_write_text(
            target / "LMs.yaml",
            yaml.safe_dump(
                {"mapName": safe_name, "coordinateFrame": "map_top_left", "LMs": []},
                sort_keys=False,
                allow_unicode=True,
            ),
        )
        atomic_write_text(
            target / "graphs.yaml",
            yaml.safe_dump(
                {"mapName": safe_name, "coordinateFrame": "map_top_left", "primitives": []},
                sort_keys=False,
                allow_unicode=True,
            ),
        )
        atomic_write_text(
            target / "graph_edges_lengths.yaml",
            yaml.safe_dump([], sort_keys=False, allow_unicode=True),
        )
        atomic_write_text(
            target / "primitives_lengths.csv",
            "idx,kind,type,start_x,start_y,end_x,end_y,length_m\n",
        )
        created_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        atomic_write_text(
            target / ".operator_meta.json",
            json.dumps(
                {
                    "source": "slam_toolbox",
                    "robotId": self.robot_id,
                    "createdAt": created_at,
                    "mapName": safe_name,
                },
                ensure_ascii=False,
                indent=2,
            ),
        )
        summary = {
            "header": {
                "mapType": "2D-Map",
                "mapName": safe_name,
                "minPos": {"x": min_x, "y": min_y},
                "maxPos": {"x": max_x, "y": max_y},
                "resolution": resolution,
                "version": "slam_toolbox",
            },
            "counts": {
                "LMs_found": 0,
                "edges_total": 0,
                "grid": {"width": width, "height": height},
            },
            "outputs": {
                "pgm": f"{safe_name}.pgm",
                "ros_map_yaml": f"{safe_name}.yaml",
                "LMs_yaml": "LMs.yaml",
                "graphs_yaml": "graphs.yaml",
                "graph_edges_lengths_yaml": "graph_edges_lengths.yaml",
                "primitives_lengths_csv": "primitives_lengths.csv",
                "summary_json": "smap_summary.json",
            },
        }
        atomic_write_text(
            target / "smap_summary.json",
            json.dumps(summary, ensure_ascii=False, indent=2),
        )

    def _pgm_size(self, path: Path) -> tuple[int, int]:
        return read_pgm_size(path)

    def _stop_slam_process(self) -> None:
        process = self._slam_process
        self._slam_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except Exception:
                try:
                    process.terminate()
                except Exception:
                    pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except Exception:
                    process.kill()
        temp_dir = self._slam_temp_dir
        self._slam_temp_dir = None
        if temp_dir is not None:
            shutil.rmtree(temp_dir, ignore_errors=True)
