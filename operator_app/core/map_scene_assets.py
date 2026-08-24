"""Persist browser-ready static 3D assets inside an smap directory."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import struct
import tempfile
from typing import Any

from PIL import Image
import yaml

from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.storage import atomic_write_json

from .map_scene import MapSceneBuilder


SCENE3D_SCHEMA_VERSION = 1
SCENE3D_ALGORITHM_VERSION = "merged-wall-matrix-v1"
SCENE3D_DIRECTORY = "scene3d"
SCENE3D_MANIFEST = "manifest.json"
SCENE3D_WALLS = "walls.f32"
SCENE3D_FLOOR = "floor.png"


class MapSceneAssetBuilder:
    """Build and validate immutable static assets for one loaded map."""

    def __init__(self, scene_builder: MapSceneBuilder) -> None:
        self.scene_builder = scene_builder
        self.map_dir = scene_builder.map_dir

    @property
    def asset_dir(self) -> Path:
        return self.map_dir / SCENE3D_DIRECTORY

    def ensure(self, *, wall_height: float = 1.8) -> dict[str, Any]:
        source_digest = self.source_digest(wall_height=wall_height)
        manifest = self._read_manifest()
        if self._manifest_is_current(manifest, source_digest):
            return manifest
        return self.build(
            wall_height=wall_height,
            source_digest=source_digest,
        )

    def build(
        self,
        *,
        wall_height: float = 1.8,
        source_digest: str | None = None,
    ) -> dict[str, Any]:
        digest = source_digest or self.source_digest(wall_height=wall_height)
        ros_map_path = self.scene_builder.find_ros_map_yaml()
        ros_map = yaml.safe_load(ros_map_path.read_text(encoding="utf-8"))
        if not isinstance(ros_map, dict):
            raise ValueError(f"Unexpected ROS map file format: {ros_map_path}")
        image_path = (self.map_dir / str(ros_map["image"])).resolve()
        width, height, pixels = WarehouseMapLoader(self.map_dir).load_pgm(
            image_path
        )
        walls = self.scene_builder.wall_rectangles(wall_height=wall_height)

        staging = Path(
            tempfile.mkdtemp(
                prefix=".scene3d.incoming-",
                dir=str(self.map_dir),
            )
        ).resolve()
        target = self.asset_dir.resolve()
        backup = (self.map_dir / ".scene3d.backup").resolve()
        swapped = False
        backed_up = False
        try:
            wall_bytes = _wall_matrix_bytes(walls)
            wall_path = staging / SCENE3D_WALLS
            wall_path.write_bytes(wall_bytes)
            floor_path = staging / SCENE3D_FLOOR
            Image.frombytes("L", (width, height), pixels).save(
                floor_path,
                format="PNG",
                optimize=True,
            )
            manifest = {
                "schemaVersion": SCENE3D_SCHEMA_VERSION,
                "algorithmVersion": SCENE3D_ALGORITHM_VERSION,
                "sourceDigest": digest,
                "coordinateFrame": "map_top_left",
                "wallHeight": float(wall_height),
                "walls": {
                    "path": SCENE3D_WALLS,
                    "encoding": "float32-le-matrix4",
                    "count": len(walls),
                    "stride": int(walls[0].get("stride", 1)) if walls else 1,
                    "size": len(wall_bytes),
                    "sha256": _sha256_bytes(wall_bytes),
                },
                "floor": {
                    "path": SCENE3D_FLOOR,
                    "width": width,
                    "height": height,
                    "size": floor_path.stat().st_size,
                    "sha256": _sha256_file(floor_path),
                },
            }
            atomic_write_json(staging / SCENE3D_MANIFEST, manifest)
            if backup.exists():
                shutil.rmtree(backup)
            if target.exists():
                target.rename(backup)
                backed_up = True
            staging.rename(target)
            swapped = True
            verified = self._read_manifest()
            if not self._manifest_is_current(verified, digest):
                raise ValueError("generated scene3d assets failed verification")
            if backup.exists():
                shutil.rmtree(backup)
            return verified
        except Exception:
            if swapped and target.exists():
                shutil.rmtree(target)
            if backed_up and backup.exists():
                backup.rename(target)
            raise
        finally:
            if staging.exists():
                shutil.rmtree(staging)

    def source_digest(self, *, wall_height: float) -> str:
        ros_map_path = self.scene_builder.find_ros_map_yaml()
        ros_map = yaml.safe_load(ros_map_path.read_text(encoding="utf-8"))
        if not isinstance(ros_map, dict):
            raise ValueError(f"Unexpected ROS map file format: {ros_map_path}")
        image_path = (self.map_dir / str(ros_map["image"])).resolve()
        digest = hashlib.sha256()
        digest.update(SCENE3D_ALGORITHM_VERSION.encode("ascii"))
        digest.update(struct.pack("<f", float(wall_height)))
        digest.update(ros_map_path.read_bytes())
        digest.update(image_path.read_bytes())
        return digest.hexdigest()

    def resolve_asset(self, source_digest: str, relative_path: str) -> Path:
        manifest = self.ensure()
        expected_digest = str(manifest.get("sourceDigest") or "")
        if source_digest != expected_digest:
            raise ValueError("scene3d asset version is stale")
        allowed = {
            str(manifest.get("walls", {}).get("path") or ""),
            str(manifest.get("floor", {}).get("path") or ""),
            SCENE3D_MANIFEST,
        }
        normalized = str(relative_path or "").strip().replace("\\", "/")
        if normalized not in allowed:
            raise ValueError("unknown scene3d asset")
        target = (self.asset_dir / normalized).resolve()
        if self.asset_dir.resolve() not in target.parents:
            raise ValueError("scene3d asset must stay inside the map directory")
        if not target.is_file():
            raise ValueError("scene3d asset is missing")
        return target

    def _read_manifest(self) -> dict[str, Any]:
        path = self.asset_dir / SCENE3D_MANIFEST
        if not path.is_file():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return payload if isinstance(payload, dict) else {}

    def _manifest_is_current(
        self,
        manifest: dict[str, Any],
        source_digest: str,
    ) -> bool:
        if (
            int(manifest.get("schemaVersion") or 0) != SCENE3D_SCHEMA_VERSION
            or str(manifest.get("algorithmVersion") or "")
            != SCENE3D_ALGORITHM_VERSION
            or str(manifest.get("sourceDigest") or "") != source_digest
        ):
            return False
        for section_name in ("walls", "floor"):
            section = manifest.get(section_name)
            if not isinstance(section, dict):
                return False
            relative = str(section.get("path") or "")
            path = (self.asset_dir / relative).resolve()
            if self.asset_dir.resolve() not in path.parents or not path.is_file():
                return False
            recorded_size = section.get("size")
            if recorded_size is None or int(recorded_size) != path.stat().st_size:
                return False
            if str(section.get("sha256") or "") != _sha256_file(path):
                return False
        return True


def _wall_matrix_bytes(walls: list[dict[str, Any]]) -> bytes:
    payload = bytearray(len(walls) * 16 * 4)
    for index, wall in enumerate(walls):
        height = max(0.05, float(wall.get("height", 1.8) or 1.8))
        values = (
            max(0.01, float(wall.get("width", 0.01) or 0.01)),
            0.0,
            0.0,
            0.0,
            0.0,
            height,
            0.0,
            0.0,
            0.0,
            0.0,
            max(0.01, float(wall.get("depth", 0.01) or 0.01)),
            0.0,
            float(wall.get("x", 0.0) or 0.0),
            height / 2.0,
            float(wall.get("z", 0.0) or 0.0),
            1.0,
        )
        struct.pack_into("<16f", payload, index * 64, *values)
    return bytes(payload)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def ensure_map_scene_assets(map_dir: Path) -> dict[str, Any]:
    """Build assets for a map cache entry without requiring a fleet manager."""

    loaded_map = WarehouseMapLoader(map_dir).load()
    return MapSceneAssetBuilder(MapSceneBuilder(loaded_map)).ensure()
