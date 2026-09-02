"""Validate a fleet/SMAP contract and generate one shared Stage world."""

from __future__ import annotations

from dataclasses import dataclass, replace
from ipaddress import ip_address, IPv4Address
import math
import os
from pathlib import Path
import re
import tempfile

import yaml


_ROBOT_ID_PATTERN = re.compile(r'[A-Za-z][A-Za-z0-9_-]{1,63}')
_WORLD_NAME_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}')
_COLOR_PATTERN = re.compile(r'[A-Za-z][A-Za-z0-9_-]{0,63}')
_SUPPORT_YAML_NAMES = {
    'lms.yaml',
    'graphs.yaml',
    'graph_edges_lengths.yaml',
    'traffic_zones.yaml',
}


@dataclass(frozen=True)
class FleetMap:
    """The canonical ROS map resolved from an SMAP bundle."""

    smap_path: Path
    map_yaml_path: Path
    bitmap_path: Path
    width_pixels: int
    height_pixels: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def width(self) -> float:
        return self.width_pixels * self.resolution

    @property
    def height(self) -> float:
        return self.height_pixels * self.resolution

    @property
    def center_x(self) -> float:
        width_offset = math.cos(self.origin_yaw) * self.width / 2.0
        height_offset = math.sin(self.origin_yaw) * self.height / 2.0
        return self.origin_x + width_offset - height_offset

    @property
    def center_y(self) -> float:
        width_offset = math.sin(self.origin_yaw) * self.width / 2.0
        height_offset = math.cos(self.origin_yaw) * self.height / 2.0
        return self.origin_y + width_offset + height_offset

    @property
    def window_width(self) -> float:
        aspect = self.width / self.height
        if aspect >= 1400.0 / 900.0:
            return 1400.0
        return max(600.0, round(900.0 * aspect))

    @property
    def window_height(self) -> float:
        aspect = self.width / self.height
        if aspect < 1400.0 / 900.0:
            return 900.0
        return max(600.0, round(1400.0 / aspect))

    @property
    def window_scale(self) -> float:
        return max(
            5.0,
            min(self.window_width / self.width, self.window_height / self.height) * 0.9,
        )


@dataclass(frozen=True)
class FleetRobot:
    """One Stage model and its namespace-free network/ROS-domain identity."""

    robot_id: str
    ros_domain_id: int
    ip: str
    grpc_port: int
    lm: str
    x: float
    y: float
    z: float
    yaw: float
    color: str


@dataclass(frozen=True)
class FleetDefinition:
    """Validated fleet file with its single canonical map and robot set."""

    world: str
    fleet_config_path: Path
    smap: FleetMap
    robots: tuple[FleetRobot, ...]


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{field} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field} must be a finite number') from error
    if not math.isfinite(result):
        raise ValueError(f'{field} must be a finite number')
    return result


def _integer(value: object, field: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool):
        raise ValueError(f'{field} must be an integer from {minimum} to {maximum}')
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field} must be an integer from {minimum} to {maximum}'
        ) from error
    if str(value).strip() != str(result) or not minimum <= result <= maximum:
        raise ValueError(f'{field} must be an integer from {minimum} to {maximum}')
    return result


def _domain_id(value: object, field: str) -> int:
    return _integer(value, field, 0, 232)


def _load_yaml(path: Path, description: str) -> object:
    try:
        return yaml.safe_load(path.read_text(encoding='utf-8'))
    except OSError as error:
        raise ValueError(f'Cannot read {description} {path}: {error}') from error
    except yaml.YAMLError as error:
        raise ValueError(f'Invalid YAML in {description} {path}: {error}') from error


def _resolve_smap(config_path: Path, value: object) -> Path:
    raw_path = str(value or '').strip()
    if not raw_path:
        raise ValueError('fleet config must contain a non-empty smap path')
    candidate = Path(raw_path).expanduser()
    if not candidate.is_absolute():
        candidate = config_path.parent / candidate
    result = candidate.resolve()
    if result.suffix != '.smap' or not result.is_dir():
        raise ValueError(f'smap must be an existing .smap directory: {result}')
    return result


def _find_map_yaml(smap_path: Path) -> tuple[Path, dict]:
    candidates: list[tuple[Path, dict]] = []
    for path in sorted(smap_path.glob('*.yaml')):
        if path.name.lower() in _SUPPORT_YAML_NAMES:
            continue
        payload = _load_yaml(path, 'ROS map YAML')
        if isinstance(payload, dict) and 'image' in payload and 'resolution' in payload:
            candidates.append((path.resolve(), payload))
    if len(candidates) != 1:
        raise ValueError(
            f'SMAP {smap_path} must contain exactly one ROS map YAML; '
            f'found {len(candidates)}'
        )
    return candidates[0]


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    try:
        header = path.read_bytes()[:65536]
    except OSError as error:
        raise ValueError(f'Cannot read map image {path}: {error}') from error
    cleaned = re.sub(rb'#[^\r\n]*', b'', header)
    tokens = cleaned.split()
    if len(tokens) < 4 or tokens[0] not in (b'P2', b'P5'):
        raise ValueError(f'map image must be a P2 or P5 PGM: {path}')
    try:
        width, height, maximum = (int(value) for value in tokens[1:4])
    except ValueError as error:
        raise ValueError(f'invalid PGM header: {path}') from error
    if width <= 0 or height <= 0 or not 1 <= maximum <= 65535:
        raise ValueError(f'invalid PGM dimensions or maximum value: {path}')
    return width, height


def _load_map(smap_path: Path) -> FleetMap:
    map_yaml_path, payload = _find_map_yaml(smap_path)
    raw_image = str(payload.get('image') or '').strip()
    if not raw_image:
        raise ValueError(f'{map_yaml_path} image must be a non-empty path')
    bitmap_path = Path(raw_image).expanduser()
    if not bitmap_path.is_absolute():
        bitmap_path = map_yaml_path.parent / bitmap_path
    bitmap_path = bitmap_path.resolve()
    if smap_path not in bitmap_path.parents or not bitmap_path.is_file():
        raise ValueError(f'map image must be an existing file inside SMAP: {bitmap_path}')

    resolution = _finite_number(payload.get('resolution'), 'map.resolution')
    if resolution <= 0.0:
        raise ValueError('map.resolution must be greater than zero')
    origin = payload.get('origin')
    if not isinstance(origin, list) or len(origin) < 3:
        raise ValueError('map.origin must contain x, y and yaw')
    width, height = _pgm_dimensions(bitmap_path)
    return FleetMap(
        smap_path=smap_path,
        map_yaml_path=map_yaml_path,
        bitmap_path=bitmap_path,
        width_pixels=width,
        height_pixels=height,
        resolution=resolution,
        origin_x=_finite_number(origin[0], 'map.origin[0]'),
        origin_y=_finite_number(origin[1], 'map.origin[1]'),
        origin_yaw=_finite_number(origin[2], 'map.origin[2]'),
    )


def _load_landmarks(fleet_map: FleetMap) -> dict[str, tuple[float, float]]:
    path = fleet_map.smap_path / 'LMs.yaml'
    payload = _load_yaml(path, 'SMAP landmarks')
    entries = payload.get('LMs') if isinstance(payload, dict) else None
    if not isinstance(entries, list) or not entries:
        raise ValueError(f'{path} must contain a non-empty LMs list')
    coordinate_frame = str(payload.get('coordinateFrame') or '').strip()
    landmarks: dict[str, tuple[float, float]] = {}
    for index, entry in enumerate(entries):
        field = f'LMs[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{field} must be a mapping')
        name = str(entry.get('name') or '').strip()
        if not name:
            raise ValueError(f'{field}.name must not be empty')
        if name in landmarks:
            raise ValueError(f'duplicate LM name: {name}')
        x = _finite_number(entry.get('x'), f'{field}.x')
        y = _finite_number(entry.get('y'), f'{field}.y')
        if coordinate_frame == 'map_top_left':
            local_x = x
            local_y = fleet_map.height - y
            cos_yaw = math.cos(fleet_map.origin_yaw)
            sin_yaw = math.sin(fleet_map.origin_yaw)
            x = fleet_map.origin_x + cos_yaw * local_x - sin_yaw * local_y
            y = fleet_map.origin_y + sin_yaw * local_x + cos_yaw * local_y
        elif coordinate_frame not in ('', 'map'):
            raise ValueError(
                f'unsupported LMs coordinateFrame {coordinate_frame!r}; '
                'expected map or map_top_left'
            )
        landmarks[name] = (x, y)
    return landmarks


def _ipv4(value: object, field: str) -> str:
    raw_value = str(value or '').strip()
    try:
        result = ip_address(raw_value)
    except ValueError as error:
        raise ValueError(f'{field} must be a valid IPv4 address') from error
    if not isinstance(result, IPv4Address):
        raise ValueError(f'{field} must be a valid IPv4 address')
    return str(result)


def load_fleet_definition(path: str | os.PathLike[str]) -> FleetDefinition:
    """Load the strict world, SMAP and LM-based robot fleet contract."""
    config_path = Path(path).expanduser().resolve()
    payload = _load_yaml(config_path, 'fleet config')
    if not isinstance(payload, dict):
        raise ValueError('fleet config must be a mapping')
    world = str(payload.get('world') or '').strip()
    if not _WORLD_NAME_PATTERN.fullmatch(world):
        raise ValueError(f'fleet config world is invalid: {world!r}')
    fleet_map = _load_map(_resolve_smap(config_path, payload.get('smap')))
    landmarks = _load_landmarks(fleet_map)

    entries = payload.get('robots')
    if not isinstance(entries, list) or not entries:
        raise ValueError('fleet config must contain a non-empty robots list')
    robots: list[FleetRobot] = []
    robot_ids: set[str] = set()
    domain_ids: set[int] = set()
    ip_addresses: set[str] = set()
    for index, entry in enumerate(entries):
        field = f'robots[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{field} must be a mapping')
        robot_id = str(entry.get('robot_id') or '').strip()
        if not _ROBOT_ID_PATTERN.fullmatch(robot_id):
            raise ValueError(f'{field}.robot_id is invalid: {robot_id!r}')
        domain_id = _domain_id(entry.get('ros_domain_id'), f'{field}.ros_domain_id')
        address = _ipv4(entry.get('ip'), f'{field}.ip')
        port = _integer(entry.get('grpc_port', 50051), f'{field}.grpc_port', 1, 65535)
        lm = str(entry.get('lm') or '').strip()
        if lm not in landmarks:
            raise ValueError(f'{field}.lm does not exist in SMAP: {lm!r}')
        color = str(entry.get('color', 'LightSteelBlue')).strip()
        if not _COLOR_PATTERN.fullmatch(color):
            raise ValueError(f'{field}.color is invalid: {color!r}')
        if robot_id in robot_ids:
            raise ValueError(f'duplicate robot_id: {robot_id}')
        if domain_id in domain_ids:
            raise ValueError(f'duplicate ros_domain_id: {domain_id}')
        if address in ip_addresses:
            raise ValueError(f'duplicate robot ip: {address}')
        robot_ids.add(robot_id)
        domain_ids.add(domain_id)
        ip_addresses.add(address)
        x, y = landmarks[lm]
        robots.append(
            FleetRobot(
                robot_id=robot_id,
                ros_domain_id=domain_id,
                ip=address,
                grpc_port=port,
                lm=lm,
                x=x,
                y=y,
                z=_finite_number(entry.get('z', 0.0), f'{field}.z'),
                yaw=_finite_number(entry.get('yaw', 0.0), f'{field}.yaw'),
                color=color,
            )
        )
    return FleetDefinition(
        world=world,
        fleet_config_path=config_path,
        smap=fleet_map,
        robots=tuple(robots),
    )


def load_fleet_config(path: str | os.PathLike[str]) -> list[FleetRobot]:
    """Compatibility helper returning just the validated robot list."""
    return list(load_fleet_definition(path).robots)


def apply_domain_override(
    robots: list[FleetRobot] | tuple[FleetRobot, ...], config: str
) -> list[FleetRobot]:
    """Override domains while requiring the configured robot set to match."""
    if not config.strip():
        return list(robots)

    overrides: dict[str, int] = {}
    used_domains: set[int] = set()
    for raw_entry in config.split(','):
        entry = raw_entry.strip()
        if not entry or entry.count('=') != 1:
            raise ValueError(
                f'invalid robot_domain_map entry {entry!r}; '
                'expected robot_id=domain_id'
            )
        robot_id, raw_domain = (part.strip() for part in entry.split('=', 1))
        if not _ROBOT_ID_PATTERN.fullmatch(robot_id):
            raise ValueError(f'invalid robot_id in robot_domain_map: {robot_id!r}')
        domain_id = _domain_id(raw_domain, f'robot_domain_map[{robot_id}]')
        if robot_id in overrides:
            raise ValueError(f'duplicate robot_id in robot_domain_map: {robot_id}')
        if domain_id in used_domains:
            raise ValueError(f'duplicate ROS domain in robot_domain_map: {domain_id}')
        overrides[robot_id] = domain_id
        used_domains.add(domain_id)

    configured_ids = {robot.robot_id for robot in robots}
    if set(overrides) != configured_ids:
        missing = sorted(configured_ids - set(overrides))
        unknown = sorted(set(overrides) - configured_ids)
        details = []
        if missing:
            details.append(f'missing {missing}')
        if unknown:
            details.append(f'not present in fleet config {unknown}')
        raise ValueError('robot_domain_map does not match fleet config: ' + ', '.join(details))
    return [replace(robot, ros_domain_id=overrides[robot.robot_id]) for robot in robots]


def robot_domain_map(robots: list[FleetRobot] | tuple[FleetRobot, ...]) -> str:
    """Return the C++ Stage node mapping syntax."""
    return ','.join(f'{robot.robot_id}={robot.ros_domain_id}' for robot in robots)


def find_robot(
    robots: list[FleetRobot] | tuple[FleetRobot, ...], robot_id: str
) -> FleetRobot | None:
    """Find a robot identity in a validated fleet."""
    return next((robot for robot in robots if robot.robot_id == robot_id), None)


def _stage_path(path: str | os.PathLike[str], field: str) -> str:
    result = str(Path(path).expanduser().resolve())
    if '"' in result or '\n' in result or '\r' in result:
        raise ValueError(f'{field} cannot be represented in a Stage world')
    return result


def _format_number(value: float) -> str:
    result = f'{value:.6f}'
    return '0.000000' if result == '-0.000000' else result


def render_fleet_world(
    template: str,
    fleet: FleetDefinition,
    *,
    trp1_include: str | os.PathLike[str],
) -> str:
    """Render robots and the canonical SMAP PGM into a Stage world."""
    tokens = {
        '@TRP1_INCLUDE@': _stage_path(trp1_include, 'trp1_include'),
        '@WORLD_NAME@': fleet.world,
        '@MAP_BITMAP@': _stage_path(fleet.smap.bitmap_path, 'map_bitmap'),
        '@MAP_WIDTH@': _format_number(fleet.smap.width),
        '@MAP_HEIGHT@': _format_number(fleet.smap.height),
        '@MAP_CENTER_X@': _format_number(fleet.smap.center_x),
        '@MAP_CENTER_Y@': _format_number(fleet.smap.center_y),
        '@MAP_ORIGIN_YAW_DEG@': _format_number(
            math.degrees(fleet.smap.origin_yaw)
        ),
        '@WINDOW_WIDTH@': _format_number(fleet.smap.window_width),
        '@WINDOW_HEIGHT@': _format_number(fleet.smap.window_height),
        '@WINDOW_SCALE@': _format_number(fleet.smap.window_scale),
    }
    for token in (*tokens, '@ROBOT_MODELS@'):
        if template.count(token) < 1:
            raise ValueError(f'world template must contain {token}')

    blocks = []
    for robot in fleet.robots:
        blocks.append(
            '\n'.join(
                (
                    f'# {robot.robot_id} / {robot.ip}:{robot.grpc_port} / '
                    f'ROS_DOMAIN_ID={robot.ros_domain_id} / LM={robot.lm}',
                    'trp1_with_laser',
                    '(',
                    f'  name "{robot.robot_id}"',
                    f'  color "{robot.color}"',
                    '  pose [ '
                    f'{_format_number(robot.x)} {_format_number(robot.y)} '
                    f'{_format_number(robot.z)} {_format_number(robot.yaw)} ]',
                    ')',
                )
            )
        )
    result = template
    for token, value in tokens.items():
        result = result.replace(token, value)
    return result.replace('@ROBOT_MODELS@', '\n\n'.join(blocks))


def write_generated_world(
    template_path: str | os.PathLike[str],
    fleet: FleetDefinition,
    *,
    trp1_include: str | os.PathLike[str],
) -> Path:
    """Generate a temporary world file and return its absolute path."""
    source = Path(template_path).expanduser().resolve()
    try:
        template = source.read_text(encoding='utf-8')
    except OSError as error:
        raise ValueError(f'Cannot read world template {source}: {error}') from error
    rendered = render_fleet_world(template, fleet, trp1_include=trp1_include)
    descriptor, output_path = tempfile.mkstemp(
        prefix='stage_ros2_fleet_', suffix='.world'
    )
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        stream.write(rendered)
    return Path(output_path)
