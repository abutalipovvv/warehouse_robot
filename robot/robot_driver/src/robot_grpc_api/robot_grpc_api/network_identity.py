"""Derive a stable per-robot identity from the host's LAN IPv4 address."""

from __future__ import annotations

from dataclasses import dataclass
import fcntl
from ipaddress import IPv4Address
from pathlib import Path
import re
import socket
import struct
from typing import Callable, Iterable


_SIOCGIFADDR = 0x8915
_AUTO_VALUES = {"", "auto"}
_ROBOT_ID_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_-]{1,63}")
_VIRTUAL_PREFIXES = (
    "br-",
    "docker",
    "lo",
    "tailscale",
    "veth",
    "virbr",
    "vmnet",
)


@dataclass(frozen=True)
class NetworkIdentity:
    robot_id: str
    domain_id: int
    ipv4: str
    interface: str


def _interface_ipv4(interface: str) -> str | None:
    request = struct.pack("256s", interface[:15].encode("utf-8"))
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            response = fcntl.ioctl(sock.fileno(), _SIOCGIFADDR, request)
    except OSError:
        return None
    return socket.inet_ntoa(response[20:24])


def _default_route_interface(route_file: Path = Path("/proc/net/route")) -> str:
    try:
        lines = route_file.read_text(encoding="utf-8").splitlines()[1:]
    except OSError:
        return ""
    for line in lines:
        fields = line.split()
        if len(fields) < 4 or fields[1] != "00000000":
            continue
        try:
            flags = int(fields[3], 16)
        except ValueError:
            continue
        if flags & 0x2:
            return fields[0]
    return ""


def _is_wireless(interface: str) -> bool:
    return (Path("/sys/class/net") / interface / "wireless").is_dir()


def _usable_ipv4(value: str, *, private_only: bool) -> bool:
    try:
        address = IPv4Address(value)
    except ValueError:
        return False
    if address.is_loopback or address.is_link_local or address.is_multicast:
        return False
    if address.is_unspecified or address.is_reserved:
        return False
    return address.is_private if private_only else True


def select_robot_ipv4(
    *,
    interfaces: Iterable[str] | None = None,
    address_for: Callable[[str], str | None] = _interface_ipv4,
    wireless_for: Callable[[str], bool] = _is_wireless,
    default_interface: str | None = None,
) -> tuple[str, str]:
    """Return ``(interface, IPv4)`` with Wi-Fi and the default route preferred."""

    names = (
        [name for _, name in socket.if_nameindex()]
        if interfaces is None
        else list(interfaces)
    )
    default = (
        _default_route_interface()
        if default_interface is None
        else default_interface
    )
    wireless = [name for name in names if wireless_for(name)]
    physical = [
        name
        for name in names
        if not name.startswith(_VIRTUAL_PREFIXES) and name not in wireless
    ]
    ordered = []
    if default in wireless:
        ordered.append(default)
    ordered.extend(wireless)
    if default:
        ordered.append(default)
    ordered.extend(physical)

    unique = list(dict.fromkeys(ordered))
    candidates = [
        (name, address)
        for name in unique
        if (address := address_for(name)) is not None
    ]
    for private_only in (True, False):
        for name, address in candidates:
            if _usable_ipv4(address, private_only=private_only):
                return name, address
    raise RuntimeError(
        "Cannot determine the robot LAN IPv4 address. Connect Wi-Fi/Ethernet "
        "or pass robot_id:=... ros_domain_id:=... explicitly."
    )


def resolve_network_identity(
    requested_robot_id: str = "auto",
    requested_domain_id: str = "auto",
    *,
    selected_network: tuple[str, str] | None = None,
) -> NetworkIdentity:
    robot_id = str(requested_robot_id).strip()
    domain_raw = str(requested_domain_id).strip()
    needs_network = robot_id.lower() in _AUTO_VALUES or domain_raw.lower() in _AUTO_VALUES

    if selected_network is None:
        if needs_network:
            interface, ipv4 = select_robot_ipv4()
        else:
            interface, ipv4 = "explicit", "0.0.0.0"
    else:
        interface, ipv4 = selected_network

    try:
        last_octet = int(IPv4Address(ipv4).packed[-1])
    except ValueError as exc:
        raise RuntimeError(f"Invalid robot IPv4 address: {ipv4}") from exc

    if robot_id.lower() in _AUTO_VALUES:
        robot_id = f"robot{last_octet}"
    if _ROBOT_ID_PATTERN.fullmatch(robot_id) is None:
        raise RuntimeError(
            "ROBOT_ID must start with a letter and contain 2-64 letters, "
            "digits, underscores or hyphens"
        )

    if domain_raw.lower() in _AUTO_VALUES:
        domain_id = last_octet % 233
    else:
        try:
            domain_id = int(domain_raw)
        except ValueError as exc:
            raise RuntimeError("ROS_DOMAIN_ID must be an integer") from exc
    if not 0 <= domain_id <= 232:
        raise RuntimeError("ROS_DOMAIN_ID must be between 0 and 232")

    return NetworkIdentity(
        robot_id=robot_id,
        domain_id=domain_id,
        ipv4=ipv4,
        interface=interface,
    )
