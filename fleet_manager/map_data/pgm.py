"""Portable Graymap parsing shared by map loading and SMAP conversion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class PgmImage:
    """Validated 8-bit grayscale image decoded from P5 or P2 PGM."""

    width: int
    height: int
    maximum: int
    pixels: bytes
    encoding: str

    @classmethod
    def read(cls, path: Path) -> "PgmImage":
        source = Path(path)
        try:
            return cls.from_bytes(source.read_bytes())
        except ValueError as error:
            raise ValueError(f"Invalid PGM in {source}: {error}") from error

    @classmethod
    def from_bytes(cls, data: bytes) -> "PgmImage":
        magic, index = _next_token(data, 0)
        width_token, index = _next_token(data, index)
        height_token, index = _next_token(data, index)
        maximum_token, index = _next_token(data, index)
        if magic not in {b"P5", b"P2"}:
            raise ValueError(f"unsupported format {magic!r}")
        try:
            width = int(width_token)
            height = int(height_token)
            maximum = int(maximum_token)
        except ValueError as error:
            raise ValueError("width, height and maximum must be integers") from error
        if width <= 0 or height <= 0:
            raise ValueError("width and height must be positive")
        if maximum <= 0:
            raise ValueError("maximum must be positive")

        pixel_count = width * height
        if magic == b"P5":
            if maximum > 255:
                raise ValueError("only 8-bit binary PGM is supported")
            # Pixel bytes may themselves be ASCII whitespace. Locating the
            # payload after exactly one header separator avoids consuming a
            # valid first pixel while accepting both LF and CRLF.
            payload_start = _binary_payload_start(data, index)
            payload_end = payload_start + pixel_count
            if payload_end > len(data):
                raise ValueError("binary pixel data is shorter than expected")
            pixels = data[payload_start:payload_end]
            return cls(
                width=width,
                height=height,
                maximum=maximum,
                pixels=pixels,
                encoding="P5",
            )

        samples: list[int] = []
        while len(samples) < pixel_count:
            token, index = _next_token(data, index)
            if not token:
                break
            try:
                sample = int(token)
            except ValueError as error:
                raise ValueError(f"invalid ASCII sample {token!r}") from error
            if not 0 <= sample <= maximum:
                raise ValueError(
                    f"ASCII sample {sample} outside 0..{maximum}"
                )
            samples.append(sample)
        if len(samples) != pixel_count:
            raise ValueError("ASCII pixel data is shorter than expected")
        scale = 255.0 / maximum
        return cls(
            width=width,
            height=height,
            maximum=255,
            pixels=bytes(round(sample * scale) for sample in samples),
            encoding="P2",
        )

    def binary_bytes(self) -> bytes:
        """Serialize normalized pixels as a conventional P5 image."""

        return (
            f"P5\n{self.width} {self.height}\n{self.maximum}\n".encode(
                "ascii"
            )
            + self.pixels
        )


def read_pgm_size(path: Path) -> tuple[int, int]:
    """Read validated dimensions; kept explicit for ROS map capture paths."""

    image = PgmImage.read(path)
    return image.width, image.height


def _next_token(data: bytes, index: int) -> tuple[bytes, int]:
    length = len(data)
    while index < length:
        value = data[index]
        if value == ord("#"):
            while index < length and data[index] not in {10, 13}:
                index += 1
        elif chr(value).isspace():
            index += 1
        else:
            break
    start = index
    while index < length:
        value = data[index]
        if value == ord("#") or chr(value).isspace():
            break
        index += 1
    return data[start:index], index


def _binary_payload_start(data: bytes, index: int) -> int:
    if index >= len(data) or not chr(data[index]).isspace():
        raise ValueError("binary header is missing its pixel separator")
    if data[index:index + 2] == b"\r\n":
        return index + 2
    return index + 1


__all__ = ["PgmImage", "read_pgm_size"]
