from __future__ import annotations

from pathlib import Path

import pytest

from fleet_manager.core.mapping.formats.pgm import PgmImage, read_pgm_size


@pytest.mark.parametrize("separator", [b"\n", b"\r\n", b" "])
def test_binary_parser_preserves_a_whitespace_valued_first_pixel(
    separator: bytes,
) -> None:
    pixels = b"\x0a\x20\x00\xfe"
    raw = b"P5\n2 2\n255" + separator + pixels

    image = PgmImage.from_bytes(raw)

    assert (image.width, image.height, image.maximum) == (2, 2, 255)
    assert image.pixels == pixels
    assert image.encoding == "P5"


def test_header_comments_are_supported(tmp_path: Path) -> None:
    path = tmp_path / "map.pgm"
    path.write_bytes(b"P5\n# generated map\n2 1\n255\n\x00\xfe")

    assert read_pgm_size(path) == (2, 1)
    assert PgmImage.read(path).pixels == b"\x00\xfe"


def test_ascii_parser_ignores_comments_and_normalizes_maximum() -> None:
    image = PgmImage.from_bytes(
        b"P2\n2 2\n10\n0 5 # middle gray\n10 1\n"
    )

    assert image.encoding == "P2"
    assert image.maximum == 255
    assert image.pixels == bytes([0, 128, 255, 26])


@pytest.mark.parametrize(
    ("raw", "message"),
    [
        (b"P9\n1 1\n255\n\x00", "unsupported format"),
        (b"P5\n0 1\n255\n", "must be positive"),
        (b"P5\n1 1\n256\n\x00\x00", "only 8-bit"),
        (b"P5\n2 1\n255\n\x00", "shorter than expected"),
        (b"P2\n1 1\n10\n11", "outside 0..10"),
    ],
)
def test_invalid_images_have_clear_errors(raw: bytes, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        PgmImage.from_bytes(raw)
