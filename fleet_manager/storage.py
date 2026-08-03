"""Compatibility import for durable file-writing helpers."""

from fleet_manager.core.io.atomic_files import (
    atomic_write_bytes,
    atomic_write_json,
    atomic_write_text,
    os,
)

__all__ = ["atomic_write_bytes", "atomic_write_json", "atomic_write_text"]
