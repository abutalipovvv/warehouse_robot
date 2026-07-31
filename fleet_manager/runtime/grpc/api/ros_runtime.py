"""Composition root for the canonical ROS-backed robot runtime."""

from __future__ import annotations

# Preserve the historical module namespace for integrations importing these
# symbols directly from ``ros_runtime``.
import json
import math
import os
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from time import monotonic, sleep
from typing import Any

import yaml

from fleet_manager.core.route_core.map_loader import WarehouseMapLoader
from fleet_manager.core.route_core.models import WorldPoint
from fleet_manager.map_data.pgm import read_pgm_size
from fleet_manager.storage import atomic_write_bytes, atomic_write_text

from .contracts import (
    DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC,
)
from .ros_runtime_control import RosRuntimeControlMixin
from .ros_runtime_lifecycle import (
    RosRuntimeLifecycleMixin,
    _clean_node_suffix,
)
from .ros_runtime_maps import RosRuntimeMapTransferMixin
from .ros_runtime_params import (
    NAV2_RUNTIME_PARAMETERS,
    RosRuntimeParametersMixin,
)
from .ros_runtime_ros_helpers import RosRuntimeMessageServiceMixin
from .ros_runtime_slam import RosRuntimeSlamMixin


class RosRobotRuntime(
    RosRuntimeLifecycleMixin,
    RosRuntimeControlMixin,
    RosRuntimeMapTransferMixin,
    RosRuntimeSlamMixin,
    RosRuntimeParametersMixin,
    RosRuntimeMessageServiceMixin,
):
    """Compose ROS lifecycle, control, maps, SLAM and parameter capabilities."""


__all__ = [
    "NAV2_RUNTIME_PARAMETERS",
    "RosRobotRuntime",
]
