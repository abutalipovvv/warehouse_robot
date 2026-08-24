"""Composition root for the independently deployed ROS robot runtime."""

from __future__ import annotations

# Retain the historical module namespace for standalone integrations.
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
from robot_planner.route_core import WarehouseMapLoader, WorldPoint
from robot_planner.route_core.atomic_storage import (
    atomic_write_bytes,
    atomic_write_text,
)
from robot_planner.route_core.pgm import read_pgm_size

from .contracts import (
    DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC,
)
from .ros_runtime_control import RosRuntimeControlMixin
from .ros_runtime_lifecycle import (
    STATUS_STALE_TIMEOUT_SEC,
    RosRuntimeLifecycleMixin,
    _clean_node_suffix,
)
from .ros_runtime_maps import RosRuntimeMapTransferMixin
from .ros_runtime_nav2_lifecycle import (
    NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES,
    NAV2_LIFECYCLE_MANAGER_SERVICES,
    NAV2_LIFECYCLE_NODES,
    NAV2_LIFECYCLE_RESUME_NODES,
    RosRuntimeNav2LifecycleMixin,
)
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
    RosRuntimeNav2LifecycleMixin,
    RosRuntimeParametersMixin,
    RosRuntimeMessageServiceMixin,
):
    """Compose standalone lifecycle, control, maps, SLAM and Nav2 policy."""


__all__ = [
    "NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES",
    "NAV2_LIFECYCLE_MANAGER_SERVICES",
    "NAV2_LIFECYCLE_NODES",
    "NAV2_LIFECYCLE_RESUME_NODES",
    "NAV2_RUNTIME_PARAMETERS",
    "RosRobotRuntime",
    "STATUS_STALE_TIMEOUT_SEC",
]
