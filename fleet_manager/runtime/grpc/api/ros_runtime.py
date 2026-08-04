"""Composition root for the canonical ROS-backed robot runtime."""

from __future__ import annotations

from .ros_runtime_control import RosRuntimeControlMixin
from .ros_runtime_lifecycle import RosRuntimeLifecycleMixin
from .ros_runtime_maps import RosRuntimeMapTransferMixin
from .ros_runtime_params import RosRuntimeParametersMixin
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
