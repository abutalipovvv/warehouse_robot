#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
  pwd
)"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

cd "${ROBOT_ROOT}/ros2_libs"
mapfile -t LOCAL_PACKAGES < <(colcon list --names-only)

# ros2_libs intentionally overrides selected packages shipped by ROS Desktop.
# Declaring that policy keeps local Jazzy sources authoritative and prevents the
# colcon override check from becoming a build error in a future release.
colcon build \
  --symlink-install \
  --allow-overriding "${LOCAL_PACKAGES[@]}" \
  "$@"
