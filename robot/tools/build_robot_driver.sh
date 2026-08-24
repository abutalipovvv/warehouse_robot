#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
  pwd
)"
LIBS_SETUP="${ROBOT_ROOT}/ros2_libs/install/local_setup.bash"

if [[ ! -f "${LIBS_SETUP}" ]]; then
  echo "ros2_libs is not built. Run robot/tools/build_ros2_libs.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${LIBS_SETUP}"
set -u

cd "${ROBOT_ROOT}/robot_driver"
colcon build --symlink-install "$@"
