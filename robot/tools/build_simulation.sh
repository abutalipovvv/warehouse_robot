#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
  pwd
)"
LIBS_SETUP="${ROBOT_ROOT}/ros2_libs/install/local_setup.bash"
DRIVER_SETUP="${ROBOT_ROOT}/robot_driver/install/local_setup.bash"

for setup_file in "${LIBS_SETUP}" "${DRIVER_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Required overlay is not built: ${setup_file}" >&2
    exit 1
  fi
done

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${LIBS_SETUP}"
# shellcheck disable=SC1090
source "${DRIVER_SETUP}"
set -u

cd "${ROBOT_ROOT}/simulation"
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "$@"
