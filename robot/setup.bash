#!/usr/bin/env bash

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
  pwd
)"

for required_setup in \
  "${ROBOT_ROOT}/ros2_libs/install/local_setup.bash" \
  "${ROBOT_ROOT}/robot_driver/install/local_setup.bash"; do
  if [[ ! -f "${required_setup}" ]]; then
    echo "Required robot overlay is missing: ${required_setup}" >&2
    echo "Run robot/tools/package_prebuilt.sh, then build robot/robot_driver with colcon." >&2
    unset ROBOT_ROOT required_setup
    return 1
  fi
done
unset required_setup

source /opt/ros/jazzy/setup.bash
source "${ROBOT_ROOT}/ros2_libs/install/local_setup.bash"
source "${ROBOT_ROOT}/robot_driver/install/local_setup.bash"

if [[ -f "${ROBOT_ROOT}/simulation/install/local_setup.bash" ]]; then
  source "${ROBOT_ROOT}/simulation/install/local_setup.bash"
fi

unset ROBOT_ROOT
