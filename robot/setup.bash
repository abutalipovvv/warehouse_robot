#!/usr/bin/env bash

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
  pwd
)"

source /opt/ros/jazzy/setup.bash
source "${ROBOT_ROOT}/ros2_libs/install/local_setup.bash"
source "${ROBOT_ROOT}/robot_driver/install/local_setup.bash"

if [[ -f "${ROBOT_ROOT}/simulation/install/local_setup.bash" ]]; then
  source "${ROBOT_ROOT}/simulation/install/local_setup.bash"
fi

unset ROBOT_ROOT
