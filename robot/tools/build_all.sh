#!/usr/bin/env bash
set -euo pipefail

TOOLS_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
  pwd
)"

"${TOOLS_ROOT}/build_ros2_libs.sh" "$@"
"${TOOLS_ROOT}/build_robot_driver.sh" "$@"
"${TOOLS_ROOT}/build_simulation.sh" "$@"
