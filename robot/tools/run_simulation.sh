#!/usr/bin/env bash
set -euo pipefail

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
  pwd
)"
SETUP_FILE="${ROBOT_ROOT}/setup.bash"
SIMULATION_SETUP="${ROBOT_ROOT}/simulation/install/local_setup.bash"

if [[ ! -f "${SIMULATION_SETUP}" ]]; then
  echo "Stage overlay is missing: ${SIMULATION_SETUP}" >&2
  echo "Run robot/tools/restore_prebuilt.sh or robot/tools/build_simulation.sh." >&2
  exit 1
fi

GUI_ENABLED=true
for argument in "$@"; do
  case "${argument}" in
    stage_enable_gui:=false|stage_enable_gui:=False|stage_enable_gui:=0)
      GUI_ENABLED=false
      ;;
  esac
done
unset argument

if [[ "${GUI_ENABLED}" == true && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "No graphical session detected (DISPLAY and WAYLAND_DISPLAY are empty)." >&2
  echo "Run from a desktop terminal, or pass stage_enable_gui:=false." >&2
  exit 1
fi
unset GUI_ENABLED

# shellcheck disable=SC1090
set +u
source "${SETUP_FILE}"
set -u

exec ros2 launch stage_ros2 simulation.launch.py "$@"
