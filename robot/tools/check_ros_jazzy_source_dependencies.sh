#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
  pwd
)"
ROBOT_ROOT="${PROJECT_ROOT}/robot"
SOURCE_ROOTS=(
  "${ROBOT_ROOT}/ros2_libs/src"
  "${ROBOT_ROOT}/robot_driver/src"
  "${ROBOT_ROOT}/simulation/src"
)

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed under /opt/ros/jazzy." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

declare -A desktop_packages=()
declare -A source_packages=()

while IFS= read -r package; do
  [[ -n "${package}" ]] && desktop_packages["${package}"]=1
done < <(
  {
    echo ros-jazzy-desktop
    apt-cache depends --recurse --no-recommends ros-jazzy-desktop 2>/dev/null |
      sed -n 's/^[[:space:]]*Depends: \([^ <][^ ]*\).*/\1/p'
  } | sort -u
)

while IFS= read -r package; do
  [[ -n "${package}" ]] && source_packages["${package}"]=1
done < <(colcon list --base-paths "${SOURCE_ROOTS[@]}" --names-only)

missing_dependencies=()
unresolved_keys=()

while IFS= read -r rosdep_key; do
  [[ -n "${source_packages[$rosdep_key]+x}" ]] && continue

  if ! resolution="$(rosdep resolve --rosdistro jazzy "${rosdep_key}" 2>/dev/null)"; then
    unresolved_keys+=("${rosdep_key}")
    continue
  fi

  while IFS= read -r binary_package; do
    [[ -z "${binary_package}" ]] && continue
    [[ "${binary_package}" != ros-jazzy-* ]] && continue
    [[ -n "${desktop_packages[$binary_package]+x}" ]] && continue
    missing_dependencies+=("${rosdep_key} -> ${binary_package}")
  done < <(printf '%s\n' "${resolution}" | awk '!/^#/ && NF {print $1}')
done < <(
  rosdep keys \
    --from-paths "${SOURCE_ROOTS[@]}" \
    --rosdistro jazzy 2>/dev/null | sort -u
)

if (( ${#unresolved_keys[@]} > 0 )); then
  printf 'Unresolved rosdep keys:\n' >&2
  printf '  %s\n' "${unresolved_keys[@]}" | sort -u >&2
fi

if (( ${#missing_dependencies[@]} > 0 )); then
  printf 'ROS packages missing from ros-jazzy-desktop and local src:\n' >&2
  printf '  %s\n' "${missing_dependencies[@]}" | sort -u >&2
fi

if (( ${#unresolved_keys[@]} > 0 || ${#missing_dependencies[@]} > 0 )); then
  exit 1
fi

printf 'Dependency audit passed: %d ROS packages are available from source or ros-jazzy-desktop.\n' \
  "${#source_packages[@]}"
