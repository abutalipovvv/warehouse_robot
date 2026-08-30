#!/usr/bin/env bash
set -euo pipefail

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
  pwd
)"
FORCE=false

if [[ "${1:-}" == "--force" ]]; then
  FORCE=true
elif [[ -n "${1:-}" ]]; then
  echo "Usage: $0 [--force]" >&2
  exit 2
fi

# shellcheck disable=SC1091
source /etc/os-release
ARCH="$(uname -m)"
ROS_VERSION="${ROS_DISTRO:-jazzy}"
PLATFORM="${ID}-${VERSION_ID}-${ARCH}-ros-${ROS_VERSION}"
PREBUILT_DIR="${ROBOT_ROOT}/prebuilt/${PLATFORM}"

if [[ ! -d "${PREBUILT_DIR}" ]]; then
  echo "No prebuilt overlay for ${PLATFORM}." >&2
  echo "Build locally or add a platform-specific bundle under robot/prebuilt/." >&2
  exit 1
fi
if [[ ! -f "/opt/ros/${ROS_VERSION}/setup.bash" ]]; then
  echo "ROS ${ROS_VERSION} is required at /opt/ros/${ROS_VERSION}." >&2
  exit 1
fi
if ! command -v zstd >/dev/null 2>&1; then
  echo "zstd is required to restore prebuilt overlays" >&2
  exit 1
fi

(
  cd "${PREBUILT_DIR}"
  sha256sum -c SHA256SUMS
)

if [[ "${FORCE}" != true ]]; then
  for overlay in ros2_libs simulation; do
    target="${ROBOT_ROOT}/${overlay}/install"
    if [[ -e "${target}" ]]; then
      echo "Refusing to replace existing ${target}; use --force if intended." >&2
      exit 1
    fi
  done
fi

for overlay in ros2_libs simulation; do
  archive="${PREBUILT_DIR}/${overlay}-install.tar.zst"
  prefix_file="${PREBUILT_DIR}/${overlay}-build-prefix.txt"
  target_parent="${ROBOT_ROOT}/${overlay}"
  target="${target_parent}/install"
  staging="$(mktemp -d "${target_parent}/.prebuilt-install.XXXXXX")"
  tar --zstd -xf "${archive}" -C "${staging}"
  python3 "${ROBOT_ROOT}/tools/relocate_prebuilt.py" \
    --root "${staging}/install" \
    --from-prefix "$(<"${prefix_file}")" \
    --to-prefix "${target}"

  if [[ -e "${target}" ]]; then
    backup="${target_parent}/install.backup.$(date -u +%Y%m%dT%H%M%SZ)"
    mv -- "${target}" "${backup}"
    echo "Existing overlay moved to ${backup}"
  fi
  mv -- "${staging}/install" "${target}"
  rmdir -- "${staging}"
  echo "Restored ${overlay} for ${PLATFORM}"
done

echo "Prebuilt overlays are ready. Build robot_driver, then source robot/setup.bash."
