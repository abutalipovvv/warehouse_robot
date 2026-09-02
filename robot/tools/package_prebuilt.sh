#!/usr/bin/env bash
set -euo pipefail

ROBOT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
  pwd
)"

# shellcheck disable=SC1091
source /etc/os-release
ARCH="$(uname -m)"
ROS_VERSION="${ROS_DISTRO:-jazzy}"
PLATFORM="${ID}-${VERSION_ID}-${ARCH}-ros-${ROS_VERSION}"
PREBUILT_DIR="${ROBOT_ROOT}/prebuilt/${PLATFORM}"

usage() {
  echo "Usage: $0 [extract] [--force]" >&2
  echo "       $0 create" >&2
}

require_common_tools() {
  if ! command -v zstd >/dev/null 2>&1; then
    echo "zstd is required for prebuilt overlays" >&2
    exit 1
  fi
}

verify_release_build() {
  local overlay="$1"
  local build_dir="${ROBOT_ROOT}/${overlay}/build"
  local cache
  local found=false

  while IFS= read -r -d '' cache; do
    found=true
    if grep -q '^CMAKE_BUILD_TYPE:STRING=' "${cache}" \
      && ! grep -qx 'CMAKE_BUILD_TYPE:STRING=Release' "${cache}"; then
      echo "Not a Release build: ${cache}" >&2
      echo "Build with: colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" >&2
      exit 1
    fi
  done < <(find "${build_dir}" -name CMakeCache.txt -type f -print0)

  if [[ "${found}" != true ]]; then
    echo "No CMake build cache found for ${overlay}: ${build_dir}" >&2
    exit 1
  fi
}

create_archives() {
  require_common_tools
  mkdir -p "${PREBUILT_DIR}"

  local overlay
  for overlay in ros2_libs simulation; do
    local install_dir="${ROBOT_ROOT}/${overlay}/install"
    if [[ ! -f "${install_dir}/local_setup.bash" ]]; then
      echo "Overlay is not built: ${install_dir}" >&2
      exit 1
    fi
    verify_release_build "${overlay}"

    local archive="${PREBUILT_DIR}/${overlay}-install.tar.zst"
    local temporary="${archive}.incoming"
    tar \
      --sort=name \
      --mtime="UTC 1970-01-01" \
      --owner=0 \
      --group=0 \
      --numeric-owner \
      -I "zstd -19 -T0" \
      -cf "${temporary}" \
      -C "${ROBOT_ROOT}/${overlay}" \
      install
    mv -- "${temporary}" "${archive}"
    printf '%s\n' "${install_dir}" \
      > "${PREBUILT_DIR}/${overlay}-build-prefix.txt"
  done

  (
    cd "${PREBUILT_DIR}"
    artifacts=(
      ros2_libs-install.tar.zst
      simulation-install.tar.zst
    )
    if [[ -f container-runtime.tar.zst ]]; then
      artifacts=(container-runtime.tar.zst "${artifacts[@]}")
    fi
    sha256sum "${artifacts[@]}" > SHA256SUMS
  )
  echo "Prebuilt Release overlays packaged in ${PREBUILT_DIR}"
}

extract_archives() {
  local force="$1"
  require_common_tools

  if [[ ! -d "${PREBUILT_DIR}" ]]; then
    echo "No prebuilt overlay for ${PLATFORM}." >&2
    exit 1
  fi
  if [[ ! -f "/opt/ros/${ROS_VERSION}/setup.bash" ]]; then
    echo "ROS ${ROS_VERSION} is required at /opt/ros/${ROS_VERSION}." >&2
    exit 1
  fi

  (
    cd "${PREBUILT_DIR}"
    sha256sum -c SHA256SUMS
  )

  local overlay
  if [[ "${force}" != true ]]; then
    for overlay in ros2_libs simulation; do
      local target="${ROBOT_ROOT}/${overlay}/install"
      if [[ -e "${target}" ]]; then
        echo "Refusing to replace existing ${target}; use --force." >&2
        exit 1
      fi
    done
  fi

  for overlay in ros2_libs simulation; do
    local archive="${PREBUILT_DIR}/${overlay}-install.tar.zst"
    local prefix_file="${PREBUILT_DIR}/${overlay}-build-prefix.txt"
    local target_parent="${ROBOT_ROOT}/${overlay}"
    local target="${target_parent}/install"
    local staging
    staging="$(mktemp -d "${target_parent}/.prebuilt-install.XXXXXX")"
    tar --zstd -xf "${archive}" -C "${staging}"
    python3 "${ROBOT_ROOT}/tools/relocate_prebuilt.py" \
      --root "${staging}/install" \
      --from-prefix "$(<"${prefix_file}")" \
      --to-prefix "${target}"

    if [[ -e "${target}" ]]; then
      local backup="${target_parent}/install.backup.$(date -u +%Y%m%dT%H%M%SZ)"
      mv -- "${target}" "${backup}"
      echo "Existing overlay moved to ${backup}"
    fi
    mv -- "${staging}/install" "${target}"
    rmdir -- "${staging}"
    echo "Extracted ${overlay} for ${PLATFORM}"
  done

  echo "Prebuilt overlays are ready. Build robot/robot_driver manually, then source robot/setup.bash."
}

command="${1:-extract}"
case "${command}" in
  create)
    if [[ $# -ne 1 ]]; then
      usage
      exit 2
    fi
    create_archives
    ;;
  extract)
    shift || true
    force=false
    if [[ "${1:-}" == "--force" && $# -eq 1 ]]; then
      force=true
    elif [[ $# -ne 0 ]]; then
      usage
      exit 2
    fi
    extract_archives "${force}"
    ;;
  --force)
    if [[ $# -ne 1 ]]; then
      usage
      exit 2
    fi
    extract_archives true
    ;;
  *)
    usage
    exit 2
    ;;
esac
