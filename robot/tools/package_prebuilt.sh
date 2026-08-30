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
OUTPUT_DIR="${ROBOT_ROOT}/prebuilt/${PLATFORM}"

if ! command -v zstd >/dev/null 2>&1; then
  echo "zstd is required to package prebuilt overlays" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

for overlay in ros2_libs simulation; do
  install_dir="${ROBOT_ROOT}/${overlay}/install"
  if [[ ! -f "${install_dir}/local_setup.bash" ]]; then
    echo "Overlay is not built: ${install_dir}" >&2
    exit 1
  fi
  archive="${OUTPUT_DIR}/${overlay}-install.tar.zst"
  temporary="${archive}.incoming"
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
  printf '%s\n' "${install_dir}" > "${OUTPUT_DIR}/${overlay}-build-prefix.txt"
done

(
  cd "${OUTPUT_DIR}"
  sha256sum \
    ros2_libs-install.tar.zst \
    simulation-install.tar.zst \
    > SHA256SUMS
)

echo "Prebuilt overlays packaged in ${OUTPUT_DIR}"
