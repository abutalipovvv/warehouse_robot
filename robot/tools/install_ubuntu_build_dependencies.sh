#!/usr/bin/env bash
set -euo pipefail

# ROS packages outside ros-jazzy-desktop are vendored in robot/ros2_libs/src.
# Only non-ROS Ubuntu build dependencies belong here.
UBUNTU_PACKAGES=(
  clang-format
  libbenchmark-dev
  libceres-dev
  libdw-dev
  libfltk1.3-dev
  libnanoflann-dev
  libomp-dev
  libxsimd-dev
  libxtensor-dev
  libzmq3-dev
  python3-grpcio
)

sudo apt-get update
sudo apt-get install --yes "${UBUNTU_PACKAGES[@]}"
