#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
  pwd
)"
ROS_SOURCE_ROOT="${PROJECT_ROOT}/robot/ros2_libs/src"

NAV2_PACKAGES=(
  nav2_dwb_controller/nav_2d_msgs
  nav2_dwb_controller/nav_2d_utils
  nav2_amcl
  nav2_behavior_tree
  nav2_behaviors
  nav2_bt_navigator
  nav2_common
  nav2_controller
  nav2_core
  nav2_costmap_2d
  nav2_lifecycle_manager
  nav2_map_server
  nav2_mppi_controller
  nav2_msgs
  nav2_navfn_planner
  nav2_planner
  nav2_route
  nav2_rviz_plugins
  nav2_smoother
  nav2_util
  nav2_voxel_grid
)

AMENT_LINT_PACKAGES=(
  ament_clang_format
  ament_cmake_clang_format
  ament_pycodestyle
  ament_cmake_pycodestyle
)

BOND_PACKAGES=(
  bond
  bondcpp
  smclib
)

prepare_checkout() {
  local source_dir="$1"
  local repository="$2"
  local branch="$3"
  local expected_commit="$4"
  shift 4
  local packages=("$@")
  local checkout_created=false

  if [[ -e "${source_dir}" ]] && [[ ! -d "${source_dir}/.git" ]]; then
    echo "Vendored source already exists: ${source_dir}"
    return
  fi

  if [[ -d "${source_dir}/.git" ]]; then
    local actual_repository
    actual_repository="$(git -C "${source_dir}" remote get-url origin)"
    if [[ "${actual_repository}" != "${repository}" ]]; then
      echo "Unexpected origin for ${source_dir}: ${actual_repository}" >&2
      exit 1
    fi
    echo "Source already exists: ${source_dir}"
  else
    local clone_options=(
      --depth 1
      --filter=blob:none
      --branch "${branch}"
      --no-checkout
      --single-branch
    )
    if (( ${#packages[@]} > 0 )); then
      clone_options+=(--sparse)
    fi
    git clone "${clone_options[@]}" "${repository}" "${source_dir}"
    checkout_created=true

    if ! git -C "${source_dir}" cat-file -e "${expected_commit}^{commit}"; then
      git -C "${source_dir}" fetch --depth 1 origin "${expected_commit}"
    fi
  fi

  if (( ${#packages[@]} > 0 )); then
    git -C "${source_dir}" sparse-checkout set "${packages[@]}"
  fi

  if [[ "${checkout_created}" == true ]]; then
    git -C "${source_dir}" checkout --detach "${expected_commit}"
  fi

  local actual_commit
  actual_commit="$(git -C "${source_dir}" rev-parse HEAD)"
  if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    echo "Source checkout does not match the reviewed commit: ${repository}" >&2
    echo "Expected: ${expected_commit}" >&2
    echo "Actual:   ${actual_commit}" >&2
    echo "Review upstream changes and update ros_jazzy_sources.repos." >&2
    exit 1
  fi

  echo "Ready: ${source_dir} (${actual_commit})"
}

prepare_checkout \
  "${ROS_SOURCE_ROOT}/navigation2" \
  "https://github.com/ros-navigation/navigation2.git" \
  "jazzy" \
  "f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501" \
  "${NAV2_PACKAGES[@]}"

prepare_checkout \
  "${ROS_SOURCE_ROOT}/geographic_info" \
  "https://github.com/ros-geographic-info/geographic_info.git" \
  "ros2" \
  "f70b81a438172cd7a066dc1b18314d70e0eb6389" \
  geographic_msgs

prepare_checkout \
  "${ROS_SOURCE_ROOT}/rcl_interfaces" \
  "https://github.com/ros2/rcl_interfaces.git" \
  "jazzy" \
  "7aa3caf43377ea6ad615bc1040832e2c7566bfbe" \
  test_msgs

prepare_checkout \
  "${ROS_SOURCE_ROOT}/test_interface_files" \
  "https://github.com/ros2/test_interface_files.git" \
  "jazzy" \
  "1c92e082fe14f90b772d308d113e510ad2c79fb9"

prepare_checkout \
  "${ROS_SOURCE_ROOT}/ament_lint" \
  "https://github.com/ament/ament_lint.git" \
  "jazzy" \
  "9121ac787058642730206a52daba614f9c86b0fb" \
  "${AMENT_LINT_PACKAGES[@]}"

prepare_checkout \
  "${ROS_SOURCE_ROOT}/backward_ros" \
  "https://github.com/pal-robotics/backward_ros.git" \
  "foxy-devel" \
  "3c71c9f184223f885459ad67a223eb36a5ca347f"

prepare_checkout \
  "${ROS_SOURCE_ROOT}/behaviortree_cpp" \
  "https://github.com/BehaviorTree/BehaviorTree.CPP.git" \
  "master" \
  "c88a9f429a421b312599a07fa8902524b09bf90a"

prepare_checkout \
  "${ROS_SOURCE_ROOT}/bond_core" \
  "https://github.com/ros/bond_core.git" \
  "ros2" \
  "44cc20e678f80f8279d2344045e2193540f3988c" \
  "${BOND_PACKAGES[@]}"

prepare_checkout \
  "${ROS_SOURCE_ROOT}/diagnostics" \
  "https://github.com/ros/diagnostics.git" \
  "ros2-jazzy" \
  "ed64dc9c3b9d048b1699cf81692e6b0e3926c3ac" \
  diagnostic_updater

prepare_checkout \
  "${ROS_SOURCE_ROOT}/xacro" \
  "https://github.com/ros/xacro.git" \
  "ros2" \
  "da4b3849f8320903d625250089f67f0632be86f2"

echo "ROS 2 Jazzy source dependencies are ready."
