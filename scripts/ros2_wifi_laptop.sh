#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/jazzy/setup.bash

if [ -f "${REPO_ROOT}/install/local_setup.bash" ]; then
  source "${REPO_ROOT}/install/local_setup.bash"
elif [ -f "${REPO_ROOT}/aivison_robot/install/local_setup.bash" ]; then
  source "${REPO_ROOT}/aivison_robot/install/local_setup.bash"
elif [ -f "${REPO_ROOT}/robot/install/local_setup.bash" ]; then
  source "${REPO_ROOT}/robot/install/local_setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset ROS_STATIC_PEERS
unset CYCLONEDDS_URI
unset ROS_LOCALHOST_ONLY

echo "ROS 2 Wi-Fi environment ready. Operator app will create CycloneDDS peer config from the Add Robot IP, default domain ${ROS_DOMAIN_ID}."
