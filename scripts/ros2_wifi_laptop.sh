#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/jazzy/setup.bash

if [ -f "${REPO_ROOT}/install/local_setup.bash" ]; then
  source "${REPO_ROOT}/install/local_setup.bash"
elif [ -f "${REPO_ROOT}/robot/install/local_setup.bash" ]; then
  source "${REPO_ROOT}/robot/install/local_setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
export ROS_STATIC_PEERS=192.168.11.189
export CYCLONEDDS_URI="file://${REPO_ROOT}/config/dds/cyclonedds_laptop_wifi.xml"
unset ROS_LOCALHOST_ONLY

echo "ROS 2 Wi-Fi DDS profile: laptop wlp3s0 -> Jetson 192.168.11.189, domain ${ROS_DOMAIN_ID}"
