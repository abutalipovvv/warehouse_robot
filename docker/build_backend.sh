#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd /root/stage_ws
source /root/stage_ws/install/setup.bash

export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 &
XVFB_PID=$!

cleanup() {
  kill "${XVFB_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 1

exec ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=false
