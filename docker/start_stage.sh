#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd /root/stage_ws
source /root/stage_ws/install/setup.bash
colcon build

export DISPLAY=:99
Xvfb :99 -screen 0 1600x900x24 &
XVFB_PID=$!

x11vnc -display :99 -forever -shared -rfbport 5900 -nopw &
X11VNC_PID=$!

websockify --web /usr/share/novnc/ 8081 localhost:5900 &
NOVNC_PID=$!

cleanup() {
  kill "${NOVNC_PID}" 2>/dev/null || true
  kill "${X11VNC_PID}" 2>/dev/null || true
  kill "${XVFB_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 1

exec ros2 launch stage_ros2 stage.launch.py world:=us_office enforce_prefixes:=false one_tf_tree:=false
