# real_robot

ROS 2 driver for AIvison Robokit mobile robots.

The driver talks to the robot over the Robokit TCP/IP API and bridges it into ROS 2:

- subscribes `/cmd_vel` as `geometry_msgs/Twist` and sends API `2010`
- subscribes `/go_to_lm` as `std_msgs/String` and sends API `3051`
- publishes `/robot_status` as `robot_msgs/RobotStatus`

Pose, velocity, battery, robot status, and navigation state are read with API `1100` at 10 Hz by default
and published in `/robot_status`.

## Build

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_msgs real_robot
source install/local_setup.bash
```

## Run

```bash
ros2 launch real_robot robot_driver.launch.py robot_ip:=192.168.192.5 robot_id:=robot1
```

If the robot requires dispatch control rights before motion/navigation:

```bash
ros2 launch real_robot robot_driver.launch.py \
  robot_ip:=192.168.192.5 \
  acquire_control_on_start:=true \
  acquire_control_before_command:=true
```

Send a one-shot landmark navigation command:

```bash
ros2 topic pub --once /go_to_lm std_msgs/msg/String "{data: 'LM105'}"
```

You can also publish JSON to override `source_id`, `task_id`, or optional Robokit `3051` fields:

```bash
ros2 topic pub --once /go_to_lm std_msgs/msg/String \
  "{data: '{\"id\":\"LM105\",\"source_id\":\"SELF_POSITION\",\"max_speed\":0.4}'}"
```

Pose navigation can use the same topic by sending Robokit `3051` with the built-in
`syspy/goPath.py` script:

```bash
ros2 topic pub --once /go_to_lm std_msgs/msg/String \
  "{data: '{\"id\":\"SELF_POSITION\",\"source_id\":\"SELF_POSITION\",\"operation\":\"Script\",\"script_name\":\"syspy/goPath.py\",\"script_args\":{\"x\":1.0,\"y\":2.0,\"theta\":0.0,\"coordinate\":\"world\"}}'}"
```
