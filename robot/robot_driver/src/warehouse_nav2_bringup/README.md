# Warehouse Nav2 bringup

This package owns only warehouse-specific Nav2 launch and configuration.

- `profile:=localization` starts map server, AMCL and their lifecycle managers.
- `profile:=navigation` additionally starts the future NavigateToPose stack.
- Nav2 velocity output is remapped to `motion/nav2_cmd_vel`; only
  `robot_motion_gateway` may publish the final `cmd_vel`.

The default robot bringup uses `localization`. The current LM/edge route
execution remains owned by `robot_planner`.

Nav2 and every ROS dependency outside `ros-jazzy-desktop` are built from
reviewed official sources, not from project-owned binary archives. Fetch the
sparse checkouts:

```bash
./robot/tools/fetch_ros_jazzy_sources.sh
```

The checkouts remain independent Git repositories under `robot/ros2_libs/src`.
`robot/ros2_libs/ros_jazzy_sources.repos` records every reviewed upstream commit.
With `ros-jazzy-desktop` and the explicitly listed Ubuntu build libraries
installed, build the local dependency closure required by this bringup:

```bash
source /opt/ros/jazzy/setup.bash
source robot/ros2_libs/install/local_setup.bash
cd robot/robot_driver
colcon build --packages-up-to warehouse_nav2_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```
