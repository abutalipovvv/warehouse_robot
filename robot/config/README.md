# Physical robot identity

By default, both real and simulation launch files derive a robot identity from
the active Wi-Fi IPv4 address (or the default-route Ethernet address). The last
octet becomes both the robot suffix and DDS domain: `192.168.1.6` becomes
`ROBOT_ID=robot6` and `ROS_DOMAIN_ID=6`.

Use a persistent identity file only when the network-derived value is not
suitable. Copy `robot.env.example` to `/etc/warehouse-robot/robot.env`, assign a
fleet-unique `ROBOT_ID` and `ROS_DOMAIN_ID`, and use an underscore-only
`ROS_NAMESPACE`.

Use `cyclonedds.localhost.xml` when all ROS nodes and sensor drivers run on one
computer. Use `cyclonedds.robot_lan.xml` when DDS must connect several
computers inside the robot; pin its `NetworkInterface` during provisioning.

The Fleet Manager communicates with robots over gRPC. DDS is intentionally
local to each robot and must not be used as the fleet transport.

Validate a provisioned file before starting the service:

```bash
python3 robot/tools/validate_robot_identity.py \
  --env-file /etc/warehouse-robot/robot.env
```

Explicit environment values or `robot_id:=... ros_domain_id:=...` launch
arguments override automatic detection.
