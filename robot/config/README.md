# Physical robot identity

Each physical robot must have one persistent identity file. Copy
`robot.env.example` to `/etc/warehouse-robot/robot.env`, assign a fleet-unique
`ROBOT_ID` and `ROS_DOMAIN_ID`, and use an underscore-only `ROS_NAMESPACE`.

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

Simulation does not set `WAREHOUSE_REQUIRE_IDENTITY`; it keeps one DDS domain
and separates simulated robots using ROS namespaces and frame prefixes.
