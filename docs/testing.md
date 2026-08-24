# Testing

The root pytest suite checks Fleet Manager, Operator App, shared map contracts,
and robot-side pure Python logic. It does not start ROS nodes. One launch-path
contract imports ROS 2 launch libraries, so run the suite in the project's ROS
2 Jazzy environment.

## Prerequisites

ROS 2 Jazzy uses Python 3.12. Install the small set of runtime libraries that
the existing tests import:

```bash
sudo apt install \
  python3-grpcio \
  python3-protobuf \
  python3-pytest \
  python3-yaml
source /opt/ros/jazzy/setup.bash
```

The root `pytest.ini` limits discovery to `tests/test_*.py`. This is
intentional: build and install artifacts inside the three `robot` overlays are
ROS packages, not part of root pytest discovery.

## Fast feedback

Run the focused planning and traffic-scheduling suite while changing the
algorithms:

```bash
python3 -m pytest -q \
  tests/test_fleet_mapf_lm_cbs.py \
  tests/test_fleet_mapf_rolling_sipp.py \
  tests/test_corridor_scheduler.py \
  tests/test_cbs_components.py \
  tests/test_rolling_sipp_components.py \
  tests/test_traffic_routing_components.py \
  tests/test_motion_components.py \
  tests/test_robot_timed_route.py
```

For one file or one test:

```bash
python3 -m pytest -q tests/test_corridor_scheduler.py
python3 -m pytest -q tests/test_corridor_scheduler.py::test_name
```

## Full suite

From the repository root:

```bash
python3 -m pytest -q
```

To verify discovery without running tests:

```bash
python3 -m pytest --collect-only -q
```

`tests/browser_smoke_corridors.py` is a manual browser smoke test and is not
part of pytest discovery. It requires a running Operator App on port `8780`
and the `google-chrome` executable.

The suite also enforces dependency direction, declarative package
initializers, gRPC source parity for the standalone ROS package, atomic file
replacement, runtime-loop ownership, and component boundaries for the
planning algorithms.

For a behavior-preserving algorithm split, also run a seeded differential
harness against the committed implementation. Compare both the return value
and mutated state/call order. Benchmark control-tick, occupancy, A*, and other
hot paths separately; a recovery path that runs only after a timeout should be
reported in absolute microseconds as well as a ratio.

## Coverage

Install coverage separately, then measure the application packages:

```bash
sudo apt install python3-coverage
python3 -m coverage erase
python3 -m coverage run \
  --branch \
  --source=fleet_manager,operator_app \
  -m pytest -q
python3 -m coverage report --show-missing
```

For a local HTML report:

```bash
python3 -m coverage html
```
