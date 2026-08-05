# Fleet Manager package boundaries

Fleet Manager uses four code layers. Imports point downward: runtime uses
manager, manager uses robot and core, and robot uses core. A lower layer must
not import a higher layer.

```text
runtime
   |
   v
manager ------> robot
   |              |
   +--------------+
   v
 core
```

Architecture tests in `tests/test_fleet_architecture.py` enforce these import
rules. They also reject wildcard imports, `__all__`, re-exporting package
initializers and removed proxy paths.

## Core

`fleet_manager/core` is deterministic computation with no live manager or
runtime objects:

- `math` contains geometry, curves, polygons and intervals;
- `search` contains the shared deterministic A* implementation;
- `mapping` contains map models, SMAP/PGM conversion and graph navigation;
- `mapf` contains traffic graph construction, reservations, SIPP, Rolling
  SIPP, local CBS and the fleet planner;
- `traffic` contains collision calculations, controlled-corridor geometry and
  scheduling, and the pure wait-for graph.

MAPF remains a top-level core subsystem because it has its own models and
several cooperating algorithms. It is not hidden under a generic
`algorithms` directory. New pure algorithms belong in the narrowest existing
core package and receive ordinary input values rather than `FleetManagerCore`.

The old `core/algorithms` and `core/io` directories no longer exist. Shared
atomic file operations live in `fleet_manager/storage.py` because maps, runtime
parameter writers and the operator workspace all use them.

The gRPC `contracts.py` module keeps five endpoint names required by the
independently deployed robot client's byte-parity contract. This is the only
documented internal re-export exception; `tests/test_grpc_source_parity.py`
guards the external deployment requirement. Ordinary Fleet Manager modules use
the owning module directly.

## Robot

`fleet_manager/robot/model.py` owns the transport-neutral `FleetRobot` domain
model. It describes pose, connection, route, movement and traffic-wait state,
but it does not know about manager services or runtime adapters.

Fleet movement mixins currently remain in `manager/movement`. They coordinate
orders, shared reservations, traffic recovery and the complete fleet, so
placing them in the lower robot layer would create a reverse dependency. Pure
robot-specific calculations can move to `robot` later when their inputs no
longer require live manager state.

## Manager

`fleet_manager/manager` owns orchestration and all live mutable state:

- `manager.py` is the composition root;
- `state.py` contains `FleetState`, `TrafficState`, `PlanningState`,
  `RecoveryState`, `RevisionClock` and snapshot creation;
- `planning.py` contains immutable planning models, solver boundary and atomic
  commit;
- `scheduler.py` contains the bounded priority worker;
- `tasks` contains order admission, dispatch, rolling continuation,
  replanning, lifecycle and recovery;
- `coordination` contains live corridor/zone admission, reservations,
  deadlock recovery and planning preparation;
- `ports.py` describes robot gateway operations needed by manager policy.

`OrderAdmissionService`, `RollingContinuationService` and
`ReplanningService` receive explicit state containers and callables. They do
not receive the complete manager. Admission validation and deterministic robot
selection, rolling candidate selection/request construction, and replan
transaction/retry recording now live in these services. Existing mixins still
compose older traffic and motion policy; migrated entry points are short
adapters and do not retain a second implementation.

`manager/coordination` is shallow by domain:

```text
coordination/
    coordinator.py
    runtime_conflicts.py
    zones.py
    corridors/     # admission, requests, prefetch, intent, validation
    deadlocks/     # detection, priority, recovery, evacuation
    planning/      # preparation, continuous waits, reservations, results
    routing/       # spatial and rolling route helpers
```

The corridor and deadlock subpackages no longer contain nested `admission`,
`prefetch`, `arbitration`, `cycles` or `evacuation` packages. Small value
objects are grouped in one `models.py` per domain. The three high-level mixins
remain temporary composition boundaries; lower composition-only mixins were
removed.

Map parsing and serialization are pure core operations. Active-map changes are
currently orchestrated by manager snapshot/update methods and the Operator
Fleet map service. A separate `manager/mapping` package is intentionally not
created until there are multiple manager-owned map lifecycle components.

## Runtime

`fleet_manager/runtime` owns execution infrastructure:

- `loop.py` owns the single-writer runtime thread;
- `application` starts and stops the process;
- `gateways` implements manager ports for simulation and gRPC;
- `grpc` contains protobuf, client/server and ROS2 integration;
- `simulation` selects the in-memory robot gateway.

Manager code imports only gateway protocols from `manager/ports.py`. To add a
new robot transport, implement those operations under `runtime/gateways` and
select the implementation in a runtime composition module. Do not add concrete
transport checks to manager policy.

## Planning transaction

```text
RuntimeLoop
    |
    | PlanningSnapshot(revision=N)
    v
PlanningWorker.submit_job
    |
    v
Rolling SIPP / traffic planner
    |
    | cooperative cancellation and deadline checks
    |
    | local CBS only for a coupled fallback
    v
PlanCandidate(expected_revision=N)
    |
    v
RuntimeLoop
    |
    +-- current revision == N --> validate --> atomic commit
    |
    +-- current revision != N --> mark stale --> discard/replan
```

Only the runtime owner changes live robots, orders, routes, reservations or
leases. The worker receives an immutable snapshot, runs one solver job and
publishes a candidate. Its only submission API is the typed `submit_job`; it
has no reference to `FleetManagerCore`.

`PlanCommitService` checks revision before validation and again before taking
the rollback checkpoint. A failed apply restores the checkpoint. A stale
candidate changes neither routes nor reservations.

## Adding behavior

- Add math, search, mapping, MAPF or traffic calculations to `core` when they
  can be tested from plain values.
- Add robot-only state or validation to `robot` when it needs no fleet-wide
  mutable state.
- Add task, traffic or recovery decisions to a focused manager service with
  explicit constructor dependencies.
- Keep public manager methods small: validate the command, call a service and
  apply its result on the runtime owner thread.
- Add gRPC, simulation or ROS2 details only under `runtime`.

These boundaries do not change the system design: Fleet Manager remains
centralized, Nav2 stays on each robot, Rolling SIPP is the main temporal
planner, local CBS is a fallback, and gRPC remains the robot boundary.

To add a planning backend, implement it inside `core/mapf`, normalize its name
in the existing backend selector, pass cancellation into every expensive loop,
and return the existing planner result shape. Do not let a backend import
manager state. To add a manager service, place it in the owning manager module,
inject state containers and the few required callables in `manager.py`, and
keep the old entry point as a short adapter until its callers are migrated.
