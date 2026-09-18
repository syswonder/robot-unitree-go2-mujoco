# floor_transition

Deployment-owned skill for moving the simulated Go2 between the two mapped
floors of `scenesmith_multilevel_house`. It exposes asynchronous `execute`,
`status`, and `cancel` MCP capabilities for `UP`, `DOWN`, and
`GO_TO_FLOOR(1|2)`.

## Scope

This is a fixed-scene transition controller, not a general stair detector or a
multi-floor planner. It supports exactly:

- floor 1 to floor 2 through the annotated `up` connection;
- floor 2 to floor 1 through the reciprocal `down` connection;
- idempotent requests for the current floor; and
- one active transition task at a time.

It is suitable for a controlled MuJoCo demo, integration testing, map-switching
regression, and evaluating the packaged rough-terrain policy on a known stair.
Its calibration and safety evidence do not transfer automatically to another
scene or to physical Go2 hardware.

The skill explicitly does not provide:

- visual or lidar-based discovery of an unknown stair;
- online step/ramp geometry estimation;
- multiple stair selection, spiral/multi-flight stairs, or elevators;
- dynamic obstacle avoidance while on the stair;
- cross-floor Nav2 planning on one 2D map;
- post-transition exploration, remapping, or semantic-object navigation; or
- real-hardware functional safety.

## Ownership and Dependencies

The deployment mounts the scene-owned `multifloor.yaml` read-only at
`/scene_profile/multifloor.yaml`. The profile owns floor IDs, map IDs, entry
poses, centerline, direction, landing heights, speed, tolerances, and timeouts.
Do not copy the sample coordinates into a different scene.

The skill resolves these existing contracts through Atlas:

- chassis odometry and Twist input;
- IMU orientation;
- Mapping pose and map loading; and
- Nav2 navigate, status, and cancel.

It also uses the local simulator bridge for acknowledged `moe_rough` policy
load/unload commands. It does not patch Mapping, Navigation, or Scene. Scene is
not required by the transition state machine, and stair annotations are not
inserted into Scene's semantic object graph.

## Preconditions

Before activation and every transition:

1. The simulator must be healthy and report `scenesmith_multilevel_house`.
2. `scenesmith_multilevel_floor_1_v2` and
   `scenesmith_multilevel_floor_2_v2` must be installed in the Mapping map
   directory. These are full sensor-built maps of the safely reachable areas,
   with 439 and 787 RTAB-Map nodes respectively.
3. The robot must physically be on the same floor as persisted
   `current_floor`; simulator resets or teleports invalidate that assumption.
4. Odom, IMU, Mapping, Nav2, and the simulator policy endpoint must be healthy.
5. The stair must be clear and its geometry/friction must match the calibrated
   profile.
6. Explore, ordinary Navigation, manual driving, and other Twist or policy
   controllers must be stopped. This skill requires exclusive control.

The packaged maps are compressed source assets.
`scripts/install-prebuilt-maps.sh` expands missing complete maps into the
runtime map directory without overwriting an existing complete map. The maps
are available to another user only if `assets/maps` is included in the
repository push or release artifact; neither the skill nor the installer
uploads maps.

## Execution Model

Activation validates the environment, unloads an already loaded stair policy,
waits for odometry, and loads the persisted floor map using actual odometry as
the localization seed.

For each stair leg the controller:

1. unloads `moe_rough` and uses the deterministic gait for flat-floor motion;
2. uses Nav2 outside the connection zone, then directly aligns at the marked
   entry;
3. verifies entry position and heading before loading `moe_rough`;
4. traverses the fixed centerline while checking lane error, body uprightness,
   direction, endpoint, height, cancellation, and timeout;
5. stops and unloads the policy on the destination landing;
6. aligns to the destination floor's standard heading; and
7. loads the destination map in Mapping localization mode and persists the new
   floor only after that load succeeds.

An already loaded policy is therefore unloaded for the approach and reloaded
for the stair. A policy operation that is not acknowledged fails closed. Do not
change policy state concurrently from the simulator UI.

Mapping performs an explicit map replacement. Nav2 remains active and receives
the new `/map` through its static costmap layers for subsequent navigation. The
skill currently does not issue an explicit global/local costmap clear. Scene
does not gain a new floor context or filter semantic objects after the switch.

## Failure and Cancellation Semantics

The controller publishes zero velocity on terminal paths. Entry, heading, lane,
upright, landing-height, timeout, policy, and map-load failures prevent the new
floor from being committed. If Nav2 is active during the approach, cancellation
targets the exact navigation `run_id` and waits for its terminal state.

One important boundary remains: a physical traversal can finish before a target
map load fails. In that case the task reports failure and does not persist the
new floor, so an operator must reconcile physical position and state before the
next command.

## Operations

Build and package validation are handled by the deployment bootstrap. From the
deployment root, with the simulator and Robonix stack running, deterministic
acceptance can bypass Pilot/VLM:

```bash
bash scripts/floor-transition.sh up
bash scripts/floor-transition.sh down
bash scripts/floor-transition.sh floor 2
bash scripts/floor-transition.sh floor 1
```

The client prints the accepted `run_id`, phase changes, active map, current
floor, and terminal result. After every run verify that the robot reached the
expected landing, Mapping selected the matching floor map, Nav2 is active, and
the policy is unloaded.

See `docs/MULTI_FLOOR_DEMO.zh-CN.md` in the deployment root for the complete
annotation schema, calibrated values, test results, and manual acceptance
procedure.
