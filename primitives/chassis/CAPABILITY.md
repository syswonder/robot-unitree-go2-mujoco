---
description: Control the simulated Go2 base and expose its local odometry.
---

# Go2 chassis

`move` executes one bounded motion and publishes a zero velocity when it
finishes. `forward_m` and `rotate_deg` are closed against measured odometry;
explicit velocity fields remain duration-bounded. `twist_in` is the continuous
velocity input used by Nav2. Commands are expressed in `base_link`; odometry is
reported in `odom`.

The simulator and bridge must be ready before this provider is activated. On
shutdown it sends a zero Twist. `move` is intended for short direct motions;
use the navigation service for collision-aware travel through a room.

Direct motion defaults to 0.18 m/s and is capped at 0.25 m/s; angular commands
are capped at 0.55 rad/s. `SIM_LINEAR_SPEED` and `SIM_ANGULAR_SPEED` may lower
the speeds. The maximum distance tolerance defaults to 0.03 m and is reduced
for short requests to at most 15 percent of their length, with a 0.015 m
physical floor; angle tolerance defaults to 4 degrees. A relative move fails
when fresh odometry is unavailable, when it makes no new progress for 20
seconds, or when its bounded total timeout expires; these are configurable
with `SIM_DISTANCE_TOLERANCE_M`, `SIM_ANGLE_TOLERANCE_DEG`,
`SIM_RELATIVE_STALL_TIMEOUT_S`, and `SIM_RELATIVE_TIMEOUT_S`. Do not issue
direct moves while navigation or Explore runs.
