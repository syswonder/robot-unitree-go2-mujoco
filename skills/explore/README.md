# Go2 Explore Adaptation

Deployment-owned copy of `syswonder/skill-explore-rbnx`; see [UPSTREAM.md](UPSTREAM.md)
for the exact commit and retained MulanPSL-2.0 license attribution.
Runtime identity remains `explore`. All capability TOML and IDL files are unchanged.

## Goal Selection

The original cluster centroid could fall inside unknown furniture interiors.
A failed goal was then selected again because failure did not affect ranking.
Here NumPy disk erosion requires the whole conservative robot footprint to be
known-free, and a four-connected breadth-first search restricts candidates to
the robot's reachable component. Another bounded search links approaches to
actual frontier members through known-free cells. Unknown centroids are never
used as goals.

Navigation failures exclude a world-coordinate neighbourhood for a finite time.
Each iteration recomputes from the latest map; map resizing reprojects visited
cells. If no approach exists, the skill waits for map updates or suppression
expiry, without claiming completion or repeatedly issuing recovery sweeps.
Completed navigation legs appear in the existing status `detail` field as
`completed_legs=N`; periodic camera sweeps remain after every third leg.

Cancellation only signals the task worker. That worker cancels its exact
navigation `run_id` on cancellation, timeout or a status-call error. It uses a
fresh MCP client per call, with a ten-second RPC timeout, so no client crosses
event loops. After cancel acknowledgement it polls that exact run for up to
30 seconds until Navigation confirms terminal state. A failed or unconfirmed
stop becomes a visible FAILED task, retains the low speed cap, and blocks
another task. The cancel response acknowledges the request; poll status for
terminal state. Shutdown allows a 90-second worker join and Docker allows
120 seconds before removal, covering cancellation and speed cleanup.

## Deployment

```yaml
skill:
  - name: explore
    path: ./skills/explore
    config:
      robot_radius_m: 0.4031128874149275
      approach_distance_m: 0.8
      failed_goal_radius_m: 0.6
      failed_goal_ttl_s: 60
```

Values are finite and positive; the approach distance must exceed the radius.
Initialization rejects a radius smaller than the Go2 footprint's enclosing disk.
The default radius encloses the 0.70 x 0.40 m Go2 footprint. See `config.spec`.
`timeout_s` remains per request. Before motion, `max_speed_m_s` is converted
to a percentage of the actual maximum from Navigation's existing
`get_speed_limit` capability and applied with `set_speed_limit`. An already
lower cap is preserved; requests below Navigation's minimum are rejected.
After confirmed stop, a changed session cap is restored only if its percentage,
maximum and session ownership remain unchanged. External changes are preserved.
Do not overlap Explore with another motion task. These two additional consumed
capabilities are Atlas-resolved; the skill's exported contracts are unchanged.

Build from the deployment with `rbnx build`, then boot normally.
The reused upstream scripts generate stubs and build/run Docker. Defaults are
image `robonix-go2-explore`, container `robonix_go2_explore`, and Fast DDS.
Existing `ROBONIX_EXPLORE_IMAGE`, `ROBONIX_EXPLORE_CONTAINER`, `ROS_DOMAIN_ID`,
and `RMW_IMPLEMENTATION` overrides remain supported.

## Validation

With Python and NumPy, from this package directory:

```bash
python3 -m unittest discover -s tests -v
rbnx validate .
```

Tests include unknown furniture interiors, disconnected free rooms, a narrow
blocked gap, a newly observed 0.90 m doorway, failure expiry across map-origin
changes, cancellation and timeouts. They do not read scene XML or move a robot.
End-to-end acceptance must show completed frontier legs and an observed doorway
crossing. Static geometry and growing map area alone are insufficient.
