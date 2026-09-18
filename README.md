# Unitree Go2 · Native MuJoCo × Robonix

A standalone, simulation-only Go2 body package. It reuses the pinned Unitree
MJCF and a pinned `rl_sar` Go2 walking policy, then supplies a Native runtime,
ROS 2 feedback bridge and four Robonix device primitives. No physical robot,
Unitree network interface, sport daemon, Docker image or closed-source gait is
needed. This is separate from the validated physical Go2 navigation/RobotTrack
deployment and does not replace it.

![Actual MuJoCo Go2 rendering](docs/walking-demo.png)

Optional [courtyard and action showcase](docs/courtyard-showcase.md): a campus
courtyard with collision obstacles, an Atlas/gRPC choreography, torque-driven
low-hurdle jumping, stairs, ramps, slalom, crouch, bow, dance, and optional
pretrained backflip/handstand walking. See the guide for policy preparation. Start with
`bash start.sh --scene courtyard --viewer --follow-camera`; the default room
remains available. These are simulation actions, not proprietary Unitree sport
tricks or autonomous navigation.

For other scene owners: [reuse the actions in your scene](docs/reusing-actions.md).
After building and booting this body package, `bash scripts/action.sh --list`
lists available actions and `bash scripts/action.sh backflip` invokes one action
through Atlas/gRPC, without running or resetting the courtyard route.

## Integration and current acceptance

```text
Robonix caller → Atlas discovery → chassis/move gRPC ─────┐
ROS /go2_sim/cmd_vel → Bridge ────────────────────────────┤
Local web velocity controls ─────────────────────────────┤
                                                       ▼
                           localhost HTTP /command → one-writer 0.4 s lease
                                                       ▼
                       body velocity → yaw feedback → pinned RL policy (50 Hz)
                                                       ▼
                       12 named joint targets → bounded PD torque (200 Hz)
                                                       ▼
                             official MJCF + MuJoCo mj_step
                                                       ▼
              odometry / joints / IMU / ray scan / rendered RGB-D / simulation time
                                                       ▼
                         Bridge → ROS 2 / TF → Robonix sensor capabilities
```

The Native process, not the browser, owns physics. `/web` is a live RGB camera
preview and command panel, **not a MuJoCo WASM implementation**. `--viewer`
opens the full Native 3D scene. Reset is the only direct root-pose assignment;
walking is produced by motor torques and contacts, not pose animation.

On 2026-09-11, with a repeat on 2026-09-12, actual local acceptance covered:

- Standing, forward/backward motion, left lateral motion and both yaw directions.
  A 0.4 m/s forward command over 5 simulated seconds produced about 1.92 m.
- ROS RGB-D, scan, IMU, joint states, odometry, TF and advancing `/clock`;
  ROS command movement, command-expiry stop, Trigger stop and reset.
- `rbnx boot`: Atlas, Soma, Executor and four ACTIVE device primitives; 13
  primitive capabilities, without namespace mismatches.
- Atlas-discovered gRPC movement: timed velocity, 0.5 m relative motion
  (measured about 0.47 m at return), 45° rotation (about 42.8° at return),
  concurrent-call rejection, lifecycle cancellation and invalid-input rejection.

Measured reports: [acceptance](docs/acceptance.md). The 18-second
[Native recording](docs/walking-demo.mp4) shows real simulated walking and
turning. These results do not claim terrain robustness, calibrated EDU payload
physics, proprietary Unitree 2010 gait equivalence, physical robot performance,
autonomous SLAM/Nav2, or language-driven room exploration. Downstream services
can consume the interfaces below, but those complete tasks need their own tests.

## Requirements and installation

Validated host profile: Ubuntu 22.04 x86_64, Python 3.10, ROS 2 Humble,
MuJoCo 3.3.6, NumPy 1.26.4, CPU PyTorch 2.8.0; EGL for rendered camera frames.
Physics and policy run on CPU. A working OpenGL/EGL context is required for RGB-D;
X11/Wayland is additionally needed for `--viewer`. Headless does not mean
camera rendering needs no graphics context.

Install ROS 2 Humble, CycloneDDS RMW, standard messages, `tf2_ros`, `colcon` and
Robonix using their own installation instructions. Robonix must provide `rbnx`,
`robonix-atlas`, `robonix-soma`, `robonix-executor`, codegen and a Python
environment containing `robonix_api`, `grpcio` and `grpcio-tools`. Keep gRPC
runtime/codegen versions compatible. No system/ROS dependency installation is
performed implicitly by this repository's build or startup scripts.

```bash
git clone https://github.com/syswonder/robot-unitree-go2-mujoco.git
cd robot-unitree-go2-mujoco
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-sim.txt
.venv/bin/python -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
export GO2_SIM_PYTHON="$PWD/.venv/bin/python"
# Optional: a separate existing Robonix Python environment, with grpc and rclpy:
# export GO2_PROVIDER_PYTHON=/absolute/path/to/robonix-env/bin/python
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
bash build.sh
```

`build.sh` verifies/downloads 22 immutable model, configuration and license
files, exports a kinematic/inertial URDF for Soma, and runs Robonix codegen/build.
Assets stay in `.runtime/assets/`. Model and policy pins/checksums are in
`assets.lock.json`; a modified cached asset causes a digest error instead of being
overwritten. For offline model reuse:

```bash
.venv/bin/python scripts/prepare_assets.py --model-source /path/to/unitree_mujoco
```

Robonix generates a canonical ROS IDL overlay once in the chassis package; the
sensor primitives share it. Other generated Python/protobuf artifacts remain
per-package. Do not point this overlay at the physical robot deployment.

## Start, inspect, stop

Terminal 1 (Native simulation plus ROS Bridge):

```bash
bash start.sh                         # headless Native + RGB-D + web preview
# bash start.sh --viewer              # Native interactive 3D viewer
# bash start.sh --duration 75          # bounded simulator lifetime in wall seconds
```

Open **http://127.0.0.1:18765/web**. Hold a movement button to refresh its command;
releasing it requests zero velocity. Stop/reset affect simulation only. A web
owner, ROS stream, and Primitive RPC never add their velocities together.
Only one source can refresh a live lease; it expires after 0.4 s without updates.

Terminal 2 (Robonix):

```bash
export ROBONIX_ATLAS=127.0.0.1:54151
rbnx boot -f robonix_manifest.yaml --no-update-check
```

Terminal 3 (inspection):

```bash
rbnx caps --server 127.0.0.1:54151 -v
```

`rbnx shutdown -f robonix_manifest.yaml` stops this Robonix stack. Ctrl-C in
Terminal 1 stops this simulator and Bridge and releases the renderer. Shutdown
uses this launcher's own child PIDs, never broad `pkill`. Running without camera
via `--no-camera` is useful for physics debugging, but the full camera-enabled
Robonix manifest will correctly fail camera readiness in that mode.
Alternatively, after `rbnx shutdown`, run `bash sim/stop.sh` from another terminal
to ask the verified local Native endpoint to exit and release its resources.

Private ports: Native HTTP **18765**, Atlas **54151**, Executor **54161**, Soma
**54191**; provider ports are allocated by Robonix. ROS uses localhost-only
domain **141**, CycloneDDS, with task-process DDS configuration overrides cleared.
This is process isolation, not a change to host network configuration. Avoid
using domain 141 for another simulation at the same time. Do not remap these
interfaces to `/api/sport/request`, `/lowcmd`, or a physical robot's command topic.

## Interface reference

All distances are meters, linear velocities m/s and angular velocities rad/s.
Go2 body axes: +X forward, +Y left, +Z up; positive yaw turns left.

| Robonix capability | Native/ROS endpoint | Content |
| --- | --- | --- |
| `chassis/move` | discovered gRPC | `chassis/ExecuteMoveCommand` |
| `chassis/twist_in` | `/go2_sim/cmd_vel` | `geometry_msgs/Twist` |
| `chassis/odom` | `/go2_sim/odom` | `nav_msgs/Odometry`, body-frame twist |
| `lidar/lidar` | `/go2_sim/scan` | 180 actual geometric rays, 20 Hz target |
| `camera/rgb` | `/go2_sim/camera/color/image_raw` | 320×240, `rgb8`, 5 Hz target |
| `camera/depth` | `/go2_sim/camera/depth/image_raw` | same optical geometry, `32FC1` meters |
| `camera/intrinsics` | `/go2_sim/camera/camera_info` | `CameraInfo`, reliable/transient-local |
| `camera/extrinsics` | `/go2_sim/camera/extrinsics` | `TransformStamped`, reliable/transient-local |
| `imu/imu` | `/go2_sim/imu` | simulated native gyro/accelerometer, 20 Hz target |

Capability prefixes above abbreviate `robonix/primitive/`. Every device also
exposes the standard `*/driver` lifecycle capability. Other ROS endpoints:
`/clock`, `/tf`, `/tf_static`, `/go2_sim/joint_states`, `/go2_sim/status`,
and Trigger services `/go2_sim/stop`, `/go2_sim/reset`.

`chassis/move` modes follow the standard priority: nonzero `forward_m`, then
nonzero `rotate_deg`, otherwise velocity fields plus `duration_sec` (default 1 s).
Relative movement uses measured pose, not speed-times-duration estimates.
Supported axes are `linear_x`, `linear_y`, `angular_z`. Native limits are
±0.5 m/s forward/backward, ±0.3 m/s lateral, ±0.6 rad/s yaw. Bursts are bounded
to 30 s; larger plans should compose moves. Relative requests support up to 5 m
or 360° with a 30 s timeout. A timeout is reported, never claimed as completion.

Raw HTTP examples for a local simulator only:

```bash
curl -s http://127.0.0.1:18765/state
curl -s -H 'Content-Type: application/json' -d '{"owner":"example","velocity":[0.2,0,0]}' http://127.0.0.1:18765/command
curl -s -H 'Content-Type: application/json' -d '{"stop":true}' http://127.0.0.1:18765/command
```

A single HTTP command lasts at most 0.4 s. Repeat at 10 Hz for sustained motion,
or use the timed Robonix interface. This API is loopback-only and is not designed
as an authenticated remote-control service.

## Reproduce acceptance

```bash
bash scripts/validate.sh              # assets, 15 unit tests, six dynamic cases
# With start.sh running, from this repository:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=141 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
"$GO2_SIM_PYTHON" scripts/test_ros.py
# With rbnx boot also running; use the Robonix Python environment:
export ROBONIX_ATLAS=127.0.0.1:54151
export PYTHONPATH="$PWD:$(rbnx path robonix-api):$PWD/primitives/go2_sim_chassis/rbnx-build/codegen/proto_gen:$PYTHONPATH"
"${GO2_PROVIDER_PYTHON:-python3}" scripts/test_robonix.py
# Optional actual dynamics recording (ffmpeg required):
MUJOCO_GL=egl "$GO2_SIM_PYTHON" scripts/record_demo.py
```

Tests actively move only the isolated simulator, so do not run them while
manually driving it. JSON reports are generated under `.runtime/`. The provided
`ci/validate.yaml` workflow template runs offline model/command/physics tests;
move it to `.github/workflows/validate.yaml` when workflow-write permission is
available. The publishing credential does not have that scope, so GitHub Actions
is not claimed as executed. The local ROS/Robonix/EGL acceptance is
recorded separately rather than misrepresented as a cloud end-to-end CI run.

## Sources and integration boundaries

- [Robonix MuJoCo onboarding guide](https://book.robonix.ai/integration-guide/mujoco-simulation-onboarding).
- [Unitree Go2 model](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2), BSD-3-Clause.
- [rl_sar Go2 policy](https://github.com/fan-ziqi/rl_sar/tree/376d42c9b128f963ab08579762d5a216a976ce39/policy/go2), Apache-2.0.
- [Earlier physical-Go2 resource handoff PR #8](https://github.com/syswonder/robot-unitree-go2/pull/8) is merged; this independent runtime extends that resource work.

The model is the upstream generic Go2, not a calibrated model of every EDU
accessory. RGB-D and planar LiDAR are ideal virtual sensors, not exact D435i or
MID-360 replicas. IMU/TF/mounts and the Soma URDF agree with this simulation.
The URDF intentionally exports kinematics/inertia without another visual/contact
model; the MJCF is authoritative. Its planning footprint is conservative.

The added yaw feedback corrects measured idle drift of this model-policy pairing:
`policy_yaw = clip(requested_yaw + 2 × wrap(target_heading − measured_heading), −1, 1)`.
It does not change policy weights, mesh files, joint ordering or source MJCF.
See [NOTICE](NOTICE) and [LICENSE](LICENSE) for attribution.
