#!/usr/bin/env bash
# Run against an already booted deployment. No boot, reset, or VLM credentials required.
# Arguments are documented by --help; stdout is one JSON report, including failures.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ROOT/sim/tests/ros_acceptance.py" \
  --container "${ROBONIX_SIM_CONTAINER:-mujoco_go2_sim}" "$@"
