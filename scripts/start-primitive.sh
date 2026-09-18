#!/usr/bin/env bash
set -euo pipefail
PKG="${1:?package directory required}"
MODULE="${2:?python module required}"
DEFAULT_PROVIDER="${3:?default provider id required}"
PROVIDER="${RBNX_INSTANCE_NAME:-$DEFAULT_PROVIDER}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CT="${ROBONIX_SIM_CONTAINER:-mujoco_go2_sim}"
docker ps --format '{{.Names}}' | grep -qx "$CT" || { echo "sim container $CT is not running" >&2; exit 1; }
exec docker exec \
  -w "/workspace/$PKG" \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_ADVERTISE_HOST="127.0.0.1" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e RBNX_INSTANCE_NAME="$PROVIDER" \
  -e RBNX_PACKAGE_ROOT="/workspace/$PKG" \
  -e SIM_PROVIDER_ID="$PROVIDER" \
  -e PYTHONPATH="/robonix/pylib/robonix-api:/workspace/$PKG/rbnx-build/codegen/proto_gen:/workspace/$PKG/rbnx-build/codegen/robonix_mcp_types:/workspace" \
  "$CT" bash -lc "source /opt/ros/humble/setup.bash && exec python3 -m $MODULE mujoco-robonix-provider=$PROVIDER"
