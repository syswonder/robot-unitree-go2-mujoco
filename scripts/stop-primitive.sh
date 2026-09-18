#!/usr/bin/env bash
set -euo pipefail

DEFAULT_PROVIDER="${1:?default provider id required}"
PROVIDER="${RBNX_INSTANCE_NAME:-$DEFAULT_PROVIDER}"
CT="${ROBONIX_SIM_CONTAINER:-mujoco_go2_sim}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CT"; then
  exit 0
fi

# The marker is appended by start-primitive.sh and identifies exactly one
# deployment instance inside the shared bridge container.
docker exec "$CT" pkill -TERM -f "[m]ujoco-robonix-provider=$PROVIDER" 2>/dev/null || true
