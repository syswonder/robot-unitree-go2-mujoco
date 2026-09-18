#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:54151}"
API="$(rbnx path robonix-api)"
export PYTHONPATH="$ROOT:$API:$ROOT/primitives/go2_sim_chassis/rbnx-build/codegen/proto_gen${PYTHONPATH:+:$PYTHONPATH}"
exec "${GO2_PROVIDER_PYTHON:-python3}" -u "$ROOT/scripts/showcase.py" "$@"
