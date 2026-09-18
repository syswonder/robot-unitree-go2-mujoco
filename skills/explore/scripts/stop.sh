#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

CT="${ROBONIX_EXPLORE_CONTAINER:-robonix_go2_explore}"
if ! docker container inspect "$CT" >/dev/null 2>&1; then
    exit 0
fi
# Allow worker RPCs, terminal-state polling, speed restoration and thread joins.
docker stop --time 120 "$CT" >/dev/null
if docker container inspect "$CT" >/dev/null 2>&1; then
    docker rm "$CT" >/dev/null
fi
