#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# floor_transition_rbnx start phase — docker-run wrapper.
#
# Container shape: --network host + --ipc=host so the skill can
# subscribe to /map (mapping container) and call the nav
# service's gRPC endpoint without DDS isolation getting in the way.
#
# Trap: when boot SIGTERMs our PGID, this trap stops the container so
# the skill doesn't outlive the deploy.
set -euo pipefail

CT="${ROBONIX_FLOOR_TRANSITION_CONTAINER:-robonix_go2_floor_transition}"
IMG="${ROBONIX_FLOOR_TRANSITION_IMAGE:-robonix-go2-floor-transition}"
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
DEPLOY_ROOT="$(cd "$PKG/../.." && pwd)"
SCENE_PROFILE="$DEPLOY_ROOT/assets/environments/scenesmith_multilevel_house/multifloor.yaml"
RUNTIME_PROTO_TMP=""
cd "$PKG"
[[ -f "$SCENE_PROFILE" ]] || { echo "[floor_transition/start] missing scene profile: $SCENE_PROFILE" >&2; exit 1; }

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [[ -n "$RUNTIME_PROTO_TMP" && -d "$RUNTIME_PROTO_TMP" ]]; then
        rm -rf -- "$RUNTIME_PROTO_TMP"
    fi
    bash "$PKG/scripts/stop.sh" || return $?
    kill -- "-$$" 2>/dev/null || true
    return "$status"
}
trap cleanup EXIT INT TERM

bash "$PKG/scripts/stop.sh"

# Host codegen still supplies MCP dataclasses. Generate the protobuf modules
# used by Docker with the exact image runtime, validate every generated module
# without network access, and atomically publish the complete output.
prepare_runtime_proto_gen() {
    local proto_staging="$PKG/rbnx-build/proto-staging"
    local runtime_proto
    local runtime_proto_gen="$PKG/rbnx-build/codegen/floor_transition_proto_gen"

    runtime_proto="$(rbnx path runtime-proto)" || {
        echo "[floor_transition/start] cannot resolve Robonix runtime proto directory" >&2
        return 1
    }
    [[ -d "$runtime_proto" && -f "$runtime_proto/atlas.proto" ]] || {
        echo "[floor_transition/start] missing runtime atlas.proto: $runtime_proto" >&2
        return 1
    }
    [[ -d "$proto_staging" ]] \
        && find "$proto_staging" -maxdepth 1 -type f -name '*.proto' -print -quit \
            | grep -q . || {
        echo "[floor_transition/start] missing staged package protos; run rbnx build first" >&2
        return 1
    }

    mkdir -p "$PKG/rbnx-build/codegen"
    RUNTIME_PROTO_TMP="$(mktemp -d "${runtime_proto_gen}.tmp.XXXXXX")"
    docker run --rm \
        --network none \
        --entrypoint sh \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        -v "$runtime_proto:/runtime-proto:ro" \
        -v "$proto_staging:/proto-staging:ro" \
        -v "$RUNTIME_PROTO_TMP:/proto-gen" \
        "$IMG" -ec '
            python3 -m grpc_tools.protoc \
                -I/runtime-proto \
                -I/proto-staging \
                --python_out=/proto-gen \
                --grpc_python_out=/proto-gen \
                /runtime-proto/*.proto \
                /proto-staging/*.proto
            PYTHONPATH=/proto-gen python3 -c '\''import importlib, pathlib; p = pathlib.Path("/proto-gen"); modules = sorted({f.stem for f in p.glob("*_pb2.py")} | {f.stem for f in p.glob("*_pb2_grpc.py")}); assert modules; [importlib.import_module(name) for name in modules]'\''
        '

    rm -rf -- "$runtime_proto_gen"
    mv "$RUNTIME_PROTO_TMP" "$runtime_proto_gen"
    RUNTIME_PROTO_TMP=""
    echo "[floor_transition/start] runtime-compatible protobuf stubs ready"
}

prepare_runtime_proto_gen
mkdir -p rbnx-build/data

declare -a EXTRA_MOUNTS=()
if [[ -n "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "${RBNX_CONFIG_FILE}:${RBNX_CONFIG_FILE}:ro")
fi

declare -a ZENOH_ARGS=()
if [[ -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    ZENOH_ARGS=(-e "ROBONIX_ZENOH_ROUTER=${ROBONIX_ZENOH_ROUTER}")
fi
if [[ -n "${ROBONIX_ZENOH_MODE:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_MODE=${ROBONIX_ZENOH_MODE}")
fi
if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_LISTEN=${ROBONIX_ZENOH_LISTEN}")
fi

# Bash 3.2 treats an empty array expansion as an unset variable under `set -u`.
# All scalar values below already have explicit defaults.
set +u
exec docker run --rm \
    --name "$CT" \
    --stop-timeout 120 \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-com.robonix.skill.floor_transition}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    "${ZENOH_ARGS[@]}" \
    -v "$(pwd)":/floor_transition \
    -v "$SCENE_PROFILE:/scene_profile/multifloor.yaml:ro" \
    -v "$PKG/rbnx-build/codegen/floor_transition_proto_gen:/floor_transition/rbnx-build/codegen/proto_gen:ro" \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
