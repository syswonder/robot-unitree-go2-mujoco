#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
mkdir -p "${XDG_RUNTIME_DIR:-/tmp/runtime-user}"
python3 sim/bridge/bridge_node.py &
bridge_pid=$!
runtime_pid=""

terminate_process() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  terminate_process "$runtime_pid"
  terminate_process "$bridge_pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

for _ in $(seq 1 100); do
  curl -fsS http://127.0.0.1:8766/health >/dev/null 2>&1 && break
  kill -0 "$bridge_pid" 2>/dev/null || { wait "$bridge_pid"; exit $?; }
  sleep 0.1
done
curl -fsS http://127.0.0.1:8766/health >/dev/null

if [[ "${SIM_GPU_REQUIRED:-1}" == "1" ]]; then
  renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p' | head -n 1)"
  if [[ -z "$renderer" ]]; then
    echo "[native] GPU check failed: no GLX renderer is available on DISPLAY=${DISPLAY:-}" >&2
    exit 3
  fi
  if [[ "$renderer" =~ llvmpipe|softpipe|Software[[:space:]]Rasterizer ]]; then
    echo "[native] GPU check failed: software renderer detected: $renderer" >&2
    exit 3
  fi
  echo "[native] OpenGL renderer: $renderer"
fi

args=(--environment "${SIM_ENVIRONMENT:?SIM_ENVIRONMENT is required}" --robot go2)
[[ "${SIM_HEADLESS:-0}" == "1" ]] && args+=(--headless)
[[ "${SIM_DEV:-0}" == "1" ]] && args+=(--dev)
python3 -m sim.native.runtime "${args[@]}" &
runtime_pid=$!
set +e
wait "$runtime_pid"
status=$?
set -e
exit "$status"
