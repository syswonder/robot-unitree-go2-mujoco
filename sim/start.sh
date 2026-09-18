#!/usr/bin/env bash
# Start the simulator; Robonix boot and chat have independent lifecycles.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
export ROBONIX_SOURCE_PATH="${ROBONIX_SOURCE_PATH:-$ROOT/../robonix}"
export ROBONIX_SIM_CONTAINER="${ROBONIX_SIM_CONTAINER:-mujoco_go2_sim}"
export MUJOCO_RMW_IMPLEMENTATION="${MUJOCO_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
HOST_PYTHON="${HOST_PYTHON:-/usr/bin/python3}"
backend="${SIM_BACKEND:-web}"
headless="${SIM_HEADLESS:-0}"
developer_mode="${SIM_DEV:-0}"
environment="${SIM_ENVIRONMENT:-}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend|-b|--environment|-e)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      if [[ "$1" == --backend || "$1" == -b ]]; then backend="$2"; else environment="$2"; fi
      shift 2 ;;
    --headless) headless=1; shift ;;
    --viewer) headless=0; shift ;;
    --dev) developer_mode=1; shift ;;
    --no-dev) developer_mode=0; shift ;;
    --help|-h) echo "Usage: bash sim/start.sh [--backend web|native] [--environment ID] [--viewer|--headless] [--dev|--no-dev]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$backend" == web || "$backend" == native ]] || { echo "Invalid backend: $backend" >&2; exit 2; }
environment="$("$HOST_PYTHON" -c 'import json,sys; m=json.load(open("assets/environments/manifest.json")); e=sys.argv[1] or m["defaultEnvironment"]; assert e in [x["id"] for x in m["environments"]], "unknown environment"; print(e)' "$environment")"
export SIM_BACKEND="$backend" SIM_ENVIRONMENT="$environment" SIM_HEADLESS="$headless" SIM_DEV="$developer_mode" SIM_ROBOT=go2
export MUJOCO_GL="${MUJOCO_GL:-glfw}"
mkdir -p .runtime
if [[ -f .runtime/sim.pid ]] && kill -0 "$(cat .runtime/sim.pid)" 2>/dev/null; then
  echo "Simulator is already running; stop it with bash sim/stop.sh" >&2
  exit 1
fi
"$HOST_PYTHON" -c 'import socket
for port in (5181,8765,8766):
 s=socket.socket()
 s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
 try: s.bind(("127.0.0.1",port))
 except OSError: raise SystemExit(f"Port {port} is occupied; stop its service before starting Go2")
 finally: s.close()'
[[ "$backend" != web || -d node_modules/playwright ]] || { echo "Run bash scripts/bootstrap.sh first" >&2; exit 1; }
echo $$ > .runtime/sim.pid
compose=(docker compose -f sim/compose.yaml)
[[ "$backend" != native ]] || compose+=(-f sim/compose.native.yaml)
cleanup() {
  # Only terminate processes and containers started for this deployment.
  status=$?
  trap - EXIT INT TERM HUP
  bash "$ROOT/sim/stop.sh"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
"$HOST_PYTHON" scripts/serve.py --bind 127.0.0.1 --port 5181 >.runtime/web.log 2>&1 &
echo $! > .runtime/web.pid
"${compose[@]}" up -d >.runtime/compose.log 2>&1
ready=false
for _ in $(seq 1 120); do
  if curl -fsS http://127.0.0.1:8766/health >/dev/null 2>&1; then ready=true; break; fi
  state="$(docker inspect "$ROBONIX_SIM_CONTAINER" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "$state" == running ]] || break
  sleep 0.25
done
if [[ "$ready" != true ]]; then
  docker logs "$ROBONIX_SIM_CONTAINER" >.runtime/bridge.log 2>&1 || true
  echo "Bridge failed; inspect .runtime/bridge.log and .runtime/compose.log" >&2
  exit 1
fi
if [[ "$backend" == web ]]; then
  sim_url="http://127.0.0.1:5181/?runtime=1&environment=$environment"
  [[ "$developer_mode" != 1 ]] || sim_url+="&dev=1"
  SIM_URL="$sim_url" node scripts/launch-browser.mjs >.runtime/browser.log 2>&1 &
  echo $! > .runtime/browser.pid
fi
ready=false
for _ in $(seq 1 360); do
  if curl -fsS http://127.0.0.1:8766/health | "$HOST_PYTHON" -c 'import json,sys; h=json.load(sys.stdin); raise SystemExit(not(h.get("ok") and h.get("backend")==sys.argv[1] and h.get("environment")==sys.argv[2] and h.get("frames",{}).get("state",0)>0))' "$backend" "$environment" 2>/dev/null; then
    ready=true; break
  fi
  state="$(docker inspect "$ROBONIX_SIM_CONTAINER" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "$state" == running ]] || break
  sleep 0.5
done
if [[ "$ready" != true ]]; then
  docker logs "$ROBONIX_SIM_CONTAINER" >.runtime/bridge.log 2>&1 || true
  echo "Runtime failed; inspect .runtime/bridge.log and .runtime/browser.log" >&2
  exit 1
fi
echo "[sim/start] $backend ready; environment=$environment; dev=$developer_mode; ONNX policy initially OFF"
echo "[sim/start] UI: http://127.0.0.1:5181/?backend=$backend&environment=$environment"
echo "[sim/start] Health: http://127.0.0.1:8766/health"
echo "[sim/start] Terminal 2: source scripts/env.sh && rbnx boot"
echo "[sim/start] Terminal 3: source scripts/env.sh && rbnx chat"
if [[ "$backend" == web ]]; then wait "$(cat .runtime/browser.pid)"; else docker wait "$ROBONIX_SIM_CONTAINER" >/dev/null; fi
