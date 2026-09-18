#!/usr/bin/env bash
# Stop only the MuJoCo simulator. Stop Robonix separately with `rbnx shutdown`.
set -euo pipefail

if [[ $# -gt 0 ]]; then
  case "$1" in
    --help|-h)
      echo "Usage: $0"
      exit 0
      ;;
    *)
      echo "[sim/stop] unknown argument: $1" >&2
      echo "Usage: $0" >&2
      exit 2
      ;;
  esac
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNTIME="$ROOT/.runtime"

cd "$ROOT"
for name in browser web; do
  pid_file="$RUNTIME/$name.pid"
  if [[ -f "$pid_file" ]]; then
    pid="$(cat "$pid_file")"
    kill "$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    rm -f "$pid_file"
  fi
done

docker compose -f sim/compose.yaml down
rm -f "$RUNTIME/sim.pid"
echo "[sim/stop] simulator stopped; Robonix lifecycle is managed by rbnx shutdown"
