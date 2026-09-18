#!/usr/bin/env bash
# Deterministic floor-transition client for demo acceptance; bypasses Pilot/VLM.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/floor-transition.sh up
  scripts/floor-transition.sh down
  scripts/floor-transition.sh floor 1|2
  scripts/floor-transition.sh status RUN_ID
  scripts/floor-transition.sh cancel RUN_ID

The transition commands wait for a terminal skill state and print phase changes.
EOF
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }

operation="$1"
target_floor="0"
run_id=""
case "$operation" in
  up|down)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    ;;
  floor)
    [[ $# -eq 2 && ( "$2" == "1" || "$2" == "2" ) ]] || { usage >&2; exit 2; }
    target_floor="$2"
    ;;
  status|cancel)
    [[ $# -eq 2 && -n "$2" ]] || { usage >&2; exit 2; }
    run_id="$2"
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

container="${ROBONIX_FLOOR_TRANSITION_CONTAINER:-robonix_go2_floor_transition}"
docker inspect "$container" >/dev/null 2>&1 || {
  echo "Floor-transition container is not running: $container" >&2
  exit 1
}

mcp_url="$({ rbnx inspect || true; } | python3 -c '
import json, sys
try:
    provider = json.load(sys.stdin)["providers"]["floor_transition"]
    endpoint = next(
        item["endpoint"] for item in provider["endpoints"]
        if item["contract_id"] == "robonix/skill/floor_transition/execute"
        and item["transport"] == "TRANSPORT_MCP"
    )
except Exception as error:
    raise SystemExit(f"Cannot discover floor_transition MCP endpoint: {error}")
print(endpoint)
')"

docker exec -i \
  -e FLOOR_MCP_URL="$mcp_url" \
  -e FLOOR_CLI_OPERATION="$operation" \
  -e FLOOR_CLI_TARGET="$target_floor" \
  -e FLOOR_CLI_RUN_ID="$run_id" \
  "$container" python3 - <<'PY'
import asyncio
import json
import os
import sys
import time

from fastmcp import Client


def result_payload(result):
    structured = result.structured_content or {}
    payload = structured.get("result", structured)
    if not isinstance(payload, dict):
        raise RuntimeError(f"unexpected MCP response: {result}")
    return payload


def emit(payload):
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), flush=True)


async def main():
    operation = os.environ["FLOOR_CLI_OPERATION"]
    target = int(os.environ["FLOOR_CLI_TARGET"])
    run_id = os.environ["FLOOR_CLI_RUN_ID"]
    async with Client(os.environ["FLOOR_MCP_URL"]) as client:
        if operation == "status":
            emit(result_payload(await client.call_tool("status", {"run_id": run_id})))
            return 0
        if operation == "cancel":
            payload = result_payload(await client.call_tool("cancel", {"run_id": run_id}))
            emit(payload)
            return 0 if payload.get("ok") else 1

        command = "GO_TO_FLOOR" if operation == "floor" else operation.upper()
        payload = result_payload(await client.call_tool("execute", {
            "command": command,
            "target_floor": target,
            "timeout_s": 0.0,
        }))
        emit(payload)
        if not payload.get("accepted"):
            return 1

        run_id = str(payload["run_id"])
        deadline = time.monotonic() + 150.0
        previous = None
        while time.monotonic() < deadline:
            status = result_payload(await client.call_tool("status", {"run_id": run_id}))
            snapshot = (status.get("state"), status.get("phase"), status.get("detail"))
            if snapshot != previous:
                emit(status)
                previous = snapshot
            state = str(status.get("state", "")).upper()
            if state in {"SUCCEEDED", "FAILED", "CANCELED"}:
                return 0 if state == "SUCCEEDED" else 1
            await asyncio.sleep(0.5)
        print(f"Timed out waiting for run {run_id}; it was not canceled.", file=sys.stderr)
        return 1


raise SystemExit(asyncio.run(main()))
PY
