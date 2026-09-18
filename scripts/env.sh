#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$PROJECT_ROOT/.env" ]]; then
  set -a
  source "$PROJECT_ROOT/.env"
  set +a
fi
export ROBONIX_SOURCE_PATH="${ROBONIX_SOURCE_PATH:-$PROJECT_ROOT/../robonix}"
export ROBONIX_DEPLOY_DIR="${ROBONIX_DEPLOY_DIR:-$PROJECT_ROOT}"
export VLM_BASE_URL="${VLM_BASE_URL:-https://api.openai.com/v1}"
export VLM_MODEL="${VLM_MODEL:-gpt-5.6-sol}"
export ROBONIX_SIM_CONTAINER="${ROBONIX_SIM_CONTAINER:-mujoco_go2_sim}"
export RMW_IMPLEMENTATION="${MUJOCO_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -z "${VLM_API_KEY:-}" ]]; then
  echo "VLM_API_KEY is not configured; export it or add it to the project .env" >&2
  return 1 2>/dev/null || exit 1
fi
