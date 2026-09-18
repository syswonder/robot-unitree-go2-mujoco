#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/../${1:?package path required}" && pwd)}"
VENV="$(cd "$(dirname "$0")/.." && pwd)/.venv-codegen"
if [[ ! -x "$VENV/bin/python" ]]; then
  uv venv "$VENV"
  uv pip install --python "$VENV/bin/python" protobuf==6.33.6 grpcio-tools==1.76.0 grpcio==1.78.0
fi
RBNX_CODEGEN_PYTHON="$VENV/bin/python" rbnx codegen -p "$PKG" --mcp
