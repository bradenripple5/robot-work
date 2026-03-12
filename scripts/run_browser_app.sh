#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

HOST="127.0.0.1"
UI_PORT="5176"
API_PORT="8080"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="$2"
      shift 2
      ;;
    --ui-port)
      UI_PORT="$2"
      shift 2
      ;;
    --api-port)
      API_PORT="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./scripts/run_browser_app.sh [--host HOST] [--ui-port PORT] [--api-port PORT]

Examples:
  ./scripts/run_browser_app.sh
  ./scripts/run_browser_app.sh --ui-port 5177
  ./scripts/run_browser_app.sh --host 0.0.0.0 --ui-port 5176 --api-port 8081

Notes:
  - User-facing browser UI runs on the UI port only
  - The backend bridge stays internal on the API port and is proxied by Vite
  - Open the Vite URL, not the API URL
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

"${ROOT_DIR}/scripts/run_browser_bridge.sh" --host "${HOST}" --port "${API_PORT}" >/tmp/bcr_browser_bridge.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
  kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

exec "${ROOT_DIR}/scripts/run_browser_vite.sh" \
  --host "${HOST}" \
  --port "${UI_PORT}" \
  --api-url "http://${HOST}:${API_PORT}"
