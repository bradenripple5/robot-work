#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UI_DIR="${ROOT_DIR}/browser_ui"

HOST="127.0.0.1"
PORT="5173"
API_URL="http://127.0.0.1:8080"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="$2"
      shift 2
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    --api-url)
      API_URL="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./scripts/run_browser_vite.sh [--host HOST] [--port PORT] [--api-url URL]

Examples:
  ./scripts/run_browser_vite.sh
  ./scripts/run_browser_vite.sh --port 5174
  ./scripts/run_browser_vite.sh --api-url http://127.0.0.1:8081
  ./scripts/run_browser_vite.sh --host 0.0.0.0 --port 5173 --api-url http://127.0.0.1:8080

Notes:
  - Run the ROS browser bridge separately first, for example:
      ./scripts/run_browser_bridge.sh --port 8080
  - The Vite dev server defaults to http://127.0.0.1:5173
  - If 5173 is busy, pick another port such as 5174
  - --api-url should point at the ROS bridge host and port
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -d "${UI_DIR}/node_modules" ]]; then
  echo "Dependencies are missing. Run: cd ${UI_DIR} && npm install" >&2
  exit 1
fi

cd "${UI_DIR}"
exec env \
  BCR_UI_HOST="${HOST}" \
  BCR_UI_PORT="${PORT}" \
  BCR_BROWSER_BRIDGE_URL="${API_URL}" \
  npm run dev -- --host "${HOST}" --port "${PORT}"
