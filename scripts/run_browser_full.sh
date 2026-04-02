#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

HOST="127.0.0.1"
UI_PORT="5176"
API_PORT="8080"
GAZEBO_LOG="/tmp/bcr_browser_gazebo.log"
BRIDGE_LOG="/tmp/bcr_browser_bridge.log"
UI_LOG="/tmp/bcr_browser_ui.log"
DETACH=0
GAZEBO_PID_FILE="/tmp/bcr_browser_gazebo.pid"
BRIDGE_PID_FILE="/tmp/bcr_browser_bridge.pid"
UI_PID_FILE="/tmp/bcr_browser_ui.pid"

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
    --detach)
      DETACH=1
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./scripts/run_browser_full.sh [--host HOST] [--ui-port PORT] [--api-port PORT] [--detach]

Examples:
  ./scripts/run_browser_full.sh
  ./scripts/run_browser_full.sh --detach
  ./scripts/run_browser_full.sh --ui-port 5177
  ./scripts/run_browser_full.sh --host 0.0.0.0 --ui-port 5176 --api-port 8081

What it starts:
  - Gazebo software stack
  - Browser bridge backend API
  - Vite dev server via npm run dev

Notes:
  - Open the UI URL, not the API URL
  - Gazebo logs: /tmp/bcr_browser_gazebo.log
  - Bridge logs: /tmp/bcr_browser_bridge.log
  - UI logs: /tmp/bcr_browser_ui.log
  - --detach leaves all three services running after this shell exits
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "${DETACH}" -eq 1 ]]; then
  nohup "${ROOT_DIR}/scripts/run_gazebo_software.sh" >"${GAZEBO_LOG}" 2>&1 < /dev/null &
  GAZEBO_PID=$!
  nohup "${ROOT_DIR}/scripts/run_browser_bridge.sh" --host "${HOST}" --port "${API_PORT}" >"${BRIDGE_LOG}" 2>&1 < /dev/null &
  BRIDGE_PID=$!
  nohup "${ROOT_DIR}/scripts/run_browser_vite.sh" \
    --host "${HOST}" \
    --port "${UI_PORT}" \
    --api-url "http://${HOST}:${API_PORT}" >"${UI_LOG}" 2>&1 < /dev/null &
  UI_PID=$!

  printf '%s\n' "${GAZEBO_PID}" > "${GAZEBO_PID_FILE}"
  printf '%s\n' "${BRIDGE_PID}" > "${BRIDGE_PID_FILE}"
  printf '%s\n' "${UI_PID}" > "${UI_PID_FILE}"

  echo "Browser stack started in detached mode."
  echo "UI: http://${HOST}:${UI_PORT}"
  echo "Gazebo pid ${GAZEBO_PID} (${GAZEBO_PID_FILE}), log: ${GAZEBO_LOG}"
  echo "Bridge pid ${BRIDGE_PID} (${BRIDGE_PID_FILE}), log: ${BRIDGE_LOG}"
  echo "UI pid ${UI_PID} (${UI_PID_FILE}), log: ${UI_LOG}"
  exit 0
fi

"${ROOT_DIR}/scripts/run_gazebo_software.sh" >"${GAZEBO_LOG}" 2>&1 &
GAZEBO_PID=$!

"${ROOT_DIR}/scripts/run_browser_bridge.sh" --host "${HOST}" --port "${API_PORT}" >"${BRIDGE_LOG}" 2>&1 &
BRIDGE_PID=$!

cleanup() {
  kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
  kill "${GAZEBO_PID}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

echo "Gazebo starting in background (pid ${GAZEBO_PID}), log: ${GAZEBO_LOG}"
echo "Browser bridge starting in background (pid ${BRIDGE_PID}), log: ${BRIDGE_LOG}"
echo "Starting Vite dev server on http://${HOST}:${UI_PORT} (log: ${UI_LOG})"

exec "${ROOT_DIR}/scripts/run_browser_vite.sh" \
  --host "${HOST}" \
  --port "${UI_PORT}" \
  --api-url "http://${HOST}:${API_PORT}" >>"${UI_LOG}" 2>&1
