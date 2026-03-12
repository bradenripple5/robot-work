#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: ./scripts/run_browser_bridge.sh [--host HOST] [--port PORT]

Examples:
  ./scripts/run_browser_bridge.sh
  ./scripts/run_browser_bridge.sh --port 8081
  ./scripts/run_browser_bridge.sh --host 0.0.0.0 --port 8090

Notes:
  - This starts the backend API only, not a browser UI
  - Default API URL is http://127.0.0.1:8080
  - If port 8080 is already in use, pick another port such as 8081
  - Use --host 0.0.0.0 if you want to access the page from another machine
EOF
  exit 0
fi

exec python3 "${ROOT_DIR}/install/bcr_arm_gazebo/lib/bcr_arm_gazebo/browser_bridge.py" "$@"
