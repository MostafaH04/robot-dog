#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros_env.sh"

WORKSPACE_SETUP="${PROJECT_ROOT}/robot_ws/install/setup.bash"
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Workspace is not built. Run 'make build' first." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${WORKSPACE_SETUP}"
set -u
exec ros2 launch robot_simulation master.launch.py "$@"
