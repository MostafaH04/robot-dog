#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="python3"
if [[ -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
  PYTHON_BIN="${PROJECT_ROOT}/.venv/bin/python"
fi
OUTPUT_DIRECTORY="${1:-${PROJECT_ROOT}/robot_ws/log/mujoco-review}"

export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONPATH="${PROJECT_ROOT}/robot_ws/src/robot_simulation${PYTHONPATH:+:${PYTHONPATH}}"
"${PYTHON_BIN}" -m robot_simulation.mujoco_record "${OUTPUT_DIRECTORY}"
