#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Use the devcontainer or install ROS 2 ${ROS_DISTRO} first." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${ROS_SETUP}"
set -u

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep is required. Install python3-rosdep and retry." >&2
  exit 1
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  if [[ "$(id -u)" -eq 0 ]]; then
    rosdep init
  elif command -v sudo >/dev/null 2>&1; then
    sudo rosdep init
  else
    echo "rosdep is not initialized and sudo is unavailable." >&2
    exit 1
  fi
fi

rosdep update --rosdistro "${ROS_DISTRO}"

if command -v apt-get >/dev/null 2>&1; then
  if [[ "$(id -u)" -eq 0 ]]; then
    apt-get update
  elif command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
  else
    echo "APT package indexes must be updated, but sudo is unavailable." >&2
    exit 1
  fi
fi

rosdep install \
  --from-paths "${PROJECT_ROOT}/robot_ws/src" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  -y

python3 -m venv --system-site-packages "${PROJECT_ROOT}/.venv"
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/.venv/bin/activate"
python -m pip install --upgrade pip==25.1.1
python -m pip install -r "${PROJECT_ROOT}/requirements.txt"

echo "Dependencies are ready. Run 'make build' next."
