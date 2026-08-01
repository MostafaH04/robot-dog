#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Install ROS 2 ${ROS_DISTRO}, use the devcontainer, or set ROS_SETUP." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${ROS_SETUP}"

if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi
set -u

export PROJECT_ROOT ROS_DISTRO
