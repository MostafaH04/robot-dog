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

SMOKE_LOG="$(mktemp -t robot-dog-smoke.XXXXXX.log)"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        break
      fi
      sleep 0.5
    done
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    fi
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  rm -f "${SMOKE_LOG}"
}
trap cleanup EXIT

ros2 launch robot_simulation experiment.launch.py \
  use_foxglove:=true \
  use_rviz:=false >"${SMOKE_LOG}" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 20); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Simulation launch exited before becoming ready." >&2
    cat "${SMOKE_LOG}" >&2
    exit 1
  fi

  TOPICS="$(ros2 topic list 2>/dev/null || true)"
  NODES="$(ros2 node list 2>/dev/null || true)"
  if grep -qx '/clock' <<<"${TOPICS}" \
      && grep -qx '/control_inputs' <<<"${TOPICS}" \
      && grep -qx '/cmd_jnts' <<<"${TOPICS}" \
      && grep -qx '/joint_states' <<<"${TOPICS}" \
      && grep -qx '/tf' <<<"${TOPICS}" \
      && grep -qx '/sim/experiment/ready' <<<"${TOPICS}" \
      && grep -qx '/sim/ground_truth/odom' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/imu' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/joint_states' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/foot_contacts/front_left' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/foot_contacts/front_left/normal_force' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/foot_contacts/front_left/wrench' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/foot_contacts/front_right' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/foot_contacts/rear_left' <<<"${TOPICS}" \
      && grep -qx '/sim/sensors/foot_contacts/rear_right' <<<"${TOPICS}" \
      && grep -qx '/foxglove_bridge' <<<"${NODES}" \
      && grep -qx '/example_stance_controller' <<<"${NODES}" \
      && grep -qx '/quad_controller' <<<"${NODES}" \
      && grep -qx '/quad_sim' <<<"${NODES}" \
      && grep -qx '/robot_state_publisher' <<<"${NODES}" \
      && grep -qx '/state_interface_monitor' <<<"${NODES}"; then
    READY_STATUS="$(
      timeout 3 ros2 topic echo \
        --once /sim/experiment/ready std_msgs/msg/Bool 2>/dev/null || true
    )"
    CONTROLLER_SIM_TIME="$(
      ros2 param get /example_stance_controller use_sim_time 2>/dev/null || true
    )"
    MONITOR_SIM_TIME="$(
      ros2 param get /state_interface_monitor use_sim_time 2>/dev/null || true
    )"
    SIMULATOR_SIM_TIME="$(
      ros2 param get /quad_sim use_sim_time 2>/dev/null || true
    )"
    GROUND_TRUTH_TF="$(
      ros2 param get /quad_sim publish_ground_truth_tf 2>/dev/null || true
    )"
    GROUND_TRUTH_SAMPLE="$(
      timeout 3 ros2 topic echo \
        --once /sim/ground_truth/odom nav_msgs/msg/Odometry 2>/dev/null || true
    )"
    TF_SAMPLE="$(
      timeout 3 ros2 topic echo \
        --once /tf tf2_msgs/msg/TFMessage 2>/dev/null || true
    )"
    if grep -q 'data: true' <<<"${READY_STATUS}" \
        && grep -q 'True' <<<"${CONTROLLER_SIM_TIME}" \
        && grep -q 'True' <<<"${MONITOR_SIM_TIME}" \
        && grep -q 'False' <<<"${SIMULATOR_SIM_TIME}" \
        && grep -q 'False' <<<"${GROUND_TRUTH_TF}" \
        && grep -q 'frame_id: sim_ground_truth_world' <<<"${GROUND_TRUTH_SAMPLE}" \
        && grep -q 'child_frame_id: sim_ground_truth_base_link' \
          <<<"${GROUND_TRUTH_SAMPLE}" \
        && grep -q 'transforms:' <<<"${TF_SAMPLE}" \
        && ! grep -q 'child_frame_id: base_link' <<<"${TF_SAMPLE}" \
        && ! grep -q 'sim_ground_truth_' <<<"${TF_SAMPLE}" \
        && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      echo "Simulation smoke test passed: step clock, clean TF, commands, sensors, contact forces, and isolated ground truth are coherent."
      exit 0
    fi
  fi
  sleep 1
done

echo "Experiment nodes and topics did not become ready within 20 seconds." >&2
cat "${SMOKE_LOG}" >&2
exit 1
