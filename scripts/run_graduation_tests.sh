#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
SIM_WORKSPACE_DIR="${SIM_WORKSPACE_DIR:-${HOME}/workspaces/ur_gazebo}"
DEFAULT_SIM_LAUNCH_FILE="${SIM_WORKSPACE_DIR}/src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/launch/ur_sim_control.launch.py"

ACTION_NAME="/move_joints"
ACTION_TYPE="robot_motion_msgs/action/MoveJoints"
RESET_SERVICE="/reset_system"
RESET_TYPE="robot_motion_msgs/srv/ResetSystem"

JOINT_NAMES="[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]"
BAG_TOPICS="/joint_states /planned_traj /joint_cmd /task_state /system_state /motion_event"
BASE_PLAN_DURATION_SEC="${BASE_PLAN_DURATION_SEC:-5.0}"

source_workspace() {
  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${WORKSPACE_DIR}/install/setup.bash"
    set -u
  else
    echo "ERROR: ${WORKSPACE_DIR}/install/setup.bash not found."
    echo "Build first: cd ${WORKSPACE_DIR} && colcon build"
    exit 1
  fi
}

source_sim_workspace() {
  if [[ -f "${SIM_WORKSPACE_DIR}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${SIM_WORKSPACE_DIR}/install/setup.bash"
    set -u
  else
    echo "ERROR: ${SIM_WORKSPACE_DIR}/install/setup.bash not found."
    echo "Build the simulation workspace first: cd ${SIM_WORKSPACE_DIR} && colcon build"
    exit 1
  fi
}

print_usage() {
  cat <<EOF
Usage:
  $0 launch-system
  $0 launch-sim [absolute_sim_launch_file]
  $0 test-normal
  $0 test-alt
  $0 test-slow
  $0 test-fast
  $0 test-invalid
  $0 test-timeout
  $0 test-recovery
  $0 test-abnormal-suite
  $0 bag [bag_output_dir]
  $0 reset

Environment:
  WORKSPACE_DIR=${WORKSPACE_DIR}
  SIM_WORKSPACE_DIR=${SIM_WORKSPACE_DIR}

Notes:
  - Run launch, bag recording, and action tests in separate terminals.
  - launch-sim starts Gazebo by default using:
      ${DEFAULT_SIM_LAUNCH_FILE}
  - launch-sim opens Gazebo GUI and RViz by default for manual observation.
  - If either workspace path changes, run:
      WORKSPACE_DIR=/path/to/ros2_ws SIM_WORKSPACE_DIR=/path/to/ur_gazebo $0 <command>
EOF
}

send_goal() {
  local task_name="$1"
  local positions="$2"
  local speed_scale="$3"
  local timeout_sec="$4"
  local expected_duration_sec

  source_workspace
  expected_duration_sec="$(awk -v base="${BASE_PLAN_DURATION_SEC}" -v speed="${speed_scale}" 'BEGIN {printf "%.3f", base / speed}')"

  echo "Task: ${task_name}"
  echo "Target positions: ${positions}"
  echo "speed_scale=${speed_scale}, timeout_sec=${timeout_sec}, expected_planned_duration=${expected_duration_sec}s"

  local start_ns
  local end_ns
  local elapsed_sec
  local status

  start_ns="$(date +%s%N)"
  set +e
  ros2 action send_goal "${ACTION_NAME}" "${ACTION_TYPE}" \
    "{task_name: '${task_name}', joint_names: ${JOINT_NAMES}, target_positions: ${positions}, speed_scale: ${speed_scale}, timeout_sec: ${timeout_sec}}" \
    --feedback
  status=$?
  set -e
  end_ns="$(date +%s%N)"
  elapsed_sec="$(awk -v start="${start_ns}" -v end="${end_ns}" 'BEGIN {printf "%.3f", (end - start) / 1000000000.0}')"

  echo "Actual action elapsed: ${elapsed_sec}s"
  echo "Action command exit code: ${status}"

  return "${status}"
}

call_reset_system() {
  source_workspace

  local start_ns
  local end_ns
  local elapsed_sec
  local status

  echo "Calling reset service: ${RESET_SERVICE}"
  start_ns="$(date +%s%N)"
  set +e
  ros2 service call "${RESET_SERVICE}" "${RESET_TYPE}" "{force_reset: true}"
  status=$?
  set -e
  end_ns="$(date +%s%N)"
  elapsed_sec="$(awk -v start="${start_ns}" -v end="${end_ns}" 'BEGIN {printf "%.3f", (end - start) / 1000000000.0}')"

  echo "Reset elapsed: ${elapsed_sec}s"
  echo "Reset command exit code: ${status}"

  return "${status}"
}

case "${1:-}" in
  launch-system)
    source_workspace
    exec ros2 launch bringup_pkg bringup.launch.py \
      launch_mode:=system_only \
      use_sim_time:=false \
      log_level:=info
    ;;

  launch-sim)
    source_sim_workspace
    source_workspace
    SIM_LAUNCH_FILE="${2:-${DEFAULT_SIM_LAUNCH_FILE}}"
    if [[ ! -f "${SIM_LAUNCH_FILE}" ]]; then
      echo "ERROR: simulation launch file not found: ${SIM_LAUNCH_FILE}"
      echo "Pass an absolute launch file path, or set SIM_WORKSPACE_DIR."
      exit 1
    fi
    exec ros2 launch bringup_pkg bringup.launch.py \
      launch_mode:=sim_with_system \
      start_sim:=true \
      sim_launch_file:="${SIM_LAUNCH_FILE}" \
      use_sim_time:=true \
      log_level:=info
    ;;

  test-normal)
    send_goal "demo_large_motion_normal" "[-1.15, -1.05, 1.15, -1.80, -1.57, -1.35]" "0.65" "18.0"
    ;;

  test-alt)
    send_goal "demo_large_motion_alt" "[1.15, -1.60, 2.05, -1.15, -1.57, 1.50]" "0.65" "18.0"
    ;;

  test-slow)
    send_goal "demo_slow_motion" "[1.20, -1.65, 2.10, -1.15, -1.57, 1.65]" "0.35" "25.0"
    ;;

  test-fast)
    send_goal "demo_fast_motion" "[-1.20, -0.95, 1.05, -1.85, -1.57, -1.65]" "1.00" "12.0"
    ;;

  test-invalid)
    source_workspace
    ros2 action send_goal "${ACTION_NAME}" "${ACTION_TYPE}" \
      "{task_name: 'test_invalid_mismatch', joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint], target_positions: [0.0, -1.20, 1.40, -1.20, -1.57, 0.0], speed_scale: 1.0, timeout_sec: 10.0}" \
      --feedback
    ;;

  test-timeout)
    send_goal "test_timeout_0_5_sec" "[1.00, -1.55, 1.85, -1.45, -1.57, 1.00]" "0.40" "0.5"
    ;;

  test-recovery)
    echo "[1/3] Triggering execution timeout"
    set +e
    send_goal "test_timeout_0_5_sec" "[1.00, -1.55, 1.85, -1.45, -1.57, 1.00]" "0.40" "0.5"
    timeout_status=$?
    set -e
    echo "Timeout test command exit code: ${timeout_status}"

    echo "[2/3] Resetting system"
    sleep 2
    call_reset_system

    echo "[3/3] Sending recovery motion"
    sleep 1
    send_goal "demo_recovery_after_timeout" "[0.00, -1.25, 1.45, -1.35, -1.57, 0.00]" "0.85" "12.0"
    ;;

  test-abnormal-suite)
    echo "[1/4] Sending invalid goal; expected result: rejected/failed"
    source_workspace
    set +e
    ros2 action send_goal "${ACTION_NAME}" "${ACTION_TYPE}" \
      "{task_name: 'test_invalid_mismatch', joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint], target_positions: [0.0, -1.20, 1.40, -1.20, -1.57, 0.0], speed_scale: 1.0, timeout_sec: 10.0}" \
      --feedback
    invalid_status=$?
    set -e
    echo "Invalid goal command exit code: ${invalid_status}"

    echo "[2/4] Triggering execution timeout"
    set +e
    send_goal "test_timeout_0_5_sec" "[1.00, -1.55, 1.85, -1.45, -1.57, 1.00]" "0.40" "0.5"
    timeout_status=$?
    set -e
    echo "Timeout test command exit code: ${timeout_status}"

    echo "[3/4] Resetting system"
    sleep 2
    call_reset_system

    echo "[4/4] Sending normal motion after abnormal cases"
    sleep 1
    send_goal "demo_recovery_after_abnormal_suite" "[0.00, -1.25, 1.45, -1.35, -1.57, 0.00]" "0.85" "12.0"
    ;;

  bag)
    source_workspace
    BAG_DIR="${2:-${WORKSPACE_DIR}/bags/graduation_test_$(date +%Y%m%d_%H%M%S)}"
    mkdir -p "$(dirname "${BAG_DIR}")"
    echo "Recording bag to: ${BAG_DIR}"
    exec ros2 bag record -o "${BAG_DIR}" ${BAG_TOPICS}
    ;;

  reset)
    call_reset_system
    ;;

  ""|-h|--help|help)
    print_usage
    ;;

  *)
    echo "Unknown command: $1"
    print_usage
    exit 1
    ;;
esac
