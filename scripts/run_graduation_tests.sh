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
  $0 test-invalid
  $0 test-timeout
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

  source_workspace
  ros2 action send_goal "${ACTION_NAME}" "${ACTION_TYPE}" \
    "{task_name: '${task_name}', joint_names: ${JOINT_NAMES}, target_positions: ${positions}, speed_scale: ${speed_scale}, timeout_sec: ${timeout_sec}}" \
    --feedback
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
    send_goal "demo_large_motion_right_slow" "[0.90, -1.45, 1.75, -1.45, -1.57, 0.90]" "0.40" "25.0"
    ;;

  test-alt)
    send_goal "demo_large_motion_left_slow" "[-0.90, -0.95, 1.15, -1.65, -1.57, -0.90]" "0.40" "25.0"
    ;;

  test-slow)
    send_goal "demo_very_slow_motion" "[0.60, -1.55, 1.90, -1.20, -1.57, 1.20]" "0.20" "35.0"
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

  bag)
    source_workspace
    BAG_DIR="${2:-${WORKSPACE_DIR}/bags/graduation_test_$(date +%Y%m%d_%H%M%S)}"
    mkdir -p "$(dirname "${BAG_DIR}")"
    echo "Recording bag to: ${BAG_DIR}"
    exec ros2 bag record -o "${BAG_DIR}" ${BAG_TOPICS}
    ;;

  reset)
    source_workspace
    exec ros2 service call "${RESET_SERVICE}" "${RESET_TYPE}" "{force_reset: true}"
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
