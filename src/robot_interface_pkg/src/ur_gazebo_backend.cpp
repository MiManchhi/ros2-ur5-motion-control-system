#include "robot_interface_pkg/ur_gazebo_backend.hpp"

#include "rclcpp/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace robot_interface_pkg
{

void UrGazeboBackend::set_config(const Config & config)
{
  config_ = config;

  if (config_.controller_topic.empty()) {
    config_.controller_topic = "/joint_trajectory_controller/joint_trajectory";
  }

  if (config_.point_time_from_start_sec <= 0.0) {
    config_.point_time_from_start_sec = 0.1;
  }
}

bool UrGazeboBackend::convert_command(
  const robot_motion_msgs::msg::MotionCommand & cmd,
  trajectory_msgs::msg::JointTrajectory & traj,
  std::string & error_msg) const
{
  // =========================
  // 输入校验
  // =========================
  if (cmd.task_id.empty()) {
    error_msg = "task_id 为空";
    return false;
  }

  if (cmd.joint_names.empty()) {
    error_msg = "joint_names 为空";
    return false;
  }

  if (cmd.positions.empty()) {
    error_msg = "positions 为空";
    return false;
  }

  if (cmd.joint_names.size() != cmd.positions.size()) {
    error_msg = "joint_names 与 positions 数量不一致";
    return false;
  }

  // =========================
  // 转换为标准 JointTrajectory
  // 这里采用“单点轨迹命令”的方式下发到底层控制器
  // =========================
  traj.joint_names = cmd.joint_names;
  traj.points.clear();
  traj.points.reserve(1);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = cmd.positions;
  point.time_from_start = rclcpp::Duration::from_seconds(
    config_.point_time_from_start_sec);

  traj.points.push_back(point);

  return true;
}

const std::string & UrGazeboBackend::controller_topic() const
{
  return config_.controller_topic;
}

}  // namespace robot_interface_pkg