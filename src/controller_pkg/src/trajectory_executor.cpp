#include "controller_pkg/trajectory_executor.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>

namespace controller_pkg
{

void TrajectoryExecutor::set_config(const Config & config)
{
  config_ = config;

  if (config_.goal_tolerance <= 0.0) {
    config_.goal_tolerance = 0.01;
  }
  if (config_.execution_timeout_sec <= 0.0) {
    config_.execution_timeout_sec = 15.0;
  }
  if (config_.feedback_timeout_sec <= 0.0) {
    config_.feedback_timeout_sec = 1.0;
  }
}

bool TrajectoryExecutor::start(
  const std::string & task_id,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const rclcpp::Time & now,
  std::string & error_msg)
{
  if (exec_ctx_.active) {
    error_msg = "执行器当前忙碌";
    return false;
  }

  if (task_id.empty()) {
    error_msg = "task_id 为空";
    return false;
  }

  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    error_msg = "轨迹为空";
    return false;
  }

  exec_ctx_.active = true;
  exec_ctx_.task_id = task_id;
  exec_ctx_.trajectory = trajectory;
  exec_ctx_.current_point_index = 0;
  exec_ctx_.start_time = now;

  return true;
}

void TrajectoryExecutor::stop()
{
  exec_ctx_.active = false;
  exec_ctx_.task_id.clear();
  exec_ctx_.trajectory.joint_names.clear();
  exec_ctx_.trajectory.points.clear();
  exec_ctx_.current_point_index = 0;
}

void TrajectoryExecutor::update_joint_state(
  const sensor_msgs::msg::JointState & joint_state,
  const rclcpp::Time & now)
{
  latest_joint_state_ = joint_state;
  has_joint_state_ = true;
  last_joint_state_time_ = now;
}

TrajectoryExecutor::StepResult TrajectoryExecutor::step(const rclcpp::Time & now)
{
  StepResult result;

  if (!exec_ctx_.active) {
    result.message = "当前无活动任务";
    return result;
  }

  if (!has_joint_state_) {
    result.has_error = true;
    result.message = "尚未收到 joint_states";
    return result;
  }

  const double feedback_elapsed = (now - last_joint_state_time_).seconds();
  if (feedback_elapsed > config_.feedback_timeout_sec) {
    result.has_error = true;
    result.message = "joint_states 反馈超时";
    return result;
  }

  const double exec_elapsed = (now - exec_ctx_.start_time).seconds();
  if (exec_elapsed > config_.execution_timeout_sec) {
    result.has_error = true;
    result.message = "轨迹执行超时";
    return result;
  }

  // 如果还有轨迹点未发送，则当前周期发送一个点
  if (exec_ctx_.current_point_index < exec_ctx_.trajectory.points.size()) {
    const auto & point = exec_ctx_.trajectory.points[exec_ctx_.current_point_index];

    result.need_publish_command = true;
    result.joint_names = exec_ctx_.trajectory.joint_names;
    result.positions = point.positions;
    result.message = "发送轨迹点";

    ++exec_ctx_.current_point_index;
    return result;
  }

  // 所有轨迹点发完后，检查最终是否到位
  const auto & final_point = exec_ctx_.trajectory.points.back();
  result.current_error =
    compute_max_error(exec_ctx_.trajectory.joint_names, final_point.positions);

  if (is_goal_reached(exec_ctx_.trajectory.joint_names, final_point.positions)) {
    result.finished = true;
    result.message = "轨迹执行完成，已到达目标位置";
    return result;
  }

  result.message = "轨迹点已全部发送，等待到位";
  return result;
}

bool TrajectoryExecutor::is_active() const
{
  return exec_ctx_.active;
}

const std::string & TrajectoryExecutor::active_task_id() const
{
  return exec_ctx_.task_id;
}

double TrajectoryExecutor::current_error() const
{
  if (!exec_ctx_.active || exec_ctx_.trajectory.points.empty()) {
    return 0.0;
  }

  const auto & final_point = exec_ctx_.trajectory.points.back();
  return compute_max_error(exec_ctx_.trajectory.joint_names, final_point.positions);
}

double TrajectoryExecutor::compute_max_error(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & target_positions) const
{
  if (!has_joint_state_) {
    return std::numeric_limits<double>::infinity();
  }

  if (joint_names.size() != target_positions.size()) {
    return std::numeric_limits<double>::infinity();
  }

  if (latest_joint_state_.name.size() != latest_joint_state_.position.size()) {
    return std::numeric_limits<double>::infinity();
  }

  std::unordered_map<std::string, double> joint_map;
  joint_map.reserve(latest_joint_state_.name.size());

  for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
    joint_map[latest_joint_state_.name[i]] = latest_joint_state_.position[i];
  }

  double max_error = 0.0;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    auto it = joint_map.find(joint_names[i]);
    if (it == joint_map.end()) {
      return std::numeric_limits<double>::infinity();
    }

    const double err = std::fabs(it->second - target_positions[i]);
    if (err > max_error) {
      max_error = err;
    }
  }

  return max_error;
}

bool TrajectoryExecutor::is_goal_reached(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & target_positions) const
{
  return compute_max_error(joint_names, target_positions) < config_.goal_tolerance;
}

}  // namespace controller_pkg