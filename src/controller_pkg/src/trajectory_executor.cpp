#include "controller_pkg/trajectory_executor.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>

namespace controller_pkg
{

namespace
{

// 将 ROS2 Duration 消息转换为 double 秒
double duration_msg_to_sec(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

}  // namespace

void TrajectoryExecutor::set_config(const Config & config)
{
  config_ = config;

  // ============================================================
  // 对非法参数做保护
  // ============================================================
  if (config_.goal_tolerance <= 0.0) {
    config_.goal_tolerance = 0.01;
  }
  if (config_.execution_timeout_sec <= 0.0) {
    config_.execution_timeout_sec = 15.0;
  }
  if (config_.feedback_timeout_sec <= 0.0) {
    config_.feedback_timeout_sec = 1.0;
  }
  if (config_.min_publish_interval_sec <= 0.0) {
    config_.min_publish_interval_sec = 0.02;
  }
}

bool TrajectoryExecutor::start(
  const std::string & task_id,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const rclcpp::Time & now,
  double task_timeout_sec,
  std::string & error_msg)
{
  // 当前已有活动轨迹，不允许重复启动
  if (exec_ctx_.active) {
    error_msg = "执行器当前忙碌，无法启动新轨迹";
    return false;
  }

  // task_id 不能为空
  if (task_id.empty()) {
    error_msg = "task_id 为空，无法启动执行";
    return false;
  }

  // 轨迹不能为空
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    error_msg = "轨迹为空，无法启动执行";
    return false;
  }

  // 初始化执行上下文
  exec_ctx_.active = true;
  exec_ctx_.task_id = task_id;
  exec_ctx_.trajectory = trajectory;
  exec_ctx_.current_point_index = 0;
  exec_ctx_.start_time = now;

  // 优先使用任务级超时，否则回退到配置默认值
  exec_ctx_.task_execution_timeout_sec =
    (task_timeout_sec > 0.0) ? task_timeout_sec : config_.execution_timeout_sec;

  return true;
}

void TrajectoryExecutor::stop()
{
  exec_ctx_.active = false;
  exec_ctx_.task_id.clear();
  exec_ctx_.trajectory.joint_names.clear();
  exec_ctx_.trajectory.points.clear();
  exec_ctx_.current_point_index = 0;
  exec_ctx_.task_execution_timeout_sec = 0.0;
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

  // ============================================================
  // 基础状态检查
  // ============================================================
  if (!exec_ctx_.active) {
    result.message = "当前无活动任务";
    return result;
  }

  result.progress = compute_progress();
  result.current_error = current_error();

  if (!has_joint_state_) {
    result.has_error = true;
    result.message = "尚未收到 joint_states，无法执行闭环控制";
    return result;
  }

  // joint_states 反馈超时
  const double feedback_elapsed = (now - last_joint_state_time_).seconds();
  if (feedback_elapsed > config_.feedback_timeout_sec) {
    result.has_error = true;
    result.message = "joint_states 反馈超时，停止执行";
    return result;
  }

  // 当前任务执行超时
  const double exec_elapsed = (now - exec_ctx_.start_time).seconds();
  if (exec_elapsed > exec_ctx_.task_execution_timeout_sec) {
    result.has_error = true;
    result.message = "轨迹执行超时，停止执行";
    return result;
  }

  // ============================================================
  // 若还有轨迹点未发送，则当前周期发送一个点
  // ============================================================
  if (exec_ctx_.current_point_index < exec_ctx_.trajectory.points.size()) {
    const auto & point = exec_ctx_.trajectory.points[exec_ctx_.current_point_index];

    // 单点 JointTrajectory 命令需要在“上一轨迹点时间”发布下一段运动：
    // point_interval_sec 表示从上一点运动到当前点的时长。
    double publish_time_sec = 0.0;
    if (exec_ctx_.current_point_index > 0U) {
      const auto & prev_point = exec_ctx_.trajectory.points[exec_ctx_.current_point_index - 1U];
      publish_time_sec = duration_msg_to_sec(prev_point.time_from_start);
    }

    if (exec_elapsed < publish_time_sec) {
      return result;
    }

    // 从轨迹时间中推导当前点的建议执行节拍
    double point_interval_sec = config_.min_publish_interval_sec;

    if (exec_ctx_.current_point_index == 0U) {
      // 第一个点通常 time_from_start=0，此时退回最小发送间隔
      const double first_time_sec = duration_msg_to_sec(point.time_from_start);
      if (first_time_sec > 0.0) {
        point_interval_sec = first_time_sec;
      }
    } else {
      const auto & prev_point = exec_ctx_.trajectory.points[exec_ctx_.current_point_index - 1U];
      const double curr_sec = duration_msg_to_sec(point.time_from_start);
      const double prev_sec = duration_msg_to_sec(prev_point.time_from_start);
      const double diff_sec = curr_sec - prev_sec;
      if (diff_sec > 0.0) {
        point_interval_sec = diff_sec;
      }
    }

    result.need_publish_command = true;
    result.joint_names = exec_ctx_.trajectory.joint_names;
    result.positions = point.positions;
    result.point_interval_sec = point_interval_sec;
    result.message = "发送下一个轨迹点";

    // 推进轨迹点索引
    ++exec_ctx_.current_point_index;
    result.progress = compute_progress();
    result.current_error = current_error();
    return result;
  }

  // ============================================================
  // 所有轨迹点都已发送，开始判断最终是否到位
  // ============================================================
  const auto & final_point = exec_ctx_.trajectory.points.back();
  result.current_error =
    compute_max_error(exec_ctx_.trajectory.joint_names, final_point.positions);

  if (is_goal_reached(exec_ctx_.trajectory.joint_names, final_point.positions)) {
    result.finished = true;
    result.message = "轨迹点全部发送完毕，机械臂已到达目标位置";
    return result;
  }

  // 点发完了，但还没到位
  result.message = "轨迹点已全部发送，当前仍在等待机械臂到位";
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

const std::string & TrajectoryExecutor::get_active_task_id() const
{
  return active_task_id();
}

double TrajectoryExecutor::current_error() const
{
  if (!exec_ctx_.active || exec_ctx_.trajectory.points.empty()) {
    return 0.0;
  }

  // 当前误差按最终目标点计算
  const auto & final_point = exec_ctx_.trajectory.points.back();
  return compute_max_error(exec_ctx_.trajectory.joint_names, final_point.positions);
}

size_t TrajectoryExecutor::current_point_index() const
{
  return exec_ctx_.current_point_index;
}

size_t TrajectoryExecutor::total_points() const
{
  return exec_ctx_.trajectory.points.size();
}

float TrajectoryExecutor::compute_progress() const
{
  if (!exec_ctx_.active) {
    return 0.0F;
  }

  const size_t total_points = exec_ctx_.trajectory.points.size();
  if (total_points == 0U) {
    return 0.0F;
  }

  // 控制层进度大致映射到 0.30 ~ 0.95，最终完成由 controller_node 发布 1.0。
  const double ratio =
    static_cast<double>(exec_ctx_.current_point_index) /
    static_cast<double>(total_points);

  double progress = 0.30 + ratio * 0.65;

  if (progress < 0.30) {
    progress = 0.30;
  }
  if (progress > 0.95) {
    progress = 0.95;
  }

  return static_cast<float>(progress);
}

double TrajectoryExecutor::compute_max_error(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & target_positions) const
{
  // 尚未收到反馈，无法计算误差
  if (!has_joint_state_) {
    return std::numeric_limits<double>::infinity();
  }

  // 输入维度不一致，视为异常
  if (joint_names.size() != target_positions.size()) {
    return std::numeric_limits<double>::infinity();
  }

  // joint_state 数据非法
  if (latest_joint_state_.name.size() != latest_joint_state_.position.size()) {
    return std::numeric_limits<double>::infinity();
  }

  // 构造 name -> position 映射
  std::unordered_map<std::string, double> joint_map;
  joint_map.reserve(latest_joint_state_.name.size());

  for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
    joint_map[latest_joint_state_.name[i]] = latest_joint_state_.position[i];
  }

  // 逐关节计算绝对误差，取最大值
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
