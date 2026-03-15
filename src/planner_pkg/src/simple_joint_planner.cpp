#include "planner_pkg/simple_joint_planner.hpp"
#include "rclcpp/duration.hpp"

namespace planner_pkg
{

void SimpleJointPlanner::set_traj_points(int traj_points)
{
  // 至少需要两个点：起点和终点
  if (traj_points <= 1) {
    traj_points_ = 2;
    return;
  }

  traj_points_ = traj_points;
}

void SimpleJointPlanner::set_plan_duration(double plan_duration_sec)
{
  // 规划总时长必须大于 0
  if (plan_duration_sec <= 0.0) {
    plan_duration_sec_ = 2.0;
    return;
  }

  plan_duration_sec_ = plan_duration_sec;
}

SimpleJointPlanner::PlanResult SimpleJointPlanner::plan(const PlanRequest & request) const
{
  PlanResult result;

  // =========================
  // 输入校验
  // =========================
  if (request.joint_names.empty()) {
    result.error_msg = "joint_names 为空";
    return result;
  }

  if (request.current_positions.empty()) {
    result.error_msg = "current_positions 为空";
    return result;
  }

  if (request.target_positions.empty()) {
    result.error_msg = "target_positions 为空";
    return result;
  }

  if (request.joint_names.size() != request.current_positions.size() ||
      request.joint_names.size() != request.target_positions.size()) {
    result.error_msg = "规划输入维度不一致";
    return result;
  }

  // =========================
  // 初始化轨迹
  // =========================
  auto & traj = result.trajectory;
  traj.joint_names = request.joint_names;
  traj.points.clear();
  traj.points.reserve(static_cast<size_t>(traj_points_));

  // 将总时长均匀分配到每个轨迹点
  const double dt = plan_duration_sec_ / static_cast<double>(traj_points_ - 1);

  // =========================
  // 使用关节空间线性插值生成轨迹
  // q_i = q_start + ratio * (q_goal - q_start)
  // =========================
  for (int i = 0; i < traj_points_; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(traj_points_ - 1);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(request.joint_names.size());

    for (size_t j = 0; j < request.joint_names.size(); ++j) {
      point.positions[j] =
        request.current_positions[j] +
        ratio * (request.target_positions[j] - request.current_positions[j]);
    }

    // 设置当前轨迹点相对起始时刻的时间
    const double time_from_start_sec = dt * static_cast<double>(i);
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start_sec);

    traj.points.push_back(point);
  }

  result.success = true;
  return result;
}

}  // namespace planner_pkg