#ifndef PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_
#define PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_

#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace planner_pkg
{

// 简单关节规划器
// 负责根据当前关节角和目标关节角，生成关节空间线性插值轨迹
class SimpleJointPlanner
{
public:
  // 规划输入
  struct PlanRequest
  {
    std::string task_id;                        // 当前任务 ID
    std::vector<std::string> joint_names;       // 目标关节名
    std::vector<double> current_positions;      // 当前关节角
    std::vector<double> target_positions;       // 目标关节角
    double speed_scale {1.0};                   // 速度比例（当前版本预留）
    double timeout_sec {10.0};                  // 超时时间（当前版本预留）
  };

  // 规划输出
  struct PlanResult
  {
    bool success {false};                               // 是否规划成功
    std::string error_msg;                              // 失败原因
    trajectory_msgs::msg::JointTrajectory trajectory;   // 规划生成的轨迹
  };

public:
  SimpleJointPlanner() = default;
  ~SimpleJointPlanner() = default;

  // 设置规划参数
  void set_traj_points(int traj_points);
  void set_plan_duration(double plan_duration_sec);

  // 执行规划
  PlanResult plan(const PlanRequest & request) const;

private:
  // 轨迹点数
  int traj_points_ {50};

  // 规划总时长
  double plan_duration_sec_ {2.0};
};

}  // namespace planner_pkg

#endif  // PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_