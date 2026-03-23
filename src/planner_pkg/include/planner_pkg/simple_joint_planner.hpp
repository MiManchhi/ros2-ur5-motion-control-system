#ifndef PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_
#define PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_

#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace planner_pkg
{

// ============================================================
// 简单关节规划器
//
// 作用：
// 1. 根据当前关节角和目标关节角生成关节空间线性插值轨迹
// 2. 根据 speed_scale 调整轨迹总时长
// 3. 为控制层提供统一 JointTrajectory 输出
// ============================================================
class SimpleJointPlanner
{
public:
  // ============================================================
  // 规划输入
  // ============================================================
  struct PlanRequest
  {
    std::string task_id;                        // 当前任务 ID
    std::vector<std::string> joint_names;       // 目标关节名
    std::vector<double> current_positions;      // 当前关节角
    std::vector<double> target_positions;       // 目标关节角
    double speed_scale {1.0};                   // 速度缩放比例，0~1 区间有效
    double timeout_sec {10.0};                  // 任务级超时，当前主要透传给控制层
  };

  // ============================================================
  // 规划输出
  // ============================================================
  struct PlanResult
  {
    bool success {false};                               // 是否规划成功
    std::string error_msg;                              // 失败原因
    trajectory_msgs::msg::JointTrajectory trajectory;   // 规划生成的轨迹

    // 新增：用于日志和调试
    double actual_plan_duration_sec {0.0};              // 实际规划总时长
    double point_interval_sec {0.0};                    // 相邻轨迹点之间的时间间隔
  };

public:
  SimpleJointPlanner() = default;
  ~SimpleJointPlanner() = default;

  // 设置轨迹点数量
  void set_traj_points(int traj_points);

  // 设置基准规划总时长
  void set_plan_duration(double plan_duration_sec);

  // 执行规划
  PlanResult plan(const PlanRequest & request) const;

private:
  // 轨迹点数，至少为 2
  int traj_points_ {50};

  // 基准规划总时长
  double plan_duration_sec_ {2.0};
};

}  // namespace planner_pkg

#endif  // PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_