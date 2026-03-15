#ifndef CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_
#define CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_

#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace controller_pkg
{

class TrajectoryExecutor
{
public:
  // ============================================================
  // 执行器配置参数
  // ============================================================
  struct Config
  {
    double goal_tolerance {0.01};          // 目标到位误差阈值
    double execution_timeout_sec {15.0};   // 整体执行超时时间
    double feedback_timeout_sec {1.0};     // joint_states 反馈超时时间
  };

  // ============================================================
  // 单次执行上下文
  // 用于保存当前正在执行的任务轨迹和执行进度
  // ============================================================
  struct ExecutionContext
  {
    bool active {false};                               // 当前是否存在活动轨迹
    std::string task_id;                               // 当前任务 ID
    trajectory_msgs::msg::JointTrajectory trajectory;  // 当前执行轨迹
    size_t current_point_index {0};                    // 当前已推进到的轨迹点索引
    rclcpp::Time start_time;                           // 当前任务开始时间
  };

  // ============================================================
  // 单个控制周期推进后的结果
  // controller_node 会根据这个结果决定：
  // 1. 是否发布 /joint_cmd
  // 2. 是否宣布任务完成
  // 3. 是否进入失败链路
  // ============================================================
  struct StepResult
  {
    bool need_publish_command {false};     // 当前周期是否需要发布控制命令
    bool finished {false};                 // 当前任务是否完成
    bool has_error {false};                // 当前是否出现错误
    std::string message;                   // 状态描述信息
    std::vector<std::string> joint_names;  // 待发布命令中的关节名
    std::vector<double> positions;         // 待发布命令中的目标位置
    double current_error {0.0};            // 当前误差
  };

public:
  TrajectoryExecutor() = default;
  ~TrajectoryExecutor() = default;

  // 设置执行器配置参数
  void set_config(const Config & config);

  // 启动一个新轨迹执行
  bool start(
    const std::string & task_id,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const rclcpp::Time & now,
    std::string & error_msg);

  // 停止当前执行
  void stop();

  // 更新 joint_states 缓存
  void update_joint_state(
    const sensor_msgs::msg::JointState & joint_state,
    const rclcpp::Time & now);

  // 在一个控制周期中推进执行
  StepResult step(const rclcpp::Time & now);

  // 查询当前是否有活动任务
  bool is_active() const;

  // 获取当前任务 ID
  const std::string & active_task_id() const;

  // 获取当前误差（相对于最终目标点）
  double current_error() const;

  // 获取当前轨迹点索引
  size_t current_point_index() const;

  // 获取轨迹总点数
  size_t total_points() const;

private:
  // 计算当前 joint_states 与目标关节位置之间的最大绝对误差
  double compute_max_error(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target_positions) const;

  // 判断是否到达最终目标
  bool is_goal_reached(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target_positions) const;

private:
  Config config_;

  ExecutionContext exec_ctx_;

  sensor_msgs::msg::JointState latest_joint_state_; // 最近一次 joint_states
  bool has_joint_state_ {false};                    // 是否收到过 joint_states
  rclcpp::Time last_joint_state_time_;              // 最近一次 joint_states 时间
};

}  // namespace controller_pkg

#endif  // CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_