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
    double goal_tolerance {0.01};            // 目标到位误差阈值
    double execution_timeout_sec {15.0};     // 默认整体执行超时时间
    double feedback_timeout_sec {1.0};       // joint_states 反馈超时时间
    double min_publish_interval_sec {0.02};  // 最小控制命令发送节拍
  };

  // ============================================================
  // 单次执行上下文
  // ============================================================
  struct ExecutionContext
  {
    bool active {false};                               // 当前是否有活动轨迹
    std::string task_id;                               // 当前任务 ID
    trajectory_msgs::msg::JointTrajectory trajectory;  // 当前执行轨迹
    size_t current_point_index {0};                    // 当前推进到的轨迹点索引
    rclcpp::Time start_time;                           // 当前任务开始时间
    double task_execution_timeout_sec {0.0};          // 本任务实际使用的超时阈值
  };

  // ============================================================
  // 单个控制周期推进结果
  // ============================================================
  struct StepResult
  {
    bool need_publish_command {false};     // 是否需要发布 /joint_cmd
    bool finished {false};                 // 是否执行完成
    bool has_error {false};                // 是否出现错误
    std::string message;                   // 当前状态描述
    std::vector<std::string> joint_names;  // 待发布命令的关节名
    std::vector<double> positions;         // 待发布命令的关节目标位置
    double point_interval_sec {0.0};       // 本次命令建议的执行节拍
    double current_error {0.0};            // 当前误差
  };

public:
  TrajectoryExecutor() = default;
  ~TrajectoryExecutor() = default;

  // 设置执行器配置
  void set_config(const Config & config);

  // 启动轨迹执行
  bool start(
    const std::string & task_id,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const rclcpp::Time & now,
    double task_timeout_sec,
    std::string & error_msg);

  // 停止当前执行
  void stop();

  // 更新 joint_states 缓存
  void update_joint_state(
    const sensor_msgs::msg::JointState & joint_state,
    const rclcpp::Time & now);

  // 推进一步执行
  StepResult step(const rclcpp::Time & now);

  // 查询是否有活动任务
  bool is_active() const;

  // 获取当前任务 ID
  const std::string & active_task_id() const;

  // 获取当前任务 ID（active_task_id 的兼容别名）
  const std::string & get_active_task_id() const;

  // 获取当前误差（相对于最终目标点）
  double current_error() const;

  // 获取当前轨迹点索引
  size_t current_point_index() const;

  // 获取轨迹总点数
  size_t total_points() const;

private:
  // 计算当前关节状态与目标点之间的最大绝对误差
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

  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_joint_state_ {false};
  rclcpp::Time last_joint_state_time_;
};

}  // namespace controller_pkg

#endif  // CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_