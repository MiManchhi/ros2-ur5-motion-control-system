#ifndef SYSTEM_MANAGER_PKG__TASK_STATE_MACHINE_HPP_
#define SYSTEM_MANAGER_PKG__TASK_STATE_MACHINE_HPP_

#include <string>

namespace system_manager_pkg
{

// ============================================================
// TaskStateMachine
// 用于管理单个任务的生命周期状态流转
//
// 设计目标：
// 1. 让 task_id 拥有独立的正式状态机
// 2. 与 system_state 解耦
// 3. 由 system_manager_node 使用，统一收敛 motion_event
// ============================================================
class TaskStateMachine
{
public:
  // 任务状态枚举
  enum class State
  {
    ACCEPTED,   // 任务已接收
    PLANNING,   // 规划中
    EXECUTING,  // 执行中
    COMPLETED,  // 已完成
    CANCELED,   // 已取消
    FAILED,     // 执行失败
    REJECTED    // 已拒绝
  };

public:
  TaskStateMachine();
  ~TaskStateMachine() = default;

  // 获取当前状态枚举值
  State current_state() const;

  // 获取当前状态的字符串形式
  std::string current_state_string() const;

  // 重置状态机
  // 默认重置到 ACCEPTED，表示新任务已进入正式管理流程
  void reset(State state = State::ACCEPTED);

  // 尝试切换到新状态
  // 若流转合法，返回 true；否则返回 false
  bool transition_to(State new_state);

  // 枚举转字符串
  static std::string to_string(State state);

  // 字符串转枚举
  static bool from_string(const std::string & state_str, State & state);

  // 判断是否为终态
  static bool is_terminal(State state);

private:
  // 判断状态流转是否合法
  bool is_valid_transition(State from, State to) const;

private:
  State current_state_;
};

}  // namespace system_manager_pkg

#endif  // SYSTEM_MANAGER_PKG__TASK_STATE_MACHINE_HPP_