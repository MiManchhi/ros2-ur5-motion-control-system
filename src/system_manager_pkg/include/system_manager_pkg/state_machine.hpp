#ifndef SYSTEM_MANAGER_PKG__STATE_MACHINE_HPP_
#define SYSTEM_MANAGER_PKG__STATE_MACHINE_HPP_

#include <string>

namespace system_manager_pkg
{

// 系统状态机
// 负责纯状态流转逻辑，不依赖 ROS2 通信
class StateMachine
{
public:
  // 系统状态定义
  enum class State
  {
    INIT,       // 初始化中
    IDLE,       // 空闲
    PLANNING,   // 规划中
    RUNNING,    // 执行中
    FINISHED,   // 已完成
    CANCELLED,  // 已取消
    ERROR       // 错误
  };

public:
  StateMachine();
  ~StateMachine() = default;

  // 获取当前状态
  State current_state() const;

  // 获取当前状态字符串
  std::string current_state_string() const;

  // 重置到 IDLE
  void reset();

  // 请求状态切换
  // 返回 true 表示切换成功，false 表示切换非法
  bool transition_to(State new_state);

  // 状态与字符串互转
  static std::string to_string(State state);
  static bool from_string(const std::string & state_str, State & state);

private:
  // 判断当前状态是否允许切换到目标状态
  bool is_valid_transition(State from, State to) const;

private:
  State current_state_;
};

}  // namespace system_manager_pkg

#endif  // SYSTEM_MANAGER_PKG__STATE_MACHINE_HPP_