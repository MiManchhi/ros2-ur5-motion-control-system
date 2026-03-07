#ifndef SYSTEM_MANAGER_PKG__STATE_MACHINE_HPP_
#define SYSTEM_MANAGER_PKG__STATE_MACHINE_HPP_

#include <string>

class StateMachine
{
public:
  StateMachine();

  // 设置当前状态
  void set_state(const std::string & state);

  // 获取当前状态
  const std::string & get_state() const;

  // 重置状态机
  void reset();

private:
  std::string current_state_;
};

#endif  // SYSTEM_MANAGER_PKG__STATE_MACHINE_HPP_