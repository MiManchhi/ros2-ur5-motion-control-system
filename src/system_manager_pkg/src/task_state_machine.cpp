#include "system_manager_pkg/task_state_machine.hpp"

#include "robot_common_pkg/constants.hpp"

namespace system_manager_pkg
{

namespace c = robot_common_pkg::constants;

TaskStateMachine::TaskStateMachine()
: current_state_(State::ACCEPTED)
{
}

// 获取当前状态
TaskStateMachine::State TaskStateMachine::current_state() const
{
  return current_state_;
}

// 获取当前状态字符串
std::string TaskStateMachine::current_state_string() const
{
  return to_string(current_state_);
}

// 重置状态机
void TaskStateMachine::reset(State state)
{
  current_state_ = state;
}

// 状态切换
bool TaskStateMachine::transition_to(State new_state)
{
  if (!is_valid_transition(current_state_, new_state)) {
    return false;
  }

  current_state_ = new_state;
  return true;
}

// 枚举转字符串
std::string TaskStateMachine::to_string(State state)
{
  switch (state) {
    case State::ACCEPTED:
      return c::task_state::kAccepted;
    case State::PLANNING:
      return c::task_state::kPlanning;
    case State::EXECUTING:
      return c::task_state::kExecuting;
    case State::COMPLETED:
      return c::task_state::kCompleted;
    case State::CANCELED:
      return c::task_state::kCanceled;
    case State::FAILED:
      return c::task_state::kFailed;
    case State::REJECTED:
      return c::task_state::kRejected;
    default:
      return c::task_state::kFailed;
  }
}

// 字符串转枚举
bool TaskStateMachine::from_string(const std::string & state_str, State & state)
{
  if (state_str == c::task_state::kAccepted) {
    state = State::ACCEPTED;
    return true;
  }
  if (state_str == c::task_state::kPlanning) {
    state = State::PLANNING;
    return true;
  }
  if (state_str == c::task_state::kExecuting) {
    state = State::EXECUTING;
    return true;
  }
  if (state_str == c::task_state::kCompleted) {
    state = State::COMPLETED;
    return true;
  }
  if (state_str == c::task_state::kCanceled) {
    state = State::CANCELED;
    return true;
  }
  if (state_str == c::task_state::kFailed) {
    state = State::FAILED;
    return true;
  }
  if (state_str == c::task_state::kRejected) {
    state = State::REJECTED;
    return true;
  }

  return false;
}

// 判断是否为终态
bool TaskStateMachine::is_terminal(State state)
{
  return state == State::COMPLETED ||
         state == State::CANCELED ||
         state == State::FAILED ||
         state == State::REJECTED;
}

// 判断状态流转是否合法
bool TaskStateMachine::is_valid_transition(State from, State to) const
{
  // 允许重复状态，用于幂等处理
  if (from == to) {
    return true;
  }

  switch (from) {
    case State::ACCEPTED:
      // 任务已接收后，可以开始规划、被取消、被拒绝或失败
      return to == State::PLANNING ||
             to == State::CANCELED ||
             to == State::REJECTED ||
             to == State::FAILED;

    case State::PLANNING:
      // 规划完成后进入执行；规划期间也可能取消或失败
      return to == State::EXECUTING ||
             to == State::CANCELED ||
             to == State::FAILED;

    case State::EXECUTING:
      // 执行中可完成、取消或失败
      return to == State::COMPLETED ||
             to == State::CANCELED ||
             to == State::FAILED;

    case State::COMPLETED:
    case State::CANCELED:
    case State::FAILED:
    case State::REJECTED:
      // 终态之后不允许再流转
      return false;

    default:
      return false;
  }
}

}  // namespace system_manager_pkg