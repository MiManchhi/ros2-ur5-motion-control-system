#include "system_manager_pkg/state_machine.hpp"

namespace system_manager_pkg
{

StateMachine::StateMachine()
: current_state_(State::INIT)
{
}

StateMachine::State StateMachine::current_state() const
{
  return current_state_;
}

std::string StateMachine::current_state_string() const
{
  return to_string(current_state_);
}

void StateMachine::reset()
{
  current_state_ = State::IDLE;
}

bool StateMachine::transition_to(State new_state)
{
  if (!is_valid_transition(current_state_, new_state)) {
    return false;
  }

  current_state_ = new_state;
  return true;
}

std::string StateMachine::to_string(State state)
{
  switch (state) {
    case State::INIT:
      return "INIT";
    case State::IDLE:
      return "IDLE";
    case State::PLANNING:
      return "PLANNING";
    case State::RUNNING:
      return "RUNNING";
    case State::FINISHED:
      return "FINISHED";
    case State::CANCELLED:
      return "CANCELLED";
    case State::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

bool StateMachine::from_string(const std::string & state_str, State & state)
{
  if (state_str == "INIT") {
    state = State::INIT;
    return true;
  }
  if (state_str == "IDLE") {
    state = State::IDLE;
    return true;
  }
  if (state_str == "PLANNING") {
    state = State::PLANNING;
    return true;
  }
  if (state_str == "RUNNING") {
    state = State::RUNNING;
    return true;
  }
  if (state_str == "FINISHED") {
    state = State::FINISHED;
    return true;
  }
  if (state_str == "CANCELLED") {
    state = State::CANCELLED;
    return true;
  }
  if (state_str == "ERROR") {
    state = State::ERROR;
    return true;
  }

  return false;
}

bool StateMachine::is_valid_transition(State from, State to) const
{
  // 允许同状态重复进入，便于某些模块重复上报
  if (from == to) {
    return true;
  }

  switch (from) {
    case State::INIT:
      return (to == State::IDLE);

    case State::IDLE:
      return (to == State::PLANNING);

    case State::PLANNING:
      return (to == State::RUNNING ||
              to == State::ERROR ||
              to == State::CANCELLED);

    case State::RUNNING:
      return (to == State::FINISHED ||
              to == State::ERROR ||
              to == State::CANCELLED);

    case State::FINISHED:
      return (to == State::IDLE);

    case State::CANCELLED:
      return (to == State::IDLE);

    case State::ERROR:
      return (to == State::IDLE);

    default:
      return false;
  }
}

}  // namespace system_manager_pkg