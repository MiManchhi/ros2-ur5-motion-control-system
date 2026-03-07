#include "system_manager_pkg/state_machine.hpp"

#include "robot_common_pkg/constants.hpp"

StateMachine::StateMachine()
: current_state_(robot_common_pkg::constants::state::kIdle)
{
}

void StateMachine::set_state(const std::string & state)
{
  current_state_ = state;
}

const std::string & StateMachine::get_state() const
{
  return current_state_;
}

void StateMachine::reset()
{
  current_state_ = robot_common_pkg::constants::state::kIdle;
}