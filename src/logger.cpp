// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "logger.h"

namespace franka {

Logger::Logger(size_t log_size) : log_size_(log_size) {
  states_.resize(log_size);
  commands_.resize(log_size);
}

void Logger::log(const RobotState& state, const research_interface::robot::RobotCommand& command) {
  if (log_size_ == 0) {
    return;
  }

  commands_[ring_front_] = command;
  states_[ring_front_] = state;

  ring_front_ = (ring_front_ + 1) % log_size_;
  ring_size_ = std::min(log_size_, ring_size_ + 1);
}

std::vector<Record> Logger::flush() {
  std::vector<Record> log;

  for (size_t i = 0; i < ring_size_; i++) {
    size_t wrapped_index = (ring_front_ + i) % ring_size_;

    RobotCommand command;
    command.joint_positions = commands_[wrapped_index].motion.q_c;
    command.joint_velocities = commands_[wrapped_index].motion.dq_c;
    command.cartesian_pose.O_T_EE = commands_[wrapped_index].motion.O_T_EE_c;
    command.cartesian_velocities.O_dP_EE = commands_[wrapped_index].motion.O_dP_EE_c;
    command.torques.tau_J = commands_[wrapped_index].control.tau_J_d;

    Record record;
    record.state = states_[wrapped_index];
    record.command = command;
    log.push_back(record);
  }

  ring_front_ = 0;
  ring_size_ = 0;
  return log;
}

}  // namespace franka
