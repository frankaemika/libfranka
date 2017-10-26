// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "logger.h"

namespace franka {

Logger::Logger(size_t log_size) : log_size_(log_size) {}

void Logger::log(RobotState state, research_interface::robot::RobotCommand command) {
  if (commands_.size() >= log_size_) {
    commands_.pop_front();
  }
  commands_.push_back(command);

  if (states_.size() >= log_size_) {
    states_.pop_front();
  }
  states_.push_back(state);
}

std::vector<Record> Logger::makeLog() {
  std::vector<Record> log;
  while (!states_.empty()) {
    research_interface::robot::RobotCommand fci_command = commands_.front();
    RobotCommand command;
    command.joint_positions = fci_command.motion.q_d;
    command.joint_velocities = fci_command.motion.dq_d;
    command.cartesian_pose.O_T_EE = fci_command.motion.O_T_EE_d;
    command.cartesian_velocities.O_dP_EE = fci_command.motion.O_dP_EE_d;
    command.torques.tau_J = fci_command.control.tau_J_d;
    Record record;
    record.state = states_.front();
    record.command = command;
    log.push_back(record);

    states_.pop_front();
    commands_.pop_front();
  }
  return log;
}

void Logger::clear() {
  states_.clear();
  commands_.clear();
}

}  // namespace franka