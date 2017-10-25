// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "logger.h"

#include <iterator>
#include <sstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

namespace {

template <typename T, size_t N>
std::string csvName(const std::array<T, N>& /*unused*/, const std::string& name) {
  std::ostringstream os;
  for (size_t i = 0; i < N - 1; i++) {
    os << name << "[" << i << "], ";
  }
  os << name << "[" << N - 1 << "]";
  return os.str();
}

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream /*unused*/, const std::array<T, N>& array) {
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  return ostream;
}

std::string csvRobotStateHeader() {
  RobotState robot_state;
  std::ostringstream os;
  os << "duration, success rate, " << csvName(robot_state.q, "q") << ","
     << csvName(robot_state.q_d, "q_d") << "," << csvName(robot_state.dq, "dq") << ","
     << csvName(robot_state.dq_d, "dq_d") << "," << csvName(robot_state.tau_J, "tau_J") << ","
     << csvName(robot_state.tau_ext_hat_filtered, "tau_ext_hat_filtered");
  return os.str();
}

std::string csvRobotCommandHeader() {
  research_interface::robot::RobotCommand command;
  std::ostringstream os;
  os << "sent commands,id," << csvName(command.motion.q_d, "q_d") << ","
     << csvName(command.motion.dq_d, "dq_d") << "," << csvName(command.motion.O_T_EE_d, "O_T_EE_d")
     << "," << csvName(command.motion.O_dP_EE_d, "O_dP_EE_d") << ","
     << csvName(command.control.tau_J_d, "tau_J_d");
  return os.str();
}

std::string csvLine(const franka::RobotState& robot_state) {
  std::ostringstream os;
  os << robot_state.time.toMSec() << "," << robot_state.control_command_success_rate << ","
     << robot_state.q << "," << robot_state.q_d << "," << robot_state.dq << "," << robot_state.dq_d
     << "," << robot_state.tau_J << "," << robot_state.tau_ext_hat_filtered;
  return os.str();
}

std::string csvLine(const research_interface::robot::RobotCommand& command) {
  std::ostringstream os;
  os << command.message_id << "," << command.motion.q_d << "," << command.motion.dq_d << ","
     << command.motion.O_T_EE_d << "," << command.motion.O_dP_EE_d << ","
     << command.control.tau_J_d;
  return os.str();
}

}  // anonymous namespace

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

std::string Logger::makeLog() {
  std::ostringstream os;

  os << csvRobotStateHeader() << "," << csvRobotCommandHeader() << std::endl;
  for (size_t i = 0; i < states_.size(); i++) {
    os << csvLine(states_[i]) << ",,";
    if (i < commands_.size()) {
      os << csvLine(commands_[i]);
    }
    os << std::endl;
  }

  clear();
  return os.str();
}

void Logger::clear() {
  states_.clear();
  commands_.clear();
}

}  // namespace franka