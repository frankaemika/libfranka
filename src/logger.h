#pragma once

#include <queue>

#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

namespace franka {

class Logger {
 public:
  explicit Logger(size_t log_size);

  void log(RobotState state, research_interface::robot::RobotCommand command);

  std::string writeToFile();

 private:
  const size_t log_size_;
  std::queue<RobotState> state_log_;
  std::queue<research_interface::robot::RobotCommand> command_log_;
};

}  // namespace franka
