// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <deque>
#include <string>
#include <vector>

#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>
#include <franka/logging/robot_state_log.hpp>

namespace franka {

class RobotStateLogger {
 public:
  explicit RobotStateLogger(size_t log_size);

  void log(const RobotState& state, const research_interface::robot::RobotCommand& command);

  std::vector<franka::Record> flush();

 private:
  std::vector<RobotState> states_;
  std::vector<research_interface::robot::RobotCommand> commands_;
  size_t ring_front_{0};
  size_t ring_size_{0};

  const size_t log_size_;  // NOLINT(readability-identifier-naming)
};

}  // namespace franka
