// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <deque>
#include <string>
#include <vector>

#include <franka/log.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

namespace franka {

class Logger {
 public:
  explicit Logger(size_t log_size);

  void log(RobotState state, research_interface::robot::RobotCommand command);
  void clear();

  std::vector<franka::Record> makeLog();

 protected:
  std::deque<RobotState> states_;
  std::deque<research_interface::robot::RobotCommand> commands_;

 private:
  const size_t log_size_;
};

}  // namespace franka
