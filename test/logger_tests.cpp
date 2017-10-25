// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include "helpers.h"
#include "logger.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

struct Logger : public ::franka::Logger {
  using ::franka::Logger::Logger;
  using ::franka::Logger::commands_;
  using ::franka::Logger::states_;
};

TEST(Logger, LogIsFIFO) {
  size_t log_count = 5;
  Logger logger(log_count);

  std::vector<franka::RobotState> states;
  std::vector<research_interface::robot::RobotCommand> commands;

  for (size_t i = 0; i < log_count; i++) {
    franka::RobotState state;
    randomRobotState(state);
    states.push_back(state);

    research_interface::robot::RobotCommand command;
    randomRobotCommand(command);
    commands.push_back(command);

    logger.log(state, command);
  }

  for (size_t i = 0; i < log_count; i++) {
    testRobotStatesAreEqual(states[i], logger.states_[i]);
    testRobotCommandsAreEqual(commands[i], logger.commands_[i]);
  }
}

TEST(Logger, LogIsAFixedSizeRing) {
  size_t ring = 5;
  Logger logger(ring);

  std::vector<franka::RobotState> states;
  std::vector<research_interface::robot::RobotCommand> commands;

  size_t logs = ring * 2;
  for (size_t i = 0; i < logs; i++) {
    franka::RobotState state;
    randomRobotState(state);
    states.push_back(state);

    research_interface::robot::RobotCommand command;
    randomRobotCommand(command);
    commands.push_back(command);

    logger.log(state, command);
  }

  size_t expected_offset = logs - ring;
  for (size_t i = 0; i < ring; i++) {
    testRobotStatesAreEqual(states[i + expected_offset], logger.states_[i]);
    testRobotCommandsAreEqual(commands[i + expected_offset], logger.commands_[i]);
  }
  EXPECT_EQ(ring, logger.states_.size());
  EXPECT_EQ(ring, logger.commands_.size());
}

TEST(Logger, LoggerEmptyAfterWrite) {
  size_t log_count = 5;
  Logger logger(log_count);

  for (size_t i = 0; i < log_count; i++) {
    logger.log(franka::RobotState{}, research_interface::robot::RobotCommand{});
  }

  EXPECT_EQ(log_count, logger.states_.size());
  EXPECT_EQ(log_count, logger.commands_.size());

  logger.makeLog();

  EXPECT_EQ(0u, logger.states_.size());
  EXPECT_EQ(0u, logger.commands_.size());
}

TEST(Logger, LoggerEmptyAfterClear) {
  size_t log_count = 5;
  Logger logger(log_count);

  for (size_t i = 0; i < log_count; i++) {
    logger.log(franka::RobotState{}, research_interface::robot::RobotCommand{});
  }

  logger.clear();

  EXPECT_EQ(0u, logger.states_.size());
  EXPECT_EQ(0u, logger.commands_.size());
}

TEST(Logger, WellFormattedString) {
  size_t log_count = 5;
  Logger logger(log_count);

  for (size_t i = 0; i < log_count; i++) {
    logger.log(franka::RobotState{}, research_interface::robot::RobotCommand{});
  }

  std::string log = logger.makeLog();

  EXPECT_PRED2(stringContains, log, "duration");
  EXPECT_PRED2(stringContains, log, "success rate");
  EXPECT_PRED2(stringContains, log, "q");
  EXPECT_PRED2(stringContains, log, "q_d");
  EXPECT_PRED2(stringContains, log, "dq");
  EXPECT_PRED2(stringContains, log, "dq_d");
  EXPECT_PRED2(stringContains, log, "tau_J");
  EXPECT_PRED2(stringContains, log, "tau_ext_hat_filtered");
  EXPECT_PRED2(stringContains, log, "sent commands");
  EXPECT_PRED2(stringContains, log, "id");
  EXPECT_PRED2(stringContains, log, "q_d");
  EXPECT_PRED2(stringContains, log, "dq_d");
  EXPECT_PRED2(stringContains, log, "O_T_EE_d");
  EXPECT_PRED2(stringContains, log, "O_dP_EE_d");
  EXPECT_PRED2(stringContains, log, "tau_J_d");

  size_t newlines_count = std::count(log.begin(), log.end(), '\n');
  EXPECT_EQ(log_count + 1, newlines_count);
}