// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include <franka/log.h>

#include "helpers.h"
#include "logger.h"

using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

TEST(Logger, LogIsFIFO) {
  size_t log_count = 5;
  franka::Logger logger(log_count);

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

  std::vector<franka::Record> log = logger.flush();
  for (size_t i = 0; i < log_count; i++) {
    testRobotStatesAreEqual(states[i], log[i].state);
    testRobotCommandsAreEqual(commands[i], log[i].command);
  }
}

TEST(Logger, LogIsAFixedSizeRing) {
  size_t ring = 5;
  franka::Logger logger(ring);

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
  std::vector<franka::Record> log = logger.flush();
  for (size_t i = 0; i < ring; i++) {
    testRobotStatesAreEqual(states[i + expected_offset], log[i].state);
    testRobotCommandsAreEqual(commands[i + expected_offset], log[i].command);
  }
  EXPECT_EQ(ring, log.size());
}

TEST(Logger, LoggerEmptyAfterFlush) {
  size_t log_count = 5;
  franka::Logger logger(log_count);

  for (size_t i = 0; i < log_count; i++) {
    logger.log(franka::RobotState{}, research_interface::robot::RobotCommand{});
  }

  std::vector<franka::Record> log = logger.flush();
  EXPECT_EQ(log_count, log.size());

  log = logger.flush();

  EXPECT_EQ(0u, log.size());
}

TEST(Logger, NoLogWhenLogSizeZero) {
  franka::Logger logger(0);

  size_t log_count = 50;
  for (size_t i = 0; i < log_count; i++) {
    logger.log(franka::RobotState{}, research_interface::robot::RobotCommand{});
  }

  std::vector<franka::Record> log = logger.flush();
  EXPECT_EQ(0u, log.size());
}

TEST(Logger, WellFormattedString) {
  size_t log_count = 5;
  franka::Logger logger(log_count);

  for (size_t i = 0; i < log_count; i++) {
    logger.log(franka::RobotState{}, research_interface::robot::RobotCommand{});
  }

  std::string log = franka::logToCSV(logger.flush());

  EXPECT_PRED2(stringContains, log, "duration");
  EXPECT_PRED2(stringContains, log, "success rate");
  EXPECT_PRED2(stringContains, log, "q");
  EXPECT_PRED2(stringContains, log, "q_d");
  EXPECT_PRED2(stringContains, log, "dq");
  EXPECT_PRED2(stringContains, log, "dq_d");
  EXPECT_PRED2(stringContains, log, "tau_J");
  EXPECT_PRED2(stringContains, log, "tau_ext_hat_filtered");
  EXPECT_PRED2(stringContains, log, "sent commands");
  EXPECT_PRED2(stringContains, log, "q_d");
  EXPECT_PRED2(stringContains, log, "dq_d");
  EXPECT_PRED2(stringContains, log, "O_T_EE_d");
  EXPECT_PRED2(stringContains, log, "O_dP_EE_d");
  EXPECT_PRED2(stringContains, log, "tau_J_d");

  size_t newlines_count = std::count(log.begin(), log.end(), '\n');
  EXPECT_EQ(log_count + 1, newlines_count);
}

TEST(Logger, EmptyLogEmptyString) {
  size_t log_count = 5;
  franka::Logger logger(log_count);

  std::string csv_string = franka::logToCSV(logger.flush());

  EXPECT_STREQ("", csv_string.c_str());
}