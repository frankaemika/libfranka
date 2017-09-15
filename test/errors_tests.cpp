// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include <franka/errors.h>
#include <research_interface/robot/error.h>
#include <research_interface/robot/rbk_types.h>

#include "helpers.h"

TEST(Errors, IsInitializedToNoErrors) {
  franka::Errors errors;

  EXPECT_FALSE(errors);
}

TEST(Errors, EvaluatedToTrueOnError) {
  std::array<bool, sizeof(research_interface::robot::RobotState::errors)> error_flags{};
  error_flags[rand() % error_flags.size()] = true;

  franka::Errors errors(error_flags);

  EXPECT_TRUE(errors);
}

TEST(Errors, CanGetNamesOfErrors) {
  std::array<bool, sizeof(research_interface::robot::RobotState::errors)> error_flags{};
  std::fill(error_flags.begin(), error_flags.end(), false);
  error_flags[static_cast<size_t>(
      research_interface::robot::Error::kJointPositionLimitsViolation)] = true;
  error_flags[static_cast<size_t>(
      research_interface::robot::Error::kSelfcollisionAvoidanceViolation)] = true;

  franka::Errors errors(error_flags);

  std::string expected =
      R"(["joint_position_limits_violation", "self_collision_avoidance_violation"])";
  EXPECT_EQ(expected, static_cast<std::string>(errors));
}

TEST(Errors, CanBeStreamed) {
  franka::Errors errors;

  std::stringstream ss;
  ss << errors;
  std::string output(ss.str());

  EXPECT_EQ("[]", output);
}
