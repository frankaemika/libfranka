#include <gtest/gtest.h>

#include <sstream>

#include "franka/robot_state.h"

using namespace franka;

TEST(RobotState, CanPrint) {
  RobotState robot_state;

  std::stringstream ss;
  ss << robot_state;
  EXPECT_NE("", ss.str());
}
