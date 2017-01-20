#include <gtest/gtest.h>

#include "franka/robot.h"
#include "franka/robot_state.h"

TEST(Robot, CanConstruct) {
  franka::Robot robot("unit-test");
}

TEST(Robot, WaitForRobotReturnsFalse) {
  franka::Robot robot("unit-test");
  ASSERT_FALSE(robot.waitForRobotState());
}

TEST(Robot, CanGetRobotState) {
  franka::Robot robot("unit-test");
  franka::RobotState state = robot.getRobotState();
}