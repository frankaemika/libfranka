#include <gtest/gtest.h>

#include "franka/robot.h"

TEST(Robot, CanConstruct) {
  auto robot = franka::Robot::connect("unit-test");
  ASSERT_NE(nullptr, robot);
}
