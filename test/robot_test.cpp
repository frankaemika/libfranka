#include <gtest/gtest.h>

#include "franka/robot.hpp"

TEST(Robot, CanConstruct) {
  auto robot = franka::Robot::connect("unit-test");
  ASSERT_NE(nullptr, robot);
}
