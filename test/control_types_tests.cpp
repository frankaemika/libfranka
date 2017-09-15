// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include <franka/control_types.h>
#include <franka/exception.h>

TEST(Torques, CanConstructFromArray) {
  std::array<double, 7> array{0, 1, 2, 3, 4, 5, 6};
  franka::Torques t(array);
  EXPECT_EQ(array, t.tau_J);
}

TEST(Torques, CanConstructFromInitializerList) {
  std::array<double, 7> array{0, 1, 2, 3, 4, 5, 6};
  franka::Torques t({0, 1, 2, 3, 4, 5, 6});
  EXPECT_EQ(array, t.tau_J);
}

TEST(Torques, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::Torques({0, 1, 2, 3, 4, 5}), franka::ControlException);
}

TEST(JointPositions, CanConstructFromArray) {
  std::array<double, 7> array{0, 1, 2, 3, 4, 5, 6};
  franka::JointPositions jv(array);
  EXPECT_EQ(array, jv.q);
}

TEST(JointPositions, CanConstructFromInitializerList) {
  std::array<double, 7> array{0, 1, 2, 3, 4, 5, 6};
  franka::JointPositions jv({0, 1, 2, 3, 4, 5, 6});
  EXPECT_EQ(array, jv.q);
}

TEST(JointPositions, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::JointPositions({0, 1, 2, 3, 4, 5}), franka::ControlException);
}

TEST(JointVelocities, CanConstructFromArray) {
  std::array<double, 7> array{0, 1, 2, 3, 4, 5, 6};
  franka::JointVelocities jv(array);
  EXPECT_EQ(array, jv.dq);
}

TEST(JointVelocities, CanConstructFromInitializerList) {
  std::array<double, 7> array{0, 1, 2, 3, 4, 5, 6};
  franka::JointVelocities jv({0, 1, 2, 3, 4, 5, 6});
  EXPECT_EQ(array, jv.dq);
}

TEST(JointVelocities, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::JointVelocities({0, 1, 2, 3, 4, 5}), franka::ControlException);
}

TEST(CartesianPose, CanConstructFromArray) {
  std::array<double, 16> array = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  franka::CartesianPose p(array);
  EXPECT_EQ(array, p.O_T_EE);
}

TEST(CartesianPose, CanConstructFromInitializerList) {
  std::array<double, 16> array = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  franka::CartesianPose p({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
  EXPECT_EQ(array, p.O_T_EE);
}

TEST(CartesianPose, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::CartesianPose({0, 1, 2, 3, 4, 5}), franka::ControlException);
}

TEST(CartesianPose, CanNotConstructFromInvalidMatrix) {
  EXPECT_THROW(franka::CartesianPose({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}),
               franka::ControlException);

  // Translation of [1,1,1] stored in row_major order
  EXPECT_THROW(franka::CartesianPose({1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1}),
               franka::ControlException);
}

TEST(CartesianVelocities, CanConstructFromArray) {
  std::array<double, 6> array{0, 1, 2, 3, 4, 5};
  franka::CartesianVelocities cv(array);
  EXPECT_EQ(array, cv.O_dP_EE);
}

TEST(CartesianVelocities, CanConstructFromInitializerList) {
  std::array<double, 6> array{0, 1, 2, 3, 4, 5};
  franka::CartesianVelocities cv({0, 1, 2, 3, 4, 5});
  EXPECT_EQ(array, cv.O_dP_EE);
}

TEST(CartesianVelocities, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::CartesianVelocities({0, 1, 2, 3, 4}), franka::ControlException);
}
