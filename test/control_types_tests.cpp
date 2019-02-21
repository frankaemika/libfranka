// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include <exception>
#include <limits>

#include <franka/control_types.h>

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
  EXPECT_THROW(franka::Torques({0, 1, 2, 3, 4, 5}), std::invalid_argument);
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
  EXPECT_THROW(franka::JointPositions({0, 1, 2, 3, 4, 5}), std::invalid_argument);
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
  EXPECT_THROW(franka::JointVelocities({0, 1, 2, 3, 4, 5}), std::invalid_argument);
}

TEST(CartesianPose, CanConstructFromArray) {
  std::array<double, 16> array = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  franka::CartesianPose p(array);
  EXPECT_EQ(array, p.O_T_EE);
}

TEST(CartesianPose, CanConstructFromArrayWithElbow) {
  std::array<double, 16> array = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 2> elbow = {0, -1};
  franka::CartesianPose p(array, elbow);
  EXPECT_EQ(array, p.O_T_EE);
  EXPECT_EQ(elbow, p.elbow);
}

TEST(CartesianPose, CanConstructFromInitializerList) {
  std::array<double, 16> array = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  franka::CartesianPose p({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
  EXPECT_EQ(array, p.O_T_EE);
}

TEST(CartesianPose, CanConstructFromInitializerListWithElbow) {
  std::array<double, 16> array = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 2> elbow = {0, -1};
  franka::CartesianPose p({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {0, -1});
  EXPECT_EQ(array, p.O_T_EE);
  EXPECT_EQ(elbow, p.elbow);
}

TEST(CartesianPose, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::CartesianPose({0, 1, 2, 3, 4, 5}), std::invalid_argument);
  EXPECT_THROW(franka::CartesianPose({0, 1, 2, 3, 4, 5}, {0, 1}), std::invalid_argument);
  EXPECT_THROW(franka::CartesianPose({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, {0}),
               std::invalid_argument);
}

TEST(CartesianVelocities, CanConstructFromArray) {
  std::array<double, 6> array{0, 1, 2, 3, 4, 5};
  franka::CartesianVelocities cv(array);
  EXPECT_EQ(array, cv.O_dP_EE);
}

TEST(CartesianVelocities, CanConstructFromArrayWithElbow) {
  std::array<double, 6> array{0, 1, 2, 3, 4, 5};
  std::array<double, 2> elbow{0, 1};
  franka::CartesianVelocities cv(array, elbow);
  EXPECT_EQ(array, cv.O_dP_EE);
  EXPECT_EQ(elbow, cv.elbow);
}

TEST(CartesianVelocities, CanConstructFromInitializerList) {
  std::array<double, 6> array{0, 1, 2, 3, 4, 5};
  franka::CartesianVelocities cv({0, 1, 2, 3, 4, 5});
  EXPECT_EQ(array, cv.O_dP_EE);
}

TEST(CartesianVelocities, CanConstructFromInitializerListWithElbow) {
  std::array<double, 6> array{0, 1, 2, 3, 4, 5};
  std::array<double, 2> elbow{0, 1};
  franka::CartesianVelocities cv({0, 1, 2, 3, 4, 5}, {0, 1});
  EXPECT_EQ(array, cv.O_dP_EE);
  EXPECT_EQ(elbow, cv.elbow);
}

TEST(CartesianVelocities, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::CartesianVelocities({0, 1, 2, 3, 4}), std::invalid_argument);
  EXPECT_THROW(franka::CartesianVelocities({0, 1, 2, 3, 4}, {0, 1}), std::invalid_argument);
  EXPECT_THROW(franka::CartesianVelocities({0, 1, 2, 3, 4, 5}, {0}), std::invalid_argument);
}
