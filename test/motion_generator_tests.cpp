#include <gtest/gtest.h>

#include <franka/robot.h>

#include "mock_server.h"

using namespace franka;

TEST(MotionGenerator, ThrowsOnInvalidMatrixInMotionGenerator) {
  std::array<double, 16> zeroes;
  std::fill(zeroes.begin(), zeroes.end(), 0);
  EXPECT_FALSE(CartesianPoseMotionGenerator::checkHomogeneousTransformation(zeroes));

  // Translation of [1,1,1] stored in row_major order
  std::array<double, 16> row_major =
     {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1};
  EXPECT_FALSE(CartesianPoseMotionGenerator::checkHomogeneousTransformation(row_major));
}

TEST(MotionGenerator, IdentityMatrixAccepted) {
  std::array<double, 16> identity =
      {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  EXPECT_TRUE(CartesianPoseMotionGenerator::checkHomogeneousTransformation(identity));
}