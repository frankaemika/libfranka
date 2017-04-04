#include <gtest/gtest.h>

#include "motion_generator_loop.h"

using namespace franka;

TEST(MotionGenerator, ThrowsOnInvalidMatrixInMotionGenerator) {
  std::array<double, 16> zeroes;
  std::fill(zeroes.begin(), zeroes.end(), 0);
  EXPECT_FALSE(MotionGeneratorLoop<CartesianPose>::checkHomogeneousTransformation(zeroes));

  // Translation of [1,1,1] stored in row_major order
  std::array<double, 16> row_major =
     {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1};
  EXPECT_FALSE(MotionGeneratorLoop<CartesianPose>::checkHomogeneousTransformation(row_major));
}

TEST(MotionGenerator, IdentityMatrixAccepted) {
  std::array<double, 16> identity =
      {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  EXPECT_TRUE(MotionGeneratorLoop<CartesianPose>::checkHomogeneousTransformation(identity));
}
