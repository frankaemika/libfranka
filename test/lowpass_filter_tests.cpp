// Copyright (c) 2018 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <franka/lowpass_filter.h>

#include "helpers.h"

using namespace franka;

const double kNoLimit{std::numeric_limits<double>::max()};

TEST(LowpassFilter, KeepsValueIfNoChange) {
  EXPECT_NEAR(lowpassFilter(0.001, 1.0, 1.0, 100.0), 1.0, 1e-6);
  EXPECT_NEAR(lowpassFilter(0.001, 1.0, 1.0, 500.0), 1.0, 1e-6);
  EXPECT_NEAR(lowpassFilter(0.001, 1.0, 1.0, 1000.0), 1.0, 1e-6);
}

TEST(LowpassFilter, DoesFilter) {
  EXPECT_NEAR(lowpassFilter(0.001, 1.0, 0.0, 100.0), 0.3859, 1e-4);
  EXPECT_NEAR(lowpassFilter(0.001, 1.0, 0.0, 500.0), 0.7585, 1e-4);
  EXPECT_NEAR(lowpassFilter(0.001, 1.0, 0.0, 900.0), 0.8497, 1e-4);
}

TEST(CartesianLowpassFilter, CanFixNonOrthonormalRotation) {
  // These three poses are all only barely orthonormal, such that the cartesianLowpassFilter will
  // generate a jerky movement when it does not orthonormalize these first before applying the
  // filter.
  Eigen::Affine3d pose1;
  Eigen::Affine3d pose2;
  Eigen::Affine3d pose3;
  pose1.translation() << 1, 1, 1;
  pose2.translation() << 1, 1, 1;
  pose3.translation() << 1, 1, 1;
  // clang-format off
  pose1.linear() << 0.00462567, 0.999974, 0.00335239,
                    0.0145489, -0.00341934, 0.999888,
                    0.999874, -0.00457638, -0.0145646;
  pose2.linear() << 0.00463526, 0.999984, 0.00335239,
                    0.014549, -0.00341951, 0.999888,
                    0.999883, -0.00458597, -0.0145646;
  pose3.linear() << 0.00465436, 0.999984, 0.00335239,
                    0.0145489, -0.00341979, 0.999888,
                    0.999883, -0.00460507, -0.0145646;
  // clang-format on
  std::array<double, 16> pose_array1{};
  std::array<double, 16> pose_array2{};
  std::array<double, 16> pose_array3{};
  Eigen::Map<Eigen::Matrix4d>(&pose_array1[0], 4, 4) = pose1.matrix();
  Eigen::Map<Eigen::Matrix4d>(&pose_array2[0], 4, 4) = pose2.matrix();
  Eigen::Map<Eigen::Matrix4d>(&pose_array3[0], 4, 4) = pose3.matrix();

  auto output_array1 = cartesianLowpassFilter(0.001, pose_array2, pose_array1, 100.);
  auto output_array2 = cartesianLowpassFilter(0.001, pose_array3, pose_array2, 100.);
  auto velocity1 = differentiateOneSample(output_array1, pose_array1, 0.001);
  auto velocity2 = differentiateOneSample(output_array2, pose_array2, 0.001);

  std::array<double, 6> jerk;
  for (int i = 0; i < 6; i++) {
    double acceleration1 = velocity1[i] / 0.001;
    double acceleration2 = (velocity2[i] - velocity1[i]) / 0.001;
    jerk[i] = (acceleration2 - acceleration1) / 0.001;
  }
  double total_jerk = std::sqrt(std::pow(jerk[3], 2) + std::pow(jerk[4], 2) + std::pow(jerk[5], 2));
  EXPECT_LT(total_jerk, 1000);
}
