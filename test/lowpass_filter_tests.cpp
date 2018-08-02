// Copyright (c) 2018 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <franka/lowpass_filter.h>

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
