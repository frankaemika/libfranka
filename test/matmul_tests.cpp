// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <cmath>

#include <gtest/gtest.h>

#include "matmul.h"

TEST(MatMul, MultipliesIdentityCorrectly) {
  std::array<double, 16> id = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 16> a;
  std::iota(a.begin(), a.end(), 1);

  EXPECT_EQ(a, franka::matMul(a, id));
  EXPECT_EQ(a, franka::matMul(id, a));
}

TEST(MatMul, MultipliesInverseCorrectly) {
  std::array<double, 16> id = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double val = 1. / std::sqrt(2);
  std::array<double, 16> a = {val, -val, 0, 0, val, val, 0, 0, 0, 0, 1, 0, val, -val, 2, 1};
  std::array<double, 16> ai = {val, val, 0, 0, -val, val, 0, 0, 0, 0, 1, 0, -1, 0, -2, 1};

  auto out1 = franka::matMul(a, ai);
  auto out2 = franka::matMul(ai, a);

  for (size_t i = 0; i < 16; i++) {
    EXPECT_NEAR(id[i], out1[i], 1e-15);
    EXPECT_NEAR(id[i], out2[i], 1e-15);
  }
}
