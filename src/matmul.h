// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

namespace franka {

/**
 * Computes matrix multiplication C=A*B
 */
inline std::array<double, 16> matMul(const std::array<double, 16>& a,
                                     const std::array<double, 16>& b) {
  std::array<double, 16> c{};
  for (size_t row = 0; row < 4; row++) {
    for (size_t col = 0; col < 4; col++) {
      for (size_t inner = 0; inner < 4; inner++) {
        c[row + col * 4] += a[row + inner * 4] * b[inner + col * 4];
      }
    }
  }
  return c;
}

}  // namespace franka
