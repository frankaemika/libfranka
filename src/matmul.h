// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>

namespace franka {

/**
 * Computes matrix multiplication C=A*B
 */
template <size_t T>
inline std::array<double, T> matMul(const std::array<double, T>& a,
                                    const std::array<double, T>& b) {
  std::array<double, T> c{};
  for (size_t row = 0; row < std::sqrt(T); row++) {
    for (size_t col = 0; col < std::sqrt(T); col++) {
      for (size_t inner = 0; inner < std::sqrt(T); inner++) {
        c[row + col * std::sqrt(T)] +=
            a[row + inner * std::sqrt(T)] * b[inner + col * std::sqrt(T)];
      }
    }
  }
  return c;
}

}  // namespace franka
