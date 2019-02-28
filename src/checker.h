// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>

/**
 * @file checker.h
 * Contains helper methods to check arrays for invalid values.
 */

namespace franka {
/**
 * Helper method to check whether the elbow configuration is valid or not.
 *
 * @param[in] elbow Elbow configuration.
 *
 * @return True if valid, otherwise false.
 */
inline bool isValidElbow(const std::array<double, 2>& elbow) noexcept {
  return elbow[1] == -1.0 || elbow[1] == 1.0;
}

/**
 * Helper method to check if an array represents an homogeneous transformation matrix.
 *
 * @param[in] transform Array, which represents a 4x4 matrix.
 *
 * @return True if the array represents an homogeneous transformation matrix, otherwise false.
 */
inline bool isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept {
  constexpr double kOrthonormalThreshold = 1e-5;

  if (transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 || transform[15] != 1.0) {
    return false;
  }
  for (size_t j = 0; j < 3; ++j) {  // i..column
    if (std::abs(std::sqrt(std::pow(transform[j * 4 + 0], 2) + std::pow(transform[j * 4 + 1], 2) +
                           std::pow(transform[j * 4 + 2], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {  // j..row
    if (std::abs(std::sqrt(std::pow(transform[0 * 4 + i], 2) + std::pow(transform[1 * 4 + i], 2) +
                           std::pow(transform[2 * 4 + i], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  return true;
}

/**
 * Helper template to check if an array contains NaN or infinite values.
 *
 * @param[in] array Array to check.
 *
 * @throw std::invalid_argument when fields of the array contain NaN or infinite values.
 */
template <typename T, size_t N>
inline void checkFinite(const std::array<T, N>& array) {
  if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
    throw std::invalid_argument("Commanding value is infinite or NaN.");
  }
};

/**
 * Helper method to check if an array represents a valid transformation matrix.
 *
 * @param[in] transform Array to check.
 *
 * @throw std::invalid_argument if array does not represent a valid transformation matrix.
 */
inline void checkMatrix(const std::array<double, 16>& transform) {
  checkFinite(transform);
  if (!isHomogeneousTransformation(transform)) {
    throw std::invalid_argument(
        "libfranka: Attempt to set invalid transformation in motion generator. Has to be column "
        "major!");
  }
}

/**
 * Helper method to check if an array represents a valid elbow configuration.
 *
 * @param[in] elbow Array to check.
 *
 * @throw std::invalid_argument if array does not represent a valid elbow configuration.
 */
inline void checkElbow(const std::array<double, 2>& elbow) {
  checkFinite(elbow);
  if (!isValidElbow(elbow)) {
    throw std::invalid_argument(
        "Invalid elbow configuration given! Only +1 or -1 are allowed for the sign of the 4th "
        "joint.");
  }
}
}  // namespace franka