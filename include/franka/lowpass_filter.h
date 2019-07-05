// Copyright (c) 2018 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>

/**
 * @file lowpass_filter.h
 * Contains functions for filtering signals with a low-pass filter.
 */

namespace franka {
/**
 * Maximum cutoff frequency
 */
constexpr double kMaxCutoffFrequency = 1000.0;
/**
 * Default cutoff frequency
 */
constexpr double kDefaultCutoffFrequency = 100.0;
/**
 * Applies a first-order low-pass filter
 *
 * @param[in] sample_time Sample time constant
 * @param[in] y Current value of the signal to be filtered
 * @param[in] y_last Value of the signal to be filtered in the previous time step
 * @param[in] cutoff_frequency Cutoff frequency of the low-pass filter
 *
 * @throw std::invalid_argument if y is infinite or NaN.
 * @throw std::invalid_argument if y_last is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return Filtered value.
 */
double lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency);

/**
 * Applies a first-order low-pass filter to the translation and spherical linear interpolation
 * to the rotation of a transformation matrix which represents a Cartesian Motion.
 *
 * @param[in] sample_time Sample time constant
 * @param[in] y Current Cartesian transformation matrix to be filtered
 * @param[in] y_last Cartesian transformation matrix from the previous time step
 * @param[in] cutoff_frequency Cutoff frequency of the low-pass filter
 *
 * @throw std::invalid_argument if elements of y is infinite or NaN.
 * @throw std::invalid_argument if elements of y_last is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return Filtered Cartesian transformation matrix.
 */

std::array<double, 16> cartesianLowpassFilter(double sample_time,
                                              std::array<double, 16> y,
                                              std::array<double, 16> y_last,
                                              double cutoff_frequency);
}  // namespace franka
