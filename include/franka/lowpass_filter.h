// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

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
inline double lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency) {
  if (sample_time < 0 || !std::isfinite(sample_time)) {
    throw std::invalid_argument("lowpass-filter: sample_time is negative, infinite or NaN.");
  }
  if (cutoff_frequency <= 0 || !std::isfinite(cutoff_frequency)) {
    throw std::invalid_argument(
        "lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN.");
  }
  if (!std::isfinite(y) || !std::isfinite(y_last)) {
    throw std::invalid_argument(
        "lowpass-filter: current or past input value of the signal to be filtered is infinite or "
        "NaN.");
  }
  double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
  return gain * y + (1 - gain) * y_last;
}

/**
 * Applies a first-order low-pass filter to a Cartesian translation and applies slerp to the
 * rotation.
 *
 * @param[in] sample_time Sample time constant
 * @param[in] y Current value of the Cartesian transformation matrix to be filtered
 * @param[in] y_last Value of the Cartesian transformation matrix to be filtered in the previous
 * time step
 * @param[in] cutoff_frequency Cutoff frequency of the low-pass filter
 *
 * @throw std::invalid_argument if elements of y is infinite or NaN.
 * @throw std::invalid_argument if elements of y_last is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return Filtered Cartesian transformation matrix.
 */

inline std::array<double, 16> CartesianLowpassFilter(double sample_time,
                                                     std::array<double, 16> y,
                                                     std::array<double, 16> y_last,
                                                     double cutoff_frequency) {
  if (sample_time < 0 || !std::isfinite(sample_time)) {
    throw std::invalid_argument(
        "Cartesian lowpass-filter: sample_time is negative, infinite or NaN.");
  }
  if (cutoff_frequency <= 0 || !std::isfinite(cutoff_frequency)) {
    throw std::invalid_argument(
        "Cartesian lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN.");
  }
  for (size_t i = 0; i < y.size(); i++) {
    if (!std::isfinite(y[i]) || !std::isfinite(y_last[i])) {
      throw std::invalid_argument(
          "Cartesian lowpass-filter: current or past input value of the signal to be filtered is "
          "infinite or "
          "NaN.");
    }
  }
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(y.data()));
  Eigen::Affine3d transform_last(Eigen::Matrix4d::Map(y_last.data()));
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Quaterniond orientation_last(transform_last.linear());

  double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
  transform.translation() =
      gain * transform.translation() + (1.0 - gain) * transform_last.translation();
  orientation = orientation.slerp(gain, orientation_last);

  transform.linear() << orientation.normalized().toRotationMatrix();
  std::array<double, 16> filtered_values{};
  Eigen::Map<Eigen::Matrix4d>(&filtered_values[0], 4, 4) = transform.matrix();

  return filtered_values;
}
}  // namespace franka
