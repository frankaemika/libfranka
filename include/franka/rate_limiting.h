// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

/**
 * @file rate_limiting.h
 * Contains functions for limiting the rate of torques, Cartesian and joint pose and velocity.
 */

namespace franka {

/**
 * Maximum torque rate
 */
constexpr std::array<double, 7> kMaxTorqueRate{{1000, 1000, 1000, 1000, 1000, 1000, 1000}};
/**
 * Maximum joint velocity
 */
constexpr std::array<double, 7> kMaxJointVel{
    {2.3925, 2.3925, 2.3925, 2.3925, 2.8710, 2.8710, 2.8710}};
/**
 * Soft maximum joint velocity
 */
constexpr std::array<double, 7> kSoftMaxJointVel{
    {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100}};
/**
 * Maximum joint acceleration
 */
constexpr std::array<double, 7> kMaxJointAcc{
    {16.5000, 8.2500, 13.7500, 13.7500, 16.5000, 22.0000, 22.0000}};

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed time
 * derivatives.
 *
 * @param[in] max_derivatives Per-joint maximum allowed time derivative.
 * @param[in] desired_values Desired values of the current time step.
 * @param[in] last_desired_values Desired values of the previous time step.
 *
 * @return Rate-limited vector of desired values.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& desired_values,
                                const std::array<double, 7>& last_desired_values);

/**
 * Limits the rate of an input vector of Cartesian velocities considering the maximum allowed time
 * derivatives.
 *
 * @param[in] max_derivatives Maximum allowed time derivative.
 * @param[in] desired_values Desired values of the current time step.
 * @param[in] last_desired_values Desired values of the previous time step.
 *
 * @return Rate-limited vector of desired values.
 */
std::array<double, 6> limitRate(const std::array<double, 6>& max_derivatives,
                                const std::array<double, 6>& desired_values,
                                const std::array<double, 6>& last_desired_values);

}  // namespace franka
