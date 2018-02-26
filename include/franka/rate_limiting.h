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
constexpr std::array<double, 7> kMaxJointVelocity{
    {2.3925, 2.3925, 2.3925, 2.3925, 2.8710, 2.8710, 2.8710}};
/**
 * Maximum joint velocity (soft limit)
 */
constexpr std::array<double, 7> kSoftMaxJointVelocity{
    {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100}};
/**
 * Maximum joint acceleration
 */
constexpr std::array<double, 7> kMaxJointAcceleration{
    {16.5000, 8.2500, 13.7500, 13.7500, 16.5000, 22.0000, 22.0000}};
/**
 * Maximum translational velocity
 */
constexpr double kMaxTranslationalVelocity = 1.8700;
/**
 * Maximum translational acceleration
 */
constexpr double kMaxTranslationalAcceleration = 14.3000;
/**
 * Maximum rotational velocity
 */
constexpr double kMaxRotationalVelocity = 2.7500;
/**
 * Maximum rotational acceleration
 */
constexpr double kMaxRotationalAcceleration = 27.5000;

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed time
 * derivatives.
 *
 * @note
 * FCI filters must be deactivated to work properly.
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
 * Limits the rate of a desired joint velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] desired_velocities Desired joint velocity of the current time step.
 * @param[in] last_desired_velocities Desired joint velocities of the previous time step.
 *
 * @return Rate-limited vector of desired joint velocities.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& desired_velocities,
                                const std::array<double, 7>& last_desired_velocities);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] desired_positions Desired joint positions of the current time step.
 * @param[in] last_desired_positions Desired joint positions of the previous time step.
 * @param[in] last_desired_velocities Desired joint velocities of the previous time step.
 *
 * @return Rate-limited vector of desired joint positions.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& desired_positions,
                                const std::array<double, 7>& last_desired_positions,
                                const std::array<double, 7>& last_desired_velocities);

/**
 * Limits the rate of a desired Cartesian velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_translational_velocity Maximum translational velocity.
 * @param[in] max_translational_acceleration Maximum translational acceleration.
 * @param[in] max_rotational_velocity Maximum rotational velocity.
 * @param[in] max_rotational_acceleration Maximum rotational acceleration.
 * @param[in] O_dP_EE_d Desired end effector twist of the current time step.
 * @param[in] last_O_dP_EE_d Desired end effector twist of the previous time step.
 *
 * @return Rate-limited desired end effector twist.
 */
std::array<double, 6> limitRate(
    const double max_translational_velocity,
    const double max_translational_acceleration,
    const double max_rotational_velocity,
    const double max_rotational_acceleration,
    const std::array<double, 6>& O_dP_EE_d,        // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d);  // NOLINT (readability-identifier-naming)

/**
 * Limits the rate of a desired Cartesian pose considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_translational_velocity Maximum translational velocity.
 * @param[in] max_translational_acceleration Maximum translational acceleration.
 * @param[in] max_rotational_velocity Maximum rotational velocity.
 * @param[in] max_rotational_acceleration Maximum rotational acceleration.
 * @param[in] O_T_EE_d Desired pose of the current time step.
 * @param[in] last_O_T_EE_d Desired pose of the previous time step.
 * @param[in] last_O_dP_EE_d Desired end effector twist of the previous time step.
 *
 * @return Rate-limited desired pose.
 */
std::array<double, 16> limitRate(
    const double max_translational_velocity,
    const double max_translational_acceleration,
    const double max_rotational_velocity,
    const double max_rotational_acceleration,
    const std::array<double, 16>& O_T_EE_d,        // NOLINT (readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_d,   // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d);  // NOLINT (readability-identifier-naming)

}  // namespace franka
