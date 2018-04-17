// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <limits>

/**
 * @file rate_limiting.h
 * Contains functions for limiting the rate of torques, Cartesian pose, Cartesian velocity,
 * joint position and joint velocity.
 */

namespace franka {
/**
 * Sample time constant
 */
constexpr double kDeltaT = 1e-3;
/**
 * Epsilon value for checking limits
 */
constexpr double kLimitEps = 1e-3;
/**
 * Epsilon value for limiting Cartesian accelerations/jerks or not
 */
constexpr double kNormEps = std::numeric_limits<double>::epsilon();
/**
 * Maximum torque rate
 */
constexpr std::array<double, 7> kMaxTorqueRate{
    {1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps,
     1000 - kLimitEps, 1000 - kLimitEps}};
/**
 * Maximum joint velocity
 */
constexpr std::array<double, 7> kMaxJointVelocity{
    {2.1750 - kLimitEps, 2.1750 - kLimitEps, 2.1750 - kLimitEps, 2.1750 - kLimitEps,
     2.6100 - kLimitEps, 2.6100 - kLimitEps, 2.6100 - kLimitEps}};
/**
 * Maximum joint acceleration
 */
constexpr std::array<double, 7> kMaxJointAcceleration{
    {15.0000 - kLimitEps, 7.500 - kLimitEps, 10.0000 - kLimitEps, 12.5000 - kLimitEps,
     15.0000 - kLimitEps, 20.0000 - kLimitEps, 20.0000 - kLimitEps}};
/**
 * Maximum joint jerk
 */
constexpr std::array<double, 7> kMaxJointJerk{
    {7500.0 - kLimitEps, 3750.0 - kLimitEps, 5000.0 - kLimitEps, 6250.0 - kLimitEps,
     7500.0 - kLimitEps, 10000.0 - kLimitEps, 10000.0 - kLimitEps}};
/**
 * Maximum translational velocity
 */
constexpr double kMaxTranslationalVelocity = 1.7000 - kLimitEps;
/**
 * Maximum translational acceleration
 */
constexpr double kMaxTranslationalAcceleration = 13.0000 - kLimitEps;
/**
 * Maximum translational jerk
 */
constexpr double kMaxTranslationalJerk = 6500.0 - kLimitEps;
/**
 * Maximum rotational velocity
 */
constexpr double kMaxRotationalVelocity = 2.5000 - kLimitEps;
/**
 * Maximum rotational acceleration
 */
constexpr double kMaxRotationalAcceleration = 25.0000 - kLimitEps;
/**
 * Maximum rotational jerk
 */
constexpr double kMaxRotationalJerk = 12500.0 - kLimitEps;
/**
 * Maximum elbow velocity
 */
constexpr double kMaxElbowVelocity = 2.1750 - kLimitEps;
/**
 * Maximum elbow acceleration
 */
constexpr double kMaxElbowAcceleration = 10.0000 - kLimitEps;
/**
 * Maximum elbow jerk
 */
constexpr double kMaxElbowJerk = 5000 - kLimitEps;

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed
 * time
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
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] desired_position Desired joint position of the current time step.
 * @param[in] last_desired_position Desired joint position of the previous time step.
 * @param[in] last_desired_velocity Desired joint velocities of the previous time step.
 * @param[in] last_desired_acceleration Desired joint accelerations of the previous time step.
 *
 * @return Rate-limited desired joint velocity.
 */
double limitRate(double max_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double desired_velocity,
                 double last_desired_velocity,
                 double last_desired_acceleration);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] desired_position Desired joint position of the current time step.
 * @param[in] last_desired_position Desired joint position of the previous time step.
 * @param[in] last_desired_velocity Desired joint velocities of the previous time step.
 * @param[in] last_desired_acceleration Desired joint accelerations of the previous time step.
 *
 * @return Rate-limited desired joint position.
 */
double limitRate(double max_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double desired_position,
                 double last_desired_position,
                 double last_desired_velocity,
                 double last_desired_acceleration);

/**
 * Limits the rate of a desired joint velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] desired_velocities Desired joint velocity of the current time step.
 * @param[in] last_desired_velocities Desired joint velocities of the previous time step.
 * @param[in] last_desired_accelerations Desired joint accelerations of the previous time step.
 *
 * @return Rate-limited vector of desired joint velocities.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& desired_velocities,
                                const std::array<double, 7>& last_desired_velocities,
                                const std::array<double, 7>& last_desired_accelerations);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed velocity.
 * @param[in] max_jerk Per-joint maximum allowed velocity.
 * @param[in] desired_positions Per-joint maximum allowed acceleration.
 * @param[in] last_desired_positions Desired joint positions of the current time step.
 * @param[in] last_desired_velocities Desired joint positions of the previous time step.
 * @param[in] last_desired_accelerations Desired joint velocities of the previous time step.
 *
 * @return Rate-limited vector of desired joint positions.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& desired_positions,
                                const std::array<double, 7>& last_desired_positions,
                                const std::array<double, 7>& last_desired_velocities,
                                const std::array<double, 7>& last_desired_accelerations);

/**
 * Limits the rate of a desired Cartesian velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_translational_velocity Maximum translational velocity.
 * @param[in] max_translational_acceleration Maximum translational acceleration.
 * @param[in] max_translational_jerk Maximum translational jerk.
 * @param[in] max_rotational_velocity Maximum rotational velocity.
 * @param[in] max_rotational_acceleration Maximum rotational acceleration.
 * @param[in] max_rotational_jerk Maximum rotational jerk.
 * @param[in] O_dP_EE_d Desired end effector twist of the current time step.
 * @param[in] last_O_dP_EE_d Desired end effector twist of the previous time step.
 * @param[in] last_O_ddP_EE_d Desired end effector acceleration of the previous time step.
 *
 * @return Rate-limited desired end effector twist.
 */
std::array<double, 6> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 6>& O_dP_EE_d,         // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d,    // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_d);  // NOLINT (readability-identifier-naming)

/**
 * Limits the rate of a desired Cartesian pose considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_translational_velocity Maximum translational velocity.
 * @param[in] max_translational_acceleration Maximum translational acceleration.
 * @param[in] max_translational_jerk Maximum translational jerk.
 * @param[in] max_rotational_velocity Maximum rotational velocity.
 * @param[in] max_rotational_acceleration Maximum rotational acceleration.
 * @param[in] max_rotational_jerk Maximum rotational jerk.
 * @param[in] O_T_EE_d Desired pose of the current time step.
 * @param[in] last_O_T_EE_d Desired pose of the previous time step.
 * @param[in] last_O_dP_EE_d Desired end effector twist of the previous time step.
 * @param[in] last_O_ddP_EE_d Desired end effector acceleration of the previous time step.
 *
 * @return Rate-limited desired pose.
 */
std::array<double, 16> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 16>& O_T_EE_d,         // NOLINT (readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_d,    // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d,    // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_d);  // NOLINT (readability-identifier-naming)

}  // namespace franka
