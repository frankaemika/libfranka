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
 * Number of packets lost considered for the definition of velocity limits.
 * When a packet is lost, FCI assumes a constant acceleration model
 */
constexpr double kTolNumberPacketsLost = 3.0;
/**
 * Factor for the definition of rotational limits using the Cartesian Pose interface
 */
constexpr double kFactorCartesianRotationPoseInterface = 0.99;
/**
 * Maximum torque rate
 */
constexpr std::array<double, 7> kMaxTorqueRate{
    {1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps,
     1000 - kLimitEps, 1000 - kLimitEps}};
/**
 * Maximum joint jerk
 */
constexpr std::array<double, 7> kMaxJointJerk{
    {7500.0 - kLimitEps, 3750.0 - kLimitEps, 5000.0 - kLimitEps, 6250.0 - kLimitEps,
     7500.0 - kLimitEps, 10000.0 - kLimitEps, 10000.0 - kLimitEps}};
/**
 * Maximum joint acceleration
 */
constexpr std::array<double, 7> kMaxJointAcceleration{
    {15.0000 - kLimitEps, 7.500 - kLimitEps, 10.0000 - kLimitEps, 12.5000 - kLimitEps,
     15.0000 - kLimitEps, 20.0000 - kLimitEps, 20.0000 - kLimitEps}};
/**
 * Maximum joint velocity
 */
constexpr std::array<double, 7> kMaxJointVelocity{
    {2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[0],
     2.1750 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[1],
     2.1750 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[2],
     2.1750 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[3],
     2.6100 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[4],
     2.6100 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[5],
     2.6100 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[6]}};
/**
 * Maximum translational jerk
 */
constexpr double kMaxTranslationalJerk = 6500.0 - kLimitEps;
/**
 * Maximum translational acceleration
 */
constexpr double kMaxTranslationalAcceleration = 13.0000 - kLimitEps;
/**
 * Maximum translational velocity
 */
constexpr double kMaxTranslationalVelocity =
    2.0000 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxTranslationalAcceleration;
/**
 * Maximum rotational jerk
 */
constexpr double kMaxRotationalJerk = 12500.0 - kLimitEps;
/**
 * Maximum rotational acceleration
 */
constexpr double kMaxRotationalAcceleration = 25.0000 - kLimitEps;
/**
 * Maximum rotational velocity
 */
constexpr double kMaxRotationalVelocity =
    2.5000 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxRotationalAcceleration;
/**
 * Maximum elbow jerk
 */
constexpr double kMaxElbowJerk = 5000 - kLimitEps;
/**
 * Maximum elbow acceleration
 */
constexpr double kMaxElbowAcceleration = 10.0000 - kLimitEps;
/**
 * Maximum elbow velocity
 */
constexpr double kMaxElbowVelocity =
    2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxElbowAcceleration;

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed
 * time derivatives.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_derivatives Per-joint maximum allowed time derivative.
 * @param[in] commanded_values Commanded values of the current time step.
 * @param[in] last_commanded_values Commanded values of the previous time step.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 * @return Rate-limited vector of desired values.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& commanded_values,
                                const std::array<double, 7>& last_commanded_values);

/**
 * Limits the rate of a desired joint velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Maximum allowed velocity.
 * @param[in] max_acceleration Maximum allowed acceleration.
 * @param[in] max_jerk Maximum allowed jerk.
 * @param[in] commanded_velocity Commanded joint velocity of the current time step.
 * @param[in] last_commanded_velocity Commanded joint velocity of the previous time step.
 * @param[in] last_commanded_acceleration Commanded joint acceleration of the previous time step.
 *
 * @throw std::invalid_argument if commanded_velocity is infinite or NaN.
 *
 * @return Rate-limited desired joint velocity.
 */
double limitRate(double max_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double commanded_velocity,
                 double last_commanded_velocity,
                 double last_commanded_acceleration);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Maximum allowed velocity.
 * @param[in] max_acceleration Maximum allowed acceleration.
 * @param[in] max_jerk Maximum allowed jerk.
 * @param[in] commanded_position Commanded joint position of the current time step.
 * @param[in] last_commanded_position Commanded joint position of the previous time step.
 * @param[in] last_commanded_velocity Commanded joint velocity of the previous time step.
 * @param[in] last_commanded_acceleration Commanded joint acceleration of the previous time step.
 *
 * @throw std::invalid_argument if commanded_position is infinite or NaN.
 *
 * @return Rate-limited desired joint position.
 */
double limitRate(double max_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double commanded_position,
                 double last_commanded_position,
                 double last_commanded_velocity,
                 double last_commanded_acceleration);

/**
 * Limits the rate of a desired joint velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] commanded_velocities Commanded joint velocity of the current time step.
 * @param[in] last_commanded_velocities Commanded joint velocities of the previous time step.
 * @param[in] last_commanded_accelerations Commanded joint accelerations of the previous time step.
 *
 * @throw std::invalid_argument if commanded_velocities are infinite or NaN.
 *
 * @return Rate-limited vector of desired joint velocities.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_velocities,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_velocity Per-joint maximum allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] commanded_positions Commanded joint positions of the current time step.
 * @param[in] last_commanded_positions Commanded joint positions of the previous time step.
 * @param[in] last_commanded_velocities Commanded joint velocities of the previous time step.
 * @param[in] last_commanded_accelerations Commanded joint accelerations of the previous time step.
 *
 * @throw std::invalid_argument if commanded_positions are infinite or NaN.
 *
 * @return Rate-limited vector of desired joint positions.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_positions,
                                const std::array<double, 7>& last_commanded_positions,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations);

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
 * @param[in] O_dP_EE_c Commanded end effector twist of the current time step.
 * @param[in] last_O_dP_EE_c Commanded end effector twist of the previous time step.
 * @param[in] last_O_ddP_EE_c Commanded end effector acceleration of the previous time step.
 *
 * @throw std::invalid_argument if an element of O_dP_EE_c is infinite or NaN.
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
    const std::array<double, 6>& O_dP_EE_c,         // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_c,    // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_c);  // NOLINT(readability-identifier-naming)

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
 * @param[in] O_T_EE_c Commanded pose of the current time step.
 * @param[in] last_O_T_EE_c Commanded pose of the previous time step.
 * @param[in] last_O_dP_EE_c Commanded end effector twist of the previous time step.
 * @param[in] last_O_ddP_EE_c Commanded end effector acceleration of the previous time step.
 *
 * @throw std::invalid_argument if an element of O_T_EE_c is infinite or NaN.
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
    const std::array<double, 16>& O_T_EE_c,         // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_c,    // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_c,    // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_c);  // NOLINT(readability-identifier-naming)

}  // namespace franka
