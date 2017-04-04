#pragma once

#include <array>
#include <initializer_list>

/**
 * @file robot_state.h
 * Contains types for motion generation and torque control.
 */

namespace franka {

/**
 * 
 */
struct Torques {
  Torques(const std::array<double, 7>& torques); // NOLINT (google-explicit-constructor)

  /**
   * @throw std::invalid_argument Wrong size of torques.
   */
  Torques(std::initializer_list<double> torques); // NOLINT (google-explicit-constructor)

  std::array<double, 7> tau_J;  // NOLINT (readability-identifier-naming)

 protected:
  Torques() = default;
};

struct JointValues {
  JointValues(const std::array<double, 7>& joint_values); // NOLINT (google-explicit-constructor)

  /**
   * @throw std::invalid_argument Wrong size of joint_values.
   */
  JointValues(std::initializer_list<double> joint_values); // NOLINT (google-explicit-constructor)

  /*
   * Desired joint angles in [rad]
   */
  std::array<double, 7> q;

 protected:
  JointValues() = default;
};

struct JointVelocities {
  JointVelocities(const std::array<double, 7>& joint_velocities); // NOLINT (google-explicit-constructor)

  /**
   * @throw std::invalid_argument Wrong size of joint_velocities.
   */
  JointVelocities(std::initializer_list<double> joint_velocities); // NOLINT (google-explicit-constructor)

  /*
   * Desired joint velocities in [rad/s]
   */
  std::array<double, 7> dq;

 protected:
  JointVelocities() = default;
};

struct CartesianPose {
  CartesianPose(const std::array<double, 16>& pose); // NOLINT (google-explicit-constructor)

  /**
   * @throw std::invalid_argument Wrong size of pose.
   */
  CartesianPose(std::initializer_list<double> pose); // NOLINT (google-explicit-constructor)

  /*
   * Homogeneous transformation \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$
   */
  std::array<double, 16> O_T_EE;  // NOLINT (readability-identifier-naming)

 protected:
  CartesianPose() = default;
};

struct CartesianVelocities {
  CartesianVelocities(const std::array<double, 6>& velocities); // NOLINT (google-explicit-constructor)

  /**
   * @throw std::invalid_argument Wrong size of velocities.
   */
  CartesianVelocities(std::initializer_list<double> velocities); // NOLINT (google-explicit-constructor)

  /*
   * Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}
   */
  std::array<double, 6> O_dP_EE;  // NOLINT (readability-identifier-naming)

 protected:
  CartesianVelocities() = default;
};

struct StopType : Torques, JointValues, JointVelocities, CartesianPose, CartesianVelocities {
  StopType() = default;
};

static constexpr StopType Stop = StopType();  // NOLINT (readability-identifier-naming)

}  // namespace franka
