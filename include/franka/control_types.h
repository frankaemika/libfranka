#pragma once

#include <array>
#include <initializer_list>

/**
 * @file robot_state.h
 * Contains types for motion generation and torque control.
 */

namespace franka {

/**
 * Stores values for torque control.
 */
struct Torques {
  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired torques in [Nm].
   */
  Torques(const std::array<double, 7>& torques) noexcept; // NOLINT (google-explicit-constructor)

  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired torques in [Nm].
   *
   * @throw ControlException Invalid number of elements in torques.
   */
  Torques(std::initializer_list<double> torques); // NOLINT (google-explicit-constructor)

  /**
   * Desired torques in [Nm].
   */
  std::array<double, 7> tau_J{};  // NOLINT (readability-identifier-naming)

 protected:
  constexpr Torques() noexcept = default;
};

/**
 * Stores values for joint position motion generation.
 */
struct JointValues {
  /**
   * Creates a new JointValues instance.
   *
   * @param[in] joint_values Desired joint angles in [rad].
   */
  JointValues(const std::array<double, 7>& joint_values) noexcept; // NOLINT (google-explicit-constructor)

  /**
   * Creates a new JointValues instance.
   *
   * @param[in] joint_values Desired joint angles in [rad].
   *
   * @throw ControlException Invalid number of elements in joint_values.
   */
  JointValues(std::initializer_list<double> joint_values); // NOLINT (google-explicit-constructor)

  /**
   * Desired joint angles in [rad].
   */
  std::array<double, 7> q{};

 protected:
  constexpr JointValues() noexcept = default;
};

/**
 * Stores values for joint velocity motion generation.
 */
struct JointVelocities {
  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   */
  JointVelocities(const std::array<double, 7>& joint_velocities) noexcept; // NOLINT (google-explicit-constructor)

  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   *
   * @throw ControlException Invalid number of elements in joint_velocities.
   */
  JointVelocities(std::initializer_list<double> joint_velocities); // NOLINT (google-explicit-constructor)

  /**
   * Desired joint velocities in [rad/s].
   */
  std::array<double, 7> dq{};

 protected:
  constexpr JointVelocities() noexcept = default;
};

/**
 * Stores values for Cartesian pose motion generation.
 */
struct CartesianPose {
  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$.
   */
  CartesianPose(const std::array<double, 16>& cartesian_pose) noexcept; // NOLINT (google-explicit-constructor)

  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$.
   *
   * @throw ControlException Invalid number of elements in cartesian_pose.
   */
  CartesianPose(std::initializer_list<double> cartesian_pose); // NOLINT (google-explicit-constructor)

  /**
   * Desired vectorized homogeneous transformation matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$.
   */
  std::array<double, 16> O_T_EE{};  // NOLINT (readability-identifier-naming)

 protected:
  constexpr CartesianPose() noexcept = default;
};

/**
 * Stores values for Cartesian velocity motion generation.
 */
struct CartesianVelocities {
  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   */
  CartesianVelocities(const std::array<double, 6>& cartesian_velocities); // NOLINT (google-explicit-constructor)

  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   *
   * @throw ControlException Invalid number of elements in cartesian_pose.
   */
  CartesianVelocities(std::initializer_list<double> cartesian_velocities); // NOLINT (google-explicit-constructor)

  /**
   * Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   */
  std::array<double, 6> O_dP_EE{};  // NOLINT (readability-identifier-naming)

 protected:
  constexpr CartesianVelocities() noexcept = default;
};

/**
 * Used to signal the termination of a motion generation resp. control loop.
 */
struct StopType : Torques, JointValues, JointVelocities, CartesianPose, CartesianVelocities {
  constexpr StopType() noexcept = default;
};

/**
 * Used to signal the termination of a motion generation resp. control loop.
 */
static constexpr StopType Stop = StopType();  // NOLINT (readability-identifier-naming)

}  // namespace franka
