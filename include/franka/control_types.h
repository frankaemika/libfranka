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
class Torques {
 public:
  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired torques in [Nm].
   */
  Torques(const std::array<double, 7>& torques) noexcept;

  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired torques in [Nm].
   *
   * @throw ControlException Invalid number of elements in torques.
   */
  Torques(std::initializer_list<double> torques);

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
class JointValues {
 public:
  /**
   * Creates a new JointValues instance.
   *
   * @param[in] joint_values Desired joint angles in [rad].
   */
  JointValues(const std::array<double, 7>& joint_values) noexcept;

  /**
   * Creates a new JointValues instance.
   *
   * @param[in] joint_values Desired joint angles in [rad].
   *
   * @throw ControlException Invalid number of elements in joint_values.
   */
  JointValues(std::initializer_list<double> joint_values);

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
class JointVelocities {
 public:
  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   */
  JointVelocities(const std::array<double, 7>& joint_velocities) noexcept;

  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   *
   * @throw ControlException Invalid number of elements in joint_velocities.
   */
  JointVelocities(std::initializer_list<double> joint_velocities);

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
class CartesianPose {
 public:
  /**
   * Creates a new CartesianPose instance.
   *
   * @throw ControlException if cartesian_pose is not a valid vectorized
   *                         homogeneous transformation matrix (column-major).
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation
   * matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the
   * end-effector frame \f$EE\f$ to base frame \f$O\f$.
   */
  CartesianPose(const std::array<double, 16>& cartesian_pose);

  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation
   * matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the
   * end-effector frame \f$EE\f$ to base frame \f$O\f$.
   *
   * @throw ControlException if cartesian_pose is not a valid vectorized
   *                         homogeneous transformation matrix (column-major).
   */
  CartesianPose(std::initializer_list<double> cartesian_pose);

  /**
   * Homogeneous transformation \f${}_O \mathbf{T}_{EE,d}\f$, column major, that
   * transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$
   */
  std::array<double, 16> O_T_EE{};  // NOLINT (readability-identifier-naming)

 protected:
  constexpr CartesianPose() noexcept = default;

 private:
  void checkHomogeneousTransformation();

  /**
   * Checks a a homogeneous transformation for validity.
   *
   * @param[in] transform Homogeneous transformation to be checked,
   * passed as column major array.
   *
   * @return True if transformation has ortho-normal rotation matrix,
   * the last row is [0 0 0 1] and the array defines a column major matrix.
   */
  static bool isHomogeneousTransformation(
      const std::array<double, 16>& transform) noexcept;
};

/**
 * Stores values for Cartesian velocity motion generation.
 */
class CartesianVelocities {
 public:
  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame
   * {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in
   * [rad/s], omegaz in [rad/s]}.
   */
  CartesianVelocities(const std::array<double, 6>& cartesian_velocities);

  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame
   * {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in
   * [rad/s], omegaz in [rad/s]}.
   *
   * @throw ControlException Invalid number of elements in cartesian_pose.
   */
  CartesianVelocities(std::initializer_list<double> cartesian_velocities);

  /**
   * Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in
   * [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}
   */
  std::array<double, 6> O_dP_EE{};  // NOLINT (readability-identifier-naming)

 protected:
  constexpr CartesianVelocities() noexcept = default;
};

/**
 * Used to signal the termination of a motion generation resp. control loop.
 */
struct StopType : Torques,
                  JointValues,
                  JointVelocities,
                  CartesianPose,
                  CartesianVelocities {
  constexpr StopType() noexcept = default;
};

/**
 * Used to signal the termination of a motion generation resp. control loop.
 */
static constexpr StopType Stop =  // NOLINT (readability-identifier-naming)
    StopType();

/**
 * Used to decide the realtime behavior of a control loop thread.
 */
enum RealtimeConfig { kEnforce, kIgnore };

}  // namespace franka
