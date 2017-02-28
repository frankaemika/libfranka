#pragma once

#include <memory>

/**
 * @file motion_generator.h
 * Contains the Motion Generator classes.
 */

namespace franka {

/**
 * Allows to stream Cartesian pose commands to the franka robot
 */
class CartesianPoseMotionGenerator {
 public:
  class Impl;

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Generator to move
   */
  CartesianPoseMotionGenerator(
      CartesianPoseMotionGenerator&& motion_generator) noexcept;

  /**
   * Moves a motion generator implementation.
   *
   * @param[in] impl Generator impl to move
   */
  explicit CartesianPoseMotionGenerator(Impl&& impl);

  ~CartesianPoseMotionGenerator() noexcept;

  /**
   * Tries to set a cartesian motion command as a homogeneous transformation.
   *
   * @param[in] desired_pose Homogeneous transformation O_T_EE_d, column major,
   * that transforms from the end-effector frame EE to base frame O
   */
  void setDesiredPose(const std::array<double, 16>& desired_pose) noexcept;

  CartesianPoseMotionGenerator() = delete;
  CartesianPoseMotionGenerator(const CartesianPoseMotionGenerator&) = delete;
  CartesianPoseMotionGenerator& operator=(const CartesianPoseMotionGenerator&) =
      delete;
  CartesianPoseMotionGenerator& operator=(
      const CartesianPoseMotionGenerator&&) = delete;

 private:
  std::unique_ptr<Impl> impl_;
};

/**
 * Allows to stream Cartesian velocity commands to the franka robot
 */
class CartesianVelocityMotionGenerator {
 public:
  class Impl;

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Generator to move
   */
  CartesianVelocityMotionGenerator(
      CartesianVelocityMotionGenerator&& motion_generator) noexcept;

  /**
   * Moves a motion generator implementation.
   *
   * @param[in] impl Generator impl to move
   */
  explicit CartesianVelocityMotionGenerator(Impl&& impl);

  ~CartesianVelocityMotionGenerator() noexcept;

  /**
   * Sets a desired Cartesian velocity command.
   *
   * @param[in] desired_velocity Desired Cartesian velocity w.r.t. O-frame
   * {dx in [m/s], dx in [m/s], dz in [m/s],
   * omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}
   */
  void setDesiredVelocity(
      const std::array<double, 6>& desired_velocity) noexcept;

  CartesianVelocityMotionGenerator() = delete;
  CartesianVelocityMotionGenerator(const CartesianVelocityMotionGenerator&) =
      delete;
  CartesianVelocityMotionGenerator& operator=(
      const CartesianVelocityMotionGenerator&) = delete;
  CartesianVelocityMotionGenerator& operator=(
      const CartesianVelocityMotionGenerator&&) = delete;

 private:
  std::unique_ptr<Impl> impl_;
};

/**
 * Allows to stream joint pose commands to the franka robot
 */
class JointPoseMotionGenerator {
 public:
  class Impl;

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Generator to move
   */
  JointPoseMotionGenerator(
      JointPoseMotionGenerator&& motion_generator) noexcept;

  /**
   * Moves a motion generator implementation.
   *
   * @param[in] impl Generator impl to move
   */
  explicit JointPoseMotionGenerator(Impl&& impl);

  ~JointPoseMotionGenerator() noexcept;

  /**
   * Sets a desired joint pose command.
   *
   * @param[in] desired_pose Desired joint angles in [rad]
   */
  void setDesiredPose(const std::array<double, 7>& desired_pose) noexcept;

  JointPoseMotionGenerator() = delete;
  JointPoseMotionGenerator(const JointPoseMotionGenerator&) = delete;
  JointPoseMotionGenerator& operator=(const JointPoseMotionGenerator&) = delete;
  JointPoseMotionGenerator& operator=(const JointPoseMotionGenerator&&) =
      delete;

 private:
  std::unique_ptr<Impl> impl_;
};

/**
 * Allows to stream joint velocity commands to the franka robot
 */
class JointVelocityMotionGenerator {
 public:
  class Impl;

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Generator to move
   */
  JointVelocityMotionGenerator(
      JointVelocityMotionGenerator&& motion_generator) noexcept;

  /**
   * Moves a motion generator implementation.
   *
   * @param[in] impl Generator impl to move
   */
  explicit JointVelocityMotionGenerator(Impl&& impl);

  ~JointVelocityMotionGenerator() noexcept;

  /**
   * Sets a desired joint velocity command.
   *
   * @param[in] desired_velocity Desired joint velocities in [rad/s]
   */
  void setDesiredVelocity(
      const std::array<double, 7>& desired_velocity) noexcept;

  JointVelocityMotionGenerator() = delete;
  JointVelocityMotionGenerator(const JointVelocityMotionGenerator&) = delete;
  JointVelocityMotionGenerator& operator=(const JointVelocityMotionGenerator&) =
      delete;
  JointVelocityMotionGenerator& operator=(
      const JointVelocityMotionGenerator&&) = delete;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace franka
