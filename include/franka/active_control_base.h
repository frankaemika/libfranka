// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <memory>
#include <optional>
#include <utility>

/**
 * @file active_control_base.h
 * Abstract interface class as the base of the active controllers.
 */

namespace franka {

/**
 * Allows the user to read the state of a Robot and to send new control commands after starting a
 * control process of a Robot.
 *
 * hint: To create an ActiveControlBase, see franka::ActiveTorqueControl or
 * franka::ActiveMotionGenerator
 *
 */
class ActiveControlBase {
 public:
  virtual ~ActiveControlBase() = default;

  /**
   * Waits for a robot state update and returns it.
   *
   * @return Current robot state & time since last read operation
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if robot returns an unexpected message.
   * @throw ControlException if robot is in an error state.
   */
  virtual std::pair<RobotState, Duration> readOnce() = 0;

  /**
   * Updates torque commands of an active control
   *
   * hint: implemented in ActiveTorqueControl
   *
   */
  virtual void writeOnce(const Torques& /* control_input */) = 0;

  /**
   * Updates the joint position and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<JointPositions>
   *
   */
  virtual void writeOnce(const JointPositions& /* motion_generator_input */,
                         const std::optional<const Torques>& /*control_input*/) = 0;

  /**
   * Updates the joint velocity and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<JointVelocities>
   *
   */
  virtual void writeOnce(const JointVelocities& /* motion_generator_input */,
                         const std::optional<const Torques>& /* control_input */) = 0;
  /**
   * Updates the cartesian position and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<CartesianPose>
   *
   */
  virtual void writeOnce(const CartesianPose& /* motion_generator_input */,
                         const std::optional<const Torques>& /* control_input */) = 0;

  /**
   * Updates the cartesian velocity and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<CartesianVelocities>
   *
   */
  virtual void writeOnce(const CartesianVelocities& /* motion_generator_input */,
                         const std::optional<const Torques>& /* control_input */) = 0;
  /**
   * Updates the joint position commands of an active control, with internal controller
   *
   * @param motion_generator_input the new motion generator input
   *
   */
  virtual void writeOnce(const JointPositions& motion_generator_input) = 0;

  /**
   * Updates the joint velocity commands of an active control, with internal controller
   *
   * @param motion_generator_input the new motion generator input
   *
   */
  virtual void writeOnce(const JointVelocities& motion_generator_input) = 0;
  /**
   * Updates the cartesian pose commands of an active control, with internal controller
   *
   * @param motion_generator_input the new motion generator input
   *
   */
  virtual void writeOnce(const CartesianPose& motion_generator_input) = 0;

  /**
   * Updates the cartesian velocity commands of an active control, with internal controller
   *
   * @param motion_generator_input the new motion generator input
   *
   */
  virtual void writeOnce(const CartesianVelocities& motion_generator_input) = 0;

 protected:
  ActiveControlBase() = default;
};

}  // namespace franka
