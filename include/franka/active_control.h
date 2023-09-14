// Copyright (c) 2023 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <optional>
#include <utility>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

#include "robot.h"

/**
 * @file active_control.h
 * Contains the `franka::ActiveControl`, `franka::ActiveTorqueControl` and
 * `franka::ActiveMotionGenerator` type.
 */

namespace franka {

/**
 * Allows the user to read the state of a Robot and to send new control commands after starting a
 * control process of a Robot.
 *
 * hint: To create an ActiveControl, see franka::ActiveTorqueControl or
 * franka::ActiveMotionGenerator
 *
 */
class ActiveControl {
 public:
  virtual ~ActiveControl();

  /**
   * Waits for a robot state update and returns it.
   *
   * @return Current robot state & time since last read operation
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if robot returns an unexpected message.
   * @throw ControlException if robot is in an error state.
   */
  std::pair<RobotState, Duration> readOnce();

  /**
   * Updates the joint position and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<JointPositions>
   *
   * @return void
   */
  virtual void writeOnce(
      const JointPositions& /* motion_generator_input */,
      const std::optional<const Torques>& /*control_input*/ = std::optional<const Torques>()) {
    throw franka::ControlException(wrong_write_once_method_called_);
  };

  /**
   * Updates the joint velocity and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<JointVelocities>
   *
   * @return void
   */
  virtual void writeOnce(
      const JointVelocities& /* motion_generator_input */,
      const std::optional<const Torques>& /* control_input */ = std::optional<const Torques>()) {
    throw franka::ControlException(wrong_write_once_method_called_);
  };

  /**
   * Updates the cartesian position and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<CartesianPose>
   *
   * @return void
   */
  virtual void writeOnce(
      const CartesianPose& /* motion_generator_input */,
      const std::optional<const Torques>& /* control_input */ = std::optional<const Torques>()) {
    throw franka::ControlException(wrong_write_once_method_called_);
  };

  /**
   * Updates the cartesian velocity and torque commands of an active control
   *
   * hint: implemented in ActiveMotionGenerator<CartesianVelocities>
   *
   * @return void
   */
  virtual void writeOnce(
      const CartesianVelocities& /* motion_generator_input */,
      const std::optional<const Torques>& /* control_input */ = std::optional<const Torques>()) {
    throw franka::ControlException(wrong_write_once_method_called_);
  };

  protected:
  /**
   * Construct a new ActiveControl object
   *
   * @param robot_impl shared_ptr to the Robot::Impl in the Robot
   * @param motion_id id of the managed motion
   * @param control_lock of the Robot, preventing other read and write accesses during the active
   * control
   */
  ActiveControl(std::shared_ptr<Robot::Impl> robot_impl,
                uint32_t motion_id,
                std::unique_lock<std::mutex> control_lock);

  /// shared pointer to Robot::Impl instance for read and write accesses
  std::shared_ptr<Robot::Impl> robot_impl;

  /// motion id of running motion
  uint32_t motion_id;

  /// control-lock preventing parallel control processes
  std::unique_lock<std::mutex> control_lock;

  /// flag indicating if control process is finished
  bool control_finished;

  /// flag indicating if it is the first read access
  bool first_read_attempt;

  /// duration to last read access
  Duration last_read_access;

 private:
  const std::string wrong_write_once_method_called_{
      "Wrong writeOnce method called for currently active control!"};
};

}  // namespace franka
