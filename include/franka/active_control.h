// Copyright (c) 2023 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <utility>

#include <franka/control_types.h>
#include <franka/robot_state.h>

#include "robot.h"

/**
 * @file active_control.h
 * Contains the `franka::ActiveControl` type.
 */

namespace franka {

/**
 * Allows the user to read the state of a Robot and to send new control commands after starting a
 * control process of an Robot.
 *
 * hint: To create an ActiveControl, see Robot::startControl
 *
 */
class ActiveControl {
 public:
  virtual ~ActiveControl();

  /**
   * Updates the joint-level based torque commands of an active joint effort control
   *
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred, or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const Torques& control_input);

  /**
   * Updates the joint-level based joint position commands of an active joint position control
   *
   * @param motion_generator_input the new joint-level based positions
   *
   * @throw ControlException if an error related to torque control or motion generation occurred, or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const JointPositions& motion_generator_input);

  /**
   * Updates the joint-level based joint velocity commands of an active joint velocity control
   *
   * @param motion_generator_input the new joint-level based velocities
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const JointVelocities& motion_generator_input);

  /**
   * TODO: update
   *
   * @param motion_generator_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const CartesianPose& motion_generator_input);

  /**
   * TODO: update
   *
   * @param motion_generator_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const CartesianVelocities& motion_generator_input);

  /**
   * Updates the joint-level based joint position commands of an active joint position control
   *
   * @param motion_generator_input the new joint-level based positions
   *
   * @throw ControlException if an error related to torque control or motion generation occurred, or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const JointPositions& motion_generator_input, const Torques& control_input);

  /**
   * Updates the joint-level based joint velocity commands of an active joint velocity control
   *
   * @param motion_generator_input the new joint-level based velocities
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const JointVelocities& motion_generator_input, const Torques& control_input);

  /**
   * TODO: update
   *
   * @param motion_generator_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const CartesianPose& motion_generator_input, const Torques& control_input);

  /**
   * TODO: update
   *
   * @param motion_generator_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const CartesianVelocities& motion_generator_input, const Torques& control_input);

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
   * @note ActiveControl objects can only be created during the Robot::startMotion. To allow access
   * to the private constructor Robot is defined as friend
   *
   */
  friend class Robot;

 private:
  /**
   * Construct a new ActiveControl object
   *
   * @param robot shared_ptr to the Robot::Impl in the Robot
   * @param motion_id id of the managed motion
   * @param control_lock of the Robot, preventing other read and write accesses during the active
   * control
   */
  ActiveControl(std::shared_ptr<Robot::Impl> robot_impl,
                uint32_t motion_id,
                std::unique_lock<std::mutex> control_lock,
                ActiveControlControllerOptions controller_type,
                ActiveControlMotionGeneratorOptions motion_generator_type);

  std::shared_ptr<Robot::Impl> robot_impl_;
  uint32_t motion_id_;
  std::unique_lock<std::mutex> control_lock_;
  bool control_finished_;
  bool first_read_attempt_;
  Duration last_read_access_;
  ActiveControlControllerOptions controller_type_;
  ActiveControlMotionGeneratorOptions motion_generator_type_;
};

}  // namespace franka
