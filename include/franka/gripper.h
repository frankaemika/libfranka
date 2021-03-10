// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <franka/gripper_state.h>

/**
 * @file gripper.h
 * Contains the franka::Gripper type.
 */

namespace franka {

class Network;

/**
 * Maintains a network connection to the gripper, provides the current gripper state,
 * and allows the execution of commands.
 *
 * @note
 * The members of this class are threadsafe.
 */
class Gripper {
 public:
  /**
   * Version of the gripper server.
   */
  using ServerVersion = uint16_t;

  /**
   * Establishes a connection with a gripper connected to a robot.
   *
   * @param[in] franka_address IP/hostname of the robot the gripper is connected to.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this version of `libfranka` is not supported.
   */
  explicit Gripper(const std::string& franka_address);

  /**
   * Move-constructs a new Gripper instance.
   *
   * @param[in] gripper Other Gripper instance.
   */
  Gripper(Gripper&& gripper) noexcept;

  /**
   * Move-assigns this Gripper from another Gripper instance.
   *
   * @param[in] gripper Other Gripper instance.
   *
   * @return Model instance.
   */
  Gripper& operator=(Gripper&& gripper) noexcept;

  /**
   * Closes the connection.
   */
  ~Gripper() noexcept;

  /**
   * Performs homing of the gripper.
   *
   * After changing the gripper fingers, a homing needs to be done.
   * This is needed to estimate the maximum grasping width.
   *
   * @return True if command was successful, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see GripperState for the maximum grasping width.
   */
  bool homing() const;

  /**
   * Grasps an object.
   *
   * An object is considered grasped if the distance \f$d\f$ between the gripper fingers satisfies
   * \f$(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})\f$.
   *
   * @param[in] width Size of the object to grasp in \f$[m]\f$.
   * @param[in] speed Closing speed in \f$[\frac{m}{s}]\f$.
   * @param[in] force Grasping force in \f$[N]\f$.
   * @param[in] epsilon_inner Maximum tolerated deviation when the actual grasped width is smaller
   * than the commanded grasp width.
   * @param[in] epsilon_outer Maximum tolerated deviation when the actual grasped width is larger
   * than the commanded grasp width.
   *
   * @return True if an object has been grasped, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  bool grasp(double width,
             double speed,
             double force,
             double epsilon_inner = 0.005,
             double epsilon_outer = 0.005) const;

  /**
   * Moves the gripper fingers to a specified width.
   *
   * @param[in] width Intended opening width in \f$[m]\f$.
   * @param[in] speed Closing speed in \f$[\frac{m}{s}]\f$.
   *
   * @return True if command was successful, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  bool move(double width, double speed) const;

  /**
   * Stops a currently running gripper move or grasp.
   *
   * @return True if command was successful, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  bool stop() const;

  /**
   * Waits for a gripper state update and returns it.
   *
   * @return Current gripper state.
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw InvalidOperationException if another readOnce is already running.
   */
  GripperState readOnce() const;

  /**
   * Returns the software version reported by the connected server.
   *
   * @return Software version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  /// @cond DO_NOT_DOCUMENT
  Gripper(const Gripper&) = delete;
  Gripper& operator=(const Gripper&) = delete;
  /// @endcond

 private:
  std::unique_ptr<Network> network_;

  uint16_t ri_version_;
};

}  // namespace franka
