#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <franka/gripper_state.h>

/**
 * @file gripper.h
 * Contains the Gripper class.
 */

namespace franka {

class Network;

/**
 * Maintains a connection to FRANKA CONTROL, provides the current gripper state,
 * and allows to execute gripper commands.
 */
class Gripper {
 public:
  /**
   * Version of the gripper server running on FRANKA CONTROL.
   */
  using ServerVersion = uint16_t;

  /**
   * Establishes a connection with the gripper service on FRANKA CONTROL.
   *
   * @param[in] franka_address IP/hostname of FRANKA CONTROL
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this library is not supported by FRANKA CONTROL.
   * @throw ProtocolException if data received from the host is invalid.
   */
  explicit Gripper(const std::string& franka_address);

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
   * @throw CommandException if an error occurred.
   *
   * @see GripperState for the maximum grasping width.
   */
  void homing();

  /**
   * Grasp an object.
   *
   * @param[in] width Size of the object to grasp. [m]
   * @param[in] speed Closing speed. [m/s]
   * @param[in] force Grasping force. [mA]
   *
   * @return True if an object has been grasped, false otherwise.
   *
   * @throw CommandException if an error occurred.
   */
  bool grasp(double width, double speed, double force);

  /**
   * Moves the gripper fingers to a specified width.
   *
   * @param[in] width Intended opening width. [m]
   * @param[in] speed Closing speed. [m/s]
   *
   * @throw CommandException if an error occurred.
   */
  void move(double width, double speed);

  /**
   * Stops a currently running gripper move or grasp.
   *
   * @throw CommandException if an error occurred.
   */
  void stop();

  /**
   * Waits for a gripper state update and returns it.
   *
   * @return Current gripper state.
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   */
  GripperState readOnce() const;

  /**
   * Returns the software version reported by the connected server.
   *
   * @return Software version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  Gripper(const Gripper&) = delete;
  Gripper& operator=(const Gripper&) = delete;

 private:
  std::unique_ptr<Network> network_;

  uint16_t ri_version_;
  uint32_t command_id_ = 0;
};

}  // namespace franka
