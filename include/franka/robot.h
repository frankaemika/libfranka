#pragma once

#include <memory>
#include <string>

#include <franka/robot_state.h>

/** \file robot.h
    Contains the Robot class and Exception definitions.
*/

namespace franka {
/** \class Robot
 * Robot class maintains a connection to FRANKA CONTROL and provides the current
 * robot state.
 */
class Robot {
 public:
  /**
   * Version of the server running on FRANKA.
   */
  using ServerVersion = uint16_t;

  /**
   * Tries to establish a connection with the FRANKA robot.
   *
   * Throws:
   * - NetworkException if the connection is unsuccessful.
   * - IncompatibleVersionException if this library is not supported by FRANKA
   * CONTROL
   * - ProtocolException if data received from the host is invalid
   *
   * @param[in] frankaAddress IP/hostname of FRANKA CONTROL
   */
  explicit Robot(const std::string& frankaAddress);
  ~Robot();

  /**
   * Blocks until new robot state arrives. When the function returns true, the
   * reference
   * from getRobotState() points to new data.
   * Throws:
   * - NetworkException if the connetion is lost, e.g. after a timeout.
   * - ProtocolException if received data has invalid format.
   *
   * @return True if a new robot state arrived, false if the connection is
   * cleanly closed.
   */
  bool waitForRobotState();

  /**
   * Returns last obtained robot state. Updated after a call to
   * waitForRobotState().
   *
   * @return const reference to RobotState structure
   */
  const RobotState& getRobotState() const;

  /**
   * Returns the version reported by the connected server.
   *
   * @return Version of the connected server.
   */
  ServerVersion getServerVersion() const;

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * NetworkException is thrown when a connection to FRANKA cannot be established,
 * or when
 * a timeout occurs.
 */
struct NetworkException : public std::runtime_error {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit NetworkException(std::string const& message);
};

/**
 * ProtocolException is thrown when the server returns an incorrect message.
 */
struct ProtocolException : public std::runtime_error {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit ProtocolException(std::string const& message);
};

/**
 * IncompatibleVersionException is thrown if the server does not support this
 * version of libfranka.
 */
struct IncompatibleVersionException : std::runtime_error {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit IncompatibleVersionException(std::string const& message);
};

}  // namespace franka
