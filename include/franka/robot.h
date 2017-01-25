#pragma once

#include <memory>
#include <string>

#include <franka/robot_state.h>

namespace franka {
/**
 * Robot class maintains a connection to FRANKA CONTROL and provides the current
 * robot state.
 */
class Robot {
 public:
   using ServerVersion = uint16_t;

  /**
   * Tries to establish a connection to the robot. Throws an exception if
   * unsuccessful.
   *
   * @param[in] frankaAddress IP/hostname of FRANKA CONTROL
   */
  explicit Robot(const std::string& frankaAddress);
  ~Robot();

  /**
   * Blocks until new robot state arrives.
   *
   * @return True if a new robot state arrived, false if a timeout occurred.
   */
  bool waitForRobotState();

  /**
   * Returns last obtained robot state.
   *
   * @return RobotState structure
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

struct NetworkException : public std::runtime_error {
  explicit NetworkException(std::string const& message);
};

struct ProtocolException : public std::runtime_error {
  explicit ProtocolException(std::string const& message);
};

struct IncompatibleVersionException : public std::runtime_error {
  explicit IncompatibleVersionException(std::string const& message);
};

}  // namespace franka
