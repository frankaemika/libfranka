#pragma once

#include <memory>
#include <string>
#include "robot_state.h"

namespace franka {
/**
 * Robot class maintains a connection to FRANKA CONTROL and provides the current
 * robot state.
 */
class Robot {
 public:
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

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace franka
