#pragma once

#include <memory>
#include <string>

#include <franka/exception.h>
#include <franka/motion_generator.h>
#include <franka/robot_state.h>

/**
 * @file robot.h
 * Contains the Robot class.
 */

namespace franka {

/**
 * Maintains a connection to FRANKA CONTROL and provides the current
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
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this library is not supported by
   * FRANKA CONTROL
   * @throw ProtocolException if data received from the host is invalid
   *
   * @param[in] franka_address IP/hostname of FRANKA CONTROL
   */
  explicit Robot(const std::string& franka_address);
  ~Robot() noexcept;

  /**
   * Blocks until new robot state arrives. Then it sends the current command
   * over the UDP connection. When the function returns true, the
   * reference from getRobotState() points to new data and the robot command was
   * sent.
   *
   * @throw NetworkException if the connetion is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   *
   * @return True if a new robot state arrived and a robot command was sent,
   * false if the connection is
   * cleanly closed.
   */
  bool update();

  /**
   * Returns last obtained robot state. Updated after a call to
   * waitForRobotState().
   *
   * @return const reference to RobotState structure
   */
  const RobotState& robotState() const noexcept;

  /**
   * Returns the version reported by the connected server.
   *
   * @return Version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  /**
   * Tries to start and instantiate a motion generator for Cartesian
   * Poses
   *
   * @throw MotionGeneratorException if another motion generator is
   * already running
   *
   * @return A CartesianPoseMotionGenerator that allows to stream
   * Cartesian pose commands to the robot
   */
  CartesianPoseMotionGenerator startCartesianPoseMotionGenerator();

  /**
   * Tries to start and instantiate a motion generator for Cartesian
   * Velocities
   *
   * @throw MotionGeneratorException if another motion generator is
   * already running
   *
   * @return A CartesianVelocityMotionGenerator that allows to stream
   * Cartesian velocity commands to the robot
   */
  CartesianVelocityMotionGenerator startCartesianVelocityMotionGenerator();

  /**
   * Tries to start and instantiate a motion generator for joint
   * poses
   *
   * @throw MotionGeneratorException if another motion generator is
   * already running
   *
   * @return A JointPoseMotionGenerator that allows to stream
   * joint pose commands to the robot
   */
  JointPoseMotionGenerator startJointPoseMotionGenerator();

  /**
   * Tries to start and instantiate a motion generator for joint
   * velocities
   *
   * @throw MotionGeneratorException if another motion generator is
   * already running
   *
   * @return A JointVelocityMotionGenerator that allows to stream
   * joint velocity commands to the robot
   */
  JointVelocityMotionGenerator startJointVelocityMotionGenerator();

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  class Impl;

  /**
   * Gets the robot implementation.
   *
   * @return Robot implementation
   */
  Impl& impl() noexcept;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace franka
