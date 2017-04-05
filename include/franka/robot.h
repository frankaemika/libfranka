#pragma once

#include <memory>
#include <string>

#include <franka/control_types.h>
#include <franka/exception.h>
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
   * Version of the server running on FRANKA CONTROL.
   */
  using ServerVersion = uint16_t;

  /**
   * Establishes a connection with FRANKA CONTROL.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this library is not supported by
   * FRANKA CONTROL
   * @throw ProtocolException if data received from the host is invalid
   *
   * @param[in] franka_address IP/hostname of FRANKA CONTROL
   * @param[in] realtime_config if set to Enforce, an exception will be thrown
   * if realtime priority cannot be set when required. Setting realtime_config
   * to Ignore disables this behavior.
   */
  explicit Robot(const std::string& franka_address,
                 RealtimeConfig realtime_config = RealtimeConfig::kEnforce);
  ~Robot() noexcept;

  /**
   * Starts a control loop for torque control.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] control_callback Callback function for torque control.
   *
   * @throw RealtimeException if realtime priority can not be set for the
   * current thread.
   * @throw ControlException if an error related to torque control resp. motion
   * generation occured.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the
   * current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<Torques(const RobotState&)> control_callback);

  /**
   * Starts a control loop for a joint value motion generator, optionally with
   * torque control.
   *
   * Sets realtime priority for the current thread if torque control is used.
   *
   * @param[in] control_callback Callback function for torque control.
   * @param[in] motion_generator_callback Callback function for motion
   * generation.
   *
   * @throw ControlException if an error related to torque control resp. motion
   * generation occured.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the
   * current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(
      std::function<JointValues(const RobotState&)> motion_generator_callback,
      std::function<Torques(const RobotState&)> control_callback =
          std::function<Torques(const RobotState&)>());

  /**
   * Starts a control loop for a joint velocity motion generator, optionally
   * with torque control.
   *
   * Sets realtime priority for the current thread if torque control is used.
   *
   * @param[in] control_callback Callback function for torque control.
   * @param[in] motion_generator_callback Callback function for motion
   * generation.
   *
   * @throw ControlException if an error related to torque control resp. motion
   * generation occured.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the
   * current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<JointVelocities(const RobotState&)>
                   motion_generator_callback,
               std::function<Torques(const RobotState&)> control_callback =
                   std::function<Torques(const RobotState&)>());

  /**
   * Starts a control loop for a Cartesian pose motion generator, optionally
   * with torque control.
   *
   * Sets realtime priority for the current thread if torque control is used.
   *
   * @param[in] control_callback Callback function for torque control.
   * @param[in] motion_generator_callback Callback function for motion
   * generation.
   *
   * @throw ControlException if an error related to torque control resp. motion
   * generation occured.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the
   * current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(
      std::function<CartesianPose(const RobotState&)> motion_generator_callback,
      std::function<Torques(const RobotState&)> control_callback =
          std::function<Torques(const RobotState&)>());

  /**
   * Starts a control loop for a Cartesian velocity motion generator, optionally
   * with torque control.
   *
   * Sets realtime priority for the current thread if torque control is used.
   *
   * @param[in] control_callback Callback function for torque control.
   * @param[in] motion_generator_callback Callback function for motion
   * generation.
   *
   * @throw ControlException if an error related to torque control resp. motion
   * generation occured.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the
   * current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<CartesianVelocities(const RobotState&)>
                   motion_generator_callback,
               std::function<Torques(const RobotState&)> control_callback =
                   std::function<Torques(const RobotState&)>());

  /**
   * Starts a loop for reading the current robot state.
   *
   * @param[in] read_callback Callback function for robot state reading.
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   */
  void read(std::function<bool(const RobotState&)> read_callback);

  /**
   * Waits for a robot state update and returns it.
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   *
   * @return Current robot state.
   */
  RobotState readOnce();

  /**
   * Returns the version reported by the connected server.
   *
   * @return Version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

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
