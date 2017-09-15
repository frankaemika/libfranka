// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <franka/command_types.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>

/**
 * @file robot.h
 * Contains the franka::Robot type.
 */

namespace franka {

class Model;

/**
 * Maintains a network connection to the robot, provides the current robot state, gives access to
 * the model library and allows to execute commands, motions, and torque control.
 *
 * @note
 * The members of this class are threadsafe.
 *
 * @par End effector frame
 * While the end effector parameters are set in a configuration file, it is possible to change the
 * end effector frame with Robot::setEE.
 *
 * @par K frame
 * The K frame is used for Cartesian impedance control and measuring forces and torques. It can be
 * set with Robot::setK.
 */
class Robot {
 public:
  /**
   * Version of the robot server.
   */
  using ServerVersion = uint16_t;

  /**
   * Establishes a connection with the robot.
   *
   * @param[in] franka_address IP/hostname of the robot.
   * @param[in] realtime_config if set to Enforce, an exception will be thrown if realtime priority
   * cannot be set when required. Setting realtime_config to Ignore disables this behavior.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this version of `libfranka` is not supported.
   */
  explicit Robot(const std::string& franka_address,
                 RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  /**
   * Move-constructs a new Robot instance.
   *
   * @param[in] other Other Robot instance.
   */
  Robot(Robot&& other) noexcept;

  /**
   * Move-assigns this Robot from another Robot instance.
   *
   * @param[in] other Other Robot instance.
   *
   * @return Robot instance.
   */
  Robot& operator=(Robot&& other) noexcept;

  /**
   * Closes the connection.
   */
  ~Robot() noexcept;

  /**
   * @name Motion generation and torque control
   *
   * The following methods allow to perform torque control and/or motion generation by providing
   * callback functions.
   *
   * Only one of these methods can be active at the same time; a franka::ControlException is thrown
   * otherwise.
   *
   * @anchor callback-docs
   * The callback functions are called with a fixed frequency of 1 KHz and therefore need to be
   * able to compute outputs within this time frame. Callback functions take two parameters:
   *
   * * A franka::RobotState showing the current robot state.
   * * A franka::Duration to indicate the time since the last callback invocation. Thus, the
   *   duration is zero on the first invocation of the callback function!
   *
   * The following incomplete example shows the general structure of a callback function:
   *
   * @code{.cpp}
   * double time = 0.0;
   * auto control_callback = [&time](const franka::RobotState& robot_state,
   *                                 franka::Duration time_step) -> franka::JointPositions {
   *   time += time_step.toSec();  // Update time at the beginning of the callback.
   *
   *   franka::JointPositions output = getJointPositions(time);
   *
   *   if (time >= max_time) {
   *     // Return MotionFinished at the end of the trajectory.
   *     return franka::MotionFinished(output);
   *   }
   *
   *   return output;
   * }
   * @endcode
   */

  /**
   * Starts a control loop for torque control.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function for torque control. See @ref callback-docs "here"
   * for more details.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<Torques(const RobotState&, franka::Duration)> control_callback);

  /**
   * Starts a control loop for a joint position motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function for torque control. See @ref callback-docs "here"
   * for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<Torques(const RobotState&, franka::Duration)> control_callback,
      std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback);

  /**
   * Starts a control loop for a joint velocity motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function for torque control. See @ref callback-docs "here"
   * for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
               std::function<JointVelocities(const RobotState&, franka::Duration)>
                   motion_generator_callback);

  /**
   * Starts a control loop for a Cartesian pose motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function for torque control. See @ref callback-docs "here"
   * for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<Torques(const RobotState&, franka::Duration)> control_callback,
      std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback);

  /**
   * Starts a control loop for a Cartesian velocity motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function for torque control. See @ref callback-docs "here"
   * for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
               std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                   motion_generator_callback);

  /**
   * Starts a control loop for a joint position motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * Starts a control loop for a joint velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * Starts a control loop for a Cartesian pose motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * Starts a control loop for a Cartesian velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                   motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * @}
   */

  /**
   * Starts a loop for reading the current robot state.
   *
   * Cannot be executed while a control or motion generator loop is running.
   *
   * @param[in] read_callback Callback function for robot state reading.
   *
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void read(std::function<bool(const RobotState&)> read_callback);

  /**
   * Waits for a robot state update and returns it.
   *
   * Cannot be executed while a control or motion generator loop is running.
   *
   * @return Current robot state.
   *
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see Robot::read for a way to repeatedly receive the robot state.
   */
  RobotState readOnce();

  /**
   * @name Commands
   *
   * Commands are executed by communicating with the robot over the network.
   * These functions should therefore not be called from within control or motion generator loops.
   * @{
   */

  /**
   * Returns the parameters of a virtual wall.
   *
   * @param[in] id ID of the virtual wall.
   *
   * @return Parameters of virtual wall.
   *
   * @throw CommandException if an error occurred.
   */
  VirtualWallCuboid getVirtualWall(int32_t id);

  /**
   * Changes the collision behavior.
   *
   * Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] lower_torque_thresholds_acceleration Contact torque thresholds during
   * acceleration/deceleration in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_acceleration Collision torque thresholds during
   * acceleration/deceleration in \f$[Nm]\f$.
   * @param[in] lower_torque_thresholds_nominal Contact torque thresholds in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_nominal Collision torque thresholds in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds_acceleration Contact force thresholds during
   * acceleration/deceleration in \f$[N]\f$.
   * @param[in] upper_force_thresholds_acceleration Collision force thresholds during
   * acceleration/deceleration in \f$[N]\f$.
   * @param[in] lower_force_thresholds_nominal Contact force thresholds in \f$[N]\f$.
   * @param[in] upper_force_thresholds_nominal Collision force thresholds in \f$[N]\f$.
   *
   * @throw CommandException if an error occurred.
   *
   * @see RobotState::cartesian_contact
   * @see RobotState::cartesian_collision
   * @see RobotState::joint_contact
   * @see RobotState::joint_collision
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   */
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                            const std::array<double, 7>& upper_torque_thresholds_acceleration,
                            const std::array<double, 7>& lower_torque_thresholds_nominal,
                            const std::array<double, 7>& upper_torque_thresholds_nominal,
                            const std::array<double, 6>& lower_force_thresholds_acceleration,
                            const std::array<double, 6>& upper_force_thresholds_acceleration,
                            const std::array<double, 6>& lower_force_thresholds_nominal,
                            const std::array<double, 6>& upper_force_thresholds_nominal);

  /**
   * Changes the collision behavior.
   *
   * Set common torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] lower_torque_thresholds Contact torque thresholds in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds Collision torque thresholds in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds Contact force thresholds in \f$[N]\f$.
   * @param[in] upper_force_thresholds Collision force thresholds in \f$[N]\f$.
   *
   * @throw CommandException if an error occurred.
   *
   * @see RobotState::cartesian_contact
   * @see RobotState::cartesian_collision
   * @see RobotState::joint_contact
   * @see RobotState::joint_collision
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   */
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                            const std::array<double, 7>& upper_torque_thresholds,
                            const std::array<double, 6>& lower_force_thresholds,
                            const std::array<double, 6>& upper_force_thresholds);

  /**
   * Sets the impedance for each joint.
   *
   * @param[in] K_theta Joint impedance values \f$K_{\theta}\f$.
   *
   * @throw CommandException if an error occurred.
   */
  void setJointImpedance(
      const std::array<double, 7>& K_theta);  // NOLINT (readability-identifier-naming)

  /**
   * Sets the Cartesian impedance for (x, y, z, roll, pitch, yaw).
   *
   * @param[in] K_x Cartesian impedance values \f$K_x=(x, y, z, R, P, Y)\f$.
   *
   * @throw CommandException if an error occurred.
   */
  void setCartesianImpedance(
      const std::array<double, 6>& K_x);  // NOLINT (readability-identifier-naming)

  /**
   * Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).
   *
   * If a flag is set to true, movement is unlocked.
   *
   * @note
   * Guiding mode can be enabled by pressing the two opposing buttons near the robot's flange.
   *
   * @param[in] guiding_mode Unlocked movement in (x, y, z, R, P, Y) in guiding mode.
   * @param[in] elbow True if the elbow is free in guiding mode, false otherwise.
   *
   * @throw CommandException if an error occurred.
   */
  void setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow);

  /**
   * Sets the transformation \f$^{EE}T_K\f$ from end effector to K frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] EE_T_K Vectorized EE-to-K transformation matrix \f$^{EE}T_K\f$, column-major.
   *
   * @throw CommandException if an error occurred.
   *
   * @see Robot for an explanation of the K frame.
   */
  void setK(const std::array<double, 16>& EE_T_K);  // NOLINT (readability-identifier-naming)

  /**
   * Sets the transformation \f$^FT_{EE}\f$ from flange to end effector frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] F_T_EE Vectorized flange-to-EE transformation matrix \f$^FT_{EE}\f$, column-major.
   *
   * @throw CommandException if an error occurred.
   *
   * @see RobotState::O_T_EE for end effector pose in world base frame.
   * @see Robot for an explanation of the EE frame.
   */
  void setEE(const std::array<double, 16>& F_T_EE);  // NOLINT (readability-identifier-naming)

  /**
   * Sets dynamic parameters of a payload.
   *
   * @note
   * This is not for setting end effector parameters, which have to be set in a configuration file.
   *
   * @param[in] load_mass Mass of the load in \f$[kg]\f$.
   * @param[in] F_x_Cload Translation from flange to center of mass of load
   * \f$^Fx_{C_\text{load}}\f$ in \f$[m]\f$.
   * @param[in] load_inertia Inertia matrix \f$I_\text{load}\f$ in \f$[kg \times m^2]\f$,
   * column-major.
   *
   * @throw CommandException if an error occurred.
   */
  void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,  // NOLINT (readability-identifier-naming)
               const std::array<double, 9>& load_inertia);

  /**
   * Runs automatic error recovery on the robot.
   *
   * Automatic error recovery e.g. resets the robot after a collision occurred.
   *
   * @throw CommandException if an error occurred.
   */
  void automaticErrorRecovery();

  /**
   * Stops all currently running motions.
   *
   * If a control or motion generator loop is running in another thread, it will be preempted
   * with a franka::ControlException.
   *
   * @throw CommandException if an error occurred.
   */
  void stop();

  /**
   * @}
   */

  /**
   * Loads the model library from the robot.
   *
   * @return Model instance.
   *
   * @throw ModelException if the model library cannot be loaded.
   */
  Model loadModel();

  /**
   * Returns the software version reported by the connected server.
   *
   * @return Software version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  class Impl;

 private:
  std::unique_ptr<Impl> impl_;
  std::mutex control_mutex_;
};

}  // namespace franka
