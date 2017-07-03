#pragma once

#include <memory>
#include <string>

#include <franka/command_types.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

/**
 * @file robot.h
 * Contains the Robot class.
 */

namespace franka {

class Model;

/**
 * Maintains a connection to FRANKA CONTROL, provides the current robot state,
 * gives access to the model library and allows to execute commands,
 * motions, and torque control.
 *
 * @note
 * Before using this functionality, make sure FRANKA's brakes have been released.
 *
 * @par End effector frame
 * While the end effector parameters are set in a configuration file, it is
 * possible to change the end effector frame with Robot::setEE.
 *
 * @par K frame
 * The K frame is used for Cartesian impedance control and measuring forces
 * and torques. It can be set with Robot::setK.
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
   * @param[in] franka_address IP/hostname of FRANKA CONTROL
   * @param[in] realtime_config if set to Enforce, an exception will be thrown
   * if realtime priority cannot be set when required. Setting realtime_config
   * to Ignore disables this behavior.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this library is not supported by FRANKA CONTROL.
   * @throw ProtocolException if data received from the host is invalid.
   */
  explicit Robot(const std::string& franka_address,
                 RealtimeConfig realtime_config = RealtimeConfig::kEnforce);
  /**
   * Closes the connection.
   */
  ~Robot() noexcept;

  /**
   * @name Motion generation and torque control
   * The callbacks given to the control functions are called with a fixed
   * frequency of 1 \f$[KHz]\f$ and therefore need to be able to compute
   * outputs within this time frame.
   * @{
   */

  /**
   * Starts a control loop for torque control.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] control_callback Callback function for torque control.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<Torques(const RobotState&)> control_callback);

  /**
   * Starts a control loop for a joint position motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] control_callback Callback function for torque control.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<JointPositions(const RobotState&)> motion_generator_callback,
               std::function<Torques(const RobotState&)> control_callback);

  /**
   * Starts a control loop for a joint position motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<JointPositions(const RobotState&)> motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * Starts a control loop for a joint velocity motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] control_callback Callback function for torque control.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<JointVelocities(const RobotState&)> motion_generator_callback,
               std::function<Torques(const RobotState&)> control_callback);

  /**
   * Starts a control loop for a joint velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<JointVelocities(const RobotState&)> motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * Starts a control loop for a Cartesian pose motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] control_callback Callback function for torque control.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<CartesianPose(const RobotState&)> motion_generator_callback,
               std::function<Torques(const RobotState&)> control_callback);

  /**
   * Starts a control loop for a Cartesian pose motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<CartesianPose(const RobotState&)> motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * Starts a control loop for a Cartesian velocity motion generator with torque control.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] control_callback Callback function for torque control.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<CartesianVelocities(const RobotState&)> motion_generator_callback,
               std::function<Torques(const RobotState&)> control_callback);

  /**
   * Starts a control loop for a Cartesian velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   *
   * @param[in] motion_generator_callback Callback function for motion generation.
   * @param[in] controller_mode Controller to use to execute the motion.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   * @throw RealtimeException if realtime priority can not be set for the current thread.
   *
   * @see Robot::Robot to change behavior if realtime priority can not be set.
   */
  void control(std::function<CartesianVelocities(const RobotState&)> motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance);

  /**
   * @}
   */

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
   * @return Current robot state.
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw ProtocolException if received data has invalid format.
   *
   * @see Robot::read for a way to repeatedly receive the robot state.
   */
  RobotState readOnce();

  /**
   * @name Commands
   * Commands are executed by communicating with FRANKA CONTROL over the network.
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
   * Changes the type of controller used for idle mode.
   *
   * FRANKA is in idle mode when holding the current position, i.e. when no motion
   * is being performed and guiding mode is not active.
   *
   * The controller mode is reset when a motion is started or guiding mode is activated.
   *
   * @param[in] controller_mode Controller mode.
   *
   * @throw CommandException if an error occurred.
   *
   * @see setGuidingMode for an explanation of guiding mode.
   * @see control to start a motion.
   */
  void setIdleControllerMode(ControllerMode controller_mode);

  /**
   * Changes the collision behavior.
   *
   * Set separate torque and force boundaries for acceleration/deceleration
   * and constant velocity movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as
   * contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision
   * and cause FRANKA to stop moving.
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
   * Set common torque and force boundaries for acceleration/deceleration and
   * constant velocity movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause FRANKA to
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
   * Guiding mode can be enabled by pressing the two opposing buttons near FRANKA's flange.
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
   * @see RobotState::O_T_EE for end-effector pose in world base frame.
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
   * Sets a time scaling factor for all motion generators.
   *
   * Slows down or speeds up a trajectory.
   *
   * @param[in] factor Time scaling factor \f$\in [0, 1]\f$.
   *
   * @throw CommandException if an error occurred.
   */
  void setTimeScalingFactor(double factor);

  /**
   * Runs automatic error recovery on FRANKA.
   *
   * Automatic error recovery e.g. resets FRANKA after a collision occurred.
   *
   * @throw CommandException if an error occurred.
   */
  void automaticErrorRecovery();

  /**
   * @}
   */

  /**
   * Loads the Model Library from FRANKA CONTROL.
   *
   * @throw ModelException if the model library cannot be loaded.
   *
   * @return Correctly initialized model library.
   */
  std::shared_ptr<Model> loadModel();

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
};

}  // namespace franka
