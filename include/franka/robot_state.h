#pragma once

#include <array>
#include <ostream>

#include <franka/errors.h>

/**
 * @file robot_state.h
 * Contains the RobotState struct.
 */

namespace franka {

/**
 * Describes FRANKA's state.
 */
struct RobotState {
  /**
   * \f$^OT_{EE}\f$
   * Measured end effector pose in world base frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE{};  // NOLINT (readability-identifier-naming)

  /**
   * \f$^OT_{EE}\f$
   * Last desired end effector pose of motion generation in world base frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE_d{};  // NOLINT (readability-identifier-naming)

  /**
   * Elbow pose.
   * Elbow is defined as the joint position of 3rd joint and the sign of the 4th
   * joint. Unit: \f$[rad]\f$
   */
  std::array<double, 2> elbow{};

  /**
   * Desired elbow pose.
   * Elbow is defined as the joint position of 3rd joint and the sign of the 4th
   * joint. Unit: \f$[rad]\f$
   */
  std::array<double, 2> elbow_d{};

  /**
   * \f$\tau_{J}\f$
   * Measured joint torque. Unit: \f$[Nm]\f$
   */
  std::array<double, 7> tau_J{};  // NOLINT (readability-identifier-naming)

  /**
   * \f$\dot{\tau_{J}}\f$
   * Derivative of measured joint torque. Unit: \f$[\frac{Nm}{s}]\f$
   */
  std::array<double, 7> dtau_J{};  // NOLINT (readability-identifier-naming)

  /**
   * \f$q\f$
   * Measured link side position (joint values). Unit: \f$[rad]\f$
   */
  std::array<double, 7> q{};

  /**
   * \f$\dot{q}\f$
   * Measured link side velocity. Unit: \f$[\frac{rad}{s}]\f$
   */
  std::array<double, 7> dq{};

  /**
   * \f$q_d\f$
   * Desired link side position. Unit: \f$[rad]\f$
   */
  std::array<double, 7> q_d{};

  /**
   * Indicates which contact level is activated in which joint. After contact
   * disappears, value turns to zero.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   */
  std::array<double, 7> joint_contact{};

  /**
   * Indicates which contact level is activated in which cartesian dimension (x,
   * y, z, roll, pitch, yaw). After contact disappears, value turns to zero.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   */
  std::array<double, 6> cartesian_contact{};

  /**
   * Indicates which contact level is activated in which joint. After contact
   * disappears, the value stays the same until a reset command is sent.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   * @see Robot::automaticErrorRecovery for performing a reset after a
   * collision.
   */
  std::array<double, 7> joint_collision{};

  /**
   * Indicates which contact level is activated in which cartesian dimension (x,
   * y, z, roll, pitch, yaw). After contact disappears, the value stays the same
   * until a reset command is sent.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   * @see Robot::automaticErrorRecovery for performing a reset after a
   * collision.
   */
  std::array<double, 6> cartesian_collision{};

  /**
   * \f$\hat{\tau}_{\text{ext}}\f$
   * External torque, filtered. Unit: \f$[Nm]\f$.
   */
  std::array<double, 7> tau_ext_hat_filtered{};

  /**
   * \f$^OF_{K,\text{ext}}\f$
   * External wrench (force, torque) scaled by a factor acting on K frame,
   * expressed relative to the base frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
   */
  std::array<double, 6> O_F_ext_hat_K{};  // NOLINT (readability-identifier-naming)

  /**
   * \f$^{K}F_{K,\text{ext}}\f$
   * External wrench (force, torque) acting on K frame, expressed relative to
   * the end effector frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
   */
  std::array<double, 6> K_F_ext_hat_K{};  // NOLINT (readability-identifier-naming)

  /**
   * Current error state.
   */
  Errors errors{};

  /**
   * Contains the errors that aborted the previous motion.
   */
  Errors reflex_reasons{};

  /**
   * Strictly increasing sequence number for each received robot state.
   */
  uint32_t sequence_number{};
};

/**
 * Streams the RobotState according to the following format: {field_name_1:
 * [0,0,0,0,0,0,0], field_name_2: [0,0,0,0,0,0], ...}
 */
std::ostream& operator<<(std::ostream& ostream, const franka::RobotState& robot_state);

}  // namespace franka
