#pragma once

#include <array>
#include <ostream>

/** @file robot_state.h
 * Contains the RobotState struct.
*/

namespace franka {

/**
 * RobotState struct describes FRANKA's state.
 */
struct RobotState {
  RobotState();

  /**
   * \f$q_{start}\f$
   * Motion generator's start pose in joint space. Unit: \f$[rad]\f$
   */
  std::array<double, 7> q_start;
  /**
   * \f$^OT_{EE,start}\f$
   * Motion generator's start pose in world base frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE_start;
  /**
   * Elbow start position.
   * Elbow is defined as the joint position of 3rd joint and the sign of the 4th
   * joint. Unit: \f$[rad]\f$
   */
  std::array<double, 2> elbow_start;
  /**
   * \f$\tau_{J}\f$
   * Measured joint torque. Unit: \f$[Nm]\f$
   */
  std::array<double, 7> tau_J;
  /**
   * \f$\dot{\tau_{J}}\f$
   * Derivative of measured joint torque. Unit: \f$[\frac{Nm}{s}]\f$
   */
  std::array<double, 7> dtau_J;
  /**
   * \f$q\f$
   * Measured link side position (joint values). Unit: \f$[rad]\f$
   */
  std::array<double, 7> q;
  /**
   * \f$\dot{q}\f$
   * Measured link side velocity. Unit: \f$[\frac{rad}{s}]\f$
   */
  std::array<double, 7> dq;
  /**
   * \f$q_d\f$
   * Desired link side position. Unit: \f$[rad]\f$
   */
  std::array<double, 7> q_d;
  /**
   * Indicates which contact level is activated in which joint. After contact
   * disappears, value turns to zero.
   */
  std::array<double, 7> joint_contact;
  /**
   * Indicates which contact level is activated in which cartesian dimension (x,
   * y, z, roll, pitch, yaw). After contact disappears, value turns to zero.
   */
  std::array<double, 6> cartesian_contact;
  /**
   * Indicates which contact level is activated in which joint. After contact
   * disappears, the value stays the same until a reset command is sent.
   */
  std::array<double, 7> joint_collision;
  /**
   * Indicates which contact level is activated in which cartesian dimension (x,
   * y, z, roll, pitch, yaw). After contact disappears, the value stays the same
   * until a reset command is sent.
   */
  std::array<double, 6> cartesian_collision;
  /**
   * \f$\hat{\tau}_{ext}\f$
   * External torque, filtered. Unit: \f$[Nm]\f$
   */
  std::array<double, 7> tau_ext_hat_filtered;
  /**
   * \f$^OF_{EE,ext}\f$
   * External wrench(force, torque) acting on end-effector relative to the base
   * frame scaled by a factor. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$
   */
  std::array<double, 6> O_F_ext_hat_EE;
  /**
   * \f$^{EE}F_{EE,ext}\f$
   * External wrench(force, torque) acting on end-effector relative to
   * end-effector frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$
   */
  std::array<double, 6> EE_F_ext_hat_EE;
};

/**
 * Streams the RobotState according to the following format: {field_name_1:
 * [0,0,0,0,0,0,0], field_name_2: [0,0,0,0,0,0], ...}
 */
std::ostream& operator<<(std::ostream& ostream,
                         const franka::RobotState& robot_state);

}  // namespace franka
