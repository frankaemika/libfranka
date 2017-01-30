#pragma once

#include <array>
#include <ostream>

/** \file robot_state.h
    \brief Contains the RobotState struct.
*/

namespace franka {

/**
 * RobotState struct describes FRANKA's state.
 */
struct RobotState {
  RobotState();

  /**
   * Initial q.
   */
  std::array<double, 7> q_start;
  std::array<double, 16> O_T_EE_start;
  std::array<double, 2> elbow_start;
  std::array<double, 7> tau_J;
  std::array<double, 7> dtau_J;
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> q_d;
  std::array<double, 7> joint_contact;
  std::array<double, 6> cartesian_contact;
  std::array<double, 7> joint_collision;
  std::array<double, 6> cartesian_collision;
  std::array<double, 7> tau_ext_hat_filtered;
  std::array<double, 6> O_F_ext_hat_EE;
  std::array<double, 6> EE_F_ext_hat_EE;
};

std::ostream& operator<<(std::ostream& ostream,
                         const franka::RobotState& robot_state);

}  // namespace franka
