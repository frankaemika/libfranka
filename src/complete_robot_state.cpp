#include "complete_robot_state.h"

#include <algorithm>
#include <cstring>

namespace franka {

CompleteRobotState::CompleteRobotState() noexcept {
  std::memset(this, 0, sizeof(*this));
}

CompleteRobotState& CompleteRobotState::operator=(const research_interface::RobotState& robot_state) noexcept {
  rcu_robot_state = robot_state;
  std::copy(robot_state.q_start.cbegin(), robot_state.q_start.cend(),
            q_start.begin());
  std::copy(robot_state.O_T_EE_start.cbegin(), robot_state.O_T_EE_start.cend(),
            O_T_EE_start.begin());
  std::copy(robot_state.elbow_start.cbegin(), robot_state.elbow_start.cend(),
            elbow_start.begin());
  std::copy(robot_state.tau_J.cbegin(), robot_state.tau_J.cend(),
            tau_J.begin());
  std::copy(robot_state.dtau_J.cbegin(), robot_state.dtau_J.cend(),
            dtau_J.begin());
  std::copy(robot_state.q.cbegin(), robot_state.q.cend(),
            q.begin());
  std::copy(robot_state.dq.cbegin(), robot_state.dq.cend(),
            dq.begin());
  std::copy(robot_state.q_d.cbegin(), robot_state.q_d.cend(),
            q_d.begin());
  std::copy(robot_state.joint_contact.cbegin(),
            robot_state.joint_contact.cend(),
            joint_contact.begin());
  std::copy(robot_state.cartesian_contact.cbegin(),
            robot_state.cartesian_contact.cend(),
            cartesian_contact.begin());
  std::copy(robot_state.joint_collision.cbegin(),
            robot_state.joint_collision.cend(),
            joint_collision.begin());
  std::copy(robot_state.cartesian_collision.cbegin(),
            robot_state.cartesian_collision.cend(),
            cartesian_collision.begin());
  std::copy(robot_state.tau_ext_hat_filtered.cbegin(),
            robot_state.tau_ext_hat_filtered.cend(),
            tau_ext_hat_filtered.begin());
  std::copy(robot_state.O_F_ext_hat_K.cbegin(),
            robot_state.O_F_ext_hat_K.cend(),
            O_F_ext_hat_K.begin());
  std::copy(robot_state.K_F_ext_hat_K.cbegin(),
            robot_state.K_F_ext_hat_K.cend(),
            K_F_ext_hat_K.begin());
  return *this;
}


const research_interface::RobotState& CompleteRobotState::rcuRobotState() const noexcept {
  return rcu_robot_state;
}

}  // namespace franka
