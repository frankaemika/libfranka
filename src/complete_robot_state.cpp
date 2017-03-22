#include "complete_robot_state.h"

#include <algorithm>
#include <cstring>

namespace franka {

CompleteRobotState::CompleteRobotState() noexcept {
  std::memset(this, 0, sizeof(*this));
}

CompleteRobotState& CompleteRobotState::operator=(
    const research_interface::RobotState& rcu_robot_state) noexcept {
  rcu_robot_state_ = rcu_robot_state;
  std::copy(rcu_robot_state.O_T_EE.cbegin(),
            rcu_robot_state.O_T_EE.cend(),
            robot_state_.O_T_EE.begin());
  std::copy(rcu_robot_state.elbow.cbegin(),
            rcu_robot_state.elbow.cend(),
            robot_state_.elbow.begin());
  std::copy(rcu_robot_state.tau_J.cbegin(), rcu_robot_state.tau_J.cend(),
            robot_state_.tau_J.begin());
  std::copy(rcu_robot_state.dtau_J.cbegin(), rcu_robot_state.dtau_J.cend(),
            robot_state_.dtau_J.begin());
  std::copy(rcu_robot_state.q.cbegin(), rcu_robot_state.q.cend(),
            robot_state_.q.begin());
  std::copy(rcu_robot_state.dq.cbegin(), rcu_robot_state.dq.cend(),
            robot_state_.dq.begin());
  std::copy(rcu_robot_state.q_d.cbegin(), rcu_robot_state.q_d.cend(),
            robot_state_.q_d.begin());
  std::copy(rcu_robot_state.joint_contact.cbegin(),
            rcu_robot_state.joint_contact.cend(),
            robot_state_.joint_contact.begin());
  std::copy(rcu_robot_state.cartesian_contact.cbegin(),
            rcu_robot_state.cartesian_contact.cend(),
            robot_state_.cartesian_contact.begin());
  std::copy(rcu_robot_state.joint_collision.cbegin(),
            rcu_robot_state.joint_collision.cend(),
            robot_state_.joint_collision.begin());
  std::copy(rcu_robot_state.cartesian_collision.cbegin(),
            rcu_robot_state.cartesian_collision.cend(),
            robot_state_.cartesian_collision.begin());
  std::copy(rcu_robot_state.tau_ext_hat_filtered.cbegin(),
            rcu_robot_state.tau_ext_hat_filtered.cend(),
            robot_state_.tau_ext_hat_filtered.begin());
  std::copy(rcu_robot_state.O_F_ext_hat_K.cbegin(),
            rcu_robot_state.O_F_ext_hat_K.cend(),
            robot_state_.O_F_ext_hat_K.begin());
  std::copy(rcu_robot_state.K_F_ext_hat_K.cbegin(),
            rcu_robot_state.K_F_ext_hat_K.cend(),
            robot_state_.K_F_ext_hat_K.begin());
  return *this;
}

const franka::RobotState& CompleteRobotState::robotState() const noexcept {
  return robot_state_;
}

const research_interface::RobotState& CompleteRobotState::rcuRobotState() const
    noexcept {
  return rcu_robot_state_;
}

}  // namespace franka
