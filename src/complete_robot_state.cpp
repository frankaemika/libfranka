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
  robot_state_.O_T_EE = rcu_robot_state.O_T_EE;
  robot_state_.elbow = rcu_robot_state.elbow;
  robot_state_.tau_J = rcu_robot_state.tau_J;
  robot_state_.dtau_J = rcu_robot_state.dtau_J;
  robot_state_.q = rcu_robot_state.q;
  robot_state_.dq = rcu_robot_state.dq;
  robot_state_.q_d = rcu_robot_state.q_d;
  robot_state_.joint_contact = rcu_robot_state.joint_contact;
  robot_state_.cartesian_contact = rcu_robot_state.cartesian_contact;
  robot_state_.joint_collision = rcu_robot_state.joint_collision;
  robot_state_.cartesian_collision = rcu_robot_state.cartesian_collision;
  robot_state_.tau_ext_hat_filtered = rcu_robot_state.tau_ext_hat_filtered;
  robot_state_.O_F_ext_hat_K = rcu_robot_state.O_F_ext_hat_K;
  robot_state_.K_F_ext_hat_K = rcu_robot_state.K_F_ext_hat_K;
  return *this;
}

const franka::RobotState& CompleteRobotState::robotState() const noexcept {
  return robot_state_;
}

const research_interface::RobotState& CompleteRobotState::rcuRobotState() const noexcept {
  return rcu_robot_state_;
}

}  // namespace franka
