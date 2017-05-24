#pragma once

#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>

namespace franka {

class CompleteRobotState {
 public:
  const franka::RobotState& robotState() const noexcept;
  const research_interface::RobotState& rcuRobotState() const noexcept;
  CompleteRobotState& operator=(const research_interface::RobotState& rcu_robot_state) noexcept;

 private:
  franka::RobotState robot_state_;
  research_interface::RobotState rcu_robot_state_;
};

}  // namespace franka
