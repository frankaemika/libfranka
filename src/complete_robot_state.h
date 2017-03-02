#pragma once

#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>

namespace franka {

class CompleteRobotState : public RobotState {
 public:
  CompleteRobotState() noexcept;
  const research_interface::RobotState& rcuRobotState() const noexcept;
  CompleteRobotState& operator=(const research_interface::RobotState& rcu_robot_state) noexcept;

 private:
  research_interface::RobotState rcu_robot_state;
};

}  // namespace franka
