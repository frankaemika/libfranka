#pragma once

#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>

namespace franka {

RobotState convertRobotState(const research_interface::RobotState& robot_state) noexcept;

};  // namespace franka
