#include "control_loop.h"

#include <exception>

namespace franka {

ControlLoop::ControlLoop(RobotControl& robot, ControlCallback control_callback)
    : ControlLoopBase(robot, std::move(control_callback)) {
  if (!control_callback_) {
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }

  robot_.startController();
}

ControlLoop::~ControlLoop() noexcept {
  try {
    robot_.stopController();
  } catch (...) {
  }
}

void ControlLoop::operator()() {
  RobotState robot_state = robot_.update();
  // The first robot state given to a control loop should always show zero ticks.
  robot_state.ticks = 0;

  research_interface::robot::ControllerCommand command{};
  while (spinOnce(robot_state, &command)) {
    robot_state = robot_.update(nullptr, &command);
  }
}

}  // namespace franka
