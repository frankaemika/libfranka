#include "control_loop.h"

#include "conversion.h"

namespace franka {

ControlLoop::ControlLoop(RobotControl& robot, ControlCallback control_callback)
    : ControlLoopBase(robot, std::move(control_callback)) {
  if (!control_callback) {
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }

  robot_.startController();
}

ControlLoop::~ControlLoop() {
  robot_.stopController();
}

void ControlLoop::operator()() {
  RobotState robot_state = convertRobotState(robot_.update({}));
  research_interface::ControllerCommand command{};
  while (spinControlOnce(robot_state, &command)) {
    robot_state = convertRobotState(robot_.update(command));
  }
}

}  // namespace franka
