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
  robot_.throwOnMotionError(robot_state, nullptr);

  Duration previous_time = robot_state.time;
  research_interface::robot::ControllerCommand command{};
  while (spinOnce(robot_state, robot_state.time - previous_time, &command)) {
    previous_time = robot_state.time;
    robot_state = robot_.update(nullptr, &command);
    robot_.throwOnMotionError(robot_state, nullptr);
  }
}

}  // namespace franka
