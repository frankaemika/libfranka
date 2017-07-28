#include "control_loop.h"

#include <exception>

namespace franka {

ControlLoop::ControlLoop(RobotControl& robot, ControlCallback control_callback)
    : ControlLoopBase(robot, std::move(control_callback)) {
  if (!control_callback_) {
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }

  robot.startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                    research_interface::robot::Move::MotionGeneratorMode::kIdle, kDefaultDeviation,
                    kDefaultDeviation);
}

void ControlLoop::operator()() {
  RobotState robot_state = robot_.update();
  Duration previous_time = robot_state.time;
  research_interface::robot::ControllerCommand controller_command{};
  research_interface::robot::MotionGeneratorCommand zero_motion_generator_command{};
  while (spinOnce(robot_state, robot_state.time - previous_time, &controller_command)) {
    previous_time = robot_state.time;
    robot_state = robot_.update(&zero_motion_generator_command, &controller_command);
  }
}

}  // namespace franka
