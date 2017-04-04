#include "control_loop.h"

namespace franka {

ControlLoop::ControlLoop(Robot::Impl& robot_impl,
    ControlCallback control_callback)
  : robot_impl_(robot_impl),
    control_callback_(std::move(control_callback))
{
  if (control_callback_) {
    setCurrentThreadToRealtime();
    robot_impl_.startController();
  }
}

ControlLoop::~ControlLoop() {
  if (control_callback_) {
    robot_impl_.stopController();
  }
}

void ControlLoop::operator()() {
  while (robot_impl_.update()) {
    if (!spinOnce()) {
      break;
    }
  }
}

bool ControlLoop::spinOnce() {
  if (control_callback_) {
    Torques control_output = control_callback_(robot_impl_.robotState());
    if (typeid(control_output) == typeid(Stop)) {
      return false;
    }
    convertTorques(control_output, &robot_impl_.controllerCommand());
  }

  return true;
}

void ControlLoop::convertTorques(const Torques& torques, research_interface::ControllerCommand* command) {
  command->tau_J_d = torques.tau_J;
}

void ControlLoop::setCurrentThreadToRealtime() {
  // TODO
}

}  // namespace franka
