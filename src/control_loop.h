#pragma once

#include <functional>

#include <franka/robot_state.h>
#include "robot_impl.h"

namespace franka {

struct ControlLoop {
  using ControlCallback = std::function<Torques(const RobotState&)>;

  ControlLoop(Robot::Impl& robot_impl,
      ControlCallback control_callback)
    : robot_impl_(robot_impl),
    control_callback_(std::move(control_callback))
  {
    if (control_callback_) {
      setCurrentThreadToRealtime();
      robot_impl_.startController();
    }
  }

  virtual ~ControlLoop() {
    if (control_callback_) {
      robot_impl_.stopController();
    }
  }

  void operator()() {
    while (robot_impl_.update()) {
      if (!spinOnce()) {
        break;
      }
    }
  }

 protected:
  virtual bool spinOnce() {
    if (control_callback_) {
      Torques control_output = control_callback_(robot_impl_.robotState());
      if (typeid(control_output) == typeid(Stop)) {
        return false;
      }
      convertTorques(control_output, &robot_impl_.controllerCommand());
    }

    return true;
  }

  void convertTorques(const Torques& torques, research_interface::ControllerCommand* command) {
    // TODO
  }

  void setCurrentThreadToRealtime() {
    // TODO
  }

  Robot::Impl& robot_impl_;
  ControlCallback control_callback_;
};

}  // namespace franka
