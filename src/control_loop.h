#pragma once

#include <functional>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>
#include "robot_control.h"

namespace franka {

class ControlLoop {
 public:
  using ControlCallback = std::function<Torques(const RobotState&)>;

  ControlLoop(RobotControl& robot, ControlCallback control_callback, bool start_controller = false);
  virtual ~ControlLoop();

  void operator()();

 protected:
  virtual bool spinOnce();

 private:
  void setCurrentThreadToRealtime(RealtimeConfig config);

 protected:
  RobotControl& robot_;

 private:
  ControlCallback control_callback_;
  bool start_controller_;
};

}  // namespace franka
