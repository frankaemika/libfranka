#pragma once

#include <functional>

#include <franka/robot_state.h>
#include "robot_impl.h"

namespace franka {

class ControlLoop {
 public:
  using ControlCallback = std::function<Torques(const RobotState&)>;

  ControlLoop(Robot::Impl& robot_impl, ControlCallback control_callback);
  virtual ~ControlLoop();

  void operator()();

 protected:
  virtual bool spinOnce();

 private:
  void convertTorques(const Torques& torques,
                      research_interface::ControllerCommand* command);
  void setCurrentThreadToRealtime();

 protected:
  Robot::Impl& robot_impl_;

 private:
  ControlCallback control_callback_;
};

}  // namespace franka
