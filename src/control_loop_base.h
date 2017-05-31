#pragma once

#include <functional>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>

#include "robot_control.h"

namespace franka {

class ControlLoopBase {
 public:
  using ControlCallback = std::function<Torques(const RobotState&)>;

  ControlLoopBase(RobotControl& robot, ControlCallback control_callback);
  virtual ~ControlLoopBase() = default;

 protected:
  bool spinControlOnce(const RobotState& robot_state,
                       research_interface::ControllerCommand* command);
  void setCurrentThreadToRealtime(RealtimeConfig config);

  RobotControl& robot_;
  ControlCallback control_callback_;
};

}  // namespace franka
