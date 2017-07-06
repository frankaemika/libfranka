#pragma once

#include <functional>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

#include "robot_control.h"

namespace franka {

void setCurrentThreadToRealtime(RealtimeConfig config);

class ControlLoopBase {
 public:
  using ControlCallback = std::function<Torques(const RobotState&)>;

  ControlLoopBase(RobotControl& robot, ControlCallback control_callback);
  virtual ~ControlLoopBase() noexcept = default;

 protected:
  bool spinOnce(const RobotState& robot_state,
                research_interface::robot::ControllerCommand* command);

  RobotControl& robot_;
  const ControlCallback control_callback_;
};

}  // namespace franka
