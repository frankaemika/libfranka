#pragma once

#include <chrono>
#include <functional>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

#include "robot_control.h"

namespace franka {

void setCurrentThreadToRealtime(bool throw_on_error);
bool hasRealtimeKernel();

class ControlLoopBase {
 public:
  using ControlCallback = std::function<Torques(const RobotState&, franka::Duration)>;

  ControlLoopBase(RobotControl& robot, ControlCallback control_callback);
  virtual ~ControlLoopBase() noexcept = default;

 protected:
  bool spinOnce(const RobotState& robot_state,
                franka::Duration time_step,
                research_interface::robot::ControllerCommand* command);

  RobotControl& robot_;
  const ControlCallback control_callback_;
};

}  // namespace franka
