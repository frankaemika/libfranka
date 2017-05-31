#pragma once

#include <functional>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>

#include "control_loop_base.h"

namespace franka {

template <typename T>
class MotionGeneratorLoop : public ControlLoopBase {
 public:
  using MotionGeneratorCallback = std::function<T(const RobotState&)>;

  MotionGeneratorLoop(RobotControl& robot,
                      ControlCallback control_callback,
                      MotionGeneratorCallback motion_callback);

  MotionGeneratorLoop(RobotControl& robot,
                      ControllerMode controller_mode,
                      MotionGeneratorCallback motion_callback);

  ~MotionGeneratorLoop() override;

  void operator()();

 private:
  bool spinMotionOnce(const RobotState& robot_state,
                      research_interface::MotionGeneratorCommand* command);
  void convertMotion(const T& motion, research_interface::MotionGeneratorCommand* command);

  MotionGeneratorCallback motion_callback_;
};

template class MotionGeneratorLoop<JointPositions>;
template class MotionGeneratorLoop<JointVelocities>;
template class MotionGeneratorLoop<CartesianPose>;
template class MotionGeneratorLoop<CartesianVelocities>;

}  // namespace franka
