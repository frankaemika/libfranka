#pragma once

#include "control_loop.h"

namespace franka {

template <typename T>
class MotionGeneratorLoop : public ControlLoop {
 public:
  using MotionGeneratorCallback = std::function<T(const RobotState&)>;

  MotionGeneratorLoop(RobotControl& robot,
                      ControlCallback control_callback,
                      MotionGeneratorCallback motion_callback);

  ~MotionGeneratorLoop() override;

 protected:
  bool spinOnce() override;

 private:
  void convertMotion(const T& motion, research_interface::MotionGeneratorCommand* command);

  MotionGeneratorCallback motion_callback_;
};

template class MotionGeneratorLoop<JointPositions>;
template class MotionGeneratorLoop<JointVelocities>;
template class MotionGeneratorLoop<CartesianPose>;
template class MotionGeneratorLoop<CartesianVelocities>;

}  // namespace franka
