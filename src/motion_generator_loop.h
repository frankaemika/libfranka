#pragma once

#include <cmath>
#include <functional>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

#include "control_loop_base.h"

namespace franka {

template <typename T>
class MotionGeneratorLoop : public ControlLoopBase {
 public:
  using MotionGeneratorCallback = std::function<T(const RobotState&, franka::Duration)>;
  static constexpr research_interface::robot::Move::Deviation kDefaultDeviation{10.0, 3.12,
                                                                                2 * M_PI};

  MotionGeneratorLoop(RobotControl& robot,
                      ControlCallback control_callback,
                      MotionGeneratorCallback motion_callback);

  MotionGeneratorLoop(RobotControl& robot,
                      ControllerMode controller_mode,
                      MotionGeneratorCallback motion_callback);

  ~MotionGeneratorLoop() noexcept override;

  void operator()();

 protected:
  using ControlLoopBase::spinOnce;
  bool spinOnce(const RobotState& robot_state,
                franka::Duration time_step,
                research_interface::robot::MotionGeneratorCommand* command);

 private:
  void convertMotion(const T& motion, research_interface::robot::MotionGeneratorCommand* command);

  const MotionGeneratorCallback motion_callback_;
  uint32_t motion_id_;
};

template class MotionGeneratorLoop<JointPositions>;
template class MotionGeneratorLoop<JointVelocities>;
template class MotionGeneratorLoop<CartesianPose>;
template class MotionGeneratorLoop<CartesianVelocities>;

}  // namespace franka
