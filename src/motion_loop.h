#pragma once

#include <cmath>
#include <functional>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

#include "robot_control.h"

namespace franka {

template <typename T>
class MotionLoop {
 public:
  using ControlCallback = std::function<Torques(const RobotState&)>;
  using MotionGeneratorCallback = std::function<T(const RobotState&)>;

  static constexpr research_interface::robot::Move::Deviation kDefaultDeviation{10.0, 3.12,
                                                                                2 * M_PI};

  MotionLoop(RobotControl& robot, ControlCallback control_callback);

  MotionLoop(RobotControl& robot,
             ControlCallback control_callback,
             MotionGeneratorCallback motion_callback);

  MotionLoop(RobotControl& robot,
             ControllerMode controller_mode,
             MotionGeneratorCallback motion_callback);

  ~MotionLoop() noexcept;

  void operator()();

 protected:
  bool motionGeneratorSpinOnce(const RobotState& robot_state,
                               research_interface::robot::MotionGeneratorCommand* command);
  bool controllerSpinOnce(const RobotState& robot_state,
                          research_interface::robot::ControllerCommand* command);

 private:
  void setRealTime();
  void convertMotion(const T& motion, research_interface::robot::MotionGeneratorCommand* command);

  RobotControl& robot_;
  const MotionGeneratorCallback motion_callback_;
  const ControlCallback control_callback_;
};

template class MotionLoop<JointPositions>;
template class MotionLoop<JointVelocities>;
template class MotionLoop<CartesianPose>;
template class MotionLoop<CartesianVelocities>;
template class MotionLoop<Torques>;

}  // namespace franka
