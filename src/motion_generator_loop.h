#pragma once

#include "control_loop.h"
#include "motion_traits.h"

namespace franka {

template <typename T>
struct MotionGeneratorLoop : public ControlLoop {
  using MotionGeneratorCallback = std::function<T(const RobotState&)>;

  MotionGeneratorLoop(Robot::Impl& robot_impl,
      ControlCallback control_callback,
      MotionGeneratorCallback motion_callback)
    : ControlLoop(robot_impl, control_callback),
    motion_callback_(motion_callback)
  {
    if (motion_callback_) {
      robot_impl_.startMotionGenerator(MotionTraits<T>::Type);
    }
  }

  ~MotionGeneratorLoop() {
    if (motion_callback_) {
      robot_impl_.stopMotionGenerator();
    }
  }

 protected:
  bool spinOnce() override {
    if (motion_callback_) {
      T motion_output = motion_callback_(robot_impl_.robotState());
      if (typeid(motion_output) == typeid(Stop)) {
        return false;
      }
      convertMotion(motion_output, &robot_impl_.motionCommand());
    }

    return ControlLoop::spinOnce();
  }

  void convertMotion(const T& motion, research_interface::MotionGeneratorCommand* command);

  MotionGeneratorCallback motion_callback_;
};

template<>
void MotionGeneratorLoop<JointValues>::convertMotion(const JointValues& motion, research_interface::MotionGeneratorCommand* command) {
}

template<>
void MotionGeneratorLoop<JointVelocities>::convertMotion(const JointVelocities& motion, research_interface::MotionGeneratorCommand* command) {
}

template<>
void MotionGeneratorLoop<CartesianPose>::convertMotion(const CartesianPose& motion, research_interface::MotionGeneratorCommand* command) {
}

template<>
void MotionGeneratorLoop<CartesianVelocities>::convertMotion(const CartesianVelocities& motion, research_interface::MotionGeneratorCommand* command) {
}

}  // namespace franka
