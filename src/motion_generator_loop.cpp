#include "motion_generator_loop.h"

namespace franka {

template <typename T>
MotionGeneratorLoop<T>::MotionGeneratorLoop(Robot::Impl& robot_impl,
    ControlCallback control_callback,
    MotionGeneratorCallback motion_callback)
  : ControlLoop(robot_impl, control_callback),
  motion_callback_(std::move(motion_callback))
{
  if (motion_callback_) {
    robot_impl_.startMotionGenerator(MotionTraits<T>::Type);
  }
}

template <typename T>
MotionGeneratorLoop<T>::~MotionGeneratorLoop() {
  if (motion_callback_) {
    robot_impl_.stopMotionGenerator();
  }
}

template <typename T>
bool MotionGeneratorLoop<T>::spinOnce() {
  if (motion_callback_) {
    T motion_output = motion_callback_(robot_impl_.robotState());
    if (typeid(motion_output) == typeid(Stop)) {
      return false;
    }
    convertMotion(motion_output, &robot_impl_.motionCommand());
  }

  return ControlLoop::spinOnce();
}

template<>
void MotionGeneratorLoop<JointValues>::convertMotion(const JointValues& motion, research_interface::MotionGeneratorCommand* command) {
  //TODO
}

template<>
void MotionGeneratorLoop<JointVelocities>::convertMotion(const JointVelocities& motion, research_interface::MotionGeneratorCommand* command) {
  //TODO
}

template<>
void MotionGeneratorLoop<CartesianPose>::convertMotion(const CartesianPose& motion, research_interface::MotionGeneratorCommand* command) {
  //TODO
}

template<>
void MotionGeneratorLoop<CartesianVelocities>::convertMotion(const CartesianVelocities& motion, research_interface::MotionGeneratorCommand* command) {
  //TODO
}

}  // namespace franka
