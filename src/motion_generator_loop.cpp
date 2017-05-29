#include "motion_generator_loop.h"
#include "motion_generator_traits.h"

namespace franka {

template <typename T>
MotionGeneratorLoop<T>::MotionGeneratorLoop(RobotControl& robot,
                                            ControlCallback control_callback,
                                            MotionGeneratorCallback motion_callback)
    : ControlLoop(robot, control_callback), motion_callback_(std::move(motion_callback)) {
  if (motion_callback_) {
    robot.startMotionGenerator(MotionGeneratorTraits<T>::kMotionGeneratorMode);
  }
}

template <typename T>
MotionGeneratorLoop<T>::~MotionGeneratorLoop() {
  if (motion_callback_) {
    robot_.stopMotionGenerator();
  }
}

template <typename T>
bool MotionGeneratorLoop<T>::spinOnce() {
  if (motion_callback_) {
    T motion_output = motion_callback_(robot_.robotState());
    if (motion_output.stop()) {
      return false;
    }
    research_interface::MotionGeneratorCommand command{};
    convertMotion(motion_output, &command);
    robot_.motionGeneratorCommand(command);
  }

  return ControlLoop::spinOnce();
}

template <>
void MotionGeneratorLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->q_d = motion.q;
}

template <>
void MotionGeneratorLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->dq_d = motion.dq;
}

template <>
void MotionGeneratorLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->O_T_EE_d = motion.O_T_EE;
}

template <>
void MotionGeneratorLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->O_dP_EE_d = motion.O_dP_EE;
}

}  // namespace franka
