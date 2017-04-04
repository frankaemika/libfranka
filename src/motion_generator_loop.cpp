#include "motion_generator_loop.h"

#include <cmath>

namespace franka {

template <typename T>
MotionGeneratorLoop<T>::MotionGeneratorLoop(
    Robot::Impl& robot_impl,
    ControlCallback control_callback,
    MotionGeneratorCallback motion_callback)
    : ControlLoop(robot_impl, control_callback),
      motion_callback_(std::move(motion_callback)) {
  if (motion_callback_) {
    robot_impl_.startMotionGenerator(MotionTraits<T>::kMotionGeneratorType);
  }
}

template <typename T>
MotionGeneratorLoop<T>::~MotionGeneratorLoop() {
  if (motion_callback_) {
    robot_impl_.stopMotionGenerator();
  }
}

template <>
bool MotionGeneratorLoop<CartesianPose>::checkHomogeneousTransformation(
    std::array<double, 16> transform) noexcept {
  constexpr double kOrthonormalThreshold = 1e-6;

  if (transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 ||
      transform[15] != 1.0) {
    return false;
  }
  for (size_t j = 0; j < 3; ++j) {  // i..column
    if (std::abs(std::sqrt(std::pow(transform[j * 4 + 0], 2) +
                           std::pow(transform[j * 4 + 1], 2) +
                           std::pow(transform[j * 4 + 2], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {  // j..row
    if (std::abs(std::sqrt(std::pow(transform[0 * 4 + i], 2) +
                           std::pow(transform[1 * 4 + i], 2) +
                           std::pow(transform[2 * 4 + i], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool MotionGeneratorLoop<T>::spinOnce() {
  if (motion_callback_) {
    T motion_output = motion_callback_(robot_impl_.robotState());
    if (&motion_output == &Stop) {
      return false;
    }
    convertMotion(motion_output, &robot_impl_.motionCommand());
  }

  return ControlLoop::spinOnce();
}

template <>
void MotionGeneratorLoop<JointValues>::convertMotion(
    const JointValues& motion,
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
  if (!checkHomogeneousTransformation(motion.O_T_EE)) {
    throw ControlException(
        "libfranka: Attempt to set invalid transformation in motion"
        "generator. Has to be column major!");
  }
  command->O_T_EE_d = motion.O_T_EE;
}

template <>
void MotionGeneratorLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->O_dP_EE_d = motion.O_dP_EE;
}

}  // namespace franka
