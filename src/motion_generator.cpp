#include <franka/motion_generator.h>

#include <cmath>

#include "robot_impl.h"

#define ORTHONORMAL_THRESHOLD 1e-6

namespace franka {

MotionGenerator::MotionGenerator(MotionGenerator&& motion_generator) noexcept
    : robot{motion_generator.robot} {}

MotionGenerator::MotionGenerator(Robot& robot) noexcept : robot{robot} {}

MotionGenerator::~MotionGenerator() noexcept {
  robot.impl().stopMotionGenerator();
}

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator();
}

CartesianPoseMotionGenerator::~CartesianPoseMotionGenerator() noexcept =
    default;

bool CartesianPoseMotionGenerator::checkHomogeneousTransformation(
    std::array<double, 16> transform) {
  if (transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 ||
      transform[15] != 1.0) {
    return false;
  }
  for (size_t j = 0; j < 3; ++j) {  // i..column
    if (fabs(sqrt(pow(transform[j * 4 + 0], 2) + pow(transform[j * 4 + 1], 2) +
                  pow(transform[j * 4 + 2], 2)) -
             1.0) > ORTHONORMAL_THRESHOLD) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {  // j..row
    if (fabs(sqrt(pow(transform[0 * 4 + i], 2) + pow(transform[1 * 4 + i], 2) +
                  pow(transform[2 * 4 + i], 2)) -
             1.0) > ORTHONORMAL_THRESHOLD) {
      return false;
    }
  }
  return true;
}

void CartesianPoseMotionGenerator::setDesiredPose(
    const std::array<double, 16>& desired_pose) noexcept {
    if (checkHomogeneousTransformation(desired_pose)) {
        std::copy(desired_pose.cbegin(), desired_pose.cend(),
                  robot.impl().motionCommand().O_T_EE_d.begin());
    } else {
      throw MotionGeneratorException(
          "libfranka:: Attempt to set invalid transformation in motion"
          "generator.\nHas to be column major!");
    }
}

CartesianVelocityMotionGenerator::CartesianVelocityMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator();
}

CartesianVelocityMotionGenerator::~CartesianVelocityMotionGenerator() noexcept =
    default;

void CartesianVelocityMotionGenerator::setDesiredVelocity(
    const std::array<double, 6>& desired_velocity) noexcept {
    std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
              robot.impl().motionCommand().O_dP_EE_d.begin());
}

JointPoseMotionGenerator::JointPoseMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator();
}

JointPoseMotionGenerator::~JointPoseMotionGenerator() noexcept = default;

void JointPoseMotionGenerator::setDesiredPose(
    const std::array<double, 7>& desired_pose) noexcept {
  std::copy(desired_pose.cbegin(), desired_pose.cend(),
            robot.impl().motionCommand().q_d.begin());
}

JointVelocityMotionGenerator::JointVelocityMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator();
}

JointVelocityMotionGenerator::~JointVelocityMotionGenerator() noexcept =
    default;

void JointVelocityMotionGenerator::setDesiredVelocity(
    const std::array<double, 7>& desired_velocity) noexcept {
    std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
              robot.impl().motionCommand().dq_d.begin());
}

}  // namespace franka
