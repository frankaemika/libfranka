#include <franka/motion_generator.h>

#include "robot_impl.h"

namespace franka {

MotionGenerator::MotionGenerator(MotionGenerator&& motion_generator) noexcept
    : robot{motion_generator.robot} {}

MotionGenerator::MotionGenerator(Robot& robot) noexcept : robot{robot} {}

MotionGenerator::~MotionGenerator() noexcept {
  robot.impl().stopMotionGenerator();
}

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator(research_interface::StartMotionGeneratorRequest::Type::kCartesianPosition);
}

CartesianPoseMotionGenerator::~CartesianPoseMotionGenerator() noexcept =
    default;

void CartesianPoseMotionGenerator::setDesiredPose(
    const std::array<double, 16>& desired_pose) noexcept {
  std::copy(desired_pose.cbegin(), desired_pose.cend(),
            robot.impl().motionCommand().O_T_EE_d.begin());
}

CartesianVelocityMotionGenerator::CartesianVelocityMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator(research_interface::StartMotionGeneratorRequest::Type::kCartesianVelocity);
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
  robot.impl().startMotionGenerator(research_interface::StartMotionGeneratorRequest::Type::kJointPosition);
}

JointPoseMotionGenerator::~JointPoseMotionGenerator() noexcept = default;

void JointPoseMotionGenerator::setDesiredPose(
    const std::array<double, 7>& desired_pose) noexcept {
  std::copy(desired_pose.cbegin(), desired_pose.cend(),
            robot.impl().motionCommand().q_d.begin());
}

JointVelocityMotionGenerator::JointVelocityMotionGenerator(Robot& robot)
    : MotionGenerator(robot) {
  robot.impl().startMotionGenerator(research_interface::StartMotionGeneratorRequest::Type::kJointVelocity);
}

JointVelocityMotionGenerator::~JointVelocityMotionGenerator() noexcept =
    default;

void JointVelocityMotionGenerator::setDesiredVelocity(
    const std::array<double, 7>& desired_velocity) noexcept {
  std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
            robot.impl().motionCommand().dq_d.begin());
}

}  // namespace franka
