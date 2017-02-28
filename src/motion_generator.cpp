#include <franka/motion_generator.h>

#include "motion_generator_impl.h"

namespace franka {

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(
    CartesianPoseMotionGenerator&& motion_generator) noexcept
    : impl_{std::move(motion_generator.impl_)} {}

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(
    CartesianPoseMotionGenerator::Impl&& impl)
    : impl_{new CartesianPoseMotionGenerator::Impl(std::move(impl))} {}

CartesianPoseMotionGenerator::~CartesianPoseMotionGenerator() noexcept =
    default;

void CartesianPoseMotionGenerator::setDesiredPose(
    const std::array<double, 16>& desired_pose) noexcept {
  std::copy(desired_pose.cbegin(), desired_pose.cend(),
            impl_->command().O_T_EE_d.begin());
}

CartesianVelocityMotionGenerator::CartesianVelocityMotionGenerator(
    CartesianVelocityMotionGenerator&& motion_generator) noexcept
    : impl_{std::move(motion_generator.impl_)} {}

CartesianVelocityMotionGenerator::CartesianVelocityMotionGenerator(
    CartesianVelocityMotionGenerator::Impl&& impl)
    : impl_{new CartesianVelocityMotionGenerator::Impl(std::move(impl))} {}

CartesianVelocityMotionGenerator::~CartesianVelocityMotionGenerator() noexcept =
    default;

void CartesianVelocityMotionGenerator::setDesiredVelocity(
    const std::array<double, 6>& desired_velocity) noexcept {
  std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
            impl_->command().O_dP_EE_d.begin());
}

JointPoseMotionGenerator::JointPoseMotionGenerator(
    JointPoseMotionGenerator&& motion_generator) noexcept
    : impl_{std::move(motion_generator.impl_)} {}

JointPoseMotionGenerator::JointPoseMotionGenerator(
    JointPoseMotionGenerator::Impl&& impl)
    : impl_{new JointPoseMotionGenerator::Impl(std::move(impl))} {}

JointPoseMotionGenerator::~JointPoseMotionGenerator() noexcept = default;

void JointPoseMotionGenerator::setDesiredPose(
    const std::array<double, 7>& desired_pose) noexcept {
  std::copy(desired_pose.cbegin(), desired_pose.cend(),
            impl_->command().q_d.begin());
}

JointVelocityMotionGenerator::JointVelocityMotionGenerator(
    JointVelocityMotionGenerator&& motion_generator) noexcept
    : impl_{std::move(motion_generator.impl_)} {}

JointVelocityMotionGenerator::JointVelocityMotionGenerator(
    JointVelocityMotionGenerator::Impl&& impl)
    : impl_{new JointVelocityMotionGenerator::Impl(std::move(impl))} {}

JointVelocityMotionGenerator::~JointVelocityMotionGenerator() noexcept =
    default;

void JointVelocityMotionGenerator::setDesiredVelocity(
    const std::array<double, 7>& desired_velocity) noexcept {
  std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
            impl_->command().dq_d.begin());
}

}  // namespace franka
