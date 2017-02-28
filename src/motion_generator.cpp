#include <franka/motion_generator.h>

#include <cmath>

#include "motion_generator_impl.h"

#define ORTHONORMAL_THRESHOLD 1e-6

namespace franka {

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(
    CartesianPoseMotionGenerator&& motion_generator) noexcept
    : impl_{std::move(motion_generator.impl_)} {}

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(
    CartesianPoseMotionGenerator::Impl&& impl)
    : impl_{new CartesianPoseMotionGenerator::Impl(std::move(impl))} {}

CartesianPoseMotionGenerator::~CartesianPoseMotionGenerator() noexcept =
    default;

bool CartesianPoseMotionGenerator::Impl::checkHomogeneousTransformation(
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
  impl_->setDesiredPose(desired_pose);
}

void CartesianPoseMotionGenerator::Impl::setDesiredPose(
    const std::array<double, 16>& desired_pose) noexcept {
  if (checkHomogeneousTransformation(desired_pose)) {
    std::copy(desired_pose.cbegin(), desired_pose.cend(),
              command().O_T_EE_d.begin());
  } else {
    throw MotionGeneratorException(
        "libfranka:: Attempt to set invalid transformation in motion"
        "generator.\nHas to be column major!");
  }
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
  impl_->setDesiredVelocity(desired_velocity);
}

void CartesianVelocityMotionGenerator::Impl::setDesiredVelocity(
    const std::array<double, 6>& desired_velocity) noexcept {
  std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
            command().O_dP_EE_d.begin());
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
  impl_->setDesiredPose(desired_pose);
}

void JointPoseMotionGenerator::Impl::setDesiredPose(
    const std::array<double, 7>& desired_pose) noexcept {
  std::copy(desired_pose.cbegin(), desired_pose.cend(), command().q_d.begin());
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
  impl_->setDesiredVelocity(desired_velocity);
}

void JointVelocityMotionGenerator::Impl::setDesiredVelocity(
    const std::array<double, 7>& desired_velocity) noexcept {
  std::copy(desired_velocity.cbegin(), desired_velocity.cend(),
            command().dq_d.begin());
}

}  // namespace franka
