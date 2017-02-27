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

// bool CartesianPoseMotionGenerator::checkPose(const std::array<double, 7>&
// pose) {
//   // TODO
//   return true;
// }
//
// void CartesianVelocityMotionGenerator::setDesiredVelocity(std::array<double,
// 6>& desired_velocity) {
//   // TODO
// }

// void JointPoseMotionGenerator::setDesiredPose(std::array<double, 7>&
// desired_pose) {
//   if(checkPose(desired_pose)) {
//     // set
//   } else {
//     // errormessage
//   }
//   // TODO
// }
//
// bool JointPoseMotionGenerator::checkPose(std::array<double, 7>& desired_pose)
// {
//     // TODO, check unit quaternion
//     return true;
// }
//
// void JointVelocityMotionGenerator::setDesiredVelocity(std::array<double, 7>&
// desired_velocity) {
//   // TODO
// }

}  // namespace franka
