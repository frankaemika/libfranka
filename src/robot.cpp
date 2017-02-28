#include <franka/robot.h>

#include "robot_impl.h"

namespace franka {

Robot::Robot(const std::string& franka_address)
    : impl_(new Robot::Impl(franka_address)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() noexcept = default;

Robot::Impl& Robot::impl() noexcept {
  return *impl_;
}

bool Robot::update() {
  return impl_->update();
}

const RobotState& Robot::robotState() const noexcept {
  return impl_->robotState();
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

CartesianPoseMotionGenerator Robot::startCartesianPoseMotionGenerator() {
  return CartesianPoseMotionGenerator(*this);
}

CartesianVelocityMotionGenerator
Robot::startCartesianVelocityMotionGenerator() {
  return CartesianVelocityMotionGenerator(*this);
}

JointPoseMotionGenerator Robot::startJointPoseMotionGenerator() {
  return JointPoseMotionGenerator(*this);
}

JointVelocityMotionGenerator Robot::startJointVelocityMotionGenerator() {
  return JointVelocityMotionGenerator(*this);
}

}  // namespace franka
