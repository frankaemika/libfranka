#include <franka/robot.h>

#include "robot_impl.h"

namespace franka {

Robot::Robot(const std::string& franka_address)
    : impl_(new Robot::Impl(franka_address)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() noexcept = default;

bool Robot::update() {
  return impl_->update();
}

const RobotState& Robot::robotState() const noexcept {
  return impl_->robotState();
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

const CartesianPoseMotionGenerator& Robot::startCartesianPoseMotionGenerator() {
  return impl_->startCartesianPoseMotionGenerator();
}

const CartesianVelocityMotionGenerator& Robot::startCartesianVelocityMotionGenerator() {
  return impl_->startCartesianVelocityMotionGenerator();
}

const JointPoseMotionGenerator& Robot::startJointPoseMotionGenerator() {
  return impl_->startJointPoseMotionGenerator();
}

const JointVelocityMotionGenerator& Robot::startJointVelocityMotionGenerator() {
  return impl_->startJointVelocityMotionGenerator();
}

}  // namespace franka
