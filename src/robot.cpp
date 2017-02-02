#include <franka/robot.h>

#include "robot_impl.h"

namespace franka {

Robot::Robot(const std::string& franka_address)
    : impl_(new Robot::Impl(franka_address)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() = default;

bool Robot::waitForRobotState() {
  return impl_->waitForRobotState();
}

const RobotState& Robot::robotState() const noexcept {
  return impl_->robotState();
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

}  // namespace franka
