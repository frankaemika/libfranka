#include <franka/gripper.h>

#include "gripper_impl.h"

namespace franka {

Gripper::Gripper(const std::string& franka_address)
    : impl_(new Gripper::Impl(
          std::make_unique<GripperNetwork>(franka_address,
                                           research_interface::gripper::kCommandPort,
                                           Gripper::Impl::kDefaultTimeout))) {}

Gripper::~Gripper() noexcept = default;

Gripper::ServerVersion Gripper::serverVersion() const noexcept {
  return impl_->serverVersion();
}

void Gripper::homing() {
  impl_->executeCommand<research_interface::gripper::Homing>();
}

void Gripper::grasp(double width, double speed, double force) {
  impl_->executeCommand<research_interface::gripper::Grasp>(width, speed, force);
}

void Gripper::move(double width, double speed) {
  impl_->executeCommand<research_interface::gripper::Move>(width, speed);
}

void Gripper::stop() {
  impl_->executeCommand<research_interface::gripper::Stop>();
}

GripperState Gripper::readOnce() {
  return impl_->readOnce();
}

}  // namespace franka