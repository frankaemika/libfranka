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

GripperState Gripper::readOnce() {
  return impl_->readOnce();
}

}  // namespace franka