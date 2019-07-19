// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/gripper.h>

#include <sstream>

#include <franka/exception.h>
#include <research_interface/gripper/types.h>

#include "network.h"

namespace franka {

namespace {

template <typename T, typename... TArgs>
bool executeCommand(Network& network, TArgs&&... args) {
  uint32_t command_id = network.tcpSendRequest<T>(std::forward<TArgs>(args)...);
  typename T::Response response = network.tcpBlockingReceiveResponse<T>(command_id);

  switch (response.status) {
    case T::Status::kSuccess:
      return true;
    case T::Status::kFail:
      throw CommandException("libfranka gripper: Command failed!");
    case T::Status::kUnsuccessful:
      return false;
    case T::Status::kAborted:
      throw CommandException("libfranka gripper: Command aborted!");
    default:
      throw ProtocolException("libfranka gripper: Unexpected response while handling command!");
  }
}

GripperState convertGripperState(
    const research_interface::gripper::GripperState& gripper_state) noexcept {
  GripperState converted;
  converted.width = gripper_state.width;
  converted.max_width = gripper_state.max_width;
  converted.is_grasped = gripper_state.is_grasped;
  converted.temperature = gripper_state.temperature;
  converted.time = Duration(gripper_state.message_id);
  return converted;
}

}  // anonymous namespace

Gripper::Gripper(const std::string& franka_address)
    : network_{
          std::make_unique<Network>(franka_address, research_interface::gripper::kCommandPort)} {
  connect<research_interface::gripper::Connect, research_interface::gripper::kVersion>(
      *network_, &ri_version_);
}

Gripper::~Gripper() noexcept = default;
Gripper::Gripper(Gripper&&) noexcept = default;
Gripper& Gripper::operator=(Gripper&&) noexcept = default;

Gripper::ServerVersion Gripper::serverVersion() const noexcept {
  return ri_version_;
}

bool Gripper::homing() const {
  return executeCommand<research_interface::gripper::Homing>(*network_);
}

bool Gripper::grasp(double width,
                    double speed,
                    double force,
                    double epsilon_inner,
                    double epsilon_outer) const {
  research_interface::gripper::Grasp::GraspEpsilon epsilon(epsilon_inner, epsilon_outer);
  return executeCommand<research_interface::gripper::Grasp>(*network_, width, epsilon, speed,
                                                            force);
}

bool Gripper::move(double width, double speed) const {
  return executeCommand<research_interface::gripper::Move>(*network_, width, speed);
}

bool Gripper::stop() const {
  return executeCommand<research_interface::gripper::Stop>(*network_);
}

GripperState Gripper::readOnce() const {
  research_interface::gripper::GripperState gripper_state;
  // Delete old data from the UDP buffer.
  while (network_->udpReceive<decltype(gripper_state)>(&gripper_state)) {
  }

  gripper_state = network_->udpBlockingReceive<decltype(gripper_state)>();
  return convertGripperState(gripper_state);
}

}  // namespace franka
