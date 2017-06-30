#include <franka/gripper.h>

#include <sstream>

#include <research_interface/gripper/types.h>

#include "network.h"

namespace franka {

GripperState convertGripperState(
    const research_interface::gripper::GripperState& gripper_state) noexcept;

template <typename T>
bool handleCommandResponse(const typename T::Response& response) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  switch (response.status) {
    case T::Status::kSuccess:
      return true;
    case T::Status::kFail:
      throw CommandException("libfranka gripper: command failed!");
    case T::Status::kUnsuccessful:
      return false;
    default:
      throw ProtocolException("libfranka gripper: Unexpected response while handling command!");
  }
}

Gripper::Gripper(const std::string& franka_address)
    : network_{
          std::make_unique<Network>(franka_address, research_interface::gripper::kCommandPort)} {
  if (!network_) {
    throw std::invalid_argument("libfranka gripper: Invalid argument");
  }

  connect<research_interface::gripper::Connect, research_interface::gripper::kVersion>(
      *network_, &ri_version_);
}

Gripper::~Gripper() noexcept = default;

Gripper::ServerVersion Gripper::serverVersion() const noexcept {
  return ri_version_;
}

void Gripper::homing() {
  using research_interface::gripper::Homing;
  network_->tcpSendRequest<Homing>({});
  Homing::Response response = network_->tcpBlockingReceiveResponse<Homing>();

  if (!handleCommandResponse<Homing>(response)) {
    throw CommandException("libfranka gripper: command unsuccessful!");
  }
}

bool Gripper::grasp(double width, double speed, double force) {
  using research_interface::gripper::Grasp;
  network_->tcpSendRequest<Grasp>({width, speed, force});
  Grasp::Response response = network_->tcpBlockingReceiveResponse<Grasp>();

  return handleCommandResponse<Grasp>(response);
}

void Gripper::move(double width, double speed) {
  using research_interface::gripper::Move;
  network_->tcpSendRequest<Move>({width, speed});
  Move::Response response = network_->tcpBlockingReceiveResponse<Move>();

  if (!handleCommandResponse<Move>(response)) {
    throw CommandException("libfranka gripper: command unsuccessful!");
  }
}

void Gripper::stop() {
  using research_interface::gripper::Stop;
  network_->tcpSendRequest<Stop>({});
  Stop::Response response = network_->tcpBlockingReceiveResponse<Stop>();

  if (!handleCommandResponse<Stop>(response)) {
    throw CommandException("libfranka gripper: command unsuccessful!");
  }
}

GripperState Gripper::readOnce() {
  // Delete old gripper states in the UDP buffer.
  if (network_->udpAvailableData() >
      static_cast<int>(sizeof(research_interface::gripper::GripperState))) {
    network_->udpRead<research_interface::gripper::GripperState>();
  }
  return convertGripperState(network_->udpRead<research_interface::gripper::GripperState>());
}

GripperState convertGripperState(
    const research_interface::gripper::GripperState& gripper_state) noexcept {
  GripperState converted;
  converted.opening_width = gripper_state.width;
  converted.max_opening_width = gripper_state.max_width;
  converted.object_grasped = gripper_state.is_grasped;
  converted.temperature = gripper_state.temperature;
  converted.sequence_number = gripper_state.message_id;
  return converted;
}

}  // namespace franka