#include <franka/gripper.h>

#include <sstream>

#include <research_interface/gripper/types.h>

#include "network.h"

namespace franka {

namespace {

template <typename T, typename... TArgs>
bool executeCommand(Network& network, uint32_t& command_id_, TArgs&&... args) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  command_id_++;
  network.tcpSendRequest<T>(command_id_, std::forward<TArgs>(args)...);
  typename T::Response response = network.tcpBlockingReceiveResponse<T>(command_id_);

  switch (response.status) {
    case T::Status::kSuccess:
      return true;
    case T::Status::kFail:
      throw CommandException("libfranka gripper: Command failed!");
    case T::Status::kUnsuccessful:
      return false;
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
  if (!network_) {
    throw std::invalid_argument("libfranka gripper: Invalid argument");
  }

  connect<research_interface::gripper::Connect, research_interface::gripper::kVersion>(
      *network_, command_id_++, &ri_version_);
}

Gripper::~Gripper() noexcept = default;

Gripper::ServerVersion Gripper::serverVersion() const noexcept {
  return ri_version_;
}

void Gripper::homing() {
  if (!executeCommand<research_interface::gripper::Homing>(*network_, command_id_)) {
    throw CommandException("libfranka gripper: Homing command unsuccessful!");
  }
}

bool Gripper::grasp(double width, double speed, double force) {
  return executeCommand<research_interface::gripper::Grasp>(*network_, command_id_, width, speed,
                                                            force);
}

void Gripper::move(double width, double speed) {
  if (!executeCommand<research_interface::gripper::Move>(*network_, command_id_, width, speed)) {
    throw CommandException("libfranka gripper: Move command unsuccessful!");
  }
}

void Gripper::stop() {
  if (!executeCommand<research_interface::gripper::Stop>(*network_, command_id_)) {
    throw CommandException("libfranka gripper: Stop command unsuccessful!");
  }
}

GripperState Gripper::readOnce() const {
  // Delete old data from the UDP buffer.
  while (network_->udpAvailableData() > 0) {
    network_->udpRead<research_interface::gripper::GripperState>();
  }

  return convertGripperState(network_->udpRead<research_interface::gripper::GripperState>());
}

}  // namespace franka
