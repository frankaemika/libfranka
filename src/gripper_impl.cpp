#include "gripper_impl.h"

#include <sstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Gripper::Impl::Impl(std::unique_ptr<Network> network) : network_(std::move(network)) {
  research_interface::gripper::Connect::Request connect_request(network_->udpPort());

  network_->tcpSendRequest<research_interface::gripper::Connect>(connect_request);

  research_interface::gripper::Connect::Response connect_response =
      network_->tcpBlockingReceiveResponse<research_interface::gripper::Connect>();
  switch (connect_response.status) {
    case research_interface::gripper::Connect::Status::kIncompatibleLibraryVersion: {
      std::stringstream message;
      message << "libfranka gripper: incompatible library version. " << std::endl
              << "Server version: " << connect_response.version << std::endl
              << "Library version: " << research_interface::gripper::kVersion;
      throw IncompatibleVersionException(message.str());
    }
    case research_interface::gripper::Connect::Status::kSuccess: {
      ri_version_ = connect_response.version;
      break;
    }
    default:
      throw ProtocolException(
          "libfranka gripper: protocol error during gripper connection attempt");
  }
}

GripperState Gripper::Impl::readOnce() {
  return convertGripperState(receiveGripperState());
}

Gripper::ServerVersion Gripper::Impl::serverVersion() const noexcept {
  return ri_version_;
}

research_interface::gripper::GripperState Gripper::Impl::receiveGripperState() {
  return network_->udpRead<research_interface::gripper::GripperState>();
}

GripperState convertGripperState(
    const research_interface::gripper::GripperState& gripper_state) noexcept {
  GripperState converted;
  converted.opening_width = gripper_state.width;
  converted.max_opening_width = gripper_state.max_width;
  converted.object_grasped = gripper_state.is_grasped;
  converted.temperature = gripper_state.temperature;
  return converted;
}

}  //  namespace franka
