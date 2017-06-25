#include "gripper_network.h"

#include <sstream>

#include <franka/exception.h>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

GripperNetwork::GripperNetwork(const std::string& franka_address,
                               uint16_t franka_port,
                               std::chrono::milliseconds timeout)
    : Network(franka_address, franka_port, timeout) {}

research_interface::gripper::GripperState GripperNetwork::udpReadGripperState() try {
  std::array<uint8_t, sizeof(research_interface::gripper::GripperState)> buffer;
  int bytes_received =
      udp_socket_.receiveFrom(buffer.data(), static_cast<int>(buffer.size()), udp_server_address_);
  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka gripper: incorrect object size");
  }
  return *reinterpret_cast<research_interface::gripper::GripperState*>(buffer.data());
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka gripper: udp read: "s + e.what());
}

bool GripperNetwork::tcpReadResponse(research_interface::gripper::Function* function) try {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    int rv = tcpReceiveIntoBuffer();

    if (rv == 0) {
      throw NetworkException("libfranka gripper: server closed connection");
    }

    if (read_buffer_.size() < sizeof(research_interface::gripper::Function)) {
      return false;
    }

    *function = *reinterpret_cast<research_interface::gripper::Function*>(read_buffer_.data());
    return true;
  }
  return false;
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka gripper: "s + e.what());
}

}  // namespace franka