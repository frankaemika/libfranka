#include "robot_network.h"

#include <sstream>

#include <franka/exception.h>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

RobotNetwork::RobotNetwork(const std::string& franka_address,
                           uint16_t franka_port,
                           std::chrono::milliseconds timeout)
    : Network(franka_address, franka_port, timeout) {}

research_interface::robot::RobotState RobotNetwork::udpReadRobotState() try {
  std::array<uint8_t, sizeof(research_interface::robot::RobotState)> buffer;
  int bytes_received = udp_socket_.receiveFrom(buffer.data(), buffer.size(), udp_server_address_);
  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka robot: incorrect object size");
  }
  return *reinterpret_cast<research_interface::robot::RobotState*>(buffer.data());
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka robot: udp read: "s + e.what());
}

void RobotNetwork::udpSendRobotCommand(const research_interface::robot::RobotCommand& command) try {
  int bytes_sent = udp_socket_.sendTo(&command, sizeof(command), udp_server_address_);
  if (bytes_sent != sizeof(command)) {
    throw NetworkException("libfranka robot: robot command send error");
  }
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka robot: udp send: "s + e.what());
}

bool RobotNetwork::tcpReadResponse(research_interface::robot::Function* function) try {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    int rv = tcpReceiveIntoBuffer();

    if (rv == 0) {
      throw NetworkException("libfranka robot: server closed connection");
    }

    if (read_buffer_.size() < sizeof(research_interface::robot::Function)) {
      return false;
    }

    *function = *reinterpret_cast<research_interface::robot::Function*>(read_buffer_.data());
    return true;
  }
  return false;
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka robot: "s + e.what());
}

}  // namespace franka
