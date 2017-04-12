#include "network.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Network::Network(const std::string& franka_address,
                 uint16_t franka_port,
                 std::chrono::milliseconds timeout) {
  Poco::Timespan poco_timeout(1000l * timeout.count());
  try {
    tcp_socket_.connect({franka_address, franka_port}, poco_timeout);
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(poco_timeout);
    tcp_socket_.setReceiveTimeout(poco_timeout);

    udp_socket_.setReceiveTimeout(poco_timeout);
    udp_socket_.bind({"0.0.0.0", 0});
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
  } catch (Poco::TimeoutException const& e) {
    throw NetworkException("libfranka: FRANKA connection timeout");
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

Network::~Network() noexcept {
  try {
    tcp_socket_.shutdown();
  } catch (...) {
  }
}

int Network::udpSendRobotCommand(
    const research_interface::RobotCommand& command) {
  try {
    return udp_socket_.sendTo(&command, sizeof(command), udp_server_address_);
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: robot command send: "s + e.what());
  }
}

research_interface::RobotState Network::udpReadRobotState() {
  try {
    std::array<uint8_t, sizeof(research_interface::RobotState)> buffer;
    int bytes_received = udp_socket_.receiveFrom(buffer.data(), buffer.size(),
                                                 udp_server_address_);
    if (bytes_received != buffer.size()) {
      throw ProtocolException("libfranka: incorrect object size");
    }
    return *reinterpret_cast<research_interface::RobotState*>(buffer.data());
  } catch (const Poco::TimeoutException& e) {
    throw NetworkException("libfranka: robot state read timeout");
  } catch (const Poco::Net::NetException& e) {
    throw NetworkException("libfranka: robot state read: "s + e.what());
  }
}

uint16_t Network::udpPort() const {
  return udp_socket_.address().port();
}

size_t Network::bufferSize() const noexcept {
  return read_buffer_.size();
}

void* Network::bufferData() noexcept {
  return read_buffer_.data();
}

bool Network::tcpPollRead() {
  try {
    return tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

int Network::tcpReceiveIntoBuffer() {
  try {
    size_t offset = read_buffer_.size();
    read_buffer_.resize(offset + tcp_socket_.available());
    return tcp_socket_.receiveBytes(&read_buffer_[offset],
                                    tcp_socket_.available());
  } catch (const Poco::Net::NetException& e) {
    throw NetworkException("libfranka: control connection read: "s + e.what());
  }
}

}  // namespace franka