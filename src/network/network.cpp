#include "network.h"
#include "poco_socket.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Network::Network(const std::string& franka_address,
                 uint16_t franka_port,
                 std::chrono::milliseconds timeout,
                 std::unique_ptr<TcpSocket> tcp_socket,
                 std::unique_ptr<UdpSocket> udp_socket)
    : tcp_socket_(std::move(tcp_socket)), udp_socket_(std::move(udp_socket)) {
  tcp_socket_->connect(franka_address, franka_port, timeout);
  tcp_socket_->setBlocking(true);
  tcp_socket_->setSendTimeout(timeout);
  tcp_socket_->setReceiveTimeout(timeout);

  udp_socket_->setReceiveTimeout(timeout);
  udp_socket_->bind("0.0.0.0", 0);
}

int Network::udpSendRobotCommand(
    const research_interface::RobotCommand& command) {
  return udp_socket_->sendTo(&command, sizeof(command));
}

research_interface::RobotState Network::udpReadRobotState() {
  std::array<uint8_t, sizeof(research_interface::RobotState)> buffer;
  int bytes_received = udp_socket_->receiveFrom(buffer.data(), buffer.size());
  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka: incorrect object size");
  }
  return *reinterpret_cast<research_interface::RobotState*>(buffer.data());
}

uint16_t Network::udpPort() const {
  return udp_socket_->port();
}

size_t Network::bufferSize() const noexcept {
  return read_buffer_.size();
}

void* Network::bufferData() noexcept {
  return read_buffer_.data();
}

bool Network::tcpPollRead() {
  return tcp_socket_->poll();
}

int Network::tcpReceiveIntoBuffer() try {
  size_t offset = read_buffer_.size();
  read_buffer_.resize(offset + tcp_socket_->available());
  return tcp_socket_->receiveBytes(&read_buffer_[offset],
                                   tcp_socket_->available());
} catch (TimeoutException const& e) {
  throw NetworkException("libfranka: FRANKA connection timeout");
}

}  // namespace franka
