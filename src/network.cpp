#include "network.h"

#include <memory>
#include <sstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Network::Network(const std::string& franka_address,
                 uint16_t franka_port,
                 std::chrono::milliseconds timeout)
    : tcp_socket_(), udp_socket_() {
  try {
    Poco::Timespan poco_timeout(1000l * timeout.count());
    tcp_socket_.connect({franka_address, franka_port}, poco_timeout);
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(poco_timeout);
    tcp_socket_.setReceiveTimeout(poco_timeout);

    udp_socket_.setReceiveTimeout(Poco::Timespan{1000l * timeout.count()});
    udp_socket_.bind({"0.0.0.0", 0});
  } catch (const Poco::Net::NetException& e) {
    throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
  } catch (const Poco::TimeoutException& e) {
    throw NetworkException("libfranka: FRANKA connection timeout"s);
  } catch (const Poco::Exception& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

Network::~Network() {
  try {
    tcp_socket_.shutdown();
  } catch (...) {
  }
}

uint16_t Network::udpPort() const noexcept {
  return udp_socket_.address().port();
}

void Network::udpSendRobotCommand(const research_interface::robot::RobotCommand& command) try {
  int bytes_sent = udp_socket_.sendTo(&command, sizeof(command), udp_server_address_);
  if (bytes_sent != sizeof(command)) {
    throw NetworkException("libfranka: robot command send error");
  }
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: udp send: "s + e.what());
}

research_interface::robot::RobotState Network::udpReadRobotState() try {
  std::array<uint8_t, sizeof(research_interface::robot::RobotState)> buffer;
  int bytes_received = udp_socket_.receiveFrom(buffer.data(), buffer.size(), udp_server_address_);
  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka: incorrect object size");
  }
  return *reinterpret_cast<research_interface::robot::RobotState*>(buffer.data());
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: udp read: "s + e.what());
}

void Network::tcpReceiveIntoBuffer(uint8_t* buffer, size_t read_size) {
  size_t bytes_read = 0;
  try {
    while (bytes_read < read_size) {
      size_t bytes_left = read_size - bytes_read;
      int rv = tcp_socket_.receiveBytes(&buffer[bytes_read], static_cast<int>(bytes_left));
      if (rv < 0) {
        throw NetworkException("libfranka: server closed connection");
      }
      bytes_read += rv;
    }
  } catch (const Poco::TimeoutException& e) {
    if (bytes_read != 0) {
      throw ProtocolException(std::string{"libfranka: incorrect object size"});
    } else {
      throw NetworkException(std::string{"libfranka: FRANKA connection timeout"});
    }
  } catch (const Poco::Exception& e) {
    throw NetworkException(std::string{"libfranka: FRANKA connection closed"});
  }
}

bool Network::tcpReadResponse(research_interface::robot::Function* function) try {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    size_t offset = read_buffer_.size();
    int bytes_available = tcp_socket_.available();
    read_buffer_.resize(offset + bytes_available);
    tcpReceiveIntoBuffer(&read_buffer_[offset], bytes_available);

    if (read_buffer_.size() < sizeof(research_interface::robot::Function)) {
      return false;
    }

    *function = *reinterpret_cast<research_interface::robot::Function*>(read_buffer_.data());
    return true;
  }
  return false;
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

}  // namespace franka
