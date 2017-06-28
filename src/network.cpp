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

int Network::tcpReceiveIntoBuffer() try {
  size_t offset = read_buffer_.size();
  read_buffer_.resize(offset + tcp_socket_.available());
  return tcp_socket_.receiveBytes(&read_buffer_[offset], tcp_socket_.available());
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

int Network::udpAvailableData() {
  return udp_socket_.available();
}

}  // namespace franka
