#include "network.h"

#include <memory>
#include <sstream>

// Included for MSG_PEEK symbol not present in Poco
#include <sys/socket.h>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Network::Network(const std::string& franka_address,
                 uint16_t franka_port,
                 std::chrono::milliseconds tcp_timeout,
                 std::chrono::milliseconds udp_timeout,
                 std::tuple<bool, int, int, int> tcp_keepalive) {
  try {
    Poco::Timespan poco_timeout(1000l * tcp_timeout.count());
    tcp_socket_.connect({franka_address, franka_port}, poco_timeout);
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(poco_timeout);
    tcp_socket_.setReceiveTimeout(poco_timeout);

    if (std::get<0>(tcp_keepalive)) {
      tcp_socket_.setKeepAlive(true);
      tcp_socket_.setOption(IPPROTO_TCP, TCP_KEEPIDLE, std::get<1>(tcp_keepalive));
      tcp_socket_.setOption(IPPROTO_TCP, TCP_KEEPCNT, std::get<2>(tcp_keepalive));
      tcp_socket_.setOption(IPPROTO_TCP, TCP_KEEPINTVL, std::get<3>(tcp_keepalive));
    }

    udp_socket_.setReceiveTimeout(Poco::Timespan{1000l * udp_timeout.count()});
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

void Network::tcpThrowIfConnectionClosed() try {
  std::lock_guard<std::recursive_mutex> _(tcp_mutex_);
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    std::array<uint8_t, 1> buffer;
    int rv = tcp_socket_.receiveBytes(buffer.data(), static_cast<int>(buffer.size()), MSG_PEEK);

    if (rv == 0) {
      throw NetworkException("libfranka: server closed connection");
    }
  }
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

void Network::tcpReceiveIntoBuffer(uint8_t* buffer, size_t read_size) {
  std::lock_guard<std::recursive_mutex> _(tcp_mutex_);
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
      throw ProtocolException("libfranka: incorrect object size");
    } else {
      throw NetworkException("libfranka: FRANKA connection timeout");
    }
  } catch (const Poco::Exception& e) {
    throw NetworkException("libfranka: FRANKA connection closed");
  }
}

int Network::udpAvailableData() {
  std::lock_guard<std::recursive_mutex> _(udp_mutex_);
  return udp_socket_.available();
}

}  // namespace franka
