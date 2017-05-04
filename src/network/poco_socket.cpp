#include "poco_socket.h"

#include <Poco/Net/NetException.h>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

void PocoTcpSocket::setReceiveTimeout(std::chrono::milliseconds timeout) {
  tcp_socket_.setReceiveTimeout(Poco::Timespan{1000l * timeout.count()});
}

void PocoTcpSocket::connect(const std::string& franka_address,
                            uint16_t franka_port,
                            std::chrono::milliseconds timeout) {
  try {
    tcp_socket_.connect({franka_address, franka_port},
                        Poco::Timespan{1000l * timeout.count()});
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
  } catch (Poco::TimeoutException const& e) {
    throw TimeoutException("libfranka: FRANKA connection timeout"s);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

void PocoTcpSocket::setBlocking(bool flag) {
  try {
    tcp_socket_.setBlocking(flag);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

void PocoTcpSocket::setSendTimeout(std::chrono::milliseconds timeout) {
  try {
    tcp_socket_.setSendTimeout(Poco::Timespan{1000l * timeout.count()});
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

bool PocoTcpSocket::poll() {
  try {
    return tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

int PocoTcpSocket::available() {
  try {
    return tcp_socket_.available();
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

int PocoTcpSocket::receiveBytes(void* data, size_t size) {
  try {
    return tcp_socket_.receiveBytes(data, size);
  } catch (Poco::TimeoutException const& e) {
    throw TimeoutException("libfranka: FRANKA connection timeout"s);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: tcp receive bytes: "s + e.what());
  }
}

int PocoTcpSocket::sendBytes(const void* data, size_t size) {
  try {
    return tcp_socket_.sendBytes(data, size);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: tcp send bytes: "s + e.what());
  }
}
PocoTcpSocket::~PocoTcpSocket() {
  try {
    tcp_socket_.shutdown();
  } catch (...) {
  }
}

void PocoUdpSocket::setReceiveTimeout(std::chrono::milliseconds timeout) {
  try {
    return udp_socket_.setReceiveTimeout(
        Poco::Timespan{1000l * timeout.count()});
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

void PocoUdpSocket::bind(const std::string& franka_address,
                         uint16_t franka_port) {
  try {
    return udp_socket_.bind({franka_address, franka_port});
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: tcp receive bytes: "s + e.what());
  }
}

int PocoUdpSocket::sendTo(const void* data, size_t size) {
  try {
    return udp_socket_.sendTo(data, size, udp_server_address_);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: udp send: "s + e.what());
  }
}

int PocoUdpSocket::receiveFrom(void* data, size_t size) {
  try {
    return udp_socket_.receiveFrom(data, size, udp_server_address_);
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: udp send: "s + e.what());
  }
}

uint16_t PocoUdpSocket::port() {
  try {
    return udp_socket_.address().port();
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: udp send: "s + e.what());
  }
}

}  // namespace franka