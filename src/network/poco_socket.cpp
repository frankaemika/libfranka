#include "poco_socket.h"

#include <Poco/Net/NetException.h>
#include <chrono>

#include <franka/exception.h>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

bool PocoTcpSocket::poll() try {
  return tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ);
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

int PocoTcpSocket::available() try {
  return tcp_socket_.available();
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

std::pair<bool, int> PocoTcpSocket::receiveBytes(void* data, size_t size) try {
  int rv = tcp_socket_.receiveBytes(data, size);
  return {true, rv};
} catch (Poco::TimeoutException const& e) {
  return {false, 0};
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: tcp receive bytes: "s + e.what());
}

int PocoTcpSocket::sendBytes(const void* data, size_t size) try {
  return tcp_socket_.sendBytes(data, size);
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: tcp send bytes: "s + e.what());
}

PocoTcpSocket::~PocoTcpSocket() try { tcp_socket_.shutdown(); } catch (...) {
}

PocoTcpSocket::PocoTcpSocket(const std::string& franka_address,
                             uint16_t franka_port,
                             std::chrono::milliseconds timeout) try {
  Poco::Timespan poco_timeout(1000l * timeout.count());
  tcp_socket_.connect({franka_address, franka_port}, poco_timeout);
  tcp_socket_.setBlocking(true);
  tcp_socket_.setSendTimeout(poco_timeout);
  tcp_socket_.setReceiveTimeout(poco_timeout);
} catch (const Poco::Net::NetException& e) {
  throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
} catch (const Poco::TimeoutException& e) {
  throw NetworkException("libfranka: FRANKA connection timeout"s);
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

int PocoUdpSocket::sendTo(const void* data, size_t size) try {
  return udp_socket_.sendTo(data, size, udp_server_address_);
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: udp send: "s + e.what());
}

int PocoUdpSocket::receiveFrom(void* data, size_t size) try {
  return udp_socket_.receiveFrom(data, size, udp_server_address_);
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: udp send: "s + e.what());
}

uint16_t PocoUdpSocket::port() const noexcept {
  return udp_socket_.address().port();
}

PocoUdpSocket::PocoUdpSocket(std::chrono::milliseconds timeout) try {
  udp_socket_.setReceiveTimeout(Poco::Timespan{1000l * timeout.count()});
  udp_socket_.bind({"0.0.0.0", 0});
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: udp bind: "s + e.what());
}

}  // namespace franka