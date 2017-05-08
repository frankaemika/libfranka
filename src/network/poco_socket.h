#pragma once

#include <chrono>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/StreamSocket.h>

namespace franka {

class PocoTcpSocket {
 public:
  PocoTcpSocket(const std::string& franka_address,
                uint16_t franka_port,
                std::chrono::milliseconds timeout);
  ~PocoTcpSocket();

  bool poll();
  int available();
  std::pair<bool, int> receiveBytes(void* data, size_t size);
  int sendBytes(const void* data, size_t size);

 private:
  Poco::Net::StreamSocket tcp_socket_;
};

class PocoUdpSocket {
 public:
  PocoUdpSocket(std::chrono::milliseconds timeout);

  int sendTo(const void* data, size_t size);
  int receiveFrom(void* data, size_t size);
  uint16_t port() const noexcept;

 private:
  Poco::Net::DatagramSocket udp_socket_;
  Poco::Net::SocketAddress udp_server_address_;
};

}  // namespace franka