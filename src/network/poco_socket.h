#pragma  once

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/StreamSocket.h>
#include "socket.h"

namespace franka {

class PocoTcpSocket : public TcpSocket {
 public:
  ~PocoTcpSocket();

  void setReceiveTimeout(std::chrono::milliseconds timeout) override;
  void connect(const std::string &franka_address, uint16_t franka_port, std::chrono::milliseconds timeout) override;
  void setBlocking(bool flag) override;
  void setSendTimeout(std::chrono::milliseconds timeout) override;
  bool poll() override;
  int available() override;
  int receiveBytes(void *data, size_t size) override;
  int sendBytes(const void *data, size_t size) override;

 private:
  Poco::Net::StreamSocket tcp_socket_;
};

class PocoUdpSocket : public UdpSocket {
 public:
  void setReceiveTimeout(std::chrono::milliseconds timeout) override;
  void bind(const std::string &franka_address, uint16_t franka_port) override;
  int sendTo(const void *data, size_t size) override;
  int receiveFrom(void *data, size_t size) override;
  uint16_t port() override;
 private:
  Poco::Net::DatagramSocket udp_socket_;
  Poco::Net::SocketAddress udp_server_address_;
};

} // namespace franka