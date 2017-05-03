#pragma once

#include <cassert>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

#include <franka/exception.h>
#include <research_interface/rbk_types.h>
#include <research_interface/types.h>

#include "socket.h"

namespace franka {

class Network {
 public:
  explicit Network(const std::string& franka_address,
                   uint16_t franka_port,
                   std::chrono::milliseconds timeout,
                   std::unique_ptr<TcpSocket> tcp_socket,
                   std::unique_ptr<UdpSocket> udp_socket);

  uint16_t udpPort() const;
  research_interface::RobotState udpReadRobotState();
  int udpSendRobotCommand(const research_interface::RobotCommand& command);

  template <typename T>
  void tcpSendRequest(const T& request);

  template <research_interface::Function F, typename T>
  T tcpBlockingReceiveReply();

  bool tcpPollRead();
  int tcpReceiveIntoBuffer();
  size_t bufferSize() const noexcept;
  void* bufferData() noexcept;
  template <typename T>
  bool handleReply(std::function<void(T)> handle);

 private:
  std::unique_ptr<TcpSocket> tcp_socket_;
  std::unique_ptr<UdpSocket> udp_socket_;

  std::vector<uint8_t> read_buffer_;
};

template <research_interface::Function F, typename T>
T Network::tcpBlockingReceiveReply() {
  int bytes_read = 0;
  try {
    std::array<uint8_t, sizeof(T)> buffer;
    constexpr int kBytesTotal = sizeof(T);

    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_->receiveBytes(&buffer.at(bytes_read), bytes_left);
      if (rv == 0) {
        throw NetworkException(
            std::string{"libfranka: FRANKA connection closed"});
      }
      bytes_read += rv;
    }
    if (*reinterpret_cast<research_interface::Function*>(buffer.data()) != F) {
      throw ProtocolException(
          std::string{"libfranka: received reply of wrong type."});
    }
    return *reinterpret_cast<const T*>(buffer.data());
  } catch (TimeoutException const& e) {
    if (bytes_read != 0) {
      throw ProtocolException(std::string{"libfranka: incorrect object size"});
    } else {
      throw NetworkException(
          std::string{"libfranka: FRANKA connection timeout"});
    }
  }
}

template <typename T>
void Network::tcpSendRequest(const T& request) {
  tcp_socket_->sendBytes(&request, sizeof(request));
}

template <typename T>
bool Network::handleReply(std::function<void(T)> handle) {
  if (bufferSize() < sizeof(T)) {
    return false;
  }

  T reply = *reinterpret_cast<T*>(read_buffer_.data());

  size_t remaining_bytes = read_buffer_.size() - sizeof(reply);
  std::memmove(read_buffer_.data(), &read_buffer_[sizeof(reply)],
               remaining_bytes);
  read_buffer_.resize(remaining_bytes);

  handle(reply);
  return true;
}

}  // namespace franka
