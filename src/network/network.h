#pragma once
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <cassert>
#include <chrono>
#include <cstring>
#include <functional>

#include <Poco/Net/StreamSocket.h>
#include <research_interface/types.h>

#include <research_interface/rbk_types.h>

#include "franka/exception.h"

namespace franka {

class Network {
 public:
  explicit Network(const std::string& franka_address,
                   uint16_t franka_port,
                   std::chrono::milliseconds timeout);
  ~Network() noexcept;

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
  T readFromBuffer();
  template <typename T>
  bool handleReply(std::function<void(T)> handle);

 private:
  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;

  Poco::Net::SocketAddress udp_server_address_;
  std::vector<uint8_t> read_buffer_;
};

template <typename T>
T Network::readFromBuffer() {
  assert(read_buffer_.size() >= sizeof(T));

  T reply = *reinterpret_cast<T*>(read_buffer_.data());

  size_t remaining_bytes = read_buffer_.size() - sizeof(reply);
  std::memmove(read_buffer_.data(), &read_buffer_[sizeof(reply)],
               remaining_bytes);
  read_buffer_.resize(remaining_bytes);

  return reply;
}

template <research_interface::Function F, typename T>
T Network::tcpBlockingReceiveReply() {
  int bytes_read = 0;
  try {
    std::array<uint8_t, sizeof(T)> buffer;
    constexpr int kBytesTotal = sizeof(T);

    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_.receiveBytes(&buffer.at(bytes_read), bytes_left);
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
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException(std::string{"libfranka: FRANKA connection error: "} +
                           e.what());
  } catch (Poco::TimeoutException const& e) {
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
  try {
    tcp_socket_.sendBytes(&request, sizeof(request));
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException(std::string{"libfranka: FRANKA tcp error: "} +
                           e.what());
  }
}

template <typename T>
bool Network::handleReply(std::function<void(T)> handle) {
  if (bufferSize() < sizeof(T)) {
    return false;
  }

  T reply = readFromBuffer<T>();

  handle(reply);
  return true;
}

}  // namespace franka