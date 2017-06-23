#pragma once

#include <chrono>
#include <cstring>
#include <functional>
#include <unordered_set>

#include <franka/exception.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>

namespace franka {

class Network {
 public:
  explicit Network(const std::string& franka_address,
                   uint16_t franka_port,
                   std::chrono::milliseconds timeout);
  ~Network();

  research_interface::robot::RobotState udpReadRobotState();

  uint16_t udpPort() const noexcept;

  template <typename T>
  typename T::Response tcpBlockingReceiveResponse();

  void udpSendRobotCommand(const research_interface::robot::RobotCommand& command);

  template <typename T>
  void tcpSendRequest(const T& request);

  bool tcpReadResponse(research_interface::robot::Function* function);

  template <typename T>
  bool tcpHandleResponse(std::function<void(const typename T::Response&)> handler);

  void tcpReceiveIntoBuffer(char* buffer, const int read_size);

 private:
  int tcpReceiveIntoBuffer();

  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
  Poco::Net::SocketAddress udp_server_address_;

  std::vector<uint8_t> read_buffer_;
};

template <typename T>
void Network::tcpSendRequest(const T& request) try {
  tcp_socket_.sendBytes(&request, sizeof(request));
} catch (const Poco::Exception& e) {
  throw NetworkException(std::string{"libfranka: tcp send bytes: "} + e.what());
}

template <typename T>
bool Network::tcpHandleResponse(std::function<void(const typename T::Response&)> handler) {
  if (read_buffer_.size() < sizeof(typename T::Response)) {
    return false;
  }

  typename T::Response response = *reinterpret_cast<typename T::Response*>(read_buffer_.data());

  size_t remaining_bytes = read_buffer_.size() - sizeof(response);
  std::memmove(read_buffer_.data(), &read_buffer_[sizeof(response)], remaining_bytes);
  read_buffer_.resize(remaining_bytes);

  handler(response);
  return true;
}

template <typename T>
typename T::Response Network::tcpBlockingReceiveResponse() {
  std::array<char, sizeof(typename T::Response)> buffer;
  tcpReceiveIntoBuffer(buffer.data(), buffer.size());
  typename T::Response const* response =
      reinterpret_cast<const typename T::Response*>(buffer.data());
  if (response->function != T::kFunction) {
    throw ProtocolException(std::string{"libfranka: received response of wrong type."});
  }
  return *response;
}

}  // namespace franka
