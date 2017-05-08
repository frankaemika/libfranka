#pragma once

#include <chrono>
#include <functional>
#include <unordered_set>

#include <franka/exception.h>
#include <research_interface/rbk_types.h>
#include <research_interface/types.h>

#include "../command_handler.h"
#include "poco_socket.h"

namespace franka {

class Network {
 public:
  explicit Network(const std::string& franka_address,
                   uint16_t franka_port,
                   std::chrono::milliseconds timeout);

  uint16_t serverVersion() const noexcept;

  research_interface::RobotState udpReadRobotState();

  int udpSendRobotCommand(const research_interface::RobotCommand& command);

  template <typename T>
  void tcpSendRequest(const T& request);

  bool handleReplies(CommandHandler& handler);

  void expectReply(research_interface::Function function);

 private:
  template <research_interface::Function F, typename T>
  T tcpBlockingReceiveReply();

  int tcpReceiveIntoBuffer();

  template <typename T>
  bool handleReply(std::function<void(T)> handle);

  PocoTcpSocket tcp_socket_;
  PocoUdpSocket udp_socket_;

  std::vector<uint8_t> read_buffer_;

  uint16_t ri_version_;

  // Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=60970
  // taken from http://stackoverflow.com/a/24847480/249642
  struct EnumClassHash {
    template <typename T>
    size_t operator()(T t) const {
      return static_cast<size_t>(t);
    }
  };
  std::unordered_multiset<research_interface::Function, EnumClassHash>
      expected_replies_;
};

template <typename T>
void Network::tcpSendRequest(const T& request) {
  tcp_socket_.sendBytes(&request, sizeof(request));
}

}  // namespace franka
