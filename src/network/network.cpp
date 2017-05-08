#include "network.h"

#include <cstring>
#include <memory>
#include <sstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Network::Network(const std::string& franka_address,
                 uint16_t franka_port,
                 std::chrono::milliseconds timeout)
    : tcp_socket_(franka_address, franka_port, timeout), udp_socket_(timeout) {
  research_interface::ConnectRequest connect_request(udp_socket_.port());

  tcpSendRequest(connect_request);

  research_interface::ConnectReply connect_reply =
      tcpBlockingReceiveReply<research_interface::Function::kConnect,
                              research_interface::ConnectReply>();
  switch (connect_reply.status) {
    case research_interface::ConnectReply::Status::
        kIncompatibleLibraryVersion: {
      std::stringstream message;
      message << "libfranka: incompatible library version. " << std::endl
              << "Server version: " << connect_reply.version << std::endl
              << "Library version: " << research_interface::kVersion;
      throw IncompatibleVersionException(message.str());
    }
    case research_interface::ConnectReply::Status::kSuccess:
      ri_version_ = connect_reply.version;
      break;
    default:
      throw ProtocolException("libfranka: protocol error");
  }
}

int Network::udpSendRobotCommand(
    const research_interface::RobotCommand& command) {
  return udp_socket_.sendTo(&command, sizeof(command));
}

research_interface::RobotState Network::udpReadRobotState() {
  std::array<uint8_t, sizeof(research_interface::RobotState)> buffer;
  int bytes_received = udp_socket_.receiveFrom(buffer.data(), buffer.size());
  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka: incorrect object size");
  }
  return *reinterpret_cast<research_interface::RobotState*>(buffer.data());
}

int Network::tcpReceiveIntoBuffer() {
  size_t offset = read_buffer_.size();
  read_buffer_.resize(offset + tcp_socket_.available());
  std::pair<bool, int> result =
      tcp_socket_.receiveBytes(&read_buffer_[offset], tcp_socket_.available());
  if (!result.first) {
    throw NetworkException("libfranka: FRANKA connection timeout");
  }
  return result.second;
}

bool Network::handleReplies(CommandHandler& handler) {
  if (tcp_socket_.poll()) {
    int rv = tcpReceiveIntoBuffer();

    if (rv == 0) {
      return false;  // server sent EOF
    }

    if (read_buffer_.size() < sizeof(research_interface::Function)) {
      return true;
    }

    research_interface::Function function =
        *reinterpret_cast<research_interface::Function*>(read_buffer_.data());

    if (expected_replies_.find(function) == expected_replies_.end()) {
      throw ProtocolException("libfranka: unexpected reply!");
    }
    bool handled = false;
    switch (function) {
      case research_interface::Function::kStartMotionGenerator:
        handled = handleReply<research_interface::StartMotionGeneratorReply>(
            std::bind(&CommandHandler::handleStartMotionGeneratorReply,
                      &handler, std::placeholders::_1));
        break;
      case research_interface::Function::kStopMotionGenerator:
        handled = handleReply<research_interface::StopMotionGeneratorReply>(
            std::bind(&CommandHandler::handleStopMotionGeneratorReply, &handler,
                      std::placeholders::_1));
        break;
      case research_interface::Function::kStartController:
        handled = handleReply<research_interface::StartControllerReply>(
            std::bind(&CommandHandler::handleStartControllerReply, &handler,
                      std::placeholders::_1));
        break;
      case research_interface::Function::kStopController:
        handled = handleReply<research_interface::StopControllerReply>(
            std::bind(&CommandHandler::handleStopControllerReply, &handler,
                      std::placeholders::_1));
        break;
      default:
        throw ProtocolException("libfranka: unsupported reply!");
    }

    if (handled) {
      auto iterator = expected_replies_.find(function);
      if (iterator != expected_replies_.end()) {
        expected_replies_.erase(iterator);
      }
    }
  }
  return true;
}

void Network::expectReply(research_interface::Function function) {
  expected_replies_.insert(function);
}

uint16_t Network::serverVersion() const noexcept {
  return ri_version_;
}

template <research_interface::Function F, typename T>
T Network::tcpBlockingReceiveReply() {
  int bytes_read = 0;
  std::array<uint8_t, sizeof(T)> buffer;
  constexpr int kBytesTotal = sizeof(T);

  while (bytes_read < kBytesTotal) {
    int bytes_left = kBytesTotal - bytes_read;
    std::pair<bool, int> result =
        tcp_socket_.receiveBytes(&buffer.at(bytes_read), bytes_left);

    if (!result.first) {
      if (bytes_read != 0) {
        throw ProtocolException(
            std::string{"libfranka: incorrect object size"});
      } else {
        throw NetworkException(
            std::string{"libfranka: FRANKA connection timeout"});
      }
    } else if (result.second == 0) {
      throw NetworkException(
          std::string{"libfranka: FRANKA connection closed"});
    }

    bytes_read += result.second;
  }

  if (*reinterpret_cast<research_interface::Function*>(buffer.data()) != F) {
    throw ProtocolException(
        std::string{"libfranka: received reply of wrong type."});
  }
  return *reinterpret_cast<const T*>(buffer.data());
}

template <typename T>
bool Network::handleReply(std::function<void(T)> handle) {
  if (read_buffer_.size() < sizeof(T)) {
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
