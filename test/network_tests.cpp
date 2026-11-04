// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <gtest/gtest.h>

#include <franka/exception.h>
#include <research_interface/robot/service_types.h>

#include "network.h"

using namespace std::chrono_literals;  // NOLINT(google-build-using-namespace)

using franka::Network;
using franka::ProtocolException;
using research_interface::robot::CommandHeader;
using research_interface::robot::Connect;
using research_interface::robot::GetRobotModel;

namespace {

/**
 * Minimal loopback TCP peer that plays the robot side of a single connection. It accepts one
 * client, writes the given raw bytes and then keeps the connection open until destroyed, so the
 * client sees exactly the bytes under test and nothing else.
 */
class RawTcpResponder {
 public:
  explicit RawTcpResponder(std::vector<uint8_t> bytes)
      : server_socket_(Poco::Net::SocketAddress("127.0.0.1", 0)), bytes_(std::move(bytes)) {
    thread_ = std::thread([this] {
      // Poll instead of blocking in accept so that a failing test cannot hang the destructor.
      while (!server_socket_.poll(Poco::Timespan(0, 1000), Poco::Net::Socket::SELECT_READ)) {
        if (done_) {
          return;
        }
      }
      Poco::Net::StreamSocket connection = server_socket_.acceptConnection();
      if (!bytes_.empty()) {
        connection.sendBytes(bytes_.data(), static_cast<int>(bytes_.size()));
      }
      while (!done_) {
        std::this_thread::sleep_for(1ms);
      }
      connection.close();
    });
  }

  ~RawTcpResponder() {
    done_ = true;
    thread_.join();
  }

  RawTcpResponder(const RawTcpResponder&) = delete;
  RawTcpResponder& operator=(const RawTcpResponder&) = delete;

  uint16_t port() const { return server_socket_.address().port(); }

 private:
  Poco::Net::ServerSocket server_socket_;
  std::vector<uint8_t> bytes_;
  std::atomic_bool done_{false};
  std::thread thread_;
};

std::vector<uint8_t> serializeHeader(const CommandHeader& header) {
  std::vector<uint8_t> bytes(sizeof(header));
  std::memcpy(bytes.data(), &header, sizeof(header));
  return bytes;
}

template <typename T>
std::vector<uint8_t> serializeMessage(const T& message) {
  std::vector<uint8_t> bytes(sizeof(message));
  std::memcpy(bytes.data(), &message, sizeof(message));
  return bytes;
}

using ConnectResponseMessage = Connect::Message<Connect::Response>;

std::vector<uint8_t> completeConnectResponse(uint32_t command_id) {
  ConnectResponseMessage message(
      CommandHeader(Connect::kCommand, command_id, sizeof(ConnectResponseMessage)),
      Connect::Response(Connect::Status::kSuccess));
  return serializeMessage(message);
}

// A header that announces only itself: no payload follows.
std::vector<uint8_t> headerOnlyResponse(research_interface::robot::Command command,
                                        uint32_t command_id) {
  return serializeHeader(CommandHeader(command, command_id, sizeof(CommandHeader)));
}

// Polls the non-blocking receive path until the response arrives or the deadline passes.
template <typename T>
bool pollResponse(Network& network,
                  uint32_t command_id,
                  std::function<void(const typename T::Response&)> handler) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (network.tcpReceiveResponse<T>(command_id, handler)) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

}  // anonymous namespace

TEST(Network, BlockingReceiveDecodesCompleteResponse) {
  RawTcpResponder robot(completeConnectResponse(0));
  Network network("127.0.0.1", robot.port());

  Connect::Response response = network.tcpBlockingReceiveResponse<Connect>(0);
  EXPECT_EQ(Connect::Status::kSuccess, response.status);
  EXPECT_EQ(research_interface::robot::kVersion, response.version);
}

TEST(Network, AsyncReceiveDecodesCompleteResponse) {
  RawTcpResponder robot(completeConnectResponse(0));
  Network network("127.0.0.1", robot.port());

  bool handled = false;
  EXPECT_TRUE(pollResponse<Connect>(network, 0, [&](const Connect::Response& response) {
    EXPECT_EQ(Connect::Status::kSuccess, response.status);
    handled = true;
  }));
  EXPECT_TRUE(handled);
}

TEST(Network, BlockingReceiveRejectsHeaderOnlyResponse) {
  RawTcpResponder robot(headerOnlyResponse(Connect::kCommand, 0));
  Network network("127.0.0.1", robot.port());

  EXPECT_THROW(network.tcpBlockingReceiveResponse<Connect>(0), ProtocolException);
}

TEST(Network, AsyncReceiveRejectsHeaderOnlyResponse) {
  RawTcpResponder robot(headerOnlyResponse(Connect::kCommand, 0));
  Network network("127.0.0.1", robot.port());

  bool handled = false;
  EXPECT_THROW(pollResponse<Connect>(network, 0, [&](const Connect::Response&) { handled = true; }),
               ProtocolException);
  EXPECT_FALSE(handled);
}

TEST(Network, RejectsTruncatedFixedSizeResponse) {
  // Declares and delivers one byte less than a complete Connect response.
  std::vector<uint8_t> bytes = completeConnectResponse(0);
  bytes.pop_back();
  CommandHeader header(Connect::kCommand, 0, static_cast<uint32_t>(bytes.size()));
  std::memcpy(bytes.data(), &header, sizeof(header));

  RawTcpResponder robot(bytes);
  Network network("127.0.0.1", robot.port());

  EXPECT_THROW(network.tcpBlockingReceiveResponse<Connect>(0), ProtocolException);
}

TEST(Network, RejectsDeclaredSizeSmallerThanHeader) {
  RawTcpResponder robot(serializeHeader(CommandHeader(Connect::kCommand, 0, 8)));
  Network network("127.0.0.1", robot.port());

  EXPECT_THROW(network.tcpBlockingReceiveResponse<Connect>(0), ProtocolException);
}

TEST(Network, RejectsOversizedDeclaredSizeWithoutWaitingForPayload) {
  // Only the header is sent. The client must reject the declared size up front instead of
  // allocating it and then waiting for a payload that never arrives.
  RawTcpResponder robot(
      serializeHeader(CommandHeader(Connect::kCommand, 0, std::numeric_limits<uint32_t>::max())));
  Network network("127.0.0.1", robot.port());

  EXPECT_THROW(network.tcpBlockingReceiveResponse<Connect>(0), ProtocolException);
}

TEST(Network, RejectsDeclaredSizeJustAboveMaximum) {
  RawTcpResponder robot(serializeHeader(CommandHeader(
      GetRobotModel::kCommand, 0, static_cast<uint32_t>(Network::kMaxTcpResponseSize + 1))));
  Network network("127.0.0.1", robot.port());

  EXPECT_THROW(network.tcpBlockingReceiveResponse<GetRobotModel>(0), ProtocolException);
}

TEST(Network, BlockingReceiveKeepsTrailingVariableLengthData) {
  const std::vector<uint8_t> trailing{1, 2, 3, 4, 5};
  std::vector<uint8_t> bytes = completeConnectResponse(0);
  bytes.insert(bytes.end(), trailing.begin(), trailing.end());
  CommandHeader header(Connect::kCommand, 0, static_cast<uint32_t>(bytes.size()));
  std::memcpy(bytes.data(), &header, sizeof(header));

  RawTcpResponder robot(bytes);
  Network network("127.0.0.1", robot.port());

  std::vector<uint8_t> vl_buffer;
  Connect::Response response = network.tcpBlockingReceiveResponse<Connect>(0, &vl_buffer);
  EXPECT_EQ(Connect::Status::kSuccess, response.status);
  EXPECT_EQ(trailing, vl_buffer);
}

TEST(Network, ReceivesRobotModel) {
  const std::string urdf = "<robot name=\"fr3\"></robot>";
  GetRobotModel::Message<GetRobotModel::Response> message(
      CommandHeader(GetRobotModel::kCommand, 0, 0),
      GetRobotModel::Response(GetRobotModel::Status::kSuccess, urdf));

  RawTcpResponder robot(message.serialize());
  Network network("127.0.0.1", robot.port());

  GetRobotModel::Response response = network.tcpBlockingReceiveResponse<GetRobotModel>(0);
  EXPECT_EQ(GetRobotModel::Status::kSuccess, response.status);
  EXPECT_EQ(urdf, response.robot_model);
}

TEST(Network, RejectsHeaderOnlyRobotModelResponse) {
  RawTcpResponder robot(headerOnlyResponse(GetRobotModel::kCommand, 0));
  Network network("127.0.0.1", robot.port());

  EXPECT_THROW(network.tcpBlockingReceiveResponse<GetRobotModel>(0), ProtocolException);
}
