#include "mock_server.h"

#include <cstring>
#include <sstream>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <gtest/gtest.h>

MockServer::MockServer() : shutdown_{false}, continue_{false}, initialized_{false} {
  std::unique_lock<std::mutex> lock(mutex_);
  server_thread_ = std::thread(&MockServer::serverThread, this);

  cv_.wait(lock, [this] { return initialized_; });
}

MockServer::~MockServer() {
  {
    std::lock_guard<std::mutex> _(mutex_);
    shutdown_ = true;
  }
  cv_.notify_one();
  server_thread_.join();

  if (!commands_.empty()) {
    std::stringstream ss;
    ss << "Mock server did not process all commands. Unprocessed commands:" << std::endl;
    while (!commands_.empty()) {
      ss << commands_.front().first << std::endl;
      commands_.pop();
    }
    ADD_FAILURE() << ss.str();
  }
}

MockServer& MockServer::onConnect(ConnectCallbackT on_connect) {
  on_connect_ = on_connect;
  return *this;
}

MockServer& MockServer::onStartMotionGenerator(
    StartMotionGeneratorCallbackT on_start_motion_generator) {
  return waitForCommand<research_interface::StartMotionGenerator>(on_start_motion_generator);
}

MockServer& MockServer::onStopMotionGenerator(
    StopMotionGeneratorCallbackT on_stop_motion_generator) {
  return waitForCommand<research_interface::StopMotionGenerator>(on_stop_motion_generator);
}

MockServer& MockServer::onStartController(StartControllerCallbackT on_start_controller) {
  return waitForCommand<research_interface::StartController>(on_start_controller);
}

MockServer& MockServer::onStopController(StopControllerCallbackT on_stop_motion_generator) {
  return waitForCommand<research_interface::StopController>(on_stop_motion_generator);
}

MockServer& MockServer::sendEmptyRobotState() {
  return onSendRobotState(SendRobotStateAlternativeCallbackT());
}

MockServer& MockServer::onSendRobotState(SendRobotStateCallbackT on_send_robot_state) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("onSendRobotState", [=](Socket&, Socket& udp_socket) {
    research_interface::RobotState robot_state = on_send_robot_state();
    udp_socket.sendBytes(&robot_state, sizeof(robot_state));
  });
  return *this;
}

MockServer& MockServer::onSendRobotState(SendRobotStateAlternativeCallbackT on_send_robot_state) {
  return onSendRobotState([=]() {
    research_interface::RobotState robot_state;
    std::memset(&robot_state, 0, sizeof(robot_state));
    if (on_send_robot_state) {
      on_send_robot_state(robot_state);
    }
    return robot_state;
  });
}

MockServer& MockServer::onReceiveRobotCommand(
    ReceiveRobotCommandCallbackT on_receive_robot_command) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("onReceiveRobotCommand", [=](Socket&, Socket& udp_socket) {
    research_interface::RobotCommand robot_command;
    udp_socket.receiveBytes(&robot_command, sizeof(robot_command));
    on_receive_robot_command(robot_command);
  });
  return *this;
}

void MockServer::spinOnce(bool block) {
  std::unique_lock<std::mutex> lock(mutex_);
  continue_ = true;
  cv_.notify_one();
  if (block) {
    cv_.wait(lock, [this]() { return !continue_; });
  }
}

void MockServer::serverThread() {
  std::unique_lock<std::mutex> lock(mutex_);

  const std::string kHostname = "localhost";
  Poco::Net::ServerSocket srv;

  srv =
      Poco::Net::ServerSocket({kHostname, research_interface::kCommandPort});  // does bind + listen
  initialized_ = true;

  cv_.notify_one();
  cv_.wait(lock, [this] { return continue_; });

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);
  tcp_socket.setBlocking(true);
  tcp_socket.setNoDelay(true);

  Socket tcp_socket_wrapper;
  tcp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    int rv = tcp_socket.sendBytes(data, size);
    ASSERT_EQ(static_cast<int>(size), rv);
  };
  tcp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    int rv = tcp_socket.receiveBytes(data, size);
    ASSERT_EQ(static_cast<int>(size), rv);
  };

  uint16_t udp_port;
  handleCommand<research_interface::Connect>(
      tcp_socket_wrapper, [&, this](const research_interface::Connect::Request& request) {
        udp_port = request.udp_port;
        return on_connect_ ? on_connect_(request)
                           : research_interface::Connect::Response(
                                 research_interface::Connect::Status::kSuccess);
      });

  Poco::Net::DatagramSocket udp_socket({kHostname, 0});
  udp_socket.setBlocking(true);
  Socket udp_socket_wrapper;
  udp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    int rv = udp_socket.sendTo(data, size, {remote_address.host(), udp_port});
    ASSERT_EQ(static_cast<int>(size), rv);
  };
  udp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    int rv = udp_socket.receiveFrom(data, size, remote_address);
    ASSERT_EQ(static_cast<int>(size), rv);
  };

  while (!shutdown_) {
    cv_.wait(lock, [this] { return continue_ || shutdown_; });
    while (!commands_.empty()) {
      commands_.front().second(tcp_socket_wrapper, udp_socket_wrapper);
      commands_.pop();
    }

    ASSERT_FALSE(udp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ));

    if (tcp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ)) {
      // Received something on the TCP socket.
      // Test that the Robot closed the connection.
      std::array<uint8_t, 16> buffer;

      int rv = tcp_socket.receiveBytes(buffer.data(), buffer.size());
      ASSERT_EQ(0, rv);
    }

    continue_ = false;
    cv_.notify_one();
  }
}
