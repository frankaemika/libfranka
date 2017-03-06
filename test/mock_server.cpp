#include "mock_server.h"

#include <cstring>
#include <sstream>

#include <gtest/gtest.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/DatagramSocket.h>

MockServer::MockServer()
    : shutdown_{false},
      continue_{false},
      initialized_{false} {
  std::unique_lock<std::mutex> lock(mutex_);
  server_thread_ = std::thread(&MockServer::serverThread, this);

  cv_.wait(lock, [this]{ return initialized_; });
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

MockServer& MockServer::onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator) {
  return waitForCommand<research_interface::StartMotionGeneratorRequest, research_interface::StartMotionGeneratorReply>(on_start_motion_generator);
}

MockServer& MockServer::onStopMotionGenerator(StopMotionGeneratorCallbackT on_stop_motion_generator) {
  return waitForCommand<research_interface::StopMotionGeneratorRequest, research_interface::StopMotionGeneratorReply>(on_stop_motion_generator);
}

MockServer& MockServer::onStartController(StartControllerCallbackT on_start_motion_generator) {
  return waitForCommand<research_interface::StartControllerRequest, research_interface::StartControllerReply>(on_start_motion_generator);
}

MockServer& MockServer::onStopController(StopControllerCallbackT on_stop_motion_generator) {
  return waitForCommand<research_interface::StopControllerRequest, research_interface::StopControllerReply>(on_stop_motion_generator);
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

MockServer& MockServer::onReceiveRobotCommand(ReceiveRobotCommandCallbackT on_receive_robot_command) {
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

  srv = Poco::Net::ServerSocket({kHostname, research_interface::kCommandPort}); // does bind + listen
  initialized_ = true;

  cv_.notify_one();
  cv_.wait(lock, [this]{ return continue_; });

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
  handleCommand<research_interface::ConnectRequest, research_interface::ConnectReply>(tcp_socket_wrapper, [&,this](const research_interface::ConnectRequest& request) {
    udp_port = request.udp_port;
    return on_connect_
           ? on_connect_(request)
           : research_interface::ConnectReply(research_interface::ConnectReply::Status::kSuccess);
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
    cv_.wait(lock, [this]{ return continue_ || shutdown_; });
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

template <typename TRequest, typename TReply>
MockServer& MockServer::waitForCommand(std::function<TReply(const TRequest&)> callback) {
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(mutex_);
  std::string name = "waitForCommand<"s + typeid(TRequest).name() + ", " + typeid(TReply).name();
  commands_.emplace(name, [this,callback](Socket& tcp_socket, Socket&) {
    handleCommand<TRequest, TReply>(tcp_socket, callback);
  });
  return *this;
}

template <typename TRequest, typename TReply>
void MockServer::handleCommand(Socket& tcp_socket, std::function<TReply(const TRequest&)> callback) {
  std::array<uint8_t, sizeof(TRequest)> buffer;
  tcp_socket.receiveBytes(buffer.data(), buffer.size());
  TRequest request(*reinterpret_cast<TRequest*>(buffer.data()));
  TReply reply = callback(request);
  tcp_socket.sendBytes(&reply, sizeof(reply));
}
