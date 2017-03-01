#include "mock_server.h"

#include <cstring>

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
    ADD_FAILURE() << "Mock server did not process all commands.";
  }
}

MockServer& MockServer::onConnect(ConnectCallbackT on_connect) {
  on_connect_ = on_connect;
  return *this;
}

MockServer& MockServer::onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator) {
  return waitForCommand<research_interface::StartMotionGeneratorRequest, research_interface::StartMotionGeneratorReply>(on_start_motion_generator);
}

MockServer& MockServer::sendEmptyRobotState() {
  return onSendRobotState(SendRobotStateAlternativeCallbackT());
}

MockServer& MockServer::onSendRobotState(SendRobotStateCallbackT on_send_robot_state) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.push_back([=](Socket&, Socket& udp_socket) {
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
  commands_.push_back([=](Socket&, Socket& udp_socket) {
    research_interface::RobotCommand robot_command;
    udp_socket.receiveBytes(&robot_command, sizeof(robot_command));
    on_receive_robot_command(robot_command);
  });
  return *this;
}

void MockServer::spinOnce() {
  {
    std::lock_guard<std::mutex> _(mutex_);
    continue_ = true;
  }
  cv_.notify_one();
}

void MockServer::serverThread() {
  const std::string kHostname = "localhost";
  Poco::Net::ServerSocket srv;
  {
    std::lock_guard<std::mutex> _(mutex_);
    srv = Poco::Net::ServerSocket({kHostname, research_interface::kCommandPort}); // does bind + listen
    initialized_ = true;
  }
  cv_.notify_one();

  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this]{ return continue_; });

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);

  Socket tcp_socket_wrapper;
  tcp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    tcp_socket.sendBytes(data, size);
  };
  tcp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    tcp_socket.receiveBytes(data, size);
  };

  uint16_t udp_port;
  handleCommand<research_interface::ConnectRequest, research_interface::ConnectReply>(tcp_socket_wrapper, [&,this](const research_interface::ConnectRequest& request) {
    udp_port = request.udp_port;
    return on_connect_
           ? on_connect_(request)
           : research_interface::ConnectReply(research_interface::ConnectReply::Status::kSuccess);
  });

  Poco::Net::DatagramSocket udp_socket({kHostname, 0});
  Socket udp_socket_wrapper;
  udp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    udp_socket.sendTo(data, size, {remote_address.host(), udp_port});
  };
  udp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    udp_socket.receiveFrom(data, size, remote_address);
  };

  while (!shutdown_) {
    cv_.wait(lock, [this]{ return continue_ || shutdown_; });
    if (shutdown_) {
      break;
    }

    for (auto command : commands_) {
      command(tcp_socket_wrapper, udp_socket_wrapper);
    }
    commands_.clear();
    continue_ = false;
  }
}

template <typename TRequest, typename TReply>
MockServer& MockServer::waitForCommand(std::function<TReply(const TRequest&)> callback) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.push_back([this,callback](Socket& tcp_socket, Socket&) {
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
