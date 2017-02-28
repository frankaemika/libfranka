#include "mock_server.h"

#include <cstring>

#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/ServerSocket.h>

#include <franka/robot_state.h>
#include <research_interface/types.h>
#include <research_interface/rbk_types.h>

MockServer::MockServer()
    : shutdown_{false},
      continue_{false},
      initialized_{false} {
  std::memset(&last_command_, 0, sizeof(last_command_));
}

MockServer::~MockServer() {
  stop();
}

const research_interface::RobotCommand& MockServer::lastCommand() {
  return last_command_;
}

MockServer& MockServer::onConnect(ConnectCallbackT on_connect) {
  on_connect_ = on_connect;
  return *this;
}

MockServer& MockServer::onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator) {
  on_start_motion_generator_ = on_start_motion_generator;
  return *this;
}

MockServer& MockServer::onSendRobotState(SendRobotStateCallbackT on_send_robot_state) {
  on_send_robot_state_ = on_send_robot_state;
  return *this;
}

void MockServer::start() {
  std::unique_lock<std::mutex> lock(mutex_);
  server_thread_ = std::thread(&MockServer::serverThread, this);
  cv_.wait(lock, [this]{ return initialized_; });
  continue_ = true;
  cv_.notify_one();
}

void MockServer::stop() {
  if (shutdown_) {
    return;
  }
  {
    std::lock_guard<std::mutex> _(mutex_);
    shutdown_ = true;
  }
  cv_.notify_one();
  server_thread_.join();
}

void MockServer::serverThread() {
  Poco::Net::ServerSocket srv;
  {
    std::lock_guard<std::mutex> _(mutex_);
    srv = Poco::Net::ServerSocket({"localhost", research_interface::kCommandPort}); // does bind + listen
    initialized_ = true;
  }
  cv_.notify_one();

  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this]{ return continue_; });

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);

  std::array<uint8_t, sizeof(research_interface::ConnectRequest)> buffer;
  tcp_socket.receiveBytes(buffer.data(), buffer.size());
  research_interface::ConnectRequest request(*reinterpret_cast<research_interface::ConnectRequest*>(buffer.data()));

  research_interface::ConnectReply reply = on_connect_
                                           ? on_connect_(request)
                                           : research_interface::ConnectReply(research_interface::ConnectReply::Status::kSuccess);

  tcp_socket.sendBytes(&reply, sizeof(reply));

  if (on_start_motion_generator_) {
    std::array<uint8_t, sizeof(research_interface::StartMotionGeneratorRequest)> buffer;
    tcp_socket.receiveBytes(&buffer, sizeof(buffer));
    research_interface::StartMotionGeneratorReply reply = on_start_motion_generator_(*reinterpret_cast<research_interface::StartMotionGeneratorRequest*>(buffer.data()));
    tcp_socket.sendBytes(&reply, sizeof(reply));
  }

  // Send robot state over UDP
  if (!on_send_robot_state_) {
    cv_.wait(lock, [this]{ return shutdown_; });
    return;
  }

  Poco::Net::DatagramSocket udp_socket({std::string("localhost"), 0});
  research_interface::RobotState robot_state = on_send_robot_state_();

  udp_socket.sendTo(&robot_state, sizeof(robot_state), {remote_address.host(), request.udp_port});

  cv_.wait(lock, [this]{ return shutdown_; });
}
