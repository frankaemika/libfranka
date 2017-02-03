#include "mock_server.h"

#include <iostream>

#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/ServerSocket.h>

#include <franka/robot_state.h>
#include <research_interface/constants.h>

MockServer& MockServer::onConnect(ConnectCallbackT on_connect) {
  on_connect_ = on_connect;
  return *this;
}

MockServer& MockServer::onSendRobotState(SendRobotStateCallbackT on_send_robot_state) {
  on_send_robot_state_ = on_send_robot_state;
  return *this;
}

void MockServer::start() {
  server_thread_ = std::thread(&MockServer::serverThread, this);
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock);
}

void MockServer::serverThread() {
  Poco::Net::ServerSocket srv({"localhost", research_interface::kCommandPort}); // does bind + listen
  cv_.notify_one();

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);

  research_interface::ConnectRequest request;
  tcp_socket.receiveBytes(&request, sizeof(request));

  research_interface::ConnectReply reply;
  reply.version = 1;
  reply.status = research_interface::ConnectReply::Status::kSuccess;

  if (on_connect_) {
    on_connect_(request, reply);
  }

  tcp_socket.sendBytes(&reply, sizeof(reply));

  // Send robot state over UDP
  if (!on_send_robot_state_) {
    return;
  }

  Poco::Net::DatagramSocket udp_socket({std::string("localhost"), 0});
  franka::RobotState robot_state = on_send_robot_state_();

  udp_socket.sendTo(&robot_state, sizeof(robot_state), {remote_address.host(), request.udp_port});
}

MockServer::~MockServer() {
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}
