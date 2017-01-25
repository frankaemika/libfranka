#include "mock_server.h"

#include <iostream>

#include <franka/robot_state.h>

MockServer::MockServer()
  : io_service_{} {
}

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
  // Wait for TCP connection
  boost::asio::ip::tcp::acceptor acceptor(io_service_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 1337));
  cv_.notify_one();

  boost::asio::ip::tcp::socket tcp_socket(io_service_);
  acceptor.accept(tcp_socket);

  message_types::ConnectRequest request;
  boost::asio::read(tcp_socket, boost::asio::buffer(&request, sizeof(request)));

  message_types::ConnectReply reply;
  reply.ri_version = 1;
  reply.status_code = message_types::ConnectReply::StatusCode::kSuccess;

  if (on_connect_) {
    on_connect_(request, reply);
  }

  boost::asio::write(tcp_socket, boost::asio::buffer(&reply, sizeof(reply)));

  // Send robot state over UDP
  if (!on_send_robot_state_) {
    return;
  }
  boost::asio::ip::udp::socket udp_socket(io_service_, boost::asio::ip::udp::v4());
  const franka::RobotState& robot_state = on_send_robot_state_();
  boost::system::error_code error;
  boost::asio::ip::udp::endpoint remote_endpoint(tcp_socket.remote_endpoint().address(),
                                                 request.udp_port);
  udp_socket.send_to(boost::asio::buffer(&robot_state, sizeof(robot_state)),
                remote_endpoint, 0, error);
}

MockServer::~MockServer() {
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}
