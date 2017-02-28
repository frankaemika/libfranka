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
}

MockServer::~MockServer() {
  stop();
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

  // Send robot state over UDP
  if (!on_send_robot_state_) {
    cv_.wait(lock, [this]{ return shutdown_; });
    return;
  }

  Poco::Net::DatagramSocket udp_socket({std::string("localhost"), 0});
  franka::RobotState robot_state = on_send_robot_state_();
  research_interface::RobotState rbk_robot_state;
  std::memset(&rbk_robot_state, 0, sizeof(rbk_robot_state));

  std::copy(robot_state.q_start.cbegin(), robot_state.q_start.cend(),
            rbk_robot_state.q_start.begin());
  std::copy(robot_state.O_T_EE_start.cbegin(), robot_state.O_T_EE_start.cend(),
            rbk_robot_state.O_T_EE_start.begin());
  std::copy(robot_state.elbow_start.cbegin(), robot_state.elbow_start.cend(),
            rbk_robot_state.elbow_start.begin());
  std::copy(robot_state.tau_J.cbegin(), robot_state.tau_J.cend(),
            rbk_robot_state.tau_J.begin());
  std::copy(robot_state.dtau_J.cbegin(), robot_state.dtau_J.cend(),
            rbk_robot_state.dtau_J.begin());
  std::copy(robot_state.q.cbegin(), robot_state.q.cend(),
            rbk_robot_state.q.begin());
  std::copy(robot_state.dq.cbegin(), robot_state.dq.cend(),
            rbk_robot_state.dq.begin());
  std::copy(robot_state.q_d.cbegin(), robot_state.q_d.cend(),
            rbk_robot_state.q_d.begin());
  std::copy(robot_state.joint_contact.cbegin(),
            robot_state.joint_contact.cend(),
            rbk_robot_state.joint_contact.begin());
  std::copy(robot_state.cartesian_contact.cbegin(),
            robot_state.cartesian_contact.cend(),
            rbk_robot_state.cartesian_contact.begin());
  std::copy(robot_state.joint_collision.cbegin(),
            robot_state.joint_collision.cend(),
            rbk_robot_state.joint_collision.begin());
  std::copy(robot_state.cartesian_collision.cbegin(),
            robot_state.cartesian_collision.cend(),
            rbk_robot_state.cartesian_collision.begin());
  std::copy(robot_state.tau_ext_hat_filtered.cbegin(),
            robot_state.tau_ext_hat_filtered.cend(),
            rbk_robot_state.tau_ext_hat_filtered.begin());
  std::copy(robot_state.O_F_ext_hat_K.cbegin(),
            robot_state.O_F_ext_hat_K.cend(),
            rbk_robot_state.O_F_ext_hat_K.begin());
  std::copy(robot_state.K_F_ext_hat_K.cbegin(),
            robot_state.K_F_ext_hat_K.cend(),
            rbk_robot_state.K_F_ext_hat_K.begin());
  rbk_robot_state.message_id = robot_state.message_id;

  udp_socket.sendTo(&rbk_robot_state, sizeof(rbk_robot_state), {remote_address.host(), request.udp_port});

  cv_.wait(lock, [this]{ return shutdown_; });
}
