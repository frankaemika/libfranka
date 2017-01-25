#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <boost/asio.hpp>

#include <franka/robot_state.h>
#include <robot_service/messages.h>

class MockServer {
 public:
  using ConnectCallbackT = std::function<void(const robot_service::RIConnectRequest&, robot_service::RIConnectReply&)>;
  using SendRobotStateCallbackT = std::function<const franka::RobotState&()>;

  MockServer();
  ~MockServer();

  MockServer& onConnect(ConnectCallbackT on_connect);
  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  void start();

 private:
  void serverThread();

  boost::asio::io_service io_service_;
  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread server_thread_;

  ConnectCallbackT on_connect_;
  SendRobotStateCallbackT on_send_robot_state_;
};
