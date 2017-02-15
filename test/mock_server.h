#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <franka/robot_state.h>
#include <research_interface/types.h>

class MockServer {
 public:
  using ConnectCallbackT = std::function<research_interface::ConnectReply(const research_interface::ConnectRequest&)>;
  using SendRobotStateCallbackT = std::function<franka::RobotState()>;

  MockServer();
  ~MockServer();

  MockServer& onConnect(ConnectCallbackT on_connect);
  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  void start();
  void stop();

 private:
  void serverThread();

  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread server_thread_;
  bool shutdown_;
  bool continue_;
  bool initialized_;

  ConnectCallbackT on_connect_;
  SendRobotStateCallbackT on_send_robot_state_;
};
