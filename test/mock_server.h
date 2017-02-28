#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <franka/robot_state.h>
#include <research_interface/types.h>
#include <research_interface/rbk_types.h>

class MockServer {
 public:
  using ConnectCallbackT = std::function<research_interface::ConnectReply(const research_interface::ConnectRequest&)>;
  using StartMotionGeneratorCallbackT = std::function<research_interface::StartMotionGeneratorReply(const research_interface::StartMotionGeneratorRequest&)>;
  using SendRobotStateCallbackT = std::function<research_interface::RobotState()>;

  MockServer();
  ~MockServer();

  MockServer& onConnect(ConnectCallbackT on_connect);
  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  MockServer& onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator);
  void start();
  void stop();

  const research_interface::RobotCommand& lastCommand();

 private:
  void serverThread();

  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread server_thread_;
  bool shutdown_;
  bool continue_;
  bool initialized_;

  research_interface::RobotCommand last_command_;

  ConnectCallbackT on_connect_;
  SendRobotStateCallbackT on_send_robot_state_;
  StartMotionGeneratorCallbackT on_start_motion_generator_;
};
