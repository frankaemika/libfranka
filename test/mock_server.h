#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <franka/robot_state.h>
#include <research_interface/types.h>
#include <research_interface/rbk_types.h>

class MockServer {
 public:
  using ConnectCallbackT = std::function<research_interface::ConnectReply(const research_interface::ConnectRequest&)>;
  using StartMotionGeneratorCallbackT = std::function<research_interface::StartMotionGeneratorReply(const research_interface::StartMotionGeneratorRequest&)>;
  using SendRobotStateAlternativeCallbackT = std::function<void(research_interface::RobotState&)>;
  using SendRobotStateCallbackT = std::function<research_interface::RobotState()>;
  using ReceiveRobotCommandCallbackT = std::function<void(const research_interface::RobotCommand&)>;

  MockServer();
  ~MockServer();

  MockServer& sendEmptyRobotState();

  MockServer& onConnect(ConnectCallbackT on_connect);
  MockServer& onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator);

  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  MockServer& onSendRobotState(SendRobotStateAlternativeCallbackT on_send_robot_state);
  MockServer& onReceiveRobotCommand(ReceiveRobotCommandCallbackT on_receive_robot_command);

  void spinOnce();

 private:
  struct Socket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
  };

  void serverThread();

  template <typename TRequest, typename TReply>
  MockServer& waitForCommand(std::function<TReply(const TRequest&)> callback);
  template <typename TRequest, typename TReply>
  void handleCommand(Socket& tcp_socket, std::function<TReply(const TRequest&)> callback);

  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread server_thread_;
  bool shutdown_;
  bool continue_;
  bool initialized_;

  ConnectCallbackT on_connect_;
  std::vector<std::function<void(Socket&, Socket&)>> commands_;
};
