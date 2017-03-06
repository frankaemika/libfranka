#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <queue>

#include <franka/robot_state.h>
#include <research_interface/types.h>
#include <research_interface/rbk_types.h>

class MockServer {
 public:
  using ConnectCallbackT = std::function<research_interface::ConnectReply(const research_interface::ConnectRequest&)>;
  using StartMotionGeneratorCallbackT = std::function<research_interface::StartMotionGeneratorReply(const research_interface::StartMotionGeneratorRequest&)>;
  using StopMotionGeneratorCallbackT = std::function<research_interface::StopMotionGeneratorReply(const research_interface::StopMotionGeneratorRequest&)>;
  using StartControllerCallbackT = std::function<research_interface::StartControllerReply(const research_interface::StartControllerRequest&)>;
  using StopControllerCallbackT = std::function<research_interface::StopControllerReply(const research_interface::StopControllerRequest&)>;
  using SendRobotStateAlternativeCallbackT = std::function<void(research_interface::RobotState&)>;
  using SendRobotStateCallbackT = std::function<research_interface::RobotState()>;
  using ReceiveRobotCommandCallbackT = std::function<void(const research_interface::RobotCommand&)>;

  MockServer();
  ~MockServer();

  MockServer& sendEmptyRobotState();

  template <typename TReply>
  MockServer& sendReply(std::function<TReply()> create_reply);

  MockServer& onConnect(ConnectCallbackT on_connect);
  MockServer& onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator);
  MockServer& onStopMotionGenerator(StopMotionGeneratorCallbackT on_stop_motion_generator);
  MockServer& onStartController(StartControllerCallbackT on_start_motion_generator);
  MockServer& onStopController(StopControllerCallbackT on_stop_motion_generator);

  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  MockServer& onSendRobotState(SendRobotStateAlternativeCallbackT on_send_robot_state);
  MockServer& onReceiveRobotCommand(ReceiveRobotCommandCallbackT on_receive_robot_command);

  MockServer& sync(std::function<void()>* wait_function);

  void spinOnce(bool block = false);

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
  std::queue<std::pair<std::string, std::function<void(Socket&, Socket&)>>> commands_;
};

template <typename TReply>
MockServer& MockServer::sendReply(std::function<TReply()> create_reply) {
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("sendReply<"s + typeid(TReply).name() + ">" , [=](Socket& tcp_socket, Socket&) {
    TReply reply = create_reply();
    tcp_socket.sendBytes(&reply, sizeof(reply));
  });
  return *this;
}
