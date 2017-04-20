#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <queue>

#include <franka/robot_state.h>
#include <research_interface/service_types.h>
#include <research_interface/rbk_types.h>

class MockServer {
 public:
  using ConnectCallbackT = std::function<research_interface::Connect::Response(const research_interface::Connect::Request&)>;
  using StartMotionGeneratorCallbackT = std::function<research_interface::StartMotionGenerator::Response(const research_interface::StartMotionGenerator::Request&)>;
  using StopMotionGeneratorCallbackT = std::function<research_interface::StopMotionGenerator::Response(const research_interface::StopMotionGenerator::Request&)>;
  using StartControllerCallbackT = std::function<research_interface::StartController::Response(const research_interface::StartController::Request&)>;
  using StopControllerCallbackT = std::function<research_interface::StopController::Response(const research_interface::StopController::Request&)>;
  using SendRobotStateAlternativeCallbackT = std::function<void(research_interface::RobotState&)>;
  using SendRobotStateCallbackT = std::function<research_interface::RobotState()>;
  using ReceiveRobotCommandCallbackT = std::function<void(const research_interface::RobotCommand&)>;

  MockServer();
  ~MockServer();

  MockServer& sendEmptyRobotState();

  template <typename TResponse>
  MockServer& sendResponse(std::function<TResponse()> create_response);

  MockServer& onConnect(ConnectCallbackT on_connect);
  MockServer& onStartMotionGenerator(StartMotionGeneratorCallbackT on_start_motion_generator);
  MockServer& onStopMotionGenerator(StopMotionGeneratorCallbackT on_stop_motion_generator);
  MockServer& onStartController(StartControllerCallbackT on_start_motion_generator);
  MockServer& onStopController(StopControllerCallbackT on_stop_motion_generator);

  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  MockServer& onSendRobotState(SendRobotStateAlternativeCallbackT on_send_robot_state);
  MockServer& onReceiveRobotCommand(ReceiveRobotCommandCallbackT on_receive_robot_command);

  void spinOnce(bool block = false);

 private:
  struct Socket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
  };

  void serverThread();

  template <typename T>
  MockServer& waitForCommand(std::function<typename T::Response(const typename T::Request&)> callback);
  template <typename T>
  void handleCommand(Socket& tcp_socket, std::function<typename T::Response(const typename T::Request&)> callback);

  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread server_thread_;
  bool shutdown_;
  bool continue_;
  bool initialized_;

  ConnectCallbackT on_connect_;
  std::queue<std::pair<std::string, std::function<void(Socket&, Socket&)>>> commands_;
};

template <typename TResponse>
MockServer& MockServer::sendResponse(std::function<TResponse()> create_response) {
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("sendResponse<"s + typeid(TResponse).name() + ">" , [=](Socket& tcp_socket, Socket&) {
    TResponse response = create_response();
    tcp_socket.sendBytes(&response, sizeof(response));
  });
  return *this;
}
