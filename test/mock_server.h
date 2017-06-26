#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

class MockServer {
 public:
  struct Socket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
  };

  using ConnectCallbackT = std::function<research_interface::robot::Connect::Response(
      const research_interface::robot::Connect::Request&)>;
  using SendRobotStateAlternativeCallbackT =
      std::function<void(research_interface::robot::RobotState&)>;
  using SendRobotStateCallbackT = std::function<research_interface::robot::RobotState()>;
  using ReceiveRobotCommandCallbackT =
      std::function<void(const research_interface::robot::RobotCommand&)>;

  MockServer(ConnectCallbackT on_connect = ConnectCallbackT());
  ~MockServer();

  MockServer& sendEmptyRobotState();

  template <typename TResponse>
  MockServer& sendResponse(std::function<TResponse()> create_response);

  MockServer& onSendRobotState(SendRobotStateCallbackT on_send_robot_state);
  MockServer& onSendRobotState(SendRobotStateAlternativeCallbackT on_send_robot_state);
  MockServer& onReceiveRobotCommand(ReceiveRobotCommandCallbackT on_receive_robot_command);

  MockServer& generic(std::function<void(Socket&, Socket&)> generic_command);

  template <typename T>
  void handleCommand(Socket& tcp_socket,
                     std::function<typename T::Response(const typename T::Request&)> callback) {
    std::array<uint8_t, sizeof(typename T::Request)> buffer;
    tcp_socket.receiveBytes(buffer.data(), buffer.size());
    typename T::Request request(*reinterpret_cast<typename T::Request*>(buffer.data()));
    typename T::Response response = callback(request);
    tcp_socket.sendBytes(&response, sizeof(response));
  }

  template <typename T>
  MockServer& waitForCommand(
      std::function<typename T::Response(const typename T::Request&)> callback) {
    using namespace std::string_literals;

    std::lock_guard<std::mutex> _(mutex_);
    std::string name = "waitForCommand<"s + typeid(typename T::Request).name() + ", " +
                       typeid(typename T::Response).name();
    commands_.emplace(name, [this, callback](Socket& tcp_socket, Socket&) {
      handleCommand<T>(tcp_socket, callback);
    });
    return *this;
  }

  MockServer& spinOnce();

 private:
  void serverThread();

  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread server_thread_;
  bool block_;
  bool shutdown_;
  bool continue_;
  bool initialized_;

  const ConnectCallbackT on_connect_;
  std::queue<std::pair<std::string, std::function<void(Socket&, Socket&)>>> commands_;
};

template <typename TResponse>
MockServer& MockServer::sendResponse(std::function<TResponse()> create_response) {
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(mutex_);
  block_ = true;
  commands_.emplace("sendResponse<"s + typeid(TResponse).name() + ">",
                    [=](Socket& tcp_socket, Socket&) {
                      TResponse response = create_response();
                      tcp_socket.sendBytes(&response, sizeof(response));
                    });
  return *this;
}
