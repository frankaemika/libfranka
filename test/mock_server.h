#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <franka/robot_state.h>
#include <research_interface/gripper/types.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

template <typename C>
class MockServer {
 private:
  struct Socket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
  };

 public:
  using ConnectCallbackT = std::function<typename C::Response(const typename C::Request&)>;
  using ReceiveRobotCommandCallbackT =
      std::function<void(const research_interface::robot::RobotCommand&)>;

  MockServer(ConnectCallbackT on_connect = ConnectCallbackT());
  ~MockServer();

  template <typename T>
  MockServer& sendEmptyState();

  template <typename TResponse>
  MockServer& sendResponse(std::function<TResponse()> create_response);

  template <typename T>
  MockServer& onSendUDP(std::function<T()> on_send_udp);

  template <typename T>
  MockServer& onSendUDP(std::function<void(T&)> on_send_udp);

  MockServer& onReceiveRobotCommand(ReceiveRobotCommandCallbackT on_receive_robot_command);

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
  static uint16_t port;

  const ConnectCallbackT on_connect_;
  std::queue<std::pair<std::string, std::function<void(Socket&, Socket&)>>> commands_;
};

template <typename C>
template <typename TResponse>
MockServer<C>& MockServer<C>::sendResponse(std::function<TResponse()> create_response) {
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

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::sendEmptyState() {
  return onSendUDP<T>(std::function<void(T&)>());
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::onSendUDP(std::function<T()> on_send_udp) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("onSendUDP", [=](Socket&, Socket& udp_socket) {
    T state = on_send_udp();
    udp_socket.sendBytes(&state, sizeof(state));
  });
  block_ = true;
  return *this;
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::onSendUDP(std::function<void(T&)> on_send_udp) {
  return onSendUDP<T>([=]() {
    T state{};
    if (on_send_udp) {
      on_send_udp(state);
    }
    return state;
  });
}

template class MockServer<research_interface::robot::Connect>;
template class MockServer<research_interface::gripper::Connect>;