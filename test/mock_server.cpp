#include "mock_server.h"

#include <sstream>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <gtest/gtest.h>

template <typename C>
MockServer<C>::MockServer(ConnectCallbackT on_connect, uint32_t sequence_number)
    : block_{false},
      shutdown_{false},
      continue_{false},
      initialized_{false},
      sequence_number_{sequence_number},
      on_connect_{on_connect} {
  std::unique_lock<std::mutex> lock(mutex_);
  server_thread_ = std::thread(&MockServer<C>::serverThread, this);

  cv_.wait(lock, [this] { return initialized_; });
  lock.unlock();
  spinOnce();  // spin to accept connections immediately
}

template <typename C>
MockServer<C>::~MockServer() {
  {
    std::lock_guard<std::mutex> _(mutex_);
    shutdown_ = true;
  }
  cv_.notify_one();
  server_thread_.join();

  if (!commands_.empty()) {
    std::stringstream ss;
    ss << "Mock server did not process all commands. Unprocessed commands:" << std::endl;
    while (!commands_.empty()) {
      ss << commands_.front().first << std::endl;
      commands_.pop();
    }
    ADD_FAILURE() << ss.str();
  }
}

template <typename C>
MockServer<C>& MockServer<C>::onReceiveRobotCommand(
    ReceiveRobotCommandCallbackT on_receive_robot_command) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("onReceiveRobotCommand", [=](Socket&, Socket& udp_socket) {
    research_interface::robot::RobotCommand robot_command;
    udp_socket.receiveBytes(&robot_command, sizeof(robot_command));
    on_receive_robot_command(robot_command);
  });
  return *this;
}

template <typename C>
MockServer<C>& MockServer<C>::spinOnce() {
  std::unique_lock<std::mutex> lock(mutex_);
  continue_ = true;
  cv_.notify_one();
  if (block_) {
    cv_.wait(lock, [this]() { return !continue_; });
  }
  block_ = false;
  return *this;
}

template <typename C>
void MockServer<C>::serverThread() {
  std::unique_lock<std::mutex> lock(mutex_);

  const std::string kHostname = "localhost";
  Poco::Net::ServerSocket srv;

  srv = Poco::Net::ServerSocket({kHostname, C::kCommandPort});  // does bind + listen
  initialized_ = true;

  cv_.notify_one();
  cv_.wait(lock, [this] { return continue_; });

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);
  tcp_socket.setBlocking(true);
  tcp_socket.setNoDelay(true);

  Socket tcp_socket_wrapper;
  tcp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    int rv = tcp_socket.sendBytes(data, size);
    ASSERT_EQ(static_cast<int>(size), rv) << "Send error on TCP socket";
  };
  tcp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    int rv = tcp_socket.receiveBytes(data, size);
    ASSERT_EQ(static_cast<int>(size), rv) << "Receive error on TCP socket";
  };

  uint16_t udp_port;
  handleCommand<typename C::Connect>(
      tcp_socket_wrapper, [&, this](const typename C::Connect::Request& request) {
        udp_port = request.udp_port;
        return on_connect_ ? on_connect_(request)
                           : typename C::Connect::Response(C::Connect::Status::kSuccess);
      });

  Poco::Net::DatagramSocket udp_socket({kHostname, 0});
  udp_socket.setBlocking(true);
  Socket udp_socket_wrapper;
  udp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    int rv = udp_socket.sendTo(data, size, {remote_address.host(), udp_port});
    ASSERT_EQ(static_cast<int>(size), rv) << "Send error on UDP socket";
  };
  udp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    int rv = udp_socket.receiveFrom(data, size, remote_address);
    ASSERT_EQ(static_cast<int>(size), rv) << "Receive error on UDP socket";
  };

  typename C::State state{};
  state.message_id = ++sequence_number_;
  udp_socket_wrapper.sendBytes(&state, sizeof(state));

  while (!shutdown_) {
    cv_.wait(lock, [this] { return continue_ || shutdown_; });
    while (!commands_.empty()) {
      commands_.front().second(tcp_socket_wrapper, udp_socket_wrapper);
      commands_.pop();
    }

    continue_ = false;
    cv_.notify_one();
  }

  EXPECT_FALSE(udp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ))
      << "UDP socket still has data";

  if (tcp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ)) {
    // Received something on the TCP socket.
    // Test that the Server closed the connection.
    std::array<uint8_t, 16> buffer;

    int rv = tcp_socket.receiveBytes(buffer.data(), buffer.size());
    EXPECT_EQ(0, rv) << "TCP socket still has data";
  }
}

template <typename C>
MockServer<C>& MockServer<C>::generic(
    std::function<void(MockServer<C>::Socket&, MockServer<C>::Socket&)> generic_command) {
  std::lock_guard<std::mutex> _(mutex_);
  commands_.emplace("generic", generic_command);
  return *this;
}

template <typename C>
void MockServer<C>::sendInitialState(Socket&) {}

template <>
void MockServer<RobotTypes>::sendInitialState(Socket& udp_socket) {
  research_interface::robot::RobotState state{};
  state.message_id = ++sequence_number_;
  udp_socket.sendBytes(&state, sizeof(state));
}
