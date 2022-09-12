// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
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
  std::unique_lock<std::timed_mutex> lock(command_mutex_);
  server_thread_ = std::thread(&MockServer<C>::serverThread, this);

  cv_.wait(lock, [this] { return initialized_; });
  lock.unlock();
  spinOnce();  // Spin to accept connections immediately.
}

template <typename C>
MockServer<C>::~MockServer() {
  std::unique_lock<std::timed_mutex> l(command_mutex_, std::defer_lock);
  // If we can't get the lock, the thread will still be terminated
  // by its destructor.
  if (l.try_lock_for(std::chrono::seconds(1))) {
    shutdown_ = true;
    l.unlock();
    cv_.notify_one();
    server_thread_.join();
  } else {
    std::cerr << "MockServer: Couldn't terminate the server thread properly." << std::endl;
  }

  if (!commands_.empty()) {
    std::stringstream ss;
    ss << "Mock server did not process all commands. Unprocessed commands:" << std::endl;
    while (!commands_.empty()) {
      ss << commands_.front().first << std::endl;
      commands_.pop_front();
    }
    ADD_FAILURE() << ss.str();
  }
}

template <typename C>
MockServer<C>& MockServer<C>::onReceiveRobotCommand(
    ReceiveRobotCommandCallbackT on_receive_robot_command) {
  std::lock_guard<std::timed_mutex> _(command_mutex_);
  commands_.emplace_back("onReceiveRobotCommand", [=](Socket&, Socket& udp_socket) {
    research_interface::robot::RobotCommand robot_command;
    udp_socket.receiveBytes(&robot_command, sizeof(robot_command));
    if (on_receive_robot_command) {
      on_receive_robot_command(robot_command);
    }
  });
  return *this;
}

template <typename C>
MockServer<C>& MockServer<C>::spinOnce() {
  std::unique_lock<std::timed_mutex> lock(command_mutex_);
  continue_ = true;
  cv_.notify_one();
  if (block_) {
    cv_.wait(lock, [this]() { return !continue_; });
  }
  block_ = false;
  return *this;
}

template <typename C>
void MockServer<C>::ignoreUdpBuffer() {
  ignore_udp_buffer_ = true;
}

template <typename C>
void MockServer<C>::serverThread() {
  std::unique_lock<std::timed_mutex> lock(command_mutex_);

  constexpr const char* kHostname = "127.0.0.1";
  Poco::Net::ServerSocket srv;
  srv.bind({kHostname, C::kCommandPort}, true);
  srv.listen();

  initialized_ = true;

  cv_.notify_one();
  cv_.wait(lock, [this] { return continue_; });

  if (shutdown_) {
    return;
  }

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);
  tcp_socket.setBlocking(true);
  tcp_socket.setNoDelay(true);

  Socket tcp_socket_wrapper;
  tcp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    std::lock_guard<std::mutex> _(tcp_mutex_);
    int rv = tcp_socket.sendBytes(data, size);
    ASSERT_EQ(static_cast<int>(size), rv) << "Send error on TCP socket";
  };
  tcp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    std::lock_guard<std::mutex> _(tcp_mutex_);
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

  Poco::Net::DatagramSocket udp_socket;
  udp_socket.bind({kHostname, 0});
  udp_socket.setBlocking(true);
  Socket udp_socket_wrapper;
  udp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    std::lock_guard<std::mutex> _(udp_mutex_);
    int rv = udp_socket.sendTo(data, size, {remote_address.host(), udp_port});
    ASSERT_EQ(static_cast<int>(size), rv) << "Send error on UDP socket";
  };
  udp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    std::lock_guard<std::mutex> _(udp_mutex_);
    int rv = udp_socket.receiveFrom(data, size, remote_address);
    ASSERT_EQ(static_cast<int>(size), rv) << "Receive error on UDP socket";
  };

  sendInitialState(udp_socket_wrapper);

  while (!shutdown_) {
    cv_.wait(lock, [this] { return continue_ || shutdown_; });
    while (!commands_.empty()) {
      auto callback = commands_.front().second;
      commands_.pop_front();
      lock.unlock();
      callback(tcp_socket_wrapper, udp_socket_wrapper);
      lock.lock();
    }

    continue_ = false;
    cv_.notify_one();
  }

  if (!ignore_udp_buffer_) {
    EXPECT_FALSE(udp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ))
        << "UDP socket still has data";
  }

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
  std::lock_guard<std::timed_mutex> _(command_mutex_);
  commands_.emplace_back("generic", generic_command);
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

template <typename C>
MockServer<C>& MockServer<C>::doForever(std::function<bool()> callback) {
  std::lock_guard<std::timed_mutex> _(command_mutex_);
  return doForever(callback, commands_.end());
}

template <typename C>
MockServer<C>& MockServer<C>::doForever(std::function<bool()> callback,
                                        typename decltype(MockServer::commands_)::iterator it) {
  auto callback_wrapper = [=](Socket&, Socket&) {
    std::unique_lock<std::timed_mutex> lock(command_mutex_);
    if (shutdown_) {
      return;
    }
    size_t old_commands = commands_.size();
    lock.unlock();
    if (callback()) {
      lock.lock();
      size_t new_commands = commands_.size() - old_commands;

      // Reorder the commands added by callback to the front.
      decltype(commands_) commands(commands_.cbegin() + old_commands, commands_.cend());
      commands.insert(commands.end(), commands_.cbegin(), commands_.cbegin() + old_commands);
      commands_ = commands;

      // Insert after the new commands added by callback.
      doForever(callback, commands_.begin() + new_commands);
      lock.unlock();
    }
    std::this_thread::yield();
  };
  commands_.emplace(it, "doForever", callback_wrapper);
  return *this;
}

template class MockServer<RobotTypes>;
template class MockServer<GripperTypes>;
template class MockServer<VacuumGripperTypes>;
