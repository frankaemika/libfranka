#include "robot_impl.h"

#include <chrono>
#include <iostream>
#include <sstream>

#include <Poco/Net/NetException.h>

#include "message_types.h"

namespace franka {

Robot::Impl::Impl(const std::string& franka_address,
                  uint16_t franka_port, std::chrono::seconds timeout)
  : ri_version_{0} {

  Poco::Timespan poco_timeout(timeout.count(), 0);
  try {
    tcp_socket_.connect({franka_address, franka_port}, poco_timeout);
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(poco_timeout);
    tcp_socket_.setReceiveTimeout(poco_timeout);

    udp_socket_.setReceiveTimeout(poco_timeout);
    udp_socket_.bind({"0.0.0.0", 0});

    message_types::ConnectRequest connect_request;
    connect_request.function_id = message_types::FunctionId::kConnect;
    connect_request.ri_library_version = message_types::kRiLibraryVersion;
    connect_request.udp_port = udp_socket_.address().port();

    tcp_socket_.sendBytes(&connect_request, sizeof(connect_request));

    message_types::ConnectReply connect_reply;
    tcpReceiveObject(connect_reply);
    switch (connect_reply.status_code) {
      case message_types::ConnectReply::StatusCode::
          kIncompatibleLibraryVersion: {
        std::stringstream message;
        message << "libfranka: incompatible library version. "
                << "Server version: " << connect_reply.ri_version
                << "Library version: " << message_types::kRiLibraryVersion;
        throw IncompatibleVersionException(message.str());
      }
      case message_types::ConnectReply::StatusCode::kSuccess:
        ri_version_ = connect_reply.ri_version;
        break;
      default:
        throw ProtocolException("libfranka: protocol error");
    }
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException(std::string{"libfranka: FRANKA connection error: "} +
                           e.what());
  } catch (Poco::TimeoutException const& e) {
    throw NetworkException("libfranka: FRANKA connection timeout");
  } catch (Poco::Exception const& e) {
    throw NetworkException(std::string{"libfranka: "} + e.what());
  }
}

Robot::Impl::~Impl() {
  tcp_socket_.shutdown();
  tcp_socket_.close();
  udp_socket_.close();
}

bool Robot::Impl::waitForRobotState() {
  try {
    Poco::Net::SocketAddress server_address;
    int bytes_received = udp_socket_.receiveFrom(
        &robot_state_, sizeof(robot_state_), server_address, 0);
    if (bytes_received == sizeof(robot_state_)) {
      return true;
    }
    if (bytes_received == 0) {
      return false;
    }
    throw ProtocolException("libfranka:: incorrect object size");
  } catch (Poco::TimeoutException const& e) {
    throw NetworkException("libfranka: robot state read timeout");
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException(std::string{"libfranka: robot state read: "} +
                           e.what());
  }
}

const RobotState& Robot::Impl::robotState() const {
  return robot_state_;
}

Robot::ServerVersion Robot::Impl::serverVersion() const {
  return ri_version_;
}

template <class T>
void Robot::Impl::tcpReceiveObject(T& object) {
  int bytes_read = 0;
  try {
    uint8_t* buff = reinterpret_cast<uint8_t*>(&object);
    constexpr int kBytesTotal = sizeof(T);

    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_.receiveBytes(buff + bytes_read, bytes_left, 0);
      if (rv == 0) {
        throw NetworkException("libfranka:: FRANKA connection closed");
      }
      bytes_read += rv;
    }

  } catch (Poco::Net::NetException const& e) {
    throw NetworkException(std::string{"libfranka: FRANKA connection error: "} +
                           e.what());
  } catch (Poco::TimeoutException const& e) {
    if (bytes_read != 0) {
      throw ProtocolException("libfranka:: incorrect object size");
    } else {
      throw NetworkException("libfranka: FRANKA connection timeout");
    }
  }
}

}  // namespace franka
