#include <franka/robot.h>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>
#include <chrono>
#include <iostream>
#include <sstream>

#include "message_types.h"

namespace franka {
constexpr uint16_t kFrankaPortTcp = 1337;
constexpr uint16_t kRiLibraryVersion = 1;
constexpr std::chrono::seconds kTimeout{5};

class Robot::Impl {
 public:
  explicit Impl(const std::string& franka_address);
  ~Impl();

  bool waitForRobotState();
  const RobotState& robotState() const;
  ServerVersion serverVersion() const;

 protected:
  // Can throw NetworkException and ProtocolException
  template <class T>
  void tcpReceiveObject(T& object);

 private:
  uint16_t ri_version_;
  RobotState robot_state_;

  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
};

Robot::Robot(const std::string& franka_address)
    : impl_(new Robot::Impl(franka_address)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() = default;

bool Robot::waitForRobotState() {
  return impl_->waitForRobotState();
}

const RobotState& Robot::robotState() const {
  return impl_->robotState();
}

Robot::ServerVersion Robot::serverVersion() const {
  return impl_->serverVersion();
}

NetworkException::NetworkException(std::string const& message)
    : std::runtime_error(message) {}

ProtocolException::ProtocolException(std::string const& message)
    : std::runtime_error(message) {}

IncompatibleVersionException::IncompatibleVersionException(
    std::string const& message)
    : std::runtime_error(message) {}

/* Implementation */

Robot::Impl::Impl(const std::string& franka_address) : ri_version_{0} {
  try {
    tcp_socket_.connect({franka_address, kFrankaPortTcp},
                        Poco::Timespan(kTimeout.count(), 0));
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(Poco::Timespan(kTimeout.count(), 0));
    tcp_socket_.setReceiveTimeout(Poco::Timespan(kTimeout.count(), 0));

    udp_socket_.setReceiveTimeout(Poco::Timespan(kTimeout.count(), 0));
    udp_socket_.bind({"0.0.0.0", 0});

    message_types::ConnectRequest connect_request;
    connect_request.function_id = message_types::FunctionId::kConnect;
    connect_request.ri_library_version = kRiLibraryVersion;
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
                << "Library version: " << kRiLibraryVersion;
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
    uint8_t* buff = reinterpret_cast<uint8_t*>(&object);  // NOLINT
    constexpr int kBytesTotal = sizeof(T);

    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_.receiveBytes(buff + bytes_read,  // NOLINT
                                        bytes_left, 0);
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
