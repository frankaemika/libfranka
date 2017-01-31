#include <franka/robot.h>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>
#include <chrono>
#include <iostream>
#include <sstream>

#include "message_types.h"

namespace franka {

class Robot::Impl {
 public:
  explicit Impl(const std::string& frankaAddress);
  ~Impl();

  bool waitForRobotState();
  const RobotState& getRobotState() const;
  ServerVersion getServerVersion() const;

 protected:
  // Can throw NetworkException and ProtocolException
  template <class T>
  void tcpReceiveObject(T& object);

 private:
  const uint16_t kFranka_port_tcp_;
  const uint16_t kRiLibraryVersion_;
  const std::chrono::seconds kTimeout_;

  uint16_t ri_version_;
  RobotState robot_state_;

  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
};

Robot::Robot(const std::string& frankaAddress)
    : impl_(new Robot::Impl(frankaAddress)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() = default;

bool Robot::waitForRobotState() {
  return impl_->waitForRobotState();
}

const RobotState& Robot::getRobotState() const {
  return impl_->getRobotState();
}

Robot::ServerVersion Robot::getServerVersion() const {
  return impl_->getServerVersion();
}

NetworkException::NetworkException(std::string const& message)
    : std::runtime_error(message) {}

ProtocolException::ProtocolException(std::string const& message)
    : std::runtime_error(message) {}

IncompatibleVersionException::IncompatibleVersionException(
    std::string const& message)
    : std::runtime_error(message) {}

/* Implementation */

Robot::Impl::Impl(const std::string& frankaAddress)
    : kFranka_port_tcp_{1337},
      kRiLibraryVersion_{1},
      kTimeout_{5},
      ri_version_{0},
      robot_state_{},
      tcp_socket_{},
      udp_socket_{} {
  try {
    tcp_socket_.connect({frankaAddress, kFranka_port_tcp_},
                        Poco::Timespan(kTimeout_.count(), 0));
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(Poco::Timespan(kTimeout_.count(), 0));
    tcp_socket_.setReceiveTimeout(Poco::Timespan(kTimeout_.count(), 0));

    udp_socket_.setReceiveTimeout(Poco::Timespan(kTimeout_.count(), 0));
    udp_socket_.bind({"0.0.0.0", 0});

    message_types::ConnectRequest connect_request;
    connect_request.function_id = message_types::FunctionId::kConnect;
    connect_request.ri_library_version = kRiLibraryVersion_;
    connect_request.udp_port = udp_socket_.address().port();

    tcp_socket_.sendBytes(&connect_request, sizeof(connect_request));

    message_types::ConnectReply connect_reply;
    tcpReceiveObject(connect_reply);

    if (connect_reply.status_code !=
        message_types::ConnectReply::StatusCode::kSuccess) {
      if (connect_reply.status_code == message_types::ConnectReply::StatusCode::
                                           kIncompatibleLibraryVersion) {
        std::stringstream message;
        message << "libfranka: incompatible library version. "
                << "Server version: " << connect_reply.ri_version
                << "Library version: " << kRiLibraryVersion_;
        throw IncompatibleVersionException(message.str());
      }
      throw ProtocolException("libfranka: protocol error");
    }
    ri_version_ = connect_reply.ri_version;
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

const RobotState& Robot::Impl::getRobotState() const {
  return robot_state_;
}

Robot::ServerVersion Robot::Impl::getServerVersion() const {
  return ri_version_;
}

template <class T>
void Robot::Impl::tcpReceiveObject(T& object) {
  int bytes_read = 0;
  try {
    uint8_t* buff = reinterpret_cast<uint8_t*>(&object);  // NOLINT
    constexpr int bytes_total = sizeof(T);

    while (bytes_read < bytes_total) {
      int bytes_left = bytes_total - bytes_read;
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
