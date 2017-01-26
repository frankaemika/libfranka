#include <franka/robot.h>

#include <boost/asio.hpp>
#include <iostream>

#include "blocking_read_write.h"
#include "blocking_read_write.h"
#include "message_types.h"

namespace franka {

class Robot::Impl {
 public:
  explicit Impl(const std::string& frankaAddress);

  bool waitForRobotState();
  const RobotState& getRobotState() const;
  ServerVersion getServerVersion() const;

 private:
  const std::string franka_port_tcp_;
  const uint16_t kRiLibraryVersion;

  uint16_t ri_version_;
  RobotState robot_state_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket tcp_socket_;
  std::unique_ptr<boost::asio::ip::udp::socket> udp_socket_;
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
    : franka_port_tcp_{"1337"},
      kRiLibraryVersion{1},
      ri_version_{0},
      robot_state_{},
      io_service_{},
      tcp_socket_{io_service_},
      udp_socket_{nullptr} {
  using boost_tcp = boost::asio::ip::tcp;
  using boost_udp = boost::asio::ip::udp;

  boost_tcp::resolver resolver(io_service_);
  boost_tcp::resolver::query query(frankaAddress, franka_port_tcp_);

  try {
    boost_tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    // TODO: timeout on connect
    boost::asio::connect(tcp_socket_, endpoint_iterator);

    auto endpoint = boost_udp::endpoint(boost_udp::v4(), 0);
    udp_socket_.reset(new boost_udp::socket(io_service_, endpoint));

    message_types::ConnectRequest connect_request;
    connect_request.function_id = message_types::FunctionId::kConnect;
    connect_request.ri_library_version = kRiLibraryVersion;
    connect_request.udp_port = udp_socket_->local_endpoint().port();

    blockingWrite(io_service_, tcp_socket_, &connect_request,
                  sizeof(connect_request), std::chrono::seconds(5));

    message_types::ConnectReply connect_reply;
    blockingRead(io_service_, tcp_socket_, &connect_reply,
                 sizeof(connect_reply), std::chrono::seconds(5));
    if (connect_reply.status_code != message_types::ConnectReply::StatusCode::kSuccess) {
      if (connect_reply.status_code ==
          message_types::ConnectReply::StatusCode::kIncompatibleLibraryVersion) {
        throw IncompatibleVersionException(
            "libfranka: incompatible library version");
      }
      throw ProtocolException("libfranka: protocol error");
    }
    ri_version_ = connect_reply.ri_version;
  } catch (boost::system::system_error const& e) {
    throw NetworkException(std::string{"libfranka: "} + e.what());
  }
}

bool Robot::Impl::waitForRobotState() {
  try {
    blockingReceiveBytes(io_service_, *udp_socket_, &robot_state_,
                         sizeof(robot_state_), std::chrono::seconds(20));
    return true;
  } catch (boost::system::system_error const& e) {
    if (e.code() == boost::asio::error::eof) {
      return false;
    } else {
      throw NetworkException(std::string{"libfranka: "} + e.what());
    }
  }
}

const RobotState& Robot::Impl::getRobotState() const {
  return robot_state_;
}

Robot::ServerVersion Robot::Impl::getServerVersion() const {
  return ri_version_;
}

}  // namespace franka
