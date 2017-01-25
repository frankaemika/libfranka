#include <franka/robot.h>

#include <boost/asio.hpp>
#include <iostream>

#include "network/blocking_read_write.h"
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

  uint16_t ri_version_;
  const uint16_t kRiLibraryVersion;

  RobotState robot_state_;

  boost::asio::io_service io_service_;

  boost::asio::ip::tcp::socket tcp_socket_;
  std::shared_ptr<boost::asio::ip::udp::socket> udp_socket_;
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

IncompatibleVersionException::IncompatibleVersionException(std::string const& message)
    : std::runtime_error(message) {}

/* Implementation */

Robot::Impl::Impl(const std::string& frankaAddress)
    : franka_port_tcp_{"1337"},
      ri_version_{0},
      kRiLibraryVersion{1},
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
    boost_udp::endpoint local_endpoint = udp_socket_->local_endpoint();
    uint16_t udp_port = local_endpoint.port();

    message_types::ConnectRequest connect_request;
    connect_request.function_id = message_types::FunctionId::kConnect;
    connect_request.ri_library_version = kRiLibraryVersion;
    connect_request.udp_port = udp_port;

    const unsigned char* request_data =
        reinterpret_cast<const unsigned char*>(&connect_request);

    blockingWrite(io_service_, tcp_socket_, request_data,
                  sizeof(connect_request), std::chrono::seconds(5));

    message_types::ConnectReply* connect_reply;
    std::vector<unsigned char> data =
        blockingReadBytes(io_service_, tcp_socket_, sizeof(connect_reply),
                          std::chrono::seconds(5));

    connect_reply =
        reinterpret_cast<message_types::ConnectReply*>(data.data());

    if (connect_reply->status_code != message_types::ConnectReply::StatusCode::kSuccess) {
      if (connect_reply->status_code ==
          message_types::ConnectReply::StatusCode::kIncompatibleLibraryVersion) {
        throw IncompatibleVersionException("libfranka: incompatible library version");
      }
      throw ProtocolException("libfranka: protocol error");
    }
    ri_version_ = connect_reply->ri_version;

    std::cout << "Connection with FRANKA established" << std::endl;
  } catch (boost::system::system_error const& e) {
    throw NetworkException(std::string{"libfranka: "} + e.what());
  }
}

bool Robot::Impl::waitForRobotState() {
  return false;
}

const RobotState& Robot::Impl::getRobotState() const {
  return robot_state_;
}

Robot::ServerVersion Robot::Impl::getServerVersion() const {
  return ri_version_;
}

}  // namespace franka
