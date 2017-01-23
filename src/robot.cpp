#include <franka/robot.h>

#include <boost/asio.hpp>

#include "robot_service/messages.h"

namespace franka {

class Robot::Impl {
 public:
  explicit Impl(const std::string& frankaAddress);

  bool waitForRobotState();
  const RobotState& getRobotState() const;

 private:
  const std::string franka_port_tcp_;

  RobotState robot_state_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket tcp_socket_;
  std::unique_ptr<boost::asio::ip::udp::socket>  udp_socket_;
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

NetworkException::NetworkException(std::string const& message)
    : std::runtime_error(message) {}

/* Implementation */

Robot::Impl::Impl(const std::string& frankaAddress)
    : franka_port_tcp_{"8080"},
      robot_state_{},
      io_service_{},
      tcp_socket_{io_service_},
      udp_socket_{}{
  using boost_tcp = boost::asio::ip::tcp;
  using boost_udp = boost::asio::ip::udp;

  boost_tcp::resolver resolver(io_service_);
  boost_tcp::resolver::query query(frankaAddress, franka_port_tcp_);

  try {
    boost_tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::connect(tcp_socket_, endpoint_iterator);

    auto endpoint = boost_udp::endpoint(boost_udp::v4(), 0);
    udp_socket_.reset(new boost_udp::socket(io_service_, endpoint));
    boost_udp::endpoint local_endpoint = udp_socket_->local_endpoint();
    unsigned short udp_port = local_endpoint.port();

    robot_service::RIConnectRequest connect_request;
    connect_request.function_id = robot_service::RIFunctionId::kConnect;
    connect_request.ri_library_version = 1;
    connect_request.udp_port = udp_port;

    const char *request_data = reinterpret_cast<const char*>(&connect_request);
    boost::asio::write(tcp_socket_, boost::asio::buffer(request_data, sizeof(connect_request)));

    // TODO: more specific error checking
    robot_service::RIConnectReply connect_reply;
    char *reply_data = reinterpret_cast<char*>(&connect_reply);
    auto buffer = boost::asio::buffer(reply_data, sizeof(connect_reply));
    boost::asio::read(tcp_socket_, buffer);

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

}  // namespace franka
