#include <franka/robot.h>

#include <boost/asio.hpp>

#include "network/blocking_read_write.h"
#include "robot_service/messages.h"

namespace franka {

class Robot::Impl {
 public:
  explicit Impl(const std::string& frankaAddress);

  bool waitForRobotState();
  const RobotState& getRobotState() const;

 private:
  const std::string franka_port_tcp_;

  uint16_t ri_version_;
  const uint16_t kRiLibraryVersion;

  RobotState robot_state_;

  std::shared_ptr<boost::asio::io_service> io_service_;

  std::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_;
  BlockingReadWrite<boost::asio::ip::tcp::socket> tcp_operations_;
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

NetworkException::NetworkException(std::string const& message)
    : std::runtime_error(message) {}

ProtocolException::ProtocolException(std::string const& message)
    : std::runtime_error(message) {}
/* Implementation */

Robot::Impl::Impl(const std::string& frankaAddress)
    : franka_port_tcp_{"1337"},
      ri_version_{0},
      kRiLibraryVersion{1},
      robot_state_{},
      io_service_{new boost::asio::io_service(0)},
      tcp_socket_{new boost::asio::ip::tcp::socket(*io_service_)},
      tcp_operations_{io_service_, tcp_socket_},
      udp_socket_{}{
  using boost_tcp = boost::asio::ip::tcp;
  using boost_udp = boost::asio::ip::udp;

  boost_tcp::resolver resolver(*io_service_);
  boost_tcp::resolver::query query(frankaAddress, franka_port_tcp_);

  try {
    boost_tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    // TODO: timeout on connect
    boost::asio::connect(*tcp_socket_, endpoint_iterator);

    auto endpoint = boost_udp::endpoint(boost_udp::v4(), 0);
    udp_socket_.reset(new boost_udp::socket(*io_service_, endpoint));
    boost_udp::endpoint local_endpoint = udp_socket_->local_endpoint();
    unsigned short udp_port = local_endpoint.port();

    robot_service::RIConnectRequest connect_request;
    connect_request.function_id = robot_service::RIFunctionId::kConnect;
    connect_request.ri_library_version = kRiLibraryVersion;
    connect_request.udp_port = udp_port;

    const unsigned char* request_data =
        reinterpret_cast<const unsigned char*>(&connect_request);

    tcp_operations_.write(request_data, sizeof(connect_request),
                          boost::posix_time::seconds(4));

    // TODO: more specific error checking
    robot_service::RIConnectReply* connect_reply;

    // TODO: catch the timeout, if something was received, then versions are incompatible
    std::vector<unsigned char> data = tcp_operations_.receive(
        sizeof(connect_reply), boost::posix_time::seconds(5));

    connect_reply = reinterpret_cast<robot_service::RIConnectReply*>(data.data());

    if(connect_reply->status_code != robot_service::StatusCode::kSuccess) {
      if(connect_reply->status_code == robot_service::StatusCode::kIncompatibleLibraryVersion)
      {
        throw ProtocolException("libfranka: incompatible library version");
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

}  // namespace franka
