#include "robot_impl.h"

#include <chrono>
#include <sstream>

#include <Poco/Net/NetException.h>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT

namespace franka {

constexpr std::chrono::seconds Robot::Impl::kDefaultTimeout;

Robot::Impl::Impl(const std::string& franka_address,
                  uint16_t franka_port,
                  std::chrono::milliseconds timeout)
    : motion_generator_running_{false} {
  Poco::Timespan poco_timeout(1000l * timeout.count());
  try {
    tcp_socket_.connect({franka_address, franka_port}, poco_timeout);
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(poco_timeout);
    tcp_socket_.setReceiveTimeout(poco_timeout);

    udp_socket_.setReceiveTimeout(poco_timeout);
    udp_socket_.bind({"0.0.0.0", 0});

    research_interface::ConnectRequest connect_request(
        udp_socket_.address().port());

    tcp_socket_.sendBytes(&connect_request, sizeof(connect_request));

    research_interface::ConnectReply connect_reply =
        tcpReceiveObject<research_interface::ConnectReply>();
    switch (connect_reply.status) {
      case research_interface::ConnectReply::Status::
          kIncompatibleLibraryVersion: {
        std::stringstream message;
        message << "libfranka: incompatible library version. "
                << "Server version: " << connect_reply.version
                << "Library version: " << research_interface::kVersion;
        throw IncompatibleVersionException(message.str());
      }
      case research_interface::ConnectReply::Status::kSuccess:
        ri_version_ = connect_reply.version;
        break;
      default:
        throw ProtocolException("libfranka: protocol error");
    }
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
  } catch (Poco::TimeoutException const& e) {
    throw NetworkException("libfranka: FRANKA connection timeout");
  } catch (Poco::Exception const& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

Robot::Impl::~Impl() noexcept {
  tcp_socket_.shutdown();
  tcp_socket_.close();
  udp_socket_.close();
}

void Robot::Impl::setRobotState(
    const research_interface::RobotState& robot_state) {
  static_assert(sizeof(robot_state_) == sizeof(robot_state),
                "research_interface::RobotState size changed - adjust "
                "franka::RobotState?");
  robot_state_.message_id = robot_state.message_id;
  std::copy(robot_state.q_start.cbegin(), robot_state.q_start.cend(),
            robot_state_.q_start.begin());
  std::copy(robot_state.O_T_EE_start.cbegin(), robot_state.O_T_EE_start.cend(),
            robot_state_.O_T_EE_start.begin());
  std::copy(robot_state.elbow_start.cbegin(), robot_state.elbow_start.cend(),
            robot_state_.elbow_start.begin());
  std::copy(robot_state.tau_J.cbegin(), robot_state.tau_J.cend(),
            robot_state_.tau_J.begin());
  std::copy(robot_state.dtau_J.cbegin(), robot_state.dtau_J.cend(),
            robot_state_.dtau_J.begin());
  std::copy(robot_state.q.cbegin(), robot_state.q.cend(),
            robot_state_.q.begin());
  std::copy(robot_state.dq.cbegin(), robot_state.dq.cend(),
            robot_state_.dq.begin());
  std::copy(robot_state.q_d.cbegin(), robot_state.q_d.cend(),
            robot_state_.q_d.begin());
  std::copy(robot_state.joint_contact.cbegin(),
            robot_state.joint_contact.cend(),
            robot_state_.joint_contact.begin());
  std::copy(robot_state.cartesian_contact.cbegin(),
            robot_state.cartesian_contact.cend(),
            robot_state_.cartesian_contact.begin());
  std::copy(robot_state.joint_collision.cbegin(),
            robot_state.joint_collision.cend(),
            robot_state_.joint_collision.begin());
  std::copy(robot_state.cartesian_collision.cbegin(),
            robot_state.cartesian_collision.cend(),
            robot_state_.cartesian_collision.begin());
  std::copy(robot_state.tau_ext_hat_filtered.cbegin(),
            robot_state.tau_ext_hat_filtered.cend(),
            robot_state_.tau_ext_hat_filtered.begin());
  std::copy(robot_state.O_F_ext_hat_K.cbegin(),
            robot_state.O_F_ext_hat_K.cend(),
            robot_state_.O_F_ext_hat_K.begin());
  std::copy(robot_state.K_F_ext_hat_K.cbegin(),
            robot_state.K_F_ext_hat_K.cend(),
            robot_state_.K_F_ext_hat_K.begin());
}

bool Robot::Impl::update() {
  try {
    if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
      // The current server implementation does not send any data
      // to libfranka. Therefore, any message on the socket indicates
      // that the connection has been closed (server sent EOF).
      return false;
    }

    std::array<uint8_t, sizeof(research_interface::RobotState)> buffer;
    Poco::Net::SocketAddress server_address;
    int bytes_received = udp_socket_.receiveFrom(buffer.data(), buffer.size(),
                                                 server_address, 0);
    if (bytes_received == buffer.size()) {
      research_interface::RobotState robot_state(
          *reinterpret_cast<research_interface::RobotState*>(buffer.data()));
      setRobotState(robot_state);
      return true;
    }
    throw ProtocolException("libfranka:: incorrect object size");
  } catch (Poco::TimeoutException const& e) {
    throw NetworkException("libfranka: robot state read timeout");
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: robot state read: "s + e.what());
  }
}

const RobotState& Robot::Impl::robotState() const noexcept {
  return robot_state_;
}

Robot::ServerVersion Robot::Impl::serverVersion() const noexcept {
  return ri_version_;
}

research_interface::MotionGeneratorCommand&
Robot::Impl::motionCommand() noexcept {
  return robot_command_.motion;
}

template <class T>
T Robot::Impl::tcpReceiveObject() {
  int bytes_read = 0;
  try {
    std::array<uint8_t, sizeof(T)> buffer;
    constexpr int kBytesTotal = sizeof(T);

    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_.receiveBytes(&buffer.at(bytes_read), bytes_left, 0);
      if (rv == 0) {
        throw NetworkException("libfranka:: FRANKA connection closed");
      }
      bytes_read += rv;
    }
    return T(*reinterpret_cast<const T*>(buffer.data()));
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
  } catch (Poco::TimeoutException const& e) {
    if (bytes_read != 0) {
      throw ProtocolException("libfranka:: incorrect object size");
    } else {
      throw NetworkException("libfranka: FRANKA connection timeout");
    }
  }
}

void Robot::Impl::startMotionGenerator() {
  if (motion_generator_running_) {
    throw MotionGeneratorException(
        "libfranka:: Attempt to start multiple motion generators!");
  }
  motion_generator_running_ = true;
}

void Robot::Impl::stopMotionGenerator() {
  motion_generator_running_ = false;
}

}  // namespace franka
