#include "robot_impl.h"

#include <algorithm>
#include <sstream>

#include <Poco/Net/NetException.h>
#include <cstring>

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
        message << "libfranka: incompatible library version. " << std::endl
                << "Server version: " << connect_reply.version << std::endl
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
    if (!handleReplies()) {
      return false;  // server sent EOF
    }

    std::array<uint8_t, sizeof(research_interface::RobotState)> buffer;
    Poco::Net::SocketAddress server_address;
    int bytes_received = udp_socket_.receiveFrom(buffer.data(), buffer.size(),
                                                 server_address);
    if (bytes_received == buffer.size()) {
      research_interface::RobotState robot_state(
          *reinterpret_cast<research_interface::RobotState*>(buffer.data()));
      setRobotState(robot_state);

      if (motion_generator_running_) {
        if (robot_command_.motion.motion_generation_finished) {
          motion_generator_running_ = false;
        }
        robot_command_.message_id = robot_state.message_id;
        robot_command_.motion.timestamp += kCommandTimeStep;

        int bytes_sent = udp_socket_.sendTo(&robot_command_,
                           sizeof(robot_command_), server_address);
        if (bytes_sent != sizeof(robot_command_)) {
          throw NetworkException("libfranka: UDP send error");
        }
      }
      return true;
    }
    throw ProtocolException("libfranka: incorrect object size");
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

bool Robot::Impl::handleReplies() {
  using namespace std::placeholders;
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    size_t offset = read_buffer_.size();
    read_buffer_.resize(offset + tcp_socket_.available());
    int rv = tcp_socket_.receiveBytes(&read_buffer_[offset],
                                      tcp_socket_.available(), MSG_DONTWAIT);
    if (rv == 0) {
      return false;
    }

    if (read_buffer_.size() < sizeof(research_interface::Function)) {
      return true;
    }

    research_interface::Function function =
        *reinterpret_cast<research_interface::Function*>(read_buffer_.data());

    auto it =
        std::find(expected_replies_.begin(), expected_replies_.end(), function);
    if (it == std::end(expected_replies_)) {
      throw ProtocolException("libfranka: unexpected reply!");
    }
    switch (function) {
      case research_interface::Function::kStartMotionGenerator:
        handleReply<research_interface::StartMotionGeneratorReply>(
            std::bind(&Robot::Impl::handleStartMotionGeneratorReply, this, _1),
            it);
        break;
      case research_interface::Function::kStopMotionGenerator:
        handleReply<research_interface::StopMotionGeneratorReply>(
            std::bind(&Robot::Impl::handleStopMotionGeneratorReply, this, _1),
            it);
        break;
      default:
        throw ProtocolException("libfranka: unsupported reply!");
    }
  }
  return true;
}

template <typename T>
void Robot::Impl::handleReply(
    std::function<void(T)> handle,
    std::list<research_interface::Function>::iterator it) {
  if (read_buffer_.size() < sizeof(T)) {
    return;
  }

  T reply = *reinterpret_cast<T *>(read_buffer_.data());

  size_t remaining_bytes = read_buffer_.size() - sizeof(reply);
  std::memmove(read_buffer_.data(), &read_buffer_[sizeof(reply)], remaining_bytes);
  read_buffer_.resize(remaining_bytes);

  expected_replies_.erase(it);
  handle(reply);
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
        throw NetworkException("libfranka: FRANKA connection closed");
      }
      bytes_read += rv;
    }
    return T(*reinterpret_cast<const T*>(buffer.data()));
  } catch (Poco::Net::NetException const& e) {
    throw NetworkException("libfranka: FRANKA connection error: "s + e.what());
  } catch (Poco::TimeoutException const& e) {
    if (bytes_read != 0) {
      throw ProtocolException("libfranka: incorrect object size");
    } else {
      throw NetworkException("libfranka: FRANKA connection timeout");
    }
  }
}

void Robot::Impl::startMotionGenerator(
    research_interface::StartMotionGeneratorRequest::Type
        motion_generator_type) {
  if (motion_generator_running_) {
    throw MotionGeneratorException(
        "libfranka: attempted to start multiple motion generators!");
  }
  motion_generator_running_ = true;
  std::memset(&robot_command_, 0, sizeof(robot_command_));

  research_interface::StartMotionGeneratorRequest request(
      motion_generator_type);
  tcp_socket_.sendBytes(&request, sizeof(request));

  research_interface::StartMotionGeneratorReply motion_generator_reply =
      tcpReceiveObject<research_interface::StartMotionGeneratorReply>();

  switch (motion_generator_reply.status) {
    case research_interface::StartMotionGeneratorReply::Status::kSuccess:
      break;
    case research_interface::StartMotionGeneratorReply::Status::kNotConnected:
      throw ProtocolException(
          "libfranka: attempted to start motion generator, but not connected!");
    case research_interface::StartMotionGeneratorReply::Status::kInvalidType:
    default:
      throw ProtocolException(
          "libfranka: unexpected motion generator reply!");
  }
  expected_replies_.push_back(
      research_interface::Function::kStartMotionGenerator);
}

void Robot::Impl::stopMotionGenerator() {
  robot_command_.motion.motion_generation_finished = true;

  research_interface::StopMotionGeneratorRequest request;
  tcp_socket_.sendBytes(&request, sizeof(request));

  expected_replies_.push_back(
      research_interface::Function::kStopMotionGenerator);
}

void Robot::Impl::handleStartMotionGeneratorReply(
    const research_interface::StartMotionGeneratorReply&
        start_motion_generator_request) {
  switch (start_motion_generator_request.status) {
    case research_interface::StartMotionGeneratorReply::Status::kFinished:
    case research_interface::StartMotionGeneratorReply::Status::kAborted:
      break;
    case research_interface::StartMotionGeneratorReply::Status::kRejected:
      throw MotionGeneratorException(
          "libfranka: start motion generator command rejected!");
    default:
      throw ProtocolException(
          "libfranka: unexpected motion generator reply!");
  }
}
void Robot::Impl::handleStopMotionGeneratorReply(
    const research_interface::StopMotionGeneratorReply&
        stop_motion_generator_reply) {
  if (stop_motion_generator_reply.status !=
      research_interface::StopMotionGeneratorReply::Status::kSuccess) {
    throw ProtocolException(
        "libfranka: unexpected motion generator reply!");
  }
}

}  // namespace franka
