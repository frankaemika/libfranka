#include "robot_impl.h"

#include <sstream>

#include <Poco/Net/NetException.h>
#include <cstring>
#include <iostream>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

constexpr std::chrono::seconds Robot::Impl::kDefaultTimeout;

Robot::Impl::Impl(const std::string& franka_address,
                  uint16_t franka_port,
                  std::chrono::milliseconds timeout,
                  RealtimeConfig realtime_config)
    : realtime_config_{realtime_config},
      motion_generator_running_{false},
      controller_running_{false} {
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
        tcpReceiveObject<research_interface::Function::kConnect,
                         research_interface::ConnectReply>();
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
  try {
    tcp_socket_.shutdown();
  } catch (...) {
  }
}

bool Robot::Impl::update() {
  try {
    if (!handleReplies()) {
      return false;  // server sent EOF
    }
  } catch (const Poco::Net::NetException& e) {
    throw NetworkException("libfranka: control connection read: "s + e.what());
  }

  Poco::Net::SocketAddress server_address;
  try {
    std::array<uint8_t, sizeof(research_interface::RobotState)> buffer;
    int bytes_received =
        udp_socket_.receiveFrom(buffer.data(), buffer.size(), server_address);
    if (bytes_received != buffer.size()) {
      throw ProtocolException("libfranka: incorrect object size");
    }
    robot_state_ =
        *reinterpret_cast<research_interface::RobotState*>(buffer.data());
  } catch (const Poco::TimeoutException& e) {
    throw NetworkException("libfranka: robot state read timeout");
  } catch (const Poco::Net::NetException& e) {
    throw NetworkException("libfranka: robot state read: "s + e.what());
  }

  if (motion_generator_running_) {
    robot_command_.message_id = robot_state_.rcuRobotState().message_id;

    try {
      int bytes_sent = udp_socket_.sendTo(
          &robot_command_, sizeof(robot_command_), server_address);
      if (bytes_sent != sizeof(robot_command_)) {
        throw NetworkException("libfranka: robot command send error");
      }
    } catch (Poco::Net::NetException const& e) {
      throw NetworkException("libfranka: robot command send: "s + e.what());
    }
  }
  return true;
}

const RobotState& Robot::Impl::robotState() const noexcept {
  return robot_state_.robotState();
}

Robot::ServerVersion Robot::Impl::serverVersion() const noexcept {
  return ri_version_;
}

bool Robot::Impl::motionGeneratorRunning() const noexcept {
  return motion_generator_running_;
}

RealtimeConfig Robot::Impl::realtimeConfig() const noexcept {
  return realtime_config_;
}

research_interface::MotionGeneratorCommand&
Robot::Impl::motionGeneratorCommand() noexcept {
  return robot_command_.motion;
}

research_interface::ControllerCommand&
Robot::Impl::controllerCommand() noexcept {
  return robot_command_.control;
}

bool Robot::Impl::handleReplies() {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    size_t offset = read_buffer_.size();
    read_buffer_.resize(offset + tcp_socket_.available());
    int rv = tcp_socket_.receiveBytes(&read_buffer_[offset],
                                      tcp_socket_.available());
    if (rv == 0) {
      return false;
    }

    if (read_buffer_.size() < sizeof(research_interface::Function)) {
      return true;
    }

    research_interface::Function function =
        *reinterpret_cast<research_interface::Function*>(read_buffer_.data());

    if (expected_replies_.find(function) == expected_replies_.end()) {
      throw ProtocolException("libfranka: unexpected reply!");
    }
    switch (function) {
      case research_interface::Function::kStartMotionGenerator:
        handleReply<research_interface::StartMotionGeneratorReply>(
            std::bind(&Robot::Impl::handleStartMotionGeneratorReply, this,
                      std::placeholders::_1));
        break;
      case research_interface::Function::kStopMotionGenerator:
        handleReply<research_interface::StopMotionGeneratorReply>(
            std::bind(&Robot::Impl::handleStopMotionGeneratorReply, this,
                      std::placeholders::_1));
        break;
      case research_interface::Function::kStartController:
        handleReply<research_interface::StartControllerReply>(
            std::bind(&Robot::Impl::handleStartControllerReply, this,
                      std::placeholders::_1));
        break;
      case research_interface::Function::kStopController:
        handleReply<research_interface::StopControllerReply>(
            std::bind(&Robot::Impl::handleStopControllerReply, this,
                      std::placeholders::_1));
        break;
      default:
        throw ProtocolException("libfranka: unsupported reply!");
    }
  }
  return true;
}

template <typename T>
void Robot::Impl::handleReply(std::function<void(T)> handle) {
  if (read_buffer_.size() < sizeof(T)) {
    return;
  }

  T reply = *reinterpret_cast<T*>(read_buffer_.data());

  size_t remaining_bytes = read_buffer_.size() - sizeof(reply);
  std::memmove(read_buffer_.data(), &read_buffer_[sizeof(reply)],
               remaining_bytes);
  read_buffer_.resize(remaining_bytes);

  expected_replies_.erase(reply.function);
  handle(reply);
}

template <research_interface::Function F, typename T>
T Robot::Impl::tcpReceiveObject() {
  int bytes_read = 0;
  try {
    std::array<uint8_t, sizeof(T)> buffer;
    constexpr int kBytesTotal = sizeof(T);

    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_.receiveBytes(&buffer.at(bytes_read), bytes_left);
      if (rv == 0) {
        throw NetworkException("libfranka: FRANKA connection closed");
      }
      bytes_read += rv;
    }
    if (*reinterpret_cast<research_interface::Function*>(buffer.data()) != F) {
      throw ProtocolException("libfranka: received reply of wrong type.");
    }
    return *reinterpret_cast<const T*>(buffer.data());
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
    throw ControlException(
        "libfranka: attempted to start multiple motion generators!");
  }
  std::memset(&robot_command_, 0, sizeof(robot_command_));

  research_interface::StartMotionGeneratorRequest request(
      motion_generator_type);
  tcp_socket_.sendBytes(&request, sizeof(request));

  research_interface::StartMotionGeneratorReply motion_generator_reply =
      tcpReceiveObject<research_interface::Function::kStartMotionGenerator,
                       research_interface::StartMotionGeneratorReply>();

  switch (motion_generator_reply.status) {
    case research_interface::StartMotionGeneratorReply::Status::kSuccess:
      break;
    case research_interface::StartMotionGeneratorReply::Status::kNotConnected:
      throw ProtocolException(
          "libfranka: attempted to start motion generator, but not connected!");
    case research_interface::StartMotionGeneratorReply::Status::kInvalidType:
    default:
      throw ProtocolException("libfranka: unexpected motion generator reply!");
  }

  expected_replies_.insert(research_interface::Function::kStartMotionGenerator);

  research_interface::MotionGeneratorMode motion_generator_mode;
  switch (motion_generator_type) {
    case decltype(motion_generator_type)::kJointPosition:
      motion_generator_mode = decltype(motion_generator_mode)::kJointPosition;
      break;
    case decltype(motion_generator_type)::kJointVelocity:
      motion_generator_mode = decltype(motion_generator_mode)::kJointVelocity;
      break;
    case decltype(motion_generator_type)::kCartesianPosition:
      motion_generator_mode =
          decltype(motion_generator_mode)::kCartesianPosition;
      break;
    case decltype(motion_generator_type)::kCartesianVelocity:
      motion_generator_mode =
          decltype(motion_generator_mode)::kCartesianVelocity;
      break;
    default:
      throw std::runtime_error(
          "No matching research_interface::MotionGeneratorMode for the "
          "research_interface::StartMotionGeneratorRequest::Type");
  }

  while (update()) {
    if (robot_state_.rcuRobotState().motion_generator_mode ==
        motion_generator_mode) {
      motion_generator_running_ = true;
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::stopMotionGenerator() {
  if (!motion_generator_running_) {
    return;
  }

  research_interface::StopMotionGeneratorRequest request;
  tcp_socket_.sendBytes(&request, sizeof(request));

  expected_replies_.insert(research_interface::Function::kStopMotionGenerator);

  robot_command_.motion.motion_generation_finished = true;
  while (update()) {
    if (robot_state_.rcuRobotState().motion_generator_mode ==
        research_interface::MotionGeneratorMode::kIdle) {
      motion_generator_running_ = false;
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::startController() {
  research_interface::StartControllerRequest request;
  tcp_socket_.sendBytes(&request, sizeof(request));

  expected_replies_.insert(research_interface::Function::kStartController);

  while (update()) {
    if (robot_state_.rcuRobotState().controller_mode ==
        research_interface::ControllerMode::kExternalController) {
      controller_running_ = true;
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::stopController() {
  research_interface::StopControllerRequest request;
  tcp_socket_.sendBytes(&request, sizeof(request));

  expected_replies_.insert(research_interface::Function::kStopController);

  while (update()) {
    if (robot_state_.rcuRobotState().controller_mode !=
        research_interface::ControllerMode::kExternalController) {
      controller_running_ = false;
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::handleStartMotionGeneratorReply(
    const research_interface::StartMotionGeneratorReply& reply) {
  motion_generator_running_ = false;
  switch (reply.status) {
    case research_interface::StartMotionGeneratorReply::Status::kFinished:
      break;
    case research_interface::StartMotionGeneratorReply::Status::kAborted:
      throw ControlException("libfranka: motion generator command aborted!");
    case research_interface::StartMotionGeneratorReply::Status::kRejected:
      throw ControlException("libfranka: motion generator command rejected!");
    default:
      throw ProtocolException(
          "libfranka: unexpected start motion generator reply!");
  }
}

void Robot::Impl::handleStopMotionGeneratorReply(
    const research_interface::StopMotionGeneratorReply& reply) {
  if (reply.status !=
      research_interface::StopMotionGeneratorReply::Status::kSuccess) {
    throw ProtocolException(
        "libfranka: unexpected stop motion generator reply!");
  }
}

void Robot::Impl::handleStartControllerReply(
    const research_interface::StartControllerReply& reply) {
  if (reply.status !=
      research_interface::StartControllerReply::Status::kSuccess) {
    throw ProtocolException(
        "libfranka: unexpected stop motion generator reply!");
  }
}
void Robot::Impl::handleStopControllerReply(
    const research_interface::StopControllerReply& reply) {
  if (reply.status !=
      research_interface::StopControllerReply::Status::kSuccess) {
    throw ProtocolException(
        "libfranka: unexpected stop motion generator reply!");
  }
}

}  // namespace franka
