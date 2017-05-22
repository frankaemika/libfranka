#include "robot_impl.h"

#include <sstream>

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
    : network_{franka_address, franka_port, timeout},
      realtime_config_{realtime_config},
      motion_generator_running_{false},
      controller_running_{false} {
  research_interface::Connect::Request connect_request(network_.udpPort());

  network_.tcpSendRequest(connect_request);

  research_interface::Connect::Response connect_response =
      network_.tcpBlockingReceiveResponse<research_interface::Connect>();
  switch (connect_response.status) {
    case research_interface::Connect::Status::kIncompatibleLibraryVersion: {
      std::stringstream message;
      message << "libfranka: incompatible library version. " << std::endl
              << "Server version: " << connect_response.version << std::endl
              << "Library version: " << research_interface::kVersion;
      throw IncompatibleVersionException(message.str());
    }
    case research_interface::Connect::Status::kSuccess:
      ri_version_ = connect_response.version;
      break;
    default:
      throw ProtocolException("libfranka: protocol error");
  }
}

template <typename T>
bool Robot::Impl::handleCommandResponse(const typename T::Response& response) {
  switch (response.status) {
    case T::Status::kSuccess:
      break;
    case T::Status::kAborted:
      throw CommandException("libfranka: "s +
                             research_interface::CommandTraits<T>::kName +
                             " command aborted!");
    case T::Status::kRejected:
      throw CommandException("libfranka: "s +
                             research_interface::CommandTraits<T>::kName +
                             " command rejected!");
    case T::Status::kPreempted:
      throw CommandException("libfranka: "s +
                             research_interface::CommandTraits<T>::kName +
                             " command preempted!");
  }
  return true;
}

template <>
bool Robot::Impl::handleCommandResponse<
    research_interface::StartMotionGenerator>(
    const research_interface::StartMotionGenerator::Response& response) {
  switch (response.status) {
    case research_interface::StartMotionGenerator::Status::kMotionStarted:
      // After sending MotionStarted, RCU will send another response in the
      // future,
      // e.g. Finished or Aborted.
      expected_responses_.insert(
          research_interface::Function::kStartMotionGenerator);
      break;
    case research_interface::StartMotionGenerator::Status::kSuccess:
      motion_generator_running_ = false;
      break;
    case research_interface::StartMotionGenerator::Status::kAborted:
      motion_generator_running_ = false;
      throw ControlException("libfranka: motion generator command aborted!");
    case research_interface::StartMotionGenerator::Status::kPreempted:
      motion_generator_running_ = false;
      throw ControlException("libfranka: motion generator command preempted!");
    case research_interface::StartMotionGenerator::Status::kRejected:
    default:
      motion_generator_running_ = false;
      throw ControlException("libfranka: motion generator command rejected!");
  }
  return true;
}

template <>
bool Robot::Impl::handleCommandResponse<research_interface::StartController>(
    const research_interface::StartController::Response& response) {
  switch (response.status) {
    case research_interface::StartController::Status::kSuccess:
      break;
    default:
      controller_running_ = false;
      throw ProtocolException(
          "libfranka: unexpected start controller response!");
  }
  return true;
}

bool Robot::Impl::update() {
  research_interface::Function function;
  if (network_.tcpReadResponse(&function)) {
    if (expected_responses_.find(function) == expected_responses_.end()) {
      throw ProtocolException("libfranka: unexpected response!");
    }

    bool handled = false;
    switch (function) {
      case research_interface::Function::kStartMotionGenerator:
        handled =
            network_.handleResponse<research_interface::StartMotionGenerator>(
                std::bind(&Robot::Impl::handleCommandResponse<
                              research_interface::StartMotionGenerator>,
                          this, std::placeholders::_1));
        break;
      case research_interface::Function::kStopMotionGenerator:
        handled =
            network_.handleResponse<research_interface::StopMotionGenerator>(
                std::bind(&Robot::Impl::handleCommandResponse<
                              research_interface::StopMotionGenerator>,
                          this, std::placeholders::_1));
        break;
      case research_interface::Function::kStartController:
        handled = network_.handleResponse<research_interface::StartController>(
            std::bind(&Robot::Impl::handleCommandResponse<
                          research_interface::StartController>,
                      this, std::placeholders::_1));
        break;
      case research_interface::Function::kStopController:
        handled = network_.handleResponse<research_interface::StopController>(
            std::bind(&Robot::Impl::handleCommandResponse<
                          research_interface::StopController>,
                      this, std::placeholders::_1));
        break;
      default:
        throw ProtocolException("libfranka: unsupported response!");
    }

    if (handled) {
      auto iterator = expected_responses_.find(function);
      if (iterator != expected_responses_.end()) {
        expected_responses_.erase(iterator);
      }
    }
  }

  robot_state_ = network_.udpReadRobotState();

  if (motion_generator_running_ || controller_running_) {
    robot_command_.message_id = robot_state_.rcuRobotState().message_id;

    network_.udpSendRobotCommand(robot_command_);
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

bool Robot::Impl::controllerRunning() const noexcept {
  return controller_running_;
}

RealtimeConfig Robot::Impl::realtimeConfig() const noexcept {
  return realtime_config_;
}

void Robot::Impl::controllerCommand(
    const research_interface::ControllerCommand& controller_command) noexcept {
  robot_command_.control = controller_command;
}

void Robot::Impl::motionGeneratorCommand(
    const research_interface::MotionGeneratorCommand&
        motion_generator_command) noexcept {
  robot_command_.motion = motion_generator_command;
}

void Robot::Impl::startMotionGenerator(
    research_interface::StartMotionGenerator::MotionGeneratorMode mode) {
  if (motion_generator_running_) {
    throw ControlException(
        "libfranka: attempted to start multiple motion generators!");
  }
  std::memset(&robot_command_, 0, sizeof(robot_command_));

  research_interface::StartMotionGenerator::Request request(mode);
  network_.tcpSendRequest(request);

  expected_responses_.insert(
      research_interface::Function::kStartMotionGenerator);

  research_interface::MotionGeneratorMode motion_generator_mode;
  switch (mode) {
    case decltype(mode)::kJointPosition:
      motion_generator_mode = decltype(motion_generator_mode)::kJointPosition;
      break;
    case decltype(mode)::kJointVelocity:
      motion_generator_mode = decltype(motion_generator_mode)::kJointVelocity;
      break;
    case decltype(mode)::kCartesianPosition:
      motion_generator_mode =
          decltype(motion_generator_mode)::kCartesianPosition;
      break;
    case decltype(mode)::kCartesianVelocity:
      motion_generator_mode =
          decltype(motion_generator_mode)::kCartesianVelocity;
      break;
    default:
      throw std::invalid_argument("Invalid motion generator mode given.");
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

  research_interface::StopMotionGenerator::Request request;
  network_.tcpSendRequest(request);

  expected_responses_.insert(
      research_interface::Function::kStopMotionGenerator);

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
  if (controller_running_) {
    throw ControlException(
        "libfranka: attempted to start multiple controllers!");
  }

  research_interface::StartController::Request request;
  network_.tcpSendRequest(request);

  expected_responses_.insert(research_interface::Function::kStartController);

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
  if (!controller_running_) {
    return;
  }

  research_interface::StopController::Request request;
  network_.tcpSendRequest(request);

  expected_responses_.insert(research_interface::Function::kStopController);

  while (update()) {
    if (robot_state_.rcuRobotState().controller_mode !=
        research_interface::ControllerMode::kExternalController) {
      controller_running_ = false;
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

}  // namespace franka
