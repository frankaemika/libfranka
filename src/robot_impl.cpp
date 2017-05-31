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
      realtime_config_{realtime_config} {
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
    case research_interface::Connect::Status::kSuccess: {
      ri_version_ = connect_response.version;
      break;
    }
    default:
      throw ProtocolException("libfranka: protocol error");
  }
}

bool Robot::Impl::update() {
  research_interface::Function function;
  if (network_.tcpReadResponse(&function)) {
    if (!motionGeneratorRunning() || function != research_interface::Function::kMove) {
      throw ProtocolException("libfranka: unexpected response!");
    }

    network_.tcpHandleResponse<research_interface::Move>(std::bind(&Robot::Impl::handleCommandResponse<research_interface::Move>, this, std::placeholders::_1));
  }

  if (motionGeneratorRunning() || controllerRunning()) {
    robot_command_.message_id = robot_state_.rcuRobotState().message_id;

    network_.udpSendRobotCommand(robot_command_);
  }

  robot_state_ = network_.udpReadRobotState();
  return true;
}

const RobotState& Robot::Impl::robotState() const noexcept {
  return robot_state_.robotState();
}

Robot::ServerVersion Robot::Impl::serverVersion() const noexcept {
  return ri_version_;
}

bool Robot::Impl::motionGeneratorRunning() const noexcept {
  return robot_state_.rcuRobotState().motion_generator_mode != research_interface::MotionGeneratorMode::kIdle;
}

bool Robot::Impl::controllerRunning() const noexcept {
  return robot_state_.rcuRobotState().controller_mode == research_interface::ControllerMode::kExternalController;
}

RealtimeConfig Robot::Impl::realtimeConfig() const noexcept {
  return realtime_config_;
}

void Robot::Impl::controllerCommand(
    const research_interface::ControllerCommand& controller_command) noexcept {
  robot_command_.control = controller_command;
}

void Robot::Impl::motionGeneratorCommand(
    const research_interface::MotionGeneratorCommand& motion_generator_command) noexcept {
  robot_command_.motion = motion_generator_command;
}

void Robot::Impl::startMotion(research_interface::Move::ControllerMode controller_mode, research_interface::Move::MotionGeneratorMode motion_generator_mode, const research_interface::Move::Deviation& maximum_path_deviation, const research_interface::Move::Deviation& maximum_goal_pose_deviation) {
  if (motionGeneratorRunning()) {
    throw ControlException("libfranka: attempted to start multiple motion generators!");
  }

  // Reset robot command
  robot_command_ = {};

  research_interface::MotionGeneratorMode robot_state_motion_generator_mode;
  switch (motion_generator_mode) {
    case decltype(motion_generator_mode)::kJointPosition:
      robot_state_motion_generator_mode = decltype(robot_state_motion_generator_mode)::kJointPosition;
      break;
    case decltype(motion_generator_mode)::kJointVelocity:
      robot_state_motion_generator_mode = decltype(robot_state_motion_generator_mode)::kJointVelocity;
      break;
    case decltype(motion_generator_mode)::kCartesianPosition:
      robot_state_motion_generator_mode = decltype(robot_state_motion_generator_mode)::kCartesianPosition;
      break;
    case decltype(motion_generator_mode)::kCartesianVelocity:
      robot_state_motion_generator_mode = decltype(robot_state_motion_generator_mode)::kCartesianVelocity;
      break;
    default:
      throw std::invalid_argument("Invalid motion generator mode given.");
  }

  std::memset(&robot_command_, 0, sizeof(robot_command_));

  executeCommand<research_interface::Move>(
    controller_mode,
    motion_generator_mode,
    maximum_path_deviation,
    maximum_goal_pose_deviation
  );

  while (update()) {
    if (robot_state_.rcuRobotState().motion_generator_mode == robot_state_motion_generator_mode) {
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::stopMotion() {
  if (!motionGeneratorRunning()) {
    return;
  }

  // TODO: never necessary?
  // executeCommand<research_interface::StopMove>();

  robot_command_.motion.motion_generation_finished = true;
  while (update()) {
    if (!motionGeneratorRunning()) {
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::startController() {
  if (controllerRunning()) {
    return;
  }

  // Reset robot command
  robot_command_ = {};

  executeCommand<research_interface::SetControllerMode>(research_interface::SetControllerMode::ControllerMode::kExternalController);

  while (update()) {
    if (controllerRunning()) {
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

void Robot::Impl::stopController() {
  if (!controllerRunning()) {
    return;
  }

  executeCommand<research_interface::SetControllerMode>(research_interface::SetControllerMode::ControllerMode::kJointImpedance);

  while (update()) {
    if (!controllerRunning()) {
      return;
    }
  }
  throw NetworkException("libfranka: connection closed by server.");
}

}  // namespace franka
