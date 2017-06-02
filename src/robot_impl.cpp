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
    : network_{franka_address, franka_port, timeout}, realtime_config_{realtime_config} {
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

RobotState Robot::Impl::update(const research_interface::MotionGeneratorCommand& motion_command,
                               const research_interface::ControllerCommand& control_command) {
  research_interface::Function function;
  if (network_.tcpReadResponse(&function)) {
    if (!motionGeneratorRunning() || function != research_interface::Function::kMove) {
      throw ProtocolException("libfranka: unexpected response!");
    }

    try {
      network_.tcpHandleResponse<research_interface::Move>(
          std::bind(&Robot::Impl::handleCommandResponse<research_interface::Move>, this,
                    std::placeholders::_1));
    } catch (const CommandException& e) {
      // Make sure that we have an up-to-date robot state that shows the stopped motion.
      research_interface::RobotState robot_state = network_.udpReadRobotState();
      motion_generator_mode_ = robot_state.motion_generator_mode;
      controller_mode_ = robot_state.controller_mode;
      message_id_ = robot_state.message_id;

      // Rethrow as control exception to be consistent with starting/stopping of motions.
      throw ControlException(e.what());
    }
  }

  if (motionGeneratorRunning() || controllerRunning()) {
    research_interface::RobotCommand robot_command{};
    robot_command.message_id = message_id_;
    robot_command.motion = motion_command;
    robot_command.control = control_command;

    network_.udpSendRobotCommand(robot_command);
  }

  research_interface::RobotState robot_state = network_.udpReadRobotState();
  motion_generator_mode_ = robot_state.motion_generator_mode;
  controller_mode_ = robot_state.controller_mode;
  message_id_ = robot_state.message_id;
  return convertRobotState(robot_state);
}

RobotState Robot::Impl::update(const research_interface::MotionGeneratorCommand& motion_command) {
  if (!motionGeneratorRunning() || controllerRunning()) {
    throw ControlException("libfranka: Inconsistent state in update(MotionGeneratorCommand).");
  }
  return update(motion_command, {});
}

RobotState Robot::Impl::update(const research_interface::ControllerCommand& control_command) {
  if (motionGeneratorRunning() || !controllerRunning()) {
    throw ControlException("libfranka: Inconsistent state in update(ControllerCommand).");
  }
  return update({}, control_command);
}

RobotState Robot::Impl::update() {
  if (motionGeneratorRunning() || controllerRunning()) {
    throw ControlException("libfranka: Inconsistent state in update().");
  }
  return update({}, {});
}

Robot::ServerVersion Robot::Impl::serverVersion() const noexcept {
  return ri_version_;
}

bool Robot::Impl::motionGeneratorRunning() const noexcept {
  return motion_generator_mode_ != research_interface::MotionGeneratorMode::kIdle;
}

bool Robot::Impl::controllerRunning() const noexcept {
  return controller_mode_ == research_interface::ControllerMode::kExternalController;
}

RealtimeConfig Robot::Impl::realtimeConfig() const noexcept {
  return realtime_config_;
}

void Robot::Impl::startMotion(
    research_interface::Move::ControllerMode controller_mode,
    research_interface::Move::MotionGeneratorMode motion_generator_mode,
    const research_interface::Move::Deviation& maximum_path_deviation,
    const research_interface::Move::Deviation& maximum_goal_pose_deviation) {
  if (motionGeneratorRunning()) {
    throw ControlException("libfranka: attempted to start multiple motion generators!");
  }

  research_interface::MotionGeneratorMode state_motion_generator_mode;
  switch (motion_generator_mode) {
    case decltype(motion_generator_mode)::kJointPosition:
      state_motion_generator_mode = decltype(state_motion_generator_mode)::kJointPosition;
      break;
    case decltype(motion_generator_mode)::kJointVelocity:
      state_motion_generator_mode = decltype(state_motion_generator_mode)::kJointVelocity;
      break;
    case decltype(motion_generator_mode)::kCartesianPosition:
      state_motion_generator_mode = decltype(state_motion_generator_mode)::kCartesianPosition;
      break;
    case decltype(motion_generator_mode)::kCartesianVelocity:
      state_motion_generator_mode = decltype(state_motion_generator_mode)::kCartesianVelocity;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid motion generator mode given.");
  }

  research_interface::ControllerMode state_controller_mode;
  switch (controller_mode) {
    case decltype(controller_mode)::kMotorPD:
      state_controller_mode = decltype(state_controller_mode)::kMotorPD;
      break;
    case decltype(controller_mode)::kJointPosition:
      state_controller_mode = decltype(state_controller_mode)::kJointPosition;
      break;
    case decltype(controller_mode)::kJointImpedance:
      state_controller_mode = decltype(state_controller_mode)::kJointImpedance;
      break;
    case decltype(controller_mode)::kCartesianImpedance:
      state_controller_mode = decltype(state_controller_mode)::kCartesianImpedance;
      break;
    case decltype(controller_mode)::kExternalController:
      state_controller_mode = decltype(state_controller_mode)::kExternalController;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid controller mode given.");
  }

  executeCommand<research_interface::Move>(controller_mode, motion_generator_mode,
                                           maximum_path_deviation, maximum_goal_pose_deviation);

  while (motion_generator_mode_ != state_motion_generator_mode ||
         controller_mode_ != state_controller_mode) {
    update({}, {});
  }
}

void Robot::Impl::stopMotion() {
  if (!motionGeneratorRunning()) {
    return;
  }

  // TODO: StopMove never necessary?
  // executeCommand<research_interface::StopMove>();

  // TODO: needs other parameters set as well (from previous motion?)?
  research_interface::MotionGeneratorCommand command{};
  command.motion_generation_finished = true;
  while (motionGeneratorRunning()) {
    update(command);
  }
}

void Robot::Impl::startController() {
  if (controllerRunning()) {
    throw ControlException("libfranka: attempted to start multiple controllers!");
    return;
  }

  executeCommand<research_interface::SetControllerMode>(
      research_interface::SetControllerMode::ControllerMode::kExternalController);

  while (!controllerRunning()) {
    update({}, {});
  }
}

void Robot::Impl::stopController() {
  if (!controllerRunning()) {
    return;
  }

  executeCommand<research_interface::SetControllerMode>(
      research_interface::SetControllerMode::ControllerMode::kJointImpedance);

  research_interface::ControllerCommand command{};
  while (controller_mode_ != research_interface::ControllerMode::kJointImpedance) {
    update(command);
  }
}

RobotState convertRobotState(const research_interface::RobotState& robot_state) noexcept {
  RobotState converted;
  converted.O_T_EE = robot_state.O_T_EE;
  converted.elbow = robot_state.elbow;
  converted.tau_J = robot_state.tau_J;
  converted.dtau_J = robot_state.dtau_J;
  converted.q = robot_state.q;
  converted.dq = robot_state.dq;
  converted.q_d = robot_state.q_d;
  converted.joint_contact = robot_state.joint_contact;
  converted.cartesian_contact = robot_state.cartesian_contact;
  converted.joint_collision = robot_state.joint_collision;
  converted.cartesian_collision = robot_state.cartesian_collision;
  converted.tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
  converted.O_F_ext_hat_K = robot_state.O_F_ext_hat_K;
  converted.K_F_ext_hat_K = robot_state.K_F_ext_hat_K;
  return converted;
}

}  // namespace franka
