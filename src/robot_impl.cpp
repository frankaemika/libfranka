#include "robot_impl.h"

#include <sstream>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Robot::Impl::Impl(std::unique_ptr<Network> network, RealtimeConfig realtime_config)
    : network_{std::move(network)}, realtime_config_{realtime_config} {
  if (!network_) {
    throw std::invalid_argument("libfranka robot: Invalid argument");
  }

  connect<research_interface::robot::Connect, research_interface::robot::kVersion>(*network_,
                                                                                   &ri_version_);

  updateState(network_->udpRead<research_interface::robot::RobotState>());
}

RobotState Robot::Impl::update(
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) {
  network_->tcpThrowIfConnectionClosed();

  bool was_running_motion_generator = motionGeneratorRunning();
  bool was_running_controller = controllerRunning();

  sendRobotCommand(motion_command, control_command);

  uint32_t previous_message_id = message_id_;
  research_interface::robot::RobotState received_robot_state = receiveRobotState();
  RobotState robot_state = convertRobotState(received_robot_state, previous_message_id);

  if (robot_state.robot_mode != RobotMode::kReady &&
      (was_running_motion_generator || was_running_controller)) {
    // Wait until robot state shows stopped motion and controller.
    while (motionGeneratorRunning() || controllerRunning()) {
      receiveRobotState();
    }

    // If a motion generator was running and the robot state shows an error,
    // we will receive a TCP response to the Move command.
    if (was_running_motion_generator) {
      try {
        handleCommandResponse<research_interface::robot::Move>(
            network_->tcpBlockingReceiveResponse<research_interface::robot::Move>());
      } catch (const CommandException& e) {
        // Rethrow as control exception to be consistent with starting/stopping of motions.
        if (robot_state.robot_mode == RobotMode::kReflex) {
          throw ControlException(e.what() + " "s +
                                 static_cast<std::string>(robot_state.last_motion_errors));
        }
        throw ControlException(e.what());
      }
    }

    if (robot_state.last_motion_errors) {
      throw ControlException("libfranka robot: control aborted with error: " +
                             static_cast<std::string>(robot_state.last_motion_errors));
    }
    throw ControlException("libfranka robot: control aborted");
  }

  return robot_state;
}

RobotState Robot::Impl::readOnce() {
  // Delete old data from the UDP buffer.
  while (network_->udpAvailableData() > 0) {
    network_->udpRead<research_interface::robot::RobotState>();
  }

  uint32_t previous_message_id = message_id_;
  research_interface::robot::RobotState received_robot_state = receiveRobotState();
  return convertRobotState(received_robot_state, previous_message_id);
}

void Robot::Impl::sendRobotCommand(
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) const {
  if (motion_command != nullptr || control_command != nullptr) {
    research_interface::robot::RobotCommand robot_command{};
    robot_command.message_id = message_id_;
    if (motion_command != nullptr) {
      if (!motionGeneratorRunning()) {
        // Happens for example if guiding button is pressed during motion.
        throw ControlException(
            "libfranka robot: Trying to send motion command, but no motion generator running!");
      }
      robot_command.motion = *motion_command;
    }
    if (control_command != nullptr) {
      if (!controllerRunning()) {
        throw ControlException(
            "libfranka robot: Trying to send control command, but no controller running!");
      }
      robot_command.control = *control_command;
    }

    if (motionGeneratorRunning() && controllerRunning() &&
        (motion_command == nullptr || control_command == nullptr)) {
      throw ControlException("libfranka robot: Trying to send partial robot command!");
    }

    network_->udpSend<research_interface::robot::RobotCommand>(robot_command);
  }
}

research_interface::robot::RobotState Robot::Impl::receiveRobotState() {
  research_interface::robot::RobotState latest_accepted_state;
  latest_accepted_state.message_id = message_id_;

  // If states are already available on the socket, use the one with the most recent message ID.
  // If there was no valid state on the socket, we need to wait.
  while (network_->udpAvailableData() >=
             static_cast<int>(sizeof(research_interface::robot::RobotState)) ||
         latest_accepted_state.message_id == message_id_) {
    research_interface::robot::RobotState received_state =
        network_->udpRead<research_interface::robot::RobotState>();
    uint32_t new_id = received_state.message_id;
    uint32_t old_id = latest_accepted_state.message_id;
    constexpr uint32_t kMaxDiff = static_cast<uint32_t>(std::numeric_limits<uint32_t>::max() * 0.1);

    // Check if the received state is new and handle an overflow of the message ID.
    if ((new_id > old_id) ? (new_id - old_id) < kMaxDiff : (old_id - new_id) > kMaxDiff) {
      latest_accepted_state = received_state;
    }
  }

  updateState(latest_accepted_state);
  return latest_accepted_state;
}

void Robot::Impl::updateState(const research_interface::robot::RobotState& robot_state) {
  motion_generator_mode_ = robot_state.motion_generator_mode;
  controller_mode_ = robot_state.controller_mode;
  message_id_ = robot_state.message_id;
}

Robot::ServerVersion Robot::Impl::serverVersion() const noexcept {
  return ri_version_;
}

bool Robot::Impl::motionGeneratorRunning() const noexcept {
  return motion_generator_mode_ != research_interface::robot::MotionGeneratorMode::kIdle;
}

bool Robot::Impl::controllerRunning() const noexcept {
  return controller_mode_ == research_interface::robot::ControllerMode::kExternalController;
}

RealtimeConfig Robot::Impl::realtimeConfig() const noexcept {
  return realtime_config_;
}

void Robot::Impl::startMotion(
    research_interface::robot::Move::ControllerMode controller_mode,
    research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
    const research_interface::robot::Move::Deviation& maximum_path_deviation,
    const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) {
  if (motionGeneratorRunning()) {
    throw ControlException("libfranka robot: attempted to start multiple motion generators!");
  }

  research_interface::robot::MotionGeneratorMode state_motion_generator_mode;
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

  research_interface::robot::ControllerMode state_controller_mode;
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
      throw std::invalid_argument("libfranka robot: Invalid controller mode given.");
  }

  executeCommand<research_interface::robot::Move>(
      controller_mode, motion_generator_mode, maximum_path_deviation, maximum_goal_pose_deviation);

  RobotState robot_state{};
  while (motion_generator_mode_ != state_motion_generator_mode ||
         controller_mode_ != state_controller_mode) {
    research_interface::robot::Function function;
    if (network_->tcpReadResponse<research_interface::robot::Function>(&function)) {
      if (function != research_interface::robot::Function::kMove) {
        throw ProtocolException("libfranka robot: unexpected response!");
      }
      try {
        network_->tcpHandleResponse<research_interface::robot::Move>(
            std::bind(&Robot::Impl::handleCommandResponse<research_interface::robot::Move>, this,
                      std::placeholders::_1));
      } catch (const CommandException& e) {
        if (robot_state.robot_mode == RobotMode::kReflex) {
          throw ControlException(e.what() + " "s +
                                 static_cast<std::string>(robot_state.last_motion_errors));
        }
        throw ControlException(e.what());
      }
    }

    robot_state = update();
  }
}

void Robot::Impl::stopMotion() {
  if (!motionGeneratorRunning()) {
    return;
  }

  // If a controller is currently running, send zero torques while stopping motion.
  research_interface::robot::ControllerCommand controller_command{};

  research_interface::robot::MotionGeneratorCommand motion_command{};
  motion_command.motion_generation_finished = true;
  // The TCP response for the finished Move might arrive while the robot state still shows
  // that the motion is running, or afterwards. To handle both situations, we do not process
  // TCP packages in this loop and explicitly wait for the Move response over TCP afterwards.
  while (motionGeneratorRunning()) {
    sendRobotCommand(&motion_command, controllerRunning() ? &controller_command : nullptr);
    receiveRobotState();
  }
  handleCommandResponse<research_interface::robot::Move>(
      network_->tcpBlockingReceiveResponse<research_interface::robot::Move>());
}

void Robot::Impl::startController() {
  if (controllerRunning()) {
    throw ControlException("libfranka robot: attempted to start multiple controllers!");
    return;
  }

  executeCommand<research_interface::robot::SetControllerMode>(
      research_interface::robot::SetControllerMode::ControllerMode::kExternalController);

  while (!controllerRunning()) {
    update();
  }
}

void Robot::Impl::stopController() {
  if (!controllerRunning()) {
    return;
  }

  executeCommand<research_interface::robot::SetControllerMode>(
      research_interface::robot::SetControllerMode::ControllerMode::kJointImpedance);

  research_interface::robot::ControllerCommand command{};
  while (controller_mode_ != research_interface::robot::ControllerMode::kJointImpedance) {
    update(nullptr, &command);
  }
}

Model Robot::Impl::loadModel() const {
  return Model(*network_);
}

RobotState convertRobotState(const research_interface::robot::RobotState& robot_state,
                             uint32_t previous_message_id) noexcept {
  RobotState converted;
  converted.O_T_EE = robot_state.O_T_EE;
  converted.O_T_EE_d = robot_state.O_T_EE_d;
  converted.EE_T_K = robot_state.EE_T_K;
  converted.elbow = robot_state.elbow;
  converted.elbow_d = robot_state.elbow_d;
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
  converted.current_errors = robot_state.errors;
  converted.last_motion_errors = robot_state.reflex_reason;
  converted.sequence_number = robot_state.message_id;

  // We only accept valid (i.e. newer) robot states in receiveRobotState(),
  // so here we can detect overflows by a simple comparison.
  if (robot_state.message_id > previous_message_id) {
    converted.ticks = robot_state.message_id - previous_message_id;
  } else {
    converted.ticks = std::numeric_limits<uint32_t>::max() - previous_message_id +
                      robot_state.message_id + 1;
  }

  switch (robot_state.robot_mode) {
    case research_interface::robot::RobotMode::kEmergency:
      converted.robot_mode = RobotMode::kUserStopped;
      break;
    case research_interface::robot::RobotMode::kIdle:
    case research_interface::robot::RobotMode::kMove:
      converted.robot_mode = RobotMode::kReady;
      break;
    case research_interface::robot::RobotMode::kGuiding:
      converted.robot_mode = RobotMode::kGuiding;
      break;
    case research_interface::robot::RobotMode::kReflex:
      converted.robot_mode = RobotMode::kReflex;
      break;
    case research_interface::robot::RobotMode::kAutomaticErrorRecovery:
      converted.robot_mode = RobotMode::kAutomaticErrorRecovery;
      break;
    case research_interface::robot::RobotMode::kEmergency2:
    case research_interface::robot::RobotMode::kForce:
    case research_interface::robot::RobotMode::kMoveForce:
    case research_interface::robot::RobotMode::kRcuInputError:
      converted.robot_mode = RobotMode::kOther;
      break;
  }

  return converted;
}

}  // namespace franka
