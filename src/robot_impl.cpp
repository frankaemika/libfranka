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

  connect<research_interface::robot::Connect, research_interface::robot::kVersion>(
      *network_, command_id_++, &ri_version_);

  updateState(network_->udpRead<research_interface::robot::RobotState>());
}

RobotState Robot::Impl::update(
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) {
  network_->tcpThrowIfConnectionClosed();

  sendRobotCommand(motion_command, control_command);

  return convertRobotState(receiveRobotState());
}

void Robot::Impl::throwOnMotionError(const RobotState& robot_state, const uint32_t* motion_id) {
  if (robot_state.robot_mode != RobotMode::kReady) {
    // Wait until robot state shows stopped motion and controller.
    while (motionGeneratorRunning() || controllerRunning()) {
      receiveRobotState();
    }

    // TODO (fwalch): Change from uint32_t* to uint32_t when RCU 16 changes are in.
    if (motion_id != nullptr) {
      // If a motion generator was running and the robot state shows an error,
      // we will receive a TCP response to the Move command.
      try {
        handleCommandResponse<research_interface::robot::Move>(
            network_->tcpBlockingReceiveResponse<research_interface::robot::Move>(*motion_id,
                                                                                  true));
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
}

RobotState Robot::Impl::readOnce() {
  if (motionGeneratorRunning() || controllerRunning()) {
    throw ControlException("libfranka robot: Can not execute readOnce() while motion is running.");
  }

  // Delete old data from the UDP buffer.
  while (network_->udpAvailableData() > 0) {
    network_->udpRead<research_interface::robot::RobotState>();
  }

  return convertRobotState(receiveRobotState());
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

    if (received_state.message_id > latest_accepted_state.message_id) {
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

std::mutex& Robot::Impl::mutex() noexcept {
  return mutex_;
}

uint32_t Robot::Impl::startMotion(
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

  const uint32_t move_command_id = executeCommand<research_interface::robot::Move>(
      controller_mode, motion_generator_mode, maximum_path_deviation, maximum_goal_pose_deviation);

  RobotState robot_state{};
  while (motion_generator_mode_ != state_motion_generator_mode ||
         controller_mode_ != state_controller_mode) {
    try {
      if (network_->tcpReceiveResponse<research_interface::robot::Move>(
              move_command_id,
              std::bind(&Robot::Impl::handleCommandResponse<research_interface::robot::Move>, this,
                        std::placeholders::_1))) {
        break;
      }
    } catch (const CommandException& e) {
      if (robot_state.robot_mode == RobotMode::kReflex) {
        throw ControlException(e.what() + " "s +
                               static_cast<std::string>(robot_state.last_motion_errors));
      }
      throw ControlException(e.what());
    }

    robot_state = update();
    throwOnMotionError(robot_state, &move_command_id);
  }

  return move_command_id;
}

void Robot::Impl::stopMotion(uint32_t motion_id) {
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
      network_->tcpBlockingReceiveResponse<research_interface::robot::Move>(motion_id, true));
}

void Robot::Impl::startController() {
  if (controllerRunning()) {
    throw ControlException("libfranka robot: attempted to start multiple controllers!");
    return;
  }

  executeCommand<research_interface::robot::SetControllerMode>(
      research_interface::robot::SetControllerMode::ControllerMode::kExternalController);

  while (!controllerRunning()) {
    auto robot_state = update();
    throwOnMotionError(robot_state, nullptr);
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
    auto robot_state = update(nullptr, &command);
    throwOnMotionError(robot_state, nullptr);
  }
}

Model Robot::Impl::loadModel() {
  return Model(*network_, command_id_++);
}

RobotState convertRobotState(const research_interface::robot::RobotState& robot_state) noexcept {
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
  converted.time = Duration(robot_state.message_id);

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
