// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "robot_impl.h"

#include <sstream>

#include "load_calculations.h"

namespace franka {

namespace {

inline ControlException createControlException(const char* message,
                                               research_interface::robot::Move::Status move_status,
                                               const Errors& reflex_errors,
                                               const std::vector<Record>& log) {
  std::ostringstream message_stream;
  message_stream << message;
  if (move_status == decltype(move_status)::kReflexAborted) {
    message_stream << " " << reflex_errors;

    if (log.size() >= 2) {
      // Count number of lost packets in the last and before last packets.
      uint64_t lost_packets =
          log[log.size() - 1].state.time.toMSec() - log[log.size() - 2].state.time.toMSec() - 1;
      // Read second to last control_command_success_rate since the last one will always be zero and
      // consider in the success rate assumming all previous packets were lost.
      message_stream << std::endl
                     << "control_command_success_rate: "
                     << (log[log.size() - 2].state.control_command_success_rate *
                         (1 - static_cast<double>(lost_packets) / 100));
      // Packets lost in a row
      if (lost_packets > 0) {
        message_stream << " packets lost in a row in the last sample: " << lost_packets;
      }
    }
  }
  return ControlException(message_stream.str(), log);
}

}  // anonymous namespace

Robot::Impl::Impl(std::unique_ptr<Network> network, size_t log_size, RealtimeConfig realtime_config)
    : network_{std::move(network)}, logger_{log_size}, realtime_config_{realtime_config} {
  if (!network_) {
    throw std::invalid_argument("libfranka robot: Invalid argument");
  }

  connect<research_interface::robot::Connect, research_interface::robot::kVersion>(*network_,
                                                                                   &ri_version_);
  updateState(network_->udpBlockingReceive<research_interface::robot::RobotState>());
}

RobotState Robot::Impl::update(
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) {
  network_->tcpThrowIfConnectionClosed();

  research_interface::robot::RobotCommand robot_command =
      sendRobotCommand(motion_command, control_command);

  RobotState state = convertRobotState(receiveRobotState());
  logger_.log(state, robot_command);

  return state;
}

void Robot::Impl::throwOnMotionError(const RobotState& robot_state, uint32_t motion_id) {
  if (robot_state.robot_mode != RobotMode::kMove ||
      motion_generator_mode_ != current_move_motion_generator_mode_ ||
      controller_mode_ != current_move_controller_mode_) {
    // We detect a move error by changes in the robot state and we will receive a TCP response to
    // the Move command.
    auto response =
        network_->tcpBlockingReceiveResponse<research_interface::robot::Move>(motion_id);
    try {
      handleCommandResponse<research_interface::robot::Move>(response);
    } catch (const CommandException& e) {
      throw createControlException(e.what(), response.status, robot_state.last_motion_errors,
                                   logger_.flush());
    }
    throw ProtocolException("Unexpected reply to a Move command");
  }
}

RobotState Robot::Impl::readOnce() {
  // Delete old data from the UDP buffer.
  research_interface::robot::RobotState robot_state;
  while (network_->udpReceive<decltype(robot_state)>(&robot_state)) {
  }

  return convertRobotState(receiveRobotState());
}

research_interface::robot::RobotCommand Robot::Impl::sendRobotCommand(
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) const {
  research_interface::robot::RobotCommand robot_command{};
  if (motion_command != nullptr || control_command != nullptr) {
    robot_command.message_id = message_id_;
    if (motion_command != nullptr) {
      if (current_move_motion_generator_mode_ ==
          research_interface::robot::MotionGeneratorMode::kIdle) {
        throw ControlException(
            "libfranka robot: Trying to send motion command, but no motion generator running!");
      }
      robot_command.motion = *motion_command;
    }
    if (control_command != nullptr) {
      if (current_move_controller_mode_ !=
          research_interface::robot::ControllerMode::kExternalController) {
        throw ControlException(
            "libfranka robot: Trying to send control command, but no controller running!");
      }
      robot_command.control = *control_command;
    }

    if (current_move_motion_generator_mode_ !=
            research_interface::robot::MotionGeneratorMode::kIdle &&
        current_move_controller_mode_ ==
            research_interface::robot::ControllerMode::kExternalController &&
        (motion_command == nullptr || control_command == nullptr)) {
      throw ControlException("libfranka robot: Trying to send partial robot command!");
    }

    network_->udpSend<research_interface::robot::RobotCommand>(robot_command);
  }

  return robot_command;
}

research_interface::robot::RobotState Robot::Impl::receiveRobotState() {
  research_interface::robot::RobotState latest_accepted_state;
  latest_accepted_state.message_id = message_id_;

  // If states are already available on the socket, use the one with the most recent message ID.
  research_interface::robot::RobotState received_state{};
  while (network_->udpReceive(&received_state)) {
    if (received_state.message_id > latest_accepted_state.message_id) {
      latest_accepted_state = received_state;
    }
  }

  // If there was no valid state on the socket, we need to wait.
  while (latest_accepted_state.message_id == message_id_) {
    received_state = network_->udpBlockingReceive<decltype(received_state)>();
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

uint32_t Robot::Impl::startMotion(
    research_interface::robot::Move::ControllerMode controller_mode,
    research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
    const research_interface::robot::Move::Deviation& maximum_path_deviation,
    const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) {
  if (motionGeneratorRunning() || controllerRunning()) {
    throw ControlException("libfranka robot: Attempted to start multiple motions!");
  }

  switch (motion_generator_mode) {
    case decltype(motion_generator_mode)::kJointPosition:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kJointPosition;
      break;
    case decltype(motion_generator_mode)::kJointVelocity:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kJointVelocity;
      break;
    case decltype(motion_generator_mode)::kCartesianPosition:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kCartesianPosition;
      break;
    case decltype(motion_generator_mode)::kCartesianVelocity:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kCartesianVelocity;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid motion generator mode given.");
  }

  switch (controller_mode) {
    case decltype(controller_mode)::kJointImpedance:
      current_move_controller_mode_ = decltype(current_move_controller_mode_)::kJointImpedance;
      break;
    case decltype(controller_mode)::kCartesianImpedance:
      current_move_controller_mode_ = decltype(current_move_controller_mode_)::kCartesianImpedance;
      break;
    case decltype(controller_mode)::kExternalController:
      current_move_controller_mode_ = decltype(current_move_controller_mode_)::kExternalController;
      break;
    default:
      throw std::invalid_argument("libfranka robot: Invalid controller mode given.");
  }

  const uint32_t move_command_id = executeCommand<research_interface::robot::Move>(
      controller_mode, motion_generator_mode, maximum_path_deviation, maximum_goal_pose_deviation);

  RobotState robot_state{};
  while (motion_generator_mode_ != current_move_motion_generator_mode_ ||
         controller_mode_ != current_move_controller_mode_) {
    try {
      if (network_->tcpReceiveResponse<research_interface::robot::Move>(
              move_command_id,
              std::bind(&Robot::Impl::handleCommandResponse<research_interface::robot::Move>, this,
                        std::placeholders::_1))) {
        break;
      }
    } catch (const CommandException& e) {
      throw ControlException(e.what());
    }

    robot_state = update(nullptr, nullptr);
  }

  logger_.flush();

  return move_command_id;
}

void Robot::Impl::finishMotion(
    uint32_t motion_id,
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) {
  if (!motionGeneratorRunning() && !controllerRunning()) {
    current_move_motion_generator_mode_ = research_interface::robot::MotionGeneratorMode::kIdle;
    current_move_controller_mode_ = research_interface::robot::ControllerMode::kOther;
    return;
  }

  if (motion_command == nullptr) {
    throw ControlException("libfranka robot: No motion generator command given!");
  }
  research_interface::robot::MotionGeneratorCommand motion_finished_command = *motion_command;
  motion_finished_command.motion_generation_finished = true;

  // The TCP response for the finished Move might arrive while the robot state still shows that the
  // motion is running, or afterwards. To handle both situations, we do not process TCP packages in
  // this loop and explicitly wait for the Move response over TCP afterwards.
  RobotState robot_state{};
  while (motionGeneratorRunning() || controllerRunning()) {
    robot_state = update(&motion_finished_command, control_command);
  }

  auto response = network_->tcpBlockingReceiveResponse<research_interface::robot::Move>(motion_id);
  if (response.status == research_interface::robot::Move::Status::kReflexAborted) {
    throw createControlException("Motion finished commanded, but the robot is still moving!",
                                 response.status, robot_state.last_motion_errors, logger_.flush());
  }
  try {
    handleCommandResponse<research_interface::robot::Move>(response);
  } catch (const CommandException& e) {
    throw createControlException(e.what(), response.status, robot_state.last_motion_errors,
                                 logger_.flush());
  }
  current_move_motion_generator_mode_ = research_interface::robot::MotionGeneratorMode::kIdle;
  current_move_controller_mode_ = research_interface::robot::ControllerMode::kOther;
}

void Robot::Impl::cancelMotion(uint32_t motion_id) {
  try {
    executeCommand<research_interface::robot::StopMove>();
  } catch (const CommandException& e) {
    throw ControlException(e.what());
  }

  research_interface::robot::RobotState robot_state;
  do {
    robot_state = receiveRobotState();
  } while (motionGeneratorRunning() || controllerRunning());

  // Ignore Move response.
  // TODO (FWA): It is not guaranteed that the Move response won't come later
  network_->tcpReceiveResponse<research_interface::robot::Move>(motion_id, [](auto) {});
  current_move_motion_generator_mode_ = research_interface::robot::MotionGeneratorMode::kIdle;
  current_move_controller_mode_ = research_interface::robot::ControllerMode::kOther;
}

Model Robot::Impl::loadModel() const {
  return Model(*network_);
}

RobotState convertRobotState(const research_interface::robot::RobotState& robot_state) noexcept {
  RobotState converted;
  converted.O_T_EE = robot_state.O_T_EE;
  converted.O_T_EE_d = robot_state.O_T_EE_d;
  converted.F_T_NE = robot_state.F_T_NE;
  converted.NE_T_EE = robot_state.NE_T_EE;
  converted.F_T_EE = robot_state.F_T_EE;
  converted.EE_T_K = robot_state.EE_T_K;
  converted.m_ee = robot_state.m_ee;
  converted.F_x_Cee = robot_state.F_x_Cee;
  converted.I_ee = robot_state.I_ee;
  converted.m_load = robot_state.m_load;
  converted.F_x_Cload = robot_state.F_x_Cload;
  converted.I_load = robot_state.I_load;
  converted.m_total = robot_state.m_ee + robot_state.m_load;
  converted.F_x_Ctotal = combineCenterOfMass(robot_state.m_ee, robot_state.F_x_Cee,
                                             robot_state.m_load, robot_state.F_x_Cload);
  converted.I_total = combineInertiaTensor(
      robot_state.m_ee, robot_state.F_x_Cee, robot_state.I_ee, robot_state.m_load,
      robot_state.F_x_Cload, robot_state.I_load, converted.m_total, converted.F_x_Ctotal);
  converted.elbow = robot_state.elbow;
  converted.elbow_d = robot_state.elbow_d;
  converted.elbow_c = robot_state.elbow_c;
  converted.delbow_c = robot_state.delbow_c;
  converted.ddelbow_c = robot_state.ddelbow_c;
  converted.tau_J = robot_state.tau_J;
  converted.tau_J_d = robot_state.tau_J_d;
  converted.dtau_J = robot_state.dtau_J;
  converted.q = robot_state.q;
  converted.dq = robot_state.dq;
  converted.q_d = robot_state.q_d;
  converted.dq_d = robot_state.dq_d;
  converted.ddq_d = robot_state.ddq_d;
  converted.joint_contact = robot_state.joint_contact;
  converted.cartesian_contact = robot_state.cartesian_contact;
  converted.joint_collision = robot_state.joint_collision;
  converted.cartesian_collision = robot_state.cartesian_collision;
  converted.tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
  converted.O_F_ext_hat_K = robot_state.O_F_ext_hat_K;
  converted.K_F_ext_hat_K = robot_state.K_F_ext_hat_K;
  converted.O_dP_EE_d = robot_state.O_dP_EE_d;
  converted.O_T_EE_c = robot_state.O_T_EE_c;
  converted.O_dP_EE_c = robot_state.O_dP_EE_c;
  converted.O_ddP_EE_c = robot_state.O_ddP_EE_c;
  converted.theta = robot_state.theta;
  converted.dtheta = robot_state.dtheta;
  converted.current_errors = robot_state.errors;
  converted.last_motion_errors = robot_state.reflex_reason;
  converted.control_command_success_rate = robot_state.control_command_success_rate;
  converted.time = Duration(robot_state.message_id);

  converted.robot_mode = RobotMode::kOther;
  switch (robot_state.robot_mode) {
    case research_interface::robot::RobotMode::kOther:
      converted.robot_mode = RobotMode::kOther;
      break;
    case research_interface::robot::RobotMode::kIdle:
      converted.robot_mode = RobotMode::kIdle;
      break;
    case research_interface::robot::RobotMode::kMove:
      converted.robot_mode = RobotMode::kMove;
      break;
    case research_interface::robot::RobotMode::kGuiding:
      converted.robot_mode = RobotMode::kGuiding;
      break;
    case research_interface::robot::RobotMode::kReflex:
      converted.robot_mode = RobotMode::kReflex;
      break;
    case research_interface::robot::RobotMode::kUserStopped:
      converted.robot_mode = RobotMode::kUserStopped;
      break;
    case research_interface::robot::RobotMode::kAutomaticErrorRecovery:
      converted.robot_mode = RobotMode::kAutomaticErrorRecovery;
      break;
  }

  return converted;
}

}  // namespace franka
