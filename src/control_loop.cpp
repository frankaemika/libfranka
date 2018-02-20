// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "control_loop.h"

#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>

#include <franka/exception.h>

#include "motion_generator_traits.h"

// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

template <typename T>
constexpr research_interface::robot::Move::Deviation ControlLoop<T>::kDefaultDeviation;

template <typename T>
ControlLoop<T>::ControlLoop(RobotControl& robot,
                            MotionGeneratorCallback motion_callback,
                            ControlCallback control_callback)
    : robot_(robot),
      motion_callback_(std::move(motion_callback)),
      control_callback_(std::move(control_callback)) {
  bool throw_on_error = robot_.realtimeConfig() == RealtimeConfig::kEnforce;
  if (throw_on_error && !hasRealtimeKernel()) {
    throw RealtimeException("libfranka: Running kernel does not have realtime capabilities.");
  }
  setCurrentThreadToRealtime(throw_on_error);
}

template <typename T>
ControlLoop<T>::ControlLoop(RobotControl& robot,
                            ControlCallback control_callback,
                            MotionGeneratorCallback motion_callback)
    : ControlLoop(robot, std::move(motion_callback), std::move(control_callback)) {
  if (!control_callback_) {
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }
  if (!motion_callback_) {
    throw std::invalid_argument("libfranka: Invalid motion callback given.");
  }

  motion_id_ = robot.startMotion(
      research_interface::robot::Move::ControllerMode::kExternalController,
      MotionGeneratorTraits<T>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
}

template <typename T>
ControlLoop<T>::ControlLoop(RobotControl& robot,
                            ControllerMode controller_mode,
                            MotionGeneratorCallback motion_callback)
    : ControlLoop(robot, std::move(motion_callback), {}) {
  if (!motion_callback_) {
    throw std::invalid_argument("libfranka: Invalid motion callback given.");
  }
  research_interface::robot::Move::ControllerMode mode;
  switch (controller_mode) {
    case ControllerMode::kJointImpedance:
      mode = decltype(mode)::kJointImpedance;
      break;
    case ControllerMode::kCartesianImpedance:
      mode = decltype(mode)::kCartesianImpedance;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid controller mode given.");
  }
  motion_id_ = robot.startMotion(mode, MotionGeneratorTraits<T>::kMotionGeneratorMode,
                                 kDefaultDeviation, kDefaultDeviation);
}

template <typename T>
void ControlLoop<T>::operator()() try {
  RobotState robot_state = robot_.update();
  robot_.throwOnMotionError(robot_state, motion_id_);

  Duration previous_time = robot_state.time;

  research_interface::robot::MotionGeneratorCommand motion_command{};
  if (control_callback_) {
    research_interface::robot::ControllerCommand control_command{};
    while (spinMotion(robot_state, robot_state.time - previous_time, &motion_command) &&
           spinControl(robot_state, robot_state.time - previous_time, &control_command)) {
      previous_time = robot_state.time;
      robot_state = robot_.update(&motion_command, &control_command);
      robot_.throwOnMotionError(robot_state, motion_id_);
    }
    robot_.finishMotion(motion_id_, &motion_command, &control_command);
  } else {
    while (spinMotion(robot_state, robot_state.time - previous_time, &motion_command)) {
      previous_time = robot_state.time;
      robot_state = robot_.update(&motion_command, nullptr);
      robot_.throwOnMotionError(robot_state, motion_id_);
    }
    robot_.finishMotion(motion_id_, &motion_command, nullptr);
  }
} catch (...) {
  try {
    robot_.cancelMotion(motion_id_);
  } catch (...) {
  }
  throw;
}

template <typename T>
bool ControlLoop<T>::spinControl(const RobotState& robot_state,
                                 franka::Duration time_step,
                                 research_interface::robot::ControllerCommand* command) {
  Torques control_output = control_callback_(robot_state, time_step);
  command->tau_J_d = control_output.tau_J;
  return !control_output.motion_finished;
}

template <typename T>
bool ControlLoop<T>::spinMotion(const RobotState& robot_state,
                                franka::Duration time_step,
                                research_interface::robot::MotionGeneratorCommand* command) {
  T motion_output = motion_callback_(robot_state, time_step);
  convertMotion(motion_output, command);
  return !motion_output.motion_finished;
}

template <>
void ControlLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->q_d = motion.q;
}

template <>
void ControlLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->dq_d = motion.dq;
}

template <>
void ControlLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_T_EE_d = motion.O_T_EE;

  if (motion.hasValidElbow()) {
    command->valid_elbow = true;
    command->elbow_d = motion.elbow;
  } else {
    command->valid_elbow = false;
    command->elbow_d = {};
  }
}

template <>
void ControlLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_dP_EE_d = motion.O_dP_EE;

  if (motion.hasValidElbow()) {
    command->valid_elbow = true;
    command->elbow_d = motion.elbow;
  } else {
    command->valid_elbow = false;
    command->elbow_d = {};
  }
}

void setCurrentThreadToRealtime(bool throw_on_error) {
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  if (thread_priority == -1) {
    throw RealtimeException("libfranka: unable to get maximum possible thread priority: "s +
                            std::strerror(errno));
  }
  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (throw_on_error) {
      throw RealtimeException("libfranka: unable to set realtime scheduling: "s +
                              std::strerror(errno));
    }
  }
}

bool hasRealtimeKernel() {
  std::ifstream realtime("/sys/kernel/realtime", std::ios_base::in);
  bool is_realtime;
  realtime >> is_realtime;
  return is_realtime;
}

template class ControlLoop<JointPositions>;
template class ControlLoop<JointVelocities>;
template class ControlLoop<CartesianPose>;
template class ControlLoop<CartesianVelocities>;

}  // namespace franka
