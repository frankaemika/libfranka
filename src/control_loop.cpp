// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "control_loop.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>

#include "motion_generator_traits.h"

// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

namespace {

template <typename T, size_t N>
inline void checkFinite(const std::array<T, N>& array) {
  if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
    throw std::invalid_argument("Commanding value is infinite or NaN.");
  }
}

inline void checkElbow(const std::array<double, 2>& elbow) {
  checkFinite(elbow);
  if (!isValidElbow(elbow)) {
    throw std::invalid_argument(
        "Invalid elbow configuration given! Only +1 or -1 are allowed for the sign of the 4th "
        "joint.");
  }
}

inline void checkMatrix(const std::array<double, 16>& transform) {
  checkFinite(transform);
  if (!isHomogeneousTransformation(transform)) {
    throw std::invalid_argument(
        "libfranka: Attempt to set invalid transformation in motion generator. Has to be column "
        "major!");
  }
}

}  // anonymous namespace

template <typename T>
constexpr research_interface::robot::Move::Deviation ControlLoop<T>::kDefaultDeviation;

template <typename T>
ControlLoop<T>::ControlLoop(RobotControl& robot,
                            MotionGeneratorCallback motion_callback,
                            ControlCallback control_callback,
                            bool limit_rate,
                            double cutoff_frequency)
    : robot_(robot),
      motion_callback_(std::move(motion_callback)),
      control_callback_(std::move(control_callback)),
      limit_rate_(limit_rate),
      cutoff_frequency_(cutoff_frequency) {
  bool throw_on_error = robot_.realtimeConfig() == RealtimeConfig::kEnforce;
  std::string error_message;
  if (!setCurrentThreadToHighestSchedulerPriority(&error_message) && throw_on_error) {
    throw RealtimeException(error_message);
  }
  if (throw_on_error && !hasRealtimeKernel()) {
    throw RealtimeException("libfranka: Running kernel does not have realtime capabilities.");
  }
}

template <typename T>
ControlLoop<T>::ControlLoop(RobotControl& robot,
                            ControlCallback control_callback,
                            MotionGeneratorCallback motion_callback,
                            bool limit_rate,
                            double cutoff_frequency)
    : ControlLoop(robot,
                  std::move(motion_callback),
                  std::move(control_callback),
                  limit_rate,
                  cutoff_frequency) {
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
                            MotionGeneratorCallback motion_callback,
                            bool limit_rate,
                            double cutoff_frequency)
    : ControlLoop(robot, std::move(motion_callback), {}, limit_rate, cutoff_frequency) {
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
  RobotState robot_state = robot_.update(nullptr, nullptr);
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
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 7; i++) {
      control_output.tau_J[i] = lowpassFilter(kDeltaT, control_output.tau_J[i],
                                              robot_state.tau_J_d[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    control_output.tau_J = limitRate(kMaxTorqueRate, control_output.tau_J, robot_state.tau_J_d);
  }
  command->tau_J_d = control_output.tau_J;
  checkFinite(command->tau_J_d);
  return !control_output.motion_finished;
}

template <typename T>
bool ControlLoop<T>::spinMotion(const RobotState& robot_state,
                                franka::Duration time_step,
                                research_interface::robot::MotionGeneratorCommand* command) {
  T motion_output = motion_callback_(robot_state, time_step);
  convertMotion(motion_output, robot_state, command);
  return !motion_output.motion_finished;
}

template <>
void ControlLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->q_c = motion.q;
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 7; i++) {
      command->q_c[i] =
          lowpassFilter(kDeltaT, command->q_c[i], robot_state.q_d[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    command->q_c = limitRate(kMaxJointVelocity, kMaxJointAcceleration, kMaxJointJerk, command->q_c,
                             robot_state.q_d, robot_state.dq_d, robot_state.ddq_d);
  }
  checkFinite(command->q_c);
}

template <>
void ControlLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->dq_c = motion.dq;
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 7; i++) {
      command->dq_c[i] =
          lowpassFilter(kDeltaT, command->dq_c[i], robot_state.dq_d[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    command->dq_c = limitRate(kMaxJointVelocity, kMaxJointAcceleration, kMaxJointJerk,
                              command->dq_c, robot_state.dq_d, robot_state.ddq_d);
  }
  checkFinite(command->dq_c);
}

template <>
void ControlLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_T_EE_c = motion.O_T_EE;
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    command->O_T_EE_c =
        cartesianLowpassFilter(kDeltaT, command->O_T_EE_c, robot_state.O_T_EE_c, cutoff_frequency_);
  }

  if (limit_rate_) {
    command->O_T_EE_c = limitRate(
        kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
        kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk, command->O_T_EE_c,
        robot_state.O_T_EE_c, robot_state.O_dP_EE_c, robot_state.O_ddP_EE_c);
  }
  checkMatrix(command->O_T_EE_c);

  if (motion.hasElbow()) {
    command->valid_elbow = true;
    command->elbow_c = motion.elbow;
    if (cutoff_frequency_ < kMaxCutoffFrequency) {
      command->elbow_c[0] =
          lowpassFilter(kDeltaT, command->elbow_c[0], robot_state.elbow_c[0], cutoff_frequency_);
    }
    if (limit_rate_) {
      command->elbow_c[0] =
          limitRate(kMaxElbowVelocity, kMaxElbowAcceleration, kMaxElbowJerk, command->elbow_c[0],
                    robot_state.elbow_c[0], robot_state.delbow_c[0], robot_state.ddelbow_c[0]);
    }
    checkElbow(command->elbow_c);
  } else {
    command->valid_elbow = false;
    command->elbow_c = {};
  }
}

template <>
void ControlLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_dP_EE_c = motion.O_dP_EE;
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 6; i++) {
      command->O_dP_EE_c[i] = lowpassFilter(kDeltaT, command->O_dP_EE_c[i],
                                            robot_state.O_dP_EE_c[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    command->O_dP_EE_c =
        limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
                  kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk,
                  command->O_dP_EE_c, robot_state.O_dP_EE_c, robot_state.O_ddP_EE_c);
  }
  checkFinite(command->O_dP_EE_c);

  if (motion.hasElbow()) {
    command->valid_elbow = true;
    command->elbow_c = motion.elbow;
    if (cutoff_frequency_ < kMaxCutoffFrequency) {
      command->elbow_c[0] =
          lowpassFilter(kDeltaT, command->elbow_c[0], robot_state.elbow_c[0], cutoff_frequency_);
    }
    if (limit_rate_) {
      command->elbow_c[0] =
          limitRate(kMaxElbowVelocity, kMaxElbowAcceleration, kMaxElbowJerk, command->elbow_c[0],
                    robot_state.elbow_c[0], robot_state.delbow_c[0], robot_state.ddelbow_c[0]);
    }
    checkElbow(command->elbow_c);
  } else {
    command->valid_elbow = false;
    command->elbow_c = {};
  }
}

template class ControlLoop<JointPositions>;
template class ControlLoop<JointVelocities>;
template class ControlLoop<CartesianPose>;
template class ControlLoop<CartesianVelocities>;

}  // namespace franka
