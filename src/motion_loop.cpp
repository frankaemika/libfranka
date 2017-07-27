#include "motion_loop.h"

#include <cstring>
#include <fstream>

#include <franka/exception.h>

#include "motion_generator_traits.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

template <typename T>
constexpr research_interface::robot::Move::Deviation MotionLoop<T>::kDefaultDeviation;

void setCurrentThreadToRealtime(bool throw_on_error) {
  constexpr int kThreadPriority = 20;
  sched_param thread_param{};
  thread_param.sched_priority = kThreadPriority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (throw_on_error) {
      throw RealtimeException("libfranka: unable to set realtime scheduling: "s + strerror(errno));
    }
  }
}

bool hasRealtimeKernel() {
  std::ifstream realtime("/sys/kernel/realtime", std::ios_base::in);
  bool is_realtime;
  realtime >> is_realtime;
  return is_realtime;
}

template <typename T>
void MotionLoop<T>::setRealTime() {
  bool throw_on_error = robot_.realtimeConfig() == RealtimeConfig::kEnforce;
  if (throw_on_error && !hasRealtimeKernel()) {
    throw RealtimeException("libfranka: Running kernel does not have realtime capabilities.");
  }
  setCurrentThreadToRealtime(throw_on_error);
}

template <typename T>
MotionLoop<T>::MotionLoop(RobotControl& robot, ControlCallback control_callback)
    : robot_(robot), motion_callback_(nullptr), control_callback_(std::move(control_callback)) {
  setRealTime();

  if (!control_callback_) {
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }

  robot.startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                    research_interface::robot::Move::MotionGeneratorMode::kIdle, kDefaultDeviation,
                    kDefaultDeviation);
}

template <typename T>
MotionLoop<T>::MotionLoop(RobotControl& robot,
                          ControlCallback control_callback,
                          MotionGeneratorCallback motion_callback)
    : robot_(robot),
      motion_callback_(std::move(motion_callback)),
      control_callback_(std::move(control_callback)) {
  setRealTime();

  if (!control_callback_) {
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }

  if (!motion_callback_) {
    throw std::invalid_argument("libfranka: Invalid motion callback given.");
  }

  robot.startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                    MotionGeneratorTraits<T>::kMotionGeneratorMode, kDefaultDeviation,
                    kDefaultDeviation);
}

template <typename T>
MotionLoop<T>::MotionLoop(RobotControl& robot,
                          ControllerMode controller_mode,
                          MotionGeneratorCallback motion_callback)
    : robot_(robot), motion_callback_(std::move(motion_callback)), control_callback_(nullptr) {
  setRealTime();

  if (!motion_callback_) {
    throw std::invalid_argument("libfranka: Invalid motion callback given.");
  }

  research_interface::robot::Move::ControllerMode mode;
  switch (controller_mode) {
    case ControllerMode::kJointImpedance:
      mode = decltype(mode)::kJointImpedance;
      break;
    case ControllerMode::kMotorPD:
      mode = decltype(mode)::kMotorPD;
      break;
    case ControllerMode::kJointPosition:
      mode = decltype(mode)::kJointPosition;
      break;
    case ControllerMode::kCartesianImpedance:
      mode = decltype(mode)::kCartesianImpedance;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid motion generator mode given.");
  }

  robot.startMotion(mode, MotionGeneratorTraits<T>::kMotionGeneratorMode, kDefaultDeviation,
                    kDefaultDeviation);
}

template <typename T>
MotionLoop<T>::~MotionLoop() noexcept {
  try {
    robot_.stopMotion();
  } catch (...) {
  }
}

template <typename T>
void MotionLoop<T>::operator()() {
  RobotState robot_state = robot_.update();

  research_interface::robot::MotionGeneratorCommand motion_command{};
  if (control_callback_) {
    research_interface::robot::ControllerCommand control_command{};
    while (motionGeneratorSpinOnce(robot_state, &motion_command) &&
           controllerSpinOnce(robot_state, &control_command)) {
      robot_state = robot_.update(&motion_command, &control_command);
    }
  } else {
    while (motionGeneratorSpinOnce(robot_state, &motion_command)) {
      robot_state = robot_.update(&motion_command, nullptr);
    }
  }
}

template <typename T>
bool MotionLoop<T>::motionGeneratorSpinOnce(
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand* command) {
  T motion_output = motion_callback_(robot_state);
  if (motion_output.stop()) {
    return false;
  }
  convertMotion(motion_output, command);
  return true;
}

template <>
bool MotionLoop<Torques>::motionGeneratorSpinOnce(
    const RobotState& /* robot_state */,
    research_interface::robot::MotionGeneratorCommand* /* command */) {
  return true;
}

template <typename T>
bool MotionLoop<T>::controllerSpinOnce(const RobotState& robot_state,
                                       research_interface::robot::ControllerCommand* command) {
  Torques control_output = control_callback_(robot_state);
  if (control_output.stop()) {
    return false;
  }
  command->tau_J_d = control_output.tau_J;
  return true;
}

template <>
void MotionLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->q_d = motion.q;
}

template <>
void MotionLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->dq_d = motion.dq;
}

template <>
void MotionLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_T_EE_d = motion.O_T_EE;
}

template <>
void MotionLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_dP_EE_d = motion.O_dP_EE;
}

}  // namespace franka
