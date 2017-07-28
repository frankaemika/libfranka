#include "motion_generator_loop.h"

#include <exception>

#include "motion_generator_traits.h"

namespace franka {

template <typename T>
MotionGeneratorLoop<T>::MotionGeneratorLoop(RobotControl& robot,
                                            ControlCallback control_callback,
                                            MotionGeneratorCallback motion_callback)
    : ControlLoopBase(robot, control_callback), motion_callback_(std::move(motion_callback)) {
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
MotionGeneratorLoop<T>::MotionGeneratorLoop(RobotControl& robot,
                                            ControllerMode controller_mode,
                                            MotionGeneratorCallback motion_callback)
    : ControlLoopBase(robot, {}), motion_callback_(std::move(motion_callback)) {
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
void MotionGeneratorLoop<T>::operator()() {
  RobotState robot_state = robot_.update();
  Duration previous_time = robot_state.time;

  research_interface::robot::MotionGeneratorCommand motion_command{};
  if (control_callback_) {
    research_interface::robot::ControllerCommand control_command{};
    while (spinOnce(robot_state, robot_state.time - previous_time, &motion_command) &&
           spinOnce(robot_state, robot_state.time - previous_time, &control_command)) {
      previous_time = robot_state.time;
      robot_state = robot_.update(&motion_command, &control_command);
    }
  } else {
    while (spinOnce(robot_state, robot_state.time - previous_time, &motion_command)) {
      previous_time = robot_state.time;
      robot_state = robot_.update(&motion_command);
    }
  }
}

template <typename T>
bool MotionGeneratorLoop<T>::spinOnce(const RobotState& robot_state,
                                      franka::Duration time_step,
                                      research_interface::robot::MotionGeneratorCommand* command) {
  T motion_output = motion_callback_(robot_state, time_step);
  if (motion_output.stop()) {
    return false;
  }
  convertMotion(motion_output, command);
  return true;
}

template <>
void MotionGeneratorLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->q_d = motion.q;
}

template <>
void MotionGeneratorLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->dq_d = motion.dq;
}

template <>
void MotionGeneratorLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_T_EE_d = motion.O_T_EE;
}

template <>
void MotionGeneratorLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    research_interface::robot::MotionGeneratorCommand* command) {
  command->O_dP_EE_d = motion.O_dP_EE;
}

}  // namespace franka
