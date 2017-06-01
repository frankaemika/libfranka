#include "motion_generator_loop.h"

#include "conversion.h"
#include "motion_generator_traits.h"

namespace franka {

template <typename T>
MotionGeneratorLoop<T>::MotionGeneratorLoop(RobotControl& robot,
                                            ControlCallback control_callback,
                                            MotionGeneratorCallback motion_callback)
    : ControlLoopBase(robot, control_callback), motion_callback_(std::move(motion_callback)) {
  if (!motion_callback_) {
    throw std::invalid_argument("libfranka: Invalid motion callback given.");
  }

  robot.startMotion(research_interface::Move::ControllerMode::kExternalController,
                    MotionGeneratorTraits<T>::kMotionGeneratorMode,
                    research_interface::Move::Deviation(0.2, 1.5, 1.5),
                    research_interface::Move::Deviation(0.2, 1.5, 1.5));
}

template <typename T>
MotionGeneratorLoop<T>::MotionGeneratorLoop(RobotControl& robot,
                                            ControllerMode controller_mode,
                                            MotionGeneratorCallback motion_callback)
    : ControlLoopBase(robot, {}), motion_callback_(std::move(motion_callback)) {
  if (!motion_callback_) {
    std::invalid_argument("libfranka: Invalid motion callback given.");
  }
  research_interface::Move::ControllerMode mode;
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
      throw std::invalid_argument("Invalid motion generator mode given.");
  }
  robot.startMotion(mode, MotionGeneratorTraits<T>::kMotionGeneratorMode,
                    research_interface::Move::Deviation(0.2, 1.5, 1.5),
                    research_interface::Move::Deviation(0.2, 1.5, 1.5));
}

template <typename T>
MotionGeneratorLoop<T>::~MotionGeneratorLoop() {
  if (motion_callback_) {
    robot_.stopMotion();
  }
}

template <typename T>
void MotionGeneratorLoop<T>::operator()() {
  RobotState robot_state = convertRobotState(robot_.update({}));
  research_interface::ControllerCommand control_command{};
  research_interface::MotionGeneratorCommand motion_command{};
  while (spinMotionOnce(robot_state, &motion_command) &&
         spinControlOnce(robot_state, &control_command)) {
    robot_state = convertRobotState(robot_.update(motion_command, control_command));
  }
}

template <typename T>
bool MotionGeneratorLoop<T>::spinMotionOnce(const RobotState& robot_state,
                                            research_interface::MotionGeneratorCommand* command) {
  if (motion_callback_) {
    T motion_output = motion_callback_(robot_state);
    if (motion_output.stop()) {
      return false;
    }
    convertMotion(motion_output, command);
  }

  return true;
}

template <>
void MotionGeneratorLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->q_d = motion.q;
}

template <>
void MotionGeneratorLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->dq_d = motion.dq;
}

template <>
void MotionGeneratorLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->O_T_EE_d = motion.O_T_EE;
}

template <>
void MotionGeneratorLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    research_interface::MotionGeneratorCommand* command) {
  command->O_dP_EE_d = motion.O_dP_EE;
}

}  // namespace franka
