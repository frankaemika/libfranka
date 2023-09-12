// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/active_control.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <research_interface/robot/rbk_types.h>

#include "robot_impl.h"

namespace franka {

ActiveControl::ActiveControl(std::shared_ptr<Robot::Impl> robot_impl,
                             uint32_t motion_id,
                             std::unique_lock<std::mutex> control_lock)
    : robot_impl(std::move(robot_impl)),
      motion_id(motion_id),
      control_lock(std::move(control_lock)),
      control_finished(false),
      first_read_attempt(true) {}

ActiveControl::~ActiveControl() {
  if (!control_finished) {
    robot_impl->cancelMotion(motion_id);
  }
}

std::pair<RobotState, Duration> ActiveControl::readOnce() {
  auto robot_state = robot_impl->readOnce();
  robot_impl->throwOnMotionError(robot_state, motion_id);

  if (first_read_attempt) {
    first_read_attempt = false;
    last_read_access = robot_state.time;
    return std::make_pair(robot_state, Duration(0));
  }

  auto time_since_last_read = robot_state.time - last_read_access;
  last_read_access = robot_state.time;

  return std::make_pair(robot_state, time_since_last_read);
}

void ActiveTorqueControl::writeOnce(const Torques& control_input) {
  if (control_finished) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }

  if (control_input.motion_finished) {
    research_interface::robot::MotionGeneratorCommand motion_command{};
    motion_command.dq_c = {0, 0, 0, 0, 0, 0, 0};
    research_interface::robot::ControllerCommand controller_command =
        robot_impl->createControllerCommand(control_input);

    robot_impl->finishMotion(motion_id, &motion_command, &controller_command);
    control_finished = true;
    control_lock.unlock();
    return;
  }

  robot_impl->writeOnce(control_input);
}

template <typename MotionGeneratorType>
void ActiveMotionGenerator<MotionGeneratorType>::writeOnce(
    const MotionGeneratorType& motion_generator_input) {
  if (control_finished) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }
  if (controller_type_ == research_interface::robot::Move::ControllerMode::kExternalController) {
    throw franka::ControlException(
        "Torque command missing, please use writeOnce(const MotionGeneratorType& "
        "motion_generator_input, const Torques& control_input) for external controllers.");
  }

  if (motion_generator_input.motion_finished) {
    auto motion_command = robot_impl->createMotionCommand(motion_generator_input);

    robot_impl->finishMotion(motion_id, &motion_command, nullptr);
    control_finished = true;
    control_lock.unlock();
    return;
  }

  robot_impl->writeOnce(motion_generator_input);
}

template <typename MotionGeneratorType>
void ActiveMotionGenerator<MotionGeneratorType>::writeOnce(
    const MotionGeneratorType& motion_generator_input,
    const Torques& control_input) {
  if (control_finished) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }

  if (controller_type_ != research_interface::robot::Move::ControllerMode::kExternalController) {
    throw franka::ControlException("Torques can only be commanded in kExternalController mode.");
  }

  if (motion_generator_input.motion_finished || control_input.motion_finished) {
    auto control_command = robot_impl->createControllerCommand(control_input);
    auto motion_command = robot_impl->createMotionCommand(motion_generator_input);

    robot_impl->finishMotion(motion_id, &motion_command, &control_command);
    control_finished = true;
    control_lock.unlock();
    return;
  }

  robot_impl->writeOnce(motion_generator_input, control_input);
}

template class ActiveMotionGenerator<JointPositions>;
template class ActiveMotionGenerator<JointVelocities>;
template class ActiveMotionGenerator<CartesianPose>;
template class ActiveMotionGenerator<CartesianVelocities>;

}  // namespace franka