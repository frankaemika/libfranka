// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/active_control.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "robot_impl.h"

namespace franka {

template <typename MotionGeneratorType>
ActiveControl<MotionGeneratorType>::ActiveControl(
    std::shared_ptr<Robot::Impl> robot_impl,
    uint32_t motion_id,
    std::unique_lock<std::mutex> control_lock,
    research_interface::robot::Move::ControllerMode controller_type)
    : robot_impl_(std::move(robot_impl)),
      motion_id_(motion_id),
      control_lock_(std::move(control_lock)),
      control_finished_(false),
      first_read_attempt_(true),
      controller_type_(controller_type) {}

template <typename MotionGeneratorType>
ActiveControl<MotionGeneratorType>::~ActiveControl() {
  if (!control_finished_) {
    robot_impl_->cancelMotion(motion_id_);
  }
}

template <typename MotionGeneratorType>
void ActiveControl<MotionGeneratorType>::writeOnce(const Torques& control_input) {
  if (controller_type_ != research_interface::robot::Move::ControllerMode::kExternalController) {
    throw franka::ControlException("writeOnce called for wrong control mode.");
  }

  if (control_finished_) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }

  if (control_input.motion_finished) {
    control_finished_ = control_input.motion_finished;
    robot_impl_->finishMotion(motion_id_, control_input);
    control_lock_.unlock();
    return;
  }

  robot_impl_->writeOnce(control_input);
}

template <typename MotionGeneratorType>
void ActiveControl<MotionGeneratorType>::writeOnce(
    const MotionGeneratorType& motion_generator_input) {
  if (control_finished_) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }

  if (motion_generator_input.motion_finished) {
    control_finished_ = true;
    // there should be a finish motion function with motion generator
    // robot_impl_->finishMotion(motion_id_, &motion_generator_input, &control_input);
    auto motion_command = robot_impl_->createMotionCommand(motion_generator_input);

    robot_impl_->finishMotion(motion_id_, &motion_command, nullptr);
    control_lock_.unlock();
    return;
  }

  robot_impl_->writeOnce(motion_generator_input);
}

template <typename MotionGeneratorType>
void ActiveControl<MotionGeneratorType>::writeOnce(
    const MotionGeneratorType& motion_generator_input,
    const Torques& control_input) {
  if (control_finished_) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }
  if (controller_type_ != research_interface::robot::Move::ControllerMode::kExternalController) {
    throw franka::ControlException("Torques can only be commanded in kExternalController mode.");
  }
  if (motion_generator_input.motion_finished || control_input.motion_finished) {
    control_finished_ = true;

    auto control_command = robot_impl_->createControllerCommand(control_input);
    auto motion_command = robot_impl_->createMotionCommand(motion_generator_input);

    robot_impl_->finishMotion(motion_id_, &motion_command, &control_command);
    control_lock_.unlock();
    return;
  }

  robot_impl_->writeOnce(motion_generator_input, control_input);
}

template <typename MotionGeneratorType>
std::pair<RobotState, Duration> ActiveControl<MotionGeneratorType>::readOnce() {
  auto robot_state = robot_impl_->readOnce();
  robot_impl_->throwOnMotionError(robot_state, motion_id_);

  if (first_read_attempt_) {
    first_read_attempt_ = false;
    last_read_access_ = robot_state.time;
    return std::make_pair(robot_state, Duration(0));
  }

  auto time_since_last_read = robot_state.time - last_read_access_;
  last_read_access_ = robot_state.time;

  return std::make_pair(robot_state, time_since_last_read);
}

template class ActiveControl<JointPositions>;
template class ActiveControl<JointVelocities>;
template class ActiveControl<CartesianPose>;
template class ActiveControl<CartesianVelocities>;

}  // namespace franka