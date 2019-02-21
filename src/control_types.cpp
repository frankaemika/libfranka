// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <type_traits>

#include <franka/control_types.h>

namespace franka {

Torques MotionFinished(const Torques& command) {  // NOLINT(readability-identifier-naming)
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

JointPositions MotionFinished(  // NOLINT(readability-identifier-naming)
    const JointPositions& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

JointVelocities MotionFinished(  // NOLINT(readability-identifier-naming)
    const JointVelocities& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

CartesianPose MotionFinished(  // NOLINT(readability-identifier-naming)
    const CartesianPose& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

CartesianVelocities MotionFinished(  // NOLINT(readability-identifier-naming)
    const CartesianVelocities& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

// NOLINTNEXTLINE(modernize-pass-by-value)
Torques::Torques(const std::array<double, 7>& torques) : tau_J(torques) {}

Torques::Torques(std::initializer_list<double> torques) {
  if (torques.size() != tau_J.size()) {
    throw std::invalid_argument("Invalid number of elements in tau_J.");
  }
  std::copy(torques.begin(), torques.end(), tau_J.begin());
}

// NOLINTNEXTLINE(modernize-pass-by-value)
JointPositions::JointPositions(const std::array<double, 7>& joint_positions) : q(joint_positions) {}

JointPositions::JointPositions(std::initializer_list<double> joint_positions) {
  if (joint_positions.size() != q.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_positions.");
  }
  std::copy(joint_positions.begin(), joint_positions.end(), q.begin());
}

// NOLINTNEXTLINE(modernize-pass-by-value)
JointVelocities::JointVelocities(const std::array<double, 7>& joint_velocities)
    : dq(joint_velocities) {}

JointVelocities::JointVelocities(std::initializer_list<double> joint_velocities) {
  if (joint_velocities.size() != dq.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_velocities.");
  }
  std::copy(joint_velocities.begin(), joint_velocities.end(), dq.begin());
}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianPose::CartesianPose(const std::array<double, 16>& cartesian_pose)
    : O_T_EE(cartesian_pose) {}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianPose::CartesianPose(const std::array<double, 16>& cartesian_pose,
                             // NOLINTNEXTLINE(modernize-pass-by-value)
                             const std::array<double, 2>& elbow)
    : O_T_EE(cartesian_pose), elbow(elbow) {}

CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose) {
  if (cartesian_pose.size() != O_T_EE.size()) {
    throw std::invalid_argument("Invalid number of elements in cartesian_pose.");
  }
  std::copy(cartesian_pose.begin(), cartesian_pose.end(), O_T_EE.begin());
}

CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose,
                             std::initializer_list<double> elbow) {
  if (cartesian_pose.size() != O_T_EE.size()) {
    throw std::invalid_argument("Invalid number of elements in cartesian_pose.");
  }
  if (elbow.size() != this->elbow.size()) {
    throw std::invalid_argument("Invalid number of elements in elbow.");
  }
  std::copy(cartesian_pose.begin(), cartesian_pose.end(), O_T_EE.begin());
  std::copy(elbow.begin(), elbow.end(), this->elbow.begin());
}

bool CartesianPose::hasValidElbow() const noexcept {
  return isValidElbow(elbow);
}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianVelocities::CartesianVelocities(const std::array<double, 6>& cartesian_velocities)
    : O_dP_EE(cartesian_velocities) {}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianVelocities::CartesianVelocities(const std::array<double, 6>& cartesian_velocities,
                                         // NOLINTNEXTLINE(modernize-pass-by-value)
                                         const std::array<double, 2>& elbow)
    : O_dP_EE(cartesian_velocities), elbow(elbow) {}

CartesianVelocities::CartesianVelocities(std::initializer_list<double> cartesian_velocities) {
  if (cartesian_velocities.size() != O_dP_EE.size()) {
    throw std::invalid_argument("Invalid number of elements in cartesian_velocities.");
  }
  std::copy(cartesian_velocities.begin(), cartesian_velocities.end(), O_dP_EE.begin());
}

CartesianVelocities::CartesianVelocities(std::initializer_list<double> cartesian_velocities,
                                         std::initializer_list<double> elbow) {
  if (cartesian_velocities.size() != O_dP_EE.size()) {
    throw std::invalid_argument("Invalid number of elements in cartesian_velocities.");
  }
  if (elbow.size() != this->elbow.size()) {
    throw std::invalid_argument("Invalid number of elements in elbow.");
  }
  std::copy(cartesian_velocities.begin(), cartesian_velocities.end(), O_dP_EE.begin());
  std::copy(elbow.begin(), elbow.end(), this->elbow.begin());
}

bool CartesianVelocities::hasValidElbow() const noexcept {
  return isValidElbow(elbow);
}

}  // namespace franka
