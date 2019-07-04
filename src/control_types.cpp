// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <type_traits>

#include <franka/control_types.h>

namespace franka {

// NOLINTNEXTLINE(modernize-pass-by-value)
Torques::Torques(const std::array<double, 7>& torques) noexcept : tau_J(torques) {}

Torques::Torques(std::initializer_list<double> torques) {
  if (torques.size() != tau_J.size()) {
    throw std::invalid_argument("Invalid number of elements in tau_J.");
  }
  std::copy(torques.begin(), torques.end(), tau_J.begin());
}

// NOLINTNEXTLINE(modernize-pass-by-value)
JointPositions::JointPositions(const std::array<double, 7>& joint_positions) noexcept
    : q(joint_positions) {}

JointPositions::JointPositions(std::initializer_list<double> joint_positions) {
  if (joint_positions.size() != q.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_positions.");
  }
  std::copy(joint_positions.begin(), joint_positions.end(), q.begin());
}

// NOLINTNEXTLINE(modernize-pass-by-value)
JointVelocities::JointVelocities(const std::array<double, 7>& joint_velocities) noexcept
    : dq(joint_velocities) {}

JointVelocities::JointVelocities(std::initializer_list<double> joint_velocities) {
  if (joint_velocities.size() != dq.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_velocities.");
  }
  std::copy(joint_velocities.begin(), joint_velocities.end(), dq.begin());
}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianPose::CartesianPose(const std::array<double, 16>& cartesian_pose) noexcept
    : O_T_EE(cartesian_pose) {}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianPose::CartesianPose(const std::array<double, 16>& cartesian_pose,
                             // NOLINTNEXTLINE(modernize-pass-by-value)
                             const std::array<double, 2>& elbow) noexcept
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

bool CartesianPose::hasElbow() const noexcept {
  // NOLINTNEXTLINE(readability-container-size-empty)
  return elbow != decltype(elbow)();
}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianVelocities::CartesianVelocities(const std::array<double, 6>& cartesian_velocities) noexcept
    : O_dP_EE(cartesian_velocities) {}

// NOLINTNEXTLINE(modernize-pass-by-value)
CartesianVelocities::CartesianVelocities(const std::array<double, 6>& cartesian_velocities,
                                         // NOLINTNEXTLINE(modernize-pass-by-value)
                                         const std::array<double, 2>& elbow) noexcept
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

bool CartesianVelocities::hasElbow() const noexcept {
  // NOLINTNEXTLINE(readability-container-size-empty)
  return elbow != decltype(elbow)();
}

}  // namespace franka
