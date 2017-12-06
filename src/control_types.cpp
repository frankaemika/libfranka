// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <cmath>
#include <exception>
#include <type_traits>

#include <franka/control_types.h>

namespace franka {

namespace {

inline bool isValidElbow(const std::array<double, 2>& elbow) noexcept {
  return elbow[1] == -1.0 || elbow[1] == 1.0;
}

inline bool isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept {
  constexpr double kOrthonormalThreshold = 1e-5;

  if (transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 || transform[15] != 1.0) {
    return false;
  }
  for (size_t j = 0; j < 3; ++j) {  // i..column
    if (std::abs(std::sqrt(std::pow(transform[j * 4 + 0], 2) + std::pow(transform[j * 4 + 1], 2) +
                           std::pow(transform[j * 4 + 2], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {  // j..row
    if (std::abs(std::sqrt(std::pow(transform[0 * 4 + i], 2) + std::pow(transform[1 * 4 + i], 2) +
                           std::pow(transform[2 * 4 + i], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  return true;
}

template <typename T, size_t N>
inline void checkFinite(const std::array<T, N>& array) {
  if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
    throw std::invalid_argument("Commanding value is infinite or NaN.");
  }
};

inline void checkMatrix(const std::array<double, 16>& transform) {
  checkFinite(transform);
  if (!isHomogeneousTransformation(transform)) {
    throw std::invalid_argument(
        "libfranka: Attempt to set invalid transformation in motion generator. Has to be column "
        "major!");
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

}  // anonymous namespace

Torques MotionFinished(const Torques& command) {  // NOLINT (readability-identifier-naming)
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

JointPositions MotionFinished(  // NOLINT (readability-identifier-naming)
    const JointPositions& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

JointVelocities MotionFinished(  // NOLINT (readability-identifier-naming)
    const JointVelocities& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

CartesianPose MotionFinished(  // NOLINT (readability-identifier-naming)
    const CartesianPose& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

CartesianVelocities MotionFinished(  // NOLINT (readability-identifier-naming)
    const CartesianVelocities& command) {
  std::remove_const_t<std::remove_reference_t<decltype(command)>> new_command(command);
  new_command.motion_finished = true;
  return new_command;
}

Torques::Torques(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                 torques)
    : tau_J(torques) {
  checkFinite(tau_J);
}

Torques::Torques(std::initializer_list<double> torques) {
  if (torques.size() != tau_J.size()) {
    throw std::invalid_argument("Invalid number of elements in tau_J.");
  }
  std::copy(torques.begin(), torques.end(), tau_J.begin());
  checkFinite(tau_J);
}

JointPositions::JointPositions(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                               joint_positions)
    : q(joint_positions) {
  checkFinite(q);
}

JointPositions::JointPositions(std::initializer_list<double> joint_positions) {
  if (joint_positions.size() != q.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_positions.");
  }
  std::copy(joint_positions.begin(), joint_positions.end(), q.begin());
  checkFinite(q);
}

JointVelocities::JointVelocities(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                                 joint_velocities)
    : dq(joint_velocities) {
  checkFinite(dq);
}

JointVelocities::JointVelocities(std::initializer_list<double> joint_velocities) {
  if (joint_velocities.size() != dq.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_velocities.");
  }
  std::copy(joint_velocities.begin(), joint_velocities.end(), dq.begin());
  checkFinite(dq);
}

CartesianPose::CartesianPose(const std::array<double, 16>&  // NOLINT (modernize-pass-by-value)
                             cartesian_pose)
    : O_T_EE(cartesian_pose) {
  checkMatrix(O_T_EE);
}

CartesianPose::CartesianPose(
    const std::array<double, 16>&  // NOLINT (modernize-pass-by-value)
    cartesian_pose,
    const std::array<double, 2>& elbow)  // NOLINT (modernize-pass-by-value)
    : O_T_EE(cartesian_pose),
      elbow(elbow) {
  checkElbow(elbow);
  checkMatrix(O_T_EE);
}

CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose) {
  if (cartesian_pose.size() != O_T_EE.size()) {
    throw std::invalid_argument("Invalid number of elements in cartesian_pose.");
  }
  std::copy(cartesian_pose.begin(), cartesian_pose.end(), O_T_EE.begin());
  checkMatrix(O_T_EE);
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
  checkMatrix(O_T_EE);
  std::copy(elbow.begin(), elbow.end(), this->elbow.begin());
  checkElbow(this->elbow);
}

bool CartesianPose::hasValidElbow() const noexcept {
  return isValidElbow(elbow);
}

CartesianVelocities::CartesianVelocities(
    const std::array<double, 6>&  // NOLINT (modernize-pass-by-value)
    cartesian_velocities)
    : O_dP_EE(cartesian_velocities) {
  checkFinite(O_dP_EE);
}

CartesianVelocities::CartesianVelocities(
    const std::array<double, 6>&  // NOLINT (modernize-pass-by-value)
    cartesian_velocities,
    const std::array<double, 2>& elbow)  // NOLINT (modernize-pass-by-value)
    : O_dP_EE(cartesian_velocities),
      elbow(elbow) {
  checkElbow(elbow);
  checkFinite(O_dP_EE);
}

CartesianVelocities::CartesianVelocities(std::initializer_list<double> cartesian_velocities) {
  if (cartesian_velocities.size() != O_dP_EE.size()) {
    throw std::invalid_argument("Invalid number of elements in cartesian_velocities.");
  }
  std::copy(cartesian_velocities.begin(), cartesian_velocities.end(), O_dP_EE.begin());
  checkFinite(O_dP_EE);
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
  checkElbow(this->elbow);
  checkFinite(O_dP_EE);
}

bool CartesianVelocities::hasValidElbow() const noexcept {
  return isValidElbow(elbow);
}

}  // namespace franka
