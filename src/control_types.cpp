#include <cmath>

#include <franka/control_types.h>
#include <franka/exception.h>

namespace franka {

IsStop::IsStop() noexcept : IsStop(false) {}

IsStop::IsStop(bool is_stop) noexcept : is_stop_(is_stop) {}

bool IsStop::stop() const noexcept {
  return is_stop_;
}

Torques::Torques() noexcept : IsStop(true) {}

Torques::Torques(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                 torques) noexcept : tau_J(torques) {}

Torques::Torques(std::initializer_list<double> torques) {
  if (torques.size() != tau_J.size()) {
    throw ControlException("Invalid number of elements in tau_J.");
  }
  std::copy(torques.begin(), torques.end(), tau_J.begin());
}

JointPositions::JointPositions() noexcept : IsStop(true) {}

JointPositions::JointPositions(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                               joint_positions) noexcept : q(joint_positions) {}

JointPositions::JointPositions(std::initializer_list<double> joint_positions) {
  if (joint_positions.size() != q.size()) {
    throw ControlException("Invalid number of elements in joint_positions.");
  }
  std::copy(joint_positions.begin(), joint_positions.end(), q.begin());
}

JointVelocities::JointVelocities() noexcept : IsStop(true) {}

JointVelocities::JointVelocities(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                                 joint_velocities) noexcept : dq(joint_velocities) {}

JointVelocities::JointVelocities(std::initializer_list<double> joint_velocities) {
  if (joint_velocities.size() != dq.size()) {
    throw ControlException("Invalid number of elements in joint_velocities.");
  }
  std::copy(joint_velocities.begin(), joint_velocities.end(), dq.begin());
}

CartesianPose::CartesianPose() noexcept : IsStop(true) {}

CartesianPose::CartesianPose(const std::array<double, 16>&  // NOLINT (modernize-pass-by-value)
                             cartesian_pose)
    : O_T_EE(cartesian_pose) {
  checkHomogeneousTransformation();
}

CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose) {
  if (cartesian_pose.size() != O_T_EE.size()) {
    throw ControlException("Invalid number of elements in cartesian_pose.");
  }
  std::copy(cartesian_pose.begin(), cartesian_pose.end(), O_T_EE.begin());
  checkHomogeneousTransformation();
}

void CartesianPose::checkHomogeneousTransformation() {
  if (!isHomogeneousTransformation(O_T_EE)) {
    throw ControlException(
        "libfranka: Attempt to set invalid transformation in motion"
        "generator. Has to be column major!");
  }
}

bool CartesianPose::isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept {
  constexpr double kOrthonormalThreshold = 1e-6;

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

CartesianVelocities::CartesianVelocities() noexcept : IsStop(true) {}

CartesianVelocities::CartesianVelocities(
    const std::array<double, 6>&  // NOLINT (modernize-pass-by-value)
    cartesian_velocities)
    : O_dP_EE(cartesian_velocities) {}

CartesianVelocities::CartesianVelocities(std::initializer_list<double> cartesian_velocities) {
  if (cartesian_velocities.size() != O_dP_EE.size()) {
    throw ControlException("Invalid number of elements in cartesian_velocities.");
  }
  std::copy(cartesian_velocities.begin(), cartesian_velocities.end(), O_dP_EE.begin());
}

}  // namespace franka
